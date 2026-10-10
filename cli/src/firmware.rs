//! The `firmware` commands: update a board through its running firmware. The
//! image is staged next to the running one, and the board's bootloader swaps
//! the two.

use std::{
    collections::BTreeMap,
    path::Path,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, bail};
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle};
use postcard_rpc::host_client::{MultiSubRxError, MultiSubscription};
use protocol::{
    BoardId, BuildId, BuildMarker, Fetcher, ImageChunk, ImageDigest, ImageSize, OfferProgress,
    Report, UpdateImage, UpdateStatus,
};
use sha2::{Digest, Sha256};

use crate::board::Board;

/// How long to wait for a board to come back running the new build. Its
/// bootloader takes about fifteen seconds to swap the images.
const RESTART_TIMEOUT: Duration = Duration::from_secs(90);
const POLL_INTERVAL: Duration = Duration::from_millis(500);

/// Boards report every ten seconds. A board that has not been heard from
/// after this long is treated as absent.
const ALL_HEARD_AFTER: Duration = Duration::from_secs(25);

/// Timeout for the boards on the network to fetch an image and restart.
const PUSH_TIMEOUT: Duration = Duration::from_secs(900);

/// How often to ask the offering board for the fetchers' progress.
const PROGRESS_INTERVAL: Duration = Duration::from_secs(1);

/// The tick interval of the spinner on a push's summary line.
const SPINNER_INTERVAL: Duration = Duration::from_millis(120);

/// A firmware image file.
pub struct Image {
    bytes: Vec<u8>,
    describes: UpdateImage,
}

impl Image {
    pub fn read(path: &Path) -> Result<Image> {
        let bytes =
            std::fs::read(path).with_context(|| format!("could not read {}", path.display()))?;
        Image::from_bytes(bytes).with_context(|| format!("{} cannot be used", path.display()))
    }

    fn from_bytes(bytes: Vec<u8>) -> Result<Image> {
        let Some(build) = BuildMarker::find(&bytes) else {
            bail!("it is not an image of this firmware");
        };
        let size = u32::try_from(bytes.len()).context("it is far too long")?;
        let describes = UpdateImage {
            build,
            size: ImageSize(size),
            digest: ImageDigest(Sha256::digest(&bytes).into()),
        };
        Ok(Image { bytes, describes })
    }

    pub fn build(&self) -> BuildId {
        self.describes.build
    }
}

/// Send `image` to `board`, which verifies and stages it.
pub async fn stage(board: &Board, image: &Image) -> Result<()> {
    board.begin_update(&image.describes).await?;
    let chunks = image.bytes.chunks(ImageChunk::MAX_LEN);
    let total = chunks.len();
    for (i, data) in chunks.enumerate() {
        let offset = (i * ImageChunk::MAX_LEN) as u32;
        let chunk = ImageChunk::new(offset, data).expect("a chunk is no longer than MAX_LEN");
        board.write_update(&chunk).await?;
        if (i + 1) % 32 == 0 || i + 1 == total {
            eprint!(
                "\rSending build {}: {}%",
                image.build(),
                100 * (i + 1) / total
            );
        }
    }
    eprintln!();
    board.finish_update().await
}

/// Update `board` to `image`, and wait until it runs the image.
pub async fn update(board: Board, image: &Image) -> Result<()> {
    let serial = board.serial().to_owned();
    if board.firmware_status().await?.build == image.build() {
        println!("Board {serial} runs build {} already.", image.build());
        return Ok(());
    }

    stage(&board, image).await?;
    restart_into_staged(board, image).await
}

/// Tell `board` to restart into `image`, which it has staged, and wait until
/// it runs the image.
async fn restart_into_staged(board: Board, image: &Image) -> Result<()> {
    let serial = board.serial().to_owned();
    board.apply_update().await?;
    drop(board);
    println!("Board {serial} is restarting into build {}.", image.build());

    let deadline = Instant::now() + RESTART_TIMEOUT;
    let board = loop {
        tokio::time::sleep(POLL_INTERVAL).await;
        // The board is off USB while its bootloader swaps the images.
        if let Ok(board) = Board::select(Some(&serial)).await
            && let Ok(running) = board.firmware_status().await
        {
            if running.build != image.build() {
                bail!(
                    "board {serial} came back as build {}: the update did not take",
                    running.build
                );
            }
            break board;
        }
        if Instant::now() > deadline {
            bail!("board {serial} did not come back");
        }
    };
    println!(
        "Board {serial} runs build {}: {}.",
        image.build(),
        describe(&board.firmware_status().await?.update)
    );
    Ok(())
}

/// Update every board on the network to `image` through `gateway`. The
/// gateway receives the image over USB and offers it to the other boards,
/// which fetch it over the network and restart into it. The gateway itself
/// is updated last, once every board that is heard from runs the image.
pub async fn push(gateway: Board, image: &Image) -> Result<()> {
    let serial = gateway.serial().to_owned();
    let status = gateway.firmware_status().await?;
    if status.update != UpdateStatus::Staged(image.describes) {
        stage(&gateway, image).await?;
    }

    // The reports show which build each board runs.
    let mut reports = gateway.reports().await?;
    gateway.start_collecting().await?;
    gateway.start_offering().await?;
    println!(
        "Board {serial} is offering build {} to the network.",
        image.build()
    );

    let outcome = wait_for_boards(&gateway, &mut reports, image).await;
    // Best effort: a board that keeps offering and collecting only does
    // unneeded work.
    let _ = gateway.stop_offering().await;
    let _ = gateway.stop_collecting().await;
    if !outcome? {
        return Ok(());
    }

    if status.build == image.build() {
        println!("Board {serial} runs build {} already.", image.build());
        return Ok(());
    }
    restart_into_staged(gateway, image).await
}

/// Watch the reports that `gateway` forwards until every board except the
/// gateway runs `image`, and show the progress of the boards that are
/// fetching it. Returns `false` if interrupted before that.
async fn wait_for_boards(
    gateway: &Board,
    reports: &mut MultiSubscription<Report>,
    image: &Image,
) -> Result<bool> {
    let build = image.build();
    let started = Instant::now();
    let mut builds = BTreeMap::new();

    let mut progress = PushProgress::new(image.describes.size);
    progress.summarize(&builds, build);
    let mut progress_due = tokio::time::interval(PROGRESS_INTERVAL);
    progress_due.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    // Set to false if the gateway's firmware cannot report progress.
    let mut reports_progress = true;

    let interrupted = tokio::signal::ctrl_c();
    tokio::pin!(interrupted);

    loop {
        // Every board reports within this time, so after it no board is
        // still waiting to be heard from.
        let all_heard = started.elapsed() > ALL_HEARD_AFTER;
        if all_heard && builds.values().all(|&running| running == build) {
            progress.println(&match builds.len() {
                0 => "No other board was heard from.".into(),
                n => format!("All {n} other boards run build {build}."),
            });
            return Ok(true);
        }
        if started.elapsed() > PUSH_TIMEOUT {
            let behind: Vec<String> = builds
                .iter()
                .filter(|&(_, &running)| running != build)
                .map(|(board, running)| format!("{board} (build {running})"))
                .collect();
            bail!("these boards did not update: {}", behind.join(", "));
        }

        let report = tokio::select! {
            _ = &mut interrupted => return Ok(false),
            () = tokio::time::sleep(POLL_INTERVAL) => continue,
            _ = progress_due.tick(), if reports_progress => {
                match gateway.offer_progress().await {
                    Ok(Some(fetching)) => progress.show(&fetching),
                    Ok(None) => {
                        reports_progress = false;
                        progress.println(&format!(
                            "Board {} runs a firmware that cannot tell how far the others have \
                             got. Update it first (firmware update) to see that.",
                            gateway.serial()
                        ));
                    }
                    // Ignore the error: any real problem also shows in the
                    // reports.
                    Err(_) => {}
                }
                continue;
            }
            report = reports.recv() => report,
        };
        match report {
            Ok(report) if report.board.to_string() != gateway.serial() => {
                if builds.insert(report.board, report.firmware) != Some(report.firmware) {
                    let state = if report.firmware == build {
                        "updated"
                    } else {
                        "to update"
                    };
                    progress.println(&format!(
                        "{}  build {}  {state}",
                        report.board, report.firmware
                    ));
                    progress.summarize(&builds, build);
                }
            }
            // The gateway's own report. The gateway is updated last.
            Ok(_) => {}
            Err(MultiSubRxError::Lagged(_)) => {}
            Err(MultiSubRxError::IoClosed) => bail!("board {} is gone", gateway.serial()),
        }
    }
}

/// The display of a push while the boards update: a summary line with how
/// many boards run the image, and below it a bar for each board that is
/// fetching it. The bars are only drawn on a terminal. Lines printed with
/// [`PushProgress::println`] appear above the bars, on a terminal or not.
struct PushProgress {
    bars: MultiProgress,
    summary: ProgressBar,
    fetching: Vec<FetchBar>,
    size: ImageSize,
}

/// The progress bar of one fetching board.
struct FetchBar {
    /// The board's ID, once it has sent it.
    board: Option<BoardId>,
    bar: ProgressBar,
}

impl PushProgress {
    fn new(size: ImageSize) -> PushProgress {
        PushProgress::drawn_on(ProgressDrawTarget::stderr(), size)
    }

    fn drawn_on(target: ProgressDrawTarget, size: ImageSize) -> PushProgress {
        let bars = MultiProgress::with_draw_target(target);
        let style = ProgressStyle::with_template("{spinner} {msg} ({elapsed})")
            .expect("the template is well-formed");
        let summary = bars.add(ProgressBar::new_spinner().with_style(style));
        summary.enable_steady_tick(SPINNER_INTERVAL);
        PushProgress {
            bars,
            summary,
            fetching: Vec::new(),
            size,
        }
    }

    /// Print a line above the bars.
    fn println(&self, line: &str) {
        self.bars.suspend(|| println!("{line}"));
    }

    /// Update the summary with how many of the boards heard from run `build`.
    fn summarize(&self, builds: &BTreeMap<BoardId, BuildId>, build: BuildId) {
        let updated = builds.values().filter(|&&running| running == build);
        self.summary.set_message(match builds.len() {
            0 => "Waiting to hear from the other boards".into(),
            n => format!("{} of {n} other boards run build {build}", updated.count()),
        });
    }

    /// Update the bars to match the progress that the offering board reports.
    fn show(&mut self, progress: &OfferProgress) {
        let shown: Vec<Option<BoardId>> = self.fetching.iter().map(|row| row.board).collect();
        let places = bars_for(&shown, &progress.fetchers);
        let mut shown: Vec<Option<FetchBar>> = self.fetching.drain(..).map(Some).collect();
        let kept: Vec<Option<FetchBar>> = places
            .into_iter()
            .map(|place| place.and_then(|i| shown[i].take()))
            .collect();

        // The remaining bars belong to boards that have stopped requesting
        // chunks, because they finished or left.
        for gone in shown.into_iter().flatten() {
            self.bars.remove(&gone.bar);
        }
        for (fetcher, row) in progress.fetchers.iter().zip(kept) {
            let mut row = row.unwrap_or_else(|| self.new_bar(fetcher.received));
            row.board = fetcher.board;
            // Once a board has the whole image, it verifies it and restarts
            // into it.
            let template = if fetcher.received < self.size.0 {
                "{prefix:24} {wide_bar} {percent:>3}%  {bytes_per_sec:>12}  eta {eta:>3}"
            } else {
                "{prefix:24} {wide_bar} {percent:>3}%  restarting"
            };
            let style = ProgressStyle::with_template(template);
            row.bar
                .set_style(style.expect("the template is well-formed"));
            // A board whose firmware predates `UpdateMessage::Fetching` does
            // not send its ID, so it is shown without one.
            let name = fetcher.board.map_or("a board".into(), |b| b.to_string());
            row.bar.set_prefix(name);
            row.bar.set_position(fetcher.received.into());
            self.fetching.push(row);
        }
    }

    /// A new bar for a board that already has `received` bytes of the image.
    fn new_bar(&self, received: u32) -> FetchBar {
        let length = Some(self.size.0.into());
        let bar = ProgressBar::with_draw_target(length, ProgressDrawTarget::hidden());
        // The bytes a board had before it was first seen must not count
        // towards its rate. indicatif only resets its rate estimate when a
        // bar moves backwards, so move this bar back to the board's position
        // before it is shown.
        bar.set_position(u64::MAX);
        bar.set_position(received.into());
        FetchBar {
            board: None,
            bar: self.bars.add(bar),
        }
    }
}

impl Drop for PushProgress {
    fn drop(&mut self) {
        // An unfinished bar is drawn once more when it is dropped, and stays
        // on screen. Clear the bars first.
        for row in &self.fetching {
            row.bar.finish_and_clear();
        }
        self.summary.finish_and_clear();
    }
}

/// The bar to show each fetcher on, as an index into `shown`, which gives
/// the board of each existing bar. A board keeps its bar. A fetcher without
/// a bar takes the first bar that has no board ID: that is the same board,
/// whether it has sent its ID since or not. `None` means the fetcher needs a
/// new bar.
fn bars_for(shown: &[Option<BoardId>], fetchers: &[Fetcher]) -> Vec<Option<usize>> {
    let mut bars: Vec<Option<usize>> = fetchers
        .iter()
        .map(|fetcher| {
            let board = fetcher.board?;
            shown.iter().position(|&shown| shown == Some(board))
        })
        .collect();

    let mut unnamed = (0..shown.len()).filter(|&i| shown[i].is_none());
    for bar in bars.iter_mut().filter(|bar| bar.is_none()) {
        *bar = unnamed.next();
    }
    bars
}

pub fn describe(status: &UpdateStatus) -> String {
    match status {
        UpdateStatus::Starting => "starting".into(),
        UpdateStatus::Unavailable => "cannot take an update without a wireless stack".into(),
        UpdateStatus::Settled => "settled".into(),
        UpdateStatus::Receiving { image, received } => format!(
            "receiving build {}, {} of {} bytes",
            image.build, received, image.size.0
        ),
        UpdateStatus::Staged(image) => format!("build {} is staged", image.build),
        UpdateStatus::OnTrial => "on trial, and kept once the board is back on its network".into(),
        UpdateStatus::RolledBack => {
            "the last update did not work out, and the firmware before it is back".into()
        }
    }
}

#[cfg(test)]
mod tests {
    use indicatif::InMemoryTerm;

    use super::*;

    const BOARD: BoardId = BoardId([
        0x4C, 0x00, 0x32, 0x00, 0x03, 0x50, 0x47, 0x55, 0x32, 0x30, 0x33, 0x31,
    ]);

    fn fetcher(board: Option<BoardId>, received: u32) -> Fetcher {
        Fetcher { board, received }
    }

    #[test]
    fn fetcher_keeps_its_bar() {
        let (a, b) = (Some(BoardId([0xA; 12])), Some(BoardId([0xB; 12])));
        let fetchers = |boards: &[Option<BoardId>]| -> Vec<Fetcher> {
            boards.iter().map(|&board| fetcher(board, 0)).collect()
        };

        // The middle bar's board is gone, and the board after it keeps its
        // bar.
        assert_eq!(
            bars_for(&[a, None, b], &fetchers(&[a, b])),
            [Some(0), Some(2)]
        );
        // A board that has sent its ID since keeps the bar it had without
        // one.
        assert_eq!(bars_for(&[a, None], &fetchers(&[a, b])), [Some(0), Some(1)]);
        assert_eq!(bars_for(&[None, None], &fetchers(&[None])), [Some(0)]);
        // New fetchers need new bars.
        assert_eq!(
            bars_for(&[a], &fetchers(&[a, None, b])),
            [Some(0), None, None]
        );
    }

    #[test]
    fn push_shows_one_bar_per_fetcher() {
        let terminal = InMemoryTerm::new(10, 100);
        let target = ProgressDrawTarget::term_like(Box::new(terminal.clone()));
        let mut progress = PushProgress::drawn_on(target, ImageSize(200_000));
        progress.summarize(&BTreeMap::from([(BOARD, BuildId(1))]), BuildId(2));

        progress.show(&OfferProgress {
            fetchers: [fetcher(Some(BOARD), 50_000), fetcher(None, 200_000)]
                .into_iter()
                .collect(),
        });
        let screen = terminal.contents();
        let lines: Vec<&str> = screen.lines().collect();
        assert_eq!(lines.len(), 3, "{screen}");
        assert!(
            lines[0].contains("0 of 1 other boards run build 2"),
            "{screen}"
        );
        assert!(
            lines[1].starts_with("4C0032000350475532303331 "),
            "{screen}"
        );
        assert!(lines[1].contains(" 25%"), "{screen}");
        // The bytes it had when first seen do not count towards its rate.
        assert!(lines[1].contains(" 0 B/s"), "{screen}");
        assert!(lines[2].starts_with("a board "), "{screen}");
        assert!(lines[2].contains("100%"), "{screen}");
        assert!(lines[2].ends_with("restarting"), "{screen}");

        // The board that had the whole image has stopped requesting chunks.
        progress.show(&OfferProgress {
            fetchers: [fetcher(Some(BOARD), 100_000)].into_iter().collect(),
        });
        let screen = terminal.contents();
        let lines: Vec<&str> = screen.lines().collect();
        assert_eq!(lines.len(), 2, "{screen}");
        assert!(lines[1].contains(" 50%"), "{screen}");

        drop(progress);
        assert_eq!(terminal.contents().trim(), "");
    }

    #[test]
    fn rejects_images_without_build_marker() {
        assert!(Image::from_bytes(vec![0; 4096]).is_err());

        let mut bytes = vec![0x11; 300];
        bytes.extend_from_slice(b"sensor-board-fw\0");
        bytes.extend_from_slice(&1_791_145_757u32.to_le_bytes());
        bytes.extend_from_slice(&(!1_791_145_757u32).to_le_bytes());
        bytes.extend_from_slice(&[0x22; 100]);
        let image = Image::from_bytes(bytes.clone()).unwrap();
        assert_eq!(image.build(), BuildId(1_791_145_757));
        assert_eq!(image.describes.size, ImageSize(424));
        assert_eq!(
            image.describes.digest.0,
            <[u8; 32]>::from(Sha256::digest(&bytes))
        );
    }
}
