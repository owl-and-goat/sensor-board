//! The `firmware` commands: update a board through its running firmware. The
//! image is staged next to the running one, and the board's bootloader swaps
//! the two.

use std::{
    collections::BTreeMap,
    net::{Ipv6Addr, SocketAddr},
    path::Path,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, bail};
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle};
use postcard_rpc::host_client::{MultiSubRxError, MultiSubscription};
use protocol::{
    BoardId, BuildId, BuildMarker, Fetcher, ImageChunk, ImageDigest, ImageSize, Report,
    UPDATE_PORT, UpdateImage, UpdateMessage, UpdateStatus,
};
use sha2::{Digest, Sha256};
use tokio::net::UdpSocket;

use crate::{
    board::Board,
    border_router::{self, BorderRouter},
};

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

/// How often this machine offers an image to the boards. A board that offers
/// one does it at the same interval.
const OFFER_INTERVAL: Duration = Duration::from_secs(10);

/// A board that has not requested a chunk for this long is no longer shown
/// as fetching, as in a board's own [`protocol::OfferProgress`].
const FETCHER_TIMEOUT: Duration = Duration::from_secs(30);

/// A push through the border router is over once no board has asked for
/// anything for this long. Every board has had three offers by then.
const QUIET_AFTER: Duration = Duration::from_secs(30);

/// How long a board that has part of the image may stay quiet before the
/// push gives up on it. A board resumes at the next offer it receives, but
/// the network can take minutes to route around a router that is restarting
/// into the image.
const STALLED_AFTER: Duration = Duration::from_secs(180);

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

/// The attached board to push through: the one that `serial` selects, or
/// without it the first that is on a network. Any board on the network can
/// offer an image to the others.
pub async fn gateway(serial: Option<&str>) -> Result<Board> {
    if serial.is_none()
        && let Some(board) = Board::first_on_network().await?
    {
        return Ok(board);
    }
    Board::select(serial).await
}

/// Update every board on the network to `image` through `gateway`. The
/// gateway receives the image over USB and offers it to the other boards,
/// which fetch it over the network and restart into it. The gateway itself
/// is updated last, once every board that is heard from runs the image.
pub async fn push(gateway: Board, image: &Image) -> Result<()> {
    let serial = gateway.serial().to_owned();
    eprintln!("Pushing through board {serial}.");
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
                    Ok(Some(fetching)) => progress.show(&fetching.fetchers),
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

/// Update the boards on the network of the border router whose REST API is
/// at `address` to `image`, without an attached board. This machine offers
/// the image itself, to every device that the border router lists, and
/// serves the boards that fetch it. It cannot tell which build a board runs,
/// so it reports the boards that fetched the image.
pub async fn push_through_border_router(address: &str, image: &Image) -> Result<()> {
    eprintln!("Pushing through the border router at {address}.");
    let (node, devices) = BorderRouter::at(address).network().await?;
    let targets = border_router::routable_addresses(&node, &devices);
    if targets.is_empty() {
        bail!("the border router lists no other device with a routable address");
    }
    offer_to(&targets, image).await
}

/// Offer `image` to the devices at `targets` until the boards among them
/// have fetched it, and show their progress.
async fn offer_to(targets: &[Ipv6Addr], image: &Image) -> Result<()> {
    // A board sends its requests to the update port, whatever port the offer
    // came from.
    let socket = UdpSocket::bind((Ipv6Addr::UNSPECIFIED, UPDATE_PORT))
        .await
        .with_context(|| format!("could not bind UDP port {UPDATE_PORT}"))?;
    let build = image.build();
    println!(
        "This machine is offering build {build} to {}.",
        counted(targets.len(), "device")
    );

    let mut offer = Offer::of(image);
    let mut encoded = [0; UpdateMessage::MAX_LEN];
    let mut datagram = [0; 2 * UpdateMessage::MAX_LEN];
    let started = Instant::now();

    let mut progress = PushProgress::new(image.describes.size);
    let mut offer_due = tokio::time::interval(OFFER_INTERVAL);
    let mut progress_due = tokio::time::interval(PROGRESS_INTERVAL);
    progress_due.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let interrupted = tokio::signal::ctrl_c();
    tokio::pin!(interrupted);

    loop {
        tokio::select! {
            _ = &mut interrupted => return Ok(()),
            _ = offer_due.tick() => {
                let message = UpdateMessage::Offer(image.describes);
                let message = message
                    .encode(&mut encoded)
                    .expect("the buffer has room for the longest message");
                let mut sent = Ok(());
                let mut any_sent = false;
                for target in targets {
                    match socket.send_to(message, (*target, UPDATE_PORT)).await {
                        Ok(_) => any_sent = true,
                        Err(e) => sent = Err(e),
                    }
                }
                if !any_sent {
                    sent.context("could not send to any device on the Thread network")?;
                }
            }
            _ = progress_due.tick() => {
                let now = Instant::now();
                for board in offer.take_finished() {
                    progress.println(&format!("{board}  has build {build}, and restarts into it"));
                }
                progress.show(&offer.fetching(now));
                progress.set_summary(offer.summary(now));

                match offer.state(started, now) {
                    OfferState::Open if started.elapsed() > PUSH_TIMEOUT => {
                        bail!("the boards did not finish fetching build {build} in time")
                    }
                    OfferState::Open => {}
                    OfferState::Over { fetched: 0 } => {
                        progress.println(&format!(
                            "No board asked for build {build}. A board does not ask for a build \
                             that it runs already, or that has failed on it before."
                        ));
                        return Ok(());
                    }
                    OfferState::Over { fetched } => {
                        let boards = counted(fetched, "board");
                        progress.println(&format!("{boards} fetched build {build}."));
                        return Ok(());
                    }
                    OfferState::Stalled(boards) => {
                        bail!("these boards stopped fetching build {build}: {}", boards.join(", "))
                    }
                }
            }
            received = socket.recv_from(&mut datagram) => {
                let (len, source) = received.context("could not receive from the boards")?;
                let SocketAddr::V6(source) = source else {
                    continue;
                };
                // Anything else on this port is not for a machine that offers.
                let Some(message) = UpdateMessage::decode(&datagram[..len]) else {
                    continue;
                };
                if let Some(reply) = offer.handle(*source.ip(), message, Instant::now()) {
                    let reply = reply
                        .encode(&mut encoded)
                        .expect("the buffer has room for the longest message");
                    // Delivery is not confirmed: the board asks again.
                    let _ = socket.send_to(reply, source).await;
                }
            }
        }
    }
}

/// `count` of a thing, as in "1 board" or "3 boards".
fn counted(count: usize, thing: &str) -> String {
    match count {
        1 => format!("1 {thing}"),
        _ => format!("{count} {thing}s"),
    }
}

/// An image that this machine offers, and the boards that are fetching it,
/// as far as their requests show. It answers the boards as a board that
/// offers an image does (`firmware/src/update.rs`).
struct Offer<'a> {
    image: &'a Image,
    fetchers: Vec<NetworkFetcher>,
}

/// A board that is fetching the offered image from this machine.
struct NetworkFetcher {
    address: Ipv6Addr,
    /// The board's ID, once it has sent it.
    board: Option<BoardId>,
    /// How much of the image it has, judging by the last chunk it requested.
    received: u32,
    last_seen: Instant,
    /// Whether [`Offer::take_finished`] has returned it.
    reported: bool,
}

impl NetworkFetcher {
    /// The board's serial number, or its address if it has not sent its ID.
    fn name(&self) -> String {
        match self.board {
            Some(board) => board.to_string(),
            None => format!("the board at {}", self.address),
        }
    }
}

/// Where a push through the border router stands.
#[derive(Debug, PartialEq, Eq)]
enum OfferState {
    /// Boards are fetching the image, or may still start to.
    Open,
    /// Every board that started has the whole image, and no board has asked
    /// for anything for [`QUIET_AFTER`].
    Over { fetched: usize },
    /// These boards have part of the image and have gone quiet for
    /// [`STALLED_AFTER`].
    Stalled(Vec<String>),
}

impl<'a> Offer<'a> {
    fn of(image: &'a Image) -> Offer<'a> {
        Offer {
            image,
            fetchers: Vec::new(),
        }
    }

    fn size(&self) -> u32 {
        self.image.describes.size.0
    }

    /// Handle `message` from the board at `from`. Returns the reply to send
    /// it, if any.
    fn handle(
        &mut self,
        from: Ipv6Addr,
        message: UpdateMessage,
        now: Instant,
    ) -> Option<UpdateMessage> {
        match message {
            UpdateMessage::Fetching { board } => {
                self.fetcher(from, now).board = Some(board);
                None
            }
            UpdateMessage::ChunkRequest { build, offset } => {
                let size = self.size();
                if build != self.image.build() || offset >= size {
                    return None;
                }
                let len = UpdateMessage::CHUNK_LEN.min((size - offset) as usize);
                let end = offset + len as u32;
                // A board requests the chunk after the data it has, so
                // `offset` is how much it has received. No request follows
                // the last chunk, so that one is assumed to arrive.
                self.fetcher(from, now).received = if end == size { end } else { offset };

                let data = &self.image.bytes[offset as usize..end as usize];
                Some(UpdateMessage::Chunk {
                    build,
                    offset,
                    // Cannot overflow: `data` is at most one chunk.
                    data: data.iter().copied().collect(),
                })
            }
            // What a board sends to a board that fetches, not to one that
            // offers.
            UpdateMessage::Offer(_) | UpdateMessage::Chunk { .. } => None,
        }
    }

    /// The record of the board at `address`. Creates it if there is none.
    fn fetcher(&mut self, address: Ipv6Addr, now: Instant) -> &mut NetworkFetcher {
        let known = self.fetchers.iter().position(|f| f.address == address);
        let index = known.unwrap_or_else(|| {
            self.fetchers.push(NetworkFetcher {
                address,
                board: None,
                received: 0,
                last_seen: now,
                reported: false,
            });
            self.fetchers.len() - 1
        });
        let fetcher = &mut self.fetchers[index];
        fetcher.last_seen = now;
        fetcher
    }

    /// The boards that have requested a chunk recently, for the progress
    /// bars.
    fn fetching(&self, now: Instant) -> Vec<Fetcher> {
        let recent = self
            .fetchers
            .iter()
            .filter(|fetcher| now.duration_since(fetcher.last_seen) < FETCHER_TIMEOUT);
        recent
            .map(|fetcher| Fetcher {
                board: fetcher.board,
                received: fetcher.received,
            })
            .collect()
    }

    /// The names of the boards that have got the whole image since the last
    /// call.
    fn take_finished(&mut self) -> Vec<String> {
        let size = self.size();
        let finished = self
            .fetchers
            .iter_mut()
            .filter(|fetcher| fetcher.received == size && !fetcher.reported);
        finished
            .map(|fetcher| {
                fetcher.reported = true;
                fetcher.name()
            })
            .collect()
    }

    /// The summary line: how many boards have the image, and how many are
    /// fetching it.
    fn summary(&self, now: Instant) -> String {
        let fetched = self.fetchers.iter().filter(|f| f.received == self.size());
        let fetching = self.fetchers.iter().filter(|f| {
            f.received < self.size() && now.duration_since(f.last_seen) < FETCHER_TIMEOUT
        });
        format!(
            "Offering build {}: {} fetched it, {} fetching",
            self.image.build(),
            fetched.count(),
            fetching.count()
        )
    }

    /// Where the push stands at `now`, having started at `started`.
    fn state(&self, started: Instant, now: Instant) -> OfferState {
        let quiet_for = |fetcher: &NetworkFetcher| now.duration_since(fetcher.last_seen);
        let (fetched, partial): (Vec<_>, Vec<_>) = self
            .fetchers
            .iter()
            .partition(|fetcher| fetcher.received == self.size());

        if partial.iter().any(|f| quiet_for(f) < STALLED_AFTER) {
            return OfferState::Open;
        }
        let last_heard = self.fetchers.iter().map(|f| f.last_seen).max();
        if now.duration_since(last_heard.unwrap_or(started).max(started)) < QUIET_AFTER {
            return OfferState::Open;
        }
        if partial.is_empty() {
            OfferState::Over {
                fetched: fetched.len(),
            }
        } else {
            OfferState::Stalled(partial.iter().map(|f| f.name()).collect())
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
        self.set_summary(match builds.len() {
            0 => "Waiting to hear from the other boards".into(),
            n => format!("{} of {n} other boards run build {build}", updated.count()),
        });
    }

    fn set_summary(&self, summary: String) {
        self.summary.set_message(summary);
    }

    /// Update the bars to match the boards that are fetching the image, as
    /// the offering board or this machine tracks them.
    fn show(&mut self, fetchers: &[Fetcher]) {
        let shown: Vec<Option<BoardId>> = self.fetching.iter().map(|row| row.board).collect();
        let places = bars_for(&shown, fetchers);
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
        for (fetcher, row) in fetchers.iter().zip(kept) {
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

        progress.show(&[fetcher(Some(BOARD), 50_000), fetcher(None, 200_000)]);
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
        progress.show(&[fetcher(Some(BOARD), 100_000)]);
        let screen = terminal.contents();
        let lines: Vec<&str> = screen.lines().collect();
        assert_eq!(lines.len(), 2, "{screen}");
        assert!(lines[1].contains(" 50%"), "{screen}");

        drop(progress);
        assert_eq!(terminal.contents().trim(), "");
    }

    /// A 424-byte image of build 1791145757.
    fn image() -> Image {
        let mut bytes = vec![0x11; 300];
        bytes.extend_from_slice(b"sensor-board-fw\0");
        bytes.extend_from_slice(&1_791_145_757u32.to_le_bytes());
        bytes.extend_from_slice(&(!1_791_145_757u32).to_le_bytes());
        bytes.extend_from_slice(&[0x22; 100]);
        Image::from_bytes(bytes).unwrap()
    }

    #[test]
    fn offer_serves_the_image_in_chunks() {
        let image = image();
        let mut offer = Offer::of(&image);
        let now = Instant::now();
        let from: Ipv6Addr = "fdf7:2e7f:cbf2:1::a".parse().unwrap();
        let request = |build, offset| UpdateMessage::ChunkRequest { build, offset };
        let build = image.build();

        let Some(UpdateMessage::Chunk { offset, data, .. }) =
            offer.handle(from, request(build, 0), now)
        else {
            panic!("no chunk");
        };
        assert_eq!((offset, data.as_slice()), (0, &image.bytes[..192]));
        assert_eq!(offer.fetching(now), [fetcher(None, 0)]);
        assert!(offer.take_finished().is_empty());

        let named = offer.handle(from, UpdateMessage::Fetching { board: BOARD }, now);
        assert_eq!(named, None);

        // The last chunk is shorter. No request follows it, so the board
        // counts as having the whole image.
        let Some(UpdateMessage::Chunk { offset, data, .. }) =
            offer.handle(from, request(build, 384), now)
        else {
            panic!("no chunk");
        };
        assert_eq!((offset, data.as_slice()), (384, &image.bytes[384..]));
        assert_eq!(offer.fetching(now), [fetcher(Some(BOARD), 424)]);
        assert_eq!(offer.take_finished(), [BOARD.to_string()]);
        assert!(offer.take_finished().is_empty());

        // Requests past the end, and for another build.
        assert_eq!(offer.handle(from, request(build, 424), now), None);
        assert_eq!(offer.handle(from, request(BuildId(1), 0), now), None);
    }

    #[test]
    fn push_through_border_router_ends_when_the_boards_are_quiet() {
        let image = image();
        let mut offer = Offer::of(&image);
        let started = Instant::now();
        let after = |secs| started + Duration::from_secs(secs);
        let a: Ipv6Addr = "fdf7:2e7f:cbf2:1::a".parse().unwrap();
        let b: Ipv6Addr = "fdf7:2e7f:cbf2:1::b".parse().unwrap();
        let request = |offset| UpdateMessage::ChunkRequest {
            build: image.build(),
            offset,
        };

        // No board asks for the image: over after three offers.
        assert_eq!(offer.state(started, after(29)), OfferState::Open);
        assert_eq!(
            offer.state(started, after(30)),
            OfferState::Over { fetched: 0 }
        );

        // A board that has part of the image keeps the push open, also
        // after the other one has finished and both have gone quiet.
        offer.handle(a, request(0), after(5));
        offer.handle(b, request(384), after(6));
        assert_eq!(offer.state(started, after(100)), OfferState::Open);
        assert_eq!(
            offer.state(started, after(185)),
            OfferState::Stalled(vec![format!("the board at {a}")])
        );

        // It resumes and finishes.
        offer.handle(a, request(384), after(200));
        assert_eq!(offer.state(started, after(229)), OfferState::Open);
        assert_eq!(
            offer.state(started, after(230)),
            OfferState::Over { fetched: 2 }
        );
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
