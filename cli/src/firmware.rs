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
use protocol::{
    BoardId, BuildId, BuildMarker, ImageChunk, ImageDigest, ImageSize, UPDATE_PORT, UpdateImage,
    UpdateMessage, UpdateStatus,
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

/// Timeout for the boards on the network to fetch an image and restart.
const PUSH_TIMEOUT: Duration = Duration::from_secs(900);

/// How often to update the progress bars and the summary.
const PROGRESS_INTERVAL: Duration = Duration::from_secs(1);

/// The tick interval of the spinner on a push's summary line.
const SPINNER_INTERVAL: Duration = Duration::from_millis(120);

/// How often this machine offers an image to the boards.
const OFFER_INTERVAL: Duration = Duration::from_secs(10);

/// How often to look the boards up again, at the border router and over
/// mDNS. A board gets a new address when it restarts into the image, and it
/// can only tell its new build to this machine once an offer reaches it
/// there.
const TARGET_REFRESH: Duration = Duration::from_secs(30);

/// A board that has fetched the image swaps it in and restarts, which takes
/// at least this long. A reply from it that arrives sooner may have been sent
/// before the restart.
const SWAP_TIME: Duration = Duration::from_secs(10);

/// How long a board that has fetched the image has to tell its build again:
/// the swap, its trial, which ends once it is back on its network and gives
/// up after five minutes (`TRIAL_TIMEOUT` in `firmware/src/update.rs`), and
/// a look-up of its new address.
const COME_BACK_AFTER: Duration = Duration::from_secs(420);

/// A board that has not requested a chunk for this long is no longer shown
/// as fetching.
const FETCHER_TIMEOUT: Duration = Duration::from_secs(30);

/// A push through the border router is over once no board has asked for
/// anything for this long. Every board has had three offers by then.
const QUIET_AFTER: Duration = Duration::from_secs(30);

/// How long a board that has part of the image may stay quiet before the
/// push gives up on it. A board resumes at the next offer it receives, but
/// the network can take minutes to route around a router that is restarting
/// into the image.
const STALLED_AFTER: Duration = Duration::from_secs(180);

/// A board that is fetching the image, as far as its chunk requests show.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fetcher {
    /// The board's ID, if it has sent it. A board whose firmware predates
    /// [`UpdateMessage::Fetching`] does not send it.
    board: Option<BoardId>,
    /// How many bytes of the image it has.
    received: u32,
}

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

/// Update the boards on the network of the border router whose REST API is
/// at `address` to `image`. This machine offers the image to every device
/// that the border router lists and every board advertised over mDNS, and
/// serves the boards that fetch it. The boards reply to each offer with the
/// build they run, which shows which of them kept the image.
pub async fn push_through_border_router(address: &str, image: &Image) -> Result<()> {
    eprintln!("Pushing through the border router at {address}.");
    let targets = look_up(address.to_owned()).await?;
    if targets.is_empty() {
        bail!("found no other device with a routable address on the Thread network");
    }
    offer_to(address, targets, image).await
}

/// The addresses of the devices on the network of the border router at
/// `address`, and of the boards advertised over mDNS.
async fn look_up(address: String) -> Result<Vec<Ipv6Addr>> {
    let (node, devices) = BorderRouter::at(&address).network().await?;
    Ok(border_router::targets(&node, &devices).await.addresses)
}

/// Offer `image` to the devices at `targets`, and to those that the border
/// router at `address` lists later, until the boards among them have fetched
/// it and come back. Show their progress, and the build each board runs.
async fn offer_to(address: &str, mut targets: Vec<Ipv6Addr>, image: &Image) -> Result<()> {
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
    let mut refresh_due = tokio::time::interval(TARGET_REFRESH);
    refresh_due.reset();
    // The look-up in progress. It runs on its own, so that the boards are
    // served meanwhile.
    let mut looking_up: Option<tokio::task::JoinHandle<Result<Vec<Ipv6Addr>>>> = None;
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
                for target in &targets {
                    match socket.send_to(message, (*target, UPDATE_PORT)).await {
                        Ok(_) => any_sent = true,
                        Err(e) => sent = Err(e),
                    }
                }
                if !any_sent {
                    sent.context("could not send to any device on the Thread network")?;
                }
            }
            _ = refresh_due.tick(), if looking_up.is_none() => {
                looking_up = Some(tokio::spawn(look_up(address.to_owned())));
            }
            found = async { looking_up.as_mut().expect("guarded").await }, if looking_up.is_some() => {
                looking_up = None;
                // A failed look-up leaves the targets as they were.
                if let Ok(Ok(found)) = found {
                    let new: Vec<Ipv6Addr> =
                        found.into_iter().filter(|a| !targets.contains(a)).collect();
                    targets.extend(new);
                }
            }
            _ = progress_due.tick() => {
                let now = Instant::now();
                for line in offer.take_news() {
                    progress.println(&line);
                }
                progress.show(&offer.fetching(now));
                progress.set_summary(offer.summary(now));

                match offer.state(started, now) {
                    OfferState::Open if started.elapsed() > PUSH_TIMEOUT => {
                        bail!("the boards did not finish updating to build {build} in time")
                    }
                    OfferState::Open => {}
                    OfferState::Over(report) => {
                        for line in report.lines(build) {
                            progress.println(&line);
                        }
                        return match report.succeeded() {
                            true => Ok(()),
                            false => bail!("not every board that fetched build {build} kept it"),
                        };
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

/// An image that this machine offers, the boards that are fetching it, as
/// far as their requests show, and the builds that the boards say they run.
struct Offer<'a> {
    image: &'a Image,
    fetchers: Vec<NetworkFetcher>,
    /// The build each board last said it runs, in reply to an offer
    /// (`UpdateMessage::Running`), and when. A board whose firmware predates
    /// that reply is only known while it fetches.
    builds: BTreeMap<BoardId, (BuildId, Instant)>,
    /// Lines to print about what has changed since [`Offer::take_news`].
    news: Vec<String>,
}

/// What became of a board that fetched the image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Outcome {
    /// It came back with the image's build.
    Kept,
    /// It came back with another build: its trial of the image failed.
    RolledBack(BuildId),
}

/// A board that is fetching the offered image from this machine.
struct NetworkFetcher {
    address: Ipv6Addr,
    /// The board's ID, once it has sent it.
    board: Option<BoardId>,
    /// How much of the image it has, judging by the last chunk it requested.
    received: u32,
    last_seen: Instant,
    /// When it got the whole image.
    finished: Option<Instant>,
    /// What it did with the image, once it has told its build again.
    outcome: Option<Outcome>,
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
    /// Boards are fetching the image, may still start to, or have yet to
    /// come back with it.
    Open,
    /// Every board that started has the whole image and has come back, or
    /// has had [`COME_BACK_AFTER`] to, and no board has asked for anything
    /// for [`QUIET_AFTER`].
    Over(Report),
    /// These boards have part of the image and have gone quiet for
    /// [`STALLED_AFTER`].
    Stalled(Vec<String>),
}

/// What a push through the border router did.
#[derive(Debug, Default, PartialEq, Eq)]
struct Report {
    /// The boards that fetched the image.
    fetched: usize,
    /// Those that came back with it.
    kept: Vec<String>,
    /// Those that came back with another build.
    rolled_back: Vec<String>,
    /// Those that did not tell their build again in time.
    not_back: Vec<String>,
    /// The boards that run another build and did not fetch the image: it
    /// failed on them before, or does not fit their flash.
    declined: Vec<(BoardId, BuildId)>,
    /// The boards that ran the image already.
    current: usize,
}

impl Report {
    /// Whether every board that fetched the image kept it.
    fn succeeded(&self) -> bool {
        self.rolled_back.is_empty() && self.not_back.is_empty()
    }

    fn lines(&self, build: BuildId) -> Vec<String> {
        let mut lines = Vec::new();
        if self.fetched == 0 {
            lines.push(format!(
                "No board asked for build {build}. A board does not ask for a build that it \
                 runs already, or that has failed on it before."
            ));
        } else {
            let boards = counted(self.fetched, "board");
            lines.push(format!("{boards} fetched build {build}."));
        }
        if !self.kept.is_empty() {
            lines.push(format!("Kept it: {}.", self.kept.join(", ")));
        }
        if !self.rolled_back.is_empty() {
            lines.push(format!("Rolled it back: {}.", self.rolled_back.join(", ")));
        }
        if !self.not_back.is_empty() {
            lines.push(format!(
                "Did not come back in time: {}.",
                self.not_back.join(", ")
            ));
        }
        for (board, runs) in &self.declined {
            lines.push(format!(
                "{board} runs build {runs} and did not ask for build {build}: it failed on that \
                 board before, or does not fit there."
            ));
        }
        if self.current > 0 {
            let boards = counted(self.current, "board");
            lines.push(format!("{boards} ran build {build} already."));
        }
        lines
    }
}

impl<'a> Offer<'a> {
    fn of(image: &'a Image) -> Offer<'a> {
        Offer {
            image,
            fetchers: Vec::new(),
            builds: BTreeMap::new(),
            news: Vec::new(),
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
                let fetcher = self.fetcher(from, now);
                fetcher.received = if end == size { end } else { offset };
                if end == size && fetcher.finished.is_none() {
                    fetcher.finished = Some(now);
                    let name = fetcher.name();
                    let build = self.image.build();
                    self.news
                        .push(format!("{name}  has build {build}, and restarts into it"));
                }

                let data = &self.image.bytes[offset as usize..end as usize];
                Some(UpdateMessage::Chunk {
                    build,
                    offset,
                    // Cannot overflow: `data` is at most one chunk.
                    data: data.iter().copied().collect(),
                })
            }
            UpdateMessage::Running { board, build } => {
                self.record_build(from, board, build, now);
                None
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
                finished: None,
                outcome: None,
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

    /// Record that `board`, at `from`, said at `now` that it runs `build`.
    fn record_build(&mut self, from: Ipv6Addr, board: BoardId, build: BuildId, now: Instant) {
        let image = self.image.build();
        let before = self.builds.insert(board, (build, now)).map(|(b, _)| b);

        // A board that fetched the image comes back with a new address, so
        // it is matched by its ID, which it sent while it fetched.
        let fetched = self.fetchers.iter_mut().find(|f| f.board == Some(board));
        if let Some(fetcher) = fetched
            && fetcher.outcome.is_none()
            && fetcher
                .finished
                .is_some_and(|at| now.duration_since(at) >= SWAP_TIME)
        {
            let outcome = match build == image {
                true => Outcome::Kept,
                false => Outcome::RolledBack(build),
            };
            fetcher.outcome = Some(outcome);
            self.news.push(match outcome {
                Outcome::Kept => format!("{board}  runs build {build}: updated"),
                Outcome::RolledBack(_) => {
                    format!("{board}  is back on build {build}: it rolled build {image} back")
                }
            });
            return;
        }
        // An older firmware never sends its ID, so name the board that
        // fetches from this address once it tells it.
        if let Some(fetcher) = self.fetchers.iter_mut().find(|f| f.address == from)
            && fetcher.board.is_none()
        {
            fetcher.board = Some(board);
        }
        if before.is_none() {
            let state = match build == image {
                true => "up to date",
                false => "to update",
            };
            self.news
                .push(format!("{board}  runs build {build}: {state}"));
        }
    }

    /// The lines to print about what has changed since the last call.
    fn take_news(&mut self) -> Vec<String> {
        std::mem::take(&mut self.news)
    }

    /// The summary line: how many of the boards heard from run the image,
    /// how many have fetched it, and how many are fetching it.
    fn summary(&self, now: Instant) -> String {
        let build = self.image.build();
        let current = self.builds.values().filter(|(b, _)| *b == build).count();
        let fetched = self.fetchers.iter().filter(|f| f.received == self.size());
        let fetching = self.fetchers.iter().filter(|f| {
            f.received < self.size() && now.duration_since(f.last_seen) < FETCHER_TIMEOUT
        });
        format!(
            "Offering build {build}: {current} of {} boards run it, {} fetched it, {} fetching",
            self.builds.len(),
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
        if !partial.is_empty() {
            return OfferState::Stalled(partial.iter().map(|f| f.name()).collect());
        }
        // Wait for the boards that fetched it to come back with a build. A
        // board whose firmware does not tell its build is not waited for.
        let coming_back = fetched.iter().any(|f| {
            f.board.is_some()
                && f.outcome.is_none()
                && f.finished
                    .is_some_and(|at| now.duration_since(at) < COME_BACK_AFTER)
        });
        if coming_back {
            return OfferState::Open;
        }

        let build = self.image.build();
        let mut report = Report {
            fetched: fetched.len(),
            ..Report::default()
        };
        for fetcher in &fetched {
            match (fetcher.board, fetcher.outcome) {
                (_, Some(Outcome::Kept)) => report.kept.push(fetcher.name()),
                (_, Some(Outcome::RolledBack(_))) => report.rolled_back.push(fetcher.name()),
                (Some(_), None) => report.not_back.push(fetcher.name()),
                // It cannot tell its build, so nothing is known.
                (None, None) => {}
            }
        }
        for (&board, &(runs, _)) in &self.builds {
            let has_fetched = self.fetchers.iter().any(|f| f.board == Some(board));
            if runs == build && !has_fetched {
                report.current += 1;
            } else if runs != build && !has_fetched {
                report.declined.push((board, runs));
            }
        }
        OfferState::Over(report)
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

    fn set_summary(&self, summary: String) {
        self.summary.set_message(summary);
    }

    /// Update the bars to match the boards that are fetching the image, as
    /// this machine tracks them.
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
        progress.set_summary("Offering build 2".into());

        progress.show(&[fetcher(Some(BOARD), 50_000), fetcher(None, 200_000)]);
        let screen = terminal.contents();
        let lines: Vec<&str> = screen.lines().collect();
        assert_eq!(lines.len(), 3, "{screen}");
        assert!(lines[0].contains("Offering build 2"), "{screen}");
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
        assert!(offer.take_news().is_empty());

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
        assert_eq!(
            offer.take_news(),
            [format!("{BOARD}  has build {build}, and restarts into it")]
        );
        assert!(offer.take_news().is_empty());

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
            OfferState::Over(Report::default())
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

        // It resumes and finishes. Neither board has sent its ID, so the
        // push does not wait for them to come back.
        offer.handle(a, request(384), after(200));
        assert_eq!(offer.state(started, after(229)), OfferState::Open);
        assert_eq!(
            offer.state(started, after(230)),
            OfferState::Over(Report {
                fetched: 2,
                ..Report::default()
            })
        );
    }

    #[test]
    fn push_through_border_router_follows_the_builds() {
        let image = image();
        let build = image.build();
        let old = BuildId(1);
        let mut offer = Offer::of(&image);
        let started = Instant::now();
        let after = |secs| started + Duration::from_secs(secs);
        let a: Ipv6Addr = "fdf7:2e7f:cbf2:1::a".parse().unwrap();
        let b: Ipv6Addr = "fdf7:2e7f:cbf2:1::b".parse().unwrap();
        let c: Ipv6Addr = "fdf7:2e7f:cbf2:1::c".parse().unwrap();
        let (board_a, board_b, board_c, board_d) = (
            BoardId([0xA; 12]),
            BoardId([0xB; 12]),
            BoardId([0xC; 12]),
            BoardId([0xD; 12]),
        );
        let running = |board, build| UpdateMessage::Running { board, build };
        let fetch = |offer: &mut Offer, from, board, at| {
            offer.handle(from, UpdateMessage::Fetching { board }, at);
            let request = UpdateMessage::ChunkRequest { build, offset: 384 };
            offer.handle(from, request, at);
        };

        // A and B run the old build and fetch the image. C runs it already.
        // D runs the old build and does not ask for it.
        offer.handle(a, running(board_a, old), after(1));
        offer.handle(b, running(board_b, old), after(1));
        offer.handle(c, running(board_c, build), after(1));
        offer.handle(c, running(board_d, old), after(1));
        fetch(&mut offer, a, board_a, after(2));
        fetch(&mut offer, b, board_b, after(2));
        assert_eq!(
            offer.summary(after(3)),
            format!("Offering build {build}: 1 of 4 boards run it, 2 fetched it, 0 fetching")
        );
        // A reply from before the restart doesn't count.
        offer.handle(a, running(board_a, old), after(5));

        // Quiet, but A and B haven't come back.
        assert_eq!(offer.state(started, after(100)), OfferState::Open);

        // A comes back, from another address, with the image. B comes back
        // with the old build: it rolled the image back.
        let a2: Ipv6Addr = "fdf7:2e7f:cbf2:1::a2".parse().unwrap();
        offer.handle(a2, running(board_a, build), after(120));
        offer.handle(b, running(board_b, old), after(130));
        let news = offer.take_news();
        assert!(
            news.contains(&format!("{board_a}  runs build {build}: updated")),
            "{news:?}"
        );
        assert!(
            news.contains(&format!(
                "{board_b}  is back on build {old}: it rolled build {build} back"
            )),
            "{news:?}"
        );

        let OfferState::Over(report) = offer.state(started, after(131)) else {
            panic!("the push is not over");
        };
        assert_eq!(report.fetched, 2);
        assert_eq!(report.kept, [board_a.to_string()]);
        assert_eq!(report.rolled_back, [board_b.to_string()]);
        assert!(report.not_back.is_empty());
        assert_eq!(report.declined, [(board_d, old)]);
        assert_eq!(report.current, 1);
        assert!(!report.succeeded());
    }

    #[test]
    fn push_through_border_router_gives_up_on_a_board_that_does_not_come_back() {
        let image = image();
        let mut offer = Offer::of(&image);
        let started = Instant::now();
        let after = |secs| started + Duration::from_secs(secs);
        let a: Ipv6Addr = "fdf7:2e7f:cbf2:1::a".parse().unwrap();
        offer.handle(a, UpdateMessage::Fetching { board: BOARD }, after(1));
        let request = UpdateMessage::ChunkRequest {
            build: image.build(),
            offset: 384,
        };
        offer.handle(a, request, after(1));

        assert_eq!(offer.state(started, after(420)), OfferState::Open);
        let OfferState::Over(report) = offer.state(started, after(421)) else {
            panic!("the push is not over");
        };
        assert_eq!(report.not_back, [BOARD.to_string()]);
        assert!(!report.succeeded());
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
