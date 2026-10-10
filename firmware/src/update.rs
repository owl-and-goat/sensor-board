//! Firmware updates. A new image is written to the staging area, the flash
//! region after the running firmware (`DFU` in memory.x), and checked
//! against its digest. At the next reset the bootloader (`bootloader/`)
//! swaps the two. The update then runs on trial. It is confirmed once the
//! board is back on its network. If the board resets before that, or does
//! not rejoin in time, the bootloader swaps the old firmware back.
//!
//! [`init`] returns the [`Handle`] for the rest of the firmware and the
//! [`Service`]. The service runs inside the coprocessor task, because flash
//! writes have to be coordinated with CPU2.

use core::cell::{Cell, RefCell};
use core::ptr::read_volatile;

use embassy_boot::{AlignedBuffer, FirmwareState, FirmwareUpdaterConfig, State};
use embassy_embedded_hal::flash::partition::Partition;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::{
    Mutex,
    raw::{NoopRawMutex, ThreadModeRawMutex},
};
use embassy_time::{Duration, Instant, Timer};
use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use protocol::{
    BoardId, BuildId, BuildMarker, Fetcher, ImageChunk, NetworkError, NetworkStatus, OfferProgress,
    UpdateError, UpdateImage, UpdateMessage, UpdateResult, UpdateStatus,
};
use sha2::{Digest, Sha256};
use static_cell::StaticCell;

use crate::{
    radio_flash::{self, PAGE, RadioFlash},
    request, thread,
};

static MARKER: BuildMarker = BuildMarker::new(BuildId(decimal(env!("BUILD_ID"))));

/// The build ID of this firmware.
pub fn build() -> BuildId {
    // The volatile read stops the compiler from optimizing the marker away.
    // The host tool looks for the marker in the image.
    unsafe { read_volatile(&MARKER) }.build()
}

const fn decimal(digits: &str) -> u32 {
    let digits = digits.as_bytes();
    let (mut value, mut i) = (0, 0);
    while i < digits.len() {
        value = value * 10 + (digits[i] - b'0') as u32;
        i += 1;
    }
    value
}

unsafe extern "C" {
    static __bootloader_dfu_end: u32;
}

/// The end of the update staging area, as an offset into flash. Flash below
/// this offset is in use by CPU1, so a coprocessor image must not go there.
pub fn staging_end() -> u32 {
    // The linker gives the offset as the address of the symbol.
    (&raw const __bootloader_dfu_end) as u32
}

/// Timeout for a request. The longest one checks the digest of a whole image.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

/// How long an update on trial has to get the board back on its network. A
/// board that finds no network forms its own after about two minutes.
const TRIAL_TIMEOUT: Duration = Duration::from_secs(300);

/// How often a board on trial checks its network status.
const TRIAL_POLL: Duration = Duration::from_secs(1);

/// Delay between an apply and the reset, to let the response reach the
/// caller.
const RESET_DELAY: Duration = Duration::from_millis(200);

/// The interval between offers of the staged image.
const OFFER_INTERVAL: Duration = Duration::from_secs(10);

/// How long a fetching board waits for a requested chunk before it asks
/// again, and how many times in a row it asks. After that it stops until
/// the next offer.
const CHUNK_TIMEOUT: Duration = Duration::from_millis(1500);
const MAX_ATTEMPTS: u8 = 8;

/// A board that has requested no chunk for this long no longer counts as
/// fetching.
const FETCHER_TIMEOUT: Duration = Duration::from_secs(30);

/// Wait until the earlier of two instants. `None` means never.
async fn until_earlier(a: Option<Instant>, b: Option<Instant>) {
    let at = match (a, b) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };
    match at {
        Some(at) => Timer::at(at).await,
        None => core::future::pending().await,
    }
}

enum Request {
    Begin(UpdateImage),
    Write(ImageChunk),
    Finish,
    Apply,
    /// Start or stop offering the staged image to the boards on the network.
    Offer(bool),
}

/// The last published update status.
struct Status(Mutex<ThreadModeRawMutex, Cell<UpdateStatus>>);

impl Status {
    fn get(&self) -> UpdateStatus {
        self.0.lock(|s| s.get())
    }

    fn publish(&self, status: UpdateStatus) {
        self.0.lock(|s| s.set(status));
    }
}

/// The last published progress of the boards that are fetching the image
/// this board offers.
struct Progress(Mutex<ThreadModeRawMutex, RefCell<OfferProgress>>);

struct Shared {
    requests: request::Channel<Request, UpdateResult>,
    status: Status,
    progress: Progress,
}

/// Create the [`Handle`] and the [`Service`]. Panics if called a second time.
pub fn init() -> (Handle, Service) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        status: Status(Mutex::new(Cell::new(UpdateStatus::Starting))),
        progress: Progress(Mutex::new(RefCell::new(OfferProgress::default()))),
    });

    let (client, server) = shared.requests.split();
    let handle = Handle {
        requests: client,
        status: &shared.status,
        progress: &shared.progress,
    };
    let service = Service {
        requests: server,
        status: &shared.status,
        progress: &shared.progress,
    };
    (handle, service)
}

/// Stages, applies and offers updates for the rest of the firmware.
pub struct Handle {
    requests: request::Client<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
}

impl Handle {
    pub fn status(&self) -> UpdateStatus {
        self.status.get()
    }

    /// The progress of the boards that are fetching the image this board
    /// offers.
    pub fn offer_progress(&self) -> OfferProgress {
        self.progress.0.lock(|progress| progress.borrow().clone())
    }

    /// Start receiving `image`. This discards any image that was staged or
    /// being received.
    pub async fn begin(&mut self, image: UpdateImage) -> UpdateResult {
        self.request(Request::Begin(image)).await
    }

    /// Write the next chunk of the image. Chunks have to arrive in order,
    /// and every chunk but the last has to be a whole number of flash words.
    pub async fn write(&mut self, chunk: ImageChunk) -> UpdateResult {
        self.request(Request::Write(chunk)).await
    }

    /// Check the received image against its digest. On success the image is
    /// staged.
    pub async fn finish(&mut self) -> UpdateResult {
        self.request(Request::Finish).await
    }

    /// Restart into the staged image. The response is sent before the reset.
    pub async fn apply(&mut self) -> UpdateResult {
        self.request(Request::Apply).await
    }

    /// Start or stop offering the staged image to the boards on the network.
    /// A board that takes the offer fetches the image and restarts into it.
    pub async fn offer(&mut self, on: bool) -> UpdateResult {
        self.request(Request::Offer(on)).await
    }

    async fn request(&mut self, request: Request) -> UpdateResult {
        self.requests
            .call(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(UpdateError::Unresponsive))
    }
}

/// Serves requests from the [`Handle`] and update messages from other
/// boards.
pub struct Service {
    requests: request::Server<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
}

impl Service {
    /// Run the service on a board whose CPU2 runs a wireless stack. An
    /// update on trial is confirmed based on the status from `network`.
    pub async fn run(
        self,
        flash: &radio_flash::Shared<'_>,
        network: thread::Monitor,
        socket: thread::Socket,
    ) -> ! {
        let FirmwareUpdaterConfig { dfu, state } =
            FirmwareUpdaterConfig::from_linkerfile(flash, flash);
        let mut aligned = AlignedBuffer([0; <RadioFlash as NorFlash>::WRITE_SIZE]);
        let mut updates = Updates {
            staging_area: dfu,
            state_page: state.clone(),
            state: FirmwareState::new(state, &mut aligned.0),
            requests: self.requests,
            status: self.status,
            progress: self.progress,
            incoming: None,
            staged: None,
            socket,
            fetching: None,
            offering: None,
            rolled_back: None,
            board: BoardId(embassy_stm32::uid::uid()),
            fetchers: heapless::Vec::new(),
        };

        match updates.state.get_state().await {
            Ok(State::Swap) => updates.run_trial(network).await,
            Ok(State::Revert) => {
                // The bootloader has swapped the update back into the
                // staging area.
                let whole = updates.staging_area.capacity() as u32;
                updates.rolled_back = updates.build_in_staging_area(whole).await;
                defmt::warn!(
                    "update: build {} did not work out, and was put back",
                    updates.rolled_back
                );
                self.status.publish(UpdateStatus::RolledBack);
            }
            _ => self.status.publish(UpdateStatus::Settled),
        }
        updates.serve().await
    }

    /// Run in place of [`Service::run`] on a board that cannot take an
    /// update. Every request fails.
    pub async fn unavailable(self) -> ! {
        self.status.publish(UpdateStatus::Unavailable);
        loop {
            let (pending, _) = self.requests.receive().await;
            self.requests
                .respond(pending, Err(UpdateError::Unavailable));
        }
    }
}

type Region<'a, 'd> = Partition<'a, NoopRawMutex, RadioFlash<'d>>;

/// An image that is being received.
struct Incoming {
    image: UpdateImage,
    received: u32,
    /// The staging area has been erased up to this offset.
    erased_to: u32,
}

/// A board that is fetching the staged image from this one, as far as its
/// requests show.
struct TrackedFetcher {
    peer: thread::Peer,
    /// The board's ID, once it has sent it.
    board: Option<BoardId>,
    /// How much of the image it has, judging by the last chunk it requested.
    received: u32,
    last_seen: Instant,
}

/// The state of fetching an image, chunk by chunk, from a board that offers
/// it.
struct Fetch {
    from: thread::Peer,
    /// How many times in a row the next chunk has been requested.
    attempts: u8,
    /// When to request it again. `None` after giving up: the next offer of
    /// the image resumes the fetch where it stopped.
    retry_at: Option<Instant>,
}

struct Updates<'a, 'd> {
    /// Offsets into it are offsets into the image.
    staging_area: Region<'a, 'd>,
    /// The bootloader's instruction for the next reset, and the page that
    /// stores it.
    state: FirmwareState<'a, Region<'a, 'd>>,
    state_page: Region<'a, 'd>,
    requests: request::Server<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
    incoming: Option<Incoming>,
    staged: Option<UpdateImage>,
    /// The UDP socket for [`UpdateMessage`]s.
    socket: thread::Socket,
    /// Set while the image being received comes from a board on the network.
    fetching: Option<Fetch>,
    /// When to offer the staged image next. `None` if it is not being
    /// offered.
    offering: Option<Instant>,
    /// The build of an update that was rolled back after its trial. It is
    /// not fetched again.
    rolled_back: Option<BuildId>,
    /// This board's ID.
    board: BoardId,
    /// The boards that are fetching the staged image from this one.
    fetchers: heapless::Vec<TrackedFetcher, { OfferProgress::MAX_FETCHERS }>,
}

impl Updates<'_, '_> {
    /// Run the trial of the running firmware, which is an update that was
    /// just swapped in. Returns once the update is confirmed. If the trial
    /// fails, this resets the board, and the bootloader restores the old
    /// firmware.
    async fn run_trial(&mut self, network: thread::Monitor) {
        defmt::info!("update: on trial as build {}", build());
        self.status.publish(UpdateStatus::OnTrial);
        let deadline = Instant::now() + TRIAL_TIMEOUT;

        loop {
            match network.status() {
                NetworkStatus::Configured(link) if link.role.is_attached() => break,
                // There is no network to rejoin, so starting up is the only
                // test.
                NetworkStatus::Unconfigured
                | NetworkStatus::Unavailable(NetworkError::NoThreadStack) => break,
                _ if Instant::now() > deadline => {
                    defmt::warn!("update: not back on the network; giving it up");
                    cortex_m::peripheral::SCB::sys_reset()
                }
                _ => {}
            }
            if let Either::First((pending, _)) =
                select(self.requests.receive(), Timer::after(TRIAL_POLL)).await
            {
                self.requests.respond(pending, Err(UpdateError::OnTrial));
            }
        }

        match self.state.mark_booted().await {
            Ok(()) => {
                defmt::info!("update: kept");
                self.status.publish(UpdateStatus::Settled);
            }
            // The bootloader still considers the update on trial, so the
            // next reset rolls it back. Refuse every request until then.
            Err(_) => loop {
                defmt::error!("update: could not be marked as kept");
                let (pending, _) = self.requests.receive().await;
                self.requests.respond(pending, Err(UpdateError::Flash));
            },
        }
    }

    async fn serve(&mut self) -> ! {
        // Offers are multicast, so every board has to listen for them.
        if let Err(e) = self.socket.listen(true).await {
            defmt::warn!("update: no updates over the network: {}", e);
        }

        loop {
            let fetch_due = self.fetching.as_ref().and_then(|fetch| fetch.retry_at);
            let next = select3(
                self.requests.receive(),
                self.socket.receive(),
                until_earlier(fetch_due, self.offering),
            );
            match next.await {
                Either3::First((pending, request)) => self.handle_request(pending, request).await,
                Either3::Second(received) => self.handle_datagram(received).await,
                Either3::Third(()) => self.handle_timers().await,
            }
        }
    }

    async fn handle_request(&mut self, pending: request::Pending, request: Request) {
        let apply = matches!(request, Request::Apply);
        let outcome = match request {
            Request::Begin(image) => {
                // An image from the host replaces one that is being fetched
                // from a board on the network.
                self.fetching = None;
                self.begin(image)
            }
            Request::Write(chunk) if self.fetching.is_none() => {
                self.write(chunk.offset, &chunk.data).await
            }
            Request::Finish if self.fetching.is_none() => self.finish().await,
            Request::Write(_) | Request::Finish => Err(UpdateError::OutOfSequence),
            Request::Apply => self.mark_for_swap().await,
            Request::Offer(true) => match self.staged {
                Some(image) => {
                    defmt::info!("update: offering build {} to the network", image.build);
                    self.offering = Some(Instant::now());
                    Ok(())
                }
                None => Err(UpdateError::OutOfSequence),
            },
            Request::Offer(false) => {
                self.offering = None;
                self.clear_fetchers();
                Ok(())
            }
        };
        let applying = apply && outcome.is_ok();
        self.requests.respond(pending, outcome);

        if applying {
            Timer::after(RESET_DELAY).await;
            cortex_m::peripheral::SCB::sys_reset()
        }
    }

    /// Handle the timers that are due: request the missing chunk again, and
    /// offer the staged image again.
    async fn handle_timers(&mut self) {
        let now = Instant::now();

        if let Some(fetch) = &mut self.fetching
            && fetch.retry_at.is_some_and(|at| now >= at)
        {
            fetch.attempts += 1;
            if fetch.attempts > MAX_ATTEMPTS {
                defmt::warn!("update: no answer; waiting for the next offer");
                fetch.retry_at = None;
            } else {
                self.request_next_chunk().await;
            }
        }

        if self.offering.is_some_and(|at| now >= at) {
            self.offering = match self.staged {
                Some(image) => {
                    self.send(None, &UpdateMessage::Offer(image)).await;
                    Some(now + OFFER_INTERVAL)
                }
                None => None,
            };
            // Publishing here removes fetchers that have gone quiet. Nothing
            // else would, because the other calls follow a request.
            self.publish_progress();
        }
    }

    async fn handle_datagram(&mut self, received: thread::Received) {
        let thread::Received { from, datagram } = received;
        match UpdateMessage::decode(&datagram) {
            Some(UpdateMessage::Offer(image)) => self.handle_offer(image, from).await,
            Some(UpdateMessage::ChunkRequest { build, offset }) => {
                self.handle_chunk_request(build, offset, from).await
            }
            Some(UpdateMessage::Chunk {
                build,
                offset,
                data,
            }) => self.handle_chunk(build, offset, &data).await,
            Some(UpdateMessage::Fetching { board }) => {
                if self.offering.is_some() {
                    self.fetcher(from).board = Some(board);
                    self.publish_progress();
                }
            }
            // Not a message this firmware understands. A newer firmware may
            // have sent it.
            None => {}
        }
    }

    /// Handle an offer of `image` from another board. Fetch the image,
    /// unless this board already runs it, has it staged, or has rolled it
    /// back.
    async fn handle_offer(&mut self, image: UpdateImage, from: thread::Peer) {
        // A board also receives its own offers.
        if image.build == build()
            || self.staged == Some(image)
            || self.rolled_back == Some(image.build)
        {
            return;
        }

        match (&self.incoming, &mut self.fetching) {
            (None, _) => {
                if self.begin(image).is_err() {
                    return;
                }
                self.fetching = Some(Fetch {
                    from,
                    attempts: 0,
                    retry_at: None,
                });
                self.send_board_id(from).await;
                self.request_next_chunk().await;
            }
            // This is the image being fetched, or the one whose fetch
            // stalled when its source stopped answering: resume it.
            (Some(incoming), Some(fetch)) if incoming.image == image => {
                fetch.from = from;
                let given_up = fetch.retry_at.is_none();
                if given_up {
                    fetch.attempts = 0;
                }
                // Sent again at every offer, in case the first one was lost.
                self.send_board_id(from).await;
                if given_up {
                    self.request_next_chunk().await;
                }
            }
            // Another image is being received, from the host or from a board.
            (Some(_), _) => {}
        }
    }

    /// Send this board's ID to the board that offers the image, so that it
    /// can report whose progress it is tracking.
    async fn send_board_id(&mut self, peer: thread::Peer) {
        let message = UpdateMessage::Fetching { board: self.board };
        self.send(Some(peer), &message).await;
    }

    async fn request_next_chunk(&mut self) {
        let (Some(incoming), Some(fetch)) = (&self.incoming, &mut self.fetching) else {
            return;
        };
        let request = UpdateMessage::ChunkRequest {
            build: incoming.image.build,
            offset: incoming.received,
        };
        let from = fetch.from;
        fetch.retry_at = Some(Instant::now() + CHUNK_TIMEOUT);
        self.send(Some(from), &request).await;
    }

    async fn handle_chunk(&mut self, build: BuildId, offset: u32, data: &[u8]) {
        let (Some(incoming), Some(_)) = (&self.incoming, &self.fetching) else {
            return;
        };
        // Ignore a late or duplicate chunk: it is not the one that was last
        // requested.
        if build != incoming.image.build || offset != incoming.received {
            return;
        }
        let size = incoming.image.size.0;

        if self.write(offset, data).await.is_err() {
            self.incoming = None;
            self.fetching = None;
            self.status.publish(UpdateStatus::Settled);
            return;
        }
        if offset + (data.len() as u32) < size {
            if let Some(fetch) = &mut self.fetching {
                fetch.attempts = 0;
            }
            self.request_next_chunk().await;
            return;
        }

        self.fetching = None;
        if self.finish().await.is_ok() && self.mark_for_swap().await.is_ok() {
            Timer::after(RESET_DELAY).await;
            cortex_m::peripheral::SCB::sys_reset()
        }
    }

    /// Handle a chunk request from a board that is fetching the staged image.
    async fn handle_chunk_request(&mut self, build: BuildId, offset: u32, from: thread::Peer) {
        let Some(image) = self.staged else {
            return;
        };
        if self.offering.is_none() || build != image.build || offset >= image.size.0 {
            return;
        }
        let len = UpdateMessage::CHUNK_LEN.min((image.size.0 - offset) as usize);
        // A board requests the chunk after the data it has, so `offset` is
        // how much it has received. No request follows the last chunk, so
        // that one is assumed to arrive.
        let end = offset + len as u32;
        self.fetcher(from).received = if end == image.size.0 { end } else { offset };
        self.publish_progress();

        let mut data = heapless::Vec::new();
        // Cannot fail: `len` is at most the capacity.
        let _ = data.resize_default(len);
        if let Err(e) = self.staging_area.read(offset, &mut data).await {
            defmt::error!("update: flash: {}", e);
            return;
        }
        let chunk = UpdateMessage::Chunk {
            build,
            offset,
            data,
        };
        self.send(Some(from), &chunk).await;
    }

    /// The record of the board at `peer` as a fetcher of the staged image.
    /// Creates the record if there is none.
    fn fetcher(&mut self, peer: thread::Peer) -> &mut TrackedFetcher {
        let now = Instant::now();
        let known = self.fetchers.iter().position(|f| f.peer == peer);
        let index = known.unwrap_or_else(|| {
            let new = TrackedFetcher {
                peer,
                board: None,
                received: 0,
                last_seen: now,
            };
            match self.fetchers.push(new) {
                Ok(()) => self.fetchers.len() - 1,
                // The list is full: replace the fetcher that has been quiet
                // the longest.
                Err(new) => {
                    let oldest = (0..self.fetchers.len())
                        .min_by_key(|&i| self.fetchers[i].last_seen)
                        .unwrap_or(0);
                    self.fetchers[oldest] = new;
                    oldest
                }
            }
        });
        let fetcher = &mut self.fetchers[index];
        fetcher.last_seen = now;
        fetcher
    }

    fn clear_fetchers(&mut self) {
        self.fetchers.clear();
        self.publish_progress();
    }

    /// Publish the fetchers' progress, after dropping those that have gone
    /// quiet because they finished or left.
    fn publish_progress(&mut self) {
        let now = Instant::now();
        self.fetchers
            .retain(|fetcher| now - fetcher.last_seen < FETCHER_TIMEOUT);
        let fetchers = self.fetchers.iter().map(|fetcher| Fetcher {
            board: fetcher.board,
            received: fetcher.received,
        });
        let progress = OfferProgress {
            fetchers: fetchers.collect(),
        };
        self.progress
            .0
            .lock(|published| published.replace(progress));
    }

    /// Send `message` to one board, or with `None` to every board. Delivery
    /// is not confirmed: chunk requests are retried and offers are repeated.
    async fn send(&mut self, to: Option<thread::Peer>, message: &UpdateMessage) {
        let mut datagram = thread::Datagram::new();
        // Cannot fail: a datagram has room for the longest message.
        let _ = datagram.resize_default(UpdateMessage::MAX_LEN);
        let Some(len) = message.encode(&mut datagram).map(<[u8]>::len) else {
            return;
        };
        datagram.truncate(len);

        let sent = match to {
            Some(peer) => self.socket.send_to(peer, datagram).await,
            None => self.socket.broadcast(datagram).await,
        };
        if let Err(e) = sent {
            defmt::debug!("update: not sent: {}", e);
        }
    }

    /// The build ID of the firmware image in the first `len` bytes of the
    /// staging area. `None` if those bytes hold no build marker.
    async fn build_in_staging_area(&mut self, len: u32) -> Option<BuildId> {
        // Read in pieces that overlap by more than the length of a marker,
        // so that a marker split across the end of one piece is complete in
        // the next.
        const OVERLAP: u32 = 32;
        const _: () = assert!(BuildMarker::LEN <= OVERLAP as usize);

        let mut piece = [0; 256];
        let mut offset = 0;
        while offset < len {
            let piece = &mut piece[..(len - offset).min(256) as usize];
            self.staging_area.read(offset, piece).await.ok()?;
            if let Some(build) = BuildMarker::find(piece) {
                return Some(build);
            }
            offset += 256 - OVERLAP;
        }
        None
    }

    fn begin(&mut self, image: UpdateImage) -> UpdateResult {
        self.incoming = None;
        self.staged = None;
        self.offering = None;
        self.clear_fetchers();
        // The bootloader swaps the staging area with a region that is one
        // page shorter, so an image can only be as long as that region.
        let room = self.staging_area.capacity() as u32 - PAGE;
        if image.size.0 == 0 || image.size.0 > room {
            self.status.publish(UpdateStatus::Settled);
            return Err(UpdateError::DoesNotFit);
        }

        defmt::info!("update: build {} is arriving", image.build);
        self.incoming = Some(Incoming {
            image,
            received: 0,
            erased_to: 0,
        });
        self.status
            .publish(UpdateStatus::Receiving { image, received: 0 });
        Ok(())
    }

    /// Write the chunk of the image that starts at `offset` to the staging
    /// area.
    async fn write(&mut self, offset: u32, data: &[u8]) -> UpdateResult {
        const WORD: usize = <RadioFlash as NorFlash>::WRITE_SIZE;

        let Some(incoming) = &mut self.incoming else {
            return Err(UpdateError::OutOfSequence);
        };
        let len = data.len();
        let end = offset.saturating_add(len as u32);
        let is_last = end == incoming.image.size.0;
        if offset != incoming.received
            || len == 0
            || len > ImageChunk::MAX_LEN
            || end > incoming.image.size.0
            || (len % WORD != 0 && !is_last)
        {
            return Err(UpdateError::OutOfSequence);
        }

        // Flash is written in whole words. Only the last chunk can end
        // partway through a word, and it is padded with 0xff, like erased
        // flash.
        let mut words = [0xff; ImageChunk::MAX_LEN.next_multiple_of(WORD)];
        words[..len].copy_from_slice(data);
        let words = &words[..len.next_multiple_of(WORD)];

        let written_to = offset + words.len() as u32;
        while incoming.erased_to < written_to {
            let page = incoming.erased_to;
            let erased = self.staging_area.erase(page, page + PAGE).await;
            erased.map_err(flash_error)?;
            incoming.erased_to += PAGE;
        }
        let written = self.staging_area.write(offset, words).await;
        written.map_err(flash_error)?;

        incoming.received = end;
        self.status.publish(UpdateStatus::Receiving {
            image: incoming.image,
            received: end,
        });
        Ok(())
    }

    async fn finish(&mut self) -> UpdateResult {
        let image = match self.incoming.take() {
            Some(incoming) if incoming.received == incoming.image.size.0 => incoming.image,
            _ => {
                self.status.publish(UpdateStatus::Settled);
                return Err(UpdateError::OutOfSequence);
            }
        };

        let mut digest = Sha256::new();
        let mut piece = [0; 256];
        for offset in (0..image.size.0).step_by(piece.len()) {
            let len = piece.len().min((image.size.0 - offset) as usize);
            let read = self.staging_area.read(offset, &mut piece[..len]).await;
            read.map_err(flash_error)?;
            digest.update(&piece[..len]);
        }
        if digest.finalize().as_slice() != image.digest.0 {
            defmt::warn!("update: build {} arrived damaged", image.build);
            self.status.publish(UpdateStatus::Settled);
            return Err(UpdateError::DigestMismatch);
        }
        // Boards decide whether to fetch an offered image by the build ID in
        // the offer, so the image has to contain that same build ID.
        if self.build_in_staging_area(image.size.0).await != Some(image.build) {
            defmt::warn!("update: what arrived is not build {}", image.build);
            self.status.publish(UpdateStatus::Settled);
            return Err(UpdateError::DigestMismatch);
        }

        defmt::info!("update: build {} is staged", image.build);
        self.staged = Some(image);
        self.status.publish(UpdateStatus::Staged(image));
        Ok(())
    }

    /// Have the bootloader swap the staged image in at the next reset.
    async fn mark_for_swap(&mut self) -> UpdateResult {
        let Some(image) = self.staged else {
            return Err(UpdateError::OutOfSequence);
        };
        defmt::info!("update: restarting into build {}", image.build);

        // A page that a flash loader filled with 0xff reads as erased but
        // cannot be programmed: every write fails with a programming error
        // until the page is erased again. The state page is in that state
        // after a probe or the ROM bootloader has written the whole flash
        // image, so erase it here. Nothing is lost by that, because no swap
        // is in progress while this firmware serves requests.
        let size = self.state_page.capacity() as u32;
        let erased = self.state_page.erase(0, size).await;
        erased.map_err(flash_error)?;

        let marked = self.state.mark_updated().await;
        marked.map_err(flash_error)
    }
}

/// Log a flash error and convert it to the error returned to the caller.
fn flash_error(error: impl defmt::Format) -> UpdateError {
    defmt::error!("update: flash: {}", error);
    UpdateError::Flash
}
