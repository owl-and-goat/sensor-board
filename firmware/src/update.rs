//! Firmware updates. A new image is staged in the flash region next to the
//! one that runs (`DFU` in memory.x) and checked against its digest. At the
//! next reset the bootloader (`bootloader/`) swaps the two. The update then
//! runs on trial: it is kept once the board is back on its network, and if
//! the board resets first, or does not get there in time, the bootloader
//! swaps the old firmware back.
//!
//! [`init`] makes the two ends: the [`Handle`] for the rest of the firmware,
//! and the [`Service`], which runs inside the coprocessor task, because
//! flash is written in step with CPU2.

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

/// Which build of the firmware this is.
pub fn build() -> BuildId {
    // Read out of the marker, and in a way the compiler does not see
    // through: that keeps the marker in the image, where the host tool looks
    // for it.
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

/// Where the region an update is staged in ends, as an offset into flash.
/// Nothing else of CPU1's may be put below this.
pub fn staging_end() -> u32 {
    // The linker gives the offset as the address of the symbol.
    (&raw const __bootloader_dfu_end) as u32
}

/// How long a request gets. The longest is the check of a whole image.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

/// How long an update on trial has to get the board back on its network.
/// A board that finds no network makes one of its own in about two minutes.
const TRIAL_TIMEOUT: Duration = Duration::from_secs(300);

/// How often a board on trial looks at where it stands with its network.
const TRIAL_POLL: Duration = Duration::from_secs(1);

/// How long the answer to an apply gets to reach whoever asked, before the
/// reset.
const RESET_DELAY: Duration = Duration::from_millis(200);

/// How often a board that offers its staged image says so.
const OFFER_INTERVAL: Duration = Duration::from_secs(10);

/// How long a board that fetches an image waits for the chunk it asked for
/// before it asks again, and how many times in a row it does. After that it
/// leaves it until the next offer.
const CHUNK_TIMEOUT: Duration = Duration::from_millis(1500);
const MAX_ATTEMPTS: u8 = 8;

/// A board that has asked for nothing for this long is no longer taken for
/// one that is fetching.
const FETCHER_QUIET: Duration = Duration::from_secs(30);

/// Wait for the earlier of two moments, each of which may never come.
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
    /// Start offering the staged image to the boards on the network, or stop.
    Offer(bool),
}

/// Where the board stands with updates, as last published.
struct Status(Mutex<ThreadModeRawMutex, Cell<UpdateStatus>>);

impl Status {
    fn get(&self) -> UpdateStatus {
        self.0.lock(|s| s.get())
    }

    fn publish(&self, status: UpdateStatus) {
        self.0.lock(|s| s.set(status));
    }
}

/// How far the boards that fetch the image this one offers have got, as last
/// published.
struct Progress(Mutex<ThreadModeRawMutex, RefCell<OfferProgress>>);

struct Shared {
    requests: request::Channel<Request, UpdateResult>,
    status: Status,
    progress: Progress,
}

/// Make the two ends of firmware updates. Panics if called a second time.
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

/// How the rest of the firmware gets an update in.
pub struct Handle {
    requests: request::Client<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
}

impl Handle {
    pub fn status(&self) -> UpdateStatus {
        self.status.get()
    }

    /// How far the boards that fetch the image this one offers have got.
    pub fn offer_progress(&self) -> OfferProgress {
        self.progress.0.lock(|progress| progress.borrow().clone())
    }

    /// Start taking in `image`, in place of whatever was staged or arriving.
    pub async fn begin(&mut self, image: UpdateImage) -> UpdateResult {
        self.request(Request::Begin(image)).await
    }

    /// The next piece of the image. Pieces come in order, and every one but
    /// the last is a whole number of flash words.
    pub async fn write(&mut self, chunk: ImageChunk) -> UpdateResult {
        self.request(Request::Write(chunk)).await
    }

    /// Check the image that has arrived against its digest. From here on it
    /// is staged.
    pub async fn finish(&mut self) -> UpdateResult {
        self.request(Request::Finish).await
    }

    /// Restart into the staged image. The answer comes first.
    pub async fn apply(&mut self) -> UpdateResult {
        self.request(Request::Apply).await
    }

    /// Start offering the staged image to the boards on the network, which
    /// fetch it and restart into it, or stop.
    pub async fn offer(&mut self, on: bool) -> UpdateResult {
        self.request(Request::Offer(on)).await
    }

    async fn request(&mut self, request: Request) -> UpdateResult {
        self.requests
            .ask(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(UpdateError::Unresponsive))
    }
}

/// The end of firmware updates that answers the [`Handle`].
pub struct Service {
    requests: request::Server<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
}

impl Service {
    /// Serve updates on a board whose CPU2 runs a wireless stack. `network`
    /// is what an update on trial is judged by.
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
            arriving: None,
            staged: None,
            socket,
            fetching: None,
            offering: None,
            refused: None,
            board: BoardId(embassy_stm32::uid::uid()),
            fetchers: heapless::Vec::new(),
        };

        match updates.state.get_state().await {
            Ok(State::Swap) => updates.trial(network).await,
            Ok(State::Revert) => {
                // The bootloader has swapped the update back into the
                // staging area.
                let whole = updates.staging_area.capacity() as u32;
                updates.refused = updates.build_in_staging_area(whole).await;
                defmt::warn!(
                    "update: build {} did not work out, and was put back",
                    updates.refused
                );
                self.status.publish(UpdateStatus::RolledBack);
            }
            _ => self.status.publish(UpdateStatus::Settled),
        }
        updates.serve().await
    }

    /// Stand in for updates on a board that cannot take one.
    pub async fn unavailable(self) -> ! {
        self.status.publish(UpdateStatus::Unavailable);
        loop {
            let (pending, _) = self.requests.receive().await;
            self.requests.answer(pending, Err(UpdateError::Unavailable));
        }
    }
}

type Region<'a, 'd> = Partition<'a, NoopRawMutex, RadioFlash<'d>>;

/// An image on its way in.
struct Arriving {
    image: UpdateImage,
    received: u32,
    /// Up to where the staging area has been erased for it.
    erased_to: u32,
}

/// A board that fetches the staged image from this one, as its requests show
/// it.
struct FetcherHeard {
    peer: thread::Peer,
    /// Which board, once it has said.
    board: Option<BoardId>,
    /// How much of the image it has, going by the chunk it last asked for.
    received: u32,
    heard_at: Instant,
}

/// The fetching of an image from a board that offers it, a chunk at a time.
struct Fetch {
    from: thread::Peer,
    /// How many times in a row the chunk that is due has been asked for.
    attempts: u8,
    /// When to ask for it again. `None` once that has been given up: the
    /// next offer of the image takes it up again where it stopped.
    retry_at: Option<Instant>,
}

struct Updates<'a, 'd> {
    /// Offsets into it are offsets into the image.
    staging_area: Region<'a, 'd>,
    /// What the bootloader is to do at the next reset, and the page that is
    /// kept in.
    state: FirmwareState<'a, Region<'a, 'd>>,
    state_page: Region<'a, 'd>,
    requests: request::Server<Request, UpdateResult>,
    status: &'static Status,
    progress: &'static Progress,
    arriving: Option<Arriving>,
    staged: Option<UpdateImage>,
    /// What boards say to each other about updates.
    socket: thread::Socket,
    /// Set while what is arriving comes from a board on the network.
    fetching: Option<Fetch>,
    /// When the staged image is next offered to the network, if it is being
    /// offered.
    offering: Option<Instant>,
    /// The build that an update on trial was, if it was put back. It is not
    /// fetched a second time.
    refused: Option<BuildId>,
    /// This board.
    board: BoardId,
    /// The boards that are fetching the staged image from this one.
    fetchers: heapless::Vec<FetcherHeard, { OfferProgress::MAX_FETCHERS }>,
}

impl Updates<'_, '_> {
    /// See the running firmware, an update that has just been swapped in,
    /// through its trial. Returns once it is kept. If it is not to be, this
    /// resets the board, and the bootloader puts back what was there.
    async fn trial(&mut self, network: thread::Monitor) {
        defmt::info!("update: on trial as build {}", build());
        self.status.publish(UpdateStatus::OnTrial);
        let deadline = Instant::now() + TRIAL_TIMEOUT;

        loop {
            match network.status() {
                NetworkStatus::Configured(link) if link.role.is_attached() => break,
                // Nothing to get back on: having started is all there is to
                // show.
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
                self.requests.answer(pending, Err(UpdateError::OnTrial));
            }
        }

        match self.state.mark_booted().await {
            Ok(()) => {
                defmt::info!("update: kept");
                self.status.publish(UpdateStatus::Settled);
            }
            // Still on trial as far as the bootloader knows, so the next
            // reset undoes the update. Nothing more can be taken until then.
            Err(_) => loop {
                defmt::error!("update: could not be marked as kept");
                let (pending, _) = self.requests.receive().await;
                self.requests.answer(pending, Err(UpdateError::Flash));
            },
        }
    }

    async fn serve(&mut self) -> ! {
        // An offer goes to every board, so every board listens for one.
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
                Either3::First((pending, request)) => self.answer(pending, request).await,
                Either3::Second(received) => self.heard(received).await,
                Either3::Third(()) => self.tick().await,
            }
        }
    }

    async fn answer(&mut self, pending: request::Pending, request: Request) {
        let apply = matches!(request, Request::Apply);
        let outcome = match request {
            Request::Begin(image) => {
                // What the host sends takes the place of what a board on
                // the network was sending.
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
                self.forget_fetchers();
                Ok(())
            }
        };
        let applying = apply && outcome.is_ok();
        self.requests.answer(pending, outcome);

        if applying {
            Timer::after(RESET_DELAY).await;
            cortex_m::peripheral::SCB::sys_reset()
        }
    }

    /// What is due: the chunk that did not come is asked for again, and the
    /// staged image is offered again.
    async fn tick(&mut self) {
        let now = Instant::now();

        if let Some(fetch) = &mut self.fetching
            && fetch.retry_at.is_some_and(|at| now >= at)
        {
            fetch.attempts += 1;
            if fetch.attempts > MAX_ATTEMPTS {
                defmt::warn!("update: no answer; waiting for the next offer");
                fetch.retry_at = None;
            } else {
                self.ask_for_next_chunk().await;
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
            // Nothing else drops a board that has stopped asking.
            self.publish_progress();
        }
    }

    async fn heard(&mut self, received: thread::Received) {
        let thread::Received { from, datagram } = received;
        match UpdateMessage::decode(&datagram) {
            Some(UpdateMessage::Offer(image)) => self.offered(image, from).await,
            Some(UpdateMessage::ChunkRequest { build, offset }) => {
                self.asked_for_chunk(build, offset, from).await
            }
            Some(UpdateMessage::Chunk {
                build,
                offset,
                data,
            }) => self.got_chunk(build, offset, &data).await,
            Some(UpdateMessage::Fetching { board }) => {
                if self.offering.is_some() {
                    self.fetcher(from).board = Some(board);
                    self.publish_progress();
                }
            }
            // From a firmware that says things this one does not know.
            None => {}
        }
    }

    /// Another board offers `image`. Fetch it, unless it is what this board
    /// has or runs already, or what it has tried and put back.
    async fn offered(&mut self, image: UpdateImage, from: thread::Peer) {
        // A board hears its own offers too.
        if image.build == build() || self.staged == Some(image) || self.refused == Some(image.build)
        {
            return;
        }

        match (&self.arriving, &mut self.fetching) {
            (None, _) => {
                if self.begin(image).is_err() {
                    return;
                }
                self.fetching = Some(Fetch {
                    from,
                    attempts: 0,
                    retry_at: None,
                });
                self.introduce_to(from).await;
                self.ask_for_next_chunk().await;
            }
            // The image that is being fetched, or was until its board stopped
            // answering: go on from where that got to.
            (Some(arriving), Some(fetch)) if arriving.image == image => {
                fetch.from = from;
                let given_up = fetch.retry_at.is_none();
                if given_up {
                    fetch.attempts = 0;
                }
                // Said again at every offer: the first may not have arrived.
                self.introduce_to(from).await;
                if given_up {
                    self.ask_for_next_chunk().await;
                }
            }
            // Another image is arriving, from the host or from a board.
            (Some(_), _) => {}
        }
    }

    /// Tell the board that offers the image which board this is, so that it
    /// can say whose progress it sees.
    async fn introduce_to(&mut self, offering: thread::Peer) {
        let introduction = UpdateMessage::Fetching { board: self.board };
        self.send(Some(offering), &introduction).await;
    }

    async fn ask_for_next_chunk(&mut self) {
        let (Some(arriving), Some(fetch)) = (&self.arriving, &mut self.fetching) else {
            return;
        };
        let request = UpdateMessage::ChunkRequest {
            build: arriving.image.build,
            offset: arriving.received,
        };
        let from = fetch.from;
        fetch.retry_at = Some(Instant::now() + CHUNK_TIMEOUT);
        self.send(Some(from), &request).await;
    }

    async fn got_chunk(&mut self, build: BuildId, offset: u32, data: &[u8]) {
        let (Some(arriving), Some(_)) = (&self.arriving, &self.fetching) else {
            return;
        };
        // An answer that comes late, or twice, is to a question that is no
        // longer open.
        if build != arriving.image.build || offset != arriving.received {
            return;
        }
        let size = arriving.image.size.0;

        if self.write(offset, data).await.is_err() {
            self.arriving = None;
            self.fetching = None;
            self.status.publish(UpdateStatus::Settled);
            return;
        }
        if offset + (data.len() as u32) < size {
            if let Some(fetch) = &mut self.fetching {
                fetch.attempts = 0;
            }
            self.ask_for_next_chunk().await;
            return;
        }

        self.fetching = None;
        if self.finish().await.is_ok() && self.mark_for_swap().await.is_ok() {
            Timer::after(RESET_DELAY).await;
            cortex_m::peripheral::SCB::sys_reset()
        }
    }

    /// A board that fetches the staged image asks for a chunk of it.
    async fn asked_for_chunk(&mut self, build: BuildId, offset: u32, from: thread::Peer) {
        let Some(image) = self.staged else {
            return;
        };
        if self.offering.is_none() || build != image.build || offset >= image.size.0 {
            return;
        }
        let len = UpdateMessage::CHUNK_LEN.min((image.size.0 - offset) as usize);
        // A board asks for what comes after what it has. No request follows
        // the last chunk, which is taken to arrive.
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

    /// What is known of the board at `peer` as one that fetches the staged
    /// image, which it is taken for from now on.
    fn fetcher(&mut self, peer: thread::Peer) -> &mut FetcherHeard {
        let now = Instant::now();
        let known = self.fetchers.iter().position(|f| f.peer == peer);
        let index = known.unwrap_or_else(|| {
            let new = FetcherHeard {
                peer,
                board: None,
                received: 0,
                heard_at: now,
            };
            match self.fetchers.push(new) {
                Ok(()) => self.fetchers.len() - 1,
                // No room: it takes the place of the one that has been quiet
                // the longest.
                Err(new) => {
                    let quietest = (0..self.fetchers.len())
                        .min_by_key(|&i| self.fetchers[i].heard_at)
                        .unwrap_or(0);
                    self.fetchers[quietest] = new;
                    quietest
                }
            }
        });
        let fetcher = &mut self.fetchers[index];
        fetcher.heard_at = now;
        fetcher
    }

    fn forget_fetchers(&mut self) {
        self.fetchers.clear();
        self.publish_progress();
    }

    /// Publish how far the fetchers have got, less those that have gone
    /// quiet: done, or gone.
    fn publish_progress(&mut self) {
        let now = Instant::now();
        self.fetchers
            .retain(|fetcher| now - fetcher.heard_at < FETCHER_QUIET);
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

    /// Send to one board, or to all of them. Nothing tells whether it
    /// arrives, and what does not is asked for or said again.
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

    /// The build of the firmware image in the first `len` bytes of the
    /// staging area, if that is what they are.
    async fn build_in_staging_area(&mut self, len: u32) -> Option<BuildId> {
        // Read in pieces that overlap by more than a marker is long, so that
        // one that lies across the end of a piece is whole in the next.
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
        self.arriving = None;
        self.staged = None;
        self.offering = None;
        self.forget_fetchers();
        // The bootloader swaps the staging area with a region one page
        // shorter, which is all an image may fill.
        let room = self.staging_area.capacity() as u32 - PAGE;
        if image.size.0 == 0 || image.size.0 > room {
            self.status.publish(UpdateStatus::Settled);
            return Err(UpdateError::DoesNotFit);
        }

        defmt::info!("update: build {} is arriving", image.build);
        self.arriving = Some(Arriving {
            image,
            received: 0,
            erased_to: 0,
        });
        self.status
            .publish(UpdateStatus::Receiving { image, received: 0 });
        Ok(())
    }

    /// Put the piece of the arriving image that starts at `offset` in the
    /// staging area.
    async fn write(&mut self, offset: u32, data: &[u8]) -> UpdateResult {
        const WORD: usize = <RadioFlash as NorFlash>::WRITE_SIZE;

        let Some(arriving) = &mut self.arriving else {
            return Err(UpdateError::OutOfSequence);
        };
        let len = data.len();
        let end = offset.saturating_add(len as u32);
        let is_last = end == arriving.image.size.0;
        if offset != arriving.received
            || len == 0
            || len > ImageChunk::MAX_LEN
            || end > arriving.image.size.0
            || (len % WORD != 0 && !is_last)
        {
            return Err(UpdateError::OutOfSequence);
        }

        // Flash is written in whole words. Only the last piece can end short
        // of one, and is filled up with what erased flash holds.
        let mut words = [0xff; ImageChunk::MAX_LEN.next_multiple_of(WORD)];
        words[..len].copy_from_slice(data);
        let words = &words[..len.next_multiple_of(WORD)];

        let written_to = offset + words.len() as u32;
        while arriving.erased_to < written_to {
            let page = arriving.erased_to;
            let erased = self.staging_area.erase(page, page + PAGE).await;
            erased.map_err(flash_error)?;
            arriving.erased_to += PAGE;
        }
        let written = self.staging_area.write(offset, words).await;
        written.map_err(flash_error)?;

        arriving.received = end;
        self.status.publish(UpdateStatus::Receiving {
            image: arriving.image,
            received: end,
        });
        Ok(())
    }

    async fn finish(&mut self) -> UpdateResult {
        let image = match self.arriving.take() {
            Some(arriving) if arriving.received == arriving.image.size.0 => arriving.image,
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
        // Boards go by the build they are told when another offers them an
        // image, so the image has to bear out what was said of it.
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

        // A page that a flash loader has put its 0xff filler in reads as
        // erased and cannot be programmed: every write to it ends in a
        // programming error until it is erased again. The state page is in
        // that condition after the whole flash image has been written by a
        // probe or the ROM bootloader, so erase it here. It holds nothing
        // then: no swap is under way while this firmware takes requests.
        let size = self.state_page.capacity() as u32;
        let erased = self.state_page.erase(0, size).await;
        erased.map_err(flash_error)?;

        let marked = self.state.mark_updated().await;
        marked.map_err(flash_error)
    }
}

/// What a failed flash operation comes to for whoever asked, with what it
/// was in the log.
fn flash_error(error: impl defmt::Format) -> UpdateError {
    defmt::error!("update: flash: {}", error);
    UpdateError::Flash
}
