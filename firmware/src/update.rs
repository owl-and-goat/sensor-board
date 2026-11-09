//! Firmware updates. A new image is written to the staging area, the flash
//! region after the running firmware (`DFU` in memory.x), and checked
//! against its digest. At the next reset the bootloader (`bootloader/`)
//! swaps the two. The update then runs on trial. It is confirmed once the
//! board is back on its network, has received a datagram, and has run for
//! three minutes. If the board resets before that, or doesn't get there in
//! time, the bootloader swaps the old firmware back. A board whose firmware
//! fails several times in a row after the trial goes back to the old
//! firmware too ([`go_back_to_previous_firmware`]).
//!
//! An image comes from the host over USB, or from the network: a host offers
//! it to each board through a border router, and a board that runs another
//! build fetches it.
//!
//! [`init`] returns the [`Handle`] for the rest of the firmware and the
//! [`Service`]. The service runs inside the coprocessor task, because flash
//! writes have to be coordinated with CPU2.

use core::cell::Cell;
use core::ptr::read_volatile;

use embassy_boot::{AlignedBuffer, FirmwareState, FirmwareUpdaterConfig, State};
use embassy_embedded_hal::flash::partition::Partition;
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::{
    Mutex,
    raw::{NoopRawMutex, ThreadModeRawMutex},
};
use embassy_time::{Duration, Instant, Timer};
use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use protocol::{
    BoardId, BuildId, BuildMarker, ImageChunk, NetworkError, NetworkStatus, UpdateError,
    UpdateImage, UpdateMessage, UpdateResult, UpdateStatus,
};
use sha2::{Digest, Sha256};
use static_cell::StaticCell;

use crate::{
    radio_flash::{self, PAGE, RadioFlash},
    request, thread,
};

unsafe extern "C" {
    static __sfirmware: u8;
    static __efirmware: u8;
    static __bootloader_state_start: u32;
    static __bootloader_dfu_start: u32;
}

/// The magic number that tells the bootloader to swap, as embassy-boot
/// 0.7 writes it at the start of the state page (`SWAP_MAGIC`).
const SWAP_MAGIC: u8 = 0xF0;

/// Have the bootloader put back the firmware that the last update replaced,
/// and restart. Returns if the board doesn't have that firmware any more,
/// or if the state page can't be written.
///
/// After a swap, the staging area holds the firmware that was replaced, one
/// page in. Its first page still holds a copy of the first page of the
/// running firmware, and whatever writes the staging area afterwards starts
/// with that page. So the old firmware is still there for as long as the
/// two first pages match. The bootloader puts it back when the state page
/// says that a swap is complete but wasn't confirmed: the swap magic
/// number, followed by a progress word for each step of the swap, two for
/// each page. That is what this writes.
///
/// Runs at the start, before CPU2 does, so it writes flash directly instead
/// of through `RadioFlash`.
pub fn go_back_to_previous_firmware() {
    const WORD: u32 = embassy_stm32::flash::WRITE_SIZE as u32;
    let base = embassy_stm32::flash::FLASH_BASE as u32;
    let firmware = (&raw const __sfirmware) as u32;
    let len = (&raw const __efirmware) as u32 - firmware;
    // The linker gives the offsets as the addresses of the symbols.
    let state = (&raw const __bootloader_state_start) as u32;
    let staging = base + (&raw const __bootloader_dfu_start) as u32;

    // SAFETY: both are in CPU1's flash, which nothing writes while this
    // reads it.
    let (running, staged, previous) = unsafe {
        let flash = |at: u32, len: u32| core::slice::from_raw_parts(at as *const u8, len as usize);
        (
            flash(firmware, PAGE),
            flash(staging, PAGE),
            flash(staging + PAGE, len),
        )
    };
    if running != staged || !is_firmware(previous) {
        return;
    }

    // SAFETY: nothing else uses the flash controller this early.
    let p = unsafe { embassy_stm32::Peripherals::steal() };
    // See `main`: FUS and the ROM bootloader leave this flag set, and the
    // first flash operation fails while it is.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));
    let mut flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH);
    let written = (|| {
        flash.blocking_erase(state, state + PAGE)?;
        // The first progress word is at the third word of the page.
        for step in 0..2 * len / PAGE {
            flash.blocking_write(state + (2 + step) * WORD, &[0; WORD as usize])?;
        }
        // Last, so that the page means nothing until it is complete.
        flash.blocking_write(state, &[SWAP_MAGIC; WORD as usize])
    })();
    if written.is_ok() {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// Whether `image` looks like a firmware for this board, other than the
/// running one: its vector table starts the stack in RAM and the code in
/// the firmware's region, and it has a build marker.
fn is_firmware(image: &[u8]) -> bool {
    let word =
        |at: usize| u32::from_le_bytes([image[at], image[at + 1], image[at + 2], image[at + 3]]);
    let firmware = (&raw const __sfirmware) as u32..(&raw const __efirmware) as u32;
    let ram = 0x2000_0000..=0x2003_0000;
    ram.contains(&word(0))
        && firmware.contains(&word(4))
        && BuildMarker::find(image).is_some_and(|found| found != build())
}

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

/// How long an update on trial has to pass it: to get the board back on its
/// network and receive a datagram. A board that finds no network forms its
/// own after about two minutes.
const TRIAL_TIMEOUT: Duration = Duration::from_secs(300);

/// How long an update stays on trial at least. Until 2026-11-08 an update
/// was confirmed as soon as the board was back on its network, a few
/// seconds after it started. That day an update that crashed at the first
/// datagram it received was confirmed on every board, and only then crashed
/// them: they started to listen for offers after the trial.
const TRIAL_MIN: Duration = Duration::from_secs(180);

/// How often a board on trial checks its network status.
const TRIAL_POLL: Duration = Duration::from_secs(1);

/// How often a board on trial sends a datagram to every board, itself
/// included, until it has received a datagram.
const TRIAL_DATAGRAM_INTERVAL: Duration = Duration::from_secs(30);

/// Delay between an apply and the reset, to let the response reach the
/// caller.
const RESET_DELAY: Duration = Duration::from_millis(200);

/// How long a fetching board waits for a requested chunk before it asks
/// again, and how many times in a row it asks. After that it stops until
/// the next offer.
const CHUNK_TIMEOUT: Duration = Duration::from_millis(1500);
const MAX_ATTEMPTS: u8 = 8;

/// Wait until `at`. `None` means never.
async fn until(at: Option<Instant>) {
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

struct Shared {
    requests: request::Channel<Request, UpdateResult>,
    status: Status,
}

/// Create the [`Handle`] and the [`Service`]. Panics if called a second time.
pub fn init() -> (Handle, Service) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        status: Status(Mutex::new(Cell::new(UpdateStatus::Starting))),
    });

    let (client, server) = shared.requests.split();
    let handle = Handle {
        requests: client,
        status: &shared.status,
    };
    let service = Service {
        requests: server,
        status: &shared.status,
    };
    (handle, service)
}

/// Stages and applies updates for the rest of the firmware.
pub struct Handle {
    requests: request::Client<Request, UpdateResult>,
    status: &'static Status,
}

impl Handle {
    pub fn status(&self) -> UpdateStatus {
        self.status.get()
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
            incoming: None,
            staged: None,
            socket,
            listening: false,
            fetching: None,
            rolled_back: None,
            board: BoardId(embassy_stm32::uid::uid()),
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

/// The state of fetching an image, chunk by chunk, from the host that offers
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
    incoming: Option<Incoming>,
    staged: Option<UpdateImage>,
    /// The UDP socket for [`UpdateMessage`]s.
    socket: thread::Socket,
    /// Whether the socket receives datagrams.
    listening: bool,
    /// Set while the image being received comes from the network.
    fetching: Option<Fetch>,
    /// The build of an update that was rolled back after its trial. It is
    /// not fetched again.
    rolled_back: Option<BuildId>,
    /// This board's ID.
    board: BoardId,
}

impl Updates<'_, '_> {
    /// Run the trial of the running firmware, which is an update that was
    /// just swapped in. Returns once the update is confirmed. If the trial
    /// fails, this resets the board, and the bootloader restores the old
    /// firmware.
    ///
    /// The update has to get the board back on its network, receive a
    /// datagram, and keep running for [`TRIAL_MIN`]. The datagram can be an
    /// offer, or the board's own message to every board, which it sends
    /// every [`TRIAL_DATAGRAM_INTERVAL`] until one arrives. Offers are not
    /// answered until the update is kept: the host takes an answer with the
    /// new build to mean that.
    async fn run_trial(&mut self, network: thread::Monitor) {
        defmt::info!("update: on trial as build {}", build());
        self.status.publish(UpdateStatus::OnTrial);
        let start = Instant::now();
        let mut received = false;
        let mut next_datagram = start;

        loop {
            let now = Instant::now();
            let attached = match network.status() {
                NetworkStatus::Configured(link) => Some(link.role.is_attached()),
                // There is no network to rejoin or to receive from, so
                // running is the only test.
                NetworkStatus::Unconfigured
                | NetworkStatus::Unavailable(NetworkError::NoThreadStack) => None,
                _ => Some(false),
            };
            let passed = match attached {
                Some(attached) => attached && received,
                None => true,
            };
            if passed && now >= start + TRIAL_MIN {
                break;
            }
            if now > start + TRIAL_TIMEOUT {
                defmt::warn!("update: did not pass its trial; giving it up");
                cortex_m::peripheral::SCB::sys_reset()
            }

            if attached == Some(true) && !received && now >= next_datagram {
                next_datagram = now + TRIAL_DATAGRAM_INTERVAL;
                if self.listen().await {
                    let running = UpdateMessage::Running {
                        board: self.board,
                        build: build(),
                    };
                    self.broadcast(&running).await;
                }
            }

            let next = select3(
                self.requests.receive(),
                self.socket.receive(),
                Timer::after(TRIAL_POLL),
            );
            match next.await {
                Either3::First((pending, _)) => {
                    self.requests.respond(pending, Err(UpdateError::OnTrial));
                }
                Either3::Second(_) => received = true,
                Either3::Third(()) => {}
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
        if !self.listen().await {
            defmt::warn!("update: no updates over the network");
        }

        loop {
            let fetch_due = self.fetching.as_ref().and_then(|fetch| fetch.retry_at);
            let next = select3(
                self.requests.receive(),
                self.socket.receive(),
                until(fetch_due),
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
                // An image from the host over USB replaces one that is being
                // fetched from the network.
                self.fetching = None;
                self.begin(image)
            }
            Request::Write(chunk) if self.fetching.is_none() => {
                self.write(chunk.offset, &chunk.data).await
            }
            Request::Finish if self.fetching.is_none() => self.finish().await,
            Request::Write(_) | Request::Finish => Err(UpdateError::OutOfSequence),
            Request::Apply => self.mark_for_swap().await,
        };
        let applying = apply && outcome.is_ok();
        self.requests.respond(pending, outcome);

        if applying {
            Timer::after(RESET_DELAY).await;
            cortex_m::peripheral::SCB::sys_reset()
        }
    }

    /// Request the missing chunk again, if that is due.
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
    }

    async fn handle_datagram(&mut self, received: thread::Received) {
        let thread::Received { from, datagram } = received;
        match UpdateMessage::decode(&datagram) {
            Some(UpdateMessage::Offer(image)) => self.handle_offer(image, from).await,
            Some(UpdateMessage::Chunk {
                build,
                offset,
                data,
            }) => self.handle_chunk(build, offset, &data).await,
            // For a board that offers an image, which this firmware doesn't:
            // a board on an older firmware may still send them to one that
            // does.
            Some(
                UpdateMessage::ChunkRequest { .. }
                | UpdateMessage::Fetching { .. }
                | UpdateMessage::Running { .. },
            ) => {}
            // Not a message this firmware understands. A newer firmware may
            // have sent it.
            None => {}
        }
    }

    /// Handle an offer of `image`. Fetch the image, unless this board already
    /// runs it, has it staged, or has rolled it back.
    async fn handle_offer(&mut self, image: UpdateImage, from: thread::Peer) {
        let running = UpdateMessage::Running {
            board: self.board,
            build: build(),
        };
        self.send(from, &running).await;

        if image.build == build()
            || self.staged == Some(image)
            || self.rolled_back == Some(image.build)
        {
            return;
        }

        // A fetch of another image that has given up, because the host that
        // offered it stopped answering, gives way to this one. Until
        // 2026-11-01 the stalled image blocked every other offer until the
        // board restarted: two boards ignored a push that way, after the
        // push before it had been stopped partway.
        if let (Some(incoming), Some(fetch)) = (&self.incoming, &self.fetching)
            && incoming.image != image
            && fetch.retry_at.is_none()
        {
            defmt::info!(
                "update: giving up the stalled fetch of build {}",
                incoming.image.build
            );
            self.incoming = None;
            self.fetching = None;
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
            // Another image is being received, from the host over USB, or
            // fetched from the network.
            (Some(_), _) => {}
        }
    }

    /// Send this board's ID to the host or board that offers the image, so
    /// that it can report whose progress it is tracking.
    async fn send_board_id(&mut self, peer: thread::Peer) {
        let message = UpdateMessage::Fetching { board: self.board };
        self.send(peer, &message).await;
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
        self.send(from, &request).await;
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

    /// Start receiving update messages, unless the socket already does.
    /// Returns whether it does.
    async fn listen(&mut self) -> bool {
        if !self.listening {
            self.listening = self.socket.listen(true).await.is_ok();
        }
        self.listening
    }

    /// Send `message` to `to`. Delivery is not confirmed: chunk requests are
    /// retried, and offers are repeated.
    async fn send(&mut self, to: thread::Peer, message: &UpdateMessage) {
        let Some(datagram) = encode(message) else {
            return;
        };
        if let Err(e) = self.socket.send_to(to, datagram).await {
            defmt::debug!("update: not sent: {}", e);
        }
    }

    /// Send `message` to every board on the network, this one included.
    async fn broadcast(&mut self, message: &UpdateMessage) {
        let Some(datagram) = encode(message) else {
            return;
        };
        if let Err(e) = self.socket.broadcast(datagram).await {
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

fn encode(message: &UpdateMessage) -> Option<thread::Datagram> {
    let mut datagram = thread::Datagram::new();
    // Cannot fail: a datagram has room for the longest message.
    let _ = datagram.resize_default(UpdateMessage::MAX_LEN);
    let len = message.encode(&mut datagram).map(<[u8]>::len)?;
    datagram.truncate(len);
    Some(datagram)
}

/// Log a flash error and convert it to the error returned to the caller.
fn flash_error(error: impl defmt::Format) -> UpdateError {
    defmt::error!("update: flash: {}", error);
    UpdateError::Flash
}
