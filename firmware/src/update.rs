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

use core::cell::Cell;
use core::ptr::read_volatile;

use embassy_boot::{AlignedBuffer, FirmwareState, FirmwareUpdaterConfig, State};
use embassy_embedded_hal::flash::partition::Partition;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{
    Mutex,
    raw::{NoopRawMutex, ThreadModeRawMutex},
};
use embassy_time::{Duration, Instant, Timer};
use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use protocol::{
    BuildId, BuildMarker, ImageChunk, NetworkError, NetworkStatus, UpdateError, UpdateImage,
    UpdateResult, UpdateStatus,
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

enum Request {
    Begin(UpdateImage),
    Write(ImageChunk),
    Finish,
    Apply,
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

struct Shared {
    requests: request::Channel<Request, UpdateResult>,
    status: Status,
}

/// Make the two ends of firmware updates. Panics if called a second time.
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

/// How the rest of the firmware gets an update in.
pub struct Handle {
    requests: request::Client<Request, UpdateResult>,
    status: &'static Status,
}

impl Handle {
    pub fn status(&self) -> UpdateStatus {
        self.status.get()
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
}

impl Service {
    /// Serve updates on a board whose CPU2 runs a wireless stack. `network`
    /// is what an update on trial is judged by.
    pub async fn run(self, flash: &radio_flash::Shared<'_>, network: thread::Monitor) -> ! {
        let FirmwareUpdaterConfig { dfu, state } =
            FirmwareUpdaterConfig::from_linkerfile(flash, flash);
        let mut aligned = AlignedBuffer([0; <RadioFlash as NorFlash>::WRITE_SIZE]);
        let mut updates = Updates {
            staging_area: dfu,
            state_page: state.clone(),
            state: FirmwareState::new(state, &mut aligned.0),
            requests: self.requests,
            status: self.status,
            arriving: None,
            staged: None,
        };

        match updates.state.get_state().await {
            Ok(State::Swap) => updates.trial(network).await,
            Ok(State::Revert) => {
                defmt::warn!("update: the last one did not work out, and was put back");
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

struct Updates<'a, 'd> {
    /// Offsets into it are offsets into the image.
    staging_area: Region<'a, 'd>,
    /// What the bootloader is to do at the next reset, and the page that is
    /// kept in.
    state: FirmwareState<'a, Region<'a, 'd>>,
    state_page: Region<'a, 'd>,
    requests: request::Server<Request, UpdateResult>,
    status: &'static Status,
    arriving: Option<Arriving>,
    staged: Option<UpdateImage>,
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
        loop {
            let (pending, request) = self.requests.receive().await;
            let apply = matches!(request, Request::Apply);
            let outcome = match request {
                Request::Begin(image) => self.begin(image),
                Request::Write(chunk) => self.write(&chunk).await,
                Request::Finish => self.finish().await,
                Request::Apply => self.mark_for_swap().await,
            };
            let applying = apply && outcome.is_ok();
            self.requests.answer(pending, outcome);

            if applying {
                Timer::after(RESET_DELAY).await;
                cortex_m::peripheral::SCB::sys_reset()
            }
        }
    }

    fn begin(&mut self, image: UpdateImage) -> UpdateResult {
        self.arriving = None;
        self.staged = None;
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

    async fn write(&mut self, chunk: &ImageChunk) -> UpdateResult {
        const WORD: usize = <RadioFlash as NorFlash>::WRITE_SIZE;

        let Some(arriving) = &mut self.arriving else {
            return Err(UpdateError::OutOfSequence);
        };
        let len = chunk.data.len();
        let end = chunk.offset.saturating_add(len as u32);
        let is_last = end == arriving.image.size.0;
        if chunk.offset != arriving.received
            || len == 0
            || end > arriving.image.size.0
            || (len % WORD != 0 && !is_last)
        {
            return Err(UpdateError::OutOfSequence);
        }

        // Flash is written in whole words. Only the last chunk can end short
        // of one, and is filled up with what erased flash holds.
        let mut words = [0xff; ImageChunk::MAX_LEN.next_multiple_of(WORD)];
        words[..len].copy_from_slice(&chunk.data);
        let words = &words[..len.next_multiple_of(WORD)];

        let written_to = chunk.offset + words.len() as u32;
        while arriving.erased_to < written_to {
            let page = arriving.erased_to;
            let erased = self.staging_area.erase(page, page + PAGE).await;
            erased.map_err(flash_error)?;
            arriving.erased_to += PAGE;
        }
        let written = self.staging_area.write(chunk.offset, words).await;
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
