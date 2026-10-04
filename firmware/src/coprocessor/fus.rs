//! Installing and removing coprocessor firmware through FUS.
//!
//! FUS installs an image that it finds in flash right below its own secure
//! area: a newer FUS, or a wireless stack. The image arrives here in chunks
//! and is written to that place, and FUS is then told to take it from there.
//! Removing a stack is one command to FUS, once CPU2 has been restarted into
//! FUS in place of the stack.
//!
//! FUS resets the whole chip whenever it sees fit, and between resets it can
//! claim to be idle with the work still ahead of it. So what it was set to do
//! is written down where it survives a reset ([`Pending`]), and each boot of
//! this firmware takes it from there until the result shows in what CPU2
//! reports having.

use core::{mem::MaybeUninit, ptr};

use embassy_futures::select::{Either, select};
use embassy_stm32::{
    flash::{Blocking, Flash},
    pac::FLASH,
};
use embassy_stm32_wpan::{shci::SchiCommandStatus, sub::sys::Sys};
use embassy_time::{Duration, Instant, Timer};
use protocol::{
    CoprocessorError, CoprocessorFirmware, CoprocessorResult, CoprocessorStatus, FusError,
    FusState, ImageChunk, ImageSize, Version,
};

use super::Status;
use crate::{dfu, request, update};

pub enum Request {
    Begin(ImageSize),
    Write(ImageChunk),
    Finish,
    Uninstall,
}

type Requests = request::Server<Request, CoprocessorResult>;

/// What FUS has been set to do and is not known to be done with.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Pending {
    /// Installing an image. Carries what was installed beforehand, since the
    /// install is over when that has changed.
    Install {
        from: Installed,
    },
    Uninstall(UninstallStep),
}

/// The versions of FUS and of the wireless stack, if there is one.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Installed {
    fus: Version,
    stack: Option<Version>,
}

impl From<CoprocessorFirmware> for Installed {
    fn from(firmware: CoprocessorFirmware) -> Self {
        Installed {
            fus: firmware.fus,
            stack: firmware.stack.map(|stack| stack.version),
        }
    }
}

/// Removing the stack takes a reset between each of these.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum UninstallStep {
    /// Restart CPU2 into FUS.
    EnterFus,
    /// Tell FUS to delete the stack.
    Delete,
    /// See whether the stack is gone.
    Check,
}

/// [`Pending`] as it is kept: in RAM that is neither initialised at boot nor
/// lost in a reset. The words are a magic number, to tell this from what a
/// power-up leaves there, then what is pending, then for an install the
/// versions of FUS and of the stack it started from.
#[unsafe(link_section = ".uninit.FUS_PENDING")]
static mut PENDING: MaybeUninit<[u32; 4]> = MaybeUninit::uninit();
const PENDING_MAGIC: u32 = 0xF05B_0512;

const NOTHING: u32 = 0;
const INSTALL: u32 = 1;
const UNINSTALL_ENTER_FUS: u32 = 2;
const UNINSTALL_DELETE: u32 = 3;
const UNINSTALL_CHECK: u32 = 4;

/// No version is 0.0.0, which leaves a word of zero for "no stack".
fn version_word(version: Version) -> u32 {
    u32::from_be_bytes([version.major, version.minor, version.patch, 0])
}

fn word_version(word: u32) -> Version {
    let [major, minor, patch, _] = word.to_be_bytes();
    Version {
        major,
        minor,
        patch,
    }
}

pub fn pending() -> Option<Pending> {
    // SAFETY: nothing else touches this RAM, and any four words are a valid
    // value of what is read.
    let words = unsafe { ptr::read_volatile((&raw const PENDING).cast::<[u32; 4]>()) };
    match words {
        [PENDING_MAGIC, INSTALL, fus, stack] => Some(Pending::Install {
            from: Installed {
                fus: word_version(fus),
                stack: (stack != 0).then(|| word_version(stack)),
            },
        }),
        [PENDING_MAGIC, UNINSTALL_ENTER_FUS, ..] => {
            Some(Pending::Uninstall(UninstallStep::EnterFus))
        }
        [PENDING_MAGIC, UNINSTALL_DELETE, ..] => Some(Pending::Uninstall(UninstallStep::Delete)),
        [PENDING_MAGIC, UNINSTALL_CHECK, ..] => Some(Pending::Uninstall(UninstallStep::Check)),
        _ => None,
    }
}

pub fn set_pending(pending: Option<Pending>) {
    let words = match pending {
        None => [PENDING_MAGIC, NOTHING, 0, 0],
        Some(Pending::Install { from }) => [
            PENDING_MAGIC,
            INSTALL,
            version_word(from.fus),
            from.stack.map_or(0, version_word),
        ],
        Some(Pending::Uninstall(step)) => {
            let step = match step {
                UninstallStep::EnterFus => UNINSTALL_ENTER_FUS,
                UninstallStep::Delete => UNINSTALL_DELETE,
                UninstallStep::Check => UNINSTALL_CHECK,
            };
            [PENDING_MAGIC, step, 0, 0]
        }
    };
    // SAFETY: nothing else touches this RAM.
    unsafe { ptr::write_volatile((&raw mut PENDING).cast::<[u32; 4]>(), words) };
    // While FUS is at work it resets the chip as it sees fit, and a watchdog
    // reset is no crash.
    dfu::fus_busy(pending.is_some());
}

/// Serve requests on a board whose wireless stack is running: FUS is not
/// there to install anything, but the stack can be removed.
pub async fn serve_beside_stack(requests: Requests, status: &Status) -> ! {
    loop {
        let (pending, request) = requests.receive().await;
        match request {
            Request::Uninstall => {
                set_pending(Some(Pending::Uninstall(UninstallStep::EnterFus)));
                status.publish(CoprocessorStatus::Starting);
                requests.answer(pending, Ok(()));
                // Time for that answer to get out. The next boot does the
                // rest: at boot, nothing else is using CPU2 yet.
                Timer::after_millis(100).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
            _ => requests.answer(pending, Err(CoprocessorError::StackRunning)),
        }
    }
}

/// Restart CPU2 into FUS, from its wireless stack.
pub async fn enter(sys: &mut Sys<'_>) {
    // Asked for the state of FUS, a stack says that FUS is not running. Asked
    // again, it resets the chip into FUS.
    sys.shci_c2_fus_get_state().await;
    sys.shci_c2_fus_get_state().await;
    Timer::after_secs(5).await;
    defmt::warn!("fus: the wireless stack did not give way to FUS");
}

const PAGE: u32 = 4096;

/// Flash is written in words of this many bytes.
const WORD: u32 = 8;

/// Where the application's part of flash ends, as an offset into flash: the
/// last of it is where a firmware update is staged.
fn application_end() -> u32 {
    update::staging_end()
}

/// `FUS_STATE_IDLE` and `FUS_STATE_ERROR`; everything in between is one
/// operation or another in progress (AN5185).
const FUS_STATE_IDLE: u8 = 0x00;
const FUS_STATE_ERROR: u8 = 0xFF;

const POLL_INTERVAL: Duration = Duration::from_millis(500);

/// How long FUS may say it is idle, with nothing to show for an install,
/// before that is taken as its last word. It said so for 14 s while it
/// installed a 420 KB stack.
const INSTALL_SETTLE: Duration = Duration::from_secs(60);

/// How long FUS has to stay idle after deleting a stack for that to count.
const DELETE_SETTLE: Duration = Duration::from_secs(5);

pub struct Installer<'d> {
    pub sys: Sys<'d>,
    pub flash: Flash<'d, Blocking>,
    pub firmware: CoprocessorFirmware,
    pub requests: Requests,
    pub status: &'static Status,
}

/// An image on its way into flash. All offsets are into flash.
struct Staging {
    start: u32,
    len: u32,
    written: u32,
    /// Flash from `start` up to here has been erased.
    erased_to: u32,
}

impl Installer<'_> {
    pub async fn run(mut self) -> ! {
        let requests = self.requests;
        let resumed = async {
            match pending() {
                Some(Pending::Install { from }) => self.installed(from).await,
                Some(Pending::Uninstall(step)) => self.uninstall(step).await,
                None => self.idle(Duration::MIN).await,
            }
        };
        let mut state = while_busy(requests, resumed).await;
        set_pending(None);

        if state == FusState::Idle && self.firmware.stack.is_some() {
            // FUS has a stack and nothing left to do: this board is meant to
            // run its stack.
            while_busy(requests, self.start_stack()).await;
        }

        let mut staging = None;
        loop {
            self.publish(state);

            let (pending, request) = requests.receive().await;
            let result = match request {
                Request::Begin(size) => self.begin(size).map(|s| staging = Some(s)),
                Request::Write(chunk) => match &mut staging {
                    Some(staging) => self.write(staging, &chunk),
                    None => Err(CoprocessorError::OutOfSequence),
                },
                Request::Finish => match staging.take() {
                    Some(staging) if staging.written == staging.len => {
                        match self.install().await {
                            Ok(from) => {
                                // Published before the answer, so that
                                // whoever asked never reads a status older
                                // than it.
                                self.publish(FusState::Busy);
                                requests.answer(pending, Ok(()));
                                // If FUS takes the image, it resets the chip
                                // and this never gets to its end.
                                state = while_busy(requests, self.installed(from)).await;
                                set_pending(None);
                                continue;
                            }
                            Err(e) => Err(e),
                        }
                    }
                    _ => Err(CoprocessorError::OutOfSequence),
                },
                // There is a stack here only if FUS would not start it.
                Request::Uninstall if self.firmware.stack.is_some() => {
                    set_pending(Some(Pending::Uninstall(UninstallStep::Delete)));
                    self.status.publish(CoprocessorStatus::Starting);
                    requests.answer(pending, Ok(()));
                    Timer::after_millis(100).await;
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Request::Uninstall => Ok(()),
            };
            requests.answer(pending, result);
        }
    }

    fn publish(&self, state: FusState) {
        self.status.publish(CoprocessorStatus::Fus {
            firmware: self.firmware,
            state,
        });
    }

    /// Wait until FUS has been idle for `settle`, or has failed.
    async fn idle(&mut self, settle: Duration) -> FusState {
        let mut idle_since = None;
        loop {
            match self.sys.shci_c2_fus_get_state().await {
                (FUS_STATE_IDLE, 0) => {
                    if idle_since.get_or_insert_with(Instant::now).elapsed() >= settle {
                        return FusState::Idle;
                    }
                }
                (FUS_STATE_IDLE | FUS_STATE_ERROR, error) => {
                    return FusState::Failed(fus_error(error));
                }
                _ => idle_since = None,
            }
            self.publish(FusState::Busy);
            Timer::after(POLL_INTERVAL).await;
        }
    }

    /// See an install through, in the boot it was begun in or a later one.
    async fn installed(&mut self, from: Installed) -> FusState {
        if Installed::from(self.firmware) != from {
            // The image is in. What FUS does after that is go idle.
            return self.idle(Duration::MIN).await;
        }
        // Nothing has changed yet. FUS resets the chip when something does,
        // so to get to the end of this wait is to learn that nothing will.
        self.idle(INSTALL_SETTLE).await
    }

    /// Take the next step of removing the stack. Returns once there is
    /// nothing more to do about it.
    async fn uninstall(&mut self, step: UninstallStep) -> FusState {
        if self.firmware.stack.is_none() {
            return self.idle(Duration::MIN).await;
        }
        if step == UninstallStep::Check {
            // The stack has outlived the delete: leave it be.
            defmt::warn!("fus: the wireless stack is still there");
            return self.idle(Duration::MIN).await;
        }

        let state = self.idle(Duration::MIN).await;
        if state != FusState::Idle {
            return state;
        }
        set_pending(Some(Pending::Uninstall(UninstallStep::Check)));
        self.publish(FusState::Busy);
        let status = self.sys.shci_c2_fus_fw_delete().await;
        defmt::info!("fus: delete -> {}", status);
        let state = self.idle(DELETE_SETTLE).await;
        if state != FusState::Idle {
            return state;
        }
        // CPU2 says what it has only when it boots.
        cortex_m::peripheral::SCB::sys_reset()
    }

    async fn start_stack(&mut self) {
        self.publish(FusState::Busy);
        dfu::fus_busy(true);
        let status = self.sys.shci_c2_fus_startws().await;
        // FUS answers, and then resets the chip into the stack. To still be
        // here a while later means that it did not.
        Timer::after_secs(10).await;
        dfu::fus_busy(false);
        defmt::warn!("fus: the wireless stack did not start ({})", status);
    }

    /// Find the place in flash for an image of `size`.
    fn begin(&mut self, ImageSize(len): ImageSize) -> Result<Staging, CoprocessorError> {
        // Where FUS looks, and where ST's release notes put each image: on
        // the page boundary that leaves the image ending just below the
        // secure flash area.
        let secure_start = FLASH.sfr().read().sfsa() as u32 * PAGE;
        let start = secure_start
            .checked_sub(len)
            .map(|start| start - start % PAGE)
            .filter(|&start| len > 0 && start >= application_end())
            .ok_or(CoprocessorError::DoesNotFit)?;

        defmt::info!("fus: staging {} bytes at {:#x}", len, 0x0800_0000 + start);
        Ok(Staging {
            start,
            len,
            written: 0,
            erased_to: start,
        })
    }

    fn write(&mut self, staging: &mut Staging, chunk: &ImageChunk) -> CoprocessorResult {
        let len = chunk.data.len() as u32;
        let end = chunk.offset + len;
        // Only the image's last chunk may stop short of a flash word.
        if chunk.offset != staging.written
            || len == 0
            || end > staging.len
            || (len % WORD != 0 && end != staging.len)
        {
            return Err(CoprocessorError::OutOfSequence);
        }

        // Pad the image's tail out to a whole word with erased bytes.
        let mut data = [0xff; ImageChunk::MAX_LEN];
        data[..chunk.data.len()].copy_from_slice(&chunk.data);
        let data = &data[..chunk.data.len().next_multiple_of(WORD as usize)];

        let at = staging.start + chunk.offset;
        // One page at a time, as the image gets there, so that no request
        // holds everything else up for longer than one erase.
        while staging.erased_to < at + data.len() as u32 {
            self.flash
                .blocking_erase(staging.erased_to, staging.erased_to + PAGE)
                .map_err(|_| CoprocessorError::Flash)?;
            staging.erased_to += PAGE;
        }
        self.flash
            .blocking_write(at, data)
            .map_err(|_| CoprocessorError::Flash)?;

        staging.written = end;
        Ok(())
    }

    /// Tell FUS to install the image it finds. Returns what the install
    /// starts from.
    async fn install(&mut self) -> Result<Installed, CoprocessorError> {
        let from = self.firmware.into();
        set_pending(Some(Pending::Install { from }));
        match self.sys.shci_c2_fus_fwupgrade(0, 0).await {
            Ok(SchiCommandStatus::ShciSuccess) => Ok(from),
            status => {
                set_pending(None);
                defmt::warn!("fus: upgrade command -> {}", status);
                Err(CoprocessorError::Rejected)
            }
        }
    }
}

/// Do `work`, which keeps CPU2 to itself, and tell whoever asks for anything
/// in the meantime that FUS is busy.
async fn while_busy<T>(requests: Requests, work: impl Future<Output = T>) -> T {
    let turn_down = async {
        loop {
            let (pending, _) = requests.receive().await;
            requests.answer(pending, Err(CoprocessorError::Busy));
        }
    };
    match select(work, turn_down).await {
        Either::First(done) => done,
        Either::Second(never) => never,
    }
}

fn fus_error(code: u8) -> FusError {
    match code {
        0x01 => FusError::ImageNotFound,
        0x02 => FusError::ImageCorrupt,
        0x03 => FusError::ImageNotAuthentic,
        0x04 => FusError::NotEnoughSpace,
        0x05 => FusError::Aborted,
        0x06 => FusError::EraseFailed,
        0x07 => FusError::WriteFailed,
        0x08 => FusError::StAuthTagNotFound,
        0x09 => FusError::CustomerAuthTagNotFound,
        0x0A => FusError::AuthKeyLocked,
        0x11 => FusError::RollbackRefused,
        other => FusError::Other(other),
    }
}
