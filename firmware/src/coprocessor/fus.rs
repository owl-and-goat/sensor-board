//! Installing and removing coprocessor firmware through FUS.
//!
//! FUS installs an image that it finds in flash just below its own secure
//! area: a newer FUS, or a wireless stack. This module receives the image in
//! chunks, writes it there, and then tells FUS to install it. Removing a
//! stack takes one FUS command, after CPU2 has been restarted into FUS.
//!
//! FUS resets the whole chip at times of its own choosing, and between
//! resets it can report itself idle while the work is still ahead of it. So
//! the operation in progress is stored where it survives a reset
//! ([`Pending`]). Each boot of this firmware resumes it, until the firmware
//! versions that CPU2 reports show the result.

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

/// An operation that FUS was told to do, and is not yet known to have
/// finished.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Pending {
    /// Installing an image. `from` is what was installed before: the install
    /// is over once the installed versions differ from it.
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

/// The steps of removing the stack. A reset separates each from the next.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum UninstallStep {
    /// Restart CPU2 into FUS.
    EnterFus,
    /// Tell FUS to delete the stack.
    Delete,
    /// Check whether the stack is gone.
    Check,
}

/// [`Pending`] as stored: in RAM that is not initialised at boot and that
/// survives a reset. The four words are a magic number (to distinguish
/// stored state from the contents of RAM after power-up), the operation,
/// and for an install the FUS and stack versions it started from.
#[unsafe(link_section = ".uninit.FUS_PENDING")]
static mut PENDING: MaybeUninit<[u32; 4]> = MaybeUninit::uninit();
const PENDING_MAGIC: u32 = 0xF05B_0512;

const NOTHING: u32 = 0;
const INSTALL: u32 = 1;
const UNINSTALL_ENTER_FUS: u32 = 2;
const UNINSTALL_DELETE: u32 = 3;
const UNINSTALL_CHECK: u32 = 4;

/// No real version is 0.0.0, so a zero word can stand for "no stack".
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
    // SAFETY: nothing else accesses this RAM, and every bit pattern is a
    // valid `[u32; 4]`.
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
    // FUS resets the chip while it works, so a watchdog reset during that
    // time must not count as a crash.
    dfu::fus_busy(pending.is_some());
}

/// Serve requests while the wireless stack is running. Installing needs
/// FUS, so only an uninstall is accepted.
pub async fn serve_beside_stack(requests: Requests, status: &Status) -> ! {
    loop {
        let (pending, request) = requests.receive().await;
        match request {
            Request::Uninstall => {
                set_pending(Some(Pending::Uninstall(UninstallStep::EnterFus)));
                status.publish(CoprocessorStatus::Starting);
                requests.answer(pending, Ok(()));
                // Give the response time to reach the host. The next boot
                // does the rest, because at boot nothing else is using CPU2
                // yet.
                Timer::after_millis(100).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
            _ => requests.answer(pending, Err(CoprocessorError::StackRunning)),
        }
    }
}

/// Restart CPU2 into FUS, from its wireless stack.
pub async fn enter(sys: &mut Sys<'_>) {
    // The first FUS get-state command makes the stack answer that FUS is
    // not running. The second makes it reset the chip into FUS.
    sys.shci_c2_fus_get_state().await;
    sys.shci_c2_fus_get_state().await;
    Timer::after_secs(5).await;
    defmt::warn!("fus: the wireless stack did not give way to FUS");
}

const PAGE: u32 = 4096;

/// Flash is written in words of this many bytes.
const WORD: u32 = 8;

/// The end of the application's part of flash, as an offset into flash. The
/// update staging area is the last thing in it.
fn application_end() -> u32 {
    update::staging_end()
}

/// `FUS_STATE_IDLE` and `FUS_STATE_ERROR`; everything in between is one
/// operation or another in progress (AN5185).
const FUS_STATE_IDLE: u8 = 0x00;
const FUS_STATE_ERROR: u8 = 0xFF;

const POLL_INTERVAL: Duration = Duration::from_millis(500);

/// How long FUS has to report idle, with no change in what is installed,
/// before an install counts as having done nothing. FUS reported idle for
/// 14 s in the middle of installing a 420 KB stack.
const INSTALL_SETTLE: Duration = Duration::from_secs(60);

/// How long FUS has to stay idle after a delete command before the delete
/// counts as finished.
const DELETE_SETTLE: Duration = Duration::from_secs(5);

pub struct Installer<'d> {
    pub sys: Sys<'d>,
    pub flash: Flash<'d, Blocking>,
    pub firmware: CoprocessorFirmware,
    pub requests: Requests,
    pub status: &'static Status,
}

/// An image being written to flash. All offsets are flash offsets.
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
            // FUS is idle and a stack is installed. The board is meant to
            // run its stack, so start it.
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
                                // Publish the status before responding, so
                                // that the caller never reads a status that
                                // is older than the response.
                                self.publish(FusState::Busy);
                                requests.answer(pending, Ok(()));
                                // If FUS accepts the image, it resets the
                                // chip and this call never returns.
                                state = while_busy(requests, self.installed(from)).await;
                                set_pending(None);
                                continue;
                            }
                            Err(e) => Err(e),
                        }
                    }
                    _ => Err(CoprocessorError::OutOfSequence),
                },
                // A stack is still installed here only if FUS failed to
                // start it.
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

    /// Wait for an install to finish. This runs in the boot that started
    /// the install, and again in each later boot until it is over.
    async fn installed(&mut self, from: Installed) -> FusState {
        if Installed::from(self.firmware) != from {
            // The installed versions have changed: the image is installed.
            // FUS goes idle after that.
            return self.idle(Duration::MIN).await;
        }
        // Nothing has changed yet. FUS resets the chip when it installs
        // something, so if this wait completes, nothing was installed.
        self.idle(INSTALL_SETTLE).await
    }

    /// Run the next step of removing the stack. Returns when there is
    /// nothing left to do.
    async fn uninstall(&mut self, step: UninstallStep) -> FusState {
        if self.firmware.stack.is_none() {
            return self.idle(Duration::MIN).await;
        }
        if step == UninstallStep::Check {
            // The stack is still installed after the delete. Do not retry.
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
        // CPU2 only reports what is installed when it boots, so reset.
        cortex_m::peripheral::SCB::sys_reset()
    }

    async fn start_stack(&mut self) {
        self.publish(FusState::Busy);
        dfu::fus_busy(true);
        let status = self.sys.shci_c2_fus_startws().await;
        // FUS responds, and then resets the chip into the stack. If this
        // code is still running ten seconds later, the stack did not start.
        Timer::after_secs(10).await;
        dfu::fus_busy(false);
        defmt::warn!("fus: the wireless stack did not start ({})", status);
    }

    /// Compute the flash location for an image of the given size.
    fn begin(&mut self, ImageSize(len): ImageSize) -> Result<Staging, CoprocessorError> {
        // FUS looks for the image on the page boundary that makes it end
        // just below the secure flash area. ST's release notes give the
        // same address for each image.
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
        // Only the image's last chunk may end partway through a flash word.
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
        // Erase one page at a time, as the image reaches it, so that no
        // request blocks everything else for longer than one erase.
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

    /// Tell FUS to install the image in flash. Returns the versions that
    /// were installed before.
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

/// Run `work`, which needs exclusive use of CPU2. Requests that arrive in
/// the meantime are answered with [`CoprocessorError::Busy`].
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
