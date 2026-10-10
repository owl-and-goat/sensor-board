//! The radio coprocessor, CPU2. It runs ST's firmware: either a wireless
//! stack, or FUS, the firmware upgrade service that installs one.
//!
//! [`Task`] boots CPU2, keeps its mailbox serviced, and then runs the side of
//! this firmware that goes with what it finds there: [`crate::thread`] on a
//! Thread stack, the installer in [`fus`] on FUS. Getting from one to the
//! other always takes a reset, so that choice holds for as long as the task
//! lives, and the side that is not running only turns requests down.

use core::cell::Cell;

use defmt::debug;
use embassy_futures::{
    join::{join, join4},
    select::{Either, select},
};
use embassy_stm32::{
    Peri, bind_interrupts,
    flash::Flash,
    ipcc::{self, ReceiveInterruptHandler, TransmitInterruptHandler},
    peripherals::{FLASH, IPCC},
};
use embassy_stm32_wpan::{
    TlMbox,
    shci::SchiSysEventReady,
    sub::{sys::Sys, traces::Traces},
};
use embassy_sync::blocking_mutex::{Mutex, raw::ThreadModeRawMutex};
use embassy_time::{Duration, Timer};
use protocol::{
    CoprocessorError, CoprocessorFirmware, CoprocessorResult, CoprocessorStatus, ImageChunk,
    ImageSize, NetworkError, StackKind, Version, WirelessStack,
};
use static_cell::StaticCell;

use crate::{
    board_config,
    radio_flash::{self, RadioFlash},
    request, thread, update,
};

mod fus;

use fus::{Pending, UninstallStep};

bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
});

/// What CPU2 is running, as last published.
struct Status(Mutex<ThreadModeRawMutex, Cell<CoprocessorStatus>>);

impl Status {
    fn get(&self) -> CoprocessorStatus {
        self.0.lock(|s| s.get())
    }

    fn publish(&self, status: CoprocessorStatus) {
        self.0.lock(|s| {
            if s.replace(status) != status {
                defmt::info!("coprocessor: {}", status);
            }
        });
    }
}

/// Shared state between the task and the handle
struct Shared {
    requests: request::Channel<fus::Request, CoprocessorResult>,
    status: Status,
}

/// How long an install request gets: at most a flash page erased and a chunk
/// written, or one command to FUS.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

pub struct Builder<'d> {
    pub ipcc: Peri<'d, IPCC>,
    pub flash: Peri<'d, FLASH>,
    /// The firmware-update service, which this task runs: it writes flash in
    /// step with CPU2.
    pub update: update::Service,
    /// The board-configuration service. This task runs it for the same
    /// reason.
    pub board_config: board_config::Service,
}

impl<'d> Builder<'d> {
    /// Panics if called a second time: there is one coprocessor.
    /// Returns the reports socket and the TCP connection for other tasks to
    /// use. The task keeps the sockets for firmware updates and board
    /// configuration, because it runs those services.
    pub fn init(
        self,
    ) -> (
        Task<'d>,
        Handle,
        thread::Handle,
        thread::Socket,
        thread::Tcp,
    ) {
        static SHARED: StaticCell<Shared> = StaticCell::new();
        let shared: &'static Shared = SHARED.init(Shared {
            requests: request::Channel::new(),
            status: Status(Mutex::new(Cell::new(CoprocessorStatus::Starting))),
        });

        let (client, server) = shared.requests.split();
        let (thread_handle, sockets, tcp, thread) = thread::init();
        let task = Task {
            ipcc: self.ipcc,
            flash: self.flash,
            requests: server,
            status: &shared.status,
            thread,
            update: self.update,
            update_socket: sockets.updates,
            board_config: self.board_config,
            config_socket: sockets.configs,
        };
        let handle = Handle {
            requests: client,
            status: &shared.status,
        };
        (task, handle, thread_handle, sockets.reports, tcp)
    }
}

/// How the rest of the firmware reaches the coprocessor's own firmware. Its
/// Thread networking has a handle of its own, [`thread::Handle`].
pub struct Handle {
    requests: request::Client<fus::Request, CoprocessorResult>,
    status: &'static Status,
}

impl Handle {
    pub fn status(&self) -> CoprocessorStatus {
        self.status.get()
    }

    /// Get ready to take an image of `size` for FUS to install.
    pub async fn begin_install(&mut self, size: ImageSize) -> CoprocessorResult {
        self.request(fus::Request::Begin(size)).await
    }

    /// Take the next piece of the image.
    pub async fn write_install(&mut self, chunk: ImageChunk) -> CoprocessorResult {
        self.request(fus::Request::Write(chunk)).await
    }

    /// Hand the complete image to FUS. [`Handle::status`] tells how the
    /// install goes from there.
    pub async fn finish_install(&mut self) -> CoprocessorResult {
        self.request(fus::Request::Finish).await
    }

    /// Start removing the wireless stack, which leaves the coprocessor with
    /// FUS alone. [`Handle::status`] tells how that goes.
    pub async fn uninstall_stack(&mut self) -> CoprocessorResult {
        self.request(fus::Request::Uninstall).await
    }

    async fn request(&mut self, request: fus::Request) -> CoprocessorResult {
        self.requests
            .ask(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(CoprocessorError::Unresponsive))
    }
}

pub struct Task<'d> {
    ipcc: Peri<'d, IPCC>,
    flash: Peri<'d, FLASH>,
    requests: request::Server<fus::Request, CoprocessorResult>,
    status: &'static Status,
    thread: thread::Service,
    update: update::Service,
    update_socket: thread::Socket,
    board_config: board_config::Service,
    config_socket: thread::Socket,
}

impl<'d> Task<'d> {
    pub async fn run(self) -> ! {
        let mbox = ensure_booted(self.ipcc).await;
        let mut mm = mbox.mm_subsystem;
        let mut sys = mbox.sys_subsystem;
        let flash = Flash::new_blocking(self.flash);
        let (ot, cli_rx, notif_rx) = mbox.thread_subsystem.split();

        let sides = async {
            match Running::read(&sys) {
                Running::Stack(firmware) => {
                    if fus::pending() == Some(Pending::Uninstall(UninstallStep::EnterFus)) {
                        // The stack is to go, and that starts with putting
                        // FUS in its place: this resets the chip.
                        fus::set_pending(Some(Pending::Uninstall(UninstallStep::Delete)));
                        fus::enter(&mut sys).await;
                    }
                    // Whatever else was pending, it is over: an install has
                    // put a stack here, or a stack has outlived its removal.
                    fus::set_pending(None);
                    self.status.publish(CoprocessorStatus::Stack(firmware));

                    let installer = fus::serve_beside_stack(self.requests, self.status);
                    // From here on flash is only written in step with the
                    // stack: by Thread for its dataset, by the updates, and
                    // for the board's configuration.
                    let flash = radio_flash::Shared::new(RadioFlash::new(flash, sys));
                    let network = self.thread.monitor();
                    let update = self.update.run(&flash, network, self.update_socket);
                    let configs = self.board_config.run(&flash, self.config_socket);
                    if firmware.stack.map(|s| s.kind) == Some(StackKind::ThreadFtd) {
                        let thread = thread::Thread {
                            ot,
                            cli_rx,
                            notif_rx,
                            flash: &flash,
                        };
                        join4(self.thread.run(thread), installer, update, configs)
                            .await
                            .0
                    } else {
                        let thread = self.thread.unavailable(NetworkError::NoThreadStack);
                        join4(thread, installer, update, configs).await.0
                    }
                }
                Running::Fus(firmware) => {
                    let installer = fus::Installer {
                        sys,
                        flash,
                        firmware,
                        requests: self.requests,
                        status: self.status,
                    };
                    join4(
                        self.thread.unavailable(NetworkError::NoThreadStack),
                        installer.run(),
                        self.update.unavailable(),
                        self.board_config.unavailable(),
                    )
                    .await
                    .0
                }
            }
        };

        // Whatever CPU2 runs, it stalls unless its event buffers are returned
        // and its traces read.
        join(
            join(mm.run_queue(), drain_traces(mbox.traces_subsystem)),
            sides,
        )
        .await
        .1
    }
}

async fn drain_traces(mut traces: Traces<'_>) -> ! {
    loop {
        let event = traces.read().await;
        defmt::trace!("cpu2: {=[u8]:a}", event.payload());
    }
}

/// CPU2's ready event, absent when it never sent one.
type Ready = Option<Result<SchiSysEventReady, ()>>;

/// Hardware semaphore 5 guards the 48 MHz clock (CLK48/HSI48). When a wireless stack starts,
/// CPU2 takes it and switches that clock off unless CPU1 already holds it. USB needs the clock,
/// so take it before C2BOOT and never release it (AN5289; same as ST's USB examples on WB).
const HSEM_CLK48: usize = 5;

fn lock_clk48_semaphore() -> bool {
    use embassy_stm32::pac::{HSEM, RCC};
    RCC.ahb3enr().modify(|w| w.set_hsemen(true));
    // One-step lock: reading RLR locks the semaphore for this core if free.
    let r = HSEM.rlr(HSEM_CLK48).read();
    r.lock()
}

/// "Fake a C2BOOT when it has already been set" (ST's words): SHCI_C2_REINIT
/// followed by a SEV instruction makes CPU2 restart its firmware, re-read the
/// reference table and send its ready event again.
async fn reinit(sys: &mut Sys<'_>) -> Ready {
    let status = sys.shci_c2_reinit().await;
    debug!("cpu2: no ready event; REINIT -> {}", status);
    cortex_m::asm::sev();
    match select(sys.read_ready(), Timer::after_secs(3)).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    }
}

async fn ensure_booted<'d>(ipcc: Peri<'d, IPCC>) -> TlMbox<'d> {
    lock_clk48_semaphore();
    let mut mbox = TlMbox::init_without_ready(ipcc, Irqs, ipcc::Config::default());

    // CPU2 sends its ready event once after it boots. A CPU1-only reset (our
    // DFU round trips) leaves CPU2 running, so don't wait forever.
    let ready = match select(mbox.sys_subsystem.read_ready(), Timer::after_secs(2)).await {
        Either::First(r) => Some(r),
        // No ready event means CPU2 was already up: the ROM bootloader boots it
        // for its own FUS commands, and a CPU1-only reset does not restart it.
        Either::Second(()) => reinit(&mut mbox.sys_subsystem).await,
    };
    debug!("cpu2: ready event {}", ready);

    mbox
}

/// What CPU2 says about itself, in the device info table it fills in when it
/// boots.
enum Running {
    Fus(CoprocessorFirmware),
    Stack(CoprocessorFirmware),
}

/// FUS writes FUS_DEVICE_INFO_TABLE_VALIDITY_KEYWORD as the first word when
/// it is the one running; the table then has the MB_FUS_DeviceInfoTable_t
/// layout instead of the wireless-firmware one.
const FUS_TABLE_KEYWORD: u32 = 0xA946_56B9;

/// `INFO_STACK_TYPE_THREAD_FTD`
const STACK_TYPE_THREAD_FTD: u8 = 0x10;

impl Running {
    fn read(sys: &Sys<'_>) -> Running {
        let raw = sys.device_info_raw();
        if raw[0] == FUS_TABLE_KEYWORD {
            // MB_FUS_DeviceInfoTable_t: word 1 ends in the installed stack's
            // type, word 3 is the FUS version and word 5 the stack's.
            Running::Fus(CoprocessorFirmware {
                fus: version(raw[3]),
                stack: stack(raw[5], (raw[1] >> 24) as u8),
            })
        } else {
            let info = sys.device_info();
            let (fus, wireless) = (info.rss_info_table, info.wireless_fw_info_table);
            Running::Stack(CoprocessorFirmware {
                fus: version(fus.version),
                // ST's name for this field is InfoStack.
                stack: stack(wireless.version, wireless.thread_info as u8),
            })
        }
    }
}

/// Major, minor and sub-version are the top three bytes; the last holds
/// branch and build.
fn version(word: u32) -> Version {
    let [major, minor, patch, _] = word.to_be_bytes();
    Version {
        major,
        minor,
        patch,
    }
}

/// A version of zero means that no stack is installed.
fn stack(version_word: u32, stack_type: u8) -> Option<WirelessStack> {
    (version_word != 0).then(|| WirelessStack {
        kind: match stack_type {
            STACK_TYPE_THREAD_FTD => StackKind::ThreadFtd,
            other => StackKind::Other(other),
        },
        version: version(version_word),
    })
}
