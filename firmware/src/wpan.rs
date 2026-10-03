//! CPU2 (the wireless coprocessor) bring-up and the IPCC mailbox.
//!
//! Hardware and mechanism only: no console text lives here. The command
//! surface and everything that formats output is in `console.rs`, which this
//! task lends the mailbox subsystem handles for its lifetime.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::Timer;

use embassy_stm32::flash::Flash;
use embassy_stm32::ipcc::{self, ReceiveInterruptHandler, TransmitInterruptHandler};
use embassy_stm32::peripherals::{FLASH, IPCC};
use embassy_stm32::{Peri, bind_interrupts};
use embassy_stm32_wpan::TlMbox;
use embassy_stm32_wpan::shci::{SchiCommandStatus, SchiSysEventReady};
use embassy_stm32_wpan::sub::mm::MemoryManager;
use embassy_stm32_wpan::sub::sys::Sys;

use crate::console;

bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
});

/// Range-test mode: the board pings the leader every few seconds and the LED
/// shows the link (see usb_device.rs): fast blink = detached, off = attached
/// but no reply, on = replies coming in (short off-pulse per reply).
pub static RANGE_MODE: AtomicBool = AtomicBool::new(false);
/// 0 detached, 1 attached without reply, 2 attached and getting replies.
pub static LINK: AtomicU8 = AtomicU8::new(0);
/// One-shot LED pulse request (a ping reply arrived).
pub static PULSE: AtomicBool = AtomicBool::new(false);

/// Where this task is: 0 not started, 1 booting CPU2 (waiting for its ready
/// event), 2 running.
pub static STATE: AtomicU8 = AtomicU8::new(0);
/// Ready event value: 0 wireless fw, 1 FUS, 0xFE unknown value, 0xFF none.
pub static READY: AtomicU8 = AtomicU8::new(0xFF);

/// FUS writes FUS_DEVICE_INFO_TABLE_VALIDITY_KEYWORD as the first word when
/// it is the one running; the table then has the MB_FUS_DeviceInfoTable_t
/// layout instead of the wireless-firmware one.
pub const FUS_TABLE_KEYWORD: u32 = 0xA946_56B9;

/// Hardware semaphore 5 guards the 48 MHz clock (CLK48/HSI48). When a
/// wireless stack starts, CPU2 takes it and switches that clock off unless
/// CPU1 already holds it. USB needs the clock, so take it before C2BOOT and
/// never release it (AN5289; same as ST's USB examples on WB).
const HSEM_CLK48: usize = 5;

pub fn lock_clk48_semaphore() -> bool {
    use embassy_stm32::pac::{HSEM, RCC};
    RCC.ahb3enr().modify(|w| w.set_hsemen(true));
    // One-step lock: reading RLR locks the semaphore for this core if free.
    let r = HSEM.rlr(HSEM_CLK48).read();
    r.lock()
}

pub fn clk48_semaphore_held() -> bool {
    let r = embassy_stm32::pac::HSEM.r(HSEM_CLK48).read();
    r.lock()
}

/// Release semaphore 5 (CPU1 core id is 4 in the HSEM registers).
pub fn release_clk48_semaphore() {
    embassy_stm32::pac::HSEM.r(HSEM_CLK48).write(|w| {
        w.set_lock(false);
        w.set_coreid(4);
        w.set_procid(0);
    });
}

/// Shared SRAM2a as found at boot, before the mailbox init clears it. When
/// the ROM bootloader ran before us it booted CPU2, and FUS wrote its device
/// info table (MB_FUS_DeviceInfoTable_t) for the bootloader in here.
static mut SRAM2A_AT_BOOT: [u32; 64] = [0; 64];

fn snapshot_sram2a() {
    unsafe {
        let snap = core::ptr::read_volatile(0x2003_0000 as *const [u32; 64]);
        core::ptr::write_volatile(&raw mut SRAM2A_AT_BOOT, snap);
    }
}

pub fn sram2a_snapshot() -> [u32; 64] {
    unsafe { core::ptr::read_volatile(&raw const SRAM2A_AT_BOOT) }
}

/// CPU2's ready event, absent when it never sent one.
pub type Ready = Option<Result<SchiSysEventReady, ()>>;

fn ready_code(ready: Ready) -> u8 {
    match ready {
        None => 0xFF,
        Some(Err(())) => 0xFE,
        Some(Ok(r)) => r as u8,
    }
}

/// What bring-up did, for `console::run` to report.
pub struct BringUp {
    pub clk48_semaphore_locked: bool,
    /// `Some` when no ready event arrived and a REINIT was issued.
    pub reinit: Option<Result<SchiCommandStatus, ()>>,
    pub ready: Ready,
}

/// "Fake a C2BOOT when it has already been set" (ST's words): SHCI_C2_REINIT
/// followed by a SEV instruction makes CPU2 restart its firmware, re-read the
/// reference table and send its ready event again.
pub async fn reinit(sys: &mut Sys<'_>) -> (Result<SchiCommandStatus, ()>, Ready) {
    let status = sys.shci_c2_reinit().await;
    cortex_m::asm::sev();
    let ready = match select(sys.read_ready(), Timer::after_secs(3)).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    };
    READY.store(ready_code(ready), Ordering::Relaxed);
    (status, ready)
}

/// True when CPU2 reports a running Thread stack (not FUS, not another stack).
pub fn thread_stack_running(sys: &Sys<'_>) -> bool {
    sys.device_info_raw()[0] != FUS_TABLE_KEYWORD
        && sys
            .wireless_fw_info()
            .map(|i| i.thread_info & 0xff == 0x10)
            .unwrap_or(false)
}

#[embassy_executor::task]
async fn run_mm_queue(mut mm: MemoryManager<'static>) {
    mm.run_queue().await;
}

#[embassy_executor::task]
pub async fn run(spawner: Spawner, ipcc: Peri<'static, IPCC>, flash: Peri<'static, FLASH>) {
    snapshot_sram2a();
    let clk48_semaphore_locked = lock_clk48_semaphore();
    STATE.store(1, Ordering::Relaxed);

    let mbox = TlMbox::init_without_ready(ipcc, Irqs, ipcc::Config::default());
    let mut sys = mbox.sys_subsystem;

    // CPU2 sends its ready event once after it boots. A CPU1-only reset (our
    // DFU round trips) leaves CPU2 running, so don't wait forever.
    let mut ready = match select(sys.read_ready(), Timer::after_secs(2)).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    };
    // No ready event means CPU2 was already up: the ROM bootloader boots it
    // for its own FUS commands, and a CPU1-only reset does not restart it.
    let mut reinit_status = None;
    if ready.is_none() {
        let (status, r) = reinit(&mut sys).await;
        reinit_status = Some(status);
        ready = r;
    }
    STATE.store(2, Ordering::Relaxed);
    READY.store(ready_code(ready), Ordering::Relaxed);

    spawner.spawn(run_mm_queue(mbox.mm_subsystem).unwrap());

    let (ot, mut cli_rx, mut notif_rx) = mbox.thread_subsystem.split();
    let mut thread = crate::thread::Thread::new(ot);
    let mut traces = mbox.traces_subsystem;
    let mut flash = Flash::new_blocking(flash);
    let bringup = BringUp {
        clk48_semaphore_locked,
        reinit: reinit_status,
        ready,
    };

    console::run(
        bringup,
        &mut sys,
        &mut thread,
        &mut cli_rx,
        &mut notif_rx,
        &mut traces,
        &mut flash,
    )
    .await;
}
