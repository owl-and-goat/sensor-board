use core::mem::MaybeUninit;
use core::ptr;

use embassy_futures::select::{Either, select};
use embassy_stm32::pac::RCC;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

const MAGIC: u32 = 0xB007_10AD;

/// System memory (ROM bootloader) base on STM32WB55, per AN2606.
const SYSTEM_MEMORY: u32 = 0x1FFF_0000;

/// After this many consecutive boots that ended in a failure, the board
/// goes back to the firmware it ran before the last update, if it still has
/// it, and otherwise enters the ROM bootloader. That way a firmware that
/// cannot stay up is replaced by one that could, or can at least be replaced
/// over USB.
const MAX_FAILED_BOOTS: u32 = 3;

/// A boot counts as successful once the firmware has run for this long.
const HEALTHY_AFTER: Duration = Duration::from_secs(60);

/// Lives in `.uninit`, so cortex-m-rt leaves it alone during RAM init and it
/// keeps its value across a software reset (SRAM1 is retained; only a
/// power cycle clears it).
#[unsafe(link_section = ".uninit.DFU_FLAG")]
static mut DFU_FLAG: MaybeUninit<u32> = MaybeUninit::uninit();

/// Set while a CPU2 firmware (FUS/stack) operation is in progress. FUS stalls
/// and resets CPU1 during those; a watchdog reset then must not be treated as
/// a crash. Survives resets, cleared by a power cycle or `fus_busy(false)`.
#[unsafe(link_section = ".uninit.FUS_BUSY")]
static mut FUS_BUSY: MaybeUninit<u32> = MaybeUninit::uninit();
const BUSY_MAGIC: u32 = 0xF05B_0511;

/// The number of consecutive failed boots, in the low half of the word. The
/// high half is `FAILED_MAGIC`. Any other value is uninitialized RAM after
/// power-up, and counts as zero.
#[unsafe(link_section = ".uninit.FAILED_BOOTS")]
static mut FAILED_BOOTS: MaybeUninit<u32> = MaybeUninit::uninit();
const FAILED_MAGIC: u32 = 0xFA11_0000;

/// What the firmware recorded when it ended the last boot because something
/// failed, for [`after_reset`] to read. Cleared at every start.
#[unsafe(link_section = ".uninit.LAST_FAILURE")]
static mut LAST_FAILURE: MaybeUninit<Record> = MaybeUninit::uninit();

/// A failure as it is stored in [`LAST_FAILURE`]. `tag` is `RECORD_MAGIC`
/// plus the kind of failure. Any other value is uninitialized RAM after
/// power-up, or no failure.
#[derive(Clone, Copy)]
#[repr(C)]
struct Record {
    tag: u32,
    detail: u32,
}

const RECORD_MAGIC: u32 = 0x1A57_B000;
const FAULT: u32 = RECORD_MAGIC | 1;
const PANIC: u32 = RECORD_MAGIC | 2;
const CPU2: u32 = RECORD_MAGIC | 3;

/// How the last boot ended.
#[derive(Clone, Copy)]
pub enum End {
    /// The board was powered up, or its supply dropped below the brown-out
    /// level.
    PowerUp,
    /// Something pulled the reset pin, usually a probe.
    ResetPin,
    /// The firmware or FUS restarted the board on purpose: for an update, a
    /// change of the power mode, or the ROM bootloader.
    Restart,
    /// The watchdog reset the board, because the firmware stopped running.
    Watchdog,
    /// A fault at `pc`.
    Fault { pc: u32 },
    /// A panic. Where it happened isn't recorded: that would keep the
    /// location of every place that can panic in the image, which took 21 K
    /// of flash (measured on 2026-11-08).
    Panic,
    /// CPU2 did not answer the OpenThread call `command`.
    Cpu2 { command: u32 },
}

/// How the last boot ended, as the firmware finds out at the start of this
/// one.
#[derive(Clone, Copy)]
pub struct LastBoot {
    pub end: End,
    /// How many boots in a row, up to the last one, ended in a failure.
    pub failed_in_a_row: u32,
}

impl End {
    fn is_failure(self) -> bool {
        match self {
            End::Watchdog => !is_fus_busy(),
            End::Fault { .. } | End::Panic | End::Cpu2 { .. } => true,
            End::PowerUp | End::ResetPin | End::Restart => false,
        }
    }
}

fn record() -> *mut Record {
    (&raw mut LAST_FAILURE).cast()
}

/// Read and clear the record of the last boot's failure, and the chip's
/// reset flags.
fn last_end() -> End {
    // SAFETY: a linker-reserved RAM word that only this module touches.
    let Record { tag, detail } = unsafe { ptr::read_volatile(record()) };
    unsafe { ptr::write_volatile(record(), Record { tag: 0, detail: 0 }) };
    let csr = RCC.csr().read();
    RCC.csr().modify(|w| w.set_rmvf(true));

    match tag {
        FAULT => End::Fault { pc: detail },
        PANIC => End::Panic,
        CPU2 => End::Cpu2 { command: detail },
        _ if csr.iwdgrstf() => End::Watchdog,
        _ if csr.borrstf() => End::PowerUp,
        _ if csr.sftrstf() => End::Restart,
        _ => End::ResetPin,
    }
}

fn flag() -> *mut u32 {
    (&raw mut DFU_FLAG).cast()
}

fn failed_boots() -> u32 {
    let word = unsafe { ptr::read_volatile((&raw const FAILED_BOOTS).cast::<u32>()) };
    match word & 0xFFFF_0000 {
        FAILED_MAGIC => word & 0xFFFF,
        _ => 0,
    }
}

fn set_failed_boots(count: u32) {
    unsafe { ptr::write_volatile((&raw mut FAILED_BOOTS).cast::<u32>(), FAILED_MAGIC | count) };
}

pub fn fus_busy(busy: bool) {
    unsafe {
        ptr::write_volatile(
            (&raw mut FUS_BUSY).cast::<u32>(),
            if busy { BUSY_MAGIC } else { 0 },
        )
    };
}

pub fn is_fus_busy() -> bool {
    unsafe { ptr::read_volatile((&raw const FUS_BUSY).cast::<u32>()) == BUSY_MAGIC }
}

/// Find out how the last boot ended, and act on it. Jumps to the ROM
/// bootloader if the last boot requested it. After [`MAX_FAILED_BOOTS`]
/// failed boots in a row, goes back to the firmware from before the last
/// update if the board still has it, and otherwise jumps to the ROM
/// bootloader. A single failure only restarts the firmware. If the failed
/// boot was a firmware update on trial, the bootloader has already restored
/// the old firmware by now.
///
/// Runs before any clock or peripheral is set up, and before CPU2 starts.
pub fn after_reset() -> LastBoot {
    let end = last_end();
    if end.is_failure() {
        set_failed_boots(failed_boots() + 1);
    }
    let failed_in_a_row = failed_boots();

    let flag = flag();
    // SAFETY: `flag` is a linker-reserved, aligned RAM word that nothing else
    // touches; the bootloader jump happens while the chip is in reset state.
    unsafe {
        let requested = ptr::read_volatile(flag) == MAGIC;
        ptr::write_volatile(flag, 0);
        if requested {
            jump_to_bootloader();
        }
    }
    if failed_in_a_row >= MAX_FAILED_BOOTS {
        set_failed_boots(0);
        // Restarts the board if it can.
        crate::update::go_back_to_previous_firmware();
        // SAFETY: as above.
        unsafe { jump_to_bootloader() };
    }
    LastBoot {
        end,
        failed_in_a_row,
    }
}

/// If the ROM bootloader ran before us (a DFU `:leave` jumps here without a
/// reset), CPU2 is already up with the *bootloader's* mailbox pointers and
/// will never talk to ours. The bootloader leaves its reference table in
/// shared SRAM2a: its first word points at 0x20030024. Wipe that marker and
/// take a real system reset so CPU2 starts over against our tables.
pub fn reset_if_launched_by_bootloader() {
    const SRAM2A: *mut u32 = 0x2003_0000 as *mut u32;
    const BOOTLOADER_DEVICE_INFO_PTR: u32 = 0x2003_0024;
    unsafe {
        if ptr::read_volatile(SRAM2A) == BOOTLOADER_DEVICE_INFO_PTR {
            ptr::write_volatile(SRAM2A, 0);
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

type Requested = Signal<ThreadModeRawMutex, ()>;

/// Create the [`Handle`] that requests a reboot into the ROM bootloader and
/// the [`Task`] that performs it. Panics if called a second time.
pub fn init() -> (Task, Handle) {
    static REQUESTED: StaticCell<Requested> = StaticCell::new();
    let requested: &'static Requested = REQUESTED.init(Signal::new());
    (Task { requested }, Handle { requested })
}

pub struct Handle {
    requested: &'static Requested,
}

impl Handle {
    /// Request a reboot into the ROM bootloader. The reboot is delayed
    /// briefly, so that the caller (the USB host, for example) still gets
    /// its response.
    pub fn request_bootloader(&self) {
        self.requested.signal(());
    }
}

pub struct Task {
    requested: &'static Requested,
}

impl Task {
    pub async fn run(self) -> ! {
        if let Either::Second(()) = select(self.requested.wait(), Timer::after(HEALTHY_AFTER)).await
        {
            set_failed_boots(0);
            self.requested.wait().await;
        }
        // Give the response to the request time to reach the USB host.
        Timer::after_millis(100).await;
        reboot_into_bootloader()
    }
}

/// Mark the next boot for the ROM bootloader and reset.
pub fn reboot_into_bootloader() -> ! {
    // SAFETY: see `after_reset`.
    unsafe { ptr::write_volatile(flag(), MAGIC) };
    cortex_m::peripheral::SCB::sys_reset()
}

/// Record a panic. The panic handler then raises a fault, and
/// [`reset_after_fault`] keeps this record.
pub fn record_panic() {
    write_record(PANIC, 0);
}

/// Restart after a fault at `pc`, which counts this boot as failed. A panic
/// that was recorded first stays the reason.
pub fn reset_after_fault(pc: u32) -> ! {
    // SAFETY: see `last_end`.
    if unsafe { ptr::read_volatile(record()) }.tag != PANIC {
        write_record(FAULT, pc);
    }
    cortex_m::peripheral::SCB::sys_reset()
}

/// Restart because CPU2 didn't answer the OpenThread call `command`, which
/// counts this boot as failed. Only a system reset restarts CPU2.
pub fn reset_after_cpu2_timeout(command: u32) -> ! {
    write_record(CPU2, command);
    cortex_m::peripheral::SCB::sys_reset()
}

fn write_record(tag: u32, detail: u32) {
    // SAFETY: see `last_end`.
    unsafe { ptr::write_volatile(record(), Record { tag, detail }) };
}

unsafe fn jump_to_bootloader() -> ! {
    // SAFETY: caller guarantees reset state. Point VTOR at the bootloader's
    // vector table (harmless if the bootloader sets it itself) and hand over
    // MSP and the reset vector, exactly like a boot from system memory.
    unsafe {
        (*cortex_m::peripheral::SCB::PTR).vtor.write(SYSTEM_MEMORY);
        cortex_m::asm::bootload(SYSTEM_MEMORY as *const u32)
    }
}
