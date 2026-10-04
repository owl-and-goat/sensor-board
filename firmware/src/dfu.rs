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

/// How many boots in a row may end in a panic, a fault or a watchdog reset
/// before the next one goes to the ROM bootloader instead. A firmware that
/// cannot stay up is then one that can be replaced over USB.
const MAX_FAILED_BOOTS: u32 = 3;

/// A firmware that has run for this long has not failed to boot.
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

/// How many boots in a row have failed, on top of `FAILED_MAGIC`: whatever
/// else is here is what RAM holds after power-up, and counts as none.
#[unsafe(link_section = ".uninit.FAILED_BOOTS")]
static mut FAILED_BOOTS: MaybeUninit<u32> = MaybeUninit::uninit();
const FAILED_MAGIC: u32 = 0xFA11_0000;

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

/// Jumps to the ROM bootloader if the previous boot asked for it, or if the
/// last few boots all failed: ended in a panic, a fault, or an
/// independent-watchdog reset (a hang, for example waiting for a crystal
/// that never started). One failure is only a restart. If it was a firmware
/// update on trial that failed, the bootloader has put the old firmware
/// back by now.
pub fn enter_bootloader_if_requested() {
    let watchdog_reset = RCC
        .csr()
        .read()
        // Independent window watchdog reset flag
        .iwdgrstf()
        && !is_fus_busy();
    RCC.csr().modify(|w|
                     // Remove reset flag
                     w.set_rmvf(true));
    if watchdog_reset {
        set_failed_boots(failed_boots() + 1);
    }

    let flag = flag();
    // SAFETY: `flag` is a linker-reserved, aligned RAM word that nothing else
    // touches; the bootloader jump happens while the chip is in reset state.
    unsafe {
        let requested = ptr::read_volatile(flag) == MAGIC;
        ptr::write_volatile(flag, 0);
        if requested || failed_boots() >= MAX_FAILED_BOOTS {
            set_failed_boots(0);
            jump_to_bootloader();
        }
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

/// Make the two ends of a request for the ROM bootloader: the handle to ask
/// with, and the task that goes there. Panics if called a second time.
pub fn init() -> (Task, Handle) {
    static REQUESTED: StaticCell<Requested> = StaticCell::new();
    let requested: &'static Requested = REQUESTED.init(Signal::new());
    (Task { requested }, Handle { requested })
}

pub struct Handle {
    requested: &'static Requested,
}

impl Handle {
    /// Ask for a reboot into the ROM bootloader, without doing it on the
    /// spot: whoever asks (the USB host, say) may still be owed an answer.
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
        // Time for the answer to whoever asked to reach the USB host.
        Timer::after_millis(100).await;
        reboot_into_bootloader()
    }
}

/// Mark the next boot for the ROM bootloader and reset.
pub fn reboot_into_bootloader() -> ! {
    // SAFETY: see `enter_bootloader_if_requested`.
    unsafe { ptr::write_volatile(flag(), MAGIC) };
    cortex_m::peripheral::SCB::sys_reset()
}

/// Reset after a panic or a fault, and count this boot as one that failed.
pub fn reset_after_failure() -> ! {
    set_failed_boots(failed_boots() + 1);
    cortex_m::peripheral::SCB::sys_reset()
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
