use core::mem::MaybeUninit;
use core::ptr;

use embassy_stm32::pac::RCC;

const MAGIC: u32 = 0xB007_10AD;

/// System memory (ROM bootloader) base on STM32WB55, per AN2606.
const SYSTEM_MEMORY: u32 = 0x1FFF_0000;

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

fn flag() -> *mut u32 {
    (&raw mut DFU_FLAG).cast()
}

pub fn is_fus_busy() -> bool {
    unsafe { ptr::read_volatile((&raw const FUS_BUSY).cast::<u32>()) == BUSY_MAGIC }
}

/// Jumps to the ROM bootloader if the previous boot asked for it, or if the
/// previous boot ended in an independent-watchdog reset (a hang, for example
/// waiting for a crystal that never started).
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

    let flag = flag();
    // SAFETY: `flag` is a linker-reserved, aligned RAM word that nothing else
    // touches; the bootloader jump happens while the chip is in reset state.
    unsafe {
        let requested = ptr::read_volatile(flag) == MAGIC;
        ptr::write_volatile(flag, 0);
        if requested || watchdog_reset {
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

/// Mark the next boot for the ROM bootloader and reset.
pub fn reboot_into_bootloader() -> ! {
    // SAFETY: see `enter_bootloader_if_requested`.
    unsafe { ptr::write_volatile(flag(), MAGIC) };
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
