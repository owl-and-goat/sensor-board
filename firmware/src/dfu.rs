//! Getting back into the STM32WB55 ROM bootloader (USB DFU) from the app.
//!
//! The app never jumps into the bootloader from running code. Instead it
//! writes a magic word into a RAM word that survives a system reset, resets,
//! and the very first thing `main` does after reset is check for that word
//! and jump into system memory while the chip is still in its reset state:
//! no clocks, peripherals or CPU2 have been touched, so nothing needs to be
//! de-initialised. JP1 (BOOT0) stays the hardware fallback if this ever fails.

use core::mem::MaybeUninit;
use core::ptr;

use embassy_stm32::pac::iwdg::vals::Key;
use embassy_stm32::pac::{IWDG, RCC};

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

/// Call first thing in `main`, before any clock or peripheral setup.
///
/// Jumps to the ROM bootloader if the previous boot asked for it, or if the
/// previous boot ended in an independent-watchdog reset (a hang, for example
/// waiting for a crystal that never started).
pub fn enter_bootloader_if_requested() {
    let watchdog_reset = RCC.csr().read().iwdgrstf() && !is_fus_busy();
    RCC.csr().modify(|w| w.set_rmvf(true));

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

/// Independent watchdog, 4 s, clocked by LSI so it does not depend on any
/// crystal. Start it before clock init; feed it with [`pet_watchdog`].
/// Independent watchdog, 4 s, clocked by LSI so it does not depend on any
/// crystal. Start it before clock init; feed it with [`pet_watchdog`].
///
/// On this STM32WB55 the prescaler register ends up at /256 no matter what is
/// written (the write goes through, PVU clears, and PR reads back 6), so the
/// reload value is derived from the prescaler actually in effect.
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

pub fn start_watchdog() {
    use embassy_stm32::pac::iwdg::vals::Pr;
    const LSI_HZ: u32 = 32_000;
    const PERIOD_MS: u32 = 4_000;

    fn wait_update() {
        let mut spins = 0u32;
        while (IWDG.sr().read().pvu() || IWDG.sr().read().rvu()) && spins < 200_000 {
            spins += 1;
        }
    }

    // Same order as ST's HAL: start first (this also starts the LSI that
    // clocks the register updates), then program PR and RLR one at a time,
    // each followed by a wait for its update flag.
    IWDG.kr().write(|w| w.set_key(Key::START));
    IWDG.kr().write(|w| w.set_key(Key::ENABLE));
    IWDG.pr().write(|w| w.set_pr(Pr::DIVIDE_BY32));
    wait_update();

    let divider = 4u32 << IWDG.pr().read().pr().to_bits();
    let reload = (PERIOD_MS * (LSI_HZ / divider) / 1000 - 1).min(0xFFF) as u16;
    IWDG.kr().write(|w| w.set_key(Key::ENABLE));
    IWDG.rlr().write(|w| w.set_rl(reload));
    wait_update();
    IWDG.kr().write(|w| w.set_key(Key::RESET));
}

pub fn pet_watchdog() {
    IWDG.kr().write(|w| w.set_key(Key::RESET));
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
