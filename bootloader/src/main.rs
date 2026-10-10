//! The first code to run after a reset. If a firmware update is staged, it
//! swaps the update with the running firmware. If an update was swapped in
//! but never confirmed itself, it swaps the old firmware back. Then it starts
//! the firmware. `firmware/src/update.rs` describes how the firmware stages
//! an update and confirms it.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::{BootLoader, BootLoaderConfig};
use embassy_stm32::flash::{BANK1_REGION, Flash};
use embassy_sync::blocking_mutex::Mutex;

#[entry]
fn main() -> ! {
    // Not `embassy_stm32::init`: the firmware's first statements need the
    // chip as a reset leaves it (see `firmware/src/dfu.rs`), and this code
    // only needs the flash controller.
    let p = unsafe { embassy_stm32::Peripherals::steal() };

    // FUS and ROM bootloader activity leaves the option-validity error flag
    // set, which would make the first flash operation fail. Clear it.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));

    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let flash = Mutex::new(RefCell::new(layout.bank1_region));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = config.active.offset();
    let bootloader = BootLoader::prepare::<_, _, _, 2048>(config);

    unsafe { bootloader.load(BANK1_REGION.base() + active_offset) }
}

/// Reset on any fault. The bootloader resumes an interrupted swap after the
/// reset, so there is nothing else to recover here.
#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
