//! The first thing to run after a reset. It puts a staged firmware update in
//! the place of the firmware, or puts back the firmware that an update
//! replaced if the update never said it was working, and then starts the
//! firmware. How the firmware stages an update and vouches for itself is in
//! `firmware/src/update.rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::{BootLoader, BootLoaderConfig};
use embassy_stm32::flash::{BANK1_REGION, Flash};
use embassy_sync::blocking_mutex::Mutex;

#[entry]
fn main() -> ! {
    // Not `embassy_stm32::init`: the firmware's first statements want the
    // chip the way a reset leaves it (see `firmware/src/dfu.rs`), and all
    // that is needed here is the flash controller.
    let p = unsafe { embassy_stm32::Peripherals::steal() };

    // The option-validity error is set after FUS or ROM bootloader activity,
    // and would fail the first flash operation.
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

/// Anything that goes wrong here goes wrong again after a reset, or does
/// not: a swap that was cut short is picked up where it stopped.
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
