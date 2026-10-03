//! Panic and hard fault handling. The panic handler is panic-probe's: it
//! prints the message over defmt and raises a HardFault. A probe attached
//! with `probe-rs run` stops there and prints a backtrace; with no probe the
//! handler below runs and bounces us to the bootloader.

use panic_probe as _;

use crate::dfu;

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    dfu::reboot_into_bootloader()
}
