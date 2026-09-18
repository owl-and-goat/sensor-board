//! Panic and hard fault handlers. Both of these, right now, bounce us to the
//! bootloader.

use crate::dfu;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    dfu::reboot_into_bootloader()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    dfu::reboot_into_bootloader()
}
