//! Panic and hard fault handling. A panic is recorded, and then raises a
//! HardFault, as panic-probe does. A probe attached with `probe-rs run`
//! stops there and prints a backtrace. Without a probe, the HardFault
//! handler restarts the board, and the metrics report the panic or the
//! fault after the restart (see `dfu.rs`). Neither the panic message nor
//! where the panic happened is kept: the code that formats panic messages,
//! and the messages themselves, took 27 K of flash, and the locations alone
//! took 21 K.
//!
//! The restart rolls back a firmware update that is on trial, and several
//! failed boots in a row make the board go back to its previous firmware,
//! or enter the ROM bootloader (see `dfu.rs`).

use core::panic::PanicInfo;

use crate::dfu;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    dfu::record_panic();
    cortex_m::asm::udf()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    dfu::reset_after_fault(ef.pc())
}
