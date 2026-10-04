//! Panic and hard fault handling. The panic handler is panic-probe's: it
//! prints the message over defmt and raises a HardFault. A probe attached
//! with `probe-rs run` stops there and prints a backtrace; with no probe the
//! handler below runs and restarts the board. A firmware update on trial is
//! undone by that restart, and a board that fails several times in a row
//! ends up in the ROM bootloader (see `dfu.rs`).

use panic_probe as _;

use crate::dfu;

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    dfu::reset_after_failure()
}
