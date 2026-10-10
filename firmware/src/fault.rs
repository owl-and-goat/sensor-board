//! Panic and hard fault handling. The panic handler is panic-probe's: it
//! prints the message over defmt and raises a HardFault. A probe attached
//! with `probe-rs run` stops there and prints a backtrace. Without a probe,
//! the handler below runs and restarts the board. The restart rolls back a
//! firmware update that is on trial, and several failed boots in a row put
//! the board in the ROM bootloader (see `dfu.rs`).

use panic_probe as _;

use crate::dfu;

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    dfu::reset_after_failure()
}
