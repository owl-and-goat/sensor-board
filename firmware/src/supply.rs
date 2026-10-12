//! The supply monitor. The chip's programmable voltage detector (PVD)
//! compares VDD with a threshold, and raises an interrupt when VDD falls
//! below it. VDD is the board's 3.0 V rail. On a coin cell the regulator
//! cannot regulate and the rail follows the cell, so the detector shows when
//! the cell is low. On USB or aux power the rail stays at 3.0 V.
//!
//! That the supply was low is remembered until the next reset. A coin cell
//! sags while the radio transmits and recovers afterwards, so its voltage at
//! the moment it is asked for says little.

use embassy_stm32::interrupt::typelevel::{Binding, Handler, Interrupt as _, PVD_PVM};
use embassy_stm32::pac::{EXTI, PWR, exti::regs::Lines};

/// The detector's threshold, as `PLS` in `PWR_CR2`. At level 4 the detector
/// trips when VDD falls below 2.64 V (2.59 V to 2.69 V). That is just under
/// the 2.7 V that the capacitance sensor needs.
const LEVEL: u8 = 4;

/// The EXTI line that the detector's output is connected to.
const LINE: usize = 16;

/// Handles the interrupt that the detector raises when VDD falls below the
/// threshold.
pub struct InterruptHandler;

impl Handler<PVD_PVM> for InterruptHandler {
    unsafe fn on_interrupt() {
        // Mask the line, and leave it masked: that is the record that the
        // supply was low (see `Monitor::is_low`). Then clear the pending
        // bit, which would otherwise keep the chip out of Stop mode.
        EXTI.cpu(0).imr(0).modify(|w| w.set_line(LINE, false));
        EXTI.pr(0).write_value(Lines(1 << LINE));
    }
}

/// Start the detector.
pub fn init(_irq: impl Binding<PVD_PVM, InterruptHandler>) -> Monitor {
    // A rising edge of the detector's output is VDD falling below the
    // threshold. Interrupts are off because a handler may change the mask
    // register too.
    cortex_m::interrupt::free(|_| {
        EXTI.rtsr(0).modify(|w| w.set_line(LINE, true));
        EXTI.cpu(0).imr(0).modify(|w| w.set_line(LINE, true));
    });
    PVD_PVM::unpend();
    // SAFETY: `_irq` shows that the interrupt is bound to its handler, and
    // nothing here relies on the interrupt being masked.
    unsafe { PVD_PVM::enable() };

    PWR.cr2().modify(|w| {
        w.set_pls(LEVEL);
        w.set_pvde(true);
    });
    Monitor(())
}

/// Reads the detector.
#[derive(Clone, Copy)]
pub struct Monitor(());

impl Monitor {
    /// Whether VDD is below the threshold, or has been since [`init`].
    pub fn is_low(self) -> bool {
        // The interrupt handler leaves the line masked. The detector's
        // output covers a supply that was already low when it started.
        !EXTI.cpu(0).imr(0).read().line(LINE) || PWR.sr2().read().pvdo()
    }
}
