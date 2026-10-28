//! The supply monitor. The chip's programmable voltage detector (PVD)
//! compares VDD with a threshold. VDD is the board's 3.0 V rail. On a coin
//! cell the regulator cannot regulate and the rail follows the cell, so the
//! detector shows when the cell is low. On USB or aux power the rail stays
//! at 3.0 V.
//!
//! The detector's output is read when it is asked for, which for the
//! metrics is while the board is idle: before a push, or for a scrape. A
//! coin cell sags while the radio transmits and recovers afterwards, and a
//! fresh cell sagged below the threshold within minutes of every start (seen
//! on boards 4D0021 and 4B002E in October 2026). So the output during a
//! sag says little, and the output at rest says whether the cell is low.

use embassy_stm32::pac::PWR;

/// The detector's threshold, as `PLS` in `PWR_CR2`. At level 4 the detector
/// trips when VDD falls below 2.64 V (2.59 V to 2.69 V). That is just under
/// the 2.7 V that the capacitance sensor needs.
const LEVEL: u8 = 4;

/// Start the detector.
pub fn init() -> Monitor {
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
    /// Whether VDD is below the threshold now.
    pub fn is_low(self) -> bool {
        PWR.sr2().read().pvdo()
    }
}
