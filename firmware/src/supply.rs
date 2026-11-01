//! The supply monitor. The chip's programmable voltage detector (PVD)
//! compares VDD with a threshold. VDD is the board's 3.0 V rail. On a coin
//! cell the regulator cannot regulate and the rail follows the cell, so the
//! detector shows when the cell is low. On USB or aux power the rail stays
//! at 3.0 V.
//!
//! A coin cell sags while the radio transmits, and takes seconds to recover
//! afterwards. So the [`Task`] reads the detector only once the board has
//! had no exchange of its own over the radio for [`QUIET_FOR`], and the
//! supply counts as low only once every reading for [`LOW_FOR`] has been
//! low. Read right after an exchange, a cell that measured 2.91 V on a
//! meter read low on board 4D0021 (2026-10-31), and a fresh cell read low
//! within minutes of every start.

use core::cell::Cell;

use embassy_stm32::pac::PWR;
use embassy_sync::blocking_mutex::{Mutex, raw::ThreadModeRawMutex};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use crate::thread;

/// The detector's threshold, as `PLS` in `PWR_CR2`. At level 4 the detector
/// trips when VDD falls below 2.64 V (2.59 V to 2.69 V). That is just under
/// the 2.7 V that the capacitance sensor needs.
const LEVEL: u8 = 4;

/// How long after the board's last exchange over the radio the detector is
/// read.
const QUIET_FOR: Duration = Duration::from_secs(5);

/// The shortest time between two readings, so that a board that is quiet
/// for long doesn't wake for them more than it has to.
const READ_EVERY: Duration = Duration::from_secs(30);

/// How long every reading has to be low for the supply to count as low.
const LOW_FOR: Duration = Duration::from_secs(600);

/// Since when every reading has been low. `None` after a reading that
/// wasn't, and before the first.
type LowSince = Mutex<ThreadModeRawMutex, Cell<Option<Instant>>>;

pub struct Builder {
    /// Shows when the board uses the radio.
    pub network: thread::Monitor,
}

impl Builder {
    /// Start the detector. Panics if called a second time: the chip has one.
    pub fn init(self) -> (Task, Monitor) {
        static LOW_SINCE: StaticCell<LowSince> = StaticCell::new();
        let low_since: &'static LowSince = LOW_SINCE.init(Mutex::new(Cell::new(None)));
        PWR.cr2().modify(|w| {
            w.set_pls(LEVEL);
            w.set_pvde(true);
        });
        let task = Task {
            network: self.network,
            low_since,
        };
        (task, Monitor { low_since })
    }
}

pub struct Task {
    network: thread::Monitor,
    low_since: &'static LowSince,
}

impl Task {
    pub async fn run(self) -> ! {
        let mut last_read = Instant::from_ticks(0);
        loop {
            let busy_until = self.network.busy_until();
            let at = (busy_until + QUIET_FOR).max(last_read + READ_EVERY);
            Timer::at(at).await;
            // Read only if no exchange has started in the meantime.
            if self.network.busy_until() != busy_until {
                continue;
            }
            last_read = Instant::now();
            let low = PWR.sr2().read().pvdo();
            self.low_since.lock(|since| match (low, since.get()) {
                (false, _) => since.set(None),
                (true, None) => since.set(Some(last_read)),
                (true, Some(_)) => {}
            });
        }
    }
}

/// Reads what the [`Task`] has found.
#[derive(Clone, Copy)]
pub struct Monitor {
    low_since: &'static LowSince,
}

impl Monitor {
    /// Whether every reading of the detector for at least [`LOW_FOR`] has
    /// been below the threshold. A board that never has [`QUIET_FOR`] without
    /// an exchange never counts as low.
    pub fn is_low(self) -> bool {
        let since = self.low_since.lock(|since| since.get());
        since.is_some_and(|since| since.elapsed() >= LOW_FOR)
    }
}
