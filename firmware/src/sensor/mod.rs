//! Support for the various sensors on the board.
//!
//! Note that not all boards have all sensors.

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

pub mod capacitance;

/// A sensor that several tasks read: the RPC handlers, the reports and the
/// metrics.
pub type Shared<S> = Mutex<ThreadModeRawMutex, S>;
