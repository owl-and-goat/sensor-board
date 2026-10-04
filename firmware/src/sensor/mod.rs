//! Support for the various sensors on the board.
//!
//! Note that not all boards have all sensors.

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

pub mod capacitance;

/// A sensor that more than one task reads: the host's requests and the
/// reports both do.
pub type Shared<S> = Mutex<ThreadModeRawMutex, S>;
