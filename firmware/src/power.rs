//! What the board's power mode means for the firmware as a whole.
//!
//! A board on battery is never on USB: its cell feeds the 3 V rail directly,
//! and USB power on top of that would destroy the board (see `TODO.org`). So
//! a board on battery does not start USB. That is decided once, when the
//! board starts. When the power mode changes later, the board restarts.

use embassy_time::{Duration, Timer};
use protocol::{BoardConfig, PowerMode};

use crate::board_config;

/// How long the board keeps running after its power mode has changed. In
/// that time the reply to the request that changed it goes out, and the LED
/// confirms an erased configuration.
const RESTART_DELAY: Duration = Duration::from_secs(2);

/// The power mode of a board with this configuration. A board without one is
/// taken to be on aux power.
fn power_mode(config: Option<&BoardConfig>) -> PowerMode {
    config.map_or(PowerMode::Aux, |config| config.power_mode)
}

/// The power mode that the board starts with, and the [`Task`] that restarts
/// the board when the mode changes.
pub fn init(mut config: board_config::Monitor) -> (PowerMode, Task) {
    let mode = power_mode(config.config().as_ref());
    (mode, Task { config, mode })
}

pub struct Task {
    config: board_config::Monitor,
    /// The power mode that the board started with.
    mode: PowerMode,
}

impl Task {
    pub async fn run(mut self) -> ! {
        loop {
            let config = self.config.changed().await;
            if power_mode(config.as_ref()) != self.mode {
                defmt::info!("power: the power mode has changed, restarting");
                Timer::after(RESTART_DELAY).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}
