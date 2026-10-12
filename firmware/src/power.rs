//! What the board's power mode means for the firmware as a whole.
//!
//! A board on battery is never on USB: its cell feeds the 3 V rail directly,
//! and USB power on top of that would destroy the board (see `TODO.org`). So
//! a board on battery does not start USB, and it puts the chip in Stop mode
//! whenever the firmware has nothing to do. Both are decided once, when the
//! board starts. When the power mode changes later, the board restarts.

use cortex_m::peripheral::DCB;
use embassy_stm32::rcc::{StopMode, WakeGuard};
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
/// the board when the mode changes. This uses no peripheral, so that the
/// clocks can be configured for the power mode.
pub fn init(mut config: board_config::Monitor) -> (PowerMode, Task) {
    let mode = power_mode(config.config().as_ref());
    (mode, Task { config, mode })
}

/// Whether the chip enters Stop mode while the firmware waits. Only a board
/// on battery does: on aux power there is nothing to save, and USB needs its
/// clock. A board with a debugger attached does not either, because the
/// debugger cannot read the memory of a chip that stops and starts, and the
/// log goes through that. The debug port stays enabled once a probe has used
/// it, until the board loses power.
pub fn stops(mode: PowerMode) -> bool {
    mode == PowerMode::Battery && !DCB::is_debugger_attached()
}

pub struct Task {
    config: board_config::Monitor,
    /// The power mode that the board started with.
    mode: PowerMode,
}

impl Task {
    pub async fn run(mut self) -> ! {
        // embassy-stm32's executor enters Stop mode when every task waits
        // and nothing holds a guard. This runs before the executor first
        // sleeps.
        let stops = stops(self.mode);
        defmt::info!("power: {}, stops when idle: {}", self.mode, stops);
        let _awake = (!stops).then(|| WakeGuard::new(StopMode::Stop1));

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
