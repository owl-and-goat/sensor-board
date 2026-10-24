//! The motion sensor, an EKMB130711. It's a passive infrared sensor whose
//! output goes high while it detects motion.
//!
//! The [`Task`] waits for the output with an interrupt, so a board that stops
//! stays stopped until there is motion. It publishes a [`State`]: whether
//! there has been motion in the last [`HOLD`], and how many times motion has
//! been detected. Motion that goes on, or that comes back within the hold,
//! counts as one detection. So a board on battery pushes at most twice for
//! it: once when it starts and once when it has ended.

use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Peri, bind_interrupts,
    exti::{self, ExtiInput},
    gpio::Pull,
    interrupt,
    mode::Async,
    peripherals::{EXTI2, PA2},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    watch::{self, Watch},
};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
});

/// How long a detection lasts after the output was last high. It's as long
/// as the default Prometheus scrape interval, so that a scrape of the board
/// or of the Pushgateway sees every detection in the gauge. A longer hold
/// also means fewer pushes when there's motion on and off.
pub const HOLD: Duration = Duration::from_secs(60);

/// From the datasheet:
///
/// > Twu：Circuit Stability Time: About 10s. (max.)
/// > During this stage, the output’s status is undefined (ON/OFF) and
/// > detection is therefore not guaranteed.
const WARM_UP: Duration = Duration::from_secs(10);

/// What the motion sensor has detected.
#[derive(Clone, Copy, Default)]
pub struct State {
    /// Whether there has been motion in the last [`HOLD`].
    pub detected: bool,
    /// How many times motion has been detected since the board started.
    pub detections: usize,
}

/// The current state, from the first motion or the first hold without any
/// on.
type Current = Watch<ThreadModeRawMutex, State, 1>;

pub struct Builder {
    /// The sensor's output.
    pub pin: Peri<'static, PA2>,
    pub exti: Peri<'static, EXTI2>,
}

impl Builder {
    pub fn init(self) -> (Task, Monitor) {
        static CURRENT: StaticCell<Current> = StaticCell::new();
        let current: &'static Current = CURRENT.init(Watch::new());
        let task = Task {
            input: ExtiInput::new(self.pin, self.exti, Pull::None, Irqs),
            current: current.sender(),
        };
        let monitor = Monitor {
            current,
            // Cannot fail: the `Watch` allows one receiver.
            changes: current.receiver().unwrap(),
        };
        (task, monitor)
    }
}

pub struct Task {
    input: ExtiInput<'static, Async>,
    current: watch::Sender<'static, ThreadModeRawMutex, State, 1>,
}

impl Task {
    pub async fn run(mut self) -> ! {
        Timer::after(WARM_UP).await;
        let mut state = State::default();

        // Someone may have been moving in front of the sensor since before
        // the board started, and the output is low between their movements.
        // So, as at the end of a detection, "no motion" is published only
        // once the output has stayed low for a whole hold. Otherwise every
        // restart in front of someone would tell Home Assistant that the room
        // had cleared.
        let start = select(self.input.wait_for_high(), Timer::after(HOLD)).await;
        if let Either::Second(()) = start {
            self.current.send(state);
            self.input.wait_for_high().await;
        }

        loop {
            state.detected = true;
            state.detections = state.detections.wrapping_add(1);
            self.current.send(state);

            // The detection ends once the output has stayed low for the
            // whole hold.
            loop {
                self.input.wait_for_low().await;
                let next = select(self.input.wait_for_high(), Timer::after(HOLD));
                if let Either::Second(()) = next.await {
                    break;
                }
            }
            state.detected = false;
            self.current.send(state);
            self.input.wait_for_high().await;
        }
    }
}

/// Watches what the motion sensor detects.
pub struct Monitor {
    current: &'static Current,
    changes: watch::Receiver<'static, ThreadModeRawMutex, State, 1>,
}

impl Monitor {
    /// The current state. `None` until the first motion after the warm-up,
    /// or the first hold without any.
    pub fn state(&self) -> Option<State> {
        self.current.try_get()
    }

    /// Wait for the state to change. Returns the new one.
    pub async fn changed(&mut self) -> State {
        self.changes.changed().await
    }
}
