//! The button and the LED.
//!
//! The LED blinks when the board starts. Holding the button for five seconds
//! erases the board's configuration, and the LED then blinks fast to confirm
//! it. A board without a configuration is taken to be on aux power, so this
//! is how a board on battery is made a board on aux power again without the
//! network. The dataset is kept: the board stays on its network.

use embassy_stm32::{
    Peri, bind_interrupts,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    interrupt,
    mode::Async,
    peripherals::{EXTI7, PA6, PA7},
};
use embassy_time::{Duration, Timer, with_timeout};

use crate::board_config;

bind_interrupts!(struct Irqs {
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
});

/// How long the button has to be held to erase the configuration.
const HOLD: Duration = Duration::from_secs(5);

/// A number of blinks, and how long the LED is on and then off in each.
struct Blinks {
    count: u32,
    half_period: Duration,
}

const STARTED: Blinks = Blinks {
    count: 3,
    half_period: Duration::from_millis(200),
};
const ERASED: Blinks = Blinks {
    count: 5,
    half_period: Duration::from_millis(60),
};

pub struct Builder {
    /// SW1, which connects the pin to ground while it is pressed.
    pub button: Peri<'static, PA7>,
    pub exti: Peri<'static, EXTI7>,
    /// D1, which is lit while the pin is low.
    pub led: Peri<'static, PA6>,
    pub config: board_config::Eraser,
}

impl Builder {
    pub fn init(self) -> Task {
        Task {
            button: ExtiInput::new(self.button, self.exti, Pull::Up, Irqs),
            led: Output::new(self.led, Level::High, Speed::Low),
            config: self.config,
        }
    }
}

pub struct Task {
    button: ExtiInput<'static, Async>,
    led: Output<'static>,
    config: board_config::Eraser,
}

impl Task {
    pub async fn run(mut self) -> ! {
        self.blink(STARTED).await;
        loop {
            self.button.wait_for_low().await;
            // The button is held if it is not released within the time. A
            // contact that bounces when the button goes down counts as a
            // release, and the next pass starts the time again.
            let released = with_timeout(HOLD, self.button.wait_for_high()).await;
            if released.is_err() {
                match self.config.erase().await {
                    Ok(()) => self.blink(ERASED).await,
                    Err(e) => defmt::warn!("button: could not erase the configuration: {}", e),
                }
                self.button.wait_for_high().await;
            }
        }
    }

    async fn blink(&mut self, blinks: Blinks) {
        for _ in 0..blinks.count {
            self.led.set_low();
            Timer::after(blinks.half_period).await;
            self.led.set_high();
            Timer::after(blinks.half_period).await;
        }
    }
}
