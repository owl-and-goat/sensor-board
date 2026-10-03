#![no_std]
#![no_main]

mod dfu;
mod fault;
mod persistent_config;
mod thread;

use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::{
    Config, Peri, bind_interrupts, peripherals, rcc,
    rtc::{Rtc, RtcConfig},
    usb,
    wdg::IndependentWatchdog,
};
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

fn configure_clocks() -> rcc::Config {
    let mut clocks = rcc::WPAN_DEFAULT;

    clocks.sys = rcc::Sysclk::HSE;
    clocks.hsi = true;
    clocks.core2_ahb_pre = rcc::AHBPrescaler::DIV1;

    // PLL stays on (not as sysclk) only to give USB its 48 MHz from PLL Q,
    // out of reach of CPU2's HSI48 handling.
    // TODO: disable this PLL and remove USB entirely so that we can go into
    // proper low-power sleep!
    clocks.hsi48 = None;
    clocks.mux.clk48sel = rcc::mux::Clk48sel::PLL1_Q;

    clocks
}

#[embassy_executor::task]
async fn thread(thread_task: thread::Task<'static>) -> ! {
    thread_task.run().await
}

#[embassy_executor::task]
async fn watchdog(iwdg: Peri<'static, peripherals::IWDG>) -> ! {
    // 32 s because this chip pins the IWDG prescaler at /256 whatever is written, and that is the
    // period /256 expresses with the driver's reload maths (measurements in README.org).
    // TODO(aspen): verify that this is true
    let mut watchdog = IndependentWatchdog::new(iwdg, 32_000_000);
    watchdog.unleash();

    loop {
        Timer::after_millis(20).await;
        watchdog.pet();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must run before any clock or peripheral setup.
    dfu::enter_bootloader_if_requested();
    dfu::reset_if_launched_by_bootloader();

    let mut config = Config::default();
    config.rcc = configure_clocks();
    let p = embassy_stm32::init(config);
    defmt::info!("sensor-board (built {})", env!("BUILD_STAMP"));

    // Option-validity error is set after FUS/bootloader activity; ST clears it
    // first thing in every WB application before any flash use.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));

    let (_rtc, _rtc_time) = Rtc::new(p.RTC, RtcConfig::default());

    // let mut led = Output::new(p.PA6, Level::High, Speed::Low); // D1, active low
    // let button = Input::new(p.PA7, Pull::Up); // SW1 to GND

    let (thread_task, _thread_handle) = thread::Builder { ipcc: p.IPCC }.init().await.unwrap();
    spawner.spawn(thread(thread_task).unwrap());
}
