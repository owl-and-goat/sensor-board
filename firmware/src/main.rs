//! Bring-up firmware for the sensor board (STM32WB55CG).
//!
//! What it does:
//! - blinks D1 (PA6, active low) as a sign of life;
//! - enumerates on USB as a CDC-ACM console plus a DFU *runtime* interface, so
//!   `dfu-util -d 1209:0001,0483:df11 ...` can detach it into the ROM
//!   bootloader and flash a new image when no SWD probe is attached (see
//!   README.org);
//! - holding SW1 (PA7) for two seconds, typing `dfu` on the console, a panic,
//!   a hard fault or a watchdog reset all end up in the ROM bootloader (see
//!   `dfu.rs`).
//!
//! JP1 (BOOT0) shorted while plugging in USB is the hardware fallback and
//! does not depend on any of this.

#![no_std]
#![no_main]

mod console;
mod dfu;
mod fault;
mod persistent_config;
mod thread;
mod usb_device;
mod wpan;

use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::{
    Config, bind_interrupts,
    gpio::{Input, Level, Output, Pull, Speed},
    peripherals, rcc,
    rtc::{Rtc, RtcConfig},
    usb::{self, Driver},
    wdg::IndependentWatchdog,
};

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must run before any clock or peripheral setup.
    dfu::enter_bootloader_if_requested();
    dfu::reset_if_launched_by_bootloader();

    let mut config = Config::default();
    config.rcc = configure_clocks();
    let p = embassy_stm32::init(config);
    defmt::info!("sensor-board (built {})", env!("BUILD_STAMP"));

    // 32 s because this chip pins the IWDG prescaler at /256 whatever is written, and that is the
    // period /256 expresses with the driver's reload maths (measurements in README.org).
    // TODO(aspen): verify that this is true
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 32_000_000);
    watchdog.unleash();

    // Option-validity error is set after FUS/bootloader activity; ST clears it
    // first thing in every WB application before any flash use.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));

    let (_rtc, rtc_time) = Rtc::new(p.RTC, RtcConfig::default());

    let mut led = Output::new(p.PA6, Level::High, Speed::Low); // D1, active low
    let button = Input::new(p.PA7, Pull::Up); // SW1 to GND

    // CPU2 bring-up runs on its own so the console works even if CPU2 is silent.
    spawner.spawn(wpan::run(spawner, p.IPCC, p.FLASH).unwrap());

    usb_device::run(
        Driver::new(p.USB, Irqs, p.PA12, p.PA11),
        rtc_time,
        &mut led,
        &button,
        watchdog,
    )
    .await
}
