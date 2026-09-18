//! Bring-up firmware for the sensor board (STM32WB55CG).
//!
//! What it does:
//! - blinks D1 (PA6, active low) as a sign of life;
//! - enumerates on USB as a CDC-ACM console plus a DFU *runtime* interface, so
//!   `dfu-util -d 1209:0001,0483:df11 ...` can detach it into the ROM
//!   bootloader and flash a new image (see `flash.sh`);
//! - holding SW1 (PA7) for two seconds, typing `dfu` on the console, a panic,
//!   a hard fault or a watchdog reset all end up in the ROM bootloader (see
//!   `dfu.rs`).
//!
//! JP1 (BOOT0) shorted while plugging in USB is the hardware fallback and
//! does not depend on any of this.

#![no_std]
#![no_main]

mod config;
mod dfu;
mod fault;
mod ot;
mod otids;
mod usb_device;
mod wpan;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{Config, bind_interrupts, peripherals};

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must run before any clock or peripheral setup.
    dfu::enter_bootloader_if_requested();
    dfu::reset_if_launched_by_bootloader();
    // From here on a hang ends in a watchdog reset, which lands in DFU.
    dfu::start_watchdog();

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Clocks as in ST's Thread examples: Y1 (32 MHz HSE) is the system
        // clock directly, no PLL, CPU2 also at 32 MHz. Y2 (32.768 kHz LSE)
        // clocks the RTC and the RF wakeup timer. USB gets its 48 MHz from
        // HSI48 trimmed against USB SOF by the CRS (the one internal
        // oscillator in use; the CPU2 stack switches it off unless CPU1 holds
        // hardware semaphore 5, see wpan.rs). HSI16 stays on but unused by
        // CPU1: the RF core clocks itself from HSI16 by default (RCC_EXTCFGR
        // RFCSS) and ST's Thread examples keep it on for that reason.
        config.rcc = WPAN_DEFAULT;
        config.rcc.sys = Sysclk::HSE;
        config.rcc.hsi = true;
        config.rcc.core2_ahb_pre = AHBPrescaler::DIV1;
        // PLL stays on (not as sysclk) only to give USB its 48 MHz from PLL Q,
        // out of reach of CPU2's HSI48 handling.
        config.rcc.hsi48 = None;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    // Option-validity error is set after FUS/bootloader activity; ST clears it
    // first thing in every WB application before any flash use.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));

    // Calendar RTC on the LSE: if it advances, Y2 is oscillating.
    let (_rtc, rtc_time) = Rtc::new(p.RTC, RtcConfig::default());

    let mut led = Output::new(p.PA6, Level::High, Speed::Low); // D1, active low
    let button = Input::new(p.PA7, Pull::Up); // SW1 to GND

    // CPU2 bring-up runs on its own so the console works even if CPU2 is silent.
    // Relay K1 (PA8 high = energised): optional keep-alive load for USB power banks.
    let relay = Output::new(p.PA8, Level::Low, Speed::Low);
    spawner.spawn(wpan::wpan_task(spawner, p.IPCC, p.FLASH, relay).unwrap());

    usb_device::start(
        Driver::new(p.USB, Irqs, p.PA12, p.PA11),
        rtc_time,
        &mut led,
        &button,
    )
    .await
}
