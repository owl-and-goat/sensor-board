#![no_std]
#![no_main]

mod board_config;
mod button;
mod coprocessor;
mod dfu;
mod discovery;
mod fault;
mod http;
mod i2c_device;
mod metrics;
mod persistent_config;
mod power;
mod radio_flash;
mod report;
mod request;
mod rpc;
mod sensor;
mod supply;
mod thread;
mod update;
mod usb;

use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::{
    Config, Peri, bind_interrupts, dma, i2c,
    mode::Async,
    peripherals::{self, DMA1_CH1, DMA1_CH2, DMA2_CH1, I2C1},
    rcc,
    wdg::IndependentWatchdog,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use protocol::PowerMode;
use static_cell::StaticCell;

use crate::sensor::{
    capacitance::CapacitanceSensor,
    color::ColorSensor,
    mic::{
        Mic,
        pdm::{self, Pdm},
    },
    temp_rh::TempRhSensor,
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
    DMA2_CHANNEL1 => dma::InterruptHandler<DMA2_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<DMA1_CH2>;
    PVD_PVM => supply::InterruptHandler;
});

/// The clocks of a board that starts with the power mode `power`.
fn configure_clocks(power: PowerMode) -> rcc::Config {
    let mut clocks = rcc::WPAN_DEFAULT;

    clocks.sys = rcc::Sysclk::HSE;
    clocks.hsi = true;
    clocks.core2_ahb_pre = rcc::AHBPrescaler::DIV1;

    // The 48 MHz clock. On aux power it is for USB, and comes from PLL Q:
    // USB cannot use HSI48, because CPU2 can switch that off. A board on
    // battery has no USB. Its 48 MHz clock is HSI48, which CPU2 turns on
    // while it needs random numbers (see `coprocessor`).
    let (usb_clock, clk48sel) = match power {
        PowerMode::Aux => (Some(rcc::PllQDiv::DIV4), rcc::mux::Clk48sel::PLL1_Q),
        PowerMode::Battery => (None, rcc::mux::Clk48sel::HSI48),
    };
    clocks.hsi48 = None;
    clocks.mux.clk48sel = clk48sel;

    // The microphone (SAI1) wants a PDM clock of 2.048 MHz, which decimated by
    // 128 is 16 kHz audio. From 32 MHz that takes a /125 somewhere, and the
    // only place for it is the PLL input divider (shared by both PLLs) times
    // PLLSAI1's P: 32 / 5 = 6.4 MHz in, then 6.4 * 32 / 25 = 8.192 MHz for
    // SAI1, and 6.4 * 30 / 4 = 48 MHz for USB. The main PLL is not the
    // system clock.
    clocks.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV5,
        mul: rcc::PllMul::MUL30,
        divp: None,
        divq: usb_clock,
        divr: None,
    });
    clocks.pllsai1 = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV5,
        mul: rcc::PllMul::MUL32,
        divp: Some(rcc::PllPDiv::DIV25),
        divq: None,
        divr: None,
    });
    clocks.mux.sai1sel = rcc::mux::Sai1sel::PLLSAI1_P;

    clocks
}

/// Turn off the clocks that `embassy_stm32::init` leaves running and that
/// nothing uses. PLLSAI1 is the microphone's to turn on and off.
fn stop_unused_clocks(power: PowerMode) {
    let rcc = embassy_stm32::pac::RCC;
    // MSI is the clock that the chip starts on.
    rcc.cr().modify(|w| w.set_msion(false));
    // Without USB the main PLL has no output. Its register still holds the
    // input divider that PLLSAI1 uses.
    if power == PowerMode::Battery {
        rcc.cr().modify(|w| w.set_pllon(false));
    }
}

/// Start the chip's step-down converter (SMPS). It then feeds the core and
/// the radio from the 3 V rail through L1 and L2, at about half the current
/// that the linear regulator takes for them. The chip takes the converter
/// out of the circuit by itself in Stop mode.
fn start_smps() {
    use embassy_stm32::pac::{PWR, RCC, rcc::vals::Smps};

    /// The output voltage is set in levels of 50 mV, from 1.20 V. At 1.40 V,
    /// which ST's applications use, the radio sends with at most 3.7 dBm.
    const LEVEL_1V40: i32 = 4;
    const LEVEL_1V50: i32 = 6;
    /// Where ST stores the level that gave 1.50 V on this chip in
    /// production, or 0 (`SMPS_VOLTAGE_CAL_ADDR` in ST's drivers).
    const CALIBRATION: *const u32 = 0x1FFF_7558 as *const u32;
    // SAFETY: a word of the chip's read-only engineering bytes.
    let calibrated = (unsafe { CALIBRATION.read_volatile() } >> 8 & 0xF) as i32;
    let level = match calibrated {
        0 => LEVEL_1V40,
        calibrated => (LEVEL_1V40 + calibrated - LEVEL_1V50).clamp(0, 0xF),
    };

    // The converter switches at 4 MHz, which is what the 10 µH of L2 is for.
    // Its clock is HSI16 / 4: HSI16 is the system clock around a stop, so
    // the converter's clock never has to change.
    RCC.smpscr().modify(|w| {
        w.set_smpssel(Smps::HSI);
        w.set_smpsdiv(1);
    });
    PWR.cr5().modify(|w| {
        // The lowest limit for the current at its start, 80 mA.
        w.set_sdsc(0);
        w.set_sdvos(level as u8);
    });
    PWR.cr5().modify(|w| w.set_sdeb(true));
}

#[embassy_executor::task]
async fn coprocessor(task: coprocessor::Task<'static>) -> ! {
    task.run().await
}

#[embassy_executor::task]
async fn usb(mut device: usb::Device) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn rpc(mut server: rpc::Server) -> ! {
    loop {
        // `run` only returns with an error when the USB connection drops
        // (cable pulled, host reset the bus). Call it again so the server is
        // listening when the host reconnects.
        let _ = server.run().await;
    }
}

#[embassy_executor::task]
async fn bootloader(task: dfu::Task) -> ! {
    task.run().await
}

#[embassy_executor::task]
async fn reports(task: report::Task, publisher: Option<rpc::Publisher>) -> ! {
    task.run(publisher).await
}

#[embassy_executor::task]
async fn discovery(task: discovery::Task, publisher: Option<rpc::Publisher>) -> ! {
    task.run(publisher).await
}

#[embassy_executor::task]
async fn power(task: power::Task) -> ! {
    task.run().await
}

#[embassy_executor::task]
async fn metrics(task: metrics::Task) -> ! {
    task.run().await
}

#[embassy_executor::task]
async fn watchdog(iwdg: Peri<'static, peripherals::IWDG>) -> ! {
    // 32 s because this chip pins the IWDG prescaler at /256 whatever is written, and that is the
    // period /256 expresses with the driver's reload maths (measurements in AGENTS.md).
    // TODO(aspen): verify that this is true
    let mut watchdog = IndependentWatchdog::new(iwdg, 32_000_000);
    watchdog.unleash();

    // Often enough for the watchdog, and rarely enough for the chip to
    // sleep in between.
    loop {
        Timer::after_secs(8).await;
        watchdog.pet();
    }
}

#[embassy_executor::task]
async fn button(task: button::Task) -> ! {
    task.run().await
}

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    // Must run before any clock or peripheral setup.
    dfu::enter_bootloader_if_requested();
    dfu::reset_if_launched_by_bootloader();

    // The configuration is read first, because the clocks depend on the
    // power mode.
    let (
        config_handle,
        config_eraser,
        config_service,
        [metrics_config, report_config, power_config],
    ) = board_config::init();
    let (power_mode, power_task) = power::init(power_config);

    let mut config = Config::default();
    config.rcc = configure_clocks(power_mode);
    // Keeping the debug port clocked while the chip sleeps or stops costs
    // most of what stopping saves.
    config.enable_debug_during_sleep = !power::stops(power_mode);
    let p = embassy_stm32::init(config);
    stop_unused_clocks(power_mode);
    // Only on battery so far: it has not been tried on any other board.
    if power_mode == PowerMode::Battery {
        start_smps();
    }
    defmt::info!(
        "sensor-board (built {} from {})",
        env!("BUILD_STAMP"),
        env!("BUILD_COMMIT")
    );

    // Option-validity error is set after FUS/bootloader activity; ST clears it
    // first thing in every WB application before any flash use.
    embassy_stm32::pac::FLASH
        .sr()
        .write(|w| w.set_optverr(true));

    spawner.spawn(watchdog(p.IWDG).unwrap());

    let (bootloader_task, bootloader_handle) = dfu::init();
    spawner.spawn(bootloader(bootloader_task).unwrap());

    let (update_handle, update_service) = update::init();
    spawner.spawn(power(power_task).unwrap());
    let (coprocessor_task, coprocessor_handle, thread_handle, sockets, tcp) =
        coprocessor::Builder {
            ipcc: p.IPCC,
            flash: p.FLASH,
            update: update_service,
            board_config: config_service,
            power: power_mode,
        }
        .init();
    spawner.spawn(coprocessor(coprocessor_task).unwrap());

    // A board on battery is never on USB, so it has no host to talk to.
    let link = match power_mode {
        PowerMode::Aux => {
            let (usb_device, link) = usb::Builder {
                usb: p.USB,
                dp: p.PA12,
                dm: p.PA11,
            }
            .init();
            spawner.spawn(usb(usb_device).unwrap());
            Some(link)
        }
        PowerMode::Battery => None,
    };

    let scl = p.PB8;
    let sda = p.PB9;
    let mut config = i2c::Config::default();
    config.timeout = Duration::from_millis(10);

    static I2C: StaticCell<Mutex<ThreadModeRawMutex, i2c::I2c<'static, Async, i2c::Master>>> =
        StaticCell::new();
    let i2c = I2C.init(Mutex::new(i2c::I2c::new(
        p.I2C1, scl, sda, p.DMA1_CH1, p.DMA2_CH1, Irqs, config,
    )));

    static CAPACITANCE: StaticCell<sensor::Shared<CapacitanceSensor<'static>>> = StaticCell::new();
    let capacitance = &*CAPACITANCE.init(Mutex::new(CapacitanceSensor::new(p.PB4, i2c)));

    static COLOR: StaticCell<sensor::Shared<ColorSensor<'static>>> = StaticCell::new();
    let color = match ColorSensor::init(i2c).await {
        Ok(sensor) => Some(&*COLOR.init(Mutex::new(sensor))),
        Err(e) => {
            defmt::warn!("color sensor failed to initialize: {}", e);
            None
        }
    };

    static TEMP_RH: StaticCell<sensor::Shared<TempRhSensor<'static>>> = StaticCell::new();
    let temp_rh = match TempRhSensor::init(i2c).await {
        Ok(sensor) => Some(&*TEMP_RH.init(Mutex::new(sensor))),
        Err(e) => {
            defmt::warn!("temp-rh sensor failed to initialize: {}", e);
            None
        }
    };

    static MIC_BUF: StaticCell<[u8; 512]> = StaticCell::new();
    let pdm = Pdm::new(
        p.SAI1,
        p.PA3,
        p.PA10,
        pdm::Channel::Right,
        p.DMA1_CH2,
        Irqs,
        MIC_BUF.init([0; 512]),
    );
    static MIC: StaticCell<sensor::Shared<Mic<'static>>> = StaticCell::new();
    let mic = &*MIC.init(Mutex::new(Mic::new(pdm)));

    let (report_task, report_handle) = report::Builder {
        socket: sockets.reports,
        config: report_config,
        capacitance,
        color,
        temp_rh,
        mic,
    }
    .init();

    let (discovery_task, discovery_handle) = discovery::Builder {
        socket: sockets.discovery,
        network: thread_handle.monitor(),
    }
    .init();

    let (metrics_task, metrics_handle) = metrics::Builder {
        tcp,
        config: metrics_config,
        supply: supply::init(Irqs),
        capacitance,
        color,
        temp_rh,
        mic,
    }
    .init();
    spawner.spawn(metrics(metrics_task).unwrap());

    let context = rpc::Context {
        thread: thread_handle,
        coprocessor: coprocessor_handle,
        bootloader: bootloader_handle,
        reports: report_handle,
        update: update_handle,
        config: config_handle,
        metrics: metrics_handle,
        discovery: discovery_handle,
        capacitance,
        temp_rh,
    };
    let publisher = link.map(|link| {
        let (server, publisher) = rpc::server(spawner, link, context);
        spawner.spawn(rpc(server).unwrap());
        publisher
    });
    spawner.spawn(reports(report_task, publisher.clone()).unwrap());
    spawner.spawn(discovery(discovery_task, publisher).unwrap());

    let button_task = button::Builder {
        button: p.PA7,
        exti: p.EXTI7,
        led: p.PA6,
        config: config_eraser,
    }
    .init();
    spawner.spawn(button(button_task).unwrap());
}
