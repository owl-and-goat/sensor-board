#![no_std]
#![no_main]

mod board_config;
mod coprocessor;
mod dfu;
mod discovery;
mod fault;
mod http;
mod i2c_device;
mod metrics;
mod persistent_config;
mod radio_flash;
mod report;
mod request;
mod rpc;
mod sensor;
mod thread;
mod update;
mod usb;

use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::{
    Config, Peri, bind_interrupts, dma,
    gpio::{self, Output},
    i2c,
    mode::Async,
    peripherals::{self, DMA1_CH1, DMA1_CH2, DMA2_CH1, I2C1},
    rcc,
    rtc::{Rtc, RtcConfig},
    wdg::IndependentWatchdog,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
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
});

fn configure_clocks() -> rcc::Config {
    let mut clocks = rcc::WPAN_DEFAULT;

    clocks.sys = rcc::Sysclk::HSE;
    clocks.hsi = true;
    clocks.core2_ahb_pre = rcc::AHBPrescaler::DIV1;

    // The PLL stays on only to give USB its 48 MHz from PLL Q. It is not the
    // system clock. USB cannot use HSI48, because CPU2 can switch that off.
    // TODO: disable this PLL and remove USB entirely so that we can go into
    // proper low-power sleep!
    clocks.hsi48 = None;
    clocks.mux.clk48sel = rcc::mux::Clk48sel::PLL1_Q;

    // The microphone (SAI1) wants a PDM clock of 2.048 MHz, which decimated by
    // 128 is 16 kHz audio. From 32 MHz that takes a /125 somewhere, and the
    // only place for it is the PLL input divider (shared by both PLLs) times
    // PLLSAI1's P: 32 / 5 = 6.4 MHz in, then 6.4 * 32 / 25 = 8.192 MHz for
    // SAI1, and 6.4 * 30 / 4 = 48 MHz for USB.
    clocks.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV5,
        mul: rcc::PllMul::MUL30,
        divp: None,
        divq: Some(rcc::PllQDiv::DIV4),
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
async fn reports(task: report::Task, publisher: rpc::Publisher) -> ! {
    task.run(publisher).await
}

#[embassy_executor::task]
async fn discovery(task: discovery::Task, publisher: rpc::Publisher) -> ! {
    task.run(publisher).await
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

    loop {
        Timer::after_millis(20).await;
        watchdog.pet();
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    for _ in 0..=2 {
        led.set_low();
        Timer::after(Duration::from_millis(200)).await;
        led.set_high();
        Timer::after(Duration::from_millis(200)).await;
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

    // let button = Input::new(p.PA7, Pull::Up); // SW1 to GND
    //

    spawner.spawn(watchdog(p.IWDG).unwrap());

    let (bootloader_task, bootloader_handle) = dfu::init();
    spawner.spawn(bootloader(bootloader_task).unwrap());

    let (update_handle, update_service) = update::init();
    let (config_handle, config_service, config_monitor) = board_config::init();
    let (coprocessor_task, coprocessor_handle, thread_handle, sockets, tcp) =
        coprocessor::Builder {
            ipcc: p.IPCC,
            flash: p.FLASH,
            update: update_service,
            board_config: config_service,
        }
        .init();
    spawner.spawn(coprocessor(coprocessor_task).unwrap());

    let (usb_device, link) = usb::Builder {
        usb: p.USB,
        dp: p.PA12,
        dm: p.PA11,
    }
    .init();
    spawner.spawn(usb(usb_device).unwrap());

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
        config: config_monitor,
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
    let (server, publisher) = rpc::server(spawner, link, context);
    spawner.spawn(rpc(server).unwrap());
    spawner.spawn(reports(report_task, publisher.clone()).unwrap());
    spawner.spawn(discovery(discovery_task, publisher).unwrap());

    let led = Output::new(p.PA6, gpio::Level::High, gpio::Speed::Low); // D1, active low
    spawner.spawn(blink(led).unwrap());
}
