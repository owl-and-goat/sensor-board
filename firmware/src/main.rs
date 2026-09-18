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
mod wpan;

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{Config, bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State as CdcState};
use embassy_usb::class::dfu::app_mode::{DfuState, Handler as DfuHandler, usb_dfu};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::driver::Driver as UsbDriver;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

/// pid.codes test-range VID/PID; fine for a one-off board.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;

/// Set from the USB control handler, the console or the button; acted on by
/// the watch loop so the USB status stage can complete before we reset.
static DFU_REQUESTED: AtomicBool = AtomicBool::new(false);

struct EnterDfu;

impl DfuHandler for EnterDfu {
    fn enter_dfu(&mut self) {
        DFU_REQUESTED.store(true, Ordering::SeqCst);
    }
}

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

    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // CPU2 bring-up runs on its own so the console works even if CPU2 is silent.
    // Relay K1 (PA8 high = energised): optional keep-alive load for USB power banks.
    let relay = Output::new(p.PA8, Level::Low, Speed::Low);
    spawner.spawn(wpan::wpan_task(spawner, p.IPCC, p.FLASH, relay).unwrap());

    let mut usb_config = embassy_usb::Config::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some("aspen");
    usb_config.product = Some("sensor-board");
    // Chip UID as the USB serial, so two boards can be told apart.
    usb_config.serial_number = Some(embassy_stm32::uid::uid_hex());
    usb_config.max_power = 100;
    // CDC-ACM + DFU runtime is a composite device: needs IADs.
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut msos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];
    let mut cdc_state = CdcState::new();
    let mut dfu_state = DfuState::new(
        EnterDfu,
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::WILL_DETACH,
        Duration::from_millis(1000),
    );

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );
    let cdc = CdcAcmClass::new(&mut builder, &mut cdc_state, 64);
    usb_dfu(&mut builder, &mut dfu_state, |_| {});
    let mut usb = builder.build();

    let usb_fut = usb.run();

    // Console: heartbeat every two seconds, echo, line-buffered commands
    // (`dfu`, `hang`, and the CPU2 commands in `wpan.rs`), plus CPU2 output.
    // Output is dropped when no host has the port open (DTR low) and never
    // blocks for long: a headless board must keep running.
    let (mut tx, mut rx) = cdc.split();
    let console_fut = async {
        let mut buf = [0u8; 64];
        let mut wbuf = [0u8; 64];
        let mut line: heapless::String<192> = heapless::String::new();
        let mut cmdline: heapless::String<512> = heapless::String::new();
        loop {
            match select3(
                rx.read_packet(&mut buf),
                Timer::after_secs(2),
                wpan::OUT.read(&mut wbuf),
            )
            .await
            {
                Either3::First(Ok(n)) => {
                    write_all(&mut tx, &buf[..n]).await;
                    for &b in &buf[..n] {
                        match b {
                            b'\r' | b'\n' => {
                                let cmd = cmdline.trim();
                                if !cmd.is_empty() {
                                    handle_command(cmd, &mut tx).await;
                                }
                                cmdline.clear();
                            }
                            _ => {
                                let _ = cmdline.push(b as char);
                            }
                        }
                    }
                }
                Either3::First(Err(_)) => Timer::after_millis(200).await,
                Either3::Second(()) => {
                    // SSR must be read before TR/DR (which `now()` reads):
                    // that locks the shadow registers into one snapshot.
                    let ss = embassy_stm32::pac::RTC.ssr().read().ss() as u32;
                    let (h, m, s) = match rtc_time.now() {
                        Ok(t) => (t.hour(), t.minute(), t.second()),
                        Err(_) => (99, 99, 99),
                    };
                    // PREDIV_S is 255: subsecond fraction = (255 - SS) / 256.
                    let ms = (255u32.saturating_sub(ss)) * 1000 / 256;
                    line.clear();
                    let _ = write!(
                        line,
                        "sensor-board (built {}) alive: uptime {}ms rtc {:02}:{:02}:{:02}.{:03} button {}\r\n",
                        env!("BUILD_STAMP"),
                        Instant::now().as_millis(),
                        h,
                        m,
                        s,
                        ms,
                        if button.is_low() { "down" } else { "up" }
                    );
                    write_all(&mut tx, line.as_bytes()).await;
                    line.clear();
                    clock_report(&mut line);
                    write_all(&mut tx, line.as_bytes()).await;
                }
                Either3::Third(n) => {
                    write_all(&mut tx, &wbuf[..n]).await;
                    // Flush everything already queued so a long message is not
                    // interleaved with the heartbeat.
                    while let Ok(n) = wpan::OUT.try_read(&mut wbuf) {
                        write_all(&mut tx, &wbuf[..n]).await;
                    }
                }
            }
        }
    };

    // LED, button, watchdog, DFU requests. LED: heartbeat blink normally;
    // in range mode it shows the link (see wpan::LINK).
    let watch_fut = async {
        let mut held_since: Option<Instant> = None;
        let mut tick: u32 = 0;
        let mut pulse: u8 = 0;
        loop {
            Timer::after_millis(20).await;
            dfu::pet_watchdog();
            tick = tick.wrapping_add(1);
            if !wpan::RANGE_MODE.load(Ordering::Relaxed) {
                if tick % 25 == 0 {
                    led.toggle();
                }
            } else {
                match wpan::LINK.load(Ordering::Relaxed) {
                    // detached: fast blink
                    0 => {
                        if tick % 5 == 0 {
                            led.toggle();
                        }
                    }
                    // attached, no ping reply: short flash every 2 s
                    1 => {
                        if tick % 100 < 3 {
                            led.set_low();
                        } else {
                            led.set_high();
                        }
                    }
                    // attached and pinging: on, with a short off-pulse per reply
                    _ => {
                        if wpan::PULSE.swap(false, Ordering::Relaxed) {
                            pulse = 4;
                        }
                        if pulse > 0 {
                            pulse -= 1;
                            led.set_high();
                        } else {
                            led.set_low();
                        }
                    }
                }
            }
            if button.is_low() {
                let since = *held_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_secs(2) {
                    DFU_REQUESTED.store(true, Ordering::SeqCst);
                }
            } else {
                held_since = None;
            }
            if DFU_REQUESTED.load(Ordering::SeqCst) {
                // Let the DFU_DETACH status stage / console write finish.
                led.set_low();
                Timer::after_millis(100).await;
                dfu::reboot_into_bootloader();
            }
        }
    };

    join3(usb_fut, console_fut, watch_fut).await;
}

/// Console commands that live in this file; the rest go to `wpan`.
async fn handle_command<'d, D: UsbDriver<'d>>(cmd: &str, tx: &mut Sender<'d, D>) {
    match cmd {
        "dfu" => {
            write_all(tx, b"\r\nrebooting into DFU\r\n").await;
            DFU_REQUESTED.store(true, Ordering::SeqCst);
        }
        // Deliberate hang to exercise the watchdog -> DFU path.
        "hang" => {
            write_all(tx, b"\r\nhanging; watchdog should fire in 4 s\r\n").await;
            loop {
                cortex_m::asm::nop();
            }
        }
        "status" => {
            let mut l: heapless::String<224> = heapless::String::new();
            wpan::status_line(&mut l);
            write_all(tx, l.as_bytes()).await;
            l.clear();
            wpan::status2_line(&mut l);
            write_all(tx, l.as_bytes()).await;
        }
        _ if wpan::owns(cmd) => {
            let mut l = wpan::Line::new();
            let _ = l.push_str(cmd);
            if wpan::CMD.try_send(l).is_err() {
                write_all(tx, b"wpan: busy\r\n").await;
            }
        }
        _ => {
            write_all(
                tx,
                b"? (dfu, hang, wpan info, fus ..., thread init, ot <cli>)\r\n",
            )
            .await
        }
    }
}

/// One line describing where the clocks actually come from, straight from
/// the RCC registers rather than from our own config.
fn clock_report(line: &mut heapless::String<192>) {
    use embassy_stm32::pac::RCC;
    let cr = RCC.cr().read();
    let sys = match RCC.cfgr().read().sws().to_bits() {
        0 => "MSI",
        1 => "HSI16",
        2 => "HSE",
        _ => "PLL",
    };
    let pllsrc = match RCC.pllcfgr().read().pllsrc().to_bits() {
        1 => "MSI",
        2 => "HSI16",
        3 => "HSE",
        _ => "none",
    };
    let clk48 = match RCC.ccipr().read().clk48sel().to_bits() {
        0 => "HSI48",
        1 => "PLLSAI1Q",
        2 => "PLLQ",
        _ => "MSI",
    };
    let bdcr = RCC.bdcr().read();
    let rtcsel = match bdcr.rtcsel().to_bits() {
        1 => "LSE",
        2 => "LSI",
        3 => "HSE/32",
        _ => "none",
    };
    let _ = write!(
        line,
        "clocks: sysclk={sys} pllsrc={pllsrc} usb48={clk48} rtc={rtcsel} hse_rdy={} lse_rdy={} msi_on={} hsi16_on={} hsi48_on={} iwdg pr={} rl={}\r\n",
        cr.hserdy(),
        bdcr.lserdy(),
        cr.msion(),
        cr.hsion(),
        RCC.crrcr().read().hsi48on(),
        embassy_stm32::pac::IWDG.pr().read().pr().to_bits(),
        embassy_stm32::pac::IWDG.rlr().read().rl(),
    );
}

/// Write `data` as a run of max-size packets, plus a zero-length packet when
/// the last one was full, so the host sees where the transfer ends. A single
/// `write_packet` call fails on anything longer than one packet (64 bytes).
/// Dropped outright when the host has not opened the port (no DTR), and
/// abandoned if the host stops reading, so nothing upstream ever blocks.
async fn write_all<'d, D: UsbDriver<'d>>(tx: &mut Sender<'d, D>, data: &[u8]) {
    if !tx.dtr() {
        return;
    }
    let max = tx.max_packet_size() as usize;
    for chunk in data.chunks(max) {
        match with_timeout(Duration::from_millis(50), tx.write_packet(chunk)).await {
            Ok(Ok(())) => {}
            _ => return,
        }
    }
    if !data.is_empty() && data.len() % max == 0 {
        let _ = with_timeout(Duration::from_millis(50), tx.write_packet(&[])).await;
    }
}
