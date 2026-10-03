//! USB device setup, including DFU entry requests and the console.

use core::{
    fmt::Write as _,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_futures::{
    join::join3,
    select::{Either3, select3},
};

use embassy_stm32::{
    gpio::{Input, Output},
    peripherals::IWDG,
    rtc::RtcTimeProvider,
    usb::{self, Driver},
    wdg::IndependentWatchdog,
};

use embassy_time::{Duration, Instant, Timer, with_timeout};

use embassy_usb::{
    Builder,
    class::{
        cdc_acm::{CdcAcmClass, Sender, State as CdcState},
        dfu::{
            app_mode::{DfuState, Handler as DfuHandler, usb_dfu},
            consts::DfuAttributes,
        },
    },
    driver::Driver as UsbDriver,
};

use crate::{console, dfu, wpan};

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

pub(crate) async fn run<T>(
    driver: Driver<'_, T>,
    rtc_time: RtcTimeProvider,
    led: &mut Output<'_>,
    button: &Input<'_>,
    watchdog: IndependentWatchdog<'_, IWDG>,
) where
    T: usb::Instance,
{
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
    let mut dfu_state = DfuState::new(
        EnterDfu,
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::WILL_DETACH,
        Duration::from_millis(1000),
    );

    let mut cdc_state = CdcState::new();

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

    join3(
        usb.run(),
        console(cdc, rtc_time, button),
        watchdog_housekeeping(led, button, watchdog),
    )
    .await;
}

// Console: heartbeat every two seconds, echo, line-buffered commands
// (`dfu`, `hang`, and the CPU2 commands in `wpan.rs`), plus CPU2 output.
// Output is dropped when no host has the port open (DTR low) and never
// blocks for long: a headless board must keep running.
async fn console<'d, T>(
    cdc: CdcAcmClass<'d, Driver<'d, T>>,
    rtc_time: RtcTimeProvider,
    button: &Input<'_>,
) where
    T: usb::Instance,
{
    let (mut tx, mut rx) = cdc.split();
    let mut buf = [0u8; 64];
    let mut wbuf = [0u8; 64];
    let mut line: heapless::String<192> = heapless::String::new();
    let mut cmdline: heapless::String<512> = heapless::String::new();
    loop {
        match select3(
            rx.read_packet(&mut buf),
            Timer::after_secs(2),
            console::OUT.read(&mut wbuf),
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
                write_all(&mut tx, line.as_bytes()).await;
            }
            Either3::Third(n) => {
                write_all(&mut tx, &wbuf[..n]).await;
                // Flush everything already queued so a long message is not
                // interleaved with the heartbeat.
                while let Ok(n) = console::OUT.try_read(&mut wbuf) {
                    write_all(&mut tx, &wbuf[..n]).await;
                }
            }
        }
    }
}

// LED, button, watchdog, DFU requests. LED: heartbeat blink normally;
// in range mode it shows the link (see wpan::LINK).
async fn watchdog_housekeeping(
    led: &mut Output<'_>,
    button: &Input<'_>,
    mut watchdog: IndependentWatchdog<'_, IWDG>,
) {
    let mut held_since: Option<Instant> = None;
    let mut tick: u32 = 0;
    let mut pulse: u8 = 0;
    loop {
        Timer::after_millis(20).await;
        watchdog.pet();
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
        // Deliberate panic to exercise panic-probe and the HardFault -> DFU path.
        "panic" => {
            write_all(tx, b"\r\npanicking\r\n").await;
            panic!("console `panic` command");
        }
        "status" => {
            let mut l: heapless::String<224> = heapless::String::new();
            console::status_line(&mut l);
            write_all(tx, l.as_bytes()).await;
            l.clear();
            console::status2_line(&mut l);
            write_all(tx, l.as_bytes()).await;
        }
        _ if console::owns(cmd) => {
            let mut l = console::Line::new();
            let _ = l.push_str(cmd);
            if console::CMD.try_send(l).is_err() {
                write_all(tx, b"wpan: busy\r\n").await;
            }
        }
        _ => {
            write_all(
                tx,
                b"? (dfu, hang, panic, wpan info, fus ..., thread init, ot <cli>)\r\n",
            )
            .await
        }
    }
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
