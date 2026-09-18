//! CPU2 (wireless coprocessor) control from the console.
//!
//! - `wpan info`: what runs on CPU2 (FUS or a wireless stack) and versions.
//! - `fus state | upgrade [src] [dst] | start | delete`: FUS commands, used
//!   to install the CPU2 firmware. The image itself is written into flash by
//!   the host over DFU beforehand (`dfu-util -s <install address>`).
//! - `thread init`: start the Thread stack after CPU2 reports it running.
//! - `ot <line>`: relay one line to the OpenThread CLI inside the CPU2 Thread
//!   stack; its output comes back on the console.

use core::fmt::Write as _;

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker, Timer};

use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::ipcc::{self, ReceiveInterruptHandler, TransmitInterruptHandler};
use embassy_stm32::peripherals::{FLASH, IPCC};
use embassy_stm32::{Peri, bind_interrupts};
use embassy_stm32_wpan::TlMbox;
use embassy_stm32_wpan::sub::mm::MemoryManager;
use embassy_stm32_wpan::sub::sys::Sys;
use embassy_stm32_wpan::sub::thread::{ThreadCliRx, ThreadNotifRx, ThreadOt};
use embassy_stm32_wpan::sub::traces::Traces;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;

bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
});

pub type Line = heapless::String<512>;

/// Range-test mode: the board pings the leader every few seconds and the LED
/// shows the link (see main.rs): fast blink = detached, off = attached but no
/// reply, on = replies coming in (short off-pulse per reply).
pub static RANGE_MODE: AtomicBool = AtomicBool::new(false);
/// 0 detached, 1 attached without reply, 2 attached and getting replies.
pub static LINK: AtomicU8 = AtomicU8::new(0);
/// One-shot LED pulse request (a ping reply arrived).
pub static PULSE: AtomicBool = AtomicBool::new(false);

/// Where the wpan task is: 0 not started, 1 booting CPU2 (waiting for its
/// ready event), 2 running.
pub static STATE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
/// Ready event value: 0 wireless fw, 1 FUS, 0xFE unknown value, 0xFF none.
pub static READY: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

/// Flash controller (both cores) and hardware semaphores, for debugging a
/// CPU2 that stops answering.
pub fn status2_line(l: &mut heapless::String<224>) {
    use embassy_stm32::pac::{FLASH, HSEM, RCC};
    let _ = write!(
        l,
        "wpan: flash cr 0x{:08x} sr 0x{:08x} c2cr 0x{:08x} c2sr 0x{:08x} acr 0x{:08x} c2acr 0x{:08x} | ccipr 0x{:08x} crrcr 0x{:08x} extcfgr 0x{:08x} cr 0x{:08x} | hsem",
        FLASH.cr().read().0,
        FLASH.sr().read().0,
        FLASH.c2cr().read().0,
        FLASH.c2sr().read().0,
        FLASH.acr().read().0,
        FLASH.c2acr().read().0,
        RCC.ccipr().read().0,
        RCC.crrcr().read().0,
        RCC.extcfgr().read().0,
        RCC.cr().read().0,
    );
    for i in 0..10 {
        let r = HSEM.r(i).read();
        if r.lock() {
            let _ = write!(l, " {}:c{}", i, r.coreid());
        }
    }
    let _ = l.push_str("\r\n");
}

/// Register-level view for debugging CPU2 bring-up, usable from the console
/// even while the wpan task is blocked.
pub fn status_line(l: &mut heapless::String<224>) {
    use core::sync::atomic::Ordering;
    use embassy_stm32::pac::{FLASH, IPCC, PWR};
    let raw = unsafe {
        core::ptr::read_volatile(
            (&raw const embassy_stm32_wpan::tables::TL_DEVICE_INFO_TABLE) as *const [u32; 16],
        )
    };
    let ref0 = unsafe { core::ptr::read_volatile(0x2003_0000 as *const [u32; 3]) };
    let _ = write!(
        l,
        "wpan: state {} ready 0x{:02x} hsem5 {} fus_busy {} sfr 0x{:08x} srrvr 0x{:08x} c2boot {} ipccdba 0x{:x} c1.sr 0x{:03x} c1.mr 0x{:08x} c2.sr 0x{:03x} | ref {:08x} {:08x} {:08x} | devinfo {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}\r\n",
        STATE.load(Ordering::Relaxed),
        READY.load(Ordering::Relaxed),
        clk48_semaphore_held(),
        crate::dfu::is_fus_busy(),
        FLASH.sfr().read().0,
        FLASH.srrvr().read().0,
        PWR.cr4().read().c2boot(),
        FLASH.ipccbr().read().0,
        IPCC.cpu(0).sr().read().0,
        IPCC.cpu(0).mr().read().0,
        IPCC.cpu(1).sr().read().0,
        ref0[0],
        ref0[1],
        ref0[2],
        raw[0],
        raw[1],
        raw[2],
        raw[3],
        raw[4],
        raw[5]
    );
}

/// Console -> wpan task: one command line per message.
pub static CMD: Channel<CriticalSectionRawMutex, Line, 4> = Channel::new();
/// wpan task -> console: text to print.
pub static OUT: Pipe<CriticalSectionRawMutex, 2048> = Pipe::new();

/// True for console lines that belong to this module.
pub fn owns(line: &str) -> bool {
    let first = line.split_whitespace().next().unwrap_or("");
    matches!(first, "wpan" | "fus" | "thread" | "ot")
}

/// Queue console text. If the console is not draining (no host reading),
/// give up after a short wait rather than block the caller.
pub async fn out(s: &str) {
    let _ =
        embassy_time::with_timeout(Duration::from_millis(100), OUT.write_all(s.as_bytes())).await;
}

/// The relay output, driven by keep-alive; `None` until the wpan task owns it.
pub static RELAY_ON: AtomicBool = AtomicBool::new(false);

pub async fn outf(args: core::fmt::Arguments<'_>) {
    let mut l: heapless::String<224> = heapless::String::new();
    let _ = l.write_fmt(args);
    out(&l).await;
}

#[embassy_executor::task]
async fn run_mm_queue(mut mm: MemoryManager<'static>) {
    mm.run_queue().await;
}

/// Shared SRAM2a as found at boot, before the mailbox init clears it. When
/// the ROM bootloader ran before us it booted CPU2, and FUS wrote its device
/// info table (MB_FUS_DeviceInfoTable_t) for the bootloader in here.
static mut SRAM2A_AT_BOOT: [u32; 64] = [0; 64];

fn snapshot_sram2a() {
    unsafe {
        let snap = core::ptr::read_volatile(0x2003_0000 as *const [u32; 64]);
        core::ptr::write_volatile(&raw mut SRAM2A_AT_BOOT, snap);
    }
}

async fn print_snapshot() {
    let snap = unsafe { core::ptr::read_volatile(&raw const SRAM2A_AT_BOOT) };
    for row in 0..8 {
        let w = &snap[row * 8..row * 8 + 8];
        outf(format_args!(
            "wpan: sram2a+{:03x}: {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}\r\n",
            row * 32,
            w[0],
            w[1],
            w[2],
            w[3],
            w[4],
            w[5],
            w[6],
            w[7]
        ))
        .await;
    }
    if let Some(k) = snap.iter().position(|&w| w == FUS_TABLE_KEYWORD) {
        if k + 9 < snap.len() {
            let (a, b, c) = version(snap[k + 3]);
            let (d, e, f) = version(snap[k + 5]);
            outf(format_args!(
                "wpan: FUS table at boot (+0x{:x}): FUS v{a}.{b}.{c} mem 0x{:08x} | stack v{d}.{e}.{f} mem 0x{:08x} type 0x{:02x} thread_info 0x{:08x} | last fus state 0x{:02x} last stack state 0x{:02x}\r\n",
                k * 4, snap[k + 4], snap[k + 6], snap[k + 1] >> 24, snap[k + 8], (snap[k + 1] >> 8) & 0xff, (snap[k + 1] >> 16) & 0xff
            ))
            .await;
        }
    } else {
        out("wpan: no FUS table keyword in the boot snapshot\r\n").await;
    }
}

/// Hardware semaphore 5 guards the 48 MHz clock (CLK48/HSI48). When a
/// wireless stack starts, CPU2 takes it and switches that clock off unless
/// CPU1 already holds it. USB needs the clock, so take it before C2BOOT and
/// never release it (AN5289; same as ST's USB examples on WB).
const HSEM_CLK48: usize = 5;

fn lock_clk48_semaphore() -> bool {
    use embassy_stm32::pac::{HSEM, RCC};
    RCC.ahb3enr().modify(|w| w.set_hsemen(true));
    // One-step lock: reading RLR locks the semaphore for this core if free.
    let r = HSEM.rlr(HSEM_CLK48).read();
    r.lock()
}

pub fn clk48_semaphore_held() -> bool {
    let r = embassy_stm32::pac::HSEM.r(HSEM_CLK48).read();
    r.lock()
}

/// Release semaphore 5 (CPU1 core id is 4 in the HSEM registers).
fn release_clk48_semaphore() {
    embassy_stm32::pac::HSEM.r(HSEM_CLK48).write(|w| {
        w.set_lock(false);
        w.set_coreid(4);
        w.set_procid(0);
    });
}

fn clk48_line(l: &mut heapless::String<224>) {
    use embassy_stm32::pac::RCC;
    let _ = write!(
        l,
        "wpan: clk48sel {} hsi48on {} hsi48rdy {} pllon {} pllrdy {} hsem5 {}\r\n",
        RCC.ccipr().read().clk48sel().to_bits(),
        RCC.crrcr().read().hsi48on(),
        RCC.crrcr().read().hsi48rdy(),
        RCC.cr().read().pllon(),
        RCC.cr().read().pllrdy(),
        clk48_semaphore_held()
    );
}

#[embassy_executor::task]
pub async fn wpan_task(
    spawner: Spawner,
    ipcc: Peri<'static, IPCC>,
    flash: Peri<'static, FLASH>,
    mut relay: embassy_stm32::gpio::Output<'static>,
) {
    // Relay follows RELAY_ON, checked every 100 ms by a small side loop.
    let relay_fut = async move {
        loop {
            Timer::after_millis(100).await;
            if RELAY_ON.load(Ordering::Relaxed) {
                relay.set_high();
            } else {
                relay.set_low();
            }
        }
    };
    snapshot_sram2a();
    let sem = lock_clk48_semaphore();
    outf(format_args!("wpan: CLK48 semaphore locked: {sem}\r\n")).await;
    out("wpan: booting CPU2\r\n").await;
    STATE.store(1, core::sync::atomic::Ordering::Relaxed);
    let mbox = TlMbox::init_without_ready(ipcc, Irqs, ipcc::Config::default());
    let mut sys = mbox.sys_subsystem;
    // CPU2 sends its ready event once after it boots. A CPU1-only reset (our
    // DFU round trips) leaves CPU2 running, so don't wait forever.
    let mut ready = match select(sys.read_ready(), Timer::after_secs(2)).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    };
    if ready.is_none() {
        // No ready event: CPU2 was already up (the ROM bootloader boots it for
        // its own FUS commands, and a CPU1-only reset does not restart it).
        out("wpan: no ready event, ").await;
        ready = reinit(&mut sys).await;
    }
    STATE.store(2, core::sync::atomic::Ordering::Relaxed);
    READY.store(
        match ready {
            None => 0xFF,
            Some(Err(())) => 0xFE,
            Some(Ok(r)) => r as u8,
        },
        core::sync::atomic::Ordering::Relaxed,
    );
    outf(format_args!("wpan: CPU2 ready event: {ready:?}\r\n")).await;
    spawner.spawn(run_mm_queue(mbox.mm_subsystem).unwrap());

    let (mut ot, mut cli_rx, mut notif_rx) = mbox.thread_subsystem.split();
    let mut traces = mbox.traces_subsystem;
    let mut flash = Flash::new_blocking(flash);
    print_info(&sys).await;

    embassy_futures::join::join5(
        command_loop(&mut sys, &mut ot, &mut flash),
        cli_output_loop(&mut cli_rx),
        notification_loop(&mut notif_rx),
        traces_loop(&mut traces),
        relay_fut,
    )
    .await;
}

/// "Fake a C2BOOT when it has already been set" (ST's words): SHCI_C2_REINIT
/// followed by a SEV instruction makes CPU2 restart its firmware, re-read the
/// reference table and send its ready event again.
async fn reinit(
    sys: &mut Sys<'_>,
) -> Option<Result<embassy_stm32_wpan::shci::SchiSysEventReady, ()>> {
    let r = sys.shci_c2_reinit().await;
    cortex_m::asm::sev();
    outf(format_args!(
        "wpan: REINIT -> {r:?}, SEV sent, waiting for ready\r\n"
    ))
    .await;
    let ready = match select(sys.read_ready(), Timer::after_secs(3)).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    };
    READY.store(
        match ready {
            None => 0xFF,
            Some(Err(())) => 0xFE,
            Some(Ok(r)) => r as u8,
        },
        core::sync::atomic::Ordering::Relaxed,
    );
    outf(format_args!("wpan: ready event: {ready:?}\r\n")).await;
    ready
}

fn version(v: u32) -> (u8, u8, u8) {
    ((v >> 24) as u8, (v >> 16) as u8, (v >> 8) as u8)
}

/// FUS writes FUS_DEVICE_INFO_TABLE_VALIDITY_KEYWORD as the first word when
/// it is the one running; the table then has the MB_FUS_DeviceInfoTable_t
/// layout instead of the wireless-firmware one.
const FUS_TABLE_KEYWORD: u32 = 0xA946_56B9;

async fn print_info(sys: &Sys<'_>) {
    let raw = sys.device_info_raw();
    if raw[0] == FUS_TABLE_KEYWORD {
        let (a, b, c) = version(raw[3]);
        let (d, e, f) = version(raw[5]);
        outf(format_args!(
            "wpan: FUS running: FUS v{a}.{b}.{c} (0x{:08x}) mem 0x{:08x} | installed stack v{d}.{e}.{f} (0x{:08x}) mem 0x{:08x} type 0x{:02x} thread_info 0x{:08x} | last fus state 0x{:02x} last stack state 0x{:02x}\r\n",
            raw[3], raw[4], raw[5], raw[6], raw[1] >> 24, raw[8], (raw[1] >> 8) & 0xff, (raw[1] >> 16) & 0xff
        ))
        .await;
        return;
    }
    let info = sys.device_info();
    let (fus_v, fus_mem) = {
        let t = info.rss_info_table;
        (t.version, t.memory_size)
    };
    let (ws_v, ws_mem, ws_info) = {
        let t = info.wireless_fw_info_table;
        (t.version, t.memory_size, t.thread_info)
    };
    let (a, b, c) = version(fus_v);
    outf(format_args!(
        "wpan: FUS v{a}.{b}.{c} (0x{fus_v:08x}) mem 0x{fus_mem:08x}\r\n"
    ))
    .await;
    let (a, b, c) = version(ws_v);
    let stack = match ws_info & 0xff {
        0x00 => "none",
        0x01 => "BLE full",
        0x02 => "BLE HCI",
        0x10 => "Thread FTD",
        0x11 => "Thread MTD",
        0x40 => "802.15.4 MAC",
        0x50 => "BLE+Thread static",
        _ => "?",
    };
    outf(format_args!(
        "wpan: stack v{a}.{b}.{c} (0x{ws_v:08x}) mem 0x{ws_mem:08x} info 0x{ws_info:08x} = {stack}\r\n"
    ))
    .await;
}

fn parse_hex(s: Option<&str>) -> u32 {
    let s = s.unwrap_or("0");
    let s = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .unwrap_or(s);
    u32::from_str_radix(s, 16).unwrap_or(0)
}

/// True when CPU2 reports a running Thread stack (not FUS, not another stack).
fn thread_stack_running(sys: &Sys<'_>) -> bool {
    sys.device_info_raw()[0] != FUS_TABLE_KEYWORD
        && sys
            .wireless_fw_info()
            .map(|i| i.thread_info & 0xff == 0x10)
            .unwrap_or(false)
}

async fn command_loop(sys: &mut Sys<'_>, ot: &mut ThreadOt<'_>, flash: &mut Flash<'_, Blocking>) {
    // Autostart: a stored dataset with the autostart flag, and a Thread stack
    // on CPU2, means join the network now and run the range probe.
    let mut prefix: Option<[u8; 8]> = None;
    if let Some(cfg) = crate::config::load() {
        prefix = cfg.mesh_local_prefix();
        RELAY_ON.store(cfg.keepalive(), Ordering::Relaxed);
        if cfg.autostart() && thread_stack_running(sys) {
            let r = sys.shci_c2_thread_init().await;
            outf(format_args!("wpan: autostart: thread init -> {r:?}\r\n")).await;
            crate::ot::autostart(ot, &cfg).await;
            RANGE_MODE.store(true, Ordering::Relaxed);
        }
    }

    let mut ticker = Ticker::every(Duration::from_secs(3));
    let mut replies_seen = 0u32;
    loop {
        let line = match select(CMD.receive(), ticker.next()).await {
            Either::First(line) => line,
            Either::Second(()) => {
                if RANGE_MODE.load(Ordering::Relaxed) {
                    range_tick(ot, &mut prefix, &mut replies_seen).await;
                }
                continue;
            }
        };
        let mut words = line.split_whitespace();
        match (words.next(), words.next()) {
            (Some("wpan"), Some("snapshot")) => print_snapshot().await,
            (Some("wpan"), Some("sem5")) => {
                match words.next() {
                    Some("release") => release_clk48_semaphore(),
                    Some("lock") => {
                        lock_clk48_semaphore();
                    }
                    _ => {}
                }
                let mut l: heapless::String<224> = heapless::String::new();
                clk48_line(&mut l);
                out(&l).await;
            }
            (Some("wpan"), Some("clk48")) => {
                if words.next() == Some("fix") {
                    use embassy_stm32::pac::RCC;
                    RCC.ccipr().modify(|w| {
                        w.set_clk48sel(embassy_stm32::pac::rcc::vals::Clk48sel::PLL1_Q)
                    });
                }
                let mut l: heapless::String<224> = heapless::String::new();
                clk48_line(&mut l);
                out(&l).await;
            }
            (Some("wpan"), Some("reinit")) => {
                reinit(sys).await;
            }
            (Some("wpan"), _) => print_info(sys).await,
            (Some("fus"), Some("state")) => {
                let (state, err) = sys.shci_c2_fus_get_state().await;
                outf(format_args!(
                    "fus: state 0x{state:02x} error 0x{err:02x}\r\n"
                ))
                .await;
                if state == 0x00 && crate::dfu::is_fus_busy() {
                    crate::dfu::fus_busy(false);
                    out("fus: idle, busy marker cleared\r\n").await;
                }
            }
            (Some("fus"), Some("upgrade")) => {
                let src = parse_hex(words.next());
                let dst = parse_hex(words.next());
                outf(format_args!(
                    "fus: fw_upgrade src 0x{src:08x} dst 0x{dst:08x}\r\n"
                ))
                .await;
                crate::dfu::fus_busy(true);
                let r = sys.shci_c2_fus_fwupgrade(src, dst).await;
                outf(format_args!("fus: fw_upgrade -> {r:?}\r\n")).await;
            }
            (Some("fus"), Some("start")) => {
                crate::dfu::fus_busy(true);
                let r = sys.shci_c2_fus_startws().await;
                outf(format_args!("fus: start_ws -> {r:?}\r\n")).await;
            }
            (Some("fus"), Some("delete")) => {
                let r = sys.shci_c2_fus_fw_delete().await;
                outf(format_args!("fus: fw_delete -> {r:?}\r\n")).await;
            }
            (Some("thread"), Some("init")) => {
                let r = sys.shci_c2_thread_init().await;
                outf(format_args!("thread: init -> {r:?}\r\n")).await;
            }
            (Some("ot"), _) => crate::ot::command(&line, ot, sys, flash).await,
            _ => out("wpan: unknown command\r\n").await,
        }
    }
}

/// One range-probe step: ping the leader anycast address and report the link.
async fn range_tick(ot: &mut ThreadOt<'_>, prefix: &mut Option<[u8; 8]>, replies_seen: &mut u32) {
    use core::fmt::Write as _;
    let role = crate::ot::role(ot).await;
    if role < 2 {
        LINK.store(0, Ordering::Relaxed);
        outf(format_args!(
            "range: role {} (not attached)\r\n",
            crate::ot::role_name(role)
        ))
        .await;
        return;
    }
    if prefix.is_none() {
        let eid = ot
            .call(crate::otids::MSG_M4TOM0_OT_THREAD_GET_MESH_LOCAL_EID, &[])
            .await;
        if (0x2000_0000..0x2004_0000).contains(&eid) {
            let a = unsafe { core::ptr::read_volatile(eid as *const [u8; 16]) };
            let mut p = [0u8; 8];
            p.copy_from_slice(&a[..8]);
            *prefix = Some(p);
        }
    }
    let Some(p) = *prefix else {
        return;
    };
    let before = crate::ot::REPLIES.load(Ordering::Relaxed);
    let r = crate::ot::ping(ot, &crate::ot::leader_aloc(&p), 1, 900).await;
    Timer::after_millis(1200).await;
    let got = crate::ot::REPLIES.load(Ordering::Relaxed) != before;
    *replies_seen = crate::ot::REPLIES.load(Ordering::Relaxed);
    LINK.store(if got { 2 } else { 1 }, Ordering::Relaxed);
    let mut l: heapless::String<224> = heapless::String::new();
    let _ = write!(l, "range: role {}", crate::ot::role_name(role));
    if role == 2 {
        if let Some((avg, last)) = crate::ot::parent_rssi(ot).await {
            let _ = write!(l, " parent rssi {avg}/{last} dBm");
        }
    } else {
        let _ = l.push_str(" neighbors");
        if crate::ot::neighbors_summary(ot, &mut l).await == 0 {
            let _ = l.push_str(" none");
        }
    }
    if r != 0 {
        let _ = write!(l, " | ping -> {}", crate::ot::err_name(r));
    } else if got {
        let _ = write!(
            l,
            " | leader ping {} ms",
            crate::ot::LAST_RTT.load(Ordering::Relaxed)
        );
    } else {
        let _ = l.push_str(" | leader ping: no reply");
    }
    let _ = l.push_str("\r\n");
    out(&l).await;
}

async fn cli_output_loop(cli_rx: &mut ThreadCliRx<'_>) {
    let mut buf = [0u8; 256];
    loop {
        let n = cli_rx.receive(&mut buf).await;
        OUT.write_all(&buf[..n]).await;
    }
}

async fn traces_loop(traces: &mut Traces<'_>) {
    loop {
        let evt = traces.read().await;
        let payload = evt.payload();
        out("cpu2: ").await;
        let mut i = 0;
        while i < payload.len() {
            let chunk = &payload[i..(i + 64).min(payload.len())];
            let mut l: heapless::String<224> = heapless::String::new();
            for &b in chunk {
                let _ = l.push(if (0x20..0x7f).contains(&b) || b == b'\n' || b == b'\r' {
                    b as char
                } else {
                    '.'
                });
            }
            out(&l).await;
            i += chunk.len();
        }
        out("\r\n").await;
    }
}

async fn notification_loop(notif_rx: &mut ThreadNotifRx<'_>) {
    loop {
        let n = notif_rx.receive().await;
        crate::ot::notification(n).await;
    }
}
