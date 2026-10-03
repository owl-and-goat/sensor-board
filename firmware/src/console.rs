//! The USB console's command surface: everything that turns board state into
//! text and text back into actions.
//!
//! - `wpan info | snapshot | sem5 [lock|release] | clk48 [fix] | reinit`
//! - `fus state | upgrade [src] [dst] | start | delete`: FUS commands used to
//!   install CPU2 firmware. The image itself is written into flash by the host
//!   over DFU beforehand (`dfu-util -s <install address>`).
//! - `thread init`: start the Thread stack after CPU2 reports it running.
//! - `ot ...`: OpenThread, see `ot.rs`.
//!
//! `wpan.rs` owns the hardware and holds no text; it lends this module the
//! mailbox subsystem handles for the lifetime of its task.

use core::fmt::Write as _;
use core::sync::atomic::Ordering;

use embassy_futures::join::join4;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker, Timer};

use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32_wpan::sub::sys::Sys;
use embassy_stm32_wpan::sub::thread::{ThreadCliRx, ThreadNotifRx};
use embassy_stm32_wpan::sub::traces::Traces;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;

use crate::thread::{Dataset, Event, Ip6Address, PingConfig, Role, Thread};
use crate::{dfu, persistent_config, wpan};

pub type Line = heapless::String<512>;

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

pub async fn outf(args: core::fmt::Arguments<'_>) {
    let mut l: heapless::String<224> = heapless::String::new();
    let _ = l.write_fmt(args);
    out(&l).await;
}

/// Report what CPU2 bring-up did, then serve the console for the rest of time.
pub async fn run(
    bringup: wpan::BringUp,
    sys: &mut Sys<'_>,
    thread: &mut Thread<'_>,
    cli_rx: &mut ThreadCliRx<'_>,
    notif_rx: &mut ThreadNotifRx<'_>,
    traces: &mut Traces<'_>,
    flash: &mut Flash<'_, Blocking>,
) {
    outf(format_args!(
        "wpan: CLK48 semaphore locked: {}\r\n",
        bringup.clk48_semaphore_locked
    ))
    .await;
    if let Some(status) = bringup.reinit {
        outf(format_args!(
            "wpan: no ready event; REINIT -> {status:?}, SEV sent\r\n"
        ))
        .await;
    }
    outf(format_args!(
        "wpan: CPU2 ready event: {:?}\r\n",
        bringup.ready
    ))
    .await;
    print_info(sys).await;

    join4(
        command_loop(sys, thread, flash),
        cli_output_loop(cli_rx),
        notification_loop(notif_rx),
        traces_loop(traces),
    )
    .await;
}

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
        wpan::STATE.load(Ordering::Relaxed),
        wpan::READY.load(Ordering::Relaxed),
        wpan::clk48_semaphore_held(),
        dfu::is_fus_busy(),
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
        wpan::clk48_semaphore_held()
    );
}

async fn print_snapshot() {
    let snap = wpan::sram2a_snapshot();
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
    if let Some(k) = snap.iter().position(|&w| w == wpan::FUS_TABLE_KEYWORD) {
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

fn version(v: u32) -> (u8, u8, u8) {
    ((v >> 24) as u8, (v >> 16) as u8, (v >> 8) as u8)
}

async fn print_info(sys: &Sys<'_>) {
    let raw = sys.device_info_raw();
    if raw[0] == wpan::FUS_TABLE_KEYWORD {
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

/// Replies seen so far, and the last round-trip time, fed by the ping events.
static REPLIES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static LAST_RTT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

async fn command_loop(sys: &mut Sys<'_>, thread: &mut Thread<'_>, flash: &mut Flash<'_, Blocking>) {
    // Autostart: a stored dataset with the autostart flag, and a Thread stack
    // on CPU2, means join the network now and run the range probe.
    let mut prefix: Option<[u8; 8]> = None;
    if let Some(cfg) = config::load() {
        prefix = cfg.mesh_local_prefix();
        if cfg.autostart() && wpan::thread_stack_running(sys) {
            let r = sys.shci_c2_thread_init().await;
            outf(format_args!("wpan: autostart: thread init -> {r:?}\r\n")).await;
            let init = thread.init().await;
            let set = thread
                .set_active_dataset(&Dataset::from_bytes(cfg.tlvs()))
                .await;
            let up = thread.up().await;
            outf(format_args!(
                "ot: autostart: init {init:?}, dataset {set:?}, up {up:?}\r\n"
            ))
            .await;
            wpan::RANGE_MODE.store(true, Ordering::Relaxed);
        }
    }

    let mut ticker = Ticker::every(Duration::from_secs(3));
    loop {
        let line = match select(CMD.receive(), ticker.next()).await {
            Either::First(line) => line,
            Either::Second(()) => {
                if wpan::RANGE_MODE.load(Ordering::Relaxed) {
                    range_tick(thread, &mut prefix).await;
                }
                continue;
            }
        };
        let mut words = line.split_whitespace();
        match (words.next(), words.next()) {
            (Some("wpan"), Some("snapshot")) => print_snapshot().await,
            (Some("wpan"), Some("sem5")) => {
                match words.next() {
                    Some("release") => wpan::release_clk48_semaphore(),
                    Some("lock") => {
                        wpan::lock_clk48_semaphore();
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
                let (status, ready) = wpan::reinit(sys).await;
                outf(format_args!(
                    "wpan: REINIT -> {status:?}, SEV sent; ready event: {ready:?}\r\n"
                ))
                .await;
            }
            (Some("wpan"), _) => print_info(sys).await,
            (Some("fus"), Some("state")) => {
                let (state, err) = sys.shci_c2_fus_get_state().await;
                outf(format_args!(
                    "fus: state 0x{state:02x} error 0x{err:02x}\r\n"
                ))
                .await;
                if state == 0x00 && dfu::is_fus_busy() {
                    dfu::fus_busy(false);
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
                dfu::fus_busy(true);
                let r = sys.shci_c2_fus_fwupgrade(src, dst).await;
                outf(format_args!("fus: fw_upgrade -> {r:?}\r\n")).await;
            }
            (Some("fus"), Some("start")) => {
                dfu::fus_busy(true);
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
            (Some("ot"), sub) => ot_command(thread, flash, sys, &line, sub, words).await,
            _ => out("wpan: unknown command\r\n").await,
        }
    }
}

/// The `ot ...` command surface over [`Thread`].
async fn ot_command<'a>(
    thread: &mut Thread<'_>,
    flash: &mut Flash<'_, Blocking>,
    sys: &mut Sys<'_>,
    line: &str,
    sub: Option<&str>,
    mut words: core::str::SplitWhitespace<'a>,
) {
    match sub {
        Some("init") => outf(format_args!("ot: init -> {:?}\r\n", thread.init().await)).await,
        Some("new") => {
            outf(format_args!(
                "ot: create new network -> {:?}\r\n",
                thread.create_new_network().await
            ))
            .await
        }
        Some("up") => outf(format_args!("ot: up -> {:?}\r\n", thread.up().await)).await,
        Some("down") => outf(format_args!("ot: down -> {:?}\r\n", thread.down().await)).await,
        Some("tlvs") => match thread.active_dataset().await {
            Ok(d) => {
                outf(format_args!(
                    "ot: active tlvs -> ok ({} bytes)\r\n",
                    d.as_bytes().len()
                ))
                .await;
                let mut l: heapless::String<512> = heapless::String::new();
                let _ = l.push_str("ottlvs ");
                for b in d.as_bytes() {
                    let _ = write!(l, "{b:02x}");
                }
                let _ = l.push_str("\r\n");
                out(&l).await;
            }
            Err(e) => outf(format_args!("ot: active tlvs -> {e}\r\n")).await,
        },
        Some("settlvs") => {
            let hex = words.next().unwrap_or("").as_bytes();
            let mut buf = [0u8; 254];
            let mut n = 0;
            let mut i = 0;
            while i + 1 < hex.len() && n < buf.len() {
                match (hex_nibble(hex[i]), hex_nibble(hex[i + 1])) {
                    (Some(a), Some(b)) => buf[n] = (a << 4) | b,
                    _ => break,
                }
                n += 1;
                i += 2;
            }
            let r = thread
                .set_active_dataset(&Dataset::from_bytes(&buf[..n]))
                .await;
            outf(format_args!("ot: set active tlvs ({n} bytes) -> {r:?}\r\n")).await;
        }
        Some("info") | Some("role") => {
            let mut l: heapless::String<224> = heapless::String::new();
            let _ = write!(
                l,
                "ot: role {} rloc16 0x{:04x} channel {} panid 0x{:04x} commissioned {}",
                thread.role().await,
                thread.rloc16().await,
                thread.channel().await,
                thread.pan_id().await,
                thread.is_commissioned().await as u8
            );
            match thread.mesh_local_eid().await {
                Some(a) => {
                    let _ = write!(l, " mleid {a}");
                }
                None => {
                    let _ = l.push_str(" mleid ?");
                }
            }
            let _ = l.push_str("\r\n");
            out(&l).await;
        }
        Some("neighbors") => {
            let neighbors = thread.neighbors::<8>().await;
            for n in &neighbors {
                outf(format_args!(
                    "ot: neighbor rloc16 0x{:04x} ext {} age {}s lqi {} rssi avg {} last {} dBm margin {} dB {} {}\r\n",
                    n.rloc16(),
                    n.ext_address(),
                    n.age_secs(),
                    n.link_quality_in(),
                    n.average_rssi(),
                    n.last_rssi(),
                    n.link_margin(),
                    if n.is_full_thread_device() { "ftd" } else { "mtd" },
                    if n.is_child() { "child" } else { "router" },
                ))
                .await;
            }
            outf(format_args!("ot: {} neighbor(s)\r\n", neighbors.len())).await;
        }
        Some("parent") => match thread.parent_rssi().await {
            Some((avg, last)) => {
                outf(format_args!(
                    "ot: parent rssi avg {avg} dBm last {last} dBm\r\n"
                ))
                .await
            }
            None => out("ot: no parent\r\n").await,
        },
        Some("txpower") => {
            if let Some(v) = words.next().and_then(|v| v.parse::<i8>().ok()) {
                let r = thread.set_tx_power(v).await;
                outf(format_args!("ot: set tx power {v} dBm -> {r:?}\r\n")).await;
            }
            outf(format_args!(
                "ot: tx power {:?} dBm\r\n",
                thread.tx_power().await
            ))
            .await;
        }
        Some("ping") => {
            let Some(dst) = words.next().and_then(Ip6Address::parse) else {
                out("ot: ping <ipv6> [count]\r\n").await;
                return;
            };
            let mut cfg = PingConfig::new(dst);
            cfg.count = words.next().and_then(|c| c.parse().ok()).unwrap_or(3);
            outf(format_args!(
                "ot: ping {}x -> {:?}\r\n",
                cfg.count,
                thread.ping(&cfg).await
            ))
            .await;
        }
        Some("save") => match thread.active_dataset().await {
            Ok(d) => {
                let cfg =
                    persistent_config::Config::new(d.as_bytes(), persistent_config::FLAG_AUTOSTART);
                let r = persistent_config::save(flash, sys, &cfg).await;
                outf(format_args!(
                    "ot: saved {} byte dataset with autostart -> {r:?}\r\n",
                    d.as_bytes().len()
                ))
                .await;
            }
            Err(e) => outf(format_args!("ot: no active dataset ({e})\r\n")).await,
        },
        Some("keepalive") => {
            let on = words.next() != Some("off");
            match persistent_config::load() {
                Some(mut c) => {
                    if on {
                        c.flags |= persistent_config::FLAG_KEEPALIVE;
                    } else {
                        c.flags &= !persistent_config::FLAG_KEEPALIVE;
                    }
                    let r = persistent_config::save(flash, sys, &c).await;
                    outf(format_args!(
                        "ot: keepalive {} (relay load) -> {r:?}\r\n",
                        if on { "on" } else { "off" }
                    ))
                    .await;
                }
                None => out("ot: no config stored; run `ot save` first\r\n").await,
            }
        }
        Some("forget") => {
            let r = persistent_config::erase(flash, sys).await;
            wpan::RANGE_MODE.store(false, Ordering::Relaxed);
            outf(format_args!("ot: config erased -> {r:?}\r\n")).await;
        }
        Some("config") => match persistent_config::load() {
            Some(c) => {
                let mut l: heapless::String<224> = heapless::String::new();
                let _ = write!(
                    l,
                    "ot: config: {} byte dataset, autostart {}, keepalive {}, mesh-local prefix ",
                    c.tlvs().len(),
                    c.autostart(),
                    c.keepalive()
                );
                match c.mesh_local_prefix() {
                    Some(p) => {
                        let mut a = [0u8; 16];
                        a[..8].copy_from_slice(&p);
                        let _ = write!(l, "{}", Ip6Address(a));
                    }
                    None => {
                        let _ = l.push_str("?");
                    }
                }
                let _ = l.push_str("\r\n");
                out(&l).await;
            }
            None => out("ot: no config stored\r\n").await,
        },
        Some("range") => {
            let on = words.next() != Some("off");
            wpan::RANGE_MODE.store(on, Ordering::Relaxed);
            outf(format_args!(
                "ot: range mode {}\r\n",
                if on { "on" } else { "off" }
            ))
            .await;
        }
        _ => {
            let _ = line;
            out("ot: init | new | tlvs | settlvs <hex> | up | down | info | neighbors | parent | ping <ipv6> [n] | save | forget | config | range [off] | keepalive [off] | txpower [dBm]\r\n").await
        }
    }
}

fn hex_nibble(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// One range-probe step: ping the leader anycast address and report the link.
async fn range_tick(thread: &mut Thread<'_>, prefix: &mut Option<[u8; 8]>) {
    let role = thread.role().await;
    if !role.is_attached() {
        wpan::LINK.store(0, Ordering::Relaxed);
        outf(format_args!("range: role {role} (not attached)\r\n")).await;
        return;
    }
    if prefix.is_none() {
        if let Some(eid) = thread.mesh_local_eid().await {
            let mut p = [0u8; 8];
            p.copy_from_slice(&eid.0[..8]);
            *prefix = Some(p);
        }
    }
    let Some(p) = *prefix else {
        return;
    };

    let before = REPLIES.load(Ordering::Relaxed);
    let mut cfg = PingConfig::new(Ip6Address::leader_aloc(&p));
    cfg.timeout_ms = 900;
    let sent = thread.ping(&cfg).await;
    Timer::after_millis(1200).await;
    let got = REPLIES.load(Ordering::Relaxed) != before;
    wpan::LINK.store(if got { 2 } else { 1 }, Ordering::Relaxed);

    let mut l: heapless::String<224> = heapless::String::new();
    let _ = write!(l, "range: role {role}");
    if role == Role::Child {
        if let Some((avg, last)) = thread.parent_rssi().await {
            let _ = write!(l, " parent rssi {avg}/{last} dBm");
        }
    } else {
        let _ = l.push_str(" neighbors");
        let neighbors = thread.neighbors::<8>().await;
        if neighbors.is_empty() {
            let _ = l.push_str(" none");
        }
        for n in &neighbors {
            let _ = write!(
                l,
                " 0x{:04x} {}/{} dBm",
                n.rloc16(),
                n.average_rssi(),
                n.last_rssi()
            );
        }
    }
    match (sent, got) {
        (Err(e), _) => {
            let _ = write!(l, " | ping -> {e}");
        }
        (Ok(()), true) => {
            let _ = write!(l, " | leader ping {} ms", LAST_RTT.load(Ordering::Relaxed));
        }
        (Ok(()), false) => {
            let _ = l.push_str(" | leader ping: no reply");
        }
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
        match Event::decode(notif_rx.receive().await) {
            Event::StateChanged(flags) => {
                outf(format_args!("ot: state changed {flags}\r\n")).await;
            }
            Event::PingReply(r) => {
                REPLIES.fetch_add(1, Ordering::Relaxed);
                LAST_RTT.store(r.round_trip_ms() as u32, Ordering::Relaxed);
                wpan::PULSE.store(true, Ordering::Relaxed);
                outf(format_args!(
                    "ot: ping reply from {}: {} bytes seq {} hop {} rtt {} ms\r\n",
                    r.sender(),
                    r.size(),
                    r.sequence(),
                    r.hop_limit(),
                    r.round_trip_ms()
                ))
                .await;
            }
            Event::PingStatistics(s) => {
                outf(format_args!(
                    "ot: ping done: {} sent, {} received, rtt min {} max {} total {} ms\r\n",
                    s.sent(),
                    s.received(),
                    s.min_round_trip_ms(),
                    s.max_round_trip_ms(),
                    s.total_round_trip_ms()
                ))
                .await;
            }
            Event::Other { id, data } => {
                outf(format_args!(
                    "ot: notification {id} data {:08x} {:08x}\r\n",
                    data[0], data[1]
                ))
                .await;
            }
        }
    }
}
