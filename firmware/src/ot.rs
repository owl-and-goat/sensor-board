//! OpenThread control through ST's API mirror on CPU2 (see `otids.rs`),
//! enough to form a network on two boards and prove the radio path:
//! `ot init | new | tlvs | settlvs <hex> | up | down | info | neighbors |
//! ping <ipv6>`.

use core::ptr::{addr_of, addr_of_mut, read_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32_wpan::sub::sys::Sys;
use embassy_stm32_wpan::sub::thread::{OtNotification, ThreadOt};

use crate::config::{self, Config};
use crate::otids::*;
use crate::wpan::{out, outf};

/// Ping replies seen so far, and the last round-trip time (ms).
pub static REPLIES: AtomicU32 = AtomicU32::new(0);
pub static LAST_RTT: AtomicU32 = AtomicU32::new(0);

/// Buffers CPU2 reads and writes directly. They live in SRAM1 like any other
/// static; CPU2 can reach all of it. Sized generously versus the C structs.
#[repr(C, align(8))]
struct Buf<const N: usize>([u8; N]);

static mut DATASET: Buf<256> = Buf([0; 256]); // otOperationalDataset (~120 B)
static mut TLVS: Buf<256> = Buf([0; 256]); // otOperationalDatasetTlvs: 254 B TLVs + length
static mut PING: Buf<64> = Buf([0; 64]); // otPingSenderConfig (57 B)
static mut NEIGHBOR: Buf<48> = Buf([0; 48]); // otNeighborInfo (40 B)
static mut NEIGHBOR_ITER: Buf<4> = Buf([0; 4]); // otNeighborInfoIterator (i16)

fn addr<const N: usize>(b: *mut Buf<N>) -> u32 {
    b as u32
}

pub fn role_name(r: u32) -> &'static str {
    match r {
        0 => "disabled",
        1 => "detached",
        2 => "child",
        3 => "router",
        4 => "leader",
        _ => "?",
    }
}

pub fn err_name(e: u32) -> &'static str {
    match e {
        0 => "ok",
        1 => "failed",
        2 => "drop",
        3 => "no-bufs",
        4 => "no-route",
        5 => "busy",
        6 => "parse",
        7 => "invalid-args",
        8 => "security",
        11 => "abort",
        12 => "not-implemented",
        13 => "invalid-state",
        14 => "no-ack",
        16 => "detached",
        23 => "not-found",
        24 => "already",
        _ => "error",
    }
}

fn fmt_ip6(a: &[u8; 16], l: &mut heapless::String<224>) {
    use core::fmt::Write as _;
    for i in 0..8 {
        if i > 0 {
            let _ = l.push(':');
        }
        let _ = write!(l, "{:x}", u16::from_be_bytes([a[2 * i], a[2 * i + 1]]));
    }
}

/// Minimal IPv6 text parser: hextets, one `::`, no embedded IPv4.
fn parse_ip6(s: &str) -> Option<[u8; 16]> {
    let mut head: heapless::Vec<u16, 8> = heapless::Vec::new();
    let mut tail: heapless::Vec<u16, 8> = heapless::Vec::new();
    let (h, t) = match s.find("::") {
        Some(i) => (&s[..i], Some(&s[i + 2..])),
        None => (s, None),
    };
    for part in h.split(':').filter(|p| !p.is_empty()) {
        head.push(u16::from_str_radix(part, 16).ok()?).ok()?;
    }
    if let Some(t) = t {
        for part in t.split(':').filter(|p| !p.is_empty()) {
            tail.push(u16::from_str_radix(part, 16).ok()?).ok()?;
        }
    } else if head.len() != 8 {
        return None;
    }
    if head.len() + tail.len() > 8 {
        return None;
    }
    let mut w = [0u16; 8];
    w[..head.len()].copy_from_slice(&head);
    w[8 - tail.len()..].copy_from_slice(&tail);
    let mut a = [0u8; 16];
    for i in 0..8 {
        a[2 * i..2 * i + 2].copy_from_slice(&w[i].to_be_bytes());
    }
    Some(a)
}

fn hex_nibble(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// True when `p` points into SRAM CPU1 may read (SRAM1, SRAM2a, SRAM2b).
fn readable(p: u32) -> bool {
    (0x2000_0000..0x2004_0000).contains(&p)
}

// ---- helpers shared by the console commands and the autostart / range loop ----

pub async fn init(ot: &mut ThreadOt<'_>) -> (u32, u32) {
    let inst = ot.call(MSG_M4TOM0_OT_INSTANCE_INIT_SINGLE, &[]).await;
    let r = ot.call(MSG_M4TOM0_OT_SET_STATE_CHANGED_CALLBACK, &[0]).await;
    (inst, r)
}

pub async fn role(ot: &mut ThreadOt<'_>) -> u32 {
    ot.call(MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE, &[]).await
}

pub async fn set_active_tlvs(ot: &mut ThreadOt<'_>, tlvs: &[u8]) -> u32 {
    let mut buf = [0u8; 256];
    let n = tlvs.len().min(254);
    buf[..n].copy_from_slice(&tlvs[..n]);
    buf[254] = n as u8;
    unsafe { core::ptr::write_volatile(addr_of_mut!(TLVS), Buf(buf)) };
    ot.call(MSG_M4TOM0_OT_DATASET_SET_ACTIVE_TLVS, &[addr(addr_of_mut!(TLVS))]).await
}

/// Active dataset TLVs from CPU2, or the OT error.
pub async fn get_active_tlvs(ot: &mut ThreadOt<'_>) -> Result<heapless::Vec<u8, 254>, u32> {
    let r = ot.call(MSG_M4TOM0_OT_DATASET_GET_ACTIVE_TLVS, &[addr(addr_of_mut!(TLVS))]).await;
    if r != 0 {
        return Err(r);
    }
    let buf = unsafe { read_volatile(addr_of!(TLVS)) }.0;
    let len = (buf[254] as usize).min(254);
    let mut v = heapless::Vec::new();
    let _ = v.extend_from_slice(&buf[..len]);
    Ok(v)
}

pub async fn up(ot: &mut ThreadOt<'_>) -> (u32, u32) {
    let r1 = ot.call(MSG_M4TOM0_OT_IP6_SET_ENABLED, &[1]).await;
    let r2 = ot.call(MSG_M4TOM0_OT_THREAD_SET_ENABLED, &[1]).await;
    (r1, r2)
}

pub async fn down(ot: &mut ThreadOt<'_>) -> (u32, u32) {
    let r1 = ot.call(MSG_M4TOM0_OT_THREAD_SET_ENABLED, &[0]).await;
    let r2 = ot.call(MSG_M4TOM0_OT_IP6_SET_ENABLED, &[0]).await;
    (r1, r2)
}

/// Send `count` pings; replies and statistics come back as notifications.
pub async fn ping(ot: &mut ThreadOt<'_>, dst: &[u8; 16], count: u16, timeout_ms: u16) -> u32 {
    // otPingSenderConfig: source[16] dest[16] replyCb u32 statsCb u32
    // context u32 size u16 count u16 interval u32 timeout u16 hopLimit u8
    // allowZeroHopLimit u8 multicastLoop u8. CPU2 forwards the callbacks as
    // notifications when the pointers are non-null.
    let mut c = [0u8; 64];
    c[16..32].copy_from_slice(dst);
    c[32..36].copy_from_slice(&1u32.to_le_bytes());
    c[36..40].copy_from_slice(&1u32.to_le_bytes());
    c[46..48].copy_from_slice(&count.to_le_bytes());
    c[48..52].copy_from_slice(&1000u32.to_le_bytes());
    c[52..54].copy_from_slice(&timeout_ms.to_le_bytes());
    unsafe { core::ptr::write_volatile(addr_of_mut!(PING), Buf(c)) };
    ot.call(MSG_M4TOM0_OT_PING_SENDER_PING, &[addr(addr_of_mut!(PING))]).await
}

/// (average, last) RSSI of the parent link in dBm, for a child.
pub async fn parent_rssi(ot: &mut ThreadOt<'_>) -> Option<(i8, i8)> {
    unsafe { core::ptr::write_volatile(addr_of_mut!(NEIGHBOR_ITER), Buf([0; 4])) };
    let p = addr(addr_of_mut!(NEIGHBOR_ITER));
    let r1 = ot.call(MSG_M4TOM0_OT_THREAD_GET_PARENT_AVERAGE_RSSI, &[p]).await;
    let avg = unsafe { read_volatile(addr_of!(NEIGHBOR_ITER)) }.0[0] as i8;
    let r2 = ot.call(MSG_M4TOM0_OT_THREAD_GET_PARENT_LAST_RSSI, &[p]).await;
    let last = unsafe { read_volatile(addr_of!(NEIGHBOR_ITER)) }.0[0] as i8;
    (r1 == 0 && r2 == 0).then_some((avg, last))
}

/// Append "rloc16 avg/last dBm" for every neighbor.
pub async fn neighbors_summary(ot: &mut ThreadOt<'_>, l: &mut heapless::String<224>) -> usize {
    use core::fmt::Write as _;
    unsafe { core::ptr::write_volatile(addr_of_mut!(NEIGHBOR_ITER), Buf([0; 4])) };
    let it = addr(addr_of_mut!(NEIGHBOR_ITER));
    let ni = addr(addr_of_mut!(NEIGHBOR));
    let mut count = 0;
    while count < 8 && ot.call(MSG_M4TOM0_OT_THREAD_GET_NEXT_NEIGHBOR_INFO, &[it, ni]).await == 0 {
        let n = unsafe { read_volatile(addr_of!(NEIGHBOR)) }.0;
        let rloc = u16::from_le_bytes([n[16], n[17]]);
        let _ = write!(l, " 0x{rloc:04x} {}/{} dBm", n[29] as i8, n[30] as i8);
        count += 1;
    }
    count
}

/// The leader's anycast address: mesh-local prefix + ::ff:fe00:fc00.
pub fn leader_aloc(prefix: &[u8; 8]) -> [u8; 16] {
    let mut a = [0u8; 16];
    a[..8].copy_from_slice(prefix);
    a[8..].copy_from_slice(&[0, 0, 0, 0xff, 0xfe, 0, 0xfc, 0]);
    a
}

/// Bring the stack up with a stored dataset. Returns the join attempt status.
pub async fn autostart(ot: &mut ThreadOt<'_>, cfg: &Config) {
    let (_, cb) = init(ot).await;
    let r = set_active_tlvs(ot, cfg.tlvs()).await;
    let (r1, r2) = up(ot).await;
    outf(format_args!(
        "ot: autostart: callback {}, dataset {}, ip6 {}, thread {}\r\n",
        err_name(cb),
        err_name(r),
        err_name(r1),
        err_name(r2)
    ))
    .await;
}

pub async fn command(line: &str, ot: &mut ThreadOt<'_>, sys: &mut Sys<'_>, flash: &mut Flash<'_, Blocking>) {
    let mut words = line.split_whitespace();
    let _ = words.next(); // "ot"
    match words.next() {
        Some("init") => {
            let (inst, r) = init(ot).await;
            outf(format_args!("ot: instance 0x{inst:08x}, state callback -> {}\r\n", err_name(r))).await;
        }
        Some("save") => {
            // Store the active dataset with autostart so the board rejoins by
            // itself after a power cycle.
            match get_active_tlvs(ot).await {
                Ok(tlvs) => {
                    let cfg = Config::new(&tlvs, config::FLAG_AUTOSTART);
                    let r = config::save(flash, sys, &cfg).await;
                    outf(format_args!("ot: saved {} byte dataset with autostart -> {r:?}\r\n", tlvs.len())).await;
                }
                Err(e) => outf(format_args!("ot: no active dataset ({})\r\n", err_name(e))).await,
            }
        }
        Some("keepalive") => {
            let on = words.next() != Some("off");
            match config::load() {
                Some(mut c) => {
                    if on {
                        c.flags |= config::FLAG_KEEPALIVE;
                    } else {
                        c.flags &= !config::FLAG_KEEPALIVE;
                    }
                    let r = config::save(flash, sys, &c).await;
                    crate::wpan::RELAY_ON.store(on, Ordering::Relaxed);
                    outf(format_args!("ot: keepalive {} (relay load) -> {r:?}\r\n", if on { "on" } else { "off" })).await;
                }
                None => out("ot: no config stored; run `ot save` first\r\n").await,
            }
        }
        Some("txpower") => {
            // otPlatRadioSetTransmitPower / GetTransmitPower, in dBm. The
            // STM32WB55 radio goes from -40 to +6 dBm in 1 dB steps.
            if let Some(v) = words.next().and_then(|v| v.parse::<i8>().ok()) {
                let r = ot.call(MSG_M4TOM0_OT_RADIO_SET_TRANSMIT_POWER, &[v as u32]).await;
                outf(format_args!("ot: set tx power {v} dBm -> {}\r\n", err_name(r))).await;
            }
            unsafe { core::ptr::write_volatile(addr_of_mut!(NEIGHBOR_ITER), Buf([0x7f; 4])) };
            let r = ot.call(MSG_M4TOM0_OT_RADIO_GET_TRANSMIT_POWER, &[addr(addr_of_mut!(NEIGHBOR_ITER))]).await;
            let p = unsafe { read_volatile(addr_of!(NEIGHBOR_ITER)) }.0[0] as i8;
            outf(format_args!("ot: tx power {p} dBm ({})\r\n", err_name(r))).await;
        }
        Some("forget") => {
            let r = config::erase(flash, sys).await;
            crate::wpan::RANGE_MODE.store(false, Ordering::Relaxed);
            outf(format_args!("ot: config erased -> {r:?}\r\n")).await;
        }
        Some("config") => match config::load() {
            Some(c) => {
                let mut l: heapless::String<224> = heapless::String::new();
                use core::fmt::Write as _;
                let _ = write!(l, "ot: config: {} byte dataset, autostart {}, keepalive {}, mesh-local prefix ", c.tlvs().len(), c.autostart(), c.keepalive());
                match c.mesh_local_prefix() {
                    Some(p) => {
                        let mut a = [0u8; 16];
                        a[..8].copy_from_slice(&p);
                        fmt_ip6(&a, &mut l);
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
            crate::wpan::RANGE_MODE.store(on, Ordering::Relaxed);
            outf(format_args!("ot: range mode {}\r\n", if on { "on" } else { "off" })).await;
        }
        Some("new") => {
            let d = addr(addr_of_mut!(DATASET));
            let r1 = ot.call(MSG_M4TOM0_OT_DATASET_CREATE_NEW_NETWORK, &[d]).await;
            let r2 = ot.call(MSG_M4TOM0_OT_DATASET_SET_ACTIVE, &[d]).await;
            outf(format_args!("ot: create new network -> {}, set active -> {}\r\n", err_name(r1), err_name(r2))).await;
        }
        Some("tlvs") => match get_active_tlvs(ot).await {
            Ok(tlvs) => {
                outf(format_args!("ot: active tlvs -> ok ({} bytes)\r\n", tlvs.len())).await;
                let mut l: heapless::String<512> = heapless::String::new();
                use core::fmt::Write as _;
                let _ = l.push_str("ottlvs ");
                for b in &tlvs {
                    let _ = write!(l, "{b:02x}");
                }
                let _ = l.push_str("\r\n");
                out(&l).await;
            }
            Err(e) => outf(format_args!("ot: active tlvs -> {}\r\n", err_name(e))).await,
        },
        Some("settlvs") => {
            let hex = words.next().unwrap_or("").as_bytes();
            let mut buf = [0u8; 254];
            let mut n = 0usize;
            let mut i = 0;
            while i + 1 < hex.len() && n < 254 {
                match (hex_nibble(hex[i]), hex_nibble(hex[i + 1])) {
                    (Some(a), Some(b)) => buf[n] = (a << 4) | b,
                    _ => break,
                }
                n += 1;
                i += 2;
            }
            let r = set_active_tlvs(ot, &buf[..n]).await;
            outf(format_args!("ot: set active tlvs ({n} bytes) -> {}\r\n", err_name(r))).await;
        }
        Some("up") => {
            let (r1, r2) = up(ot).await;
            outf(format_args!("ot: ip6 up -> {}, thread start -> {}\r\n", err_name(r1), err_name(r2))).await;
        }
        Some("down") => {
            let (r1, r2) = down(ot).await;
            outf(format_args!("ot: thread stop -> {}, ip6 down -> {}\r\n", err_name(r1), err_name(r2))).await;
        }
        Some("info") | Some("role") => {
            let role = ot.call(MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE, &[]).await;
            let rloc = ot.call(MSG_M4TOM0_OT_THREAD_GET_RLOC_16, &[]).await;
            let ch = ot.call(MSG_M4TOM0_OT_LINK_GET_CHANNEL, &[]).await;
            let pan = ot.call(MSG_M4TOM0_OT_LINK_GET_PANID, &[]).await;
            let comm = ot.call(MSG_M4TOM0_OT_DATASET_IS_COMMISSIONED, &[]).await;
            let eid = ot.call(MSG_M4TOM0_OT_THREAD_GET_MESH_LOCAL_EID, &[]).await;
            let mut l: heapless::String<224> = heapless::String::new();
            use core::fmt::Write as _;
            let _ = write!(
                l,
                "ot: role {} rloc16 0x{:04x} channel {} panid 0x{:04x} commissioned {} mleid ",
                role_name(role),
                rloc & 0xffff,
                ch & 0xff,
                pan & 0xffff,
                comm & 1
            );
            if readable(eid) {
                let a = unsafe { read_volatile(eid as *const [u8; 16]) };
                fmt_ip6(&a, &mut l);
            } else {
                let _ = write!(l, "@0x{eid:08x}");
            }
            let _ = l.push_str("\r\n");
            out(&l).await;
        }
        Some("neighbors") => {
            unsafe { core::ptr::write_volatile(addr_of_mut!(NEIGHBOR_ITER), Buf([0; 4])) };
            let it = addr(addr_of_mut!(NEIGHBOR_ITER));
            let ni = addr(addr_of_mut!(NEIGHBOR));
            let mut count = 0;
            loop {
                let r = ot.call(MSG_M4TOM0_OT_THREAD_GET_NEXT_NEIGHBOR_INFO, &[it, ni]).await;
                if r != 0 {
                    break;
                }
                let n = unsafe { read_volatile(addr_of!(NEIGHBOR)) }.0;
                let rloc = u16::from_le_bytes([n[16], n[17]]);
                let age = u32::from_le_bytes([n[8], n[9], n[10], n[11]]);
                let lqi = n[28];
                let avg = n[29] as i8;
                let last = n[30] as i8;
                let margin = n[31];
                let flags = n[38];
                let mut l: heapless::String<224> = heapless::String::new();
                use core::fmt::Write as _;
                let _ = write!(
                    l,
                    "ot: neighbor rloc16 0x{rloc:04x} ext {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} age {age}s lqi {lqi} rssi avg {avg} last {last} dBm margin {margin} dB {}{}\r\n",
                    n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7],
                    if flags & 0x02 != 0 { "ftd " } else { "mtd " },
                    if flags & 0x08 != 0 { "child" } else { "router" }
                );
                out(&l).await;
                count += 1;
                if count > 16 {
                    break;
                }
            }
            outf(format_args!("ot: {count} neighbor(s)\r\n")).await;
        }
        Some("parent") => match parent_rssi(ot).await {
            Some((avg, last)) => outf(format_args!("ot: parent rssi avg {avg} dBm last {last} dBm\r\n")).await,
            None => out("ot: no parent\r\n").await,
        },
        Some("ping") => {
            let Some(dst) = words.next().and_then(parse_ip6) else {
                out("ot: ping <ipv6>\r\n").await;
                return;
            };
            let count: u16 = words.next().and_then(|c| c.parse().ok()).unwrap_or(3);
            let r = ping(ot, &dst, count, 0).await;
            outf(format_args!("ot: ping {count}x -> {}\r\n", err_name(r))).await;
        }
        _ => out("ot: init | new | tlvs | settlvs <hex> | up | down | info | neighbors | parent | ping <ipv6> [n] | save | forget | config | range [off] | keepalive [off] | txpower [dBm]\r\n").await,
    }
}

/// Decode and print a CPU2 -> CPU1 callback.
pub async fn notification(n: OtNotification) {
    match n.id {
        MSG_M0TOM4_NOTIFY_STATE_CHANGE => {
            let f = n.data[0];
            outf(format_args!(
                "ot: state changed 0x{f:08x}{}{}{}{}\r\n",
                if f & 0x0001 != 0 { " ip6-addr" } else { "" },
                if f & 0x0004 != 0 { " role" } else { "" },
                if f & 0x0200 != 0 { " netdata" } else { "" },
                if f & 0x0040 != 0 { " child" } else { "" },
            ))
            .await;
        }
        MSG_M0TOM4_PING_SENDER_REPLY_CALLBACK => {
            let p = n.data[0];
            if readable(p) {
                let r = unsafe { read_volatile(p as *const [u8; 24]) };
                let mut a = [0u8; 16];
                a.copy_from_slice(&r[..16]);
                let rtt = u16::from_le_bytes([r[16], r[17]]);
                let size = u16::from_le_bytes([r[18], r[19]]);
                let seq = u16::from_le_bytes([r[20], r[21]]);
                let hop = r[22];
                REPLIES.fetch_add(1, Ordering::Relaxed);
                LAST_RTT.store(rtt as u32, Ordering::Relaxed);
                crate::wpan::PULSE.store(true, Ordering::Relaxed);
                let mut l: heapless::String<224> = heapless::String::new();
                use core::fmt::Write as _;
                let _ = l.push_str("ot: ping reply from ");
                fmt_ip6(&a, &mut l);
                let _ = write!(l, ": {size} bytes seq {seq} hop {hop} rtt {rtt} ms\r\n");
                out(&l).await;
            } else {
                outf(format_args!("ot: ping reply @0x{p:08x}\r\n")).await;
            }
        }
        MSG_M0TOM4_PING_SENDER_STATISTICS_CALLBACK => {
            let p = n.data[0];
            if readable(p) {
                let s = unsafe { read_volatile(p as *const [u8; 16]) };
                let sent = u16::from_le_bytes([s[0], s[1]]);
                let recv = u16::from_le_bytes([s[2], s[3]]);
                let total = u32::from_le_bytes([s[4], s[5], s[6], s[7]]);
                let min = u16::from_le_bytes([s[8], s[9]]);
                let max = u16::from_le_bytes([s[10], s[11]]);
                outf(format_args!(
                    "ot: ping done: {sent} sent, {recv} received, rtt min {min} max {max} total {total} ms\r\n"
                ))
                .await;
            }
        }
        _ => outf(format_args!("ot: notification {} size {} data {:08x} {:08x}\r\n", n.id, n.size, n.data[0], n.data[1])).await,
    }
}
