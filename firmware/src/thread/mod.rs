mod ffi;
mod ot;
mod sane;

use core::fmt;
use core::mem::MaybeUninit;
use core::net::Ipv6Addr;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use embassy_stm32_wpan::sub::thread::{OtNotification, ThreadOt};

use ffi::MsgId_M0toM4_Enum_t as notif;
use ffi::MsgId_M4toM0_Enum_t as cmd;

/// An `otError` from the stack. `Ok(())` is `OT_ERROR_NONE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Failed,
    Drop,
    NoBufs,
    NoRoute,
    Busy,
    Parse,
    InvalidArgs,
    Security,
    Abort,
    NotImplemented,
    InvalidState,
    NoAck,
    Detached,
    NotFound,
    Already,
    Other(u8),
}

impl Error {
    fn check(raw: u32) -> Result<(), Error> {
        let e = ffi::otError(raw as _);
        match e {
            ffi::otError::OT_ERROR_NONE => Ok(()),
            ffi::otError::OT_ERROR_FAILED => Err(Error::Failed),
            ffi::otError::OT_ERROR_DROP => Err(Error::Drop),
            ffi::otError::OT_ERROR_NO_BUFS => Err(Error::NoBufs),
            ffi::otError::OT_ERROR_NO_ROUTE => Err(Error::NoRoute),
            ffi::otError::OT_ERROR_BUSY => Err(Error::Busy),
            ffi::otError::OT_ERROR_PARSE => Err(Error::Parse),
            ffi::otError::OT_ERROR_INVALID_ARGS => Err(Error::InvalidArgs),
            ffi::otError::OT_ERROR_SECURITY => Err(Error::Security),
            ffi::otError::OT_ERROR_ABORT => Err(Error::Abort),
            ffi::otError::OT_ERROR_NOT_IMPLEMENTED => Err(Error::NotImplemented),
            ffi::otError::OT_ERROR_INVALID_STATE => Err(Error::InvalidState),
            ffi::otError::OT_ERROR_NO_ACK => Err(Error::NoAck),
            ffi::otError::OT_ERROR_DETACHED => Err(Error::Detached),
            ffi::otError::OT_ERROR_NOT_FOUND => Err(Error::NotFound),
            ffi::otError::OT_ERROR_ALREADY => Err(Error::Already),
            other => Err(Error::Other(other.0)),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Error::Failed => "failed",
            Error::Drop => "drop",
            Error::NoBufs => "no-bufs",
            Error::NoRoute => "no-route",
            Error::Busy => "busy",
            Error::Parse => "parse",
            Error::InvalidArgs => "invalid-args",
            Error::Security => "security",
            Error::Abort => "abort",
            Error::NotImplemented => "not-implemented",
            Error::InvalidState => "invalid-state",
            Error::NoAck => "no-ack",
            Error::Detached => "detached",
            Error::NotFound => "not-found",
            Error::Already => "already",
            Error::Other(n) => return write!(f, "error {n}"),
        };
        f.write_str(s)
    }
}

impl core::error::Error for Error {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Disabled,
    Detached,
    Child,
    Router,
    Leader,
    Other(u8),
}

impl Role {
    fn from_raw(raw: u32) -> Role {
        match ffi::otDeviceRole(raw as _) {
            ffi::otDeviceRole::OT_DEVICE_ROLE_DISABLED => Role::Disabled,
            ffi::otDeviceRole::OT_DEVICE_ROLE_DETACHED => Role::Detached,
            ffi::otDeviceRole::OT_DEVICE_ROLE_CHILD => Role::Child,
            ffi::otDeviceRole::OT_DEVICE_ROLE_ROUTER => Role::Router,
            ffi::otDeviceRole::OT_DEVICE_ROLE_LEADER => Role::Leader,
            other => Role::Other(other.0),
        }
    }

    /// True once the device is part of a network.
    pub fn is_attached(self) -> bool {
        matches!(self, Role::Child | Role::Router | Role::Leader)
    }
}

impl fmt::Display for Role {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Role::Disabled => "disabled",
            Role::Detached => "detached",
            Role::Child => "child",
            Role::Router => "router",
            Role::Leader => "leader",
            Role::Other(n) => return write!(f, "role {n}"),
        };
        f.write_str(s)
    }
}

mod ipv6 {
    use super::*;

    pub fn from_ffi(a: &ffi::otIp6Address) -> Ipv6Addr {
        // SAFETY: the union's variants are all 16 bytes of the same address.
        Ipv6Addr::from_segments(unsafe { a.mFields.m16 })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ip6Address(pub [u8; 16]);

impl Ip6Address {
    /// The leader anycast address of a mesh: prefix + `::ff:fe00:fc00`.
    pub fn leader_aloc(mesh_local_prefix: &[u8; 8]) -> Ip6Address {
        let mut a = [0u8; 16];
        a[..8].copy_from_slice(mesh_local_prefix);
        a[8..].copy_from_slice(&[0, 0, 0, 0xff, 0xfe, 0, 0xfc, 0]);
        Ip6Address(a)
    }

    /// Hextets with one optional `::`. No embedded IPv4 form.
    pub fn parse(s: &str) -> Option<Ip6Address> {
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
        Some(Ip6Address(a))
    }

    fn from_ffi(a: &ffi::otIp6Address) -> Ip6Address {
        Ip6Address(unsafe { a.mFields.m8 })
    }

    fn to_ffi(self) -> ffi::otIp6Address {
        ffi::otIp6Address {
            mFields: ffi::otIp6Address__bindgen_ty_1 { m8: self.0 },
        }
    }
}

impl fmt::Display for Ip6Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..8 {
            if i > 0 {
                f.write_str(":")?;
            }
            write!(
                f,
                "{:x}",
                u16::from_be_bytes([self.0[2 * i], self.0[2 * i + 1]])
            )?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtAddress(pub [u8; 8]);

impl fmt::Display for ExtAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for b in self.0 {
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}

// --------------------------------------------------------------- dataset ---

/// A Thread operational dataset in its TLV form, which is what joins a device
/// to an existing network.
#[derive(Clone, Copy)]
pub struct Dataset(ffi::otOperationalDatasetTlvs);

impl Dataset {
    pub fn from_bytes(tlvs: &[u8]) -> Dataset {
        let mut d = ffi::otOperationalDatasetTlvs {
            mTlvs: [0; 254],
            mLength: 0,
        };
        let n = tlvs.len().min(d.mTlvs.len());
        d.mTlvs[..n].copy_from_slice(&tlvs[..n]);
        d.mLength = n as u8;
        Dataset(d)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.0.mTlvs[..(self.0.mLength as usize).min(self.0.mTlvs.len())]
    }

    /// The Mesh-Local Prefix TLV (type 7), which every address in the mesh
    /// shares and which the leader anycast address is built from.
    pub fn mesh_local_prefix(&self) -> Option<[u8; 8]> {
        let t = self.as_bytes();
        let mut i = 0;
        while i + 2 <= t.len() {
            let (ty, len) = (t[i], t[i + 1] as usize);
            if ty == 7 && len == 8 && i + 2 + 8 <= t.len() {
                let mut p = [0u8; 8];
                p.copy_from_slice(&t[i + 2..i + 10]);
                return Some(p);
            }
            i += 2 + len;
        }
        None
    }
}

// -------------------------------------------------------------- neighbors ---

#[derive(Clone, Copy)]
pub struct Neighbor(ffi::otNeighborInfo);

impl Neighbor {
    pub fn rloc16(&self) -> u16 {
        self.0.mRloc16
    }
    pub fn ext_address(&self) -> ExtAddress {
        ExtAddress(self.0.mExtAddress.m8)
    }
    pub fn age_secs(&self) -> u32 {
        self.0.mAge
    }
    pub fn link_quality_in(&self) -> u8 {
        self.0.mLinkQualityIn
    }
    pub fn average_rssi(&self) -> i8 {
        self.0.mAverageRssi
    }
    pub fn last_rssi(&self) -> i8 {
        self.0.mLastRssi
    }
    pub fn link_margin(&self) -> u8 {
        self.0.mLinkMargin
    }
    pub fn is_child(&self) -> bool {
        self.0.mIsChild()
    }
    pub fn is_full_thread_device(&self) -> bool {
        self.0.mFullThreadDevice()
    }
}

// ------------------------------------------------------------------ ping ---

#[derive(Debug, Clone, Copy)]
pub struct PingConfig {
    pub destination: Ip6Address,
    /// Zero uses the stack default.
    pub count: u16,
    pub interval_ms: u32,
    /// Time to wait for the final reply. Zero uses the stack default.
    pub timeout_ms: u16,
}

impl PingConfig {
    pub fn new(destination: Ip6Address) -> PingConfig {
        PingConfig {
            destination,
            count: 1,
            interval_ms: 1000,
            timeout_ms: 0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PingReply(ffi::otPingSenderReply);

impl PingReply {
    pub fn sender(&self) -> Ip6Address {
        Ip6Address::from_ffi(&self.0.mSenderAddress)
    }
    pub fn round_trip_ms(&self) -> u16 {
        self.0.mRoundTripTime
    }
    pub fn size(&self) -> u16 {
        self.0.mSize
    }
    pub fn sequence(&self) -> u16 {
        self.0.mSequenceNumber
    }
    pub fn hop_limit(&self) -> u8 {
        self.0.mHopLimit
    }
}

#[derive(Clone, Copy)]
pub struct PingStatistics(ffi::otPingSenderStatistics);

impl PingStatistics {
    pub fn sent(&self) -> u16 {
        self.0.mSentCount
    }
    pub fn received(&self) -> u16 {
        self.0.mReceivedCount
    }
    pub fn total_round_trip_ms(&self) -> u32 {
        self.0.mTotalRoundTripTime
    }
    pub fn min_round_trip_ms(&self) -> u16 {
        self.0.mMinRoundTripTime
    }
    pub fn max_round_trip_ms(&self) -> u16 {
        self.0.mMaxRoundTripTime
    }
}

// ---------------------------------------------------------------- events ---

/// `otChangedFlags`: what changed when the stack reports a state change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StateFlags(pub u32);

impl StateFlags {
    pub fn ip6_address_changed(self) -> bool {
        self.0 & 0x0000_0001 != 0
    }
    pub fn role_changed(self) -> bool {
        self.0 & 0x0000_0004 != 0
    }
    pub fn child_changed(self) -> bool {
        self.0 & 0x0000_0040 != 0
    }
    pub fn net_data_changed(self) -> bool {
        self.0 & 0x0000_0200 != 0
    }
}

impl fmt::Display for StateFlags {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "0x{:08x}", self.0)?;
        for (set, name) in [
            (self.ip6_address_changed(), "ip6-addr"),
            (self.role_changed(), "role"),
            (self.child_changed(), "child"),
            (self.net_data_changed(), "netdata"),
        ] {
            if set {
                write!(f, " {name}")?;
            }
        }
        Ok(())
    }
}

/// A callback from CPU2, decoded.
pub enum Event {
    StateChanged(StateFlags),
    PingReply(PingReply),
    PingStatistics(PingStatistics),
    Other { id: u32, data: [u32; 4] },
}

impl Event {
    pub fn decode(n: OtNotification) -> Event {
        match n.id as u8 {
            notif::MSG_M0TOM4_NOTIFY_STATE_CHANGE => Event::StateChanged(StateFlags(n.data[0])),
            notif::MSG_M0TOM4_PING_SENDER_REPLY_CALLBACK => {
                match read_cpu2::<ffi::otPingSenderReply>(n.data[0]) {
                    Some(r) => Event::PingReply(PingReply(r)),
                    None => Event::Other {
                        id: n.id,
                        data: n.data,
                    },
                }
            }
            notif::MSG_M0TOM4_PING_SENDER_STATISTICS_CALLBACK => {
                match read_cpu2::<ffi::otPingSenderStatistics>(n.data[0]) {
                    Some(s) => Event::PingStatistics(PingStatistics(s)),
                    None => Event::Other {
                        id: n.id,
                        data: n.data,
                    },
                }
            }
            _ => Event::Other {
                id: n.id,
                data: n.data,
            },
        }
    }
}

/// Read a struct CPU2 passed us by address, if that address is in RAM either
/// core can reach (SRAM1, SRAM2a, SRAM2b).
fn read_cpu2<T>(addr: u32) -> Option<T> {
    if !(0x2000_0000..0x2004_0000).contains(&addr) {
        return None;
    }
    // SAFETY: the address came from CPU2 naming one of its own callback
    // structs, and the layout of T is the vendor's own (see `ffi`).
    Some(unsafe { read_volatile(addr as *const T) })
}

// -------------------------------------------------------------- the stack ---

/// Buffers CPU2 reads and writes by address. One instance of [`Thread`] exists,
/// and every access goes through `&mut self`, so these are never aliased.
static mut DATASET: MaybeUninit<ffi::otOperationalDataset> = MaybeUninit::zeroed();
static mut TLVS: MaybeUninit<ffi::otOperationalDatasetTlvs> = MaybeUninit::zeroed();
static mut PING: MaybeUninit<ffi::otPingSenderConfig> = MaybeUninit::zeroed();
static mut NEIGHBOR: MaybeUninit<ffi::otNeighborInfo> = MaybeUninit::zeroed();
static mut NEIGHBOR_ITER: MaybeUninit<ffi::otNeighborInfoIterator> = MaybeUninit::zeroed();
static mut RSSI: MaybeUninit<i8> = MaybeUninit::zeroed();

/// The OpenThread stack on CPU2.
pub struct Thread<'d> {
    ot: ThreadOt<'d>,
}

impl<'d> Thread<'d> {
    pub fn new(ot: ThreadOt<'d>) -> Thread<'d> {
        Thread { ot }
    }

    async fn call(&mut self, id: cmd::Type, args: &[u32]) -> u32 {
        self.ot.call(id as u32, args).await
    }

    /// `otInstanceInitSingle` plus `otSetStateChangedCallback`, so state
    /// changes arrive as [`Event::StateChanged`].
    pub async fn init(&mut self) -> Result<(), Error> {
        self.call(cmd::MSG_M4TOM0_OT_INSTANCE_INIT_SINGLE, &[])
            .await;
        let r = self
            .call(cmd::MSG_M4TOM0_OT_SET_STATE_CHANGED_CALLBACK, &[0])
            .await;
        Error::check(r)
    }

    pub async fn set_ip6_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let r = self
            .call(cmd::MSG_M4TOM0_OT_IP6_SET_ENABLED, &[enabled as u32])
            .await;
        Error::check(r)
    }

    pub async fn set_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let r = self
            .call(cmd::MSG_M4TOM0_OT_THREAD_SET_ENABLED, &[enabled as u32])
            .await;
        Error::check(r)
    }

    /// Bring the interface and the protocol up together.
    pub async fn up(&mut self) -> Result<(), Error> {
        self.set_ip6_enabled(true).await?;
        self.set_enabled(true).await
    }

    /// Stop the protocol and take the interface down.
    pub async fn down(&mut self) -> Result<(), Error> {
        self.set_enabled(false).await?;
        self.set_ip6_enabled(false).await
    }

    pub async fn role(&mut self) -> Role {
        Role::from_raw(
            self.call(cmd::MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE, &[])
                .await,
        )
    }

    pub async fn rloc16(&mut self) -> u16 {
        self.call(cmd::MSG_M4TOM0_OT_THREAD_GET_RLOC_16, &[]).await as u16
    }

    pub async fn channel(&mut self) -> u8 {
        self.call(cmd::MSG_M4TOM0_OT_LINK_GET_CHANNEL, &[]).await as u8
    }

    pub async fn pan_id(&mut self) -> u16 {
        self.call(cmd::MSG_M4TOM0_OT_LINK_GET_PANID, &[]).await as u16
    }

    pub async fn is_commissioned(&mut self) -> bool {
        self.call(cmd::MSG_M4TOM0_OT_DATASET_IS_COMMISSIONED, &[])
            .await
            != 0
    }

    pub async fn mesh_local_eid(&mut self) -> Option<Ip6Address> {
        let p = self
            .call(cmd::MSG_M4TOM0_OT_THREAD_GET_MESH_LOCAL_EID, &[])
            .await;
        read_cpu2::<ffi::otIp6Address>(p).map(|a| Ip6Address::from_ffi(&a))
    }

    /// Generate a fresh random network and make it the active dataset.
    pub async fn create_new_network(&mut self) -> Result<(), Error> {
        let d = addr_of_mut!(DATASET) as u32;
        Error::check(
            self.call(cmd::MSG_M4TOM0_OT_DATASET_CREATE_NEW_NETWORK, &[d])
                .await,
        )?;
        Error::check(self.call(cmd::MSG_M4TOM0_OT_DATASET_SET_ACTIVE, &[d]).await)
    }

    pub async fn active_dataset(&mut self) -> Result<Dataset, Error> {
        let t = addr_of_mut!(TLVS) as u32;
        Error::check(
            self.call(cmd::MSG_M4TOM0_OT_DATASET_GET_ACTIVE_TLVS, &[t])
                .await,
        )?;
        // SAFETY: CPU2 has just filled it; layout is the vendor's.
        Ok(Dataset(unsafe {
            read_volatile(addr_of!(TLVS) as *const ffi::otOperationalDatasetTlvs)
        }))
    }

    pub async fn set_active_dataset(&mut self, dataset: &Dataset) -> Result<(), Error> {
        // SAFETY: exclusive through &mut self; CPU2 only reads it during the call.
        unsafe {
            write_volatile(
                addr_of_mut!(TLVS) as *mut ffi::otOperationalDatasetTlvs,
                dataset.0,
            )
        };
        let r = self
            .call(
                cmd::MSG_M4TOM0_OT_DATASET_SET_ACTIVE_TLVS,
                &[addr_of_mut!(TLVS) as u32],
            )
            .await;
        Error::check(r)
    }

    /// Walk the neighbour table. Stops at `N` entries.
    pub async fn neighbors<const N: usize>(&mut self) -> heapless::Vec<Neighbor, N> {
        let mut out = heapless::Vec::new();
        // SAFETY: OT_NEIGHBOR_INFO_ITERATOR_INIT is 0.
        unsafe {
            write_volatile(
                addr_of_mut!(NEIGHBOR_ITER) as *mut ffi::otNeighborInfoIterator,
                0,
            )
        };
        let (it, ni) = (
            addr_of_mut!(NEIGHBOR_ITER) as u32,
            addr_of_mut!(NEIGHBOR) as u32,
        );
        while !out.is_full() {
            if self
                .call(cmd::MSG_M4TOM0_OT_THREAD_GET_NEXT_NEIGHBOR_INFO, &[it, ni])
                .await
                != 0
            {
                break;
            }
            // SAFETY: CPU2 has just filled it; layout is the vendor's.
            let n = unsafe { read_volatile(addr_of!(NEIGHBOR) as *const ffi::otNeighborInfo) };
            let _ = out.push(Neighbor(n));
        }
        out
    }

    /// Average and last RSSI of the link to this device's parent, in dBm.
    /// `None` unless this device is a child.
    pub async fn parent_rssi(&mut self) -> Option<(i8, i8)> {
        let p = addr_of_mut!(RSSI) as u32;
        let avg_ok = self
            .call(cmd::MSG_M4TOM0_OT_THREAD_GET_PARENT_AVERAGE_RSSI, &[p])
            .await;
        let avg = unsafe { read_volatile(addr_of!(RSSI) as *const i8) };
        let last_ok = self
            .call(cmd::MSG_M4TOM0_OT_THREAD_GET_PARENT_LAST_RSSI, &[p])
            .await;
        let last = unsafe { read_volatile(addr_of!(RSSI) as *const i8) };
        (avg_ok == 0 && last_ok == 0).then_some((avg, last))
    }

    /// Start an ICMPv6 echo run. Replies and the final statistics arrive as
    /// [`Event::PingReply`] and [`Event::PingStatistics`].
    pub async fn ping(&mut self, config: &PingConfig) -> Result<(), Error> {
        // CPU2 only checks the two callback pointers for non-null before
        // forwarding them to us as notifications; neither side dereferences
        // them, since the real callback lives on this core.
        let present = unsafe { core::mem::transmute::<usize, Option<unsafe extern "C" fn()>>(1) };
        let cfg = ffi::otPingSenderConfig {
            mSource: Ip6Address([0; 16]).to_ffi(),
            mDestination: config.destination.to_ffi(),
            mReplyCallback: unsafe { core::mem::transmute(present) },
            mStatisticsCallback: unsafe { core::mem::transmute(present) },
            mCallbackContext: core::ptr::null_mut(),
            mSize: 0,
            mCount: config.count,
            mInterval: config.interval_ms,
            mTimeout: config.timeout_ms,
            mHopLimit: 0,
            mAllowZeroHopLimit: false,
            mMulticastLoop: false,
        };
        // SAFETY: exclusive through &mut self; CPU2 reads it during the call.
        unsafe { write_volatile(addr_of_mut!(PING) as *mut ffi::otPingSenderConfig, cfg) };
        let r = self
            .call(
                cmd::MSG_M4TOM0_OT_PING_SENDER_PING,
                &[addr_of_mut!(PING) as u32],
            )
            .await;
        Error::check(r)
    }

    /// Radio transmit power in dBm. The STM32WB55 radio covers -40 to +6.
    pub async fn tx_power(&mut self) -> Result<i8, Error> {
        let p = addr_of_mut!(RSSI) as u32;
        Error::check(
            self.call(cmd::MSG_M4TOM0_OT_RADIO_GET_TRANSMIT_POWER, &[p])
                .await,
        )?;
        Ok(unsafe { read_volatile(addr_of!(RSSI) as *const i8) })
    }

    pub async fn set_tx_power(&mut self, dbm: i8) -> Result<(), Error> {
        let r = self
            .call(cmd::MSG_M4TOM0_OT_RADIO_SET_TRANSMIT_POWER, &[dbm as u32])
            .await;
        Error::check(r)
    }
}
