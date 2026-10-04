//! The communication protocol between the USB host and the sensor-board.

#![cfg_attr(not(feature = "std"), no_std)]

use core::{fmt, str::FromStr};

use postcard_rpc::{Topic, TopicDirection, endpoints, topics};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// USB IDs of a board running the firmware. This is the pid.codes test PID, which is for private
/// testing only: by its terms it must not be on a board that is redistributed, sold or
/// manufactured
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x0001;

endpoints! {
    list = ENDPOINT_LIST;
    | EndpointTy           | RequestTy     | ResponseTy        | Path                         |
    | ----------           | ---------     | ----------        | ----                         |
    | GetBoardInfo         | ()            | BoardInfo         | "board/info"                 |
    | EnterBootloader      | ()            | ()                | "board/bootloader"           |
    | GetNetworkStatus     | ()            | NetworkStatus     | "network/status"             |
    | GetNetworkDataset    | ()            | StoredDataset     | "network/dataset"            |
    | JoinNetwork          | Dataset       | NetworkResult     | "network/join"               |
    | LeaveNetwork         | ()            | NetworkResult     | "network/leave"              |
    | GetNetworkNeighbors  | ()            | NeighborsResult   | "network/neighbors"          |
    | GetNetworkRouters    | ()            | RoutersResult     | "network/routers"            |
    | GetCoprocessorStatus | ()            | CoprocessorStatus | "coprocessor/status"         |
    | BeginInstall         | ImageSize     | CoprocessorResult | "coprocessor/install/begin"  |
    | WriteInstall         | ImageChunk    | CoprocessorResult | "coprocessor/install/write"  |
    | FinishInstall        | ()            | CoprocessorResult | "coprocessor/install/finish" |
    | UninstallStack       | ()            | CoprocessorResult | "coprocessor/uninstall"      |
    | ReadSensorValue      | SensorReadReq | SensorReadResult  | "sensor/read"                |
    | StartCollecting      | ()            | NetworkResult     | "reports/collect/start"      |
    | StopCollecting       | ()            | NetworkResult     | "reports/collect/stop"       |
}

topics! {
    list = TOPICS_IN_LIST;
    direction = TopicDirection::ToServer;
    | TopicTy | MessageTy | Path |
    | ------- | --------- | ---- |
}

topics! {
    list = TOPICS_OUT_LIST;
    direction = TopicDirection::ToClient;
    | TopicTy        | MessageTy | Path               | Cfg |
    | -------        | --------- | ----               | --- |
    | ReportReceived | Report    | "reports/received" |     |
}

/// The dataset of the network a board is configured for, which it rejoins at
/// every power-up. `None` when it has none.
pub type StoredDataset = Option<Dataset>;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardInfo {
    /// When the running firmware was built, in the build machine's local
    /// time: tells images apart after a reflash.
    pub firmware_built: heapless::String<24>,
}

/// The result of [`JoinNetwork`] or [`LeaveNetwork`]. `Ok` from a join means the board has taken
/// the dataset and is looking for the network, not that it has attached. [`GetNetworkStatus`] can
/// be used to determine whether or not the device has attached to the network
pub type NetworkResult = Result<(), NetworkError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NetworkStatus {
    /// The board is still starting its radio coprocessor.
    Starting,
    /// Thread on a board is unavailable
    Unavailable(NetworkError),
    /// The board has no network to join.
    Unconfigured,
    /// The board has a network's dataset. [`Link::role`] says whether it has
    /// attached yet.
    Configured(Link),
}

/// A board's connection to the network it is configured for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Link {
    pub role: Role,
    /// The board's 16-bit routing locator. Meaningless until it has attached.
    pub rloc16: u16,
    pub channel: u8,
    pub pan_id: u16,
}

/// A Thread device role (`otDeviceRole`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Role {
    Disabled,
    Detached,
    Child,
    Router,
    Leader,
    Other(u8),
}

impl Role {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NetworkError {
    /// The radio coprocessor has no Thread stack to run.
    NoThreadStack,
    /// The radio coprocessor would not start its Thread stack.
    StartFailed,
    /// The radio coprocessor did not get the request done in time.
    Unresponsive,
    /// The OpenThread stack refused. After a refused join the board is back
    /// on the network it had before.
    Stack(OtError),
    /// The dataset could not be written to, or erased from, the board's
    /// flash.
    Storage,
}

impl From<OtError> for NetworkError {
    fn from(e: OtError) -> Self {
        NetworkError::Stack(e)
    }
}

impl fmt::Display for NetworkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NetworkError::NoThreadStack => {
                f.write_str("the radio coprocessor has no Thread stack installed")
            }
            NetworkError::StartFailed => {
                f.write_str("the radio coprocessor would not start its Thread stack")
            }
            NetworkError::Unresponsive => f.write_str("the radio coprocessor is not responding"),
            NetworkError::Stack(e) => write!(f, "the Thread stack refused: {e}"),
            NetworkError::Storage => f.write_str("could not update the dataset in flash"),
        }
    }
}

/// An `otError` from the OpenThread stack.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OtError {
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

impl fmt::Display for OtError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            OtError::Failed => "failed",
            OtError::Drop => "drop",
            OtError::NoBufs => "no-bufs",
            OtError::NoRoute => "no-route",
            OtError::Busy => "busy",
            OtError::Parse => "parse",
            OtError::InvalidArgs => "invalid-args",
            OtError::Security => "security",
            OtError::Abort => "abort",
            OtError::NotImplemented => "not-implemented",
            OtError::InvalidState => "invalid-state",
            OtError::NoAck => "no-ack",
            OtError::Detached => "detached",
            OtError::NotFound => "not-found",
            OtError::Already => "already",
            OtError::Other(n) => return write!(f, "error {n}"),
        };
        f.write_str(s)
    }
}

/// A Thread operational dataset in its TLV encoding: everything a device
/// needs to join a network, network key included. Written and parsed as hex,
/// the form the OpenThread CLI uses (`dataset active -x`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dataset(heapless::Vec<u8, { Dataset::MAX_LEN }>);

impl Dataset {
    /// `OT_OPERATIONAL_DATASET_MAX_LENGTH`
    pub const MAX_LEN: usize = 254;

    /// `None` if `tlvs` is longer than a dataset can be.
    pub fn from_tlvs(tlvs: &[u8]) -> Option<Dataset> {
        heapless::Vec::from_slice(tlvs).ok().map(Dataset)
    }

    pub fn as_tlvs(&self) -> &[u8] {
        &self.0
    }
}

impl fmt::Display for Dataset {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.iter().try_for_each(|b| write!(f, "{b:02x}"))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseDatasetError {
    NotHex,
    TooLong,
}

impl fmt::Display for ParseDatasetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ParseDatasetError::NotHex => "a dataset is an even number of hex digits",
            ParseDatasetError::TooLong => "a dataset is at most 254 bytes",
        })
    }
}

impl core::error::Error for ParseDatasetError {}

impl FromStr for Dataset {
    type Err = ParseDatasetError;

    fn from_str(s: &str) -> Result<Dataset, ParseDatasetError> {
        if !s.is_ascii() || s.len() % 2 != 0 {
            return Err(ParseDatasetError::NotHex);
        }
        let mut tlvs = heapless::Vec::new();
        for i in (0..s.len()).step_by(2) {
            let byte =
                u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| ParseDatasetError::NotHex)?;
            tlvs.push(byte).map_err(|_| ParseDatasetError::TooLong)?;
        }
        Ok(Dataset(tlvs))
    }
}

/// The result of [`GetNetworkNeighbors`].
pub type NeighborsResult = Result<NeighborTable, NetworkError>;

/// A board's neighbor table: its children, and the routers it has a direct
/// radio link with. While the board is a child, its parent is not among
/// them: the stack keeps that link somewhere else.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NeighborTable {
    pub neighbors: heapless::Vec<Neighbor, { NeighborTable::MAX_LEN }>,
    /// The board has more neighbors than these.
    pub truncated: bool,
}

impl NeighborTable {
    /// As many neighbors as one message is sure to have room for: the
    /// firmware sends from a buffer of 1024 bytes.
    pub const MAX_LEN: usize = 32;
}

/// A Thread device that a board has a direct radio link with
/// (`otNeighborInfo`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Neighbor {
    pub kind: NeighborKind,
    /// Its 16-bit routing locator.
    pub rloc16: u16,
    pub ext_address: ExtAddress,
    /// Seconds since the board last heard from it.
    pub age_secs: u32,
    /// How well the board receives it, from 0 (not at all) to 3 (best).
    pub link_quality_in: u8,
    /// The strength the board receives it at, in dBm, averaged.
    pub average_rssi: i8,
    /// The same, of the last frame alone.
    pub last_rssi: i8,
    /// How far above its noise floor the board receives it, in dB.
    pub link_margin: u8,
}

/// What a neighbor is to the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NeighborKind {
    /// A child of the board.
    Child,
    /// Another router.
    Router,
}

impl fmt::Display for NeighborKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `pad`, so that it can be lined up in a table.
        f.pad(match self {
            NeighborKind::Child => "child",
            NeighborKind::Router => "router",
        })
    }
}

/// The IEEE 802.15.4 extended address a device has on its network. Written
/// as hex.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExtAddress(pub [u8; 8]);

impl fmt::Display for ExtAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.iter().try_for_each(|b| write!(f, "{b:02x}"))
    }
}

/// The result of [`GetNetworkRouters`].
pub type RoutersResult = Result<RouterTable, NetworkError>;

/// A board's router table: every router on its network, in order of ID, with
/// what the board does to get a message to it. Empty while the board is not
/// attached to a network.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RouterTable {
    pub routers: heapless::Vec<Router, { RouterId::COUNT }>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Router {
    pub id: RouterId,
    pub route: Route,
}

/// The ID a router has on its network, 0 to 62. It is the top six bits of
/// the router's RLOC16, and of the RLOC16 of each of its children.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RouterId(pub u8);

impl RouterId {
    /// `OT_NETWORK_MAX_ROUTER_ID`
    pub const MAX: u8 = 62;
    /// How many IDs there are, and so the most routers there can be to list.
    pub const COUNT: usize = Self::MAX as usize + 1;

    /// Every ID there is, in order.
    pub fn all() -> impl Iterator<Item = RouterId> {
        (0..=Self::MAX).map(RouterId)
    }

    /// The RLOC16 of the router that has this ID.
    pub fn rloc16(self) -> u16 {
        (self.0 as u16) << 10
    }

    /// The ID of the router that has this RLOC16, or whose child has it.
    pub fn of_rloc16(rloc16: u16) -> RouterId {
        RouterId((rloc16 >> 10) as u8)
    }
}

/// What a board's Thread stack does with a message for a router. A `cost` is
/// that of the whole path: each link on it adds 1 if it is good, 2 if it is
/// middling and 4 if it is poor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Route {
    /// Nothing: the router is the board itself.
    ThisBoard,
    /// Sends it straight over the radio link the two have.
    Direct { cost: u8 },
    /// Sends it to another router to pass on: the mesh at work.
    Relayed { next_hop: RouterId, cost: u8 },
    /// The board knows of no way there.
    Unreachable,
}

/// What the radio coprocessor (CPU2) is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoprocessorStatus {
    /// It is coming up, or being restarted to run its other firmware.
    Starting,
    /// Its wireless stack: the normal state of a board that is set up.
    Stack(CoprocessorFirmware),
    /// FUS, ST's firmware upgrade service, which is what installs an image.
    /// A coprocessor with no wireless stack runs nothing else.
    Fus {
        firmware: CoprocessorFirmware,
        state: FusState,
    },
}

/// What is installed on the radio coprocessor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoprocessorFirmware {
    pub fus: Version,
    pub stack: Option<WirelessStack>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WirelessStack {
    pub kind: StackKind,
    pub version: Version,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StackKind {
    /// Thread, full Thread device: the one this firmware can drive.
    ThreadFtd,
    /// Any other stack, by ST's `INFO_STACK_TYPE` code.
    Other(u8),
}

impl fmt::Display for WirelessStack {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.kind {
            StackKind::ThreadFtd => write!(f, "Thread FTD {}", self.version),
            StackKind::Other(code) => write!(f, "stack type {code:#04x} {}", self.version),
        }
    }
}

/// A version of coprocessor firmware, numbered the way ST's release notes
/// number them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Version {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FusState {
    /// Ready for an image.
    Idle,
    /// Installing an image, or about to start the wireless stack it has.
    Busy,
    /// The last install failed. FUS is ready for another.
    Failed(FusError),
}

/// Why FUS gave up on an image (`FUS_STATE_ERROR` codes, AN5185).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FusError {
    ImageNotFound,
    ImageCorrupt,
    ImageNotAuthentic,
    NotEnoughSpace,
    Aborted,
    EraseFailed,
    WriteFailed,
    StAuthTagNotFound,
    CustomerAuthTagNotFound,
    AuthKeyLocked,
    RollbackRefused,
    Other(u8),
}

impl fmt::Display for FusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            FusError::ImageNotFound => "FUS found no image",
            FusError::ImageCorrupt => "the image is corrupt",
            FusError::ImageNotAuthentic => "the image is not signed by ST",
            FusError::NotEnoughSpace => "not enough space for the image",
            FusError::Aborted => "the install was aborted",
            FusError::EraseFailed => "FUS could not erase flash",
            FusError::WriteFailed => "FUS could not write flash",
            FusError::StAuthTagNotFound => "the image lacks ST's authentication tag",
            FusError::CustomerAuthTagNotFound => "the image lacks the customer authentication tag",
            FusError::AuthKeyLocked => "the authentication key is locked",
            FusError::RollbackRefused => "FUS will not go back to an older version",
            FusError::Other(code) => return write!(f, "FUS error {code:#04x}"),
        };
        f.write_str(s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
pub enum Sensor {
    Capacitance0,
    Capacitance1,
    Capacitance2,
    Capacitance3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorReadReq {
    pub sensor: Sensor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorValue {
    pub value: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SensorReadError {
    /// I2C Bus error
    #[error("I2C Bus Error")]
    Bus,
    /// Arbitration lost
    #[error("I2C Arbitration Lost")]
    Arbitration,
    /// ACK not received (either to the address or to a data byte)
    #[error("I2C ACK Not Received")]
    Nack,
    /// Timeout
    #[error("I2C Timeout")]
    Timeout,
    /// CRC error
    #[error("I2C CRC Error")]
    Crc,
    /// Overrun error
    #[error("I2C Buffer Overrun")]
    Overrun,
    /// Zero-length transfers are not allowed.
    #[error("Zero-Length Transfers are not allowed")]
    ZeroLengthTransfer,
    #[error("Capacitance Sensor Watchdog Timeout Error")]
    WatchdogTimeoutError,
    #[error("Capacitance Sensor Amplitude Warning")]
    AmplitudeWarning,
}

pub type SensorReadResult = Result<SensorValue, SensorReadError>;

/// A board's identity: its chip's unique ID, which is also its USB serial
/// number. Written the way that serial is, as upper-case hex.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardId(pub [u8; 12]);

impl fmt::Display for BoardId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.iter().try_for_each(|b| write!(f, "{b:02X}"))
    }
}

/// What a board says about itself. Every board sends one to all the others on
/// its network at intervals. A board that has been told to collect
/// ([`StartCollecting`]) passes those that reach it, its own among them, on to
/// the host ([`ReportReceived`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Report {
    pub board: BoardId,
    /// Counts up by one with each report, from 0 when the board starts:
    /// whoever collects them can tell that one was lost, or that the board
    /// has restarted.
    pub sequence: u32,
    pub readings: Readings,
}

/// The readings in a [`Report`]: one of every sensor that the firmware has a
/// driver for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Readings {
    /// The four channels of the capacitance sensor.
    pub capacitance: [SensorReadResult; 4],
}

impl Report {
    /// The most bytes a report may take up on its way between boards.
    pub const MAX_LEN: usize = 256;

    /// The report as boards send it to each other: the key of
    /// [`ReportReceived`], which stands for the layout of this type, and then
    /// the report in postcard's encoding. `None` if `buf` is too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        let key = Self::key();
        let (head, body) = buf.split_at_mut_checked(key.len())?;
        head.copy_from_slice(&key);
        let body = postcard_rpc::postcard::to_slice(self, body).ok()?.len();
        Some(&buf[..key.len() + body])
    }

    /// `None` for anything but a report of this very layout. A board whose
    /// firmware has another one is not understood, rather than misread.
    pub fn decode(bytes: &[u8]) -> Option<Report> {
        let body = bytes.strip_prefix(&Self::key())?;
        postcard_rpc::postcard::from_bytes(body).ok()
    }

    fn key() -> [u8; 8] {
        <ReportReceived as Topic>::TOPIC_KEY.to_bytes()
    }
}

/// How a request went. `Ok` from [`FinishInstall`] or [`UninstallStack`]
/// means that the work has begun, not that it is done.
pub type CoprocessorResult = Result<(), CoprocessorError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoprocessorError {
    /// The coprocessor has not come up yet.
    NotReady,
    /// Only FUS installs images, and the coprocessor is running its wireless
    /// stack.
    StackRunning,
    /// FUS is in the middle of an install.
    Busy,
    /// The image does not fit between the application and the coprocessor's
    /// own part of flash.
    DoesNotFit,
    /// A chunk that does not continue the image where the last one stopped,
    /// or a finish before the image is complete.
    OutOfSequence,
    /// The image could not be written to flash.
    Flash,
    /// FUS turned the upgrade command down.
    Rejected,
    /// The coprocessor did not get the request done in time.
    Unresponsive,
}

impl fmt::Display for CoprocessorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            CoprocessorError::NotReady => "the radio coprocessor has not come up yet",
            CoprocessorError::StackRunning => {
                "the radio coprocessor is running its wireless stack, not FUS"
            }
            CoprocessorError::Busy => "FUS is in the middle of an install",
            CoprocessorError::DoesNotFit => "the image does not fit in flash",
            CoprocessorError::OutOfSequence => "the image arrived out of order",
            CoprocessorError::Flash => "the image could not be written to flash",
            CoprocessorError::Rejected => "FUS turned the upgrade down",
            CoprocessorError::Unresponsive => "the radio coprocessor is not responding",
        })
    }
}

/// The length of an image about to be installed, in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImageSize(pub u32);

/// A piece of an image. Every chunk but the last is [`ImageChunk::MAX_LEN`]
/// bytes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImageChunk {
    /// Where in the image `data` goes.
    pub offset: u32,
    pub data: heapless::Vec<u8, { ImageChunk::MAX_LEN }>,
}

impl ImageChunk {
    pub const MAX_LEN: usize = 512;

    /// `None` if `data` is longer than a chunk can be.
    pub fn new(offset: u32, data: &[u8]) -> Option<ImageChunk> {
        let data = heapless::Vec::from_slice(data).ok()?;
        Some(ImageChunk { offset, data })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::string::ToString;

    use super::*;

    #[test]
    fn dataset_hex_round_trip() {
        let dataset: Dataset = "0e080000000000010000000300001a".parse().unwrap();
        assert_eq!(dataset.as_tlvs()[..4], [0x0e, 0x08, 0x00, 0x00]);
        assert_eq!(dataset.to_string(), "0e080000000000010000000300001a");
        assert_eq!(Dataset::from_tlvs(dataset.as_tlvs()), Some(dataset));
    }

    #[test]
    fn dataset_rejects_what_is_not_hex_bytes() {
        assert_eq!("0e0".parse::<Dataset>(), Err(ParseDatasetError::NotHex));
        assert_eq!("0g".parse::<Dataset>(), Err(ParseDatasetError::NotHex));
        assert_eq!("é0".parse::<Dataset>(), Err(ParseDatasetError::NotHex));
    }

    #[test]
    fn dataset_is_at_most_254_bytes() {
        assert!("00".repeat(254).parse::<Dataset>().is_ok());
        assert_eq!(
            "00".repeat(255).parse::<Dataset>(),
            Err(ParseDatasetError::TooLong)
        );
        assert_eq!(Dataset::from_tlvs(&[0; 255]), None);
    }

    /// With every field at its longest encoding, behind the longest header
    /// postcard-rpc writes: a 1-byte discriminant, an 8-byte key and a
    /// 4-byte sequence number.
    #[test]
    fn full_neighbor_table_fits_the_firmwares_send_buffer() {
        let neighbor = Neighbor {
            kind: NeighborKind::Router,
            rloc16: u16::MAX,
            ext_address: ExtAddress([0xff; 8]),
            age_secs: u32::MAX,
            link_quality_in: u8::MAX,
            average_rssi: i8::MIN,
            last_rssi: i8::MIN,
            link_margin: u8::MAX,
        };
        let mut table = NeighborTable {
            truncated: true,
            ..NeighborTable::default()
        };
        while table.neighbors.push(neighbor).is_ok() {}
        assert_eq!(table.neighbors.len(), NeighborTable::MAX_LEN);

        let result: NeighborsResult = Ok(table);
        let mut message = [0; 4096];
        let message = postcard_rpc::postcard::to_slice(&result, &mut message).unwrap();
        assert!(13 + message.len() <= 1024, "{} bytes", message.len());
    }

    fn report() -> Report {
        Report {
            board: BoardId([
                0x4b, 0x00, 0x41, 0x00, 0x03, 0x50, 0x47, 0x55, 0x32, 0x30, 0x31, 0x20,
            ]),
            sequence: u32::MAX,
            readings: Readings {
                capacitance: [
                    Ok(SensorValue { value: u32::MAX }),
                    Ok(SensorValue { value: 0 }),
                    Err(SensorReadError::Nack),
                    Err(SensorReadError::WatchdogTimeoutError),
                ],
            },
        }
    }

    #[test]
    fn report_survives_the_trip_between_boards() {
        let mut buf = [0; Report::MAX_LEN];
        let encoded = report().encode(&mut buf).unwrap();
        assert_eq!(Report::decode(encoded), Some(report()));
    }

    #[test]
    fn report_of_another_layout_is_not_understood() {
        let mut buf = [0; Report::MAX_LEN];
        let encoded = report().encode(&mut buf).unwrap();
        let mut other = encoded.to_vec();
        other[0] ^= 1;
        assert_eq!(Report::decode(&other), None);
        assert_eq!(Report::decode(&encoded[..encoded.len() - 1]), None);
        assert_eq!(Report::decode(&[]), None);
    }

    #[test]
    fn report_does_not_fit_a_buffer_that_is_too_short() {
        assert_eq!(report().encode(&mut [0; 16]), None);
    }

    #[test]
    fn board_id_is_written_like_the_usb_serial() {
        assert_eq!(report().board.to_string(), "4B0041000350475532303120");
    }

    #[test]
    fn router_id_is_the_top_of_an_rloc16() {
        assert_eq!(RouterId(27).rloc16(), 0x6c00);
        assert_eq!(RouterId::of_rloc16(0x6c00), RouterId(27));
        // A child of that router.
        assert_eq!(RouterId::of_rloc16(0x6c01), RouterId(27));
        assert_eq!(RouterId::all().count(), RouterId::COUNT);
        assert_eq!(RouterId::all().last(), Some(RouterId(62)));
    }

    #[test]
    fn ext_address_is_written_as_hex() {
        let address = ExtAddress([0x02, 0xa1, 0, 0, 0, 0, 0x0f, 0xff]);
        assert_eq!(address.to_string(), "02a1000000000fff");
    }
}
