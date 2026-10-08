//! The communication protocol between the USB host and the sensor-board.

#![cfg_attr(not(feature = "std"), no_std)]

mod board_config;

use core::{fmt, net::Ipv6Addr, str::FromStr};

use postcard_rpc::{Key, Topic, TopicDirection, endpoints, topics};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub use board_config::{BoardConfig, PowerMode, Sensor, SensorConfig, SensorsConfig};

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
    | GetFirmwareStatus    | ()            | FirmwareStatus    | "firmware/status"            |
    | BeginUpdate          | UpdateImage   | UpdateResult      | "firmware/update/begin"      |
    | WriteUpdate          | ImageChunk    | UpdateResult      | "firmware/update/write"      |
    | FinishUpdate         | ()            | UpdateResult      | "firmware/update/finish"     |
    | ApplyUpdate          | ()            | UpdateResult      | "firmware/update/apply"      |
    | StartOffering        | ()            | UpdateResult      | "firmware/offer/start"       |
    | StopOffering         | ()            | UpdateResult      | "firmware/offer/stop"        |
    | GetOfferProgress     | ()            | OfferProgress     | "firmware/offer/progress"    |
    | GetBoardConfig       | BoardId       | BoardConfigResult | "config/get"                 |
    | SetBoardConfig       | ConfigFor     | ConfigResult      | "config/set"                 |
    | GetNetworkAddresses  | ()            | AddressesResult   | "network/addresses"          |
    | GetMetrics           | u32           | MetricsChunk      | "metrics/get"                |
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

/// The dataset a board has stored: the network it is configured for and
/// rejoins at every power-up. `None` if it has none.
pub type StoredDataset = Option<Dataset>;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardInfo {
    /// When the running firmware was built, in the build machine's local
    /// time. It shows which image is running after a reflash.
    pub firmware_built: heapless::String<24>,
}

/// Identifies a build of the firmware: its build time as a Unix timestamp.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BuildId(pub u32);

impl fmt::Display for BuildId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// A marker embedded in every firmware image. It identifies an image file
/// as this firmware, and gives its build.
#[repr(C)]
pub struct BuildMarker {
    magic: [u8; 16],
    /// The [`BuildId`], least significant byte first.
    build: [u8; 4],
    /// The bitwise complement of `build`. An image contains the sixteen
    /// bytes of `magic` a second time, as the constant that the firmware
    /// searches for markers with. This check rejects that copy, because the
    /// bytes after it are not a build and its complement.
    check: [u8; 4],
}

impl BuildMarker {
    const MAGIC: [u8; 16] = *b"sensor-board-fw\0";

    /// The length of a marker in bytes.
    pub const LEN: usize = size_of::<BuildMarker>();

    pub const fn new(build: BuildId) -> BuildMarker {
        BuildMarker {
            magic: Self::MAGIC,
            build: build.0.to_le_bytes(),
            check: (!build.0).to_le_bytes(),
        }
    }

    pub const fn build(&self) -> BuildId {
        BuildId(u32::from_le_bytes(self.build))
    }

    /// The build of the firmware in `image`. `None` if `image` holds no
    /// marker.
    pub fn find(image: &[u8]) -> Option<BuildId> {
        image.windows(Self::LEN).find_map(|candidate| {
            let after_magic = candidate.strip_prefix(&Self::MAGIC)?;
            let (build, check) = after_magic.split_first_chunk()?;
            let build = u32::from_le_bytes(*build);
            let check = u32::from_le_bytes(*check.first_chunk()?);
            (check == !build).then_some(BuildId(build))
        })
    }
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
    /// The radio coprocessor failed to start its Thread stack.
    StartFailed,
    /// The radio coprocessor did not complete the request in time.
    Unresponsive,
    /// The OpenThread stack returned an error. After a failed join, the
    /// board is back on its previous network.
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
/// radio link with. While the board is a child, the table does not include
/// its parent, because the stack tracks that link separately.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NeighborTable {
    pub neighbors: heapless::Vec<Neighbor, { NeighborTable::MAX_LEN }>,
    /// Whether the board has more neighbors than fit here.
    pub truncated: bool,
}

impl NeighborTable {
    /// The most neighbors that are certain to fit in one message. The
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
    /// The average received signal strength, in dBm.
    pub average_rssi: i8,
    /// The received signal strength of the last frame, in dBm.
    pub last_rssi: i8,
    /// The link margin: the received signal strength above the noise floor,
    /// in dB.
    pub link_margin: u8,
}

/// A neighbor's relationship to the board.
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
        // Use `pad` so that the name can be aligned in a table.
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

/// A board's router table: every router on its network, in order of ID,
/// with the board's route to it. Empty while the board is not attached to a
/// network.
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
    /// The number of IDs, and so the most routers a table can list.
    pub const COUNT: usize = Self::MAX as usize + 1;

    /// Every ID, in order.
    pub fn all() -> impl Iterator<Item = RouterId> {
        (0..=Self::MAX).map(RouterId)
    }

    /// The RLOC16 of the router with this ID.
    pub fn rloc16(self) -> u16 {
        (self.0 as u16) << 10
    }

    /// The ID of the router that an RLOC16 belongs to. The RLOC16 can be the
    /// router's own or one of its children's.
    pub fn of_rloc16(rloc16: u16) -> RouterId {
        RouterId((rloc16 >> 10) as u8)
    }
}

/// A board's route to a router. A `cost` covers the whole path: each link on
/// it adds 1 if it is good, 2 if it is medium and 4 if it is poor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Route {
    /// The router is the board itself.
    ThisBoard,
    /// The board sends directly to the router over their radio link.
    Direct { cost: u8 },
    /// The board sends to another router, which forwards the message.
    Relayed { next_hop: RouterId, cost: u8 },
    /// The board has no route to the router.
    Unreachable,
}

/// What the radio coprocessor (CPU2) is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoprocessorStatus {
    /// It is starting, or restarting into its other firmware.
    Starting,
    /// Its wireless stack: the normal state of a board that is set up.
    Stack(CoprocessorFirmware),
    /// It runs FUS, ST's firmware upgrade service, which installs images. A
    /// coprocessor with no wireless stack always runs FUS.
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
    /// Thread FTD (full Thread device), the only stack this firmware can use.
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

/// A version of coprocessor firmware, in the numbering of ST's release
/// notes.
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
    /// Installing an image, or about to start the installed wireless stack.
    Busy,
    /// The last install failed. FUS is ready for another.
    Failed(FusError),
}

/// Why an install failed (`FUS_STATE_ERROR` codes, AN5185).
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
pub struct SensorReadReq {
    pub sensor: Sensor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorValue {
    pub value: u32,
}

impl From<u32> for SensorValue {
    fn from(value: u32) -> Self {
        Self { value }
    }
}

impl From<u16> for SensorValue {
    fn from(value: u16) -> Self {
        Self {
            value: value.into(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SensorReadError {
    /// The sensor did not initialize.
    #[error("Sensor Didn't Initialize at Board Start")]
    NotInitialized,
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
    /// The device at the sensor's address is not that sensor.
    #[error("Wrong Product ID")]
    WrongProductId,
    /// No conversion has finished since the sensor was enabled, or the last
    /// one produced invalid data.
    #[error("Color Sensor Data Invalid")]
    DataInvalid,
    /// The light on the channel is outside of the range it can measure.
    #[error("Color Sensor Channel Out of Range")]
    OutOfRange,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseBoardIdError;

impl fmt::Display for ParseBoardIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a board's serial number is 24 hex digits")
    }
}

impl core::error::Error for ParseBoardIdError {}

impl FromStr for BoardId {
    type Err = ParseBoardIdError;

    fn from_str(s: &str) -> Result<BoardId, ParseBoardIdError> {
        let mut id = [0; 12];
        if s.len() != 2 * id.len() || !s.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(ParseBoardIdError);
        }
        for (i, byte) in id.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).map_err(|_| ParseBoardIdError)?;
        }
        Ok(BoardId(id))
    }
}

/// A board's periodic report of its readings. Every board multicasts one to
/// its network at intervals. A board that has been told to collect
/// ([`StartCollecting`]) forwards the reports it receives, including its
/// own, to the host ([`ReportReceived`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Report {
    pub board: BoardId,
    /// The firmware the board runs.
    pub firmware: BuildId,
    /// Starts at 0 when the board starts, and increases by one with each
    /// report. A gap shows the collector that a report was lost, and a reset
    /// to 0 that the board restarted.
    pub sequence: u32,
    pub readings: Readings,
}

/// The readings in a [`Report`]: one for every sensor that has a driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Readings {
    /// The four channels of the capacitance sensor.
    pub capacitance: [SensorReadResult; 4],
    /// The color sensor's channels: red, green, blue, white, infrared.
    pub color: [SensorReadResult; 5],
}

impl Report {
    /// The longest encoded report that boards send each other.
    pub const MAX_LEN: usize = 256;

    /// Encode the report as boards send it to each other: the key of
    /// [`ReportReceived`], which is derived from the layout of this type,
    /// then the postcard encoding. `None` if `buf` is too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        encode_with_prefix(&Self::key(), self, buf)
    }

    /// Decode a report. `None` if `bytes` was not encoded with this layout,
    /// so a report from a firmware with a different layout is rejected
    /// instead of misread.
    pub fn decode(bytes: &[u8]) -> Option<Report> {
        decode_with_prefix(&Self::key(), bytes)
    }

    fn key() -> [u8; 8] {
        <ReportReceived as Topic>::TOPIC_KEY.to_bytes()
    }
}

/// Write `prefix`, then the postcard encoding of `value`, into `buf`. The
/// prefix identifies the type and layout of `value` to the receiver. Returns
/// the bytes written, or `None` if `buf` is too short.
fn encode_with_prefix<'a>(
    prefix: &[u8],
    value: &impl Serialize,
    buf: &'a mut [u8],
) -> Option<&'a [u8]> {
    let (head, body) = buf.split_at_mut_checked(prefix.len())?;
    head.copy_from_slice(prefix);
    let body = postcard_rpc::postcard::to_slice(value, body).ok()?.len();
    Some(&buf[..prefix.len() + body])
}

/// Decode the output of [`encode_with_prefix`]. `None` if `bytes` does not start
/// with `prefix`, or if what follows is not a valid `T`.
fn decode_with_prefix<T: serde::de::DeserializeOwned>(prefix: &[u8], bytes: &[u8]) -> Option<T> {
    let body = bytes.strip_prefix(prefix)?;
    postcard_rpc::postcard::from_bytes(body).ok()
}

/// The result of a coprocessor request. `Ok` from [`FinishInstall`] or
/// [`UninstallStack`] means that the operation has started, not that it has
/// finished.
pub type CoprocessorResult = Result<(), CoprocessorError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoprocessorError {
    /// The coprocessor has not started yet.
    NotReady,
    /// The coprocessor is running its wireless stack, and only FUS can
    /// install images.
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
    /// FUS rejected the upgrade command.
    Rejected,
    /// The coprocessor did not complete the request in time.
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

/// The result of a request about a firmware update. `Ok` from [`ApplyUpdate`]
/// means that the board is about to restart into the update.
pub type UpdateResult = Result<(), UpdateError>;

/// A firmware image to update a board to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UpdateImage {
    pub build: BuildId,
    pub size: ImageSize,
    pub digest: ImageDigest,
}

/// The SHA-256 digest of an image. A board checks a received image against
/// it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImageDigest(pub [u8; 32]);

/// The firmware a board runs, and its update status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FirmwareStatus {
    pub build: BuildId,
    pub update: UpdateStatus,
}

/// The state of firmware updates on a board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UpdateStatus {
    /// The board is still starting its radio coprocessor.
    Starting,
    /// The board cannot take an update. Its radio coprocessor runs no
    /// wireless stack, and flash writes depend on one.
    Unavailable,
    /// The running firmware is confirmed, and the board can take an update.
    Settled,
    /// An update is being received. `received` bytes of it have arrived.
    Receiving { image: UpdateImage, received: u32 },
    /// An update has been received in full and verified. [`ApplyUpdate`]
    /// switches to it.
    Staged(UpdateImage),
    /// The running firmware is an update that is not confirmed yet. It is
    /// confirmed once the board is back on its network. If the board
    /// restarts before that, or takes too long, the previous firmware is
    /// restored.
    OnTrial,
    /// The last update failed its trial, and the previous firmware has been
    /// restored. The board can take another update.
    RolledBack,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UpdateError {
    /// See [`UpdateStatus::Unavailable`], or the board is still starting.
    Unavailable,
    /// The board takes no update while one is on trial.
    OnTrial,
    /// The image is larger than the staging area allows.
    DoesNotFit,
    /// A chunk that does not continue the image where the last one stopped,
    /// a finish before the image is complete, or an apply with nothing
    /// staged.
    OutOfSequence,
    /// The image could not be written to flash.
    Flash,
    /// The received image does not match the announced digest or build.
    DigestMismatch,
    /// The board did not complete the request in time.
    Unresponsive,
}

impl fmt::Display for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            UpdateError::Unavailable => "the board cannot take an update as it is",
            UpdateError::OnTrial => "the board is still trying out its last update",
            UpdateError::DoesNotFit => "the image is too long",
            UpdateError::OutOfSequence => "the update arrived out of order",
            UpdateError::Flash => "the update could not be written to flash",
            UpdateError::DigestMismatch => "the update arrived damaged",
            UpdateError::Unresponsive => "the board did not get to the update in time",
        })
    }
}

/// The progress of the boards that are fetching the image a board offers,
/// judging by the chunks they have requested.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OfferProgress {
    /// The boards that have requested a chunk in the last 30 seconds.
    pub fetchers: heapless::Vec<Fetcher, { OfferProgress::MAX_FETCHERS }>,
}

impl OfferProgress {
    pub const MAX_FETCHERS: usize = 8;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fetcher {
    /// The board's ID, if it has sent it. A board whose firmware predates
    /// [`UpdateMessage::Fetching`] does not send it.
    pub board: Option<BoardId>,
    /// How many bytes of the image it has.
    pub received: u32,
}

/// The messages boards exchange over the network for firmware updates. A
/// board that has an image staged and has been told to offer it
/// ([`StartOffering`]) multicasts an offer at intervals. A board that runs
/// another build requests the image chunk by chunk, stages it, and restarts
/// into it.
///
/// The encoding of this type must never change: a board on an old firmware
/// has to understand the offer of the firmware that replaces it. So do not
/// change or reorder the variants or the types in them. A new variant can be
/// added at the end. A board that does not know it fails to decode the
/// message and ignores it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum UpdateMessage {
    Offer(UpdateImage),
    /// Asks the board that offers `build` for the chunk that starts at
    /// `offset`.
    ChunkRequest {
        build: BuildId,
        offset: u32,
    },
    /// The reply to a `ChunkRequest`. Every chunk but the last is
    /// [`UpdateMessage::CHUNK_LEN`] bytes.
    Chunk {
        build: BuildId,
        offset: u32,
        data: heapless::Vec<u8, { UpdateMessage::CHUNK_LEN }>,
    },
    /// Sent by a board that starts to fetch an image, to tell the offering
    /// board its ID. This variant was added after the first three, so a
    /// board whose firmware only has those neither sends nor decodes it.
    Fetching {
        board: BoardId,
    },
}

impl UpdateMessage {
    /// A whole number of flash words, chosen so that a message is not much
    /// longer than two radio frames.
    pub const CHUNK_LEN: usize = 192;

    /// The longest encoded message.
    pub const MAX_LEN: usize = 256;

    /// The prefix of every message. It distinguishes update messages from
    /// other data on their port, and this encoding from any later one.
    const MAGIC: [u8; 4] = *b"SBU1";

    /// `None` if `buf` is too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        encode_with_prefix(&Self::MAGIC, self, buf)
    }

    pub fn decode(bytes: &[u8]) -> Option<UpdateMessage> {
        decode_with_prefix(&Self::MAGIC, bytes)
    }
}

/// The length of an image about to be installed, in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImageSize(pub u32);

/// A chunk of an image. Every chunk but the last is [`ImageChunk::MAX_LEN`]
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

/// A board's configuration. `None` if it has none stored, or if its firmware
/// cannot decode the stored one.
pub type BoardConfigResult = Result<Option<BoardConfig>, ConfigError>;

/// The result of [`SetBoardConfig`].
pub type ConfigResult = Result<(), ConfigError>;

/// A configuration and the board to store it on. If that is not the board
/// that receives the request, it forwards the configuration over the network.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ConfigFor {
    pub board: BoardId,
    pub config: BoardConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The board's radio coprocessor runs no wireless stack, which flash
    /// writes depend on, or the board is still starting.
    Unavailable,
    /// The target board could not write the configuration to flash.
    Storage,
    /// The board could not forward the request over the network.
    Network(NetworkError),
    /// The target board did not reply to the forwarded request.
    NoAnswer,
    /// The board did not complete the request in time.
    Unresponsive,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Unavailable => {
                f.write_str("the board cannot store a configuration without a wireless stack")
            }
            ConfigError::Storage => f.write_str("the configuration could not be written to flash"),
            ConfigError::Network(e) => {
                write!(f, "could not send the request over the network: {e}")
            }
            ConfigError::NoAnswer => f.write_str("the board did not reply over the network"),
            ConfigError::Unresponsive => {
                f.write_str("the board did not complete the request in time")
            }
        }
    }
}

/// The messages boards exchange over the network to read and set each
/// other's configurations. The attached board uses them to forward
/// [`GetBoardConfig`] and [`SetBoardConfig`] requests for other boards.
///
/// An encoded message starts with a key derived from the layout of this
/// type, which includes [`BoardConfig`]. Boards whose firmwares disagree on
/// the layout ignore each other's messages instead of misreading them.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub enum ConfigMessage {
    /// Asks `board` for its configuration. Sent to every board, because the
    /// sender does not know `board`'s address. `question` identifies the
    /// request, and the reply repeats it.
    Get { board: BoardId, question: u32 },
    /// Asks `board` to store `config`. Sent the same way as `Get`.
    Set {
        board: BoardId,
        question: u32,
        config: BoardConfig,
    },
    /// The reply of `board` to a `Get` or a `Set`, sent to the board that
    /// asked: its current configuration. After a failed `Set`, that differs
    /// from the configuration it was sent.
    Has {
        board: BoardId,
        question: u32,
        config: Option<BoardConfig>,
    },
}

impl ConfigMessage {
    /// The longest encoded message: the key, the variant, the board, the
    /// question number, and an optional configuration.
    pub const MAX_LEN: usize = 8 + 1 + 12 + 5 + 1 + (BoardConfig::MAX_LEN - 8);

    /// `None` if `buf` is too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        encode_with_prefix(&Self::key(), self, buf)
    }

    /// `None` if `bytes` is not a message encoded with this layout.
    pub fn decode(bytes: &[u8]) -> Option<ConfigMessage> {
        decode_with_prefix(&Self::key(), bytes)
    }

    fn key() -> [u8; 8] {
        Key::for_path::<ConfigMessage>("config/message").to_bytes()
    }
}

/// The result of [`GetNetworkAddresses`].
pub type AddressesResult = Result<Addresses, NetworkError>;

/// The IPv6 unicast addresses of a board's Thread interface.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct Addresses {
    pub addresses: heapless::Vec<Address, { Addresses::MAX_LEN }>,
    /// Whether the board has more addresses than fit here.
    pub truncated: bool,
}

impl Addresses {
    pub const MAX_LEN: usize = 8;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct Address {
    pub address: Ipv6Addr,
    pub kind: AddressKind,
}

/// The kind of an address, which determines where it is reachable from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub enum AddressKind {
    /// Reachable only from devices in radio range.
    LinkLocal,
    /// Reachable from anywhere on the Thread network (the ML-EID). It does
    /// not change when the board's place in the topology does.
    MeshLocal,
    /// A routing locator (RLOC), or an anycast locator for a role such as
    /// leader. Reachable from anywhere on the Thread network, but it changes
    /// with the board's place in the topology.
    Locator,
    /// Reachable from outside the Thread network: an address in a prefix
    /// that a border router advertises.
    Routable,
}

impl fmt::Display for AddressKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad(match self {
            AddressKind::LinkLocal => "link-local",
            AddressKind::MeshLocal => "mesh-local",
            AddressKind::Locator => "locator",
            AddressKind::Routable => "routable",
        })
    }
}

/// A chunk of a board's metrics in Prometheus's text format, starting at the
/// byte offset given to [`GetMetrics`]. Each request renders the metrics
/// again, so values can change between one chunk and the next.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct MetricsChunk {
    pub text: heapless::String<{ MetricsChunk::MAX_LEN }>,
    /// Whether more text follows this chunk.
    pub more: bool,
}

impl MetricsChunk {
    pub const MAX_LEN: usize = 512;
}

#[cfg(test)]
mod tests {
    extern crate std;
    use core::net::SocketAddrV6;
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
    fn dataset_rejects_invalid_hex() {
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

    /// Every field has its longest encoding, and the table follows the
    /// longest header postcard-rpc writes: a 1-byte discriminant, an 8-byte
    /// key and a 4-byte sequence number.
    #[test]
    fn full_neighbor_table_fits_send_buffer() {
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
            firmware: BuildId(u32::MAX),
            sequence: u32::MAX,
            readings: Readings {
                capacitance: [
                    Ok(SensorValue { value: u32::MAX }),
                    Ok(SensorValue { value: 0 }),
                    Err(SensorReadError::Nack),
                    Err(SensorReadError::WatchdogTimeoutError),
                ],
                color: [
                    Ok(SensorValue {
                        value: u32::from(u16::MAX),
                    }),
                    Ok(SensorValue { value: 0 }),
                    Err(SensorReadError::DataInvalid),
                    Err(SensorReadError::OutOfRange),
                    Err(SensorReadError::Nack),
                ],
            },
        }
    }

    #[test]
    fn report_round_trips() {
        let mut buf = [0; Report::MAX_LEN];
        let encoded = report().encode(&mut buf).unwrap();
        assert_eq!(Report::decode(encoded), Some(report()));
    }

    #[test]
    fn report_with_other_layout_is_rejected() {
        let mut buf = [0; Report::MAX_LEN];
        let encoded = report().encode(&mut buf).unwrap();
        let mut other = encoded.to_vec();
        other[0] ^= 1;
        assert_eq!(Report::decode(&other), None);
        assert_eq!(Report::decode(&encoded[..encoded.len() - 1]), None);
        assert_eq!(Report::decode(&[]), None);
    }

    #[test]
    fn report_encode_fails_on_short_buffer() {
        assert_eq!(report().encode(&mut [0; 16]), None);
    }

    #[test]
    fn update_messages_round_trip() {
        let image = UpdateImage {
            build: BuildId(u32::MAX),
            size: ImageSize(u32::MAX),
            digest: ImageDigest([0xab; 32]),
        };
        let chunk = UpdateMessage::Chunk {
            build: BuildId(u32::MAX),
            offset: u32::MAX,
            data: heapless::Vec::from_slice(&[0xff; UpdateMessage::CHUNK_LEN]).unwrap(),
        };
        let request = UpdateMessage::ChunkRequest {
            build: BuildId(7),
            offset: 192,
        };
        for message in [UpdateMessage::Offer(image), request, chunk] {
            let mut buf = [0; UpdateMessage::MAX_LEN];
            let encoded = message.encode(&mut buf).unwrap();
            assert_eq!(UpdateMessage::decode(encoded), Some(message));
        }
        assert_eq!(UpdateMessage::decode(b"SBU2\x00"), None);
        assert_eq!(UpdateMessage::decode(&[]), None);
    }

    /// Pins the exact bytes, which boards on older firmware depend on.
    #[test]
    fn update_message_encoding_is_stable() {
        let mut buf = [0; UpdateMessage::MAX_LEN];
        let request = UpdateMessage::ChunkRequest {
            build: BuildId(1),
            offset: 2,
        };
        assert_eq!(request.encode(&mut buf).unwrap(), b"SBU1\x01\x01\x02");

        let offer = UpdateMessage::Offer(UpdateImage {
            build: BuildId(1),
            size: ImageSize(2),
            digest: ImageDigest([3; 32]),
        });
        let encoded = offer.encode(&mut buf).unwrap();
        assert_eq!(encoded[..7], *b"SBU1\x00\x01\x02");
        assert_eq!(encoded[7..], [3; 32]);

        let chunk = UpdateMessage::Chunk {
            build: BuildId(1),
            offset: 2,
            data: heapless::Vec::from_slice(&[9, 8]).unwrap(),
        };
        assert_eq!(
            chunk.encode(&mut buf).unwrap(),
            b"SBU1\x02\x01\x02\x02\x09\x08"
        );

        let fetching = UpdateMessage::Fetching {
            board: BoardId(*b"0123456789ab"),
        };
        assert_eq!(fetching.encode(&mut buf).unwrap(), b"SBU1\x030123456789ab");
    }

    /// A board whose firmware lacks a variant cannot decode a message of
    /// that variant.
    #[test]
    fn update_message_with_unknown_variant_is_rejected() {
        assert_eq!(UpdateMessage::decode(b"SBU1\x040123456789ab"), None);
    }

    #[test]
    fn build_marker_is_found_anywhere_in_image() {
        let marker = BuildMarker::new(BuildId(1_791_145_757));
        assert_eq!(marker.build(), BuildId(1_791_145_757));

        let mut image = std::vec![0xa5; 1000];
        // The copy of the magic that the firmware searches with, followed by
        // unrelated bytes.
        image.extend_from_slice(&marker.magic);
        image.extend_from_slice(b"src/update.rs");
        image.extend_from_slice(&marker.magic);
        image.extend_from_slice(&marker.build);
        image.extend_from_slice(&marker.check);
        image.extend_from_slice(&[0x5a; 333]);
        assert_eq!(BuildMarker::find(&image), Some(BuildId(1_791_145_757)));

        // No marker, and a marker truncated by the end of the file.
        assert_eq!(BuildMarker::find(&[0xa5; 1000]), None);
        let cut_short = image.len() - 333 - 1;
        assert_eq!(BuildMarker::find(&image[..cut_short]), None);
    }

    #[test]
    fn board_id_displays_as_usb_serial() {
        assert_eq!(report().board.to_string(), "4B0041000350475532303120");
    }

    #[test]
    fn board_id_parses_its_display_form() {
        let board = report().board;
        assert_eq!("4B0041000350475532303120".parse(), Ok(board));
        assert_eq!("4b0041000350475532303120".parse(), Ok(board));
        assert_eq!("4B00".parse::<BoardId>(), Err(ParseBoardIdError));
        assert_eq!(
            "4B004100035047553230312G".parse::<BoardId>(),
            Err(ParseBoardIdError)
        );
        assert_eq!(
            "+B0041000350475532303120".parse::<BoardId>(),
            Err(ParseBoardIdError)
        );
    }

    /// A configuration whose encoding is as long as possible.
    fn longest_config() -> BoardConfig {
        let mut sensors = SensorsConfig::default();
        for (_, sensor) in &mut sensors.0 {
            *sensor = Some(SensorConfig {
                poll_interval: core::time::Duration::MAX,
            });
        }
        BoardConfig {
            board_id: u8::MAX,
            sensor_config: sensors,
            power_mode: PowerMode::Battery,
            pushgateway: Some(SocketAddrV6::new([0xffff; 8].into(), u16::MAX, 0, 0)),
        }
    }

    #[test]
    fn config_messages_round_trip() {
        let board = report().board;
        let messages = [
            ConfigMessage::Get { board, question: 0 },
            ConfigMessage::Set {
                board,
                question: 1,
                config: longest_config(),
            },
            ConfigMessage::Has {
                board,
                question: u32::MAX,
                config: Some(longest_config()),
            },
            ConfigMessage::Has {
                board,
                question: 0,
                config: None,
            },
        ];
        for message in messages {
            let mut buf = [0; ConfigMessage::MAX_LEN];
            let encoded = message.encode(&mut buf).unwrap();
            assert_eq!(ConfigMessage::decode(encoded), Some(message));
        }
    }

    #[test]
    fn longest_config_message_has_max_len() {
        let longest = ConfigMessage::Has {
            board: report().board,
            question: u32::MAX,
            config: Some(longest_config()),
        };
        let mut buf = [0; 2 * ConfigMessage::MAX_LEN];
        let encoded = longest.encode(&mut buf).unwrap();
        assert_eq!(encoded.len(), ConfigMessage::MAX_LEN);
    }

    #[test]
    fn config_message_with_other_layout_is_rejected() {
        let message = ConfigMessage::Get {
            board: report().board,
            question: 0,
        };
        let mut buf = [0; ConfigMessage::MAX_LEN];
        let encoded = message.encode(&mut buf).unwrap();
        let mut other = encoded.to_vec();
        other[0] ^= 1;
        assert_eq!(ConfigMessage::decode(&other), None);
        assert_eq!(ConfigMessage::decode(&[]), None);
    }

    #[test]
    fn router_id_is_top_bits_of_rloc16() {
        assert_eq!(RouterId(27).rloc16(), 0x6c00);
        assert_eq!(RouterId::of_rloc16(0x6c00), RouterId(27));
        // A child of that router.
        assert_eq!(RouterId::of_rloc16(0x6c01), RouterId(27));
        assert_eq!(RouterId::all().count(), RouterId::COUNT);
        assert_eq!(RouterId::all().last(), Some(RouterId(62)));
    }

    #[test]
    fn ext_address_displays_as_hex() {
        let address = ExtAddress([0x02, 0xa1, 0, 0, 0, 0, 0x0f, 0xff]);
        assert_eq!(address.to_string(), "02a1000000000fff");
    }
}
