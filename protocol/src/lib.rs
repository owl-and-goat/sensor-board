//! The communication protocol between the USB host and the sensor-board.

#![no_std]

use core::{fmt, str::FromStr};

use postcard_rpc::{endpoints, topics, TopicDirection};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

/// USB IDs of a board running the firmware. This is the pid.codes test PID, which is for private
/// testing only: by its terms it must not be on a board that is redistributed, sold or
/// manufactured
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x0001;

endpoints! {
    list = ENDPOINT_LIST;
    | EndpointTy           | RequestTy  | ResponseTy        | Path                         |
    | ----------           | ---------  | ----------        | ----                         |
    | GetBoardInfo         | ()         | BoardInfo         | "board/info"                 |
    | EnterBootloader      | ()         | ()                | "board/bootloader"           |
    | GetNetworkStatus     | ()         | NetworkStatus     | "network/status"             |
    | GetNetworkDataset    | ()         | StoredDataset     | "network/dataset"            |
    | JoinNetwork          | Dataset    | NetworkResult     | "network/join"               |
    | LeaveNetwork         | ()         | NetworkResult     | "network/leave"              |
    | GetCoprocessorStatus | ()         | CoprocessorStatus | "coprocessor/status"         |
    | BeginInstall         | ImageSize  | CoprocessorResult | "coprocessor/install/begin"  |
    | WriteInstall         | ImageChunk | CoprocessorResult | "coprocessor/install/write"  |
    | FinishInstall        | ()         | CoprocessorResult | "coprocessor/install/finish" |
    | UninstallStack       | ()         | CoprocessorResult | "coprocessor/uninstall"      |
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
    | TopicTy | MessageTy | Path | Cfg |
    | ------- | --------- | ---- | --- |
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
}
