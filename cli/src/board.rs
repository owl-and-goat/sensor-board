//! Sensor boards on USB, and the calls they answer.

use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use postcard_rpc::{
    Endpoint,
    header::VarSeqKind,
    host_client::HostClient,
    standard_icd::{ERROR_PATH, WireError},
};
use protocol::{
    BeginInstall, BoardInfo, CoprocessorResult, CoprocessorStatus, Dataset, EnterBootloader,
    FinishInstall, GetBoardInfo, GetCoprocessorStatus, GetNetworkDataset, GetNetworkStatus,
    ImageChunk, ImageSize, JoinNetwork, LeaveNetwork, NetworkResult, NetworkStatus,
    ReadSensorValue, Sensor, SensorReadReq, SensorReadResult, USB_PID, USB_VID, UninstallStack,
    WriteInstall,
};

/// How long a board gets to answer. The slowest it can be is a join or leave
/// that runs into the firmware's own five-second limit on its radio
/// coprocessor.
const TIMEOUT: Duration = Duration::from_secs(10);

pub struct Board {
    serial: String,
    client: HostClient<WireError>,
}

impl Board {
    /// Every attached board, in order of serial number.
    pub async fn all() -> Result<Vec<Board>> {
        let mut boards = Vec::new();
        for device in devices().await? {
            boards.push(Board::open(&device).await?);
        }
        if boards.is_empty() {
            bail!("no sensor board found on USB");
        }
        Ok(boards)
    }

    /// The board whose serial number starts with `serial`, or the only board
    /// attached when that is `None`.
    pub async fn select(serial: Option<&str>) -> Result<Board> {
        let prefix = serial.unwrap_or("").to_uppercase();
        let mut matching = devices().await?;
        matching.retain(|d| serial_of(d).starts_with(&prefix));

        match matching.as_slice() {
            [device] => Board::open(device).await,
            [] if serial.is_some() => {
                bail!("no attached board has a serial starting with {prefix}")
            }
            [] => bail!("no sensor board found on USB"),
            several => bail!(
                "several boards are attached; pick one with --board:{}",
                several
                    .iter()
                    .map(|d| format!("\n  {}", serial_of(d)))
                    .collect::<String>()
            ),
        }
    }

    pub async fn open(device: &nusb::DeviceInfo) -> Result<Board> {
        let serial = serial_of(device).to_owned();

        // postcard-rpc's messages run over a vendor-class interface.
        let Some(interface) = device.interfaces().position(|i| i.class() == 0xFF) else {
            bail!("the firmware on board {serial} is too old to talk to; reflash it");
        };
        if let Err(e) = device.open().await {
            if e.kind() == nusb::ErrorKind::PermissionDenied {
                bail!(
                    "no permission to open board {serial}: add the udev rule from \
                     firmware/README.org, or run as root"
                );
            }
        }

        let client = HostClient::try_from_nusb_and_interface_id(
            device,
            interface,
            ERROR_PATH,
            8,
            VarSeqKind::Seq2,
        )
        .await
        .map_err(|e| anyhow!("could not open board {serial}: {e}"))?;
        Ok(Board { serial, client })
    }

    pub fn serial(&self) -> &str {
        &self.serial
    }

    pub async fn info(&self) -> Result<BoardInfo> {
        self.call::<GetBoardInfo>(&()).await
    }

    pub async fn network_status(&self) -> Result<NetworkStatus> {
        self.call::<GetNetworkStatus>(&()).await
    }

    /// The dataset of the network the board is configured for.
    pub async fn dataset(&self) -> Result<Option<Dataset>> {
        self.call::<GetNetworkDataset>(&()).await
    }

    /// Have the board start joining a network. [`Board::network_status`]
    /// tells when it has attached.
    pub async fn join(&self, dataset: &Dataset) -> Result<()> {
        let result = self.call::<JoinNetwork>(dataset).await?;
        self.network_result(result)
    }

    /// Have the board leave its network and forget it.
    pub async fn leave(&self) -> Result<()> {
        let result = self.call::<LeaveNetwork>(&()).await?;
        self.network_result(result)
    }

    fn network_result(&self, result: NetworkResult) -> Result<()> {
        result.map_err(|e| anyhow!("board {}: {e}", self.serial))
    }

    pub async fn coprocessor_status(&self) -> Result<CoprocessorStatus> {
        self.call::<GetCoprocessorStatus>(&()).await
    }

    /// Have the board get ready for a coprocessor image of this size.
    pub async fn begin_install(&self, size: ImageSize) -> Result<()> {
        let result = self.call::<BeginInstall>(&size).await?;
        self.coprocessor_result(result)
    }

    /// Give the board the next piece of the image.
    pub async fn write_install(&self, chunk: &ImageChunk) -> Result<()> {
        let result = self.call::<WriteInstall>(chunk).await?;
        self.coprocessor_result(result)
    }

    /// Have the board hand the image to FUS. The outer error is the board
    /// not answering, which is what happens when FUS resets it before the
    /// answer is out.
    pub async fn finish_install(&self) -> Result<CoprocessorResult> {
        self.call::<FinishInstall>(&()).await
    }

    /// Have the board start removing its wireless stack.
    pub async fn uninstall_stack(&self) -> Result<()> {
        let result = self.call::<UninstallStack>(&()).await?;
        self.coprocessor_result(result)
    }

    pub async fn read_sensor(&self, sensor: Sensor) -> Result<SensorReadResult> {
        self.call::<ReadSensorValue>(&SensorReadReq { sensor })
            .await
    }

    fn coprocessor_result(&self, result: CoprocessorResult) -> Result<()> {
        result.map_err(|e| anyhow!("board {}: {e}", self.serial))
    }

    /// Reboot the board into its ROM bootloader. It drops off USB and comes
    /// back as a DFU device.
    pub async fn enter_bootloader(&self) -> Result<()> {
        self.call::<EnterBootloader>(&()).await
    }

    async fn call<E: Endpoint>(&self, request: &E::Request) -> Result<E::Response>
    where
        E::Request: serde::Serialize + postcard_schema::Schema,
        E::Response: serde::de::DeserializeOwned + postcard_schema::Schema,
    {
        match tokio::time::timeout(TIMEOUT, self.client.send_resp::<E>(request)).await {
            Ok(Ok(response)) => Ok(response),
            Ok(Err(e)) => bail!("board {}: {} failed: {e:?}", self.serial, E::PATH),
            Err(_) => bail!("board {} did not answer {}", self.serial, E::PATH),
        }
    }
}

/// The attached boards, in order of serial number.
pub async fn devices() -> Result<Vec<nusb::DeviceInfo>> {
    let mut devices: Vec<nusb::DeviceInfo> = nusb::list_devices()
        .await
        .context("could not list USB devices")?
        .filter(|d| d.vendor_id() == USB_VID && d.product_id() == USB_PID)
        .collect();
    devices.sort_by(|a, b| serial_of(a).cmp(serial_of(b)));
    Ok(devices)
}

/// A board's USB serial number, which is its chip's unique ID.
pub fn serial_of(device: &nusb::DeviceInfo) -> &str {
    device.serial_number().unwrap_or("?")
}
