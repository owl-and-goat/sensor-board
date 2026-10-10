//! Sensor boards on USB, and the RPC calls to them.

use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use postcard_rpc::{
    Endpoint,
    header::VarSeqKind,
    host_client::{HostClient, HostErr, MultiSubscription},
    standard_icd::{ERROR_PATH, WireError},
};
use protocol::{
    Addresses, ApplyUpdate, BeginInstall, BeginUpdate, BoardConfig, BoardDiscovered, BoardId,
    BoardInfo, ConfigError, ConfigFor, CoprocessorResult, CoprocessorStatus, Dataset,
    DiscoverBoards, EnterBootloader, FinishInstall, FinishUpdate, FirmwareStatus, GetBoardConfig,
    GetBoardInfo, GetCoprocessorStatus, GetFirmwareStatus, GetMetrics, GetNetworkAddresses,
    GetNetworkDataset, GetNetworkNeighbors, GetNetworkRouters, GetNetworkStatus, GetOfferProgress,
    ImageChunk, ImageSize, JoinNetwork, LeaveNetwork, MeasureTempRh, Member, NeighborTable,
    NetworkError, NetworkStatus, OfferProgress, ReadSensorValue, Report, ReportReceived,
    RouterTable, Sensor, SensorReadReq, SensorReadResult, SetBoardConfig, StartCollecting,
    StartOffering, StopCollecting, StopOffering, TempRhReq, TempRhResult, USB_PID, USB_VID,
    UninstallStack, UpdateImage, UpdateResult, WriteInstall, WriteUpdate,
};

/// Timeout for a call to a board. The slowest call is a join or leave that
/// hits the firmware's own five-second timeout on its radio coprocessor.
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

    /// The board whose serial number starts with `serial`, or with `None`
    /// the only attached board.
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

        // postcard-rpc uses a vendor-class interface.
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

    /// The board's ID, parsed from its USB serial number.
    pub fn id(&self) -> Result<BoardId> {
        let id = self.serial.parse();
        id.map_err(|_| {
            anyhow!(
                "board {}: its serial number is not a valid board ID",
                self.serial
            )
        })
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

    /// Tell the board to start joining a network. [`Board::network_status`]
    /// shows when it has attached.
    pub async fn join(&self, dataset: &Dataset) -> Result<()> {
        let result = self.call::<JoinNetwork>(dataset).await?;
        self.network_result(result)
    }

    /// Tell the board to leave its network and erase the stored dataset.
    pub async fn leave(&self) -> Result<()> {
        let result = self.call::<LeaveNetwork>(&()).await?;
        self.network_result(result)
    }

    /// The board's children, and the routers it has a direct radio link with,
    /// other than its parent.
    pub async fn neighbors(&self) -> Result<NeighborTable> {
        let result = self.call::<GetNetworkNeighbors>(&()).await?;
        self.network_result(result)
    }

    /// Every router on the board's network, and the board's route to it.
    pub async fn routers(&self) -> Result<RouterTable> {
        let result = self.call::<GetNetworkRouters>(&()).await?;
        self.network_result(result)
    }

    /// Subscribe to the reports that the board forwards. It forwards none
    /// until it is told to collect.
    pub async fn reports(&self) -> Result<MultiSubscription<Report>> {
        let subscription = self.client.subscribe_multi::<ReportReceived>(64).await;
        subscription.map_err(|_| anyhow!("board {} is gone", self.serial))
    }

    /// Tell the board to receive the reports that boards on its network
    /// send, including its own, and forward them.
    pub async fn start_collecting(&self) -> Result<()> {
        let result = self.call::<StartCollecting>(&()).await?;
        self.network_result(result)
    }

    pub async fn stop_collecting(&self) -> Result<()> {
        let result = self.call::<StopCollecting>(&()).await?;
        self.network_result(result)
    }

    /// The board's IPv6 unicast addresses.
    pub async fn addresses(&self) -> Result<Addresses> {
        let result = self.call::<GetNetworkAddresses>(&()).await?;
        self.network_result(result)
    }

    /// Subscribe to the discovery replies that the board forwards: the
    /// replies to the requests that [`Board::discover`] sends.
    pub async fn discovered(&self) -> Result<MultiSubscription<Member>> {
        let subscription = self.client.subscribe_multi::<BoardDiscovered>(64).await;
        subscription.map_err(|_| anyhow!("board {} is gone", self.serial))
    }

    /// Tell the board to multicast a discovery request to its network.
    pub async fn discover(&self) -> Result<()> {
        let result = self.call::<DiscoverBoards>(&()).await?;
        self.network_result(result)
    }

    /// The board's current metrics, in Prometheus's text format.
    pub async fn metrics(&self) -> Result<String> {
        let mut text = String::new();
        loop {
            let offset = text.len() as u32;
            let chunk = self.call::<GetMetrics>(&offset).await?;
            text += &chunk.text;
            if !chunk.more {
                return Ok(text);
            }
        }
    }

    fn network_result<T>(&self, result: Result<T, NetworkError>) -> Result<T> {
        result.map_err(|e| anyhow!("board {}: {e}", self.serial))
    }

    pub async fn coprocessor_status(&self) -> Result<CoprocessorStatus> {
        self.call::<GetCoprocessorStatus>(&()).await
    }

    /// Tell the board to prepare for a coprocessor image of this size.
    pub async fn begin_install(&self, size: ImageSize) -> Result<()> {
        let result = self.call::<BeginInstall>(&size).await?;
        self.coprocessor_result(result)
    }

    /// Send the board the next chunk of the image.
    pub async fn write_install(&self, chunk: &ImageChunk) -> Result<()> {
        let result = self.call::<WriteInstall>(chunk).await?;
        self.coprocessor_result(result)
    }

    /// Tell the board to hand the image to FUS. The outer error means the
    /// board did not respond, which happens when FUS resets it before the
    /// response is sent.
    pub async fn finish_install(&self) -> Result<CoprocessorResult> {
        self.call::<FinishInstall>(&()).await
    }

    /// Tell the board to start removing its wireless stack.
    pub async fn uninstall_stack(&self) -> Result<()> {
        let result = self.call::<UninstallStack>(&()).await?;
        self.coprocessor_result(result)
    }

    pub async fn read_sensor(&self, sensor: Sensor) -> Result<SensorReadResult> {
        self.call::<ReadSensorValue>(&SensorReadReq { sensor })
            .await
    }

    pub async fn measure_temp_rh(&self, req: TempRhReq) -> Result<TempRhResult> {
        self.call::<MeasureTempRh>(&req).await
    }

    pub async fn firmware_status(&self) -> Result<FirmwareStatus> {
        self.call::<GetFirmwareStatus>(&()).await
    }

    /// Tell the board to start receiving a firmware image. This discards
    /// any image it had staged or was receiving.
    pub async fn begin_update(&self, image: &UpdateImage) -> Result<()> {
        let result = self.call::<BeginUpdate>(image).await?;
        self.update_result(result)
    }

    /// Send the board the next chunk of that image.
    pub async fn write_update(&self, chunk: &ImageChunk) -> Result<()> {
        let result = self.call::<WriteUpdate>(chunk).await?;
        self.update_result(result)
    }

    /// Tell the board to verify the image it has received. On success the
    /// image is staged.
    pub async fn finish_update(&self) -> Result<()> {
        let result = self.call::<FinishUpdate>(&()).await?;
        self.update_result(result)
    }

    /// Tell the board to restart into its staged image. It drops off USB
    /// while its bootloader swaps the two images.
    pub async fn apply_update(&self) -> Result<()> {
        let result = self.call::<ApplyUpdate>(&()).await?;
        self.update_result(result)
    }

    /// Tell the board to offer its staged image to the boards on its
    /// network, which fetch it and restart into it.
    pub async fn start_offering(&self) -> Result<()> {
        let result = self.call::<StartOffering>(&()).await?;
        self.update_result(result)
    }

    pub async fn stop_offering(&self) -> Result<()> {
        let result = self.call::<StopOffering>(&()).await?;
        self.update_result(result)
    }

    /// The progress of the boards that are fetching the image this board
    /// offers. `None` from a board whose firmware predates the endpoint.
    pub async fn offer_progress(&self) -> Result<Option<OfferProgress>> {
        let answer = self.client.send_resp::<GetOfferProgress>(&());
        match tokio::time::timeout(TIMEOUT, answer).await {
            Ok(Ok(progress)) => Ok(Some(progress)),
            Ok(Err(HostErr::Wire(WireError::UnknownKey))) => Ok(None),
            Ok(Err(e)) => bail!("board {}: could not tell: {e:?}", self.serial),
            Err(_) => bail!("board {} did not answer", self.serial),
        }
    }

    fn update_result(&self, result: UpdateResult) -> Result<()> {
        result.map_err(|e| anyhow!("board {}: {e}", self.serial))
    }

    /// Get the configuration of `board`. If `board` is not this board, this
    /// board asks it over the network.
    pub async fn board_config(&self, board: BoardId) -> Result<Option<BoardConfig>> {
        let result = self.call::<GetBoardConfig>(&board).await?;
        result.map_err(|e| self.config_error(board, e))
    }

    /// Store `config` on `board`. If `board` is not this board, this board
    /// forwards the configuration over the network.
    pub async fn set_board_config(&self, board: BoardId, config: &BoardConfig) -> Result<()> {
        let config = ConfigFor {
            board,
            config: config.clone(),
        };
        let result = self.call::<SetBoardConfig>(&config).await?;
        result.map_err(|e| self.config_error(board, e))
    }

    fn config_error(&self, board: BoardId, e: ConfigError) -> anyhow::Error {
        if self.id().is_ok_and(|this| this == board) {
            anyhow!("board {board}: {e}")
        } else {
            anyhow!("board {board}, through board {}: {e}", self.serial)
        }
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
            // An endpoint's key covers its path and its types, so the
            // firmware was built from another version of `protocol`.
            Ok(Err(HostErr::Wire(WireError::UnknownKey))) => bail!(
                "board {}: its firmware does not know {} as this CLI does; flash a build that \
                 matches",
                self.serial,
                E::PATH
            ),
            Ok(Err(e)) => bail!("board {}: {} failed: {e:?}", self.serial, E::PATH),
            // No response at all, or a response of a type this CLI does not
            // expect. A firmware built from another version of `protocol`
            // sends the latter when only the response type has changed.
            Err(_) => bail!(
                "board {} did not answer {}; its firmware may not match this CLI",
                self.serial,
                E::PATH
            ),
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
