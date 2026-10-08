//! Serves the endpoints of the `protocol` crate to the USB host. The handlers
//! only translate between the protocol and the modules that do the work.
//! Every handler that waits has a timeout, so one request cannot block the
//! next indefinitely.

use embassy_executor::Spawner;
use postcard_rpc::{
    define_dispatch,
    header::{VarHeader, VarSeq},
    server::{
        self, Dispatch,
        impls::embassy_usb_v0_6::dispatch_impl::{WireRxBuf, WireSpawnImpl},
    },
};
// `define_dispatch!` requires a spawn function, but no handler here spawns:
// every task of this firmware is in main.rs.
#[allow(unused_imports)]
use postcard_rpc::server::impls::embassy_usb_v0_6::dispatch_impl::spawn_fn;
use protocol::{
    AddressesResult, ApplyUpdate, BeginInstall, BeginUpdate, BoardConfigResult, BoardId, BoardInfo,
    ConfigFor, ConfigResult, CoprocessorResult, CoprocessorStatus, Dataset, ENDPOINT_LIST,
    EnterBootloader, FinishInstall, FinishUpdate, FirmwareStatus, GetBoardConfig, GetBoardInfo,
    GetCoprocessorStatus, GetFirmwareStatus, GetMetrics, GetNetworkAddresses, GetNetworkDataset,
    GetNetworkNeighbors, GetNetworkRouters, GetNetworkStatus, GetOfferProgress, ImageChunk,
    ImageSize, JoinNetwork, LeaveNetwork, MetricsChunk, NeighborsResult, NetworkResult,
    NetworkStatus, OfferProgress, ReadSensorValue, Report, ReportReceived, RoutersResult,
    SensorReadError, SensorReadReq, SensorReadResult, SetBoardConfig, StartCollecting, StartOffering,
    StopCollecting, StopOffering, TOPICS_IN_LIST, TOPICS_OUT_LIST, UninstallStack, UpdateImage,
    UpdateResult, WriteInstall, WriteUpdate,
};

use crate::{
    board_config, coprocessor, dfu, metrics, report,
    sensor::{self, capacitance},
    thread, update, usb,
};

/// The handles that the RPC handlers use.
pub struct Context {
    pub thread: thread::Handle,
    pub coprocessor: coprocessor::Handle,
    pub bootloader: dfu::Handle,
    pub update: update::Handle,
    pub reports: report::Handle,
    pub config: board_config::Handle,
    pub metrics: metrics::Handle,
    // TODO(aspen): Make nicer
    pub capacitance: &'static sensor::Shared<capacitance::CapacitanceSensor<'static>>,
}

define_dispatch! {
    app: Dispatcher;
    spawn_fn: spawn_fn;
    tx_impl: usb::Tx;
    spawn_impl: WireSpawnImpl;
    context: Context;

    endpoints: {
        list: ENDPOINT_LIST;

        | EndpointTy           | kind     | handler            |
        | ----------           | ----     | -------            |
        | GetBoardInfo         | blocking | board_info         |
        | EnterBootloader      | blocking | enter_bootloader   |
        | GetNetworkStatus     | blocking | network_status     |
        | GetNetworkDataset    | blocking | network_dataset    |
        | JoinNetwork          | async    | join_network       |
        | LeaveNetwork         | async    | leave_network      |
        | GetNetworkNeighbors  | async    | network_neighbors  |
        | GetNetworkRouters    | async    | network_routers    |
        | GetCoprocessorStatus | blocking | coprocessor_status |
        | BeginInstall         | async    | begin_install      |
        | WriteInstall         | async    | write_install      |
        | FinishInstall        | async    | finish_install     |
        | UninstallStack       | async    | uninstall_stack    |
        | ReadSensorValue      | async    | read_sensor        |
        | StartCollecting      | async    | start_collecting   |
        | StopCollecting       | async    | stop_collecting    |
        | GetFirmwareStatus    | blocking | firmware_status    |
        | BeginUpdate          | async    | begin_update       |
        | WriteUpdate          | async    | write_update       |
        | FinishUpdate         | async    | finish_update      |
        | ApplyUpdate          | async    | apply_update       |
        | StartOffering        | async    | start_offering     |
        | StopOffering         | async    | stop_offering      |
        | GetOfferProgress     | blocking | offer_progress     |
        | GetBoardConfig       | async    | board_config       |
        | SetBoardConfig       | async    | set_board_config   |
        | GetNetworkAddresses  | async    | network_addresses  |
        | GetMetrics           | blocking | metrics_text       |
    };
    topics_in: {
        list: TOPICS_IN_LIST;

        | TopicTy              | kind     | handler            |
        | -------              | ----     | -------            |
    };
    topics_out: {
        list: TOPICS_OUT_LIST;
    };
}

pub type Server = server::Server<usb::Tx, usb::Rx, WireRxBuf, Dispatcher>;

/// Create the RPC server, and the [`Publisher`] for sending topics to the
/// host.
pub fn server(spawner: Spawner, link: usb::Link, context: Context) -> (Server, Publisher) {
    let dispatcher = Dispatcher::new(context, spawner.into());
    let key_len = dispatcher.min_key_len();
    let server = Server::new(link.tx, link.rx, link.rx_buf, dispatcher, key_len);
    let publisher = Publisher {
        sender: server.sender(),
        sequence: 0,
    };
    (server, publisher)
}

/// Publishes the topics of the `protocol` crate to the host.
pub struct Publisher {
    sender: server::Sender<usb::Tx>,
    sequence: u16,
}

impl Publisher {
    /// Forward a report to the host. Delivery is not confirmed: if no host
    /// is listening, the report is lost.
    pub async fn report_received(&mut self, report: &Report) {
        self.sequence = self.sequence.wrapping_add(1);
        let sequence = VarSeq::Seq2(self.sequence);
        let sent = self.sender.publish::<ReportReceived>(sequence, report);
        if sent.await.is_err() {
            defmt::debug!("rpc: no host took a report");
        }
    }
}

fn board_info(_context: &mut Context, _header: VarHeader, (): ()) -> BoardInfo {
    BoardInfo {
        // The build stamp is 19 characters. If it ever outgrows the field,
        // report an empty string instead of a truncated one.
        firmware_built: env!("BUILD_STAMP").try_into().unwrap_or_default(),
    }
}

fn enter_bootloader(context: &mut Context, _header: VarHeader, (): ()) {
    context.bootloader.request_bootloader();
}

fn network_status(context: &mut Context, _header: VarHeader, (): ()) -> NetworkStatus {
    context.thread.status()
}

fn network_dataset(context: &mut Context, _header: VarHeader, (): ()) -> Option<Dataset> {
    context.thread.dataset()
}

async fn join_network(
    context: &mut Context,
    _header: VarHeader,
    dataset: Dataset,
) -> NetworkResult {
    context.thread.join(dataset).await
}

async fn leave_network(context: &mut Context, _header: VarHeader, (): ()) -> NetworkResult {
    context.thread.leave().await
}

async fn network_neighbors(context: &mut Context, _header: VarHeader, (): ()) -> NeighborsResult {
    context.thread.neighbors().await
}

async fn network_routers(context: &mut Context, _header: VarHeader, (): ()) -> RoutersResult {
    context.thread.routers().await
}

async fn network_addresses(context: &mut Context, _header: VarHeader, (): ()) -> AddressesResult {
    context.thread.addresses().await
}

fn metrics_text(context: &mut Context, _header: VarHeader, offset: u32) -> MetricsChunk {
    context.metrics.text(offset)
}

fn coprocessor_status(context: &mut Context, _header: VarHeader, (): ()) -> CoprocessorStatus {
    context.coprocessor.status()
}

async fn begin_install(
    context: &mut Context,
    _header: VarHeader,
    size: ImageSize,
) -> CoprocessorResult {
    context.coprocessor.begin_install(size).await
}

async fn write_install(
    context: &mut Context,
    _header: VarHeader,
    chunk: ImageChunk,
) -> CoprocessorResult {
    context.coprocessor.write_install(chunk).await
}

async fn finish_install(context: &mut Context, _header: VarHeader, (): ()) -> CoprocessorResult {
    context.coprocessor.finish_install().await
}

async fn uninstall_stack(context: &mut Context, _header: VarHeader, (): ()) -> CoprocessorResult {
    context.coprocessor.uninstall_stack().await
}

fn firmware_status(context: &mut Context, _header: VarHeader, (): ()) -> FirmwareStatus {
    FirmwareStatus {
        build: update::build(),
        update: context.update.status(),
    }
}

async fn begin_update(
    context: &mut Context,
    _header: VarHeader,
    image: UpdateImage,
) -> UpdateResult {
    context.update.begin(image).await
}

async fn write_update(
    context: &mut Context,
    _header: VarHeader,
    chunk: ImageChunk,
) -> UpdateResult {
    context.update.write(chunk).await
}

async fn finish_update(context: &mut Context, _header: VarHeader, (): ()) -> UpdateResult {
    context.update.finish().await
}

async fn apply_update(context: &mut Context, _header: VarHeader, (): ()) -> UpdateResult {
    context.update.apply().await
}

async fn start_offering(context: &mut Context, _header: VarHeader, (): ()) -> UpdateResult {
    context.update.offer(true).await
}

async fn stop_offering(context: &mut Context, _header: VarHeader, (): ()) -> UpdateResult {
    context.update.offer(false).await
}

fn offer_progress(context: &mut Context, _header: VarHeader, (): ()) -> OfferProgress {
    context.update.offer_progress()
}

async fn board_config(
    context: &mut Context,
    _header: VarHeader,
    board: BoardId,
) -> BoardConfigResult {
    context.config.get(board).await
}

async fn set_board_config(
    context: &mut Context,
    _header: VarHeader,
    config: ConfigFor,
) -> ConfigResult {
    context.config.set(config).await
}

async fn start_collecting(context: &mut Context, _header: VarHeader, (): ()) -> NetworkResult {
    context.reports.collect(true).await
}

async fn stop_collecting(context: &mut Context, _header: VarHeader, (): ()) -> NetworkResult {
    context.reports.collect(false).await
}

async fn read_sensor(
    context: &mut Context,
    _header: VarHeader,
    req: SensorReadReq,
) -> SensorReadResult {
    let read_cap = async |chan| -> SensorReadResult {
        Ok(context
            .capacitance
            .lock()
            .await
            .read_channel_capacitance(chan)
            .await?
            .into())
    };
    };

    match req.sensor {
        protocol::Sensor::Capacitance0 => read_cap(capacitance::Channel::Ch0).await,
        protocol::Sensor::Capacitance1 => read_cap(capacitance::Channel::Ch1).await,
        protocol::Sensor::Capacitance2 => read_cap(capacitance::Channel::Ch2).await,
        protocol::Sensor::Capacitance3 => read_cap(capacitance::Channel::Ch3).await,
        protocol::Sensor::Distance => todo!(),
        protocol::Sensor::Color => todo!(),
        protocol::Sensor::Temperature => todo!(),
        protocol::Sensor::Humidity => todo!(),
        protocol::Sensor::Acceleration => todo!(),
    }
}
