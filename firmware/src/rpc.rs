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
    AddressesResult, ApplyUpdate, BeginInstall, BeginUpdate, BoardConfigResult, BoardDiscovered,
    BoardId, BoardInfo, BuildCommit, ConfigFor, ConfigResult, CoprocessorResult, CoprocessorStatus,
    Dataset, DeviceModeResult, DiscoverBoards, ENDPOINT_LIST, EnterBootloader, FinishInstall,
    FinishUpdate, FirmwareStatus, GetBoardConfig, GetBoardInfo, GetBuildCommit,
    GetCoprocessorStatus, GetDeviceMode, GetFirmwareStatus, GetMetrics, GetNetworkAddresses,
    GetNetworkDataset, GetNetworkNeighbors, GetNetworkStatus, GetRegistration, ImageChunk,
    ImageSize, JoinNetwork, LeaveNetwork, Member, MetricsChunk, NeighborsResult, NetworkResult,
    NetworkStatus, Registration, SetBoardConfig, TOPICS_IN_LIST, TOPICS_OUT_LIST, UninstallStack,
    UpdateImage, UpdateResult, WriteInstall, WriteUpdate,
};

use crate::{board_config, coprocessor, dfu, discovery, metrics, thread, update, usb};

/// The handles that the RPC handlers use.
pub struct Context {
    pub thread: thread::Handle,
    pub coprocessor: coprocessor::Handle,
    pub bootloader: dfu::Handle,
    pub update: update::Handle,
    pub config: board_config::Handle,
    pub metrics: metrics::Handle,
    pub discovery: discovery::Handle,
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
        | GetCoprocessorStatus | blocking | coprocessor_status |
        | BeginInstall         | async    | begin_install      |
        | WriteInstall         | async    | write_install      |
        | FinishInstall        | async    | finish_install     |
        | UninstallStack       | async    | uninstall_stack    |
        | GetFirmwareStatus    | blocking | firmware_status    |
        | BeginUpdate          | async    | begin_update       |
        | WriteUpdate          | async    | write_update       |
        | FinishUpdate         | async    | finish_update      |
        | ApplyUpdate          | async    | apply_update       |
        | GetBoardConfig       | async    | board_config       |
        | SetBoardConfig       | async    | set_board_config   |
        | GetNetworkAddresses  | async    | network_addresses  |
        | GetMetrics           | blocking | metrics_text       |
        | DiscoverBoards       | async    | discover_boards    |
        | GetRegistration      | blocking | registration       |
        | GetDeviceMode        | blocking | device_mode        |
        | GetBuildCommit       | blocking | build_commit       |
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
    /// Forward a board's discovery reply to the host. Delivery is not
    /// confirmed: if no host is listening, the reply is lost.
    pub async fn board_discovered(&mut self, member: &Member) {
        let sequence = self.next_sequence();
        let sent = self.sender.publish::<BoardDiscovered>(sequence, member);
        if sent.await.is_err() {
            defmt::debug!("rpc: no host took a discovery reply");
        }
    }

    fn next_sequence(&mut self) -> VarSeq {
        self.sequence = self.sequence.wrapping_add(1);
        VarSeq::Seq2(self.sequence)
    }
}

fn board_info(_context: &mut Context, _header: VarHeader, (): ()) -> BoardInfo {
    BoardInfo {
        // The build stamp is 19 characters. If it ever outgrows the field,
        // report an empty string instead of a truncated one.
        firmware_built: env!("BUILD_STAMP").try_into().unwrap_or_default(),
    }
}

fn build_commit(_context: &mut Context, _header: VarHeader, (): ()) -> BuildCommit {
    // A hash with the suffix is 18 characters.
    BuildCommit(env!("BUILD_COMMIT").try_into().unwrap_or_default())
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

async fn network_addresses(context: &mut Context, _header: VarHeader, (): ()) -> AddressesResult {
    context.thread.addresses().await
}

fn registration(context: &mut Context, _header: VarHeader, (): ()) -> Registration {
    context.thread.registration()
}

fn device_mode(context: &mut Context, _header: VarHeader, (): ()) -> DeviceModeResult {
    context.thread.device_mode()
}

fn metrics_text(context: &mut Context, _header: VarHeader, offset: u32) -> MetricsChunk {
    context.metrics.text(offset)
}

async fn discover_boards(context: &mut Context, _header: VarHeader, (): ()) -> NetworkResult {
    context.discovery.discover().await
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
