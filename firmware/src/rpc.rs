//! Serves the endpoints in the `protocol` crate to the USB host. The handlers
//! only translate: the work happens in the modules they call. Those that
//! wait on something wait a bounded time, so one request cannot hold up the
//! next for good.

use embassy_executor::Spawner;
use postcard_rpc::{
    define_dispatch,
    header::VarHeader,
    server::{
        self, Dispatch,
        impls::embassy_usb_v0_6::dispatch_impl::{WireRxBuf, WireSpawnImpl},
    },
};
// `define_dispatch!` has to be told how to spawn, though no handler here is
// the spawning kind: every task of this firmware is in main.rs.
#[allow(unused_imports)]
use postcard_rpc::server::impls::embassy_usb_v0_6::dispatch_impl::spawn_fn;
use protocol::{
    BeginInstall, BoardInfo, CoprocessorResult, CoprocessorStatus, Dataset, ENDPOINT_LIST,
    EnterBootloader, FinishInstall, GetBoardInfo, GetCoprocessorStatus, GetNetworkDataset,
    GetNetworkStatus, ImageChunk, ImageSize, JoinNetwork, LeaveNetwork, NetworkResult,
    NetworkStatus, TOPICS_IN_LIST, TOPICS_OUT_LIST, UninstallStack, WriteInstall,
};

use crate::{coprocessor, dfu, thread, usb};

/// What the handlers act on.
pub struct Context {
    pub thread: thread::Handle,
    pub coprocessor: coprocessor::Handle,
    pub bootloader: dfu::Handle,
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
        | GetCoprocessorStatus | blocking | coprocessor_status |
        | BeginInstall         | async    | begin_install      |
        | WriteInstall         | async    | write_install      |
        | FinishInstall        | async    | finish_install     |
        | UninstallStack       | async    | uninstall_stack    |
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

pub fn server(spawner: Spawner, link: usb::Link, context: Context) -> Server {
    let dispatcher = Dispatcher::new(context, spawner.into());
    let key_len = dispatcher.min_key_len();
    Server::new(link.tx, link.rx, link.rx_buf, dispatcher, key_len)
}

fn board_info(_context: &mut Context, _header: VarHeader, (): ()) -> BoardInfo {
    BoardInfo {
        // The build stamp is 19 characters; if that ever outgrows the field,
        // report nothing rather than part of it.
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
