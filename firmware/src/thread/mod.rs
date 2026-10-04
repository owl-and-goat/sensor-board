//! Thread networking, on the OpenThread stack that runs on CPU2.
//!
//! [`init`] makes the ends of it. The [`Service`] is the only thing that
//! talks to that stack, and it runs inside the coprocessor task. The rest of
//! the firmware goes through the [`Handle`], for the network itself, and
//! through a [`Socket`], for what is sent over it: a request is handed over
//! and its outcome awaited for a bounded time, and the status is whatever was
//! last published. A CPU2 that has stopped answering therefore costs a caller
//! a timeout, not its life, and no call into CPU2 is ever abandoned halfway.
//!
//! The service does one thing at a time: it answers a request, or it takes a
//! notification from the stack, and a notification is acknowledged only when
//! it has been dealt with. That is the order ST's own code keeps, and what
//! receiving needs: a datagram is CPU2's to take back at the acknowledgement.
//!
//! The only memory CPU2 is ever pointed at is `'static` and set aside for it
//! in [`init`] (see the buffers in `ot.rs`), so that it has nothing of anyone
//! else's to read or write, however a call ends. It reads the dataset to
//! join, which is why a board is given one rather than asked to make one up,
//! and it writes what it is asked about its neighbor and router tables, an
//! entry at a time.

use core::cell::Cell;
use core::net::Ipv6Addr;

use defmt::info;
use embassy_futures::{
    join::join,
    select::{Either, Either4, select, select4},
};
use embassy_stm32_wpan::{
    shci::SchiCommandStatus,
    sub::thread::{ThreadCliRx, ThreadNotifRx, ThreadOt},
};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::ThreadModeRawMutex},
    channel::{self, Channel},
};
use embassy_time::Duration;
use protocol::{
    Dataset, Link, NeighborTable, NeighborsResult, NetworkError, NetworkResult, NetworkStatus,
    Router, RouterId, RouterTable, RoutersResult,
};
use static_cell::StaticCell;

use crate::{
    persistent_config::{self, Config},
    radio_flash, request,
};

mod ffi;
mod ot;

use ot::OpenThread as _;
pub use ot::{Datagram, MAX_DATAGRAM_LEN};

/// Where a datagram to every board goes: all the devices of the network,
/// whatever addresses they have and however many hops away they are.
const ALL_BOARDS: Ipv6Addr = Ipv6Addr::new(0xff03, 0, 0, 0, 0, 0, 0, 1);

/// What boards talk to each other about, each on a UDP port of its own. The
/// ports are among the sixteen that 6LoWPAN writes in four bits.
#[derive(Clone, Copy)]
enum Port {
    Reports,
    Updates,
}

impl Port {
    const ALL: [Port; 2] = [Port::Reports, Port::Updates];

    const fn number(self) -> u16 {
        match self {
            Port::Reports => 61620,
            Port::Updates => 61621,
        }
    }

    /// Where the port's socket is among the sockets.
    const fn index(self) -> usize {
        self as usize
    }
}

/// How many datagrams that have arrived on a port can wait to be taken. One
/// more than that is dropped.
const RECEIVED_DEPTH: usize = 8;

/// Another board, by its address on the network.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Peer(Ipv6Addr);

/// A datagram that has arrived, and the board it is from.
pub struct Received {
    pub from: Peer,
    pub datagram: Datagram,
}

/// What the [`Handle`] can ask for that changes the network the board is on.
enum Request {
    Join(Dataset),
    Leave,
}

/// The [`Handle`] asking for the neighbor table, and for the router table.
/// Each has an answer of its own type, and so a channel of its own.
struct NeighborsRequest;
struct RoutersRequest;

/// What a [`Socket`] can ask for.
enum SocketRequest {
    /// Start taking in what is sent to the socket's port, or stop.
    Listen(bool),
    /// To every board on the network.
    Broadcast(Datagram),
    Send(Peer, Datagram),
}

/// What a [`Socket`] and the service have between them.
struct SocketShared {
    requests: request::Channel<SocketRequest, NetworkResult>,
    received: Channel<ThreadModeRawMutex, Received, RECEIVED_DEPTH>,
}

impl SocketShared {
    const fn new() -> Self {
        SocketShared {
            requests: request::Channel::new(),
            received: Channel::new(),
        }
    }
}

/// Where the board stands with its network, as last published.
struct Status(Mutex<ThreadModeRawMutex, Cell<NetworkStatus>>);

impl Status {
    fn get(&self) -> NetworkStatus {
        self.0.lock(|s| s.get())
    }

    fn publish(&self, status: NetworkStatus) {
        info!("thread: {}", status);
        self.0.lock(|s| s.set(status));
    }
}

/// How long a request gets. Each one is a few calls into CPU2 (a hundred or
/// so for a table) and at most one flash page, done in well under a second.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

/// What the ends have between them, and what CPU2 gets pointed at.
struct Shared {
    requests: request::Channel<Request, NetworkResult>,
    neighbor_requests: request::Channel<NeighborsRequest, NeighborsResult>,
    router_requests: request::Channel<RoutersRequest, RoutersResult>,
    /// One for each [`Port`], at its index.
    sockets: [SocketShared; Port::ALL.len()],
    status: Status,
    buffers: Buffers,
}

/// The memory CPU2 gets pointed at: a buffer for each call that takes a
/// pointer.
struct Buffers {
    active_dataset: ot::DatasetBuffer,
    next_neighbor: ot::NeighborBuffer,
    next_hop: ot::NextHopBuffer,
    /// One for the socket of each [`Port`], at its index.
    udp: [ot::UdpBuffer; Port::ALL.len()],
}

/// Make the ends of Thread: the handle and the sockets for the rest of the
/// firmware, and the service for the coprocessor task to run. Panics if
/// called a second time: the firmware has one of each.
pub fn init() -> (Handle, Sockets, Service) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        neighbor_requests: request::Channel::new(),
        router_requests: request::Channel::new(),
        sockets: [SocketShared::new(), SocketShared::new()],
        status: Status(Mutex::new(Cell::new(NetworkStatus::Starting))),
        buffers: Buffers {
            active_dataset: ot::DatasetBuffer::new(),
            next_neighbor: ot::NeighborBuffer::new(),
            next_hop: ot::NextHopBuffer::new(),
            udp: [ot::UdpBuffer::new(), ot::UdpBuffer::new()],
        },
    });

    let (client, server) = shared.requests.split();
    let (neighbor_client, neighbor_server) = shared.neighbor_requests.split();
    let (router_client, router_server) = shared.router_requests.split();
    let handle = Handle {
        requests: client,
        neighbor_requests: neighbor_client,
        router_requests: router_client,
        status: &shared.status,
    };

    let ends = |port: Port| {
        let socket = &shared.sockets[port.index()];
        let (client, server) = socket.requests.split();
        let socket_end = Socket {
            requests: client,
            received: socket.received.receiver(),
        };
        (socket_end, server, socket.received.sender())
    };
    let (reports, report_server, report_sender) = ends(Port::Reports);
    let (updates, update_server, update_sender) = ends(Port::Updates);

    let service = Service {
        requests: Requests {
            changes: server,
            neighbors: neighbor_server,
            routers: router_server,
            sockets: [report_server, update_server],
        },
        received: [report_sender, update_sender],
        status: &shared.status,
        buffers: &shared.buffers,
    };
    (handle, Sockets { reports, updates }, service)
}

/// What boards say to each other over Thread, a socket for each thing they
/// talk about.
pub struct Sockets {
    /// Sensor reports.
    pub reports: Socket,
    /// Firmware updates.
    pub updates: Socket,
}

/// Datagrams to the boards on the network, and from them.
pub struct Socket {
    requests: request::Client<SocketRequest, NetworkResult>,
    received: channel::Receiver<'static, ThreadModeRawMutex, Received, RECEIVED_DEPTH>,
}

impl Socket {
    /// Send `datagram` to every board on the network. Nothing tells whether
    /// any of them got it.
    pub async fn broadcast(&mut self, datagram: Datagram) -> NetworkResult {
        self.request(SocketRequest::Broadcast(datagram)).await
    }

    /// Send `datagram` to one board. Nothing tells whether it got it.
    pub async fn send_to(&mut self, peer: Peer, datagram: Datagram) -> NetworkResult {
        self.request(SocketRequest::Send(peer, datagram)).await
    }

    /// Start taking in what boards send to this socket, or stop: what they
    /// send to all of them, this one among them, and what they send to this
    /// one alone. A board that is not listening is not troubled by any of
    /// it.
    pub async fn listen(&mut self, on: bool) -> NetworkResult {
        self.request(SocketRequest::Listen(on)).await
    }

    /// The next datagram that has arrived. None does unless the board is
    /// listening.
    pub async fn receive(&mut self) -> Received {
        self.received.receive().await
    }

    async fn request(&mut self, request: SocketRequest) -> NetworkResult {
        self.requests
            .ask(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }
}

/// A look at where the board stands with its network, for what only needs
/// that.
#[derive(Clone, Copy)]
pub struct Monitor {
    status: &'static Status,
}

impl Monitor {
    pub fn status(&self) -> NetworkStatus {
        self.status.get()
    }
}

/// How the rest of the firmware reaches Thread.
pub struct Handle {
    requests: request::Client<Request, NetworkResult>,
    neighbor_requests: request::Client<NeighborsRequest, NeighborsResult>,
    router_requests: request::Client<RoutersRequest, RoutersResult>,
    status: &'static Status,
}

impl Handle {
    pub fn status(&self) -> NetworkStatus {
        self.status.get()
    }

    /// The dataset of the network the board is configured for, which it
    /// rejoins at every power-up.
    pub fn dataset(&self) -> Option<Dataset> {
        stored_dataset()
    }

    /// Start joining the network `dataset` describes, and keep rejoining it
    /// across power cycles. [`Handle::status`] tells when the board has
    /// attached.
    pub async fn join(&mut self, dataset: Dataset) -> NetworkResult {
        self.request(Request::Join(dataset)).await
    }

    /// Drop off the network and forget it.
    pub async fn leave(&mut self) -> NetworkResult {
        self.request(Request::Leave).await
    }

    /// The board's children, and the routers it has a direct radio link with
    /// right now, other than its parent.
    pub async fn neighbors(&mut self) -> NeighborsResult {
        self.neighbor_requests
            .ask(NeighborsRequest, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }

    /// Every router on the board's network, and what the board does to get a
    /// message to it.
    pub async fn routers(&mut self) -> RoutersResult {
        self.router_requests
            .ask(RoutersRequest, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }

    async fn request(&mut self, request: Request) -> NetworkResult {
        self.requests
            .ask(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }
}

fn stored_dataset() -> Option<Dataset> {
    let config = persistent_config::load()?;
    Dataset::from_tlvs(config.tlvs())
}

/// What it takes to run Thread: the stack's ends of the CPU2 mailbox, and the
/// means to keep a dataset in flash next to a running radio stack.
pub struct Thread<'a, 'd> {
    pub ot: ThreadOt<'d>,
    pub cli_rx: ThreadCliRx<'d>,
    pub notif_rx: ThreadNotifRx<'d>,
    pub flash: &'a radio_flash::Shared<'d>,
}

/// The serving ends of what the [`Handle`] and the [`Socket`]s ask through.
#[derive(Clone, Copy)]
struct Requests {
    changes: request::Server<Request, NetworkResult>,
    neighbors: request::Server<NeighborsRequest, NeighborsResult>,
    routers: request::Server<RoutersRequest, RoutersResult>,
    /// One for each [`Port`], at its index.
    sockets: [request::Server<SocketRequest, NetworkResult>; Port::ALL.len()],
}

/// A request that has been taken, and is owed an answer.
enum Asked {
    Change(request::Pending, Request),
    Neighbors(request::Pending),
    Routers(request::Pending),
    Socket(request::Pending, Port, SocketRequest),
}

impl Requests {
    async fn receive(&self) -> Asked {
        let [reports, updates] = &self.sockets;
        let next = select4(
            self.changes.receive(),
            self.neighbors.receive(),
            self.routers.receive(),
            select(reports.receive(), updates.receive()),
        );
        match next.await {
            Either4::First((pending, request)) => Asked::Change(pending, request),
            Either4::Second((pending, NeighborsRequest)) => Asked::Neighbors(pending),
            Either4::Third((pending, RoutersRequest)) => Asked::Routers(pending),
            Either4::Fourth(Either::First((pending, request))) => {
                Asked::Socket(pending, Port::Reports, request)
            }
            Either4::Fourth(Either::Second((pending, request))) => {
                Asked::Socket(pending, Port::Updates, request)
            }
        }
    }

    /// Answer `asked` with `error`.
    fn refuse(&self, asked: Asked, error: NetworkError) {
        match asked {
            Asked::Change(pending, _) => self.changes.answer(pending, Err(error)),
            Asked::Neighbors(pending) => self.neighbors.answer(pending, Err(error)),
            Asked::Routers(pending) => self.routers.answer(pending, Err(error)),
            Asked::Socket(pending, port, _) => {
                self.sockets[port.index()].answer(pending, Err(error))
            }
        }
    }
}

/// What has arrived on the socket of each [`Port`], on its way to whoever
/// has that socket.
type Arrivals =
    [channel::Sender<'static, ThreadModeRawMutex, Received, RECEIVED_DEPTH>; Port::ALL.len()];

/// The end of Thread that answers the [`Handle`] and the [`Socket`]s.
pub struct Service {
    requests: Requests,
    received: Arrivals,
    status: &'static Status,
    buffers: &'static Buffers,
}

impl Service {
    pub fn monitor(&self) -> Monitor {
        Monitor {
            status: self.status,
        }
    }

    /// Run Thread on a CPU2 that has a Thread stack.
    pub async fn run(self, thread: Thread<'_, '_>) -> ! {
        let Thread {
            ot,
            cli_rx,
            notif_rx,
            flash,
        } = thread;

        let network = Network {
            ot,
            notif_rx,
            flash,
            requests: self.requests,
            received: self.received,
            status: self.status,
            buffers: self.buffers,
            listening: [false; Port::ALL.len()],
        };

        // The stack stalls unless its CLI output is acknowledged. (The CLI
        // itself is on CPU1 in this stack version and nothing uses it, but
        // starting the stack never finishes with its channel left
        // unanswered.)
        join(drain_cli(cli_rx), network.run()).await.0
    }

    /// Stand in for Thread on a board where thread init has failed. Responds with an Unavailable
    /// error to every request
    pub async fn unavailable(self, error: NetworkError) -> ! {
        unavailable(self.requests, self.status, error).await
    }
}

async fn unavailable(requests: Requests, status: &Status, error: NetworkError) -> ! {
    status.publish(NetworkStatus::Unavailable(error));
    loop {
        let asked = requests.receive().await;
        requests.refuse(asked, error);
    }
}

async fn drain_cli(mut cli_rx: ThreadCliRx<'_>) -> ! {
    let mut output = [0; 64];
    loop {
        let n = cli_rx.receive(&mut output).await;
        defmt::trace!("cpu2: cli {=[u8]:a}", output[..n]);
    }
}

struct Network<'a, 'd> {
    ot: ThreadOt<'d>,
    notif_rx: ThreadNotifRx<'d>,
    flash: &'a radio_flash::Shared<'d>,
    requests: Requests,
    received: Arrivals,
    status: &'static Status,
    buffers: &'static Buffers,
    /// Whether the socket of each [`Port`] is bound to it.
    listening: [bool; Port::ALL.len()],
}

impl Network<'_, '_> {
    async fn run(mut self) -> ! {
        if let Err(e) = self.start().await {
            // A call into a stack that did not start might never come back.
            unavailable(self.requests, self.status, e).await
        }

        if let Some(dataset) = stored_dataset()
            && let Err(e) = self.rejoin(&dataset).await
        {
            defmt::warn!("thread: could not rejoin the stored network: {}", e);
        }
        self.publish_status().await;

        loop {
            let notification =
                ot::notification(&mut self.notif_rx, &mut self.ot, &self.buffers.udp);
            match select(self.requests.receive(), notification).await {
                Either::First(asked) => self.answer(asked).await,
                Either::Second(ot::Notification::StateChanged) => self.publish_status().await,
                Either::Second(ot::Notification::UdpReceived {
                    socket,
                    from,
                    payload,
                }) => {
                    let received = Received {
                        from: Peer(from),
                        datagram: payload,
                    };
                    if self.received[socket].try_send(received).is_err() {
                        defmt::warn!("thread: nothing is taking datagrams; dropped one");
                    }
                }
                Either::Second(ot::Notification::Other(id)) => {
                    defmt::trace!("cpu2: notification {}", id);
                }
            }
        }
    }

    async fn answer(&mut self, asked: Asked) {
        match asked {
            Asked::Change(pending, request) => {
                let outcome = match request {
                    Request::Join(dataset) => self.join(&dataset).await,
                    Request::Leave => self.leave().await,
                };
                // The status is published before the outcome, so that whoever
                // asked never reads a status older than the answer.
                self.publish_status().await;
                self.requests.changes.answer(pending, outcome);
            }
            Asked::Neighbors(pending) => {
                let neighbors = self.neighbors().await;
                self.requests.neighbors.answer(pending, neighbors);
            }
            Asked::Routers(pending) => {
                let routers = self.routers().await;
                self.requests.routers.answer(pending, routers);
            }
            Asked::Socket(pending, port, request) => {
                let outcome = match request {
                    SocketRequest::Listen(on) => self.listen(port, on).await,
                    SocketRequest::Broadcast(datagram) => {
                        self.send(port, ALL_BOARDS, &datagram).await
                    }
                    SocketRequest::Send(Peer(address), datagram) => {
                        self.send(port, address, &datagram).await
                    }
                };
                self.requests.sockets[port.index()]
                    .answer(pending, outcome.map_err(NetworkError::from));
            }
        }
    }

    /// Start the Thread stack on CPU2.
    async fn start(&mut self) -> NetworkResult {
        let started = self.flash.lock().await.sys().shci_c2_thread_init().await;
        match started {
            Ok(SchiCommandStatus::ShciSuccess) => {}
            _ => return Err(NetworkError::StartFailed),
        }
        self.ot.instance_init_single().await;
        self.ot.set_state_changed_callback().await?;
        for port in Port::ALL {
            let socket = &self.buffers.udp[port.index()];
            self.ot.udp_open(socket, port.index()).await?;
        }
        Ok(())
    }

    /// Have the socket of `port` take in what is sent to it, or not.
    async fn listen(&mut self, port: Port, on: bool) -> ot::Result<()> {
        let listening = &mut self.listening[port.index()];
        if on == *listening {
            return Ok(());
        }
        // A socket cannot be taken off a port again, so either way it is a
        // fresh one.
        let socket = &self.buffers.udp[port.index()];
        self.ot.udp_close(socket).await?;
        self.ot.udp_open(socket, port.index()).await?;
        *listening = false;
        if on {
            self.ot.udp_bind(socket, port.number()).await?;
            *listening = true;
        }
        Ok(())
    }

    /// Send from the socket of `port` to that port at `address`.
    async fn send(&mut self, port: Port, address: Ipv6Addr, datagram: &Datagram) -> ot::Result<()> {
        let socket = &self.buffers.udp[port.index()];
        self.ot
            .udp_send(socket, address, port.number(), datagram)
            .await
    }

    async fn publish_status(&mut self) {
        let status = self.status().await;
        self.status.publish(status);
    }

    async fn join(&mut self, dataset: &Dataset) -> NetworkResult {
        self.down().await?;
        let set = self
            .ot
            .dataset_set_active_tlvs(&self.buffers.active_dataset, dataset);
        if let Err(e) = set.await {
            // The stack has turned the dataset down and kept the one it had.
            // Go back to that network, which is also the one in flash.
            if stored_dataset().is_some() {
                self.up().await?;
            }
            return Err(e.into());
        }
        self.up().await?;

        let config = Config::new(dataset.as_tlvs());
        persistent_config::save(&mut *self.flash.lock().await, &config)
            .await
            .map_err(|_| NetworkError::Storage)
    }

    async fn leave(&mut self) -> NetworkResult {
        self.down().await?;
        persistent_config::erase(&mut *self.flash.lock().await)
            .await
            .map_err(|_| NetworkError::Storage)
    }

    /// Go back to the network the board was on before it lost power.
    async fn rejoin(&mut self, dataset: &Dataset) -> ot::Result<()> {
        self.ot
            .dataset_set_active_tlvs(&self.buffers.active_dataset, dataset)
            .await?;
        self.up().await
    }

    /// Bring up the interface, then the Thread protocol on it.
    async fn up(&mut self) -> ot::Result<()> {
        self.ot.ip6_set_enabled(true).await?;
        self.ot.thread_set_enabled(true).await
    }

    /// Stop the Thread protocol, then take the interface down.
    async fn down(&mut self) -> ot::Result<()> {
        self.ot.thread_set_enabled(false).await?;
        self.ot.ip6_set_enabled(false).await
    }

    /// The stack's neighbor table, as far as it fits in a [`NeighborTable`].
    async fn neighbors(&mut self) -> NeighborsResult {
        let mut table = NeighborTable::default();
        let mut iterator = ot::NeighborIterator::INIT;
        while let Some(neighbor) = self
            .ot
            .thread_get_next_neighbor_info(&self.buffers.next_neighbor, &mut iterator)
            .await?
        {
            if table.neighbors.push(neighbor).is_err() {
                table.truncated = true;
                break;
            }
        }
        Ok(table)
    }

    /// Every router the stack knows of, and its way to each.
    async fn routers(&mut self) -> RoutersResult {
        let mut table = RouterTable::default();
        for id in RouterId::all() {
            if !self.ot.thread_is_router_id_allocated(id).await {
                continue;
            }
            let route = self
                .ot
                .thread_get_next_hop_and_path_cost(&self.buffers.next_hop, id)
                .await;
            // The table has room for a router of every ID.
            let _ = table.routers.push(Router { id, route });
        }
        Ok(table)
    }

    async fn status(&mut self) -> NetworkStatus {
        if stored_dataset().is_none() {
            return NetworkStatus::Unconfigured;
        }
        NetworkStatus::Configured(Link {
            role: self.ot.thread_get_device_role().await,
            rloc16: self.ot.thread_get_rloc16().await,
            channel: self.ot.link_get_channel().await,
            pan_id: self.ot.link_get_panid().await,
        })
    }
}
