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
use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::info;
use embassy_futures::{
    join::join,
    select::{Either, Either3, Either4, select, select_array, select3, select4},
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
    AddressesResult, Dataset, Link, NeighborTable, NeighborsResult, NetworkError, NetworkResult,
    NetworkStatus, OtError, Router, RouterId, RouterTable, RoutersResult,
};
use static_cell::StaticCell;

use crate::{persistent_config, radio_flash, request};

mod ffi;
mod ot;

use ot::OpenThread as _;
pub use ot::{Datagram, MAX_DATAGRAM_LEN, TCP_CHUNK_LEN, TcpChunk};

/// Where a datagram to every board goes: all the devices of the network,
/// whatever addresses they have and however many hops away they are.
const ALL_BOARDS: Ipv6Addr = Ipv6Addr::new(0xff03, 0, 0, 0, 0, 0, 0, 1);

/// What boards talk to each other about, each on a UDP port of its own. The
/// ports are among the sixteen that 6LoWPAN writes in four bits.
#[derive(Clone, Copy)]
enum Port {
    Reports,
    Updates,
    Configs,
}

impl Port {
    const ALL: [Port; 3] = [Port::Reports, Port::Updates, Port::Configs];

    const fn number(self) -> u16 {
        match self {
            Port::Reports => 61620,
            Port::Updates => 61621,
            Port::Configs => 61622,
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

/// Requests from the [`Handle`] for the neighbor table, the router table and
/// the board's addresses. Each has its own response type, so each has its
/// own channel.
struct NeighborsRequest;
struct RoutersRequest;
struct AddressesRequest;

/// A request from the [`Tcp`] handle. The response is sent when the
/// operation completes or fails. Some operations wait for the peer.
enum TcpRequest {
    /// Listen on this port, or with `None` stop listening.
    Listen(Option<u16>),
    /// Wait for an incoming connection.
    Accept,
    /// Open a connection.
    Connect(SocketAddrV6),
    /// Send a chunk, and wait for the peer to acknowledge it.
    Send(TcpChunk),
    /// Wait for data. An empty chunk means the peer has closed its side.
    Receive,
    /// Close the sending side of the connection.
    Finish,
    /// Abort the connection, whatever state it is in.
    Close,
}

/// The response to a [`TcpRequest`]. The chunk holds the received data for a
/// `Receive`, and is empty for every other request.
type TcpResult = Result<TcpChunk, TcpError>;

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TcpError {
    /// Thread is not running, or the stack returned an error.
    Network(NetworkError),
    /// The request is not valid in the current state: for example, a `Send`
    /// with no connection, or a `Connect` with one.
    OutOfSequence,
    Refused,
    Reset,
    /// The peer or the stack did not respond in time.
    TimedOut,
}

impl From<OtError> for TcpError {
    fn from(e: OtError) -> Self {
        TcpError::Network(e.into())
    }
}

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
    address_requests: request::Channel<AddressesRequest, AddressesResult>,
    /// One for each [`Port`], at its index.
    sockets: [SocketShared; Port::ALL.len()],
    tcp_requests: request::Channel<TcpRequest, TcpResult>,
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
    tcp: ot::TcpBuffer,
}

/// Create the [`Handle`], the [`Sockets`] and the [`Tcp`] handle for the rest
/// of the firmware, and the [`Service`] for the coprocessor task to run.
/// Panics if called a second time.
pub fn init() -> (Handle, Sockets, Tcp, Service) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        neighbor_requests: request::Channel::new(),
        router_requests: request::Channel::new(),
        address_requests: request::Channel::new(),
        sockets: [const { SocketShared::new() }; Port::ALL.len()],
        tcp_requests: request::Channel::new(),
        status: Status(Mutex::new(Cell::new(NetworkStatus::Starting))),
        buffers: Buffers {
            active_dataset: ot::DatasetBuffer::new(),
            next_neighbor: ot::NeighborBuffer::new(),
            next_hop: ot::NextHopBuffer::new(),
            udp: [const { ot::UdpBuffer::new() }; Port::ALL.len()],
            tcp: ot::TcpBuffer::new(),
        },
    });

    let (client, server) = shared.requests.split();
    let (neighbor_client, neighbor_server) = shared.neighbor_requests.split();
    let (router_client, router_server) = shared.router_requests.split();
    let (address_client, address_server) = shared.address_requests.split();
    let (tcp_client, tcp_server) = shared.tcp_requests.split();
    let handle = Handle {
        requests: client,
        neighbor_requests: neighbor_client,
        router_requests: router_client,
        address_requests: address_client,
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
    let (configs, config_server, config_sender) = ends(Port::Configs);

    let service = Service {
        requests: Requests {
            changes: server,
            neighbors: neighbor_server,
            routers: router_server,
            addresses: address_server,
            sockets: [report_server, update_server, config_server],
            tcp: tcp_server,
        },
        received: [report_sender, update_sender, config_sender],
        status: &shared.status,
        buffers: &shared.buffers,
    };
    let sockets = Sockets {
        reports,
        updates,
        configs,
    };
    let tcp = Tcp {
        requests: tcp_client,
    };
    (handle, sockets, tcp, service)
}

/// What boards say to each other over Thread, a socket for each thing they
/// talk about.
pub struct Sockets {
    /// Sensor reports.
    pub reports: Socket,
    /// Firmware updates.
    pub updates: Socket,
    /// The boards' configurations.
    pub configs: Socket,
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

/// How long to wait for the TCP peer: for a connection to open, for sent
/// data to be acknowledged, and for data to arrive.
const TCP_TIMEOUT: Duration = Duration::from_secs(10);

/// How often [`Tcp::accept`] renews its request while it waits. A connection
/// that arrives between two requests is not lost.
const ACCEPT_INTERVAL: Duration = Duration::from_secs(60);

/// The firmware's one TCP connection: to a device on the Thread network, or
/// beyond it through a border router. It is either opened with
/// [`Tcp::connect`] or accepted with [`Tcp::accept`].
pub struct Tcp {
    requests: request::Client<TcpRequest, TcpResult>,
}

impl Tcp {
    /// Listen for connections on `port`, or with `None` stop listening.
    pub async fn listen(&mut self, port: Option<u16>) -> Result<(), TcpError> {
        self.request(TcpRequest::Listen(port), REQUEST_TIMEOUT)
            .await
    }

    /// Wait for an incoming connection. Does not time out.
    pub async fn accept(&mut self) -> Result<(), TcpError> {
        loop {
            let accepted = self.requests.ask(TcpRequest::Accept, ACCEPT_INTERVAL);
            if let Some(outcome) = accepted.await {
                return outcome.map(drop);
            }
        }
    }

    /// Open a connection to `peer`.
    pub async fn connect(&mut self, peer: SocketAddrV6) -> Result<(), TcpError> {
        self.request(TcpRequest::Connect(peer), TCP_TIMEOUT).await
    }

    /// Send `bytes`, and wait for the peer to acknowledge them.
    pub async fn send(&mut self, bytes: &[u8]) -> Result<(), TcpError> {
        for piece in bytes.chunks(TCP_CHUNK_LEN) {
            // Cannot fail: a piece is no longer than a chunk.
            let chunk = TcpChunk::from_slice(piece).unwrap_or_default();
            self.request(TcpRequest::Send(chunk), TCP_TIMEOUT).await?;
        }
        Ok(())
    }

    /// Wait for data. An empty chunk means the peer has closed its side.
    pub async fn receive(&mut self) -> Result<TcpChunk, TcpError> {
        self.requests
            .ask(TcpRequest::Receive, TCP_TIMEOUT)
            .await
            .unwrap_or(Err(TcpError::TimedOut))
    }

    /// Close the sending side. The peer can still send.
    pub async fn finish(&mut self) -> Result<(), TcpError> {
        self.request(TcpRequest::Finish, REQUEST_TIMEOUT).await
    }

    /// Abort the connection, whatever state it is in. This frees the
    /// endpoint for the next connection.
    pub async fn close(&mut self) {
        if let Err(e) = self.request(TcpRequest::Close, REQUEST_TIMEOUT).await {
            defmt::warn!("thread: a TCP connection could not be dropped: {}", e);
        }
    }

    async fn request(&mut self, request: TcpRequest, timeout: Duration) -> Result<(), TcpError> {
        let outcome = self.requests.ask(request, timeout).await;
        outcome.unwrap_or(Err(TcpError::TimedOut)).map(drop)
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
    address_requests: request::Client<AddressesRequest, AddressesResult>,
    status: &'static Status,
}

impl Handle {
    pub fn status(&self) -> NetworkStatus {
        self.status.get()
    }

    /// The dataset of the network the board is configured for, which it
    /// rejoins at every power-up.
    pub fn dataset(&self) -> Option<Dataset> {
        persistent_config::dataset()
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

    /// The board's IPv6 unicast addresses.
    pub async fn addresses(&mut self) -> AddressesResult {
        self.address_requests
            .ask(AddressesRequest, REQUEST_TIMEOUT)
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

/// What it takes to run Thread: the stack's ends of the CPU2 mailbox, and the
/// means to keep a dataset in flash next to a running radio stack.
pub struct Thread<'a, 'd> {
    pub ot: ThreadOt<'d>,
    pub cli_rx: ThreadCliRx<'d>,
    pub notif_rx: ThreadNotifRx<'d>,
    pub flash: &'a radio_flash::Shared<'d>,
}

/// The server ends of the request channels of the [`Handle`], the
/// [`Socket`]s and the [`Tcp`] handle.
#[derive(Clone, Copy)]
struct Requests {
    changes: request::Server<Request, NetworkResult>,
    neighbors: request::Server<NeighborsRequest, NeighborsResult>,
    routers: request::Server<RoutersRequest, RoutersResult>,
    addresses: request::Server<AddressesRequest, AddressesResult>,
    /// One for each [`Port`], at its index.
    sockets: [request::Server<SocketRequest, NetworkResult>; Port::ALL.len()],
    tcp: request::Server<TcpRequest, TcpResult>,
}

/// A request that has been taken, and is owed an answer.
enum Asked {
    Change(request::Pending, Request),
    Neighbors(request::Pending),
    Routers(request::Pending),
    Addresses(request::Pending),
    Socket(request::Pending, Port, SocketRequest),
    Tcp(request::Pending, TcpRequest),
}

impl Requests {
    async fn receive(&self) -> Asked {
        let sockets = self.sockets.each_ref().map(|socket| socket.receive());
        let tables = select3(
            self.neighbors.receive(),
            self.routers.receive(),
            self.addresses.receive(),
        );
        let next = select4(
            self.changes.receive(),
            tables,
            select_array(sockets),
            self.tcp.receive(),
        );
        match next.await {
            Either4::First((pending, request)) => Asked::Change(pending, request),
            Either4::Second(Either3::First((pending, NeighborsRequest))) => {
                Asked::Neighbors(pending)
            }
            Either4::Second(Either3::Second((pending, RoutersRequest))) => Asked::Routers(pending),
            Either4::Second(Either3::Third((pending, AddressesRequest))) => {
                Asked::Addresses(pending)
            }
            Either4::Third(((pending, request), socket)) => {
                Asked::Socket(pending, Port::ALL[socket], request)
            }
            Either4::Fourth((pending, request)) => Asked::Tcp(pending, request),
        }
    }

    /// Answer `asked` with `error`.
    fn refuse(&self, asked: Asked, error: NetworkError) {
        match asked {
            Asked::Change(pending, _) => self.changes.answer(pending, Err(error)),
            Asked::Neighbors(pending) => self.neighbors.answer(pending, Err(error)),
            Asked::Routers(pending) => self.routers.answer(pending, Err(error)),
            Asked::Addresses(pending) => self.addresses.answer(pending, Err(error)),
            Asked::Socket(pending, port, _) => {
                self.sockets[port.index()].answer(pending, Err(error))
            }
            Asked::Tcp(pending, _) => self.tcp.answer(pending, Err(TcpError::Network(error))),
        }
    }
}

/// The state of the TCP endpoint's connection.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Connection {
    /// No connection. The endpoint can accept or open one.
    Closed,
    /// A connection is being established, in either direction.
    Opening,
    Open,
    /// The connection has ended, but the endpoint is in TIME-WAIT and
    /// cannot be reused yet (see [`ot::TcpDisconnected::Lingering`]).
    Over,
}

/// The event that a pending [`TcpRequest`] is waiting for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum TcpWait {
    Accepted,
    Connected,
    Sent,
    Received,
}

/// The immediate outcome of a [`TcpRequest`]: its response, or the event to
/// wait for before responding.
enum TcpStep {
    Done(TcpChunk),
    Wait(TcpWait),
}

/// The service's view of the TCP endpoint.
struct TcpEndpoint {
    /// Whether the endpoint has been initialized on CPU2.
    ready: bool,
    /// The port the listener is listening on.
    listening: Option<u16>,
    connection: Connection,
    /// Whether the last chunk sent is still unacknowledged, in which case
    /// the stack is still reading the send buffer.
    sending: bool,
    /// Whether the peer has closed its sending side.
    end_of_stream: bool,
    /// The pending request, and the event it is waiting for.
    waiting: Option<(request::Pending, TcpWait)>,
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
            tcp: TcpEndpoint {
                ready: false,
                listening: None,
                connection: Connection::Closed,
                sending: false,
                end_of_stream: false,
                waiting: None,
            },
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
    tcp: TcpEndpoint,
}

impl Network<'_, '_> {
    async fn run(mut self) -> ! {
        if let Err(e) = self.start().await {
            // A call into a stack that did not start might never come back.
            unavailable(self.requests, self.status, e).await
        }

        if let Some(dataset) = persistent_config::dataset()
            && let Err(e) = self.rejoin(&dataset).await
        {
            defmt::warn!("thread: could not rejoin the stored network: {}", e);
        }
        self.publish_status().await;

        loop {
            // Accept an incoming connection only if the listener is active
            // and the endpoint has no connection.
            let take_tcp =
                self.tcp.listening.is_some() && self.tcp.connection == Connection::Closed;
            let notification = ot::notification(
                &mut self.notif_rx,
                &mut self.ot,
                &self.buffers.udp,
                &self.buffers.tcp,
                take_tcp,
            );
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
                Either::Second(ot::Notification::Tcp(event)) => self.tcp_event(event).await,
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
            Asked::Addresses(pending) => {
                let addresses = self.ot.ip6_unicast_addresses().await;
                self.requests.addresses.answer(pending, Ok(addresses));
            }
            Asked::Tcp(pending, request) => {
                // A new request replaces the pending one, whose caller has
                // stopped waiting.
                self.tcp.waiting = None;
                match self.tcp_step(request).await {
                    Ok(TcpStep::Done(chunk)) => self.requests.tcp.answer(pending, Ok(chunk)),
                    Ok(TcpStep::Wait(wait)) => self.tcp.waiting = Some((pending, wait)),
                    Err(e) => self.requests.tcp.answer(pending, Err(e)),
                }
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
        // Thread works without TCP, so a failure here is not fatal.
        match self.ot.tcp_init(&self.buffers.tcp).await {
            Ok(()) => self.tcp.ready = true,
            Err(e) => defmt::warn!("thread: no TCP: {}", e),
        }
        Ok(())
    }

    /// Start `request`. Returns its response if it completes immediately,
    /// and otherwise the event to wait for.
    async fn tcp_step(&mut self, request: TcpRequest) -> Result<TcpStep, TcpError> {
        let buffer = &self.buffers.tcp;
        if !self.tcp.ready {
            return Err(OtError::NotImplemented.into());
        }
        let done = Ok(TcpStep::Done(TcpChunk::new()));

        match request {
            TcpRequest::Listen(port) => {
                if self.tcp.listening.is_some() && self.tcp.listening != port {
                    self.ot.tcp_stop_listening(buffer).await?;
                    self.tcp.listening = None;
                }
                if let Some(port) = port
                    && self.tcp.listening.is_none()
                {
                    self.ot.tcp_listen(buffer, port).await?;
                    self.tcp.listening = Some(port);
                }
                done
            }
            TcpRequest::Accept => match self.tcp.connection {
                Connection::Open => done,
                Connection::Closed | Connection::Opening if self.tcp.listening.is_some() => {
                    Ok(TcpStep::Wait(TcpWait::Accepted))
                }
                _ => Err(TcpError::OutOfSequence),
            },
            TcpRequest::Connect(peer) => {
                if self.tcp.connection != Connection::Closed {
                    return Err(TcpError::OutOfSequence);
                }
                self.ot.tcp_connect(buffer, peer).await?;
                self.tcp.connection = Connection::Opening;
                Ok(TcpStep::Wait(TcpWait::Connected))
            }
            TcpRequest::Send(chunk) => {
                // The stack reads from the send buffer until the peer
                // acknowledges the data, so the buffer cannot be reused
                // before then.
                if self.tcp.connection != Connection::Open || self.tcp.sending {
                    return Err(TcpError::OutOfSequence);
                }
                self.ot.tcp_send(buffer, &chunk).await?;
                self.tcp.sending = true;
                Ok(TcpStep::Wait(TcpWait::Sent))
            }
            TcpRequest::Receive => {
                let over = match self.tcp.connection {
                    Connection::Open => self.tcp.end_of_stream,
                    Connection::Over => true,
                    Connection::Closed | Connection::Opening => {
                        return Err(TcpError::OutOfSequence);
                    }
                };
                let chunk = self.ot.tcp_receive(buffer).await?;
                if chunk.is_empty() && !over {
                    Ok(TcpStep::Wait(TcpWait::Received))
                } else {
                    Ok(TcpStep::Done(chunk))
                }
            }
            TcpRequest::Finish => {
                if self.tcp.connection != Connection::Open {
                    return Err(TcpError::OutOfSequence);
                }
                self.ot.tcp_send_end_of_stream(buffer).await?;
                done
            }
            TcpRequest::Close => {
                if self.tcp.connection != Connection::Closed {
                    let (aborted, state_changed) =
                        ot::tcp_abort(&mut self.notif_rx, &mut self.ot, buffer).await;
                    if state_changed {
                        self.publish_status().await;
                    }
                    aborted?;
                }
                self.tcp.connection = Connection::Closed;
                self.tcp.sending = false;
                self.tcp.end_of_stream = false;
                done
            }
        }
    }

    /// Handle a TCP callback from the stack.
    async fn tcp_event(&mut self, event: ot::TcpEvent) {
        use ot::{TcpDisconnected, TcpEvent};

        match event {
            TcpEvent::Incoming { taken: true } => self.tcp.connection = Connection::Opening,
            TcpEvent::Incoming { taken: false } => {}
            TcpEvent::Established => {
                // The stack may report an accepted connection twice.
                if self.tcp.connection != Connection::Open {
                    self.tcp.connection = Connection::Open;
                    self.tcp.end_of_stream = false;
                }
                let opened = |wait| matches!(wait, TcpWait::Accepted | TcpWait::Connected);
                self.tcp_answer(opened, Ok(TcpChunk::new()));
            }
            TcpEvent::SendDone => {
                self.tcp.sending = false;
                self.tcp_answer(|wait| wait == TcpWait::Sent, Ok(TcpChunk::new()));
            }
            TcpEvent::ReceiveAvailable { end_of_stream } => {
                self.tcp.end_of_stream |= end_of_stream;
                if !matches!(self.tcp.waiting, Some((_, TcpWait::Received))) {
                    return;
                }
                let received = self.ot.tcp_receive(&self.buffers.tcp).await;
                match received {
                    // No data yet, and the stream is still open: keep
                    // waiting.
                    Ok(chunk) if chunk.is_empty() && !self.tcp.end_of_stream => {}
                    Ok(chunk) => self.tcp_answer(|_| true, Ok(chunk)),
                    Err(e) => self.tcp_answer(|_| true, Err(e.into())),
                }
            }
            TcpEvent::Disconnected(how) => {
                // The stack no longer reads the send buffer.
                self.tcp.sending = false;
                let ended = matches!(how, TcpDisconnected::Normal | TcpDisconnected::Lingering);
                self.tcp.connection = match how {
                    TcpDisconnected::Lingering => Connection::Over,
                    _ => Connection::Closed,
                };
                let error = match how {
                    TcpDisconnected::Refused => TcpError::Refused,
                    TcpDisconnected::TimedOut => TcpError::TimedOut,
                    _ => TcpError::Reset,
                };
                match self.tcp.waiting.as_ref().map(|(_, wait)| *wait) {
                    // No request is affected: an `Accept` keeps waiting for
                    // the next connection.
                    Some(TcpWait::Accepted) | None => {}
                    // A clean close: the peer acknowledged everything sent,
                    // and has nothing more to send.
                    Some(TcpWait::Sent | TcpWait::Received) if ended => {
                        self.tcp_answer(|_| true, Ok(TcpChunk::new()))
                    }
                    Some(_) => self.tcp_answer(|_| true, Err(error)),
                }
            }
        }
    }

    /// Respond to the pending request with `answer`, if `is_it` accepts the
    /// event it is waiting for.
    fn tcp_answer(&mut self, is_it: impl FnOnce(TcpWait) -> bool, answer: TcpResult) {
        let awaited = self.tcp.waiting.take_if(|(_, wait)| is_it(*wait));
        if let Some((pending, _)) = awaited {
            self.requests.tcp.answer(pending, answer);
        }
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
            if persistent_config::dataset().is_some() {
                self.up().await?;
            }
            return Err(e.into());
        }
        self.up().await?;

        persistent_config::set_dataset(&mut *self.flash.lock().await, Some(dataset))
            .await
            .map_err(|_| NetworkError::Storage)
    }

    async fn leave(&mut self) -> NetworkResult {
        self.down().await?;
        persistent_config::set_dataset(&mut *self.flash.lock().await, None)
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
        if persistent_config::dataset().is_none() {
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
