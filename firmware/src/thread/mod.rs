//! Thread networking, on the OpenThread stack that runs on CPU2.
//!
//! [`init`] makes the two ends of it. The [`Service`] is the only thing that
//! talks to that stack, and it runs inside the coprocessor task. The rest of
//! the firmware goes through the [`Handle`]: a request is handed over and its
//! outcome awaited for a bounded time, and the status is whatever was last
//! published. A CPU2 that has stopped answering therefore costs a caller a
//! timeout, not its life, and no call into CPU2 is ever abandoned halfway.
//!
//! The only memory CPU2 is ever pointed at is `'static` and set aside for it
//! in [`init`] (see the buffers in `ot.rs`), so that it has nothing of anyone
//! else's to read or write, however a call ends. It reads the dataset to
//! join, which is why a board is given one rather than asked to make one up,
//! and it writes what it is asked about its neighbor and router tables, an
//! entry at a time.

use core::cell::Cell;

use defmt::info;
use embassy_futures::{
    join::join3,
    select::{Either, Either3, select, select3},
};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32_wpan::{
    shci::SchiCommandStatus,
    sub::{
        sys::Sys,
        thread::{ThreadCliRx, ThreadNotifRx, ThreadOt},
    },
};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::ThreadModeRawMutex},
    signal::Signal,
};
use embassy_time::Duration;
use protocol::{
    Dataset, Link, NeighborTable, NeighborsResult, NetworkError, NetworkResult, NetworkStatus,
    Router, RouterId, RouterTable, RoutersResult,
};
use static_cell::StaticCell;

use crate::{
    persistent_config::{self, Config},
    request,
};

mod ffi;
mod ot;

use ot::OpenThread as _;

/// What the [`Handle`] can ask for that changes the network the board is on.
enum Request {
    Join(Dataset),
    Leave,
}

/// The [`Handle`] asking for the neighbor table, and for the router table.
/// Each has an answer of its own type, and so a channel of its own.
struct NeighborsRequest;
struct RoutersRequest;

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

/// What the two ends have between them, and what CPU2 gets pointed at.
struct Shared {
    requests: request::Channel<Request, NetworkResult>,
    neighbor_requests: request::Channel<NeighborsRequest, NeighborsResult>,
    router_requests: request::Channel<RoutersRequest, RoutersResult>,
    status: Status,
    buffers: Buffers,
}

/// The memory CPU2 gets pointed at: a buffer for each call that takes a
/// pointer.
struct Buffers {
    active_dataset: ot::DatasetBuffer,
    next_neighbor: ot::NeighborBuffer,
    next_hop: ot::NextHopBuffer,
}

/// Make the two ends of Thread: the handle for the rest of the firmware, and
/// the service for the coprocessor task to run. Panics if called a second
/// time: the firmware has one of each.
pub fn init() -> (Handle, Service) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        neighbor_requests: request::Channel::new(),
        router_requests: request::Channel::new(),
        status: Status(Mutex::new(Cell::new(NetworkStatus::Starting))),
        buffers: Buffers {
            active_dataset: ot::DatasetBuffer::new(),
            next_neighbor: ot::NeighborBuffer::new(),
            next_hop: ot::NextHopBuffer::new(),
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
    let service = Service {
        requests: Requests {
            changes: server,
            neighbors: neighbor_server,
            routers: router_server,
        },
        status: &shared.status,
        buffers: &shared.buffers,
    };
    (handle, service)
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
pub struct Thread<'d> {
    pub ot: ThreadOt<'d>,
    pub cli_rx: ThreadCliRx<'d>,
    pub notif_rx: ThreadNotifRx<'d>,
    pub sys: Sys<'d>,
    pub flash: Flash<'d, Blocking>,
}

/// The serving ends of what the [`Handle`] asks through.
#[derive(Clone, Copy)]
struct Requests {
    changes: request::Server<Request, NetworkResult>,
    neighbors: request::Server<NeighborsRequest, NeighborsResult>,
    routers: request::Server<RoutersRequest, RoutersResult>,
}

/// A request that has been taken, and is owed an answer.
enum Asked {
    Change(request::Pending, Request),
    Neighbors(request::Pending),
    Routers(request::Pending),
}

impl Requests {
    async fn receive(&self) -> Asked {
        let next = select3(
            self.changes.receive(),
            self.neighbors.receive(),
            self.routers.receive(),
        );
        match next.await {
            Either3::First((pending, request)) => Asked::Change(pending, request),
            Either3::Second((pending, NeighborsRequest)) => Asked::Neighbors(pending),
            Either3::Third((pending, RoutersRequest)) => Asked::Routers(pending),
        }
    }

    /// Answer `asked` with `error`.
    fn refuse(&self, asked: Asked, error: NetworkError) {
        match asked {
            Asked::Change(pending, _) => self.changes.answer(pending, Err(error)),
            Asked::Neighbors(pending) => self.neighbors.answer(pending, Err(error)),
            Asked::Routers(pending) => self.routers.answer(pending, Err(error)),
        }
    }
}

/// The end of Thread that answers the [`Handle`].
pub struct Service {
    requests: Requests,
    status: &'static Status,
    buffers: &'static Buffers,
}

impl Service {
    /// Run Thread on a CPU2 that has a Thread stack.
    pub async fn run(self, thread: Thread<'_>) -> ! {
        let Thread {
            ot,
            cli_rx,
            notif_rx,
            sys,
            flash,
        } = thread;

        // Raised when the stack reports a change (of role, for one), so that
        // a fresh status gets published.
        let state_changed = Signal::new();
        let network = Network {
            ot,
            sys,
            flash,
            requests: self.requests,
            status: self.status,
            buffers: self.buffers,
            state_changed: &state_changed,
        };

        // The stack stalls unless its notifications and its CLI output are
        // acknowledged. (The CLI itself is on CPU1 in this stack version and
        // nothing uses it, but starting the stack never finishes with its
        // channel left unanswered.)
        join3(
            acknowledge_notifications(notif_rx, &state_changed),
            drain_cli(cli_rx),
            network.run(),
        )
        .await
        .0
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

async fn acknowledge_notifications(
    mut notif_rx: ThreadNotifRx<'_>,
    state_changed: &Signal<ThreadModeRawMutex, ()>,
) -> ! {
    loop {
        let notification = notif_rx.receive().await;
        defmt::trace!("cpu2: notification {}", notification.id);
        if notification.id == ffi::MsgId_M0toM4_Enum_t::MSG_M0TOM4_NOTIFY_STATE_CHANGE as u32 {
            state_changed.signal(());
        }
    }
}

async fn drain_cli(mut cli_rx: ThreadCliRx<'_>) -> ! {
    let mut output = [0; 64];
    loop {
        let n = cli_rx.receive(&mut output).await;
        defmt::trace!("cpu2: cli {=[u8]:a}", output[..n]);
    }
}

struct Network<'d> {
    ot: ThreadOt<'d>,
    sys: Sys<'d>,
    flash: Flash<'d, Blocking>,
    requests: Requests,
    status: &'static Status,
    buffers: &'static Buffers,
    state_changed: &'d Signal<ThreadModeRawMutex, ()>,
}

impl Network<'_> {
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

        loop {
            let status = self.status().await;
            self.status.publish(status);

            match select(self.requests.receive(), self.state_changed.wait()).await {
                Either::First(Asked::Change(pending, request)) => {
                    let outcome = match request {
                        Request::Join(dataset) => self.join(&dataset).await,
                        Request::Leave => self.leave().await,
                    };
                    // The status is published before the outcome, so that
                    // whoever asked never reads a status older than the
                    // answer.
                    let status = self.status().await;
                    self.status.publish(status);
                    self.requests.changes.answer(pending, outcome);
                }
                Either::First(Asked::Neighbors(pending)) => {
                    let neighbors = self.neighbors().await;
                    self.requests.neighbors.answer(pending, neighbors);
                }
                Either::First(Asked::Routers(pending)) => {
                    let routers = self.routers().await;
                    self.requests.routers.answer(pending, routers);
                }
                // The status at the top of the loop is all there is to do.
                Either::Second(()) => {}
            }
        }
    }

    /// Start the Thread stack on CPU2.
    async fn start(&mut self) -> NetworkResult {
        match self.sys.shci_c2_thread_init().await {
            Ok(SchiCommandStatus::ShciSuccess) => {}
            _ => return Err(NetworkError::StartFailed),
        }
        self.ot.instance_init_single().await;
        self.ot.set_state_changed_callback().await?;
        Ok(())
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
        persistent_config::save(&mut self.flash, &mut self.sys, &config)
            .await
            .map_err(|_| NetworkError::Storage)
    }

    async fn leave(&mut self) -> NetworkResult {
        self.down().await?;
        persistent_config::erase(&mut self.flash, &mut self.sys)
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
