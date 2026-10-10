//! Board discovery: finding the boards on a Thread network.
//!
//! Every board listens for a discovery request and replies with its serial
//! number, its RLOC16 and its addresses. A request comes from an attached
//! board, which multicasts one when the host asks and forwards each reply
//! over USB. It can also come from a host that reaches the board through a
//! border router.

use defmt::debug;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Timer};
use protocol::{BoardId, DiscoveryMessage, Member, NetworkError, NetworkResult, NetworkStatus};
use static_cell::StaticCell;

use crate::{request, rpc, thread};

const _: () = assert!(DiscoveryMessage::MAX_LEN <= thread::MAX_DATAGRAM_LEN);

/// Timeout for a request to discover. It covers one [`thread::Socket`]
/// request, which has its own timeout. Shorter than the host's own timeout.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

/// Delay before trying again to listen. The first attempts fail while the
/// Thread stack is still starting.
const LISTEN_RETRY: Duration = Duration::from_secs(2);

/// A request from the handle to multicast a discovery request.
struct Discover;

pub struct Builder {
    /// The UDP socket for discovery messages.
    pub socket: thread::Socket,
    /// The source of this board's RLOC16 and addresses.
    pub network: thread::Monitor,
}

impl Builder {
    /// Panics if called a second time: the firmware has one discovery task.
    pub fn init(self) -> (Task, Handle) {
        static REQUESTS: StaticCell<request::Channel<Discover, NetworkResult>> = StaticCell::new();
        let (client, server) = REQUESTS.init(request::Channel::new()).split();

        let task = Task {
            socket: self.socket,
            network: self.network,
            requests: server,
            board: BoardId(embassy_stm32::uid::uid()),
            listening: false,
        };
        (task, Handle { requests: client })
    }
}

/// Starts a discovery from the rest of the firmware.
pub struct Handle {
    requests: request::Client<Discover, NetworkResult>,
}

impl Handle {
    /// Ask every board on the network to reply. The task forwards the
    /// replies to the host.
    pub async fn discover(&mut self) -> NetworkResult {
        self.requests
            .call(Discover, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }
}

pub struct Task {
    socket: thread::Socket,
    network: thread::Monitor,
    requests: request::Server<Discover, NetworkResult>,
    board: BoardId,
    /// Whether the socket is bound to the discovery port.
    listening: bool,
}

impl Task {
    pub async fn run(mut self, mut publisher: rpc::Publisher) -> ! {
        loop {
            // Every board replies to discovery requests, so every board
            // listens.
            if !self.listening {
                self.listening = self.socket.listen(true).await.is_ok();
            }
            let listening = self.listening;
            let retry = async {
                if listening {
                    core::future::pending().await
                } else {
                    Timer::after(LISTEN_RETRY).await
                }
            };

            let next = select3(self.requests.receive(), self.socket.receive(), retry);
            match next.await {
                Either3::First((pending, Discover)) => {
                    let sent = self.send(None, &DiscoveryMessage::Request).await;
                    self.requests.respond(pending, sent);
                }
                Either3::Second(received) => match DiscoveryMessage::decode(&received.datagram) {
                    Some(DiscoveryMessage::Request) => self.reply(received.from).await,
                    Some(DiscoveryMessage::Reply(member)) => {
                        publisher.board_discovered(&member).await
                    }
                    None => debug!("discovery: a datagram with another layout"),
                },
                Either3::Third(()) => {}
            }
        }
    }

    /// Describe this board to `peer`, which sent a request.
    async fn reply(&mut self, peer: thread::Peer) {
        // A request only reaches a board that is on a network.
        let NetworkStatus::Configured(link) = self.network.status() else {
            return;
        };
        let member = Member {
            board: self.board,
            rloc16: link.rloc16,
            addresses: self.network.addresses(),
        };
        let reply = DiscoveryMessage::Reply(member);
        if let Err(e) = self.send(Some(peer), &reply).await {
            debug!("discovery: reply not sent: {}", e);
        }
    }

    /// Send to one peer, or to every board. Delivery is not confirmed.
    async fn send(
        &mut self,
        to: Option<thread::Peer>,
        message: &DiscoveryMessage,
    ) -> NetworkResult {
        let mut datagram = thread::Datagram::new();
        // Neither can fail: a datagram has room for the longest message.
        let _ = datagram.resize_default(DiscoveryMessage::MAX_LEN);
        let len = message.encode(&mut datagram).map_or(0, <[u8]>::len);
        datagram.truncate(len);

        match to {
            Some(peer) => self.socket.send_to(peer, datagram).await,
            None => self.socket.broadcast(datagram).await,
        }
    }
}
