//! The board's configuration ([`BoardConfig`]): which sensors it reads and
//! how often, and how it is powered. It is stored in flash
//! ([`persistent_config`]).
//!
//! The host reads and sets the configuration of the attached board over USB.
//! For any other board, the attached board relays the request over the
//! network: it multicasts a [`ConfigMessage`], and the target board replies.
//!
//! [`init`] returns the [`Handle`] for the rest of the firmware, the
//! [`Service`], and a [`Monitor`]. The service runs inside the coprocessor
//! task, because flash writes have to be coordinated with CPU2.

use embassy_futures::select::{Either3, select3};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    watch::{self, Watch},
};
use embassy_time::{Duration, Instant, Timer};
use protocol::{
    BoardConfig, BoardConfigResult, BoardId, ConfigError, ConfigFor, ConfigMessage, ConfigResult,
    NetworkResult,
};
use static_cell::StaticCell;

use crate::{persistent_config, radio_flash, request, thread};

const _: () = assert!(ConfigMessage::MAX_LEN <= thread::MAX_DATAGRAM_LEN);

/// How long to wait for another board's reply, and how often to resend the
/// request in that time. A request is multicast, so nothing retransmits it if
/// it is lost.
const REPLY_TIMEOUT: Duration = Duration::from_secs(6);
const RESEND_INTERVAL: Duration = Duration::from_secs(2);

/// Timeout for a request through the [`Handle`]. Longer than
/// [`REPLY_TIMEOUT`], and shorter than the host's own timeout.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

enum Request {
    Get(BoardId),
    Set(ConfigFor),
}

/// The board's current configuration, updated when a new one is stored.
type Current = Watch<ThreadModeRawMutex, Option<BoardConfig>, 1>;

struct Shared {
    requests: request::Channel<Request, BoardConfigResult>,
    current: Current,
}

/// Create the [`Handle`] and [`Service`] that read and set the configuration,
/// and the [`Monitor`] that watches it. Panics if called a second time.
pub fn init() -> (Handle, Service, Monitor) {
    static SHARED: StaticCell<Shared> = StaticCell::new();
    let shared: &'static Shared = SHARED.init(Shared {
        requests: request::Channel::new(),
        current: Watch::new_with(persistent_config::board_config()),
    });

    let (client, server) = shared.requests.split();
    let service = Service {
        requests: server,
        current: &shared.current,
    };
    let monitor = Monitor {
        // Cannot fail: the `Watch` allows one receiver, and this is the
        // only one.
        current: shared.current.receiver().unwrap(),
    };
    (Handle { requests: client }, service, monitor)
}

/// Watches the board's configuration for changes.
pub struct Monitor {
    current: watch::Receiver<'static, ThreadModeRawMutex, Option<BoardConfig>, 1>,
}

impl Monitor {
    /// The board's current configuration.
    pub fn config(&mut self) -> Option<BoardConfig> {
        self.current.try_get().flatten()
    }

    /// Wait for the configuration to change. Returns the new one.
    pub async fn changed(&mut self) -> Option<BoardConfig> {
        self.current.changed().await
    }
}

/// Reads and sets the configuration of this board, or of another board on
/// its network.
pub struct Handle {
    requests: request::Client<Request, BoardConfigResult>,
}

impl Handle {
    /// Get the configuration of `board`. Another board is asked over the
    /// network.
    pub async fn get(&mut self, board: BoardId) -> BoardConfigResult {
        self.request(Request::Get(board)).await
    }

    /// Store `config.config` on `config.board`. Another board is asked over
    /// the network.
    pub async fn set(&mut self, config: ConfigFor) -> ConfigResult {
        self.request(Request::Set(config)).await.map(|_| ())
    }

    async fn request(&mut self, request: Request) -> BoardConfigResult {
        self.requests
            .call(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(ConfigError::Unresponsive))
    }
}

/// Serves requests from the [`Handle`] and from other boards.
pub struct Service {
    requests: request::Server<Request, BoardConfigResult>,
    current: &'static Current,
}

impl Service {
    /// Run the service on a board whose CPU2 runs a wireless stack.
    pub async fn run(self, flash: &radio_flash::Shared<'_>, socket: thread::Socket) -> ! {
        let mut configs = Configs {
            flash,
            socket,
            requests: self.requests,
            current: self.current,
            board: this_board(),
            forwarded: None,
            next_question: 0,
        };
        configs.serve().await
    }

    /// Run in place of [`Service::run`] on a board without a wireless stack.
    /// Such a board cannot write flash or reach the network, so only a `Get`
    /// for this board succeeds.
    pub async fn unavailable(self) -> ! {
        let board = this_board();
        loop {
            let (pending, request) = self.requests.receive().await;
            let response = match request {
                Request::Get(target) if target == board => Ok(persistent_config::board_config()),
                _ => Err(ConfigError::Unavailable),
            };
            self.requests.respond(pending, response);
        }
    }
}

fn this_board() -> BoardId {
    BoardId(embassy_stm32::uid::uid())
}

/// A pending request to another board, made on behalf of the host.
struct Forwarded {
    pending: request::Pending,
    board: BoardId,
    /// Identifies the request. The reply carries the same number.
    question: u32,
    /// The configuration to store, for a `Set`. `None` for a `Get`.
    config: Option<BoardConfig>,
    /// When to resend the request, if there is no reply by then.
    resend_at: Instant,
    deadline: Instant,
}

impl Forwarded {
    fn message(&self) -> ConfigMessage {
        let (board, question) = (self.board, self.question);
        match self.config.clone() {
            Some(config) => ConfigMessage::Set {
                board,
                question,
                config,
            },
            None => ConfigMessage::Get { board, question },
        }
    }
}

struct Configs<'a, 'd> {
    flash: &'a radio_flash::Shared<'d>,
    /// The UDP socket for [`ConfigMessage`]s.
    socket: thread::Socket,
    requests: request::Server<Request, BoardConfigResult>,
    current: &'static Current,
    /// This board's ID.
    board: BoardId,
    /// The pending request to another board, if there is one.
    forwarded: Option<Forwarded>,
    /// The number for the next request. Incremented for each one.
    next_question: u32,
}

impl Configs<'_, '_> {
    async fn serve(&mut self) -> ! {
        // Requests are multicast, so every board has to listen for them.
        if let Err(e) = self.socket.listen(true).await {
            defmt::warn!("config: cannot listen for requests: {}", e);
        }

        loop {
            let due = self
                .forwarded
                .as_ref()
                .map(|forwarded| forwarded.resend_at.min(forwarded.deadline));
            let next = select3(self.requests.receive(), self.socket.receive(), async {
                match due {
                    Some(at) => Timer::at(at).await,
                    None => core::future::pending().await,
                }
            });
            match next.await {
                Either3::First((pending, request)) => self.handle_request(pending, request).await,
                Either3::Second(received) => self.handle_datagram(received).await,
                Either3::Third(()) => self.resend().await,
            }
        }
    }

    async fn handle_request(&mut self, pending: request::Pending, request: Request) {
        // A new request replaces the pending one, whose caller has stopped
        // waiting.
        self.forwarded = None;

        match request {
            Request::Get(board) if board == self.board => {
                let stored = persistent_config::board_config();
                self.requests.respond(pending, Ok(stored));
            }
            Request::Set(ConfigFor { board, config }) if board == self.board => {
                let result = self.store(&config).await;
                self.requests
                    .respond(pending, result.map(|()| Some(config)));
            }
            Request::Get(board) => self.forward(pending, board, None).await,
            Request::Set(ConfigFor { board, config }) => {
                self.forward(pending, board, Some(config)).await
            }
        }
    }

    /// Store `config` as this board's configuration.
    async fn store(&mut self, config: &BoardConfig) -> ConfigResult {
        // A request is resent when the reply is slow or lost. Do not rewrite
        // flash with what it already holds.
        if persistent_config::board_config().as_ref() == Some(config) {
            return Ok(());
        }
        let mut flash = self.flash.lock().await;
        let written = persistent_config::set_board_config(&mut flash, config).await;
        match written {
            Ok(()) => {
                defmt::info!("config: stored a new configuration");
                self.current.sender().send(Some(config.clone()));
            }
            Err(e) => defmt::error!("config: flash write failed: {}", e),
        }
        written.map_err(|_| ConfigError::Storage)
    }

    /// Send a request to `board` over the network: a `Set` of `config`, or
    /// with `None` a `Get`.
    async fn forward(
        &mut self,
        pending: request::Pending,
        board: BoardId,
        config: Option<BoardConfig>,
    ) {
        let now = Instant::now();
        let forwarded = Forwarded {
            pending,
            board,
            question: self.next_question,
            config,
            resend_at: now + RESEND_INTERVAL,
            deadline: now + REPLY_TIMEOUT,
        };
        self.next_question = self.next_question.wrapping_add(1);

        match self.send(None, &forwarded.message()).await {
            Ok(()) => self.forwarded = Some(forwarded),
            Err(e) => {
                let error = Err(ConfigError::Network(e));
                self.requests.respond(forwarded.pending, error);
            }
        }
    }

    /// The reply has not arrived: resend the request, or give up.
    async fn resend(&mut self) {
        let now = Instant::now();
        match &mut self.forwarded {
            Some(forwarded) if now < forwarded.deadline => {
                forwarded.resend_at = now + RESEND_INTERVAL;
                let question = forwarded.message();
                // A send error is ignored here: the first send worked, and
                // the request times out anyway if no reply arrives.
                let _ = self.send(None, &question).await;
            }
            _ => {
                if let Some(forwarded) = self.forwarded.take() {
                    self.requests
                        .respond(forwarded.pending, Err(ConfigError::NoAnswer));
                }
            }
        }
    }

    async fn handle_datagram(&mut self, received: thread::Received) {
        let thread::Received { from, datagram } = received;
        match ConfigMessage::decode(&datagram) {
            Some(ConfigMessage::Get { board, question }) if board == self.board => {
                self.reply(from, question).await
            }
            Some(ConfigMessage::Set {
                board,
                question,
                config,
            }) if board == self.board => {
                // The result is not needed: the reply carries what the board
                // now has stored, which shows whether the write worked.
                let _ = self.store(&config).await;
                self.reply(from, question).await
            }
            Some(ConfigMessage::Has {
                board,
                question,
                config,
            }) => self.handle_reply(board, question, config),
            // A request for another board, this board's own multicast, or a
            // message from a firmware with a different layout.
            _ => {}
        }
    }

    /// Reply to `peer`'s request number `question` with this board's
    /// configuration.
    async fn reply(&mut self, peer: thread::Peer, question: u32) {
        let message = ConfigMessage::Has {
            board: self.board,
            question,
            config: persistent_config::board_config(),
        };
        if let Err(e) = self.send(Some(peer), &message).await {
            defmt::debug!("config: could not send a reply: {}", e);
        }
    }

    /// Handle a reply from `board` to request number `question`. If it
    /// matches the pending request, complete that request.
    fn handle_reply(&mut self, board: BoardId, question: u32, stored: Option<BoardConfig>) {
        let matches =
            |forwarded: &mut Forwarded| (forwarded.board, forwarded.question) == (board, question);
        // Otherwise it is a late reply to a request that was given up.
        let Some(forwarded) = self.forwarded.take_if(matches) else {
            return;
        };
        let outcome = match &forwarded.config {
            // The board did not store the configuration it was sent.
            Some(config) if stored.as_ref() != Some(config) => Err(ConfigError::Storage),
            _ => Ok(stored),
        };
        self.requests.respond(forwarded.pending, outcome);
    }

    /// Send `message` to one board, or with `None` to every board. Delivery
    /// is not confirmed.
    async fn send(&mut self, to: Option<thread::Peer>, message: &ConfigMessage) -> NetworkResult {
        let mut datagram = thread::Datagram::new();
        // Neither call can fail: a datagram has room for the longest message.
        let _ = datagram.resize_default(ConfigMessage::MAX_LEN);
        let len = message.encode(&mut datagram).map_or(0, <[u8]>::len);
        datagram.truncate(len);

        match to {
            Some(peer) => self.socket.send_to(peer, datagram).await,
            None => self.socket.broadcast(datagram).await,
        }
    }
}
