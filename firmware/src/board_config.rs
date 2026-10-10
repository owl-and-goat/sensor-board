//! The board's configuration ([`BoardConfig`]): which sensors it reads and
//! how often, and how it is powered. The board keeps it in flash
//! ([`persistent_config`]). The host sets it on the board it is attached to,
//! or through that one on a board on its network: the attached board asks
//! all the boards, and the one that is meant answers.
//!
//! [`init`] makes the two ends: the [`Handle`] for the rest of the firmware,
//! and the [`Service`], which runs inside the coprocessor task, because
//! flash is written in step with CPU2.

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

/// How long a board on the network gets to answer, and how long it is left
/// before it is asked again. A question goes to all the boards, and nothing
/// sends one of those a second time when it is lost on the way.
const ANSWER_TIMEOUT: Duration = Duration::from_secs(6);
const ASK_INTERVAL: Duration = Duration::from_secs(2);

/// How long a request gets: at the longest, the wait for a board on the
/// network that does not answer. Less than the host waits for its own
/// answer.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

enum Request {
    Get(BoardId),
    Set(ConfigFor),
}

/// The configuration the board has, as last kept.
type Current = Watch<ThreadModeRawMutex, Option<BoardConfig>, 1>;

struct Shared {
    requests: request::Channel<Request, BoardConfigResult>,
    current: Current,
}

/// Make the ends of the board's configuration: the two that set it, and the
/// look at it for what acts on it. Panics if called a second time.
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
        // Cannot fail: this is the one place a look at it is taken.
        current: shared.current.receiver().unwrap(),
    };
    (Handle { requests: client }, service, monitor)
}

/// A look at the configuration the board has, for what acts on it.
pub struct Monitor {
    current: watch::Receiver<'static, ThreadModeRawMutex, Option<BoardConfig>, 1>,
}

impl Monitor {
    /// The configuration the board has now.
    pub fn config(&mut self) -> Option<BoardConfig> {
        self.current.try_get().flatten()
    }

    /// Wait for the board to be given another configuration, which is the
    /// answer.
    pub async fn changed(&mut self) -> Option<BoardConfig> {
        self.current.changed().await
    }
}

/// How the rest of the firmware gets at a board's configuration.
pub struct Handle {
    requests: request::Client<Request, BoardConfigResult>,
}

impl Handle {
    /// The configuration of `board`: this board, or one on its network,
    /// which is asked.
    pub async fn get(&mut self, board: BoardId) -> BoardConfigResult {
        self.request(Request::Get(board)).await
    }

    /// Have the board that `config` is for keep it: this board, or one on
    /// its network, which is asked to.
    pub async fn set(&mut self, config: ConfigFor) -> ConfigResult {
        self.request(Request::Set(config)).await.map(|_| ())
    }

    async fn request(&mut self, request: Request) -> BoardConfigResult {
        self.requests
            .ask(request, REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(ConfigError::Unresponsive))
    }
}

/// The end of the board's configuration that answers the [`Handle`], and the
/// boards on the network.
pub struct Service {
    requests: request::Server<Request, BoardConfigResult>,
    current: &'static Current,
}

impl Service {
    /// Serve the configuration on a board whose CPU2 runs a wireless stack.
    pub async fn run(self, flash: &radio_flash::Shared<'_>, socket: thread::Socket) -> ! {
        let mut configs = Configs {
            flash,
            socket,
            requests: self.requests,
            current: self.current,
            board: this_board(),
            asking: None,
            asked: 0,
        };
        configs.serve().await
    }

    /// Stand in for it on a board that cannot write its flash, and is on no
    /// network: all it can do is say what it has.
    pub async fn unavailable(self) -> ! {
        let board = this_board();
        loop {
            let (pending, request) = self.requests.receive().await;
            let answer = match request {
                Request::Get(asked) if asked == board => Ok(persistent_config::board_config()),
                _ => Err(ConfigError::Unavailable),
            };
            self.requests.answer(pending, answer);
        }
    }
}

fn this_board() -> BoardId {
    BoardId(embassy_stm32::uid::uid())
}

/// A question to a board on the network, which the host waits for the answer
/// to.
struct Asking {
    pending: request::Pending,
    board: BoardId,
    /// The number the question goes out under, and its answer comes back
    /// under.
    question: u32,
    /// What the board is asked to keep. `None` if it is only asked what it
    /// has.
    config: Option<BoardConfig>,
    /// When it is put again, if no answer has come by then.
    again_at: Instant,
    give_up_at: Instant,
}

impl Asking {
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
    /// What boards say to each other about their configurations.
    socket: thread::Socket,
    requests: request::Server<Request, BoardConfigResult>,
    current: &'static Current,
    /// This board.
    board: BoardId,
    /// What a board on the network is being asked for the host.
    asking: Option<Asking>,
    /// How many questions this board has put, which numbers the next.
    asked: u32,
}

impl Configs<'_, '_> {
    async fn serve(&mut self) -> ! {
        // A question goes to every board, so every board listens for one.
        if let Err(e) = self.socket.listen(true).await {
            defmt::warn!("config: none over the network: {}", e);
        }

        loop {
            let due = self
                .asking
                .as_ref()
                .map(|asking| asking.again_at.min(asking.give_up_at));
            let next = select3(self.requests.receive(), self.socket.receive(), async {
                match due {
                    Some(at) => Timer::at(at).await,
                    None => core::future::pending().await,
                }
            });
            match next.await {
                Either3::First((pending, request)) => self.answer(pending, request).await,
                Either3::Second(received) => self.heard(received).await,
                Either3::Third(()) => self.ask_again().await,
            }
        }
    }

    async fn answer(&mut self, pending: request::Pending, request: Request) {
        // Whoever asks one thing has given up on what it asked before.
        self.asking = None;

        match request {
            Request::Get(board) if board == self.board => {
                let has = persistent_config::board_config();
                self.requests.answer(pending, Ok(has));
            }
            Request::Set(ConfigFor { board, config }) if board == self.board => {
                let kept = self.keep(&config).await;
                self.requests.answer(pending, kept.map(|()| Some(config)));
            }
            Request::Get(board) => self.ask(pending, board, None).await,
            Request::Set(ConfigFor { board, config }) => {
                self.ask(pending, board, Some(config)).await
            }
        }
    }

    /// Keep `config` as this board's configuration.
    async fn keep(&mut self, config: &BoardConfig) -> ConfigResult {
        // A board is asked again when its answer is slow, or lost. Flash is
        // not worn for what it holds already.
        if persistent_config::board_config().as_ref() == Some(config) {
            return Ok(());
        }
        let mut flash = self.flash.lock().await;
        let kept = persistent_config::set_board_config(&mut flash, config).await;
        match kept {
            Ok(()) => {
                defmt::info!("config: kept a new one");
                self.current.sender().send(Some(config.clone()));
            }
            Err(e) => defmt::error!("config: flash: {}", e),
        }
        kept.map_err(|_| ConfigError::Storage)
    }

    /// Put a question to `board` over the network, for the host: to keep
    /// `config`, or with `None` what it has.
    async fn ask(
        &mut self,
        pending: request::Pending,
        board: BoardId,
        config: Option<BoardConfig>,
    ) {
        let now = Instant::now();
        let asking = Asking {
            pending,
            board,
            question: self.asked,
            config,
            again_at: now + ASK_INTERVAL,
            give_up_at: now + ANSWER_TIMEOUT,
        };
        self.asked = self.asked.wrapping_add(1);

        match self.send(None, &asking.message()).await {
            Ok(()) => self.asking = Some(asking),
            Err(e) => {
                let refused = Err(ConfigError::Network(e));
                self.requests.answer(asking.pending, refused);
            }
        }
    }

    /// No answer has come: put the question again, or give it up.
    async fn ask_again(&mut self) {
        let now = Instant::now();
        match &mut self.asking {
            Some(asking) if now < asking.give_up_at => {
                asking.again_at = now + ASK_INTERVAL;
                let question = asking.message();
                // It could be sent the first time, and what comes of this
                // time is the same either way: an answer, or none.
                let _ = self.send(None, &question).await;
            }
            _ => {
                if let Some(asking) = self.asking.take() {
                    self.requests
                        .answer(asking.pending, Err(ConfigError::NoAnswer));
                }
            }
        }
    }

    async fn heard(&mut self, received: thread::Received) {
        let thread::Received { from, datagram } = received;
        match ConfigMessage::decode(&datagram) {
            Some(ConfigMessage::Get { board, question }) if board == self.board => {
                self.tell(from, question).await
            }
            Some(ConfigMessage::Set {
                board,
                question,
                config,
            }) if board == self.board => {
                // Whatever comes of it, the answer is what the board has
                // then, which tells.
                let _ = self.keep(&config).await;
                self.tell(from, question).await
            }
            Some(ConfigMessage::Has {
                board,
                question,
                config,
            }) => self.answered(board, question, config),
            // A question to another board, this board's own to all of them,
            // or the word of a firmware with another layout of all this.
            _ => {}
        }
    }

    /// Tell the board that put `question` what this one has.
    async fn tell(&mut self, asker: thread::Peer, question: u32) {
        let has = ConfigMessage::Has {
            board: self.board,
            question,
            config: persistent_config::board_config(),
        };
        if let Err(e) = self.send(Some(asker), &has).await {
            defmt::debug!("config: not sent: {}", e);
        }
    }

    /// `board` says what it has, in answer to `question`. If that is what
    /// the host waits for, the host gets its answer.
    fn answered(&mut self, board: BoardId, question: u32, has: Option<BoardConfig>) {
        let awaited = |asking: &mut Asking| (asking.board, asking.question) == (board, question);
        // Anything else comes late, to a question that has been given up.
        let Some(asking) = self.asking.take_if(awaited) else {
            return;
        };
        let outcome = match &asking.config {
            // The board was to keep this, and has something else.
            Some(config) if has.as_ref() != Some(config) => Err(ConfigError::Storage),
            _ => Ok(has),
        };
        self.requests.answer(asking.pending, outcome);
    }

    /// Send to one board, or to all of them. Nothing tells whether it
    /// arrives.
    async fn send(&mut self, to: Option<thread::Peer>, message: &ConfigMessage) -> NetworkResult {
        let mut datagram = thread::Datagram::new();
        // Neither can fail: a datagram has room for the longest message.
        let _ = datagram.resize_default(ConfigMessage::MAX_LEN);
        let len = message.encode(&mut datagram).map_or(0, <[u8]>::len);
        datagram.truncate(len);

        match to {
            Some(peer) => self.socket.send_to(peer, datagram).await,
            None => self.socket.broadcast(datagram).await,
        }
    }
}
