//! The board's metrics, for Prometheus: what its sensors read, and how
//! getting that to Prometheus goes. How they get there depends on how the
//! board is powered ([`PowerMode`]). A board on aux power serves them to
//! whoever scrapes it, at `/metrics` on [`SCRAPE_PORT`]. A board on battery
//! pushes them to the Pushgateway it is configured with, each time it has
//! read a sensor. A board that has no configuration does neither.
//!
//! Which sensors are read, and how often, is in the configuration too.

use core::cell::RefCell;
use core::fmt::{self, Write as _};
use core::net::SocketAddrV6;

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{Mutex, raw::ThreadModeRawMutex};
use embassy_time::{Duration, Instant, Timer};
use protocol::{BoardConfig, BoardId, MetricsChunk, PowerMode, Sensor};
use static_cell::StaticCell;
use tinymetrics::{Counter, FmtLabels, IntGauge, MetricBuilder, MetricFamily};

use crate::{
    board_config, http,
    sensor::{
        self,
        capacitance::{CapacitanceSensor, Channel},
    },
    thread, update,
};

/// The TCP port a board on aux power serves its metrics on.
const SCRAPE_PORT: u16 = 9469;

/// The job that a board on battery pushes its metrics as. The instance is
/// the board.
const JOB: &str = "sensor_board";

/// How long a Pushgateway that did not take a push is left alone.
const PUSH_RETRY: Duration = Duration::from_secs(60);

/// How long to leave it when connections cannot be waited for, which is so
/// until Thread has started.
const RETRY: Duration = Duration::from_secs(5);

/// No sensor is read more often than this, whatever its configuration says.
const MIN_INTERVAL: Duration = Duration::from_millis(100);

/// The sensors this firmware has a driver for: the channels of the
/// capacitance sensor.
const CAPACITANCE: [(Sensor, Channel); 4] = [
    (Sensor::Capacitance0, Channel::Ch0),
    (Sensor::Capacitance1, Channel::Ch1),
    (Sensor::Capacitance2, Channel::Ch2),
    (Sensor::Capacitance3, Channel::Ch3),
];

/// The labels of a metric of the board as a whole.
#[derive(PartialEq)]
struct OfBoard(BoardId);

impl FmtLabels for OfBoard {
    fn fmt_labels(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        write!(writer, "board=\"{}\"", self.0)
    }
}

/// The labels of a metric of one channel of the capacitance sensor.
#[derive(PartialEq)]
struct OfChannel {
    board: BoardId,
    channel: usize,
}

impl FmtLabels for OfChannel {
    fn fmt_labels(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        let OfChannel { board, channel } = self;
        write!(writer, "board=\"{board}\",channel=\"{channel}\"")
    }
}

/// Room for the metrics as text.
type Text = heapless::String<4096>;

type OfBoardFamily<M> = MetricFamily<'static, M, 1, OfBoard>;
type OfChannelFamily<M> = MetricFamily<'static, M, { CAPACITANCE.len() }, OfChannel>;

struct Metrics {
    firmware_build: OfBoardFamily<IntGauge>,
    capacitance: OfChannelFamily<IntGauge>,
    capacitance_errors: OfChannelFamily<Counter>,
    scrapes: OfBoardFamily<Counter>,
    pushes: OfBoardFamily<Counter>,
    push_failures: OfBoardFamily<Counter>,
    push_status: OfBoardFamily<IntGauge>,
}

impl Metrics {
    fn new() -> Self {
        Metrics {
            firmware_build: MetricBuilder::new("sensor_board_firmware_build")
                .with_help(
                    "The build of the firmware a board runs: when it was built, in Unix time.",
                )
                .build_labeled(),
            capacitance: MetricBuilder::new("sensor_board_capacitance")
                .with_help("Raw reading of a capacitance channel.")
                .build_labeled(),
            capacitance_errors: MetricBuilder::new("sensor_board_capacitance_errors_total")
                .with_help("Readings of a capacitance channel that failed.")
                .build_labeled(),
            scrapes: MetricBuilder::new("sensor_board_scrapes_total")
                .with_help("Times a board was asked for its metrics over the network.")
                .build_labeled(),
            pushes: MetricBuilder::new("sensor_board_pushes_total")
                .with_help("Times a board set out to push its metrics to its Pushgateway.")
                .build_labeled(),
            push_failures: MetricBuilder::new("sensor_board_push_failures_total")
                .with_help("Pushes that the Pushgateway did not take.")
                .build_labeled(),
            push_status: MetricBuilder::new("sensor_board_push_status")
                .with_help(
                    "The HTTP status of the Pushgateway's answer to the last push, or 0 for none.",
                )
                .build_labeled(),
        }
    }

    /// The metrics in Prometheus's text format.
    fn write(&self, text: &mut impl fmt::Write) -> fmt::Result {
        self.firmware_build.fmt_metric(text)?;
        self.capacitance.fmt_metric(text)?;
        self.capacitance_errors.fmt_metric(text)?;
        self.scrapes.fmt_metric(text)?;
        self.pushes.fmt_metric(text)?;
        self.push_failures.fmt_metric(text)?;
        self.push_status.fmt_metric(text)
    }
}

/// The metrics, as the task that records them and the handle that shows them
/// have them between them. Recording needs no more than a look at them. The
/// lock is for starting them afresh, and for writing them down as they are
/// at one moment.
struct Recorded(Mutex<ThreadModeRawMutex, RefCell<Metrics>>);

impl Recorded {
    fn with<R>(&self, f: impl FnOnce(&Metrics) -> R) -> R {
        self.0.lock(|metrics| f(&metrics.borrow()))
    }

    /// Forget everything that was recorded.
    fn start_afresh(&self) {
        self.0.lock(|metrics| metrics.replace(Metrics::new()));
    }
}

pub struct Builder {
    /// The TCP connection a scrape comes in on, or a push goes out on.
    pub tcp: thread::Tcp,
    pub config: board_config::Monitor,
    pub capacitance: &'static sensor::Shared<CapacitanceSensor<'static>>,
}

impl Builder {
    /// Panics if called a second time: the firmware has one metrics task.
    pub fn init(self) -> (Task, Handle) {
        static RECORDED: StaticCell<Recorded> = StaticCell::new();
        static TEXT: StaticCell<Text> = StaticCell::new();
        let recorded: &'static Recorded =
            RECORDED.init(Recorded(Mutex::new(RefCell::new(Metrics::new()))));

        let task = Task {
            config: self.config,
            exporter: Exporter {
                tcp: self.tcp,
                capacitance: self.capacitance,
                board: BoardId(embassy_stm32::uid::uid()),
                metrics: recorded,
                text: TEXT.init(Text::new()),
                polls: [None; CAPACITANCE.len()],
            },
        };
        (task, Handle { metrics: recorded })
    }
}

/// How the rest of the firmware gets at the metrics.
pub struct Handle {
    metrics: &'static Recorded,
}

impl Handle {
    /// The piece at `offset` of the metrics as text, as they are now.
    pub fn text(&self, offset: u32) -> MetricsChunk {
        let mut window = Window {
            before: offset as usize,
            chunk: MetricsChunk {
                text: heapless::String::new(),
                more: false,
            },
        };
        // Cannot fail: the window takes whatever it is given.
        let _ = self.metrics.with(|metrics| metrics.write(&mut window));
        window.chunk
    }
}

/// Keeps the piece of what is written to it that a chunk has room for, from
/// an offset on.
struct Window {
    /// How much is still to go by before the piece starts.
    before: usize,
    chunk: MetricsChunk,
}

impl fmt::Write for Window {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for character in text.chars() {
            if self.before > 0 {
                self.before = self.before.saturating_sub(character.len_utf8());
            } else if self.chunk.more || self.chunk.text.push(character).is_err() {
                self.chunk.more = true;
            }
        }
        Ok(())
    }
}

pub struct Task {
    config: board_config::Monitor,
    exporter: Exporter,
}

impl Task {
    pub async fn run(mut self) -> ! {
        let mut config = self.config.config();
        loop {
            // Until the board is given another configuration.
            let export = self.exporter.export(config.as_ref());
            config = match select(self.config.changed(), export).await {
                Either::First(config) => config,
                Either::Second(never) => never,
            };
            self.exporter.stop().await;
        }
    }
}

/// When a sensor is next read, and how long after that again.
#[derive(Clone, Copy)]
struct Poll {
    due: Instant,
    every: Duration,
}

struct Exporter {
    tcp: thread::Tcp,
    capacitance: &'static sensor::Shared<CapacitanceSensor<'static>>,
    /// This board.
    board: BoardId,
    metrics: &'static Recorded,
    /// The metrics as they were last written down, to be sent.
    text: &'static mut Text,
    /// One for each of [`CAPACITANCE`], at its index: `None` for a sensor
    /// that is not read.
    polls: [Option<Poll>; CAPACITANCE.len()],
}

impl Exporter {
    /// Read the sensors and get the metrics to Prometheus, the way `config`
    /// has it.
    async fn export(&mut self, config: Option<&BoardConfig>) -> ! {
        // Nothing stays of what another configuration had the board read.
        self.metrics.start_afresh();
        self.metrics.with(|metrics| {
            if let Some(build) = metrics.firmware_build.register(OfBoard(self.board)) {
                build.set_value(update::build().0 as usize);
            }
        });

        let now = Instant::now();
        for (poll, (sensor, _)) in self.polls.iter_mut().zip(CAPACITANCE) {
            let sensor = config.and_then(|config| config.sensor_config.0[sensor]);
            *poll = sensor.map(|sensor| {
                let micros = sensor.poll_interval.as_micros().try_into();
                let every = Duration::from_micros(micros.unwrap_or(u64::MAX));
                Poll {
                    due: now,
                    every: every.max(MIN_INTERVAL),
                }
            });
        }

        match config {
            Some(config) => match config.power_mode {
                PowerMode::Aux => self.serve().await,
                PowerMode::Battery => self.push(config.pushgateway).await,
            },
            None => self.push(None).await,
        }
    }

    /// Leave the network as a board without a configuration has it.
    async fn stop(&mut self) {
        self.tcp.close().await;
        if let Err(e) = self.tcp.listen(None).await {
            defmt::debug!("metrics: still listening: {}", e);
        }
    }

    /// Serve the metrics to whoever scrapes them.
    async fn serve(&mut self) -> ! {
        self.count(|metrics| &metrics.scrapes, 0);
        loop {
            let next = select(Self::poll_due(&self.polls), Self::scrape(&mut self.tcp));
            match next.await {
                Either::First(sensor) => self.read(sensor).await,
                Either::Second(()) => self.scraped().await,
            }
        }
    }

    /// Wait for a connection to come in.
    async fn scrape(tcp: &mut thread::Tcp) {
        loop {
            let came_in = async {
                // Asked for each time, in case Thread had not started the
                // last time. It costs nothing where it is listening already.
                tcp.listen(Some(SCRAPE_PORT)).await?;
                tcp.accept().await
            };
            match came_in.await {
                Ok(()) => return,
                Err(e) => {
                    defmt::debug!("metrics: no scrapes for now: {}", e);
                    Timer::after(RETRY).await;
                }
            }
        }
    }

    /// Answer the request that has come in.
    async fn scraped(&mut self) {
        let answered = async {
            match http::request(&mut self.tcp).await? {
                http::Asked::Metrics => {
                    self.count(|metrics| &metrics.scrapes, 1);
                    let status = match self.write() {
                        Ok(()) => http::Status::Ok,
                        Err(fmt::Error) => http::Status::InternalServerError,
                    };
                    http::respond(&mut self.tcp, status, self.text).await
                }
                http::Asked::Other => {
                    let body = "The metrics are at /metrics.\n";
                    http::respond(&mut self.tcp, http::Status::NotFound, body).await
                }
            }
        };
        if let Err(e) = answered.await {
            defmt::debug!("metrics: a scrape came to nothing: {}", e);
        }
        self.tcp.close().await;
    }

    /// Push the metrics to `gateway` each time a sensor has been read. With
    /// no gateway, the sensors are only read.
    async fn push(&mut self, gateway: Option<SocketAddrV6>) -> ! {
        // A gateway that did not take a push is left alone for a while.
        let mut not_before = Instant::now();
        loop {
            let sensor = Self::poll_due(&self.polls).await;
            self.read(sensor).await;
            // What else is due by now goes in the same push.
            let now = Instant::now();
            for sensor in 0..self.polls.len() {
                if self.polls[sensor].is_some_and(|poll| poll.due <= now) {
                    self.read(sensor).await;
                }
            }

            if let Some(gateway) = gateway
                && Instant::now() >= not_before
                && !self.push_to(gateway).await
            {
                not_before = Instant::now() + PUSH_RETRY;
            }
        }
    }

    /// Whether `gateway` took the metrics.
    async fn push_to(&mut self, gateway: SocketAddrV6) -> bool {
        self.count(|metrics| &metrics.pushes, 1);

        let pushed = async {
            self.write().map_err(|_| http::Error::Malformed)?;
            let mut path: heapless::String<64> = heapless::String::new();
            let written = write!(path, "/metrics/job/{JOB}/instance/{}", self.board);
            written.map_err(|_| http::Error::Malformed)?;
            http::put(&mut self.tcp, gateway, &path, self.text).await
        };
        let status = pushed.await;
        self.tcp.close().await;

        let taken = matches!(status, Ok(200..300));
        if let Err(e) = status {
            defmt::debug!("metrics: a push came to nothing: {}", e);
        }
        self.count(|metrics| &metrics.push_failures, usize::from(!taken));
        self.metrics.with(|metrics| {
            if let Some(last) = metrics.push_status.register(OfBoard(self.board)) {
                last.set_value(status.map_or(0, usize::from));
            }
        });
        taken
    }

    /// Wait for the next sensor to be due, and answer which: its index in
    /// [`CAPACITANCE`].
    async fn poll_due(polls: &[Option<Poll>]) -> usize {
        let dues = polls.iter().enumerate();
        let next = dues.filter_map(|(sensor, poll)| Some(((*poll)?.due, sensor)));
        match next.min() {
            Some((due, sensor)) => {
                Timer::at(due).await;
                sensor
            }
            None => core::future::pending().await,
        }
    }

    /// Read the sensor at `sensor` in [`CAPACITANCE`].
    async fn read(&mut self, sensor: usize) {
        if let Some(poll) = &mut self.polls[sensor] {
            poll.due = Instant::now() + poll.every;
        }
        let (_, channel) = CAPACITANCE[sensor];
        let labels = || OfChannel {
            board: self.board,
            channel: sensor,
        };

        let reading = {
            let mut capacitance = self.capacitance.lock().await;
            capacitance.read_channel_capacitance(channel).await
        };
        self.metrics.with(|metrics| {
            if let Some(errors) = metrics.capacitance_errors.register(labels()) {
                errors.fetch_add(usize::from(reading.is_err()));
            }
            if let (Ok(value), Some(gauge)) = (reading, metrics.capacitance.register(labels())) {
                gauge.set_value(value as usize);
            }
        });
    }

    /// Add `n` to one of the counters of the board as a whole.
    fn count(&self, counter: impl FnOnce(&Metrics) -> &OfBoardFamily<Counter>, n: usize) {
        self.metrics.with(|metrics| {
            if let Some(counter) = counter(metrics).register(OfBoard(self.board)) {
                counter.fetch_add(n);
            }
        });
    }

    /// Write the metrics down as they are now, to be sent. An error if there
    /// is more of them than there is room for, which leaves nothing written.
    fn write(&mut self) -> fmt::Result {
        self.text.clear();
        let written = self.metrics.with(|metrics| metrics.write(self.text));
        if written.is_err() {
            defmt::error!("metrics: more text than there is room for");
            self.text.clear();
        }
        written
    }
}
