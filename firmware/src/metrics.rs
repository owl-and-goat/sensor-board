//! Prometheus metrics: sensor readings, and counters for the export itself.
//! How a board exports them depends on its [`PowerMode`]:
//!
//! - `Aux`: it serves them at `GET /metrics` on [`SCRAPE_PORT`].
//! - `Battery`: it pushes them to the configured Pushgateway after every
//!   sensor read. It has no scrape endpoint, so it registers a service of
//!   its own for the CLI to find it by.
//!
//! A board without a configuration is taken to be on aux power, and reads no
//! sensors. The configuration also sets which sensors are read and how often.

use core::cell::RefCell;
use core::fmt::{self, Write as _};
use core::net::SocketAddrV6;

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{Mutex, raw::ThreadModeRawMutex};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use fixed::{
    traits::Fixed,
    types::{I14F50, I16F16},
};
use nanofish::{HttpHandler, HttpMethod, HttpRequest, HttpResponse, StatusCode};
use protocol::{
    BoardConfig, BoardId, MetricsChunk, PowerMode, Precision, Sensor, TempRh, TempRhReq,
};
use static_cell::StaticCell;
use tinymetrics::{Counter, FixedGauge, FmtLabels, IntGauge, MetricBuilder, MetricFamily};

use crate::{
    board_config, http,
    sensor::{
        self,
        capacitance::{self, CapacitanceSensor},
        color::{self, ColorSensor},
        mic::Mic,
        temp_rh::TempRhSensor,
    },
    supply, thread, update,
};

/// TCP port of the scrape endpoint.
const SCRAPE_PORT: u16 = 9469;

/// What the scrape endpoint is registered as, so that Prometheus can find it.
/// The name is the board's alone, because Prometheus labels the metrics with
/// it. So the lease is short: it is how long the board cannot register for
/// after a restart.
const SCRAPE_SERVICE: thread::SrpService = thread::SrpService {
    kind: protocol::SCRAPE_SERVICE,
    port: SCRAPE_PORT,
    name: thread::SrpName::Board,
    lease: Duration::from_secs(120),
};

/// What a board that pushes registers, so that the CLI can find it. Each
/// renewal costs a board on battery an exchange with the border router, so
/// the lease is long. For that the name has to differ at every start.
const BOARD_SERVICE: thread::SrpService = thread::SrpService {
    kind: protocol::BOARD_SERVICE,
    port: protocol::CONFIG_PORT,
    name: thread::SrpName::BoardAndStart,
    lease: Duration::from_secs(3600),
};

/// The Pushgateway job name. The instance is the board's serial number.
const JOB: &str = "sensor_board";

/// Delay before the next push after a failed one.
const PUSH_RETRY: Duration = Duration::from_secs(60);

/// How long a board that pushes waits for its network before it measures.
/// After that it goes on without, and the push fails if the network is not
/// there.
const NETWORK_WAIT: Duration = Duration::from_secs(30);

/// Delay before trying to listen or to register again. Both fail until
/// Thread has started.
const RETRY: Duration = Duration::from_secs(5);

/// The shortest poll interval. A shorter configured interval is raised to it.
const MIN_INTERVAL: Duration = Duration::from_millis(100);

/// The sensors that have a driver.
const SENSORS: [Sensor; 8] = [
    Sensor::Capacitance0,
    Sensor::Capacitance1,
    Sensor::Capacitance2,
    Sensor::Capacitance3,
    Sensor::Color,
    Sensor::Temperature,
    Sensor::Humidity,
    Sensor::Sound,
];

/// The capacitance channels, indexed by the channel number in their metrics.
const CAPACITANCE: [capacitance::Channel; 4] = [
    capacitance::Channel::Ch0,
    capacitance::Channel::Ch1,
    capacitance::Channel::Ch2,
    capacitance::Channel::Ch3,
];

/// The color channels, and the channel names in their metrics.
const COLOR: [(color::Channel, &str); 5] = [
    (color::Channel::Red, "red"),
    (color::Channel::Green, "green"),
    (color::Channel::Blue, "blue"),
    (color::Channel::White, "white"),
    (color::Channel::Infrared, "infrared"),
];

/// Labels of a per-board metric.
#[derive(PartialEq)]
struct BoardLabels(BoardId);

impl FmtLabels for BoardLabels {
    fn fmt_labels(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        write!(writer, "board=\"{}\"", self.0)
    }
}

/// Labels of the firmware build metric: the board, and the Git commit the
/// firmware was built from.
#[derive(PartialEq)]
struct BuildLabels(BoardId);

impl FmtLabels for BuildLabels {
    fn fmt_labels(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        write!(
            writer,
            "board=\"{}\",commit=\"{}\"",
            self.0,
            env!("BUILD_COMMIT")
        )
    }
}

/// Labels of a per-channel sensor metric.
#[derive(PartialEq)]
struct ChannelLabels<C> {
    board: BoardId,
    channel: C,
}

impl<C: fmt::Display> FmtLabels for ChannelLabels<C> {
    fn fmt_labels(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        let ChannelLabels { board, channel } = self;
        write!(writer, "board=\"{board}\",channel=\"{channel}\"")
    }
}

/// The type of the temperature, humidity and sound level gauges. Its range
/// is -32768 to 32768, in steps of 1.5e-5.
type Decimal = I16F16;

/// The type of the capacitance gauge, in farads. Its range is -8192 F to
/// 8192 F, in steps of 8.9e-16 F.
type Farads = I14F50;

/// Convert a reading to the fixed-point type of its gauge, saturating at the
/// type's limits. `None` if the reading is NaN.
fn fixed<F: Fixed>(value: f32) -> Option<F> {
    (!value.is_nan()).then(|| F::saturating_from_num(value))
}

/// How much of the text format to render.
#[derive(Clone, Copy)]
enum Detail {
    /// Everything, for a scrape: Prometheus keeps each metric's help text.
    Full,
    /// The samples, and the `# TYPE` line of each family that has any. The
    /// Pushgateway needs nothing else, and a shorter text makes a push
    /// faster. The type lines stay because the Pushgateway rejects an
    /// untyped metric when another group has the same metric with a type.
    Samples,
}

/// The longest line of the text format: a help line, or a sample with its
/// labels.
const LINE_LEN: usize = 192;

/// A writer that keeps only the sample lines of the text format, each
/// family's samples preceded by its `# TYPE` line. It drops the other
/// comment lines, the empty lines, and the type line of a family that has no
/// samples.
struct Samples<'a, W> {
    out: &'a mut W,
    /// The line being written.
    line: heapless::String<LINE_LEN>,
    /// The `# TYPE` line of the current family, until its first sample.
    family_type: heapless::String<LINE_LEN>,
}

impl<'a, W: fmt::Write> Samples<'a, W> {
    fn new(out: &'a mut W) -> Self {
        Samples {
            out,
            line: heapless::String::new(),
            family_type: heapless::String::new(),
        }
    }

    /// Write out or drop the line that has just ended.
    fn end_line(&mut self) -> fmt::Result {
        let line = self.line.as_str();
        if line.starts_with("# TYPE ") {
            self.family_type.clear();
            self.family_type.push_str(line).map_err(|_| fmt::Error)?;
        } else if !line.is_empty() && !line.starts_with('#') {
            if !self.family_type.is_empty() {
                writeln!(self.out, "{}", self.family_type)?;
                self.family_type.clear();
            }
            writeln!(self.out, "{line}")?;
        }
        self.line.clear();
        Ok(())
    }
}

impl<W: fmt::Write> fmt::Write for Samples<'_, W> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let mut rest = s;
        while let Some(end) = rest.find('\n') {
            self.line.push_str(&rest[..end]).map_err(|_| fmt::Error)?;
            self.end_line()?;
            rest = &rest[end + 1..];
        }
        self.line.push_str(rest).map_err(|_| fmt::Error)
    }
}

/// Buffer for the metrics in Prometheus's text format.
type Text = heapless::String<{ http::MAX_BODY }>;

type BoardFamily<M> = MetricFamily<'static, M, 1, BoardLabels>;
type CapacitanceFamily<M> = MetricFamily<'static, M, { CAPACITANCE.len() }, ChannelLabels<usize>>;
type ColorFamily<M> = MetricFamily<'static, M, { COLOR.len() }, ChannelLabels<&'static str>>;

struct Metrics {
    firmware_build: MetricFamily<'static, IntGauge, 1, BuildLabels>,
    battery_low: BoardFamily<IntGauge>,
    capacitance: CapacitanceFamily<FixedGauge<Farads>>,
    capacitance_errors: CapacitanceFamily<Counter>,
    color: ColorFamily<IntGauge>,
    color_errors: ColorFamily<Counter>,
    temperature: BoardFamily<FixedGauge<Decimal>>,
    humidity: BoardFamily<FixedGauge<Decimal>>,
    temp_rh_errors: BoardFamily<Counter>,
    sound_level: BoardFamily<FixedGauge<Decimal>>,
    sound_errors: BoardFamily<Counter>,
    scrapes: BoardFamily<Counter>,
    pushes: BoardFamily<Counter>,
    push_failures: BoardFamily<Counter>,
    push_status: BoardFamily<IntGauge>,
}

impl Metrics {
    fn new() -> Self {
        Metrics {
            firmware_build: MetricBuilder::new("sensor_board_firmware_build")
                .with_help("Build time of the running firmware, as a Unix timestamp, labeled with its Git commit.")
                .build_labeled(),
            battery_low: MetricBuilder::new("sensor_board_battery_low")
                .with_help("1 if the supply voltage has been low since the board started, else 0.")
                .build_labeled(),
            capacitance: MetricBuilder::new("sensor_board_capacitance_farads")
                .with_help("Capacitance on the electrode of a channel, in farads.")
                .build_labeled(),
            capacitance_errors: MetricBuilder::new("sensor_board_capacitance_errors_total")
                .with_help("Failed readings of a capacitance channel.")
                .build_labeled(),
            color: MetricBuilder::new("sensor_board_color")
                .with_help("Raw reading of a color channel.")
                .build_labeled(),
            color_errors: MetricBuilder::new("sensor_board_color_errors_total")
                .with_help("Failed readings of a color channel.")
                .build_labeled(),
            temperature: MetricBuilder::new("sensor_board_temperature_celsius")
                .with_help("Temperature, in °C.")
                .build_labeled(),
            humidity: MetricBuilder::new("sensor_board_relative_humidity_percent")
                .with_help("Relative humidity, in percent.")
                .build_labeled(),
            temp_rh_errors: MetricBuilder::new("sensor_board_temp_rh_errors_total")
                .with_help("Failed measurements of the temperature & humidity sensor.")
                .build_labeled(),
            sound_level: MetricBuilder::new("sensor_board_sound_level_db_spl")
                .with_help("Sound level over a second, unweighted, in dB SPL.")
                .build_labeled(),
            sound_errors: MetricBuilder::new("sensor_board_sound_errors_total")
                .with_help("Failed measurements of the sound level.")
                .build_labeled(),
            scrapes: MetricBuilder::new("sensor_board_scrapes_total")
                .with_help("Scrapes of the board's metrics endpoint.")
                .build_labeled(),
            pushes: MetricBuilder::new("sensor_board_pushes_total")
                .with_help("Push attempts to the Pushgateway.")
                .build_labeled(),
            push_failures: MetricBuilder::new("sensor_board_push_failures_total")
                .with_help("Failed pushes to the Pushgateway.")
                .build_labeled(),
            push_status: MetricBuilder::new("sensor_board_push_status")
                .with_help(
                    "HTTP status of the Pushgateway's response to the last push, or 0 for none.",
                )
                .build_labeled(),
        }
    }

    /// Write the metrics in Prometheus's text format.
    fn write(&self, text: &mut impl fmt::Write) -> fmt::Result {
        self.firmware_build.fmt_metric(text)?;
        self.battery_low.fmt_metric(text)?;
        self.capacitance.fmt_metric(text)?;
        self.capacitance_errors.fmt_metric(text)?;
        self.color.fmt_metric(text)?;
        self.color_errors.fmt_metric(text)?;
        self.temperature.fmt_metric(text)?;
        self.humidity.fmt_metric(text)?;
        self.temp_rh_errors.fmt_metric(text)?;
        self.sound_level.fmt_metric(text)?;
        self.sound_errors.fmt_metric(text)?;
        self.scrapes.fmt_metric(text)?;
        self.pushes.fmt_metric(text)?;
        self.push_failures.fmt_metric(text)?;
        self.push_status.fmt_metric(text)
    }

    /// Record whether the supply is low. Nothing reports when that changes,
    /// so this is called before every `write`.
    fn record_supply(&self, board: BoardId, supply: supply::Monitor) {
        if let Some(low) = self.battery_low.register(BoardLabels(board)) {
            low.set_value(usize::from(supply.is_low()));
        }
    }
}

/// The metrics, shared by the [`Task`] and the [`Handle`]. Recording a value
/// only needs `&Metrics`. The `RefCell` allows replacing them all in
/// [`SharedMetrics::reset`]. Each access runs to completion inside the
/// lock, so the text is always rendered from one consistent state.
struct SharedMetrics(Mutex<ThreadModeRawMutex, RefCell<Metrics>>);

impl SharedMetrics {
    fn with<R>(&self, f: impl FnOnce(&Metrics) -> R) -> R {
        self.0.lock(|metrics| f(&metrics.borrow()))
    }

    /// Reset every metric.
    fn reset(&self) {
        self.0.lock(|metrics| metrics.replace(Metrics::new()));
    }

    /// Add `n` to a per-board counter of `board`.
    fn count(
        &self,
        board: BoardId,
        counter: impl FnOnce(&Metrics) -> &BoardFamily<Counter>,
        n: usize,
    ) {
        self.with(|metrics| {
            if let Some(counter) = counter(metrics).register(BoardLabels(board)) {
                counter.fetch_add(n);
            }
        });
    }

    /// Render the current metrics into `text`, with the state of `board`'s
    /// supply, in as much `detail` as given. Fails if they do not fit, and
    /// leaves the text empty.
    fn render(
        &self,
        board: BoardId,
        supply: supply::Monitor,
        detail: Detail,
        text: &mut Text,
    ) -> fmt::Result {
        text.clear();
        let written = self.with(|metrics| {
            metrics.record_supply(board, supply);
            match detail {
                Detail::Full => metrics.write(text),
                Detail::Samples => metrics.write(&mut Samples::new(text)),
            }
        });
        if written.is_err() {
            defmt::error!("metrics: the text does not fit its buffer");
            text.clear();
        }
        written
    }
}

pub struct Builder {
    /// The TCP connection, used for scrapes and for pushes.
    pub tcp: thread::Tcp,
    pub config: board_config::Monitor,
    pub supply: supply::Monitor,
    pub capacitance: Option<&'static sensor::Shared<CapacitanceSensor<'static>>>,
    pub color: Option<&'static sensor::Shared<ColorSensor<'static>>>,
    pub temp_rh: Option<&'static sensor::Shared<TempRhSensor<'static>>>,
    pub mic: &'static sensor::Shared<Mic<'static>>,
}

impl Builder {
    /// Panics if called a second time: the firmware has one metrics task.
    pub fn init(self) -> (Task, Handle) {
        static METRICS: StaticCell<SharedMetrics> = StaticCell::new();
        static TEXT: StaticCell<Text> = StaticCell::new();
        let shared: &'static SharedMetrics =
            METRICS.init(SharedMetrics(Mutex::new(RefCell::new(Metrics::new()))));
        let board = BoardId(embassy_stm32::uid::uid());

        let task = Task {
            config: self.config,
            exporter: Exporter {
                tcp: self.tcp,
                capacitance: self.capacitance,
                color: self.color,
                temp_rh: self.temp_rh,
                mic: self.mic,
                supply: self.supply,
                board,
                metrics: shared,
                text: TEXT.init(Text::new()),
                polls: [None; SENSORS.len()],
                sound_duration: board_config::sound_duration(None),
            },
        };
        let handle = Handle {
            metrics: shared,
            supply: self.supply,
            board,
        };
        (task, handle)
    }
}

/// Gives the rest of the firmware read access to the metrics.
pub struct Handle {
    metrics: &'static SharedMetrics,
    supply: supply::Monitor,
    /// This board's ID.
    board: BoardId,
}

impl Handle {
    /// The chunk of the current metrics text that starts at byte `offset`.
    pub fn text(&self, offset: u32) -> MetricsChunk {
        let mut window = Window {
            skip: offset as usize,
            chunk: MetricsChunk {
                text: heapless::String::new(),
                more: false,
            },
        };
        // Cannot fail: `Window::write_str` always returns `Ok`.
        let _ = self.metrics.with(|metrics| {
            metrics.record_supply(self.board, self.supply);
            metrics.write(&mut window)
        });
        window.chunk
    }
}

/// A `fmt::Write` sink that skips the first `skip` bytes written to it and
/// keeps as much of the rest as fits in one chunk.
struct Window {
    /// Bytes still to skip.
    skip: usize,
    chunk: MetricsChunk,
}

impl fmt::Write for Window {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for character in text.chars() {
            if self.skip > 0 {
                self.skip = self.skip.saturating_sub(character.len_utf8());
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
            // Export until the configuration changes, then start over with
            // the new one.
            let export = self.exporter.export(config.as_ref());
            config = match select(self.config.changed(), export).await {
                Either::First(config) => config,
                Either::Second(never) => never,
            };
            self.exporter.stop().await;
        }
    }
}

/// A sensor's polling schedule: when it is next due, and its interval.
#[derive(Clone, Copy)]
struct Poll {
    due: Instant,
    interval: Duration,
}

struct Exporter {
    tcp: thread::Tcp,
    capacitance: Option<&'static sensor::Shared<CapacitanceSensor<'static>>>,
    color: Option<&'static sensor::Shared<ColorSensor<'static>>>,
    temp_rh: Option<&'static sensor::Shared<TempRhSensor<'static>>>,
    mic: &'static sensor::Shared<Mic<'static>>,
    supply: supply::Monitor,
    /// This board's ID.
    board: BoardId,
    metrics: &'static SharedMetrics,
    /// The rendered metrics text, for the response or push being sent.
    text: &'static mut Text,
    /// The schedule of each sensor in [`SENSORS`], by index. `None` if the
    /// sensor is disabled.
    polls: [Option<Poll>; SENSORS.len()],
    /// How long the microphone listens for each measurement.
    sound_duration: Duration,
}

impl Exporter {
    /// Poll the sensors and export the metrics as `config` specifies.
    async fn export(&mut self, config: Option<&BoardConfig>) -> ! {
        // Drop the metrics recorded under the previous configuration.
        self.metrics.reset();
        self.metrics.with(|metrics| {
            if let Some(build) = metrics.firmware_build.register(BuildLabels(self.board)) {
                build.set_value(update::build().0 as usize);
            }
        });

        self.sound_duration = board_config::sound_duration(config);
        let now = Instant::now();
        for (poll, sensor) in self.polls.iter_mut().zip(SENSORS) {
            let sensor = config.and_then(|config| config.sensor_config.0[sensor]);
            *poll = sensor.map(|sensor| {
                let micros = sensor.poll_interval.as_micros().try_into();
                let interval = Duration::from_micros(micros.unwrap_or(u64::MAX));
                Poll {
                    due: now,
                    interval: interval.max(MIN_INTERVAL),
                }
            });
        }

        // A board without a configuration is taken to be on aux power.
        let power_mode = config.map_or(PowerMode::Aux, |config| config.power_mode);
        match power_mode {
            PowerMode::Aux => self.serve().await,
            PowerMode::Battery => {
                let gateway = config.and_then(|config| config.pushgateway);
                self.push(gateway).await
            }
        }
    }

    /// Close the connection, stop listening, and remove the registration.
    async fn stop(&mut self) {
        self.tcp.close().await;
        if let Err(e) = self.tcp.listen(None).await {
            defmt::debug!("metrics: could not stop listening: {}", e);
        }
        if let Err(e) = self.tcp.register(None).await {
            defmt::debug!("metrics: could not remove the registration: {}", e);
        }
    }

    /// Serve the scrape endpoint and poll the sensors.
    async fn serve(&mut self) -> ! {
        self.count(|metrics| &metrics.scrapes, 0);
        loop {
            let next = select(
                Self::wait_until_due(&self.polls),
                Self::accept_scrape(&mut self.tcp),
            );
            match next.await {
                Either::First(sensor) => self.read(sensor).await,
                Either::Second(()) => self.handle_scrape().await,
            }
        }
    }

    /// Wait for an incoming connection on [`SCRAPE_PORT`].
    async fn accept_scrape(tcp: &mut thread::Tcp) {
        loop {
            let accepted = async {
                // Listen and register on every attempt: the last one may
                // have failed because Thread had not started. Doing either
                // again is a no-op.
                tcp.listen(Some(SCRAPE_PORT)).await?;
                // A scrape does not need the registration. Without it, wait
                // for a connection only until it is time to register again:
                // Prometheus does not scrape a board that is not registered.
                match tcp.register(Some(SCRAPE_SERVICE)).await {
                    Ok(()) => tcp.accept().await.map(|()| true),
                    Err(e) => {
                        defmt::debug!("metrics: could not register the endpoint: {}", e);
                        match with_timeout(RETRY, tcp.accept()).await {
                            Ok(accepted) => accepted.map(|()| true),
                            Err(_) => Ok(false),
                        }
                    }
                }
            };
            match accepted.await {
                Ok(true) => return,
                Ok(false) => {}
                Err(e) => {
                    defmt::debug!("metrics: cannot accept scrapes yet: {}", e);
                    Timer::after(RETRY).await;
                }
            }
        }
    }

    /// Handle the HTTP request on the accepted connection.
    async fn handle_scrape(&mut self) {
        let mut scrape = Scrape {
            board: self.board,
            supply: self.supply,
            metrics: self.metrics,
            text: self.text,
        };
        if let Err(e) = http::serve(&mut self.tcp, &mut scrape).await {
            defmt::debug!("metrics: scrape failed: {}", e);
        }
        self.tcp.close().await;
    }

    /// Poll the sensors, and push the metrics to `gateway` after each read.
    /// Without a gateway, only poll.
    async fn push(&mut self, gateway: Option<SocketAddrV6>) -> ! {
        // No push before this time. Set after a failed push.
        let mut not_before = Instant::now();
        let mut registered = false;
        loop {
            if !registered {
                let outcome = self.tcp.register(Some(BOARD_SERVICE)).await;
                registered = outcome.is_ok();
            }
            let due = Self::wait_until_due(&self.polls);
            let sensor = if registered {
                due.await
            } else {
                match select(due, Timer::after(RETRY)).await {
                    Either::First(sensor) => sensor,
                    Either::Second(()) => continue,
                }
            };
            // A reading that cannot be pushed is of no use, so wait for the
            // network first. That matters when the board starts: a sensor is
            // due at once, a few seconds before the board has attached.
            let gateway = gateway.filter(|_| Instant::now() >= not_before);
            if let Some(gateway) = gateway {
                let _ = with_timeout(NETWORK_WAIT, self.tcp.ready(*gateway.ip())).await;
            }

            self.read(sensor).await;
            // Also read every other sensor that is due, so that one push
            // covers them all.
            let now = Instant::now();
            for sensor in 0..self.polls.len() {
                if self.polls[sensor].is_some_and(|poll| poll.due <= now) {
                    self.read(sensor).await;
                }
            }

            if let Some(gateway) = gateway
                && !self.push_to(gateway).await
            {
                not_before = Instant::now() + PUSH_RETRY;
            }
        }
    }

    /// Push the metrics to `gateway`. Returns whether it accepted them.
    async fn push_to(&mut self, gateway: SocketAddrV6) -> bool {
        self.count(|metrics| &metrics.pushes, 1);

        let pushed = async {
            let rendered = self
                .metrics
                .render(self.board, self.supply, Detail::Samples, self.text);
            rendered.map_err(|_| http::Error::Malformed)?;
            let mut path: heapless::String<64> = heapless::String::new();
            let written = write!(path, "/metrics/job/{JOB}/instance/{}", self.board);
            written.map_err(|_| http::Error::Malformed)?;
            http::put(&mut self.tcp, gateway, &path, self.text).await
        };
        let status = pushed.await;
        self.tcp.close().await;

        let accepted = matches!(status, Ok(200..300));
        if let Err(e) = status {
            defmt::debug!("metrics: push failed: {}", e);
        }
        self.count(|metrics| &metrics.push_failures, usize::from(!accepted));
        self.metrics.with(|metrics| {
            if let Some(last) = metrics.push_status.register(BoardLabels(self.board)) {
                last.set_value(status.map_or(0, usize::from));
            }
        });
        accepted
    }

    /// Wait until a sensor is due. Returns its index in [`SENSORS`].
    async fn wait_until_due(polls: &[Option<Poll>]) -> usize {
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

    /// Read the sensor at index `sensor` in [`SENSORS`], and record the
    /// result.
    async fn read(&mut self, sensor: usize) {
        if let Some(poll) = &mut self.polls[sensor] {
            poll.due = Instant::now() + poll.interval;
        }
        match SENSORS[sensor] {
            Sensor::Capacitance0 => self.read_capacitance(0).await,
            Sensor::Capacitance1 => self.read_capacitance(1).await,
            Sensor::Capacitance2 => self.read_capacitance(2).await,
            Sensor::Capacitance3 => self.read_capacitance(3).await,
            Sensor::Color => self.read_color().await,
            Sensor::Temperature => {
                self.read_temp_rh(
                    |metrics| &metrics.temperature,
                    |reading| reading.temperature,
                )
                .await
            }
            Sensor::Humidity => {
                self.read_temp_rh(|metrics| &metrics.humidity, |reading| reading.humidity)
                    .await
            }
            Sensor::Sound => self.read_sound().await,
            // Not in `SENSORS`: these have no driver.
            Sensor::Distance | Sensor::Acceleration => {}
        }
    }

    /// Read the channel at index `channel` in [`CAPACITANCE`], and record the
    /// result.
    async fn read_capacitance(&mut self, channel: usize) {
        let labels = || ChannelLabels {
            board: self.board,
            channel,
        };

        let reading = match self.capacitance {
            Some(sensor) => sensor.lock().await.read(CAPACITANCE[channel]).await.ok(),
            None => None,
        };
        self.metrics.with(|metrics| {
            if let Some(errors) = metrics.capacitance_errors.register(labels()) {
                errors.fetch_add(usize::from(reading.is_none()));
            }
            let farads = reading.and_then(|reading| fixed(reading.picofarads * 1e-12));
            if let (Some(farads), Some(gauge)) = (farads, metrics.capacitance.register(labels())) {
                gauge.set_value(farads);
            }
        });
    }

    /// Read every channel of the color sensor, and record the results. On a
    /// board without one, every reading fails.
    async fn read_color(&mut self) {
        let mut sensor = match self.color {
            Some(color) => Some(color.lock().await),
            None => None,
        };
        for (channel, name) in COLOR {
            let reading = match &mut sensor {
                Some(sensor) => sensor.read_channel(channel).await.ok(),
                None => None,
            };
            let labels = || ChannelLabels {
                board: self.board,
                channel: name,
            };
            self.metrics.with(|metrics| {
                if let Some(errors) = metrics.color_errors.register(labels()) {
                    errors.fetch_add(usize::from(reading.is_none()));
                }
                if let (Some(value), Some(gauge)) = (reading, metrics.color.register(labels())) {
                    gauge.set_value(value.into());
                }
            });
        }
    }

    /// Measure with the temperature & humidity sensor, and set `gauge` to
    /// `value` of the measurement. On a board without one, every measurement
    /// fails.
    async fn read_temp_rh(
        &mut self,
        gauge: impl FnOnce(&Metrics) -> &BoardFamily<FixedGauge<Decimal>>,
        value: impl FnOnce(TempRh) -> f32,
    ) {
        let reading = match self.temp_rh {
            Some(temp_rh) => {
                let req = TempRhReq::Plain(Precision::High);
                temp_rh.lock().await.read(req).await.ok()
            }
            None => None,
        };
        self.count(
            |metrics| &metrics.temp_rh_errors,
            usize::from(reading.is_none()),
        );
        self.metrics.with(|metrics| {
            let gauge = gauge(metrics).register(BoardLabels(self.board));
            let value = reading.and_then(|reading| fixed(value(reading)));
            if let (Some(value), Some(gauge)) = (value, gauge) {
                gauge.set_value(value);
            }
        });
    }

    /// Measure the sound level.
    async fn read_sound(&mut self) {
        let level = self.mic.lock().await.measure(self.sound_duration).await;
        self.count(|metrics| &metrics.sound_errors, usize::from(level.is_err()));
        self.metrics.with(|metrics| {
            let gauge = metrics.sound_level.register(BoardLabels(self.board));
            let leq = level.ok().and_then(|level| fixed(level.leq));
            if let (Some(leq), Some(gauge)) = (leq, gauge) {
                gauge.set_value(leq);
            }
        });
    }

    /// Add `n` to a per-board counter.
    fn count(&self, counter: impl FnOnce(&Metrics) -> &BoardFamily<Counter>, n: usize) {
        self.metrics.count(self.board, counter, n);
    }
}

/// Answers the HTTP request of a scrape.
struct Scrape<'a> {
    board: BoardId,
    supply: supply::Monitor,
    metrics: &'static SharedMetrics,
    /// The rendered metrics text, for the response.
    text: &'a mut Text,
}

impl HttpHandler for Scrape<'_> {
    async fn handle_request(
        &mut self,
        request: &HttpRequest<'_>,
    ) -> Result<HttpResponse<'_>, nanofish::Error> {
        if request.method != HttpMethod::GET || request.route_path() != "/metrics" {
            let body = "The metrics are at /metrics.\n";
            return Ok(http::response(StatusCode::NotFound, body));
        }
        self.metrics
            .count(self.board, |metrics| &metrics.scrapes, 1);
        // nanofish answers an error with a 500.
        let rendered = self
            .metrics
            .render(self.board, self.supply, Detail::Full, self.text);
        rendered.map_err(|fmt::Error| nanofish::Error::BufferOverflow)?;
        Ok(http::response(StatusCode::Ok, self.text))
    }
}
