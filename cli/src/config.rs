//! The `config` commands: show or change a board's configuration, which sets
//! the sensors it reads, how often, and how it is powered. The target is an
//! attached board, or a board on a network that an attached board or a
//! border router reaches.

use std::{
    fmt::{self, Display},
    net::{Ipv6Addr, SocketAddrV6},
    time::Duration,
};

use anyhow::{Context, Result, bail};
use clap::{Args, ValueEnum};
use protocol::{
    BoardConfig, BoardId, CONFIG_PORT, ConfigMessage, DEFAULT_SOUND_DURATION, PowerMode,
    SLEEPY_POLL_PERIOD, SOUND_DURATIONS, Sensor, SensorConfig, SensorsConfig,
};
use tokio::{
    net::UdpSocket,
    time::{Instant, timeout_at},
};

use crate::{
    board::{self, Board},
    border_router::{self, BorderRouter},
};

/// How long to wait for a board's reply to a request sent through a border
/// router, and how often to resend the request in that time. A board that
/// relays a request for the host uses the same times. A board on battery
/// only gets the request at its next poll, so the wait is longer than a poll
/// period.
const REPLY_TIMEOUT: Duration = Duration::from_secs(SLEEPY_POLL_PERIOD.as_secs() + 4);
const RESEND_INTERVAL: Duration = Duration::from_secs(5);

/// The changes to make to a board's configuration.
#[derive(Args)]
pub struct Changes {
    /// The board's number
    #[arg(long, value_name = "N")]
    id: Option<u8>,

    /// How the board is powered
    #[arg(long, value_enum)]
    power_mode: Option<PowerMode>,

    /// Set a sensor's poll interval, as in `temperature=30s` (units: ms, s, m or h), or disable
    /// the sensor with `temperature=off`. Can be given several times
    #[arg(long = "sensor", value_name = "SENSOR=INTERVAL", value_parser = sensor_change)]
    sensors: Vec<SensorChange>,

    /// The Prometheus Pushgateway to push metrics to in battery mode, as in
    /// `[fd12:3456::1]:9091`, or `none` to clear it
    #[arg(long, value_name = "[ADDRESS]:PORT", value_parser = address_change)]
    pushgateway: Option<AddressChange>,

    /// The Home Assistant server to report the motion sensor's state to, as in
    /// `[fd12:3456::1]:8123`, or `none` to clear it. The board calls the webhook
    /// `sensor-board-<serial>` there
    #[arg(long, value_name = "[ADDRESS]:PORT", value_parser = address_change)]
    home_assistant: Option<AddressChange>,

    /// How long the microphone listens for each sound measurement, as in `125ms` (from 10ms to
    /// 10s)
    #[arg(long, value_name = "DURATION", value_parser = parse_sound_duration)]
    sound_duration: Option<Duration>,

    /// Whether or not to enable the motion sensor
    #[arg(long)]
    motion_sensor: Option<bool>,
}

/// The new setting of a server's address. `None` clears it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct AddressChange(Option<SocketAddrV6>);

/// The new configuration of one sensor. `None` disables the sensor.
#[derive(Debug, Clone, PartialEq, Eq)]
struct SensorChange {
    sensor: Sensor,
    config: Option<SensorConfig>,
}

impl Changes {
    fn is_empty(&self) -> bool {
        self.id.is_none()
            && self.power_mode.is_none()
            && self.sensors.is_empty()
            && self.pushgateway.is_none()
            && self.sound_duration.is_none()
            && self.motion_sensor.is_none()
            && self.home_assistant.is_none()
    }

    /// Apply the changes to `current`. If the board has no configuration
    /// yet, build one from the changes, which then have to include `--id`
    /// and `--power-mode`.
    fn apply(&self, current: Option<BoardConfig>) -> Result<BoardConfig> {
        let mut config = match (current, self.id, self.power_mode) {
            (Some(config), _, _) => config,
            (None, Some(board_id), Some(power_mode)) => BoardConfig {
                board_id,
                sensor_config: SensorsConfig::default(),
                power_mode,
                pushgateway: None,
                sound_duration: DEFAULT_SOUND_DURATION,
                motion_enabled: false,
                home_assistant: None,
            },
            (None, _, _) => bail!("it has no configuration yet: give it --id and --power-mode"),
        };
        if let Some(board_id) = self.id {
            config.board_id = board_id;
        }
        if let Some(power_mode) = self.power_mode {
            config.power_mode = power_mode;
        }
        for change in &self.sensors {
            config.sensor_config.0[change.sensor] = change.config;
        }
        if let Some(AddressChange(pushgateway)) = self.pushgateway {
            config.pushgateway = pushgateway;
        }
        if let Some(duration) = self.sound_duration {
            config.sound_duration = duration;
        }
        if let Some(motion_sensor) = self.motion_sensor {
            config.motion_enabled = motion_sensor;
        }
        if let Some(AddressChange(home_assistant)) = self.home_assistant {
            config.home_assistant = home_assistant;
        }
        Ok(config)
    }
}

/// How a `config` command reaches the board it applies to.
pub enum Route {
    /// Through an attached board: over USB for that board itself, and
    /// relayed over its network for another board.
    Attached(Board),
    /// Through a border router: this machine sends the [`ConfigMessage`]s
    /// itself.
    BorderRouter(Devices),
}

/// What `--board` names: one of the attached boards, by its index, or a
/// board that is not attached.
#[derive(Debug, PartialEq, Eq)]
enum Named {
    Attached(usize),
    Remote(BoardId),
}

/// Find the board that `serial` names among the boards with the serial
/// numbers `attached`. The start of a serial number is enough for an
/// attached board, and with `None` the board is the only attached one. A
/// board that is not attached needs its whole serial number.
fn named(serial: Option<&str>, attached: &[&str]) -> Result<Named> {
    let listed = || -> String { attached.iter().map(|s| format!("\n  {s}")).collect() };
    let Some(serial) = serial else {
        return match attached {
            [_] => Ok(Named::Attached(0)),
            [] => bail!("no board is attached: name a board on the network with --board"),
            _ => bail!(
                "several boards are attached; pick one with --board:{}",
                listed()
            ),
        };
    };

    let prefix = serial.to_uppercase();
    let matching = (0..attached.len()).filter(|&i| attached[i].starts_with(&prefix));
    match matching.collect::<Vec<usize>>().as_slice() {
        [index] => Ok(Named::Attached(*index)),
        [] => match serial.parse() {
            Ok(board) => Ok(Named::Remote(board)),
            Err(_) => bail!(
                "no attached board has a serial number starting with {prefix}. For a board that \
                 is not attached, give the whole serial number"
            ),
        },
        _ => bail!(
            "several attached boards have a serial number starting with {prefix}:{}",
            listed()
        ),
    }
}

/// Find the board that `serial` names (see [`named`]) and the way to reach
/// it. An attached board is reached over USB. Any other board is reached
/// through an attached board that is on a network, and without one through
/// the default border router. `border_router` is the address of a border
/// router to go through in any case.
pub async fn route(serial: Option<&str>, border_router: Option<&str>) -> Result<(Route, BoardId)> {
    if let Some(address) = border_router {
        let board = serial.and_then(|serial| serial.parse().ok());
        let Some(board) = board else {
            bail!("through the border router, --board has to give the board's whole serial number");
        };
        return Ok((Route::BorderRouter(Devices::of(address).await?), board));
    }

    let attached = board::devices().await?;
    let serials: Vec<&str> = attached.iter().map(board::serial_of).collect();
    match named(serial, &serials)? {
        Named::Attached(index) => {
            let board = Board::open(&attached[index]).await?;
            let id = board.id()?;
            Ok((Route::Attached(board), id))
        }
        Named::Remote(board) => match Board::first_on_network().await? {
            Some(relay) => {
                eprintln!("Asking through board {}.", relay.serial());
                Ok((Route::Attached(relay), board))
            }
            None => {
                let devices = Devices::of(border_router::DEFAULT_ADDRESS).await?;
                Ok((Route::BorderRouter(devices), board))
            }
        },
    }
}

impl Route {
    async fn get(&mut self, target: BoardId) -> Result<Option<BoardConfig>> {
        match self {
            Route::Attached(board) => board.board_config(target).await,
            Route::BorderRouter(devices) => devices.ask(target, None).await,
        }
    }

    async fn set(&mut self, target: BoardId, config: &BoardConfig) -> Result<()> {
        match self {
            Route::Attached(board) => board.set_board_config(target, config).await,
            Route::BorderRouter(devices) => {
                // The reply is what the board has stored after the request.
                let stored = devices.ask(target, Some(config)).await?;
                if stored.as_ref() != Some(config) {
                    bail!("board {target} could not store the configuration");
                }
                Ok(())
            }
        }
    }
}

/// The devices on a border router's network, and the socket that this
/// machine sends them [`ConfigMessage`]s from.
pub struct Devices {
    socket: UdpSocket,
    addresses: Vec<Ipv6Addr>,
    /// The number for the next request. A reply carries the number of the
    /// request it answers.
    next_question: u32,
}

impl Devices {
    /// The devices on the network of the border router whose REST API is at
    /// `address`.
    pub async fn of(address: &str) -> Result<Devices> {
        eprintln!("Asking through the border router at {address}.");
        let (node, devices) = BorderRouter::at(address).network().await?;
        let addresses = border_router::targets(&node, &devices).await.addresses;
        if addresses.is_empty() {
            bail!("found no other device with a routable address on the Thread network");
        }
        // A board replies to the configuration port, whatever port the
        // request came from.
        let socket = UdpSocket::bind((Ipv6Addr::UNSPECIFIED, CONFIG_PORT))
            .await
            .with_context(|| format!("could not bind UDP port {CONFIG_PORT}"))?;
        Ok(Devices {
            socket,
            addresses,
            // Not the number of a request from an earlier run, whose reply
            // could still arrive.
            next_question: rand::random(),
        })
    }

    /// Ask `board` to store `config`, or with `None` only for its
    /// configuration. Returns the configuration it then has. The request
    /// goes to every device, because the address of `board` is not known.
    async fn ask(
        &mut self,
        board: BoardId,
        config: Option<&BoardConfig>,
    ) -> Result<Option<BoardConfig>> {
        let question = self.next_question;
        self.next_question = question.wrapping_add(1);
        let request = match config {
            Some(config) => ConfigMessage::Set {
                board,
                question,
                config: config.clone(),
            },
            None => ConfigMessage::Get { board, question },
        };
        let mut encoded = [0; ConfigMessage::MAX_LEN];
        let request = request
            .encode(&mut encoded)
            .expect("the buffer has room for the longest message");

        let mut datagram = [0; 2 * ConfigMessage::MAX_LEN];
        let deadline = Instant::now() + REPLY_TIMEOUT;
        while Instant::now() < deadline {
            let mut sent = Ok(());
            let mut any_sent = false;
            for address in &self.addresses {
                match self.socket.send_to(request, (*address, CONFIG_PORT)).await {
                    Ok(_) => any_sent = true,
                    Err(e) => sent = Err(e),
                }
            }
            if !any_sent {
                sent.context("could not send to any device on the Thread network")?;
            }

            // Neither the request nor the reply is retransmitted when it is
            // lost, so the request is sent again.
            let resend_at = deadline.min(Instant::now() + RESEND_INTERVAL);
            while let Ok(received) =
                timeout_at(resend_at, self.socket.recv_from(&mut datagram)).await
            {
                let (len, _) = received.context("could not receive replies")?;
                if let Some(stored) = reply(&datagram[..len], board, question) {
                    return Ok(stored);
                }
            }
        }
        bail!(
            "board {board} did not reply. A board replies if it is on the border router's \
             network, its firmware has the same configuration layout as this CLI, and this \
             machine has a route to the Thread network"
        )
    }
}

/// The configuration in `datagram`, if that is the reply of `board` to
/// request number `question`.
fn reply(datagram: &[u8], board: BoardId, question: u32) -> Option<Option<BoardConfig>> {
    match ConfigMessage::decode(datagram)? {
        ConfigMessage::Has {
            board: from,
            question: answered,
            config,
        } if (from, answered) == (board, question) => Some(config),
        // A reply to an earlier request, or not a reply at all.
        _ => None,
    }
}

/// Print the configuration of the board `target`, which `route` reaches. With
/// `stored`, print it as the board stores it in flash, as a Rust array.
pub async fn show(mut route: Route, target: BoardId, stored: bool) -> Result<()> {
    match route.get(target).await? {
        Some(config) if stored => print!("{}", stored_array(&config)),
        Some(config) => print!("Board {target}\n{}", DescribeBoardConfig(&config)),
        None => println!("Board {target} has no configuration."),
    }
    Ok(())
}

/// The bytes that a board stores for `config`, as a Rust array that a test can
/// use.
fn stored_array(config: &BoardConfig) -> String {
    let mut buf = [0; BoardConfig::MAX_LEN];
    let bytes = config
        .encode(&mut buf)
        .expect("MAX_LEN fits every configuration");
    let mut text = format!("const STORED: [u8; {}] = [\n", bytes.len());
    for row in bytes.chunks(15) {
        let row: Vec<_> = row.iter().map(|byte| format!("{byte:#04x},")).collect();
        text += &format!("    {}\n", row.join(" "));
    }
    text + "];\n"
}

/// Apply `changes` to the configuration of the board `target`, which `route`
/// reaches.
pub async fn set(mut route: Route, target: BoardId, changes: &Changes) -> Result<()> {
    if changes.is_empty() {
        bail!(
            "nothing to change: give --id, --power-mode, --sensor, --pushgateway, \
             --sound-duration, --motion-sensor or --home-assistant"
        );
    }
    let current = route.get(target).await?;
    let config = match changes.apply(current) {
        Ok(config) => config,
        Err(e) => bail!("board {target}: {e}"),
    };
    route.set(target, &config).await?;
    print!("Board {target} now has\n{}", DescribeBoardConfig(&config));
    Ok(())
}

struct DescribeBoardConfig<'a>(&'a BoardConfig);

impl<'a> Display for DescribeBoardConfig<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let BoardConfig {
            board_id,
            sensor_config,
            power_mode,
            pushgateway,
            sound_duration,
            motion_enabled,
            home_assistant,
        } = self.0;
        writeln!(f, "  {:<14}  {board_id}", "id")?;
        writeln!(f, "  {:<14}  {}", "power mode", name(power_mode))?;

        write!(f, "  {:<14}", "pushgateway")?;
        match pushgateway {
            None => writeln!(f, "  none")?,
            Some(address) => writeln!(f, "  {address}")?,
        }

        writeln!(
            f,
            "  {:<14}  {}",
            "sound duration",
            describe_interval(*sound_duration)
        )?;
        for (sensor, config) in &sensor_config.0 {
            write!(f, "  {:<14}  ", name(&sensor))?;
            match config {
                Some(SensorConfig { poll_interval }) => {
                    writeln!(f, "every {}", describe_interval(*poll_interval))?
                }
                None => writeln!(f, "off")?,
            };
        }
        writeln!(
            f,
            "  {:<14}  {}",
            "motion sensor",
            if *motion_enabled { "on" } else { "off" }
        )?;
        write!(f, "  {:<14}", "home assistant")?;
        match home_assistant {
            None => writeln!(f, "  none")?,
            Some(address) => writeln!(f, "  {address}")?,
        }

        Ok(())
    }
}

/// The name of `value` on the command line.
fn name(value: &impl ValueEnum) -> String {
    let name = value.to_possible_value();
    name.map_or_else(String::new, |name| name.get_name().to_owned())
}

/// Parse a `--sensor` argument.
fn sensor_change(text: &str) -> Result<SensorChange, String> {
    let Some((sensor, interval)) = text.split_once('=') else {
        return Err("expected SENSOR=INTERVAL, as in temperature=30s".to_owned());
    };
    let Ok(sensor) = Sensor::from_str(sensor, true) else {
        let sensors: Vec<String> = Sensor::value_variants().iter().map(name).collect();
        return Err(format!(
            "unknown sensor {sensor}; the sensors are {}",
            sensors.join(", ")
        ));
    };
    let config = match interval {
        "off" => None,
        interval => Some(SensorConfig {
            poll_interval: parse_interval(interval)?,
        }),
    };
    Ok(SensorChange { sensor, config })
}

/// Parse a `--sound-duration` argument.
fn parse_sound_duration(text: &str) -> Result<Duration, String> {
    let duration = parse_interval(text)?;
    if !SOUND_DURATIONS.contains(&duration) {
        return Err(format!(
            "a sound measurement lasts from {} to {}",
            describe_interval(*SOUND_DURATIONS.start()),
            describe_interval(*SOUND_DURATIONS.end())
        ));
    }
    Ok(duration)
}

/// Parse a `--pushgateway` or `--home-assistant` argument.
fn address_change(text: &str) -> Result<AddressChange, String> {
    if text == "none" {
        return Ok(AddressChange(None));
    }
    match text.parse() {
        Ok(address) => Ok(AddressChange(Some(address))),
        Err(_) => Err("expected [ADDRESS]:PORT with an IPv6 address, or none".to_owned()),
    }
}

/// Parse an interval: a whole number followed by a unit, as in `30s`.
fn parse_interval(text: &str) -> Result<Duration, String> {
    let malformed = || format!("{text} is not an interval: expected one like 500ms, 30s, 5m or 1h");

    let number = text.trim_end_matches(|c: char| c.is_ascii_alphabetic());
    let unit = match &text[number.len()..] {
        "ms" => Duration::from_millis(1),
        "s" => Duration::from_secs(1),
        "m" => Duration::from_secs(60),
        "h" => Duration::from_secs(3600),
        _ => return Err(malformed()),
    };
    match number.parse::<u32>() {
        Ok(0) => Err("an interval must be greater than zero".to_owned()),
        Ok(number) => Ok(unit * number),
        Err(_) => Err(malformed()),
    }
}

/// Format an interval the way [`parse_interval`] reads it, using the largest
/// unit that divides it evenly.
pub fn describe_interval(interval: Duration) -> String {
    let ms = interval.as_millis();
    if interval.subsec_nanos() % 1_000_000 != 0 {
        // Not a whole number of milliseconds, so it has no such form.
        return format!("{interval:?}");
    }
    for (unit, len) in [("h", 3_600_000), ("m", 60_000), ("s", 1000)] {
        if ms > 0 && ms % len == 0 {
            return format!("{}{unit}", ms / len);
        }
    }
    format!("{ms}ms")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn every(interval: Duration) -> Option<SensorConfig> {
        Some(SensorConfig {
            poll_interval: interval,
        })
    }

    mod args {
        use clap::Parser;

        use super::*;

        #[derive(clap::Parser)]
        struct Params {
            #[command(flatten)]
            args: Changes,
        }

        #[test]
        fn no_motion_sensor() {
            let params = Params::parse_from::<[&'static str; 0], _>([]);
            assert_eq!(params.args.motion_sensor, None)
        }

        #[test]
        fn motion_sensor_true() {
            let params = Params::parse_from(["cmd", "--motion-sensor=true"]);
            assert_eq!(params.args.motion_sensor, Some(true))
        }

        #[test]
        fn motion_sensor_false() {
            let params = Params::parse_from(["cmd", "--motion-sensor=false"]);
            assert_eq!(params.args.motion_sensor, Some(false));
        }

        #[test]
        fn motion_settings_alone_are_changes() {
            let params = Params::parse_from(["cmd", "--motion-sensor=true"]);
            assert!(!params.args.is_empty());
            let params = Params::parse_from(["cmd", "--home-assistant=none"]);
            assert!(!params.args.is_empty());
        }
    }

    fn config() -> BoardConfig {
        let mut sensors = SensorsConfig::default();
        sensors.0[Sensor::Capacitance0] = every(Duration::from_secs(10));
        sensors.0[Sensor::Temperature] = every(Duration::from_secs(300));
        BoardConfig {
            board_id: 3,
            sensor_config: sensors,
            power_mode: PowerMode::Aux,
            pushgateway: None,
            sound_duration: DEFAULT_SOUND_DURATION,
            motion_enabled: false,
            home_assistant: None,
        }
    }

    #[test]
    fn stored_array_has_the_stored_bytes() {
        let text = stored_array(&config());
        let (head, body) = text.split_once(" = [\n").unwrap();
        let bytes: Vec<u8> = body
            .trim_end_matches("];\n")
            .split_whitespace()
            .map(|byte| u8::from_str_radix(&byte[2..4], 16).unwrap())
            .collect();
        assert_eq!(head, format!("const STORED: [u8; {}]", bytes.len()));
        assert_eq!(BoardConfig::decode(&bytes), Some(config()));
        assert!(text.lines().all(|line| line.len() <= 100));
    }

    #[test]
    fn parses_intervals() {
        assert_eq!(parse_interval("500ms"), Ok(Duration::from_millis(500)));
        assert_eq!(parse_interval("30s"), Ok(Duration::from_secs(30)));
        assert_eq!(parse_interval("5m"), Ok(Duration::from_secs(300)));
        assert_eq!(parse_interval("1h"), Ok(Duration::from_secs(3600)));
        for malformed in ["30", "s", "1.5s", "-1s", "3 s", "3d", ""] {
            assert!(parse_interval(malformed).is_err(), "{malformed}");
        }
        assert!(parse_interval("0s").is_err());
    }

    #[test]
    fn formats_intervals() {
        for typed in ["500ms", "1500ms", "30s", "90s", "5m", "1h", "36h"] {
            let interval = parse_interval(typed).unwrap();
            assert_eq!(describe_interval(interval), typed);
        }
        assert_eq!(describe_interval(Duration::from_secs(120)), "2m");
        assert_eq!(describe_interval(Duration::ZERO), "0ms");
        assert_eq!(describe_interval(Duration::from_micros(1500)), "1.5ms");
    }

    #[test]
    fn parses_sensor_changes() {
        assert_eq!(
            sensor_change("temperature=30s"),
            Ok(SensorChange {
                sensor: Sensor::Temperature,
                config: every(Duration::from_secs(30)),
            })
        );
        assert_eq!(
            sensor_change("Capacitance2=off"),
            Ok(SensorChange {
                sensor: Sensor::Capacitance2,
                config: None,
            })
        );
        assert!(sensor_change("temperature").is_err());
        assert!(sensor_change("temperature=").is_err());
        let unknown = sensor_change("pressure=30s").unwrap_err();
        assert!(unknown.contains("capacitance0, capacitance1"), "{unknown}");
    }

    #[test]
    fn changes_keep_other_settings() {
        let changes = Changes {
            id: None,
            power_mode: Some(PowerMode::Battery),
            sensors: vec![
                sensor_change("capacitance0=off").unwrap(),
                sensor_change("humidity=1m").unwrap(),
            ],
            pushgateway: Some(address_change("[fd12:3456::1]:9091").unwrap()),
            sound_duration: Some(Duration::from_millis(125)),
            motion_sensor: None,
            home_assistant: None,
        };
        let changed = changes.apply(Some(config())).unwrap();

        let mut expected = config();
        expected.power_mode = PowerMode::Battery;
        expected.pushgateway = Some(SocketAddrV6::new(
            "fd12:3456::1".parse().unwrap(),
            9091,
            0,
            0,
        ));
        expected.sensor_config.0[Sensor::Capacitance0] = None;
        expected.sensor_config.0[Sensor::Humidity] = every(Duration::from_secs(60));
        expected.sound_duration = Duration::from_millis(125);
        assert_eq!(changed, expected);
    }

    #[test]
    fn first_configuration_requires_id_and_power_mode() {
        let mut changes = Changes {
            id: Some(3),
            power_mode: None,
            sensors: vec![sensor_change("capacitance0=10s").unwrap()],
            pushgateway: None,
            sound_duration: None,
            motion_sensor: None,
            home_assistant: None,
        };
        assert!(changes.apply(None).is_err());

        changes.power_mode = Some(PowerMode::Aux);
        let first = changes.apply(None).unwrap();
        let mut expected = config();
        expected.sensor_config.0[Sensor::Temperature] = None;
        assert_eq!(first, expected);
    }

    #[test]
    fn describes_one_setting_per_line() {
        let text = DescribeBoardConfig(&config()).to_string();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines[..5],
            [
                "  id              3",
                "  power mode      aux",
                "  pushgateway     none",
                "  sound duration  1s",
                "  capacitance0    every 10s"
            ]
        );
        assert!(lines.contains(&"  capacitance1    off"), "{text}");
        assert!(lines.contains(&"  temperature     every 5m"), "{text}");
        assert!(lines.contains(&"  motion sensor   off"), "{text}");
        assert!(lines.contains(&"  home assistant  none"), "{text}");
        assert_eq!(lines.len(), 6 + Sensor::value_variants().len());
    }

    #[test]
    fn parses_sound_durations() {
        assert_eq!(
            parse_sound_duration("125ms"),
            Ok(Duration::from_millis(125))
        );
        assert_eq!(parse_sound_duration("10s"), Ok(Duration::from_secs(10)));
        assert!(parse_sound_duration("5ms").is_err());
        assert!(parse_sound_duration("11s").is_err());
        assert!(parse_sound_duration("0s").is_err());
    }

    #[test]
    fn board_is_named_by_the_start_of_its_serial_if_attached() {
        let attached = ["4C0032000350475532303120", "4D0021000350475532303120"];
        let remote = "4B002E000350475532303120";

        assert_eq!(named(Some("4d"), &attached).unwrap(), Named::Attached(1));
        assert_eq!(
            named(Some(attached[0]), &attached).unwrap(),
            Named::Attached(0)
        );
        assert_eq!(
            named(Some(remote), &attached).unwrap(),
            Named::Remote(remote.parse().unwrap())
        );
        assert_eq!(
            named(Some(remote), &[]).unwrap(),
            Named::Remote(remote.parse().unwrap())
        );
        // The start of a serial number does not name a board that is not
        // attached, and "4" names both attached boards.
        assert!(named(Some("4B00"), &attached).is_err());
        assert!(named(Some("4"), &attached).is_err());

        // Without a serial number, the only attached board.
        assert_eq!(named(None, &attached[..1]).unwrap(), Named::Attached(0));
        assert!(named(None, &attached).is_err());
        assert!(named(None, &[]).is_err());
    }

    #[test]
    fn reply_has_to_match_the_board_and_the_question() {
        let board = BoardId([0x4C; 12]);
        let mut buffer = [0; ConfigMessage::MAX_LEN];
        let mut encode = |message: ConfigMessage| message.encode(&mut buffer).unwrap().to_vec();
        let has = |board, question, config| ConfigMessage::Has {
            board,
            question,
            config,
        };

        let stored = encode(has(board, 7, Some(config())));
        assert_eq!(reply(&stored, board, 7), Some(Some(config())));
        // A board without a configuration.
        assert_eq!(reply(&encode(has(board, 7, None)), board, 7), Some(None));

        // A late reply to another request, another board's reply, a request,
        // and a datagram that is no message.
        assert_eq!(reply(&stored, board, 8), None);
        assert_eq!(reply(&stored, BoardId([0x4D; 12]), 7), None);
        let get = encode(ConfigMessage::Get { board, question: 7 });
        assert_eq!(reply(&get, board, 7), None);
        assert_eq!(reply(b"not a message", board, 7), None);
    }

    #[test]
    fn parses_address_changes() {
        let address = SocketAddrV6::new("fd12:3456::1".parse().unwrap(), 9091, 0, 0);
        assert_eq!(
            address_change("[fd12:3456::1]:9091"),
            Ok(AddressChange(Some(address)))
        );
        assert_eq!(address_change("none"), Ok(AddressChange(None)));
        for malformed in ["fd12:3456::1", "192.168.1.2:9091", "pushgateway:9091", ""] {
            assert!(address_change(malformed).is_err(), "{malformed}");
        }
    }
}
