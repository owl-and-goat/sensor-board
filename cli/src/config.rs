//! The `config` commands: show or change a board's configuration, which sets
//! the sensors it reads, how often, and how it is powered. The target is an
//! attached board, or a board on its network.

use std::{net::SocketAddrV6, time::Duration};

use anyhow::{Result, bail};
use clap::{Args, ValueEnum};
use protocol::{BoardConfig, BoardId, PowerMode, Sensor, SensorConfig, SensorsConfig};

use crate::board::Board;

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
    #[arg(long, value_name = "[ADDRESS]:PORT", value_parser = pushgateway_change)]
    pushgateway: Option<PushgatewayChange>,
}

/// The new Pushgateway setting. `None` clears it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct PushgatewayChange(Option<SocketAddrV6>);

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
        if let Some(PushgatewayChange(pushgateway)) = self.pushgateway {
            config.pushgateway = pushgateway;
        }
        Ok(config)
    }
}

/// Print the configuration of `remote`, which `board` asks over the network,
/// or with `None` of `board` itself.
pub async fn show(board: &Board, remote: Option<BoardId>) -> Result<()> {
    let target = target(board, remote)?;
    match board.board_config(target).await? {
        Some(config) => print!("Board {target}\n{}", describe(&config)),
        None => println!("Board {target} has no configuration."),
    }
    Ok(())
}

/// Apply `changes` to the configuration of `remote`, which `board` reaches
/// over the network, or with `None` to that of `board` itself.
pub async fn set(board: &Board, remote: Option<BoardId>, changes: &Changes) -> Result<()> {
    if changes.is_empty() {
        bail!("nothing to change: give --id, --power-mode, --sensor or --pushgateway");
    }
    let target = target(board, remote)?;
    let current = board.board_config(target).await?;
    let config = match changes.apply(current) {
        Ok(config) => config,
        Err(e) => bail!("board {target}: {e}"),
    };
    board.set_board_config(target, &config).await?;
    print!("Board {target} now has\n{}", describe(&config));
    Ok(())
}

fn target(board: &Board, remote: Option<BoardId>) -> Result<BoardId> {
    match remote {
        Some(remote) => Ok(remote),
        None => board.id(),
    }
}

/// Format the configuration as text, one setting per line.
fn describe(config: &BoardConfig) -> String {
    let BoardConfig {
        board_id,
        sensor_config,
        power_mode,
        pushgateway,
    } = config;
    let mut text = format!("  {:<12}  {board_id}\n", "id");
    text += &format!("  {:<12}  {}\n", "power mode", name(power_mode));
    let pushgateway = pushgateway.map_or("none".to_owned(), |address| address.to_string());
    text += &format!("  {:<12}  {pushgateway}\n", "pushgateway");
    for (sensor, config) in &sensor_config.0 {
        let read = match config {
            Some(SensorConfig { poll_interval }) => {
                format!("every {}", describe_interval(*poll_interval))
            }
            None => "off".to_owned(),
        };
        text += &format!("  {:<12}  {read}\n", name(&sensor));
    }
    text
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

/// Parse a `--pushgateway` argument.
fn pushgateway_change(text: &str) -> Result<PushgatewayChange, String> {
    if text == "none" {
        return Ok(PushgatewayChange(None));
    }
    match text.parse() {
        Ok(address) => Ok(PushgatewayChange(Some(address))),
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
fn describe_interval(interval: Duration) -> String {
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

    fn config() -> BoardConfig {
        let mut sensors = SensorsConfig::default();
        sensors.0[Sensor::Capacitance0] = every(Duration::from_secs(10));
        sensors.0[Sensor::Temperature] = every(Duration::from_secs(300));
        BoardConfig {
            board_id: 3,
            sensor_config: sensors,
            power_mode: PowerMode::Aux,
            pushgateway: None,
        }
    }

    #[test]
    fn intervals_are_read_as_they_are_typed() {
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
    fn intervals_are_written_as_they_are_typed() {
        for typed in ["500ms", "1500ms", "30s", "90s", "5m", "1h", "36h"] {
            let interval = parse_interval(typed).unwrap();
            assert_eq!(describe_interval(interval), typed);
        }
        assert_eq!(describe_interval(Duration::from_secs(120)), "2m");
        assert_eq!(describe_interval(Duration::ZERO), "0ms");
        assert_eq!(describe_interval(Duration::from_micros(1500)), "1.5ms");
    }

    #[test]
    fn a_sensor_is_given_an_interval_or_turned_off() {
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
    fn changes_leave_what_they_do_not_name() {
        let changes = Changes {
            id: None,
            power_mode: Some(PowerMode::Battery),
            sensors: vec![
                sensor_change("capacitance0=off").unwrap(),
                sensor_change("humidity=1m").unwrap(),
            ],
            pushgateway: Some(pushgateway_change("[fd12:3456::1]:9091").unwrap()),
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
        assert_eq!(changed, expected);
    }

    #[test]
    fn a_first_configuration_needs_an_id_and_a_power_mode() {
        let mut changes = Changes {
            id: Some(3),
            power_mode: None,
            sensors: vec![sensor_change("capacitance0=10s").unwrap()],
            pushgateway: None,
        };
        assert!(changes.apply(None).is_err());

        changes.power_mode = Some(PowerMode::Aux);
        let first = changes.apply(None).unwrap();
        let mut expected = config();
        expected.sensor_config.0[Sensor::Temperature] = None;
        assert_eq!(first, expected);
    }

    #[test]
    fn a_configuration_is_described_a_line_for_each_thing_in_it() {
        let text = describe(&config());
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines[..4],
            [
                "  id            3",
                "  power mode    aux",
                "  pushgateway   none",
                "  capacitance0  every 10s"
            ]
        );
        assert!(lines.contains(&"  capacitance1  off"), "{text}");
        assert!(lines.contains(&"  temperature   every 5m"), "{text}");
        assert_eq!(lines.len(), 3 + Sensor::value_variants().len());
    }

    #[test]
    fn a_pushgateway_is_an_ipv6_address_and_a_port_or_none() {
        let address = SocketAddrV6::new("fd12:3456::1".parse().unwrap(), 9091, 0, 0);
        assert_eq!(
            pushgateway_change("[fd12:3456::1]:9091"),
            Ok(PushgatewayChange(Some(address)))
        );
        assert_eq!(pushgateway_change("none"), Ok(PushgatewayChange(None)));
        for malformed in ["fd12:3456::1", "192.168.1.2:9091", "pushgateway:9091", ""] {
            assert!(pushgateway_change(malformed).is_err(), "{malformed}");
        }
    }
}
