//! A board's configuration: which sensors it reads and how often, and how it
//! is powered. It is shown and changed on an attached board, or through one
//! on a board on its network.

use std::time::Duration;

use anyhow::{Result, bail};
use clap::{Args, ValueEnum};
use protocol::{BoardConfig, BoardId, PowerMode, Sensor, SensorConfig, SensorsConfig};

use crate::board::Board;

/// What to change in a board's configuration.
#[derive(Args)]
pub struct Changes {
    /// The board's number
    #[arg(long, value_name = "N")]
    id: Option<u8>,

    /// How the board is powered
    #[arg(long, value_enum)]
    power_mode: Option<PowerMode>,

    /// Have the board read a sensor at an interval, as in `temperature=30s` (in ms, s, m or h),
    /// or not at all, as in `temperature=off`. Can be given several times
    #[arg(long = "sensor", value_name = "SENSOR=INTERVAL", value_parser = sensor_change)]
    sensors: Vec<SensorChange>,
}

/// What a sensor's configuration is to be. `None` turns the sensor off.
#[derive(Debug, Clone, PartialEq, Eq)]
struct SensorChange {
    sensor: Sensor,
    config: Option<SensorConfig>,
}

impl Changes {
    fn is_empty(&self) -> bool {
        self.id.is_none() && self.power_mode.is_none() && self.sensors.is_empty()
    }

    /// `current` with the changes made to it. A board that has no
    /// configuration gets one that is made of the changes alone, which then
    /// have to say what a configuration cannot be without.
    fn apply(&self, current: Option<BoardConfig>) -> Result<BoardConfig> {
        let mut config = match (current, self.id, self.power_mode) {
            (Some(config), _, _) => config,
            (None, Some(board_id), Some(power_mode)) => BoardConfig {
                board_id,
                sensor_config: SensorsConfig::default(),
                power_mode,
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
        Ok(config)
    }
}

/// Print a board's configuration: that of `remote`, which `board` asks over
/// its network, or `board`'s own.
pub async fn show(board: &Board, remote: Option<BoardId>) -> Result<()> {
    let target = target(board, remote)?;
    match board.board_config(target).await? {
        Some(config) => print!("Board {target}\n{}", describe(&config)),
        None => println!("Board {target} has no configuration."),
    }
    Ok(())
}

/// Make `changes` to a board's configuration: that of `remote`, which
/// `board` reaches over its network, or `board`'s own.
pub async fn set(board: &Board, remote: Option<BoardId>, changes: &Changes) -> Result<()> {
    if changes.is_empty() {
        bail!("nothing to change: give --id, --power-mode or --sensor");
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

/// The configuration as text: a line for each thing in it.
fn describe(config: &BoardConfig) -> String {
    let BoardConfig {
        board_id,
        sensor_config,
        power_mode,
    } = config;
    let mut text = format!("  {:<12}  {board_id}\n", "id");
    text += &format!("  {:<12}  {}\n", "power mode", name(power_mode));
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

/// What the command line calls `value`.
fn name(value: &impl ValueEnum) -> String {
    let name = value.to_possible_value();
    name.map_or_else(String::new, |name| name.get_name().to_owned())
}

/// A `--sensor` argument.
fn sensor_change(text: &str) -> Result<SensorChange, String> {
    let Some((sensor, interval)) = text.split_once('=') else {
        return Err("write it as SENSOR=INTERVAL, as in temperature=30s".to_owned());
    };
    let Ok(sensor) = Sensor::from_str(sensor, true) else {
        let sensors: Vec<String> = Sensor::value_variants().iter().map(name).collect();
        return Err(format!(
            "there is no sensor {sensor}; there are {}",
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

/// An interval as it is typed: a whole number of a unit.
fn parse_interval(text: &str) -> Result<Duration, String> {
    let malformed = || format!("{text} is no interval: write one as 500ms, 30s, 5m or 1h");

    let number = text.trim_end_matches(|c: char| c.is_ascii_alphabetic());
    let unit = match &text[number.len()..] {
        "ms" => Duration::from_millis(1),
        "s" => Duration::from_secs(1),
        "m" => Duration::from_secs(60),
        "h" => Duration::from_secs(3600),
        _ => return Err(malformed()),
    };
    match number.parse::<u32>() {
        Ok(0) => Err("an interval is longer than nothing".to_owned()),
        Ok(number) => Ok(unit * number),
        Err(_) => Err(malformed()),
    }
}

/// An interval as it is typed, in the largest unit that it is a whole number
/// of.
fn describe_interval(interval: Duration) -> String {
    let ms = interval.as_millis();
    if interval.subsec_nanos() % 1_000_000 != 0 {
        // Not anything that can be typed.
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
        };
        let changed = changes.apply(Some(config())).unwrap();

        let mut expected = config();
        expected.power_mode = PowerMode::Battery;
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
            lines[..3],
            [
                "  id            3",
                "  power mode    aux",
                "  capacitance0  every 10s"
            ]
        );
        assert!(lines.contains(&"  capacitance1  off"), "{text}");
        assert!(lines.contains(&"  temperature   every 5m"), "{text}");
        assert_eq!(lines.len(), 2 + Sensor::value_variants().len());
    }
}
