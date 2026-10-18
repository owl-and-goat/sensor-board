use core::net::SocketAddrV6;
use core::ops::RangeInclusive;
use core::time::Duration;

use enum_map::{Enum, EnumMap};
use postcard_rpc::Key;
use postcard_schema::{
    Schema,
    schema::{self, NamedType},
};
use serde::{Deserialize, Serialize};
use strum::EnumCount;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema, Enum, EnumCount)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
pub enum Sensor {
    Capacitance0,
    Capacitance1,
    Capacitance2,
    Capacitance3,
    Distance,
    Color,
    Temperature,
    Humidity,
    Acceleration,
    Sound,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorConfig {
    pub poll_interval: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
pub enum PowerMode {
    /// The device is powered via aux power. It does not register as a sleepy end device, allowing
    /// it to act as a Thread router. It serves its metrics on a Prometheus scrape endpoint.
    Aux,

    /// The device is powered via battery, meaning it should try to conserve power as much as
    /// possible. It pushes its metrics to the configured Pushgateway.
    Battery,
}

/// Map from sensors to their config, or None if the sensor should be disabled
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SensorsConfig(pub EnumMap<Sensor, Option<SensorConfig>>);

impl Schema for SensorsConfig {
    const SCHEMA: &'static schema::NamedType = &NamedType {
        name: "SensorConfig",
        ty: &schema::DataModelType::Tuple(&[<Option<SensorConfig>>::SCHEMA; <Sensor>::COUNT]),
    };
}

/// How long the microphone listens for a sound measurement, unless the
/// configuration sets another length.
pub const DEFAULT_SOUND_DURATION: Duration = Duration::from_secs(1);

/// The shortest and the longest sound measurement. The firmware measures
/// for the nearest of the two if the configuration is outside them.
pub const SOUND_DURATIONS: RangeInclusive<Duration> =
    Duration::from_millis(10)..=Duration::from_secs(10);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct BoardConfig {
    pub board_id: u8,
    pub sensor_config: SensorsConfig,
    pub power_mode: PowerMode,
    /// The Prometheus Pushgateway to push metrics to. Used only in
    /// [`PowerMode::Battery`].
    pub pushgateway: Option<SocketAddrV6>,
    /// How long the microphone listens for each sound measurement. The
    /// microphone is on for 100 ms longer, to settle first.
    pub sound_duration: Duration,
}

/// The layout of [`BoardConfig`] before `sound_duration` was added, which
/// boards stored before 2026-10-18. The type keeps its name, because the
/// name is part of the key.
mod without_sound_duration {
    use super::*;

    #[derive(Serialize, Deserialize, Schema)]
    pub struct BoardConfig {
        pub board_id: u8,
        pub sensor_config: SensorsConfig,
        pub power_mode: PowerMode,
        pub pushgateway: Option<SocketAddrV6>,
    }

    impl BoardConfig {
        pub fn key() -> [u8; 8] {
            Key::for_path::<BoardConfig>("config/stored").to_bytes()
        }
    }

    impl From<BoardConfig> for super::BoardConfig {
        fn from(old: BoardConfig) -> Self {
            super::BoardConfig {
                board_id: old.board_id,
                sensor_config: old.sensor_config,
                power_mode: old.power_mode,
                pushgateway: old.pushgateway,
                sound_duration: DEFAULT_SOUND_DURATION,
            }
        }
    }
}

impl BoardConfig {
    /// The longest encoded configuration: the key, then every field at its
    /// largest size (postcard encodes the intervals and the port as varints).
    pub const MAX_LEN: usize = 8 + 1 + Sensor::COUNT * (1 + 10 + 5) + 1 + (1 + 16 + 3) + (10 + 5);

    /// Encode the configuration for storage in flash: a key derived from the
    /// layout of this type, then the postcard encoding. `None` if `buf` is
    /// too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        crate::encode_with_prefix(&Self::key(), self, buf)
    }

    /// Decode a stored configuration. A configuration in the layout from
    /// before `sound_duration` is converted, with the default sound duration.
    /// `None` if `bytes` was encoded with any other layout, so that such a
    /// configuration is rejected instead of misread.
    pub fn decode(bytes: &[u8]) -> Option<BoardConfig> {
        crate::decode_with_prefix(&Self::key(), bytes).or_else(|| {
            let old = without_sound_duration::BoardConfig::key();
            crate::decode_with_prefix::<without_sound_duration::BoardConfig>(&old, bytes)
                .map(BoardConfig::from)
        })
    }

    fn key() -> [u8; 8] {
        Key::for_path::<BoardConfig>("config/stored").to_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> BoardConfig {
        let mut sensors = SensorsConfig::default();
        sensors.0[Sensor::Capacitance1] = Some(SensorConfig {
            poll_interval: Duration::from_secs(10),
        });
        sensors.0[Sensor::Acceleration] = Some(SensorConfig {
            poll_interval: Duration::from_millis(250),
        });
        BoardConfig {
            board_id: 7,
            sensor_config: sensors,
            power_mode: PowerMode::Aux,
            pushgateway: Some("[fd12:3456::1]:9091".parse().unwrap()),
            sound_duration: Duration::from_millis(125),
        }
    }

    /// A configuration as a firmware from before `sound_duration` stored it.
    const STORED_WITHOUT_SOUND_DURATION: [u8; 41] = [
        0x85, 0x7c, 0x93, 0x1b, 0x6f, 0xdc, 0xda, 0x3f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x0a, 0x00, 0x01, 0x01, 0xfd, 0x94, 0x69, 0x18, 0xdb, 0xe8, 0x37,
        0xdd, 0x54, 0x57, 0x0a, 0xff, 0xfe, 0xe2, 0x84, 0x30, 0x83, 0x47,
    ];

    #[test]
    fn config_without_sound_duration_is_converted() {
        let mut sensors = SensorsConfig::default();
        sensors.0[Sensor::Sound] = Some(SensorConfig {
            poll_interval: Duration::from_secs(10),
        });
        let expected = BoardConfig {
            board_id: 5,
            sensor_config: sensors,
            power_mode: PowerMode::Battery,
            pushgateway: Some(
                "[fd94:6918:dbe8:37dd:5457:aff:fee2:8430]:9091"
                    .parse()
                    .unwrap(),
            ),
            sound_duration: DEFAULT_SOUND_DURATION,
        };
        assert_eq!(
            BoardConfig::decode(&STORED_WITHOUT_SOUND_DURATION),
            Some(expected)
        );
    }

    #[test]
    fn config_round_trips() {
        let mut buf = [0; BoardConfig::MAX_LEN];
        let encoded = config().encode(&mut buf).unwrap();
        assert_eq!(BoardConfig::decode(encoded), Some(config()));
    }

    #[test]
    fn config_with_other_layout_is_rejected() {
        let mut buf = [0; BoardConfig::MAX_LEN];
        let encoded = config().encode(&mut buf).unwrap();
        let mut other = encoded.to_vec();
        other[0] ^= 1;
        assert_eq!(BoardConfig::decode(&other), None);
        assert_eq!(BoardConfig::decode(&encoded[..encoded.len() - 1]), None);
        assert_eq!(BoardConfig::decode(&[]), None);
        // Erased flash.
        assert_eq!(BoardConfig::decode(&[0xff; BoardConfig::MAX_LEN]), None);
    }

    #[test]
    fn longest_config_has_max_len() {
        let mut longest = config();
        longest.board_id = u8::MAX;
        longest.pushgateway = Some(SocketAddrV6::new([0xffff; 8].into(), u16::MAX, 0, 0));
        longest.sound_duration = Duration::MAX;
        for (_, sensor) in &mut longest.sensor_config.0 {
            *sensor = Some(SensorConfig {
                poll_interval: Duration::MAX,
            });
        }
        let mut buf = [0; 2 * BoardConfig::MAX_LEN];
        let encoded = longest.encode(&mut buf).unwrap();
        assert_eq!(encoded.len(), BoardConfig::MAX_LEN);
    }
}
