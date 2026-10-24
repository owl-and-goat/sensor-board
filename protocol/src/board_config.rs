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

use crate::max_len::max_encoded_len;

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
    /// Whether the motion sensor is enabled.
    ///
    /// This is not a "sensor" in the same way as the other sensors, since it's interrupt-driven
    /// rather than polling
    pub motion_enabled: bool,
    /// The Home Assistant server to report the motion sensor's state to, at
    /// the webhook `sensor-board-<serial>`. Used in both power modes.
    pub home_assistant: Option<SocketAddrV6>,
}

/// The layout of [`BoardConfig`] before `home_assistant` was added. The type
/// keeps its name, because the name is part of the key.
mod v2 {
    use super::*;

    #[derive(Serialize, Deserialize, Schema)]
    pub struct BoardConfig {
        pub board_id: u8,
        pub sensor_config: SensorsConfig,
        pub power_mode: PowerMode,
        pub pushgateway: Option<SocketAddrV6>,
        pub sound_duration: Duration,
        pub motion_enabled: bool,
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
                sound_duration: old.sound_duration,
                motion_enabled: old.motion_enabled,
                home_assistant: None,
            }
        }
    }
}

/// The layout of [`BoardConfig`] before `motion_enabled` was added.
mod v1 {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
    pub struct BoardConfig {
        pub board_id: u8,
        pub sensor_config: SensorsConfig,
        pub power_mode: PowerMode,
        pub pushgateway: Option<SocketAddrV6>,
        pub sound_duration: Duration,
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
                sound_duration: old.sound_duration,
                motion_enabled: false,
                home_assistant: None,
            }
        }
    }
}

/// The layout of [`BoardConfig`] before `sound_duration` was added, which
/// boards stored before 2026-10-18.
mod v0 {
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
                motion_enabled: false,
                home_assistant: None,
            }
        }
    }
}

impl BoardConfig {
    /// The longest encoded configuration: the key, then the configuration.
    pub const MAX_LEN: usize = 8 + max_encoded_len::<BoardConfig>();

    /// Encode the configuration for storage in flash: a key derived from the
    /// layout of this type, then the postcard encoding. `None` if `buf` is
    /// too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        crate::encode_with_prefix(&Self::key(), self, buf)
    }

    /// Decode a stored configuration. A configuration in an older layout (v0
    /// to v2) is converted: whatever it lacks is off, and a missing sound
    /// duration is the default. `None` if `bytes` was encoded with any other
    /// layout, so that such a configuration is rejected instead of misread.
    pub fn decode(bytes: &[u8]) -> Option<BoardConfig> {
        crate::decode_with_prefix(&Self::key(), bytes)
            .or_else(|| {
                let v0 = v0::BoardConfig::key();
                crate::decode_with_prefix::<v0::BoardConfig>(&v0, bytes).map(BoardConfig::from)
            })
            .or_else(|| {
                let v1 = v1::BoardConfig::key();
                crate::decode_with_prefix::<v1::BoardConfig>(&v1, bytes).map(BoardConfig::from)
            })
            .or_else(|| {
                let v2 = v2::BoardConfig::key();
                crate::decode_with_prefix::<v2::BoardConfig>(&v2, bytes).map(BoardConfig::from)
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
            motion_enabled: true,
            home_assistant: Some("[fd12:3456::2]:8123".parse().unwrap()),
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
            motion_enabled: false,
            home_assistant: None,
        };
        assert_eq!(
            BoardConfig::decode(&STORED_WITHOUT_SOUND_DURATION),
            Some(expected)
        );
    }

    /// A configuration as a firmware from before `motion_enabled` stored it.
    const STORED_WITHOUT_MOTION: [u8; 46] = [
        0x09, 0xc2, 0x77, 0xda, 0x0e, 0x37, 0xad, 0x6f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x0a, 0x00, 0x01, 0x01, 0xfd, 0x94, 0x69, 0x18, 0xdb, 0xe8, 0x37,
        0xdd, 0x54, 0x57, 0x0a, 0xff, 0xfe, 0xe2, 0x84, 0x30, 0x83, 0x47, 0x00, 0xc0, 0xb2, 0xcd,
        0x3b,
    ];

    #[test]
    fn config_without_motion_is_converted() {
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
            sound_duration: Duration::from_millis(125),
            motion_enabled: false,
            home_assistant: None,
        };
        assert_eq!(BoardConfig::decode(&STORED_WITHOUT_MOTION), Some(expected));
    }

    /// A configuration as a firmware from before `home_assistant` stored it.
    const STORED_WITHOUT_HOME_ASSISTANT: [u8; 47] = [
        0x3c, 0xa1, 0x1c, 0x0b, 0x30, 0x97, 0xd2, 0xa9, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x0a, 0x00, 0x01, 0x01, 0xfd, 0x94, 0x69, 0x18, 0xdb, 0xe8, 0x37,
        0xdd, 0x54, 0x57, 0x0a, 0xff, 0xfe, 0xe2, 0x84, 0x30, 0x83, 0x47, 0x00, 0xc0, 0xb2, 0xcd,
        0x3b, 0x01,
    ];

    #[test]
    fn config_without_home_assistant_is_converted() {
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
            sound_duration: Duration::from_millis(125),
            motion_enabled: true,
            home_assistant: None,
        };
        assert_eq!(
            BoardConfig::decode(&STORED_WITHOUT_HOME_ASSISTANT),
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
    fn longest_config_fits() {
        let mut longest = config();
        longest.board_id = u8::MAX;
        longest.pushgateway = Some(SocketAddrV6::new([0xffff; 8].into(), u16::MAX, 0, 0));
        longest.sound_duration = Duration::MAX;
        longest.home_assistant = longest.pushgateway;
        for (_, sensor) in &mut longest.sensor_config.0 {
            *sensor = Some(SensorConfig {
                poll_interval: Duration::MAX,
            });
        }
        let mut buf = [0; BoardConfig::MAX_LEN];
        assert!(longest.encode(&mut buf).is_some());
    }
}
