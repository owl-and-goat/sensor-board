use core::net::SocketAddrV6;
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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct BoardConfig {
    pub board_id: u8,
    pub sensor_config: SensorsConfig,
    pub power_mode: PowerMode,
    /// The Prometheus Pushgateway to push metrics to. Used only in
    /// [`PowerMode::Battery`].
    pub pushgateway: Option<SocketAddrV6>,
}

impl BoardConfig {
    /// The longest encoded configuration: the key, then every field at its
    /// largest size (postcard encodes the intervals and the port as varints).
    pub const MAX_LEN: usize = 8 + 1 + Sensor::COUNT * (1 + 10 + 5) + 1 + (1 + 16 + 3);

    /// Encode the configuration for storage in flash: a key derived from the
    /// layout of this type, then the postcard encoding. `None` if `buf` is
    /// too short.
    pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        crate::encode_behind(&Self::key(), self, buf)
    }

    /// Decode a stored configuration. `None` if `bytes` was not encoded with
    /// this layout, so a configuration that a firmware with a different
    /// layout stored is rejected instead of misread.
    pub fn decode(bytes: &[u8]) -> Option<BoardConfig> {
        crate::decode_behind(&Self::key(), bytes)
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
        }
    }

    #[test]
    fn config_survives_being_kept() {
        let mut buf = [0; BoardConfig::MAX_LEN];
        let encoded = config().encode(&mut buf).unwrap();
        assert_eq!(BoardConfig::decode(encoded), Some(config()));
    }

    #[test]
    fn config_of_another_layout_is_not_understood() {
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
    fn longest_config_is_as_long_as_one_may_be() {
        let mut longest = config();
        longest.board_id = u8::MAX;
        longest.pushgateway = Some(SocketAddrV6::new([0xffff; 8].into(), u16::MAX, 0, 0));
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
