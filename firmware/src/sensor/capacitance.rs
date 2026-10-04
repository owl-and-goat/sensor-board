use bitvec::prelude::*;
use defmt::debug;
use embassy_stm32::{
    Peri, gpio,
    i2c::{self, I2c, Master},
    mode::Async,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use protocol::SensorReadError;

use crate::i2c_device;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Channel {
    Ch0,
    Ch1,
    Ch2,
    Ch3,
}

impl Channel {
    pub fn to_u8(self) -> u8 {
        match self {
            Channel::Ch0 => 0b00,
            Channel::Ch1 => 0b01,
            Channel::Ch2 => 0b10,
            Channel::Ch3 => 0b11,
        }
    }

    pub fn from_u8(val: u8) -> Self {
        match val {
            0b00 => Channel::Ch0,
            0b01 => Channel::Ch1,
            0b10 => Channel::Ch2,
            0b11 => Channel::Ch3,
            n => panic!("Invalid value for channel: {n}"),
        }
    }

    pub fn data_register_msb_address(self) -> u8 {
        match self {
            Channel::Ch0 => 0x00,
            Channel::Ch1 => 0x02,
            Channel::Ch2 => 0x04,
            Channel::Ch3 => 0x06,
        }
    }

    pub fn data_register_lsb_address(self) -> u8 {
        match self {
            Channel::Ch0 => 0x01,
            Channel::Ch1 => 0x03,
            Channel::Ch2 => 0x05,
            Channel::Ch3 => 0x07,
        }
    }
}

mod register {
    pub const CONFIG: u8 = 0x1A;
}

#[derive(Clone, Copy)]
enum SensorActivateMode {
    FullCurrent,
    LowPower,
}

#[derive(Clone, Copy)]
enum ReferenceClockSource {
    InternalOscillator,
    Clkin,
}

#[derive(Clone, Copy)]
struct Config {
    /// Active Channel Selection
    ///
    /// Selects channel for continuous conversions when MUX_CONFIG.SEQUENTIAL is 0.
    ///
    /// b00: Perform continuous conversions on Channel 0
    /// b01: Perform continuous conversions on Channel 1
    /// b10: Perform continuous conversions on Channel 2 (FDC2114, FDC2214 only)
    /// b11: Perform continuous conversions on Channel 3 (FDC2114, FDC2214 only)
    active_chan: Channel,

    /// Sleep Mode Enable
    ///
    /// Enter or exit low power Sleep Mode
    sleep_mode: bool,

    // Sensor Activation Mode Selection
    sensor_activate: SensorActivateMode,

    reference_clock_source: ReferenceClockSource,
    intb_disable: bool,
    high_current_drive: bool,
}

impl Config {
    pub fn serialize(&self) -> u16 {
        let mut res = 0u16;
        let bits = res.view_bits_mut::<Lsb0>();

        bits[14..15].store(self.active_chan.to_u8());
        bits.set(13, self.sleep_mode);
        bits.set(12, true); // "Reserved. Set to b1."
        bits.set(
            11,
            match self.sensor_activate {
                SensorActivateMode::FullCurrent => false,
                SensorActivateMode::LowPower => true,
            },
        );
        bits.set(10, true); // "Reserved. Set to b1."
        bits.set(
            9,
            match self.reference_clock_source {
                ReferenceClockSource::InternalOscillator => false,
                ReferenceClockSource::Clkin => true,
            },
        );
        bits.set(8, false); // "Reserved. Set to b0."
        bits.set(7, self.intb_disable);
        bits.set(6, self.high_current_drive);
        bits[0..5].store(0b00_0001);

        res
    }

    pub fn deserialize(val: u16) -> Self {
        let bits = val.view_bits::<Lsb0>();
        Self {
            active_chan: Channel::from_u8(bits[14..15].load()),
            sleep_mode: bits[13],
            sensor_activate: if bits[11] {
                SensorActivateMode::LowPower
            } else {
                SensorActivateMode::FullCurrent
            },
            reference_clock_source: if bits[9] {
                ReferenceClockSource::Clkin
            } else {
                ReferenceClockSource::InternalOscillator
            },
            intb_disable: bits[7],
            high_current_drive: bits[6],
        }
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum ChannelReadError {
    I2cError(i2c::Error),
    WatchdogTimeoutError,
    AmplitudeWarning,
}

impl From<i2c::Error> for ChannelReadError {
    fn from(value: i2c::Error) -> Self {
        Self::I2cError(value)
    }
}

impl From<ChannelReadError> for SensorReadError {
    fn from(value: ChannelReadError) -> Self {
        use ChannelReadError::*;

        match value {
            I2cError(i2c::Error::Bus) => Self::Bus,
            I2cError(i2c::Error::Arbitration) => Self::Arbitration,
            I2cError(i2c::Error::Nack) => Self::Nack,
            I2cError(i2c::Error::Timeout) => Self::Timeout,
            I2cError(i2c::Error::Crc) => Self::Crc,
            I2cError(i2c::Error::Overrun) => Self::Overrun,
            I2cError(i2c::Error::ZeroLengthTransfer) => Self::ZeroLengthTransfer,
            WatchdogTimeoutError => Self::WatchdogTimeoutError,
            AmplitudeWarning => Self::AmplitudeWarning,
        }
    }
}

pub struct CapacitanceSensor<'d> {
    shutdown: gpio::Output<'d>,
    i2c: &'d Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
}

impl<'d> CapacitanceSensor<'d> {
    pub const ADDRESS: u8 = i2c_device::addr::CAPACITANCE;

    pub fn new(
        shutdown_pin: Peri<'d, impl gpio::Pin>,
        i2c: &'d Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
    ) -> Self {
        Self {
            shutdown: gpio::Output::new(shutdown_pin, gpio::Level::Low, gpio::Speed::Low),
            i2c,
        }
    }

    #[expect(dead_code)]
    pub fn set_shutdown(&mut self, shutdown: bool) {
        self.shutdown.set_level(shutdown.into());
    }

    async fn write_register(&self, register: u8, value: u16) -> Result<(), i2c::Error> {
        self.i2c
            .lock()
            .await
            .write(
                Self::ADDRESS,
                &[register, value.to_be_bytes()[0], value.to_be_bytes()[1]],
            )
            .await
    }

    async fn read_register(&self, register: u8) -> Result<u16, i2c::Error> {
        let mut buf = [0u8; 2];
        self.i2c
            .lock()
            .await
            .write_read(Self::ADDRESS, &[register], &mut buf)
            .await?;
        Ok(u16::from_be_bytes(buf))
    }

    async fn get_config(&self) -> Result<Config, i2c::Error> {
        Ok(Config::deserialize(
            self.read_register(register::CONFIG).await?,
        ))
    }

    async fn set_config(&mut self, config: Config) -> Result<(), i2c::Error> {
        self.write_register(register::CONFIG, config.serialize())
            .await
    }

    async fn modify_config(
        &mut self,
        modify: impl FnOnce(&mut Config) -> (),
    ) -> Result<(), i2c::Error> {
        let mut config = self.get_config().await?;
        modify(&mut config);
        self.set_config(config).await
    }

    pub async fn read_channel_capacitance(
        &mut self,
        channel: Channel,
    ) -> Result<u32, ChannelReadError> {
        self.modify_config(|config| {
            config.sleep_mode = false;
            config.active_chan = channel;
        })
        .await?;

        let msb = self
            .read_register(channel.data_register_msb_address())
            .await?;
        if msb.view_bits::<Lsb0>()[13] {
            debug!("Watchdog timeout");
            return Err(ChannelReadError::WatchdogTimeoutError);
        }
        if msb.view_bits::<Lsb0>()[12] {
            debug!("amplitude warning");
            return Err(ChannelReadError::AmplitudeWarning);
        }

        let lsb = self
            .read_register(channel.data_register_lsb_address())
            .await?;

        Ok(((msb as u32) << 16) | (lsb as u32))
    }
}
