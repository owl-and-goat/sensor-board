//! Capacitance sensor driver (FDC2214)

use defmt::debug;
use embassy_stm32::{
    Peri, gpio,
    i2c::{self, I2c, Master},
    mode::Async,
};
use protocol::SensorReadError;

use crate::{
    i2c_device,
    sensor::{Shared, SharedI2c},
};
pub use ll::Channel;
use ll::Fdc2214;

mod ll {
    device_driver::compile!(
        manifest: "ddsl/fdc2214.ddsl"
    );
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
            I2cError(e) => super::read_error(e),
            WatchdogTimeoutError => Self::WatchdogTimeoutError,
            AmplitudeWarning => Self::AmplitudeWarning,
        }
    }
}

pub struct CapacitanceSensor<'d> {
    shutdown: gpio::Output<'d>,
    driver: Fdc2214<SharedI2c<'d>>,
}

impl<'d> CapacitanceSensor<'d> {
    pub const BUS_ADDRESS: u8 = i2c_device::addr::CAPACITANCE;

    pub fn new(
        shutdown_pin: Peri<'d, impl gpio::Pin>,
        i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    ) -> Self {
        Self {
            shutdown: gpio::Output::new(shutdown_pin, gpio::Level::Low, gpio::Speed::Low),
            driver: Fdc2214::new(SharedI2c {
                i2c_handle,
                bus_address: Self::BUS_ADDRESS,
            }),
        }
    }

    #[expect(dead_code)]
    pub fn set_shutdown(&mut self, shutdown: bool) {
        self.shutdown.set_level(shutdown.into());
    }

    pub async fn read_channel_capacitance(
        &mut self,
        channel: Channel,
    ) -> Result<u32, ChannelReadError> {
        self.driver
            .config()
            .modify_async(|config| {
                config.set_sleep_mode_en(false);
                config.set_active_chan(channel);
                config.set_reserved_12(true);
                config.set_reserved_10(true);
                config.set_reserved_5_0(0b00_0001);
            })
            .await?;

        let msb = self.driver.data().read_at_async(channel).await?;
        if msb.err_wd() {
            debug!("Watchdog timeout");
            return Err(ChannelReadError::WatchdogTimeoutError);
        }
        if msb.err_aw() {
            debug!("amplitude warning");
            return Err(ChannelReadError::AmplitudeWarning);
        }

        let lsb = self.driver.data_lsb().read_at_async(channel).await?;

        Ok((u32::from(msb.data_msb()) << 16) | u32::from(lsb.value()))
    }
}
