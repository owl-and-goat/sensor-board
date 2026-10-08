//! Color sensor driver (CLS-16D24-44-DF8/TR8)

use embassy_stm32::{
    i2c::{self, I2c, Master},
    mode::Async,
};

use protocol::SensorReadError;

use crate::{
    i2c_device,
    sensor::{Shared, SharedI2c},
};
use ll::{Cls16D2444, DataFlag};

pub use ll::ColorChannel as Channel;

mod ll {
    device_driver::compile!(
        manifest: "ddsl/cls_16d24_44.ddsl"
    );
}

#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum ColorSensorError {
    I2cError(i2c::Error),
    InvalidProductId,
    /// The last conversion did not produce valid data, or none has finished
    /// since the sensor was enabled.
    DataInvalid,
    /// The channel's light is outside of the measurable range.
    OutOfRange,
}

impl From<i2c::Error> for ColorSensorError {
    fn from(value: i2c::Error) -> Self {
        Self::I2cError(value)
    }
}

impl From<ColorSensorError> for SensorReadError {
    fn from(value: ColorSensorError) -> Self {
        match value {
            ColorSensorError::I2cError(e) => super::read_error(e),
            ColorSensorError::InvalidProductId => Self::WrongProductId,
            ColorSensorError::DataInvalid => Self::DataInvalid,
            ColorSensorError::OutOfRange => Self::OutOfRange,
        }
    }
}

pub struct ColorSensor<'d> {
    driver: Cls16D2444<SharedI2c<'d>>,
}

impl<'d> ColorSensor<'d> {
    pub const BUS_ADDRESS: u8 = i2c_device::addr::COLOR;

    pub async fn init(
        i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    ) -> Result<Self, ColorSensorError> {
        let mut driver = Cls16D2444::new(SharedI2c {
            i2c_handle,
            bus_address: Self::BUS_ADDRESS,
        });

        let prod_id = u16::from_le_bytes([
            driver.prod_id_l().read_async().await?.value(),
            driver.prod_id_h().read_async().await?.value(),
        ]);

        let expected_prod_id = u16::from_le_bytes([
            driver.prod_id_l().reset_value().value(),
            driver.prod_id_h().reset_value().value(),
        ]);

        if prod_id != expected_prod_id {
            return Err(ColorSensorError::InvalidProductId);
        }

        // software-reset the sensor so it's in a known state
        driver
            .sysm_ctrl()
            .write_async(|flags| flags.set_swrst(true))
            .await?;

        // The power-on flag pulls INT low until it is cleared.
        driver
            .int_flag()
            .write_async(|flags| flags.set_int_por(false))
            .await?;
        // Measure continuously, with no wait in between.
        driver
            .sysm_ctrl()
            .write_async(|ctrl| {
                ctrl.set_en_cls(true);
                ctrl.set_en_ir(true);
            })
            .await?;

        Ok(Self { driver })
    }

    pub async fn read_channel(&mut self, c: Channel) -> Result<u16, ColorSensorError> {
        if self.driver.int_flag().read_async().await?.data_flag() == DataFlag::Invalid {
            return Err(ColorSensorError::DataInvalid);
        }
        if self.driver.error_flag().read_async().await?.err(c) {
            return Err(ColorSensorError::OutOfRange);
        }

        Ok(self.driver.color().read_at_async(c).await?.value())
    }
}
