//! Support for the various sensors on the board.
//!
//! Note that not all boards have all sensors.

use device_driver::{AsyncRegisterInterface, RegisterInterfaceBase};
use embassy_stm32::{
    i2c::{self, I2c, Master},
    mode::Async,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use protocol::SensorReadError;

pub mod capacitance;

/// A sensor that several tasks read: the RPC handlers, the reports and the
/// metrics.
pub type Shared<S> = Mutex<ThreadModeRawMutex, S>;

/// What the host is told of an I2C error.
fn read_error(e: i2c::Error) -> SensorReadError {
    match e {
        i2c::Error::Bus => SensorReadError::Bus,
        i2c::Error::Arbitration => SensorReadError::Arbitration,
        i2c::Error::Nack => SensorReadError::Nack,
        i2c::Error::Timeout => SensorReadError::Timeout,
        i2c::Error::Crc => SensorReadError::Crc,
        i2c::Error::Overrun => SensorReadError::Overrun,
        i2c::Error::ZeroLengthTransfer => SensorReadError::ZeroLengthTransfer,
    }
}

/// Mediates access to a shared I2C bus, which all our sensors are on.
pub struct SharedI2c<'d> {
    pub i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    pub bus_address: u8,
}

impl<'d> RegisterInterfaceBase for SharedI2c<'d> {
    type Error = i2c::Error;
    type AddressType = u8; // register address type, not bus address type
}

impl<'d> AsyncRegisterInterface for SharedI2c<'d> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c_handle
            .lock()
            .await
            .write_vectored(
                i2c::Address::SevenBit(self.bus_address),
                &[&[address], data],
            )
            .await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c_handle
            .lock()
            .await
            .write_read(self.bus_address, &[address], data)
            .await
    }
}
