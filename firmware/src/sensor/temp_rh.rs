//! Temperature & humidity sensor driver (SHT40-AD1F-R2).

mod ll {
    device_driver::compile!(
        manifest: "ddsl/sht40_ad1f_r2.ddsl"
    );
}

use device_driver::{AsyncCommandInterface, CommandInterfaceBase};
use embassy_stm32::{
    i2c::{self, I2c, Master},
    mode::Async,
};
use embassy_time::{Instant, Timer};
use ll::Sht40Ad1F;
use protocol::{
    HeaterDuration, HeaterPower, Precision, SensorReadError, TempRh, TempRhReq, TempRhResult,
};

use crate::sensor::Shared;

#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum TempRhSensorError {
    I2cError(i2c::Error),

    /// Sensor never came up with a response to our command
    NeverFinished,
}

impl From<i2c::Error> for TempRhSensorError {
    fn from(value: i2c::Error) -> Self {
        Self::I2cError(value)
    }
}

impl From<TempRhSensorError> for SensorReadError {
    fn from(value: TempRhSensorError) -> Self {
        match value {
            TempRhSensorError::I2cError(e) => super::read_error(e),
            TempRhSensorError::NeverFinished => SensorReadError::Timeout,
        }
    }
}

/// Temperature measurement in °C.
pub struct Temp(pub f32);

impl Temp {
    pub(super) fn from_raw(raw: u16) -> Self {
        // see page 12 of the datasheet; temp conversion in °C
        Self(-45f32 + 175f32 * ((raw as f32) / (0xffff as f32)))
    }
}

/// Relative humidity measurement, as a percentage.
///
/// NOTE: this *may* fall outside the 0-100% range. The datasheet for the sensor
/// is decidedly cagey about what, exactly, this means, but it does mention that
/// it's possible.
pub struct Rh(pub f32);

impl Rh {
    pub(super) fn from_raw(raw: u16) -> Self {
        // see page 12 of the datasheet; %RH conversion
        Self(-6f32 + 125f32 * ((raw as f32) / (0xffff as f32)))
    }
}

/// The command interface specific to the Sht40. For each command, we have to
/// perform *two* transactions --- one to indicate the command we're performing,
/// and one to read back the data the command generated. See page 11 in the
/// datasheet for details.
struct Sht40CommandInterface<'d> {
    pub i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    pub bus_address: u8,
}

impl Sht40CommandInterface<'_> {
    /// The longest operations for this device, the 1 s heater and a
    /// measurement, take up to 1.1 s by the datasheet.
    pub const MAXIMUM_TRANSACTION_TIME_MS: u64 = 1200;
}

impl<'d> CommandInterfaceBase for Sht40CommandInterface<'d> {
    type Error = TempRhSensorError;
    type AddressType = u8;
}

impl<'d> AsyncCommandInterface for Sht40CommandInterface<'d> {
    async fn dispatch_command(
        &mut self,
        address: Self::AddressType,
        _input: &mut [u8],
        _input_metadata: &device_driver::FieldsetMetadata,
        output: &mut [u8],
        _output_metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        if output.len() != 4 {
            unimplemented!("we only deal with two-word chunks here folks");
        }

        self.i2c_handle
            .lock()
            .await
            .write(self.bus_address, &[address])
            .await?;

        // this device is weird. after sending a command, you have to start
        // a *new* read transaction after an unspecified amount of time to
        // actually get the data, so read until we don't get NACKed.
        let transaction_start = Instant::now();
        let mut pause_ms = 1;
        let mut full_output = [0u8; 6];

        while Instant::now().duration_since(transaction_start).as_millis()
            < Self::MAXIMUM_TRANSACTION_TIME_MS
        {
            match self
                .i2c_handle
                .lock()
                .await
                .read(self.bus_address, &mut full_output)
                .await
            {
                Ok(()) => {
                    for (word, out) in full_output.chunks(3).zip(output.chunks_mut(2)) {
                        // TODO: check CRC; none of the other sensors send one, so
                        // this is probably fine.
                        out.copy_from_slice(&word[..2]);
                    }
                    return Ok(());
                }

                // XXX: apparently using async read() here hits an embassy bug
                // if the read header gets NACKed, in that a NACK immediately
                // after the read header causes a timeout.
                Err(i2c::Error::Nack) | Err(i2c::Error::Timeout) => {
                    Timer::after_millis(pause_ms).await;
                    pause_ms = pause_ms * 2;
                    continue;
                }

                Err(e) => return Err(e.into()),
            }
        }

        Err(TempRhSensorError::NeverFinished)
    }
}

pub struct TempRhSensor<'d> {
    driver: Sht40Ad1F<Sht40CommandInterface<'d>>,
}

impl<'d> TempRhSensor<'d> {
    pub const BUS_ADDRESS: u8 = 0x44;

    pub async fn init(
        i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    ) -> Result<Self, TempRhSensorError> {
        let mut driver = Sht40Ad1F::new(Sht40CommandInterface {
            i2c_handle,
            bus_address: Self::BUS_ADDRESS,
        });

        let serial = driver.read_serial_number().dispatch_out_async().await?;
        defmt::debug!("temp-rh serial number: {:x}", serial.serial());

        Ok(Self { driver })
    }

    pub async fn measure(&mut self, precision: Precision) -> Result<(Temp, Rh), TempRhSensorError> {
        let raw_result = (match precision {
            Precision::High => self.driver.measure_high_precision(),
            Precision::Medium => self.driver.measure_medium_precision(),
            Precision::Low => self.driver.measure_low_precision(),
        })
        .dispatch_out_async()
        .await?;

        let temp = Temp::from_raw(raw_result.temp());
        let rh = Rh::from_raw(raw_result.rh());
        Ok((temp, rh))
    }

    /// Take a measurement after engaging the heater to evaporate excess
    /// moisture. Measurement is always high precision.
    pub async fn measure_heated(
        &mut self,
        power: HeaterPower,
        duration: HeaterDuration,
    ) -> Result<(Temp, Rh), TempRhSensorError> {
        let raw_result = (match (power, duration) {
            (HeaterPower::Power200mW, HeaterDuration::Time1s) => {
                self.driver.heat_and_measure_200_m_w_1_s()
            }
            (HeaterPower::Power200mW, HeaterDuration::Time100ms) => {
                self.driver.heat_and_measure_200_m_w_100_ms()
            }
            (HeaterPower::Power110mW, HeaterDuration::Time1s) => {
                self.driver.heat_and_measure_110_m_w_1_s()
            }
            (HeaterPower::Power110mW, HeaterDuration::Time100ms) => {
                self.driver.heat_and_measure_110_m_w_100_ms()
            }
            (HeaterPower::Power20mW, HeaterDuration::Time1s) => {
                self.driver.heat_and_measure_20_m_w_1_s()
            }
            (HeaterPower::Power20mW, HeaterDuration::Time100ms) => {
                self.driver.heat_and_measure_20_m_w_100_ms()
            }
        })
        .dispatch_out_async()
        .await?;

        let temp = Temp::from_raw(raw_result.temp());
        let rh = Rh::from_raw(raw_result.rh());
        Ok((temp, rh))
    }

    /// A measurement as the protocol asks for it.
    pub async fn read(&mut self, req: TempRhReq) -> TempRhResult {
        let (Temp(temperature), Rh(humidity)) = match req {
            TempRhReq::Plain(precision) => self.measure(precision).await?,
            TempRhReq::Heated { power, duration } => self.measure_heated(power, duration).await?,
        };
        Ok(TempRh {
            temperature,
            humidity,
        })
    }
}
