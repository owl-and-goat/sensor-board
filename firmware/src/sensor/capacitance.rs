//! Capacitance sensor driver (FDC2214)
//!
//! The schematic says the chip is an FDC2114, but the boards have an FDC2214
//! on them. It has the same pins, and gives a 28-bit result instead of a
//! 12-bit one.
//!
//! Each channel has an 18 µH inductor (L3 to L6) in parallel with a 33 pF
//! capacitor (C25 to C28), connected between INxA and INxB. The electrode on
//! J8 to J11 is connected to INxA. TI calls this the single-ended sensor
//! configuration. The inductor and capacitor form a tank circuit, and the
//! chip measures the frequency it oscillates at. Capacitance on the
//! electrode lowers that frequency. CLKIN is connected to ground, so the chip
//! uses its internal oscillator as the reference clock.
//!
//! The chip is kept in sleep mode between readings. To take a reading, the
//! driver wakes it up on one channel, waits for a conversion to finish, and
//! puts it back to sleep.

use core::f32::consts::PI;

use defmt::debug;
use embassy_stm32::{
    Peri, gpio,
    i2c::{self, I2c, Master},
    mode::Async,
};
use embassy_time::{Duration, Timer};
use protocol::{Capacitance, SensorReadError};

use crate::{
    i2c_device,
    sensor::{Shared, SharedI2c},
};
pub use ll::Channel;
use ll::{Fdc2214, FinSel, RefClkSrc, SensorActivateSel};

mod ll {
    device_driver::compile!(
        manifest: "ddsl/fdc2214.ddsl"
    );
}

/// The inductance of every channel's tank, in H.
const TANK_INDUCTANCE: f32 = 18e-6;

/// The capacitance of every channel's tank, in pF.
const TANK_CAPACITANCE: f32 = 33.0;

/// The typical frequency of the internal oscillator, in Hz. The datasheet
/// allows anywhere from 35 to 55 MHz.
const F_INT: f32 = 43.4e6;

/// The divider from the oscillator frequency to the reference frequency:
/// f_REF = f_INT / FREF_DIVIDER. When only one channel is converting, f_REF
/// can be at most 35 MHz.
const FREF_DIVIDER: u16 = 2;

/// The chip divides the sensor frequency by 2 before measuring it, because
/// the datasheet requires that for a single-ended sensor. This matches
/// `FinSel::Div2` in `init`.
const FIN_SEL: f32 = 2.0;

/// The length of a conversion, in units of 16 reference clock cycles. 65536
/// cycles give about 16 effective bits (datasheet, section 7.3.2), which is a
/// resolution of about 0.01 pF.
const RCOUNT: u16 = 4096;

/// How long the sensor is given to settle before each conversion, in units of
/// 16 reference clock cycles. For this tank with up to 100 pF on the
/// electrode, the datasheet's minimum is 8.
const SETTLECOUNT: u16 = 64;

/// The drive current from TI's example circuit, which uses the same inductor
/// and capacitor. In TI's example it gave an oscillation amplitude of 1.68 V
/// peak (datasheet, section 8.2.3.1). The amplitude hasn't been measured on
/// our boards.
const IDRIVE: u8 = 15;

/// The chip's result is the ratio of the measured frequency to f_REF,
/// multiplied by this.
const DATA_SCALE: f32 = (1 << 28) as f32;

/// The result the chip gives when the frequency is too low to measure.
const UNDER_RANGE: u32 = 0;

/// The result the chip gives when the frequency is too high to measure.
const OVER_RANGE: u32 = 0x0FFF_FFFF;

/// How long it takes from waking the chip up to a finished conversion, when
/// the internal oscillator is at its slowest (35 MHz). The chip waits 16384
/// oscillator cycles before it starts the sensor, then waits SETTLECOUNT for
/// it to settle, then converts for RCOUNT. That adds up to 4.27 ms.
const CONVERSION_TIME: Duration = Duration::from_millis(5);

/// How long the chip takes to start answering on I2C after SD goes low.
const WAKEUP_TIME: Duration = Duration::from_millis(2);

#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum CapacitanceSensorError {
    I2cError(i2c::Error),
    /// The device at the sensor's address is not an FDC2214.
    WrongDeviceId,
    /// The sensor did not oscillate.
    WatchdogTimeoutError,
    /// The sensor's frequency is too low or too high for the chip to measure.
    OutOfRange,
}

impl From<i2c::Error> for CapacitanceSensorError {
    fn from(value: i2c::Error) -> Self {
        Self::I2cError(value)
    }
}

impl From<CapacitanceSensorError> for SensorReadError {
    fn from(value: CapacitanceSensorError) -> Self {
        use CapacitanceSensorError::*;

        match value {
            I2cError(e) => super::read_error(e),
            WrongDeviceId => Self::WrongProductId,
            WatchdogTimeoutError => Self::WatchdogTimeoutError,
            OutOfRange => Self::OutOfRange,
        }
    }
}

pub struct CapacitanceSensor<'d> {
    _shutdown: gpio::Output<'d>,
    driver: Fdc2214<SharedI2c<'d>>,
}

impl<'d> CapacitanceSensor<'d> {
    pub const BUS_ADDRESS: u8 = i2c_device::addr::CAPACITANCE;

    /// Take the chip out of shutdown, check that it is an FDC2214, and
    /// configure every channel. Leaves the chip in sleep mode.
    pub async fn init(
        shutdown_pin: Peri<'d, impl gpio::Pin>,
        i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    ) -> Result<Self, CapacitanceSensorError> {
        let shutdown = gpio::Output::new(shutdown_pin, gpio::Level::Low, gpio::Speed::Low);
        Timer::after(WAKEUP_TIME).await;

        let mut driver = Fdc2214::new(SharedI2c {
            i2c_handle,
            bus_address: Self::BUS_ADDRESS,
        });

        let device_id = driver.device_id().read_async().await?.value();
        if device_id != driver.device_id().reset_value().value() {
            return Err(CapacitanceSensorError::WrongDeviceId);
        }

        // Resetting puts the chip in sleep mode, which is where it has to be
        // configured.
        driver
            .reset_dev()
            .write_async(|reset| reset.set_reset_dev(true))
            .await?;

        for channel in [Channel::Ch0, Channel::Ch1, Channel::Ch2, Channel::Ch3] {
            driver
                .rcount()
                .write_at_async(channel, |count| count.set_value(RCOUNT))
                .await?;
            driver
                .settlecount()
                .write_at_async(channel, |count| count.set_value(SETTLECOUNT))
                .await?;
            driver
                .clock_dividers()
                .write_at_async(channel, |dividers| {
                    dividers.set_fin_sel(FinSel::Div2);
                    dividers.set_fref_divider(FREF_DIVIDER);
                })
                .await?;
            driver
                .drive_current()
                .write_at_async(channel, |current| current.set_idrive(IDRIVE))
                .await?;
        }

        driver
            .error_config()
            .write_async(|config| config.set_wd_err_2_out(true))
            .await?;
        // Convert one channel at a time instead of scanning all of them. Use
        // the 10 MHz deglitch filter, the lowest setting above the tank's
        // 6.5 MHz.
        driver
            .mux_config()
            .write_async(|config| {
                config.set_autoscan_en(false);
                config.set_deglitch(ll::Deglitch::Mhz10);
            })
            .await?;

        Ok(Self {
            _shutdown: shutdown,
            driver,
        })
    }

    /// Measure the capacitance on `channel`'s electrode.
    pub async fn read(&mut self, channel: Channel) -> Result<Capacitance, CapacitanceSensorError> {
        self.set_config(channel, false).await?;
        Timer::after(CONVERSION_TIME).await;
        let data = self.read_data(channel).await;
        self.set_config(channel, true).await?;
        let data = data?;

        if data == UNDER_RANGE || data == OVER_RANGE {
            debug!("capacitance: out of range");
            return Err(CapacitanceSensorError::OutOfRange);
        }
        Ok(Capacitance {
            picofarads: picofarads(data),
        })
    }

    /// Read `channel`'s 28-bit result.
    async fn read_data(&mut self, channel: Channel) -> Result<u32, CapacitanceSensorError> {
        // Reading DATA latches DATA_LSB, so DATA has to be read first.
        let msb = self.driver.data().read_at_async(channel).await?;
        if msb.err_wd() {
            debug!("capacitance: the sensor did not oscillate");
            return Err(CapacitanceSensorError::WatchdogTimeoutError);
        }
        let lsb = self.driver.data_lsb().read_at_async(channel).await?;
        Ok((u32::from(msb.data_msb()) << 16) | u32::from(lsb.value()))
    }

    /// Select `channel`, and put the chip to sleep or wake it.
    async fn set_config(&mut self, channel: Channel, sleep: bool) -> Result<(), i2c::Error> {
        self.driver
            .config()
            .write_async(|config| {
                config.set_active_chan(channel);
                config.set_sleep_mode_en(sleep);
                config.set_reserved_12(true);
                config.set_sensor_activate_sel(SensorActivateSel::FullCurrent);
                config.set_reserved_10(true);
                config.set_ref_clk_src(RefClkSrc::InternalOscillator);
                config.set_intb_dis(true);
                config.set_reserved_5_0(0b00_0001);
            })
            .await
    }
}

/// Convert a channel's result to the capacitance on its electrode, in pF.
/// This works out the total capacitance that would make the tank oscillate
/// at the measured frequency (datasheet, equations 7 and 9), then subtracts
/// the tank's own 33 pF capacitor.
fn picofarads(data: u32) -> f32 {
    let f_ref = F_INT / f32::from(FREF_DIVIDER);
    let f_sensor = FIN_SEL * f_ref * data as f32 / DATA_SCALE;
    let omega = 2.0 * PI * f_sensor;
    1e12 / (TANK_INDUCTANCE * omega * omega) - TANK_CAPACITANCE
}
