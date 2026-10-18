//! Capacitance sensor driver (FDC2114)
//!
//! Each channel is an 18 µH inductor (L3 to L6) in parallel with 33 pF (C25
//! to C28), between INxA and INxB. The electrode on J8 to J11 connects to
//! INxA, which TI calls the single-ended sensor configuration. The chip
//! measures the frequency at which the tank oscillates, and capacitance on
//! the electrode lowers it. CLKIN is grounded, so the reference clock is the
//! chip's internal oscillator.
//!
//! The chip sleeps between readings. A reading wakes it on one channel, waits
//! for a conversion, and puts it back to sleep.

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
use ll::{Fdc2114, FinSel, OutputGain, RefClkSrc, SensorActivateSel};

mod ll {
    device_driver::compile!(
        manifest: "ddsl/fdc2114.ddsl"
    );
}

/// The inductance of every channel's tank, in H.
const TANK_INDUCTANCE: f32 = 18e-6;

/// The capacitance of every channel's tank, in pF.
const TANK_CAPACITANCE: f32 = 33.0;

/// The internal oscillator's typical frequency, in Hz. The datasheet allows
/// 35 to 55 MHz.
const F_INT: f32 = 43.4e6;

/// f_REF = f_INT / FREF_DIVIDER. While one channel converts, f_REF must not
/// exceed 35 MHz.
const FREF_DIVIDER: u16 = 2;

/// The sensor frequency is divided by 2 before it is measured, as the
/// datasheet requires for a single-ended sensor (`FinSel::Div2`).
const FIN_SEL: f32 = 2.0;

/// The length of a conversion, in units of 16 reference cycles. 16384 cycles
/// resolve the 14 bits that the output gain makes available.
const RCOUNT: u16 = 1024;

/// How long the sensor settles before a conversion, in units of 16 reference
/// cycles. The datasheet's minimum for this tank, with up to 100 pF on the
/// electrode, is 8.
const SETTLECOUNT: u16 = 64;

/// The drive current for TI's example with the same tank, which oscillated
/// at 1.68 V peak (datasheet, 8.2.3.1). The amplitude has not been measured
/// on this board.
const IDRIVE: u8 = 15;

/// The output gain, which picks bits 13:2 of the 16-bit result.
const OUTPUT_GAIN: OutputGain = OutputGain::Gain4;

/// DATA is the measured frequency over f_REF, times this.
const DATA_SCALE: f32 = (1 << 14) as f32;

/// How long the chip takes from waking to a finished conversion, at the
/// slowest internal oscillator (35 MHz): 16384 cycles of f_INT before the
/// sensor starts, then SETTLECOUNT and RCOUNT. That is 1.46 ms.
const CONVERSION_TIME: Duration = Duration::from_millis(2);

/// The time from SD going low to the chip answering on I2C.
const WAKEUP_TIME: Duration = Duration::from_millis(2);

#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum CapacitanceSensorError {
    I2cError(i2c::Error),
    /// The device at the sensor's address is not an FDC2114.
    WrongDeviceId,
    /// The sensor did not oscillate.
    WatchdogTimeoutError,
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
        }
    }
}

pub struct CapacitanceSensor<'d> {
    _shutdown: gpio::Output<'d>,
    driver: Fdc2114<SharedI2c<'d>>,
}

impl<'d> CapacitanceSensor<'d> {
    pub const BUS_ADDRESS: u8 = i2c_device::addr::CAPACITANCE;

    /// Take the chip out of shutdown, check that it is an FDC2114, and
    /// configure every channel. The chip is left asleep.
    pub async fn init(
        shutdown_pin: Peri<'d, impl gpio::Pin>,
        i2c_handle: &'d Shared<I2c<'d, Async, Master>>,
    ) -> Result<Self, CapacitanceSensorError> {
        let shutdown = gpio::Output::new(shutdown_pin, gpio::Level::Low, gpio::Speed::Low);
        Timer::after(WAKEUP_TIME).await;

        let mut driver = Fdc2114::new(SharedI2c {
            i2c_handle,
            bus_address: Self::BUS_ADDRESS,
        });

        let device_id = driver.device_id().read_async().await?.value();
        if device_id != driver.device_id().reset_value().value() {
            return Err(CapacitanceSensorError::WrongDeviceId);
        }

        // The reset leaves the chip asleep, where it has to be configured.
        driver
            .reset_dev()
            .write_async(|reset| reset.set_reset_dev(true))
            .await?;
        driver
            .reset_dev()
            .write_async(|reset| reset.set_output_gain(OUTPUT_GAIN))
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
        // One channel at a time, with the lowest deglitch filter above the
        // tank's 6.5 MHz.
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
        let data = self.driver.data().read_at_async(channel).await;
        self.set_config(channel, true).await?;
        let data = data?;

        // DATA is 0 when the sensor did not oscillate, even if the watchdog
        // did not report it.
        if data.err_wd() || data.data() == 0 {
            debug!("capacitance: the sensor did not oscillate");
            return Err(CapacitanceSensorError::WatchdogTimeoutError);
        }

        Ok(Capacitance {
            picofarads: picofarads(data.data()),
        })
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

/// The capacitance on the electrode, in pF, from a channel's DATA: the
/// capacitance that tunes the tank to the measured frequency (datasheet,
/// equations 7 and 8), less the tank's own capacitor.
fn picofarads(data: u16) -> f32 {
    let f_ref = F_INT / f32::from(FREF_DIVIDER);
    let f_sensor = FIN_SEL * f_ref * f32::from(data) / DATA_SCALE;
    let omega = 2.0 * PI * f_sensor;
    1e12 / (TANK_INDUCTANCE * omega * omega) - TANK_CAPACITANCE
}
