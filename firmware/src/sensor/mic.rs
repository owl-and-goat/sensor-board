//! MEMS microphone (MP34DT05): the sound level, measured on request.
//!
//! The microphone is on SAI1's PDM interface (`pdm`): SAI1_CK1 (PA3) clocks
//! it and its data comes in on SAI1_D1 (PA10). Its L/R pin is left floating,
//! which its pull-down makes GND: the right microphone of the pair. The
//! interface is on only while a measurement lasts; with no clock the
//! microphone is in power-down.
//!
//! SAI1's kernel clock is PLLSAI1 at 8.192 MHz (`configure_clocks` in
//! `main.rs`), which divides down to a bitstream clock of 2.048 MHz (1.2 to
//! 3.25 MHz in the microphone's datasheet). `filter` decimates that by 128,
//! to 16 kHz.

use embassy_time::{Duration, Instant};
use protocol::{SensorReadError, SoundLevel, SoundLevelResult};

mod filter;
pub mod pdm;

use filter::Filter;
use pdm::Pdm;

/// PCM samples per second.
const RATE: u64 = 16_000;
/// What is left out at the start of a measurement, and after an overrun:
/// the microphone's turn-on time (10 ms) and the high-pass filter's settling.
const SETTLE: Duration = Duration::from_millis(100);
/// PCM samples worked out at a time.
const CHUNK: usize = 16;

/// The PCM's RMS at 94 dB SPL. The microphone's sensitivity is -26 dBFS
/// (±3 dB) there, and the PCM's full scale is 12 dB below the bitstream's.
const RMS_AT_94_DB: f32 = 4620.0;

pub struct Mic<'d> {
    pdm: Pdm<'d>,
    filter: Filter,
}

impl<'d> Mic<'d> {
    pub fn new(pdm: Pdm<'d>) -> Self {
        Self {
            pdm,
            filter: Filter::new(),
        }
    }

    /// Turns the microphone on, listens for `duration`, and turns it off.
    /// The level is the RMS of the PCM, in dB SPL. A measurement that takes
    /// twice as long as it should has fallen behind the microphone, and is
    /// given up.
    pub async fn measure(&mut self, duration: Duration) -> SoundLevelResult {
        let samples = RATE * duration.as_micros() / 1_000_000;
        let settle = RATE * SETTLE.as_micros() / 1_000_000;

        let mut pdm = [0u8; CHUNK * filter::BYTES_PER_SAMPLE];
        let mut pcm = [0i16; CHUNK];
        let (mut sum, mut n) = (0u64, 0u64);
        let mut skip = settle;
        let mut overruns = 0u32;

        let deadline = Instant::now() + (SETTLE + duration) * 2;
        self.filter.reset();
        self.pdm.start();
        while n < samples && Instant::now() < deadline {
            if self.pdm.read(&mut pdm).await.is_err() {
                overruns += 1;
                self.filter.reset();
                skip = settle;
                continue;
            }
            self.filter.process(&pdm, &mut pcm);
            for &x in &pcm {
                if skip > 0 {
                    skip -= 1;
                } else {
                    sum += (x as i32 * x as i32) as u64;
                    n += 1;
                }
            }
        }
        self.pdm.stop();

        if overruns > 0 {
            defmt::warn!("mic: {} overruns during a measurement", overruns);
        }
        if n < samples {
            return Err(SensorReadError::MicOverrun);
        }
        let mean_square = (sum / n).max(1) as f32;
        Ok(SoundLevel {
            leq: 94.0 + 10.0 * libm::log10f(mean_square / (RMS_AT_94_DB * RMS_AT_94_DB)),
        })
    }
}
