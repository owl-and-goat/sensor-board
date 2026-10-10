//! PDM to PCM: ST's OpenPDMFilter (V1.0.0, Apache-2.0), as Zephyr carries it
//! in `hal_st/audio/microphone`, for one microphone decimated by 128 and the
//! parameters Zephyr's MPxxDTyy driver gives it, but for the gain.
//!
//! The decimator is a third-order sinc filter (a boxcar of 128 convolved
//! with itself twice), worked out four samples of the bitstream at a time
//! from a look-up table built here. ST's goes a byte at a time, with a table
//! 16 times the size (48 KiB); the sums are the same. After it come a first-order high-pass (10 Hz)
//! and a first-order low-pass (Fs / 2), then the gain.

/// PDM samples per PCM sample.
const DECIMATION: usize = 128;
/// Bytes of the bitstream per PCM sample.
pub const BYTES_PER_SAMPLE: usize = DECIMATION / 8;
/// Nibbles of the bitstream per PCM sample.
const NIBBLES: usize = DECIMATION / 4;
const SINCN: usize = 3;

const FS: f32 = 16_000.0;
const LP_HZ: f32 = FS / 2.0;
const HP_HZ: f32 = 10.0;
const MAX_VOLUME: i64 = 64;
/// The gain against full scale is `VOLUME * FILTER_GAIN / MAX_VOLUME`: 4 at
/// 16, so the PCM clips 12 dB below the microphone's overload point (122.5 dB
/// SPL), at about 110 dB SPL. Zephyr passes 64, which clips from about 98 dB
/// SPL: shouting at the board does it.
const VOLUME: i64 = 16;
const FILTER_GAIN: i64 = 16;

/// What the sinc filter's coefficients add up to: those of a boxcar,
/// `DECIMATION`, to the power of its order.
const SINC_SUM: i64 = (DECIMATION as i64).pow(SINCN as u32);
/// Half of that, which centres the sinc filter's output on zero.
const SUB_CONST: i64 = SINC_SUM >> 1;
/// Constant so that the division by it, once for each sample, is shifts
/// rather than a call to the 64-bit division in software.
const DIV_CONST: i64 = max(SUB_CONST * MAX_VOLUME / 32768 / FILTER_GAIN, 1);

pub struct Filter {
    /// The sinc filter's share of each nibble value at each nibble position,
    /// for each of its three parts. By position first, so that the two
    /// nibbles of a byte are next to each other.
    lut: [[[u32; SINCN]; 16]; NIBBLES],
    lp_alfa: i64,
    hp_alfa: i64,
    coef: [i64; 2],
    old_out: i64,
    old_in: i64,
    old_z: i64,
}

impl Filter {
    pub fn new() -> Self {
        let boxcar = [1u32; DECIMATION];
        let mut sinc2 = [0u32; 2 * DECIMATION - 1];
        convolve(&boxcar, &boxcar, &mut sinc2);
        let mut sinc = [0u32; SINCN * DECIMATION];
        convolve(&sinc2, &boxcar, &mut sinc[1..SINCN * DECIMATION - 1]);

        let mut lut = [[[0u32; SINCN]; 16]; NIBBLES];
        for (p, by_value) in lut.iter_mut().enumerate() {
            for (n, by_part) in by_value.iter_mut().enumerate() {
                for (s, entry) in by_part.iter_mut().enumerate() {
                    let coef = &sinc[s * DECIMATION + p * 4..][..4];
                    *entry = (0..4)
                        .map(|bit| ((n >> (3 - bit)) & 1) as u32 * coef[bit])
                        .sum();
                }
            }
        }

        debug_assert_eq!(sinc.iter().map(|&c| c as i64).sum::<i64>(), SINC_SUM);
        Self {
            lut,
            lp_alfa: (LP_HZ * 256.0 / (LP_HZ + FS / (2.0 * 3.14159))) as i64,
            hp_alfa: (FS * 256.0 / (2.0 * 3.14159 * HP_HZ + FS)) as i64,
            coef: [0; 2],
            old_out: 0,
            old_in: 0,
            old_z: 0,
        }
    }

    /// Clears the state, for a bitstream that does not follow the last one.
    pub fn reset(&mut self) {
        self.coef = [0; 2];
        (self.old_out, self.old_in, self.old_z) = (0, 0, 0);
    }

    /// Turns `pdm`, the first sample in the MSB of each byte, into `pcm`:
    /// `BYTES_PER_SAMPLE` bytes for each sample. The state carries over to
    /// the next call, which should take the bitstream up where this one left
    /// it.
    pub fn process(&mut self, pdm: &[u8], pcm: &mut [i16]) {
        assert_eq!(pdm.len(), pcm.len() * BYTES_PER_SAMPLE);
        for (bytes, out) in pdm.chunks_exact(BYTES_PER_SAMPLE).zip(pcm) {
            // The three parts of the sinc filter, in one pass over the bytes.
            let (mut z0, mut z1, mut z2) = (0u32, 0u32, 0u32);
            for (&byte, by_value) in bytes.iter().zip(self.lut.chunks_exact(2)) {
                let [h0, h1, h2] = by_value[0][usize::from(byte >> 4)];
                let [l0, l1, l2] = by_value[1][usize::from(byte & 0xf)];
                (z0, z1, z2) = (z0 + h0 + l0, z1 + h1 + l1, z2 + h2 + l2);
            }
            let (z0, z1, z2) = (i64::from(z0), i64::from(z1), i64::from(z2));

            let z = self.coef[1] + z2 - SUB_CONST;
            self.coef[1] = self.coef[0] + z1;
            self.coef[0] = z0;

            self.old_out = (self.hp_alfa * (self.old_out + z - self.old_in)) >> 8;
            self.old_in = z;
            self.old_z = ((256 - self.lp_alfa) * self.old_z + self.lp_alfa * self.old_out) >> 8;

            let z = round_div(self.old_z * VOLUME, DIV_CONST);
            *out = z.clamp(-32700, 32700) as i16;
        }
    }
}

/// `result` is `signal` convolved with `kernel`, and as long as that is.
fn convolve(signal: &[u32], kernel: &[u32], result: &mut [u32]) {
    for (n, r) in result.iter_mut().enumerate() {
        let kmin = n.saturating_sub(kernel.len() - 1);
        let kmax = n.min(signal.len() - 1);
        *r = (kmin..=kmax).map(|k| signal[k] * kernel[n - k]).sum();
    }
}

/// Division rounded to the nearest, halves away from zero.
const fn max(a: i64, b: i64) -> i64 {
    if a > b { a } else { b }
}

#[inline(always)]
fn round_div(a: i64, b: i64) -> i64 {
    if a > 0 {
        (a + b / 2) / b
    } else {
        (a - b / 2) / b
    }
}
