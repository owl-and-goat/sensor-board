//! SAI1's PDM interface (RM0434 39.4.10), for one PDM microphone.
//!
//! The PDM interface takes the clock of sub-block A, which runs as a master
//! receiver in TDM, and hands sub-block A the bitstreams of up to two pairs
//! of microphones (data lines D1 and D2 on this chip) de-interleaved into
//! bytes: a frame has an 8-bit slot for each microphone, the left one of a
//! pair first, and the bitstream clock is SCK_A / (2 * pairs). Only the slot
//! of the microphone asked for is captured, so every byte DMA delivers is
//! eight of its samples, the first in the MSB.

use embassy_stm32::{
    Peri,
    dma::{self, ReadableRingBuffer, TransferOptions},
    gpio::{self, AfType, Flex, OutputType, Pull, Speed},
    interrupt,
    pac::{self, sai::vals},
    peripherals::{PA3, PA8, PA9, PA10, PB8, PB9, SAI1},
    rcc,
    sai::{self, Dma},
};

/// The bitstream clock: 16 kHz decimated by 128.
pub const CLOCK_HZ: u32 = 2_048_000;

/// The PDM signals are alternate function 3 wherever they are.
const AF: u8 = 3;

mod sealed {
    pub trait Sealed {}
}

/// A pin that can carry a bitstream clock, SAI1_CK1 or SAI1_CK2.
pub trait ClockPin: gpio::Pin + sealed::Sealed {
    /// Which of the `CKEN` bits enables it.
    const CKEN: usize;
}

/// A pin that can carry the data of a pair of microphones, SAI1_D1 or
/// SAI1_D2.
pub trait DataPin: gpio::Pin + sealed::Sealed {
    /// Which pair's data it carries, from 0.
    const PAIR: u8;
}

macro_rules! clock_pins {
    ($($pin:ident: $cken:literal),+) => {$(
        impl sealed::Sealed for $pin {}
        impl ClockPin for $pin {
            const CKEN: usize = $cken;
        }
    )+};
}

macro_rules! data_pins {
    ($($pin:ident: $pair:literal),+) => {$(
        impl sealed::Sealed for $pin {}
        impl DataPin for $pin {
            const PAIR: u8 = $pair;
        }
    )+};
}

clock_pins!(PA3: 0, PB8: 0, PA8: 1);
data_pins!(PA10: 0, PA9: 1, PB9: 1);

/// Which microphone of the pair on a data line: the one whose data is valid
/// on the rising edge of the clock, or the falling one. The microphone's L/R
/// pin chooses (RM0434 figure 396).
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum Channel {
    /// L/R tied to VDD.
    #[expect(dead_code, reason = "no board has its microphone strapped left")]
    Left,
    /// L/R tied to GND, or floating if the microphone pulls it down.
    Right,
}

/// DMA overwrote part of the bitstream before it was read.
#[derive(Debug, defmt::Format)]
pub struct Overrun;

pub struct Pdm<'d> {
    _sai: Peri<'d, SAI1>,
    _clock: Flex<'d>,
    _data: Flex<'d>,
    ring: ReadableRingBuffer<'d, u8>,
    mckdiv: vals::Mckdiv,
    /// MICNBR: the pairs of microphones, less one.
    micnbr: u8,
    cken: usize,
    slot: usize,
    running: bool,
}

impl<'d> Pdm<'d> {
    /// Sets the pins and DMA up, and leaves the interface off: `start` turns
    /// it on.
    ///
    /// Panics unless SAI1's kernel clock divides down to `CLOCK_HZ`.
    pub fn new<C: ClockPin, D: DataPin, Ch: Dma<SAI1, sai::A>>(
        sai: Peri<'d, SAI1>,
        clock: Peri<'d, C>,
        data: Peri<'d, D>,
        channel: Channel,
        dma: Peri<'d, Ch>,
        irq: impl interrupt::typelevel::Binding<Ch::Interrupt, dma::InterruptHandler<Ch>> + 'd,
        buf: &'d mut [u8],
    ) -> Self {
        let micnbr = D::PAIR;
        let sck_hz = CLOCK_HZ * 2 * (micnbr as u32 + 1);
        let kernel_hz = rcc::frequency::<SAI1>().0;
        let mckdiv = kernel_hz / sck_hz;
        assert!(
            kernel_hz % sck_hz == 0 && (1..64).contains(&mckdiv),
            "SAI1's kernel clock ({} Hz) does not divide down to {} Hz",
            kernel_hz,
            sck_hz
        );

        let mut clock = Flex::new(clock);
        clock.set_as_af_unchecked(AF, AfType::output(OutputType::PushPull, Speed::High));
        let mut data = Flex::new(data);
        data.set_as_af_unchecked(AF, AfType::input(Pull::None));

        let request = dma.request();
        // SAFETY: the data register of block A, which this owns through
        // `sai`, read in bytes as `DS` makes them.
        let ring = unsafe {
            ReadableRingBuffer::new(
                dma::Channel::new(dma, irq),
                request,
                pac::SAI1.ch(0).dr().as_ptr() as *mut u8,
                buf,
                TransferOptions::default(),
            )
        };

        Self {
            _sai: sai,
            _clock: clock,
            _data: data,
            ring,
            mckdiv: vals::Mckdiv::from_bits(mckdiv as u8),
            micnbr,
            cken: C::CKEN,
            slot: 2 * micnbr as usize + channel as usize,
            running: false,
        }
    }

    /// Clocks the microphone and starts taking its bitstream in. The
    /// microphone takes up to 10 ms to give valid data.
    pub fn start(&mut self) {
        rcc::enable_and_reset::<SAI1>();

        let sai = pac::SAI1;
        let a = sai.ch(0);
        let slots = 2 * (self.micnbr + 1);
        // RM0434 table 246, for 8-bit slots.
        a.cr1().modify(|w| {
            w.set_mode(vals::Mode::MASTER_RX);
            w.set_ds(vals::Ds::BIT8);
            w.set_nodiv(true);
            w.set_mckdiv(self.mckdiv);
            w.set_dmaen(true);
        });
        a.frcr().modify(|w| {
            w.set_frl(8 * slots - 1);
            // The frame sync is internal, but the PDM interface goes by it:
            // at reset (0) every byte is scrambled.
            w.set_fspol(vals::Fspol::RISING_EDGE);
        });
        a.slotr().modify(|w| {
            w.set_nbslot(slots - 1);
            w.set_sloten(vals::Sloten::from_bits(1 << self.slot));
        });
        sai.pdmcr().modify(|w| {
            w.set_micnbr(self.micnbr);
            w.set_cken(self.cken, true);
            w.set_pdmen(true);
        });

        self.ring.clear();
        self.ring.start();
        a.cr1().modify(|w| w.set_saien(true));
        self.running = true;
    }

    /// Stops the clock, which puts the microphone in power-down, and turns
    /// SAI1 off.
    pub fn stop(&mut self) {
        let a = pac::SAI1.ch(0);
        a.cr1().modify(|w| w.set_saien(false));
        // SAIEN reads back as set until the frame under way has ended.
        while a.cr1().read().saien() {}
        pac::SAI1.pdmcr().modify(|w| w.set_pdmen(false));

        self.ring.request_pause();
        while self.ring.is_running() {}
        rcc::disable::<SAI1>();
        self.running = false;
    }

    /// Fills `buf` with the bitstream that follows what was last read, eight
    /// samples to a byte, the first in the MSB. After an overrun the next
    /// read starts from what arrives then.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<(), Overrun> {
        if self.ring.read_exact(buf).await.is_err() {
            self.ring.clear();
            return Err(Overrun);
        }
        Ok(())
    }
}

impl Drop for Pdm<'_> {
    fn drop(&mut self) {
        if self.running {
            self.stop();
        }
    }
}
