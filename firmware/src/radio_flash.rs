//! CPU1's flash, written the way it has to be while a radio stack runs on
//! CPU2. Per AN5289 and ST's flash_driver.c: hold hardware semaphore 2 for
//! the whole operation, tell CPU2 about erase activity, and hold semaphore 7
//! around each erase or program step (CPU2 takes 7 to keep CPU1 out during
//! its own flash use).
//!
//! Telling CPU2 goes over the system channel, so that is owned here as well.

use embassy_stm32::flash::{Blocking, Error, FLASH_SIZE, Flash, WRITE_SIZE};
use embassy_stm32::pac::{FLASH, HSEM};
use embassy_stm32_wpan::sub::sys::Sys;
use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

/// The size of a flash page, which is what is erased at a time.
pub const PAGE: u32 = 4096;

const SEM_FLASH: usize = 2;
const SEM_BLOCK_BY_CPU2: usize = 7;
const CPU1: u8 = 4;

pub struct RadioFlash<'d> {
    flash: Flash<'d, Blocking>,
    sys: Sys<'d>,
}

impl<'d> RadioFlash<'d> {
    pub fn new(flash: Flash<'d, Blocking>, sys: Sys<'d>) -> Self {
        RadioFlash { flash, sys }
    }

    /// The system channel to CPU2, for the commands that are not about
    /// flash.
    pub fn sys(&mut self) -> &mut Sys<'d> {
        &mut self.sys
    }

    /// Erase the page that starts at `page`, and write each of `records` at
    /// its offset into flash. Nothing else on CPU1 runs in between, so
    /// nothing reads the page with only some of that done.
    pub async fn rewrite_page<'a>(
        &mut self,
        page: u32,
        records: impl IntoIterator<Item = (u32, &'a [u8])>,
    ) -> Result<(), Error> {
        self.coordinated(|flash| {
            guarded(|| flash.blocking_erase(page, page + PAGE))?;
            for (offset, bytes) in records {
                guarded(|| flash.blocking_write(offset, bytes))?;
            }
            Ok(())
        })
        .await
    }

    /// Run `steps`, each of which is to be [`guarded`], with CPU2 told that
    /// flash is being written.
    async fn coordinated<T>(
        &mut self,
        steps: impl FnOnce(&mut Flash<'d, Blocking>) -> Result<T, Error>,
    ) -> Result<T, Error> {
        if !sem_lock(SEM_FLASH) {
            return Err(Error::Unaligned);
        }
        let _ = self.sys.shci_c2_flash_erase_activity(true).await;
        let result = steps(&mut self.flash);
        let _ = self.sys.shci_c2_flash_erase_activity(false).await;
        sem_release(SEM_FLASH);
        result
    }
}

fn sem_lock(i: usize) -> bool {
    for _ in 0..2_000_000u32 {
        let r = HSEM.rlr(i).read();
        if r.lock() && r.coreid() == CPU1 {
            return true;
        }
    }
    false
}

fn sem_release(i: usize) {
    HSEM.r(i).write(|w| {
        w.set_lock(false);
        w.set_coreid(CPU1);
        w.set_procid(0);
    });
}

/// One erase or program step, with CPU2 kept from using flash meanwhile.
fn guarded<T>(step: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
    if !sem_lock(SEM_BLOCK_BY_CPU2) {
        return Err(Error::Unaligned); // no better variant; means "CPU2 holds the flash"
    }
    while FLASH.sr().read().pesd() {}
    let result = step();
    sem_release(SEM_BLOCK_BY_CPU2);
    result
}

impl ErrorType for RadioFlash<'_> {
    type Error = Error;
}

/// Offsets are into flash, from 0x08000000.
impl ReadNorFlash for RadioFlash<'_> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.flash.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl NorFlash for RadioFlash<'_> {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = PAGE as usize;

    /// Each page takes some 20 ms, in which nothing else on CPU1 runs.
    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.coordinated(|flash| {
            for page in (from..to).step_by(PAGE as usize) {
                guarded(|| flash.blocking_erase(page, page + PAGE))?;
            }
            Ok(())
        })
        .await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.coordinated(|flash| guarded(|| flash.blocking_write(offset, bytes)))
            .await
    }
}

/// The flash as the services inside the coprocessor task share it.
pub type Shared<'d> =
    embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::NoopRawMutex, RadioFlash<'d>>;
