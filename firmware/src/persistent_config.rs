//! Persistent configuration in the 4 KB page after the firmware's own flash
//! region (0x0803F000, outside the linker's FLASH region in memory.x and
//! outside what a firmware update swaps): the Thread operational dataset, so
//! a board rejoins its network by itself at power-up. CPU2's own settings do
//! not survive its restarts.

use core::ptr::read_volatile;

use embassy_stm32::flash::Error;
use embedded_storage_async::nor_flash::NorFlash;

use crate::radio_flash::{PAGE, RadioFlash};

pub const PAGE_OFFSET: u32 = 0x3F000; // relative to 0x08000000
pub const PAGE_ADDR: u32 = 0x0800_0000 + PAGE_OFFSET;
const MAGIC: u32 = 0x5342_4346; // "SBCF"
const VERSION: u32 = 1;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Config {
    magic: u32,
    version: u32,
    /// Flags, in the bring-up firmware. Nothing reads them now; the word
    /// stays so that a config written back then still loads.
    _reserved: u32,
    pub dataset_len: u32,
    pub dataset: [u8; 256],
}
const _: () = assert!(size_of::<Config>() % 8 == 0);

impl Config {
    pub fn new(tlvs: &[u8]) -> Self {
        let mut c = Config {
            magic: MAGIC,
            version: VERSION,
            _reserved: 0,
            dataset_len: tlvs.len().min(254) as u32,
            dataset: [0xff; 256],
        };
        c.dataset[..c.dataset_len as usize].copy_from_slice(&tlvs[..c.dataset_len as usize]);
        c
    }

    pub fn tlvs(&self) -> &[u8] {
        &self.dataset[..(self.dataset_len as usize).min(254)]
    }
}

pub fn load() -> Option<Config> {
    let c = unsafe { read_volatile(PAGE_ADDR as *const Config) };
    (c.magic == MAGIC && c.version == VERSION && c.dataset_len <= 254).then_some(c)
}

pub async fn save(flash: &mut RadioFlash<'_>, cfg: &Config) -> Result<(), Error> {
    let bytes = unsafe {
        core::slice::from_raw_parts(cfg as *const Config as *const u8, size_of::<Config>())
    };
    flash.erase(PAGE_OFFSET, PAGE_OFFSET + PAGE).await?;
    flash.write(PAGE_OFFSET, bytes).await
}

pub async fn erase(flash: &mut RadioFlash<'_>) -> Result<(), Error> {
    flash.erase(PAGE_OFFSET, PAGE_OFFSET + PAGE).await
}
