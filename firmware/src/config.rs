//! Persistent configuration in the last 4 KB page of the app's flash region
//! (0x0803F000, kept out of the linker's FLASH region in memory.x): the Thread
//! operational dataset and an autostart flag, so a board rejoins its network
//! by itself at power-up. CPU2's own settings do not survive its restarts.
//!
//! Writing flash while the radio stack runs follows AN5289 / ST's
//! flash_driver.c: hold hardware semaphore 2 for the whole operation, tell
//! CPU2 about erase activity, and hold semaphore 7 around each erase or
//! program step (CPU2 takes 7 to keep CPU1 out during its own flash use).

use core::ptr::read_volatile;

use embassy_stm32::flash::{Blocking, Error, Flash};
use embassy_stm32::pac::{FLASH, HSEM};
use embassy_stm32_wpan::sub::sys::Sys;

pub const PAGE_OFFSET: u32 = 0x3F000; // relative to 0x08000000
pub const PAGE_ADDR: u32 = 0x0800_0000 + PAGE_OFFSET;
const MAGIC: u32 = 0x5342_4346; // "SBCF"
const VERSION: u32 = 1;
pub const FLAG_AUTOSTART: u32 = 1;
/// Energise the relay coil while running, as a ~60 mA load that keeps USB
/// power banks from switching off.
pub const FLAG_KEEPALIVE: u32 = 2;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Config {
    magic: u32,
    version: u32,
    pub flags: u32,
    pub dataset_len: u32,
    pub dataset: [u8; 256],
}
const _: () = assert!(size_of::<Config>() % 8 == 0);

impl Config {
    pub fn new(tlvs: &[u8], flags: u32) -> Self {
        let mut c = Config {
            magic: MAGIC,
            version: VERSION,
            flags,
            dataset_len: tlvs.len().min(254) as u32,
            dataset: [0xff; 256],
        };
        c.dataset[..c.dataset_len as usize].copy_from_slice(&tlvs[..c.dataset_len as usize]);
        c
    }

    pub fn tlvs(&self) -> &[u8] {
        &self.dataset[..(self.dataset_len as usize).min(254)]
    }

    pub fn autostart(&self) -> bool {
        self.flags & FLAG_AUTOSTART != 0
    }

    pub fn keepalive(&self) -> bool {
        self.flags & FLAG_KEEPALIVE != 0
    }

    /// Mesh-Local Prefix TLV (type 7) from the dataset.
    pub fn mesh_local_prefix(&self) -> Option<[u8; 8]> {
        let t = self.tlvs();
        let mut i = 0;
        while i + 2 <= t.len() {
            let (ty, len) = (t[i], t[i + 1] as usize);
            if ty == 7 && len == 8 && i + 2 + 8 <= t.len() {
                let mut p = [0u8; 8];
                p.copy_from_slice(&t[i + 2..i + 10]);
                return Some(p);
            }
            i += 2 + len;
        }
        None
    }
}

pub fn load() -> Option<Config> {
    let c = unsafe { read_volatile(PAGE_ADDR as *const Config) };
    (c.magic == MAGIC && c.version == VERSION && c.dataset_len <= 254).then_some(c)
}

const SEM_FLASH: usize = 2;
const SEM_BLOCK_BY_CPU2: usize = 7;
const CPU1: u8 = 4;

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

fn guarded<T>(f: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
    if !sem_lock(SEM_BLOCK_BY_CPU2) {
        return Err(Error::Unaligned); // no better variant; means "CPU2 holds the flash"
    }
    while FLASH.sr().read().pesd() {}
    let r = f();
    sem_release(SEM_BLOCK_BY_CPU2);
    r
}

async fn with_flash_access<T>(sys: &mut Sys<'_>, f: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
    if !sem_lock(SEM_FLASH) {
        return Err(Error::Unaligned);
    }
    let _ = sys.shci_c2_flash_erase_activity(true).await;
    let r = f();
    let _ = sys.shci_c2_flash_erase_activity(false).await;
    sem_release(SEM_FLASH);
    r
}

pub async fn save(flash: &mut Flash<'_, Blocking>, sys: &mut Sys<'_>, cfg: &Config) -> Result<(), Error> {
    let bytes = unsafe { core::slice::from_raw_parts(cfg as *const Config as *const u8, size_of::<Config>()) };
    with_flash_access(sys, || {
        guarded(|| flash.blocking_erase(PAGE_OFFSET, PAGE_OFFSET + 4096))?;
        guarded(|| flash.blocking_write(PAGE_OFFSET, bytes))
    })
    .await
}

pub async fn erase(flash: &mut Flash<'_, Blocking>, sys: &mut Sys<'_>) -> Result<(), Error> {
    with_flash_access(sys, || guarded(|| flash.blocking_erase(PAGE_OFFSET, PAGE_OFFSET + 4096))).await
}
