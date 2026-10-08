//! Persistent configuration in the 4 KB page after the firmware's own flash
//! region (0x0803F000, outside the linker's FLASH region in memory.x and
//! outside what a firmware update swaps). Two things are kept there, each in
//! a record at a place of its own:
//!
//! - the Thread operational dataset, so a board rejoins its network by
//!   itself at power-up. CPU2's own settings do not survive its restarts.
//! - the board's configuration ([`BoardConfig`]).
//!
//! A page is erased as a whole, so a change to either writes both again. A
//! reset in the middle of that loses both.

use core::ptr::read_volatile;

use embassy_stm32::flash::Error;
use protocol::{BoardConfig, Dataset};

use crate::radio_flash::RadioFlash;

pub const PAGE_OFFSET: u32 = 0x3F000; // relative to 0x08000000
pub const PAGE_ADDR: u32 = 0x0800_0000 + PAGE_OFFSET;
const MAGIC: u32 = 0x5342_4346; // "SBCF"
const VERSION: u32 = 1;

/// The dataset, at the start of the page. This is all that the firmwares
/// from before the board's configuration was kept here know of the page:
/// one of them still finds its dataset in it.
#[repr(C)]
#[derive(Clone, Copy)]
struct DatasetRecord {
    magic: u32,
    version: u32,
    /// Flags, in the bring-up firmware. Nothing reads them now; the word
    /// stays so that a record written back then still loads.
    _reserved: u32,
    dataset_len: u32,
    dataset: [u8; 256],
}
const _: () = assert!(size_of::<DatasetRecord>() % 8 == 0);

impl DatasetRecord {
    fn new(dataset: &Dataset) -> Self {
        let tlvs = dataset.as_tlvs();
        let mut record = DatasetRecord {
            magic: MAGIC,
            version: VERSION,
            _reserved: 0,
            dataset_len: tlvs.len() as u32,
            dataset: [0xff; 256],
        };
        record.dataset[..tlvs.len()].copy_from_slice(tlvs);
        record
    }

    /// The record in flash, if there is one.
    fn load() -> Option<Self> {
        let record = unsafe { read_volatile(PAGE_ADDR as *const DatasetRecord) };
        let valid = record.magic == MAGIC
            && record.version == VERSION
            && record.dataset_len as usize <= Dataset::MAX_LEN;
        valid.then_some(record)
    }

    fn dataset(&self) -> Option<Dataset> {
        Dataset::from_tlvs(&self.dataset[..self.dataset_len as usize])
    }
}

/// Where in the page the board's configuration is: behind the dataset, with
/// room to spare.
const BOARD_OFFSET: u32 = 0x200;
const _: () = assert!(size_of::<DatasetRecord>() <= BOARD_OFFSET as usize);
const BOARD_MAGIC: u32 = 0x5342_4243; // "SBBC"

/// A whole number of flash words that the longest configuration fits in.
const BOARD_LEN: usize = BoardConfig::MAX_LEN.next_multiple_of(8);

/// The board's configuration, as [`BoardConfig::encode`] writes it.
#[repr(C)]
#[derive(Clone, Copy)]
struct BoardRecord {
    magic: u32,
    len: u32,
    config: [u8; BOARD_LEN],
}
const _: () = assert!(size_of::<BoardRecord>() % 8 == 0);

impl BoardRecord {
    /// `None` if the configuration is longer than one can be.
    fn new(config: &BoardConfig) -> Option<Self> {
        let mut record = BoardRecord {
            magic: BOARD_MAGIC,
            len: 0,
            config: [0xff; BOARD_LEN],
        };
        record.len = config.encode(&mut record.config)?.len() as u32;
        Some(record)
    }

    /// The record in flash, if there is one. It may hold a configuration
    /// that this firmware does not understand: one that a firmware with
    /// another layout of it has kept.
    fn load() -> Option<Self> {
        let record = unsafe { read_volatile((PAGE_ADDR + BOARD_OFFSET) as *const BoardRecord) };
        let valid = record.magic == BOARD_MAGIC && record.len as usize <= BOARD_LEN;
        valid.then_some(record)
    }

    fn config(&self) -> Option<BoardConfig> {
        BoardConfig::decode(&self.config[..self.len as usize])
    }
}

/// The dataset of the network the board is to be on.
pub fn dataset() -> Option<Dataset> {
    DatasetRecord::load()?.dataset()
}

/// The board's configuration, if it has been given one, and this firmware
/// understands it.
pub fn board_config() -> Option<BoardConfig> {
    BoardRecord::load()?.config()
}

/// Keep `dataset`, or with `None` keep none. The board's configuration stays
/// as it is, understood or not.
pub async fn set_dataset(
    flash: &mut RadioFlash<'_>,
    dataset: Option<&Dataset>,
) -> Result<(), Error> {
    let dataset = dataset.map(DatasetRecord::new);
    write(flash, dataset.as_ref(), BoardRecord::load().as_ref()).await
}

/// Keep `config`. The dataset stays as it is.
pub async fn set_board_config(
    flash: &mut RadioFlash<'_>,
    config: &BoardConfig,
) -> Result<(), Error> {
    // No better variant, and it does not come to that: the record has room
    // for the longest configuration.
    let config = BoardRecord::new(config).ok_or(Error::Size)?;
    write(flash, DatasetRecord::load().as_ref(), Some(&config)).await
}

/// Write the page anew, with the records that there are to be.
async fn write(
    flash: &mut RadioFlash<'_>,
    dataset: Option<&DatasetRecord>,
    board: Option<&BoardRecord>,
) -> Result<(), Error> {
    // SAFETY: both records are `repr(C)`, and of words and bytes that leave
    // no padding between them.
    let records = unsafe {
        [
            dataset.map(|record| (PAGE_OFFSET, bytes_of(record))),
            board.map(|record| (PAGE_OFFSET + BOARD_OFFSET, bytes_of(record))),
        ]
    };
    flash
        .rewrite_page(PAGE_OFFSET, records.into_iter().flatten())
        .await
}

/// A record as the bytes that go into flash.
///
/// SAFETY: every byte of `T` has to be one of its fields: no padding.
unsafe fn bytes_of<T>(record: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts((record as *const T).cast::<u8>(), size_of::<T>()) }
}
