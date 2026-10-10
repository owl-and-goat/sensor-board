//! Persistent configuration in the 4 KB page after the firmware's own flash
//! region (0x0803F000, outside the linker's FLASH region in memory.x and
//! outside what a firmware update swaps). The page holds two records, each at
//! a fixed offset:
//!
//! - the Thread operational dataset, so that a board rejoins its network at
//!   power-up. CPU2 does not keep its own settings across restarts.
//! - the board's configuration ([`BoardConfig`]).
//!
//! Flash is erased a page at a time, so changing either record rewrites
//! both. A reset during the rewrite loses both.

use core::ptr::read_volatile;

use embassy_stm32::flash::Error;
use protocol::{BoardConfig, Dataset};

use crate::radio_flash::RadioFlash;

pub const PAGE_OFFSET: u32 = 0x3F000; // relative to 0x08000000
pub const PAGE_ADDR: u32 = 0x0800_0000 + PAGE_OFFSET;
const MAGIC: u32 = 0x5342_4346; // "SBCF"
const VERSION: u32 = 1;

/// The dataset record, at the start of the page. Its layout must not change:
/// firmware from before the board configuration was added reads only this
/// record, and has to keep finding its dataset here.
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

    /// Load the record from flash, if a valid one is there.
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

/// Offset of the board configuration record in the page. It leaves spare
/// room after the dataset record.
const BOARD_OFFSET: u32 = 0x200;
const _: () = assert!(size_of::<DatasetRecord>() <= BOARD_OFFSET as usize);
const BOARD_MAGIC: u32 = 0x5342_4243; // "SBBC"

/// The longest encoded configuration, rounded up to whole 8-byte flash words.
const BOARD_LEN: usize = BoardConfig::MAX_LEN.next_multiple_of(8);

/// The board configuration record. The first `len` bytes of `config` are the
/// output of [`BoardConfig::encode`].
#[repr(C)]
#[derive(Clone, Copy)]
struct BoardRecord {
    magic: u32,
    len: u32,
    config: [u8; BOARD_LEN],
}
const _: () = assert!(size_of::<BoardRecord>() % 8 == 0);

impl BoardRecord {
    /// `None` if the encoded configuration does not fit in the record.
    fn new(config: &BoardConfig) -> Option<Self> {
        let mut record = BoardRecord {
            magic: BOARD_MAGIC,
            len: 0,
            config: [0xff; BOARD_LEN],
        };
        record.len = config.encode(&mut record.config)?.len() as u32;
        Some(record)
    }

    /// Load the record from flash, if a valid one is there. This firmware
    /// may not be able to decode it, if a firmware with a different
    /// `BoardConfig` layout wrote it.
    fn load() -> Option<Self> {
        let record = unsafe { read_volatile((PAGE_ADDR + BOARD_OFFSET) as *const BoardRecord) };
        let valid = record.magic == BOARD_MAGIC && record.len as usize <= BOARD_LEN;
        valid.then_some(record)
    }

    fn config(&self) -> Option<BoardConfig> {
        BoardConfig::decode(&self.config[..self.len as usize])
    }
}

/// The stored dataset: the network the board joins at power-up.
pub fn dataset() -> Option<Dataset> {
    DatasetRecord::load()?.dataset()
}

/// The stored board configuration. `None` if there is none, or if this
/// firmware cannot decode it.
pub fn board_config() -> Option<BoardConfig> {
    BoardRecord::load()?.config()
}

/// Store `dataset`, or with `None` remove the stored one. The board
/// configuration record is preserved, even if this firmware cannot decode it.
pub async fn set_dataset(
    flash: &mut RadioFlash<'_>,
    dataset: Option<&Dataset>,
) -> Result<(), Error> {
    let dataset = dataset.map(DatasetRecord::new);
    write(flash, dataset.as_ref(), BoardRecord::load().as_ref()).await
}

/// Store `config`. The dataset record is preserved.
pub async fn set_board_config(
    flash: &mut RadioFlash<'_>,
    config: &BoardConfig,
) -> Result<(), Error> {
    // Unreachable, as the record has room for the longest configuration.
    // `Error::Size` is the closest variant.
    let config = BoardRecord::new(config).ok_or(Error::Size)?;
    write(flash, DatasetRecord::load().as_ref(), Some(&config)).await
}

/// Erase the page and write the given records to it.
async fn write(
    flash: &mut RadioFlash<'_>,
    dataset: Option<&DatasetRecord>,
    board: Option<&BoardRecord>,
) -> Result<(), Error> {
    // SAFETY: both records are `repr(C)` structs of `u32`s and byte arrays,
    // with no padding.
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

/// The bytes of a record, as written to flash.
///
/// SAFETY: `T` must have no padding bytes.
unsafe fn bytes_of<T>(record: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts((record as *const T).cast::<u8>(), size_of::<T>()) }
}
