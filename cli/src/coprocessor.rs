//! Installing firmware on a board's radio coprocessor.

use std::{
    io::Write,
    time::{Duration, Instant},
};

use anyhow::{Result, bail};
use protocol::{CoprocessorFirmware, CoprocessorStatus, FusState, ImageChunk, ImageSize, Version};

use crate::board::{self, Board};

/// How long FUS gets to install or remove an image, resets and all. A
/// wireless stack, the largest, takes it well under a minute.
const INSTALL_TIMEOUT: Duration = Duration::from_secs(180);
const POLL_INTERVAL: Duration = Duration::from_secs(1);

/// How long a board that has just been flashed or reset gets to show up on
/// USB with its coprocessor started.
const READY_TIMEOUT: Duration = Duration::from_secs(20);

/// What one of ST's coprocessor images is, going by the footer they end in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageInfo {
    pub kind: ImageKind,
    pub version: Version,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageKind {
    Fus,
    Stack,
}

impl ImageInfo {
    /// `None` for a file that does not end the way the images of STM32CubeWB
    /// V1.24.0 do, which is not to say that FUS would not take it.
    pub fn read(image: &[u8]) -> Option<ImageInfo> {
        // The image ends in this word. 92 and 88 bytes from the end are its
        // version, and a word that tells an image of FUS from a stack's.
        const END: u32 = 0xD3A1_2C5E;
        const FUS: u32 = 0x3227_9221;
        const STACK: u32 = 0x2337_2991;

        let word = |from_end: usize| {
            let at = image.len().checked_sub(from_end)?;
            let bytes = image.get(at..at + 4)?;
            Some(u32::from_le_bytes(bytes.try_into().ok()?))
        };
        if word(4)? != END {
            return None;
        }
        let kind = match word(88)? {
            FUS => ImageKind::Fus,
            STACK => ImageKind::Stack,
            _ => return None,
        };
        let [major, minor, patch, _] = word(92)?.to_be_bytes();
        Some(ImageInfo {
            kind,
            version: Version {
                major,
                minor,
                patch,
            },
        })
    }

    /// The version of this kind of firmware that `firmware` has, if any.
    fn installed_in(&self, firmware: &CoprocessorFirmware) -> Option<Version> {
        match self.kind {
            ImageKind::Fus => Some(firmware.fus),
            ImageKind::Stack => firmware.stack.map(|stack| stack.version),
        }
    }
}

impl std::fmt::Display for ImageInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.kind {
            ImageKind::Fus => write!(f, "FUS {}", self.version),
            ImageKind::Stack => write!(f, "wireless stack {}", self.version),
        }
    }
}

/// Install `image`, one of ST's coprocessor binaries, on the board `serial`
/// picks, or else on the only attached board whose coprocessor is running
/// FUS, as a new board's is.
pub async fn install(serial: Option<&str>, image: &[u8]) -> Result<()> {
    let (board, before) = target(serial).await?;
    let serial = board.serial().to_owned();

    let info = ImageInfo::read(image);
    if let (Some(info), Some(firmware)) = (info, installed(&before))
        && info.installed_in(&firmware) == Some(info.version)
    {
        println!("{serial}  already has {info}");
        return Ok(());
    }
    match before {
        CoprocessorStatus::Fus {
            state: FusState::Idle | FusState::Failed(_),
            ..
        } => {}
        CoprocessorStatus::Stack(_) => bail!(
            "board {serial} is {}; remove that stack first, with `coprocessor uninstall`",
            describe(&before)
        ),
        _ => bail!(
            "board {serial} cannot take an image: its coprocessor is {}",
            describe(&before)
        ),
    }
    match info {
        Some(info) => println!("Installing {info} on board {serial}."),
        None => println!("Installing an image of unknown kind on board {serial}."),
    }

    let Ok(size) = u32::try_from(image.len()) else {
        bail!("the image is too large to be a coprocessor image");
    };
    board.begin_install(ImageSize(size)).await?;

    let mut sent = 0;
    for data in image.chunks(ImageChunk::MAX_LEN) {
        let chunk = ImageChunk::new(sent, data).expect("chunks are no longer than MAX_LEN");
        board.write_install(&chunk).await?;
        sent += data.len() as u32;
        print!(
            "\rSending the image: {} %",
            u64::from(sent) * 100 / u64::from(size)
        );
        std::io::stdout().flush()?;
    }
    println!();

    // FUS resets the board when it sets to work, which can be before the
    // answer is out. Only an answer that says no means anything.
    if let Ok(Err(e)) = board.finish_install().await {
        bail!("board {serial}: {e}");
    }
    drop(board);

    println!("FUS is installing it. The board resets a few times.");
    let after = settled(&serial).await?;
    if installed(&after) == installed(&before) {
        bail!("board {serial}: FUS finished without installing anything");
    }
    println!("{serial}  {}", describe(&after));
    Ok(())
}

/// Remove the wireless stack from `board`, leaving its coprocessor with FUS
/// alone, as it is on a new board.
pub async fn uninstall(board: Board) -> Result<()> {
    let serial = board.serial().to_owned();
    let before = board.coprocessor_status().await?;
    if !matches!(installed(&before), Some(firmware) if firmware.stack.is_some()) {
        bail!(
            "board {serial} has no wireless stack to remove: its coprocessor is {}",
            describe(&before)
        );
    }
    board.uninstall_stack().await?;
    drop(board);

    println!("FUS is removing the stack. The board resets a few times.");
    let after = settled(&serial).await?;
    if matches!(installed(&after), Some(firmware) if firmware.stack.is_some()) {
        bail!("board {serial}: FUS did not remove the stack");
    }
    println!("{serial}  {}", describe(&after));
    Ok(())
}

/// The board to install on, and what its coprocessor is doing. Gives a board
/// that was flashed or reset a moment ago the time to turn up.
async fn target(serial: Option<&str>) -> Result<(Board, CoprocessorStatus)> {
    let deadline = Instant::now() + READY_TIMEOUT;
    loop {
        if let Some(found) = ready_target(serial).await? {
            return Ok(found);
        }
        if Instant::now() > deadline {
            match serial {
                Some(serial) => {
                    bail!("board {serial} has not turned up with its coprocessor started")
                }
                None if board::devices().await?.is_empty() => {
                    bail!("no sensor board found on USB")
                }
                None => bail!(
                    "no attached board is waiting for coprocessor firmware; \
                     name one with --board to install on it all the same"
                ),
            }
        }
        tokio::time::sleep(POLL_INTERVAL).await;
    }
}

/// `None` if the board is not there yet, or not far enough into starting up
/// to tell.
async fn ready_target(serial: Option<&str>) -> Result<Option<(Board, CoprocessorStatus)>> {
    let mut devices = board::devices().await?;
    if let Some(serial) = serial {
        let prefix = serial.to_uppercase();
        devices.retain(|d| board::serial_of(d).starts_with(&prefix));
    }

    let mut boards = Vec::new();
    let mut unreachable = None;
    for device in &devices {
        let board = match Board::open(device).await {
            Ok(board) => board,
            Err(e) => {
                unreachable.get_or_insert(e);
                continue;
            }
        };
        match board.coprocessor_status().await {
            Ok(CoprocessorStatus::Starting) => return Ok(None),
            Ok(status) => boards.push((board, status)),
            // Firmware from before it could say: not one to install on.
            Err(_) => {}
        }
    }

    if boards.len() > 1 {
        boards.retain(|(_, status)| matches!(status, CoprocessorStatus::Fus { .. }));
    }
    match (boards.len(), unreachable) {
        (1, _) => Ok(boards.pop()),
        (0, Some(e)) => Err(e),
        (0, None) => Ok(None),
        _ => bail!(
            "several boards are waiting for coprocessor firmware; pick one with --board:{}",
            boards
                .iter()
                .map(|(board, _)| format!("\n  {}", board.serial()))
                .collect::<String>()
        ),
    }
}

fn installed(status: &CoprocessorStatus) -> Option<CoprocessorFirmware> {
    match status {
        CoprocessorStatus::Starting => None,
        CoprocessorStatus::Stack(firmware) | CoprocessorStatus::Fus { firmware, .. } => {
            Some(*firmware)
        }
    }
}

/// Wait for FUS on the board with this serial number to be done, one way or
/// the other.
async fn settled(serial: &str) -> Result<CoprocessorStatus> {
    let deadline = Instant::now() + INSTALL_TIMEOUT;
    loop {
        tokio::time::sleep(POLL_INTERVAL).await;

        // The board drops off USB with every reset: not finding it, and not
        // getting an answer, are both part of the wait.
        let status = match Board::select(Some(serial)).await {
            Ok(board) => board.coprocessor_status().await.ok(),
            Err(_) => None,
        };
        match status {
            Some(
                status @ (CoprocessorStatus::Stack(_)
                | CoprocessorStatus::Fus {
                    state: FusState::Idle,
                    ..
                }),
            ) => return Ok(status),
            Some(CoprocessorStatus::Fus {
                state: FusState::Failed(e),
                ..
            }) => bail!("board {serial}: {e}"),
            _ if Instant::now() > deadline => {
                bail!("board {serial}: FUS is not done after {INSTALL_TIMEOUT:?}")
            }
            _ => {}
        }
    }
}

pub fn describe(status: &CoprocessorStatus) -> String {
    match status {
        CoprocessorStatus::Starting => "starting".into(),
        CoprocessorStatus::Stack(firmware) => {
            let stack = firmware
                .stack
                .map_or("a wireless stack".into(), |s| s.to_string());
            format!("running {stack} (FUS {})", firmware.fus)
        }
        CoprocessorStatus::Fus { firmware, state } => {
            let stack = firmware
                .stack
                .map_or("no wireless stack".into(), |s| s.to_string());
            let state = match state {
                FusState::Idle => "idle".into(),
                FusState::Busy => "busy".into(),
                FusState::Failed(e) => format!("idle, its last install failed: {e}"),
            };
            format!("in FUS {} with {stack} installed, {state}", firmware.fus)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file that ends the way ST's images do: the three words of the
    /// footer that are read, where they are in it.
    fn image(kind: u32, version: u32) -> Vec<u8> {
        let mut image = vec![0xab; 1000];
        let end = image.len();
        image[end - 92..end - 88].copy_from_slice(&version.to_le_bytes());
        image[end - 88..end - 84].copy_from_slice(&kind.to_le_bytes());
        image[end - 4..].copy_from_slice(&0xD3A1_2C5Eu32.to_le_bytes());
        image
    }

    #[test]
    fn reads_what_an_image_is() {
        let fus = ImageInfo::read(&image(0x3227_9221, 0x0202_0000)).unwrap();
        assert_eq!(fus.to_string(), "FUS 2.2.0");
        let stack = ImageInfo::read(&image(0x2337_2991, 0x0118_0002)).unwrap();
        assert_eq!(stack.to_string(), "wireless stack 1.24.0");
    }

    #[test]
    fn does_not_guess_at_other_files() {
        assert_eq!(ImageInfo::read(&[]), None);
        assert_eq!(ImageInfo::read(&[0xab; 1000]), None);
        assert_eq!(ImageInfo::read(&image(0x1234_5678, 0x0202_0000)), None);
    }
}
