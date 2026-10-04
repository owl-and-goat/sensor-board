//! Updating a board's firmware through the firmware itself: the image is
//! staged next to the one that runs, and the board's bootloader swaps the two.

use std::{
    path::Path,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, bail};
use protocol::{
    BuildId, BuildMarker, ImageChunk, ImageDigest, ImageSize, UpdateImage, UpdateStatus,
};
use sha2::{Digest, Sha256};

use crate::board::Board;

/// How long a board gets to come back as the new build. Its bootloader takes
/// some fifteen seconds over the swap.
const RESTART_TIMEOUT: Duration = Duration::from_secs(90);
const POLL_INTERVAL: Duration = Duration::from_millis(500);

/// A firmware image file.
pub struct Image {
    bytes: Vec<u8>,
    describes: UpdateImage,
}

impl Image {
    pub fn read(path: &Path) -> Result<Image> {
        let bytes =
            std::fs::read(path).with_context(|| format!("could not read {}", path.display()))?;
        Image::from_bytes(bytes).with_context(|| format!("{} cannot be used", path.display()))
    }

    fn from_bytes(bytes: Vec<u8>) -> Result<Image> {
        let Some(build) = BuildMarker::find(&bytes) else {
            bail!("it is not an image of this firmware");
        };
        let size = u32::try_from(bytes.len()).context("it is far too long")?;
        let describes = UpdateImage {
            build,
            size: ImageSize(size),
            digest: ImageDigest(Sha256::digest(&bytes).into()),
        };
        Ok(Image { bytes, describes })
    }

    pub fn build(&self) -> BuildId {
        self.describes.build
    }
}

/// Send `image` to `board`, which checks and stages it.
pub async fn stage(board: &Board, image: &Image) -> Result<()> {
    board.begin_update(&image.describes).await?;
    let chunks = image.bytes.chunks(ImageChunk::MAX_LEN);
    let total = chunks.len();
    for (i, data) in chunks.enumerate() {
        let offset = (i * ImageChunk::MAX_LEN) as u32;
        let chunk = ImageChunk::new(offset, data).expect("a chunk is no longer than MAX_LEN");
        board.write_update(&chunk).await?;
        if (i + 1) % 32 == 0 || i + 1 == total {
            eprint!(
                "\rSending build {}: {}%",
                image.build(),
                100 * (i + 1) / total
            );
        }
    }
    eprintln!();
    board.finish_update().await
}

/// Update `board` to `image`, and wait for it to be running it.
pub async fn update(board: Board, image: &Image) -> Result<()> {
    let serial = board.serial().to_owned();
    if board.firmware_status().await?.build == image.build() {
        println!("Board {serial} runs build {} already.", image.build());
        return Ok(());
    }

    stage(&board, image).await?;
    board.apply_update().await?;
    drop(board);
    println!("Board {serial} is restarting into build {}.", image.build());

    let deadline = Instant::now() + RESTART_TIMEOUT;
    let board = loop {
        tokio::time::sleep(POLL_INTERVAL).await;
        // Gone from USB while its bootloader swaps the images.
        if let Ok(board) = Board::select(Some(&serial)).await
            && let Ok(running) = board.firmware_status().await
        {
            if running.build != image.build() {
                bail!(
                    "board {serial} came back as build {}: the update did not take",
                    running.build
                );
            }
            break board;
        }
        if Instant::now() > deadline {
            bail!("board {serial} did not come back");
        }
    };
    println!(
        "Board {serial} runs build {}: {}.",
        image.build(),
        describe(&board.firmware_status().await?.update)
    );
    Ok(())
}

pub fn describe(status: &UpdateStatus) -> String {
    match status {
        UpdateStatus::Starting => "starting".into(),
        UpdateStatus::Unavailable => "cannot take an update without a wireless stack".into(),
        UpdateStatus::Settled => "settled".into(),
        UpdateStatus::Receiving { image, received } => format!(
            "receiving build {}, {} of {} bytes",
            image.build, received, image.size.0
        ),
        UpdateStatus::Staged(image) => format!("build {} is staged", image.build),
        UpdateStatus::OnTrial => "on trial, and kept once the board is back on its network".into(),
        UpdateStatus::RolledBack => {
            "the last update did not work out, and the firmware before it is back".into()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_an_image_of_the_firmware_is_taken() {
        assert!(Image::from_bytes(vec![0; 4096]).is_err());

        let mut bytes = vec![0x11; 300];
        bytes.extend_from_slice(b"sensor-board-fw\0");
        bytes.extend_from_slice(&1_791_145_757u32.to_le_bytes());
        bytes.extend_from_slice(&[0x22; 100]);
        let image = Image::from_bytes(bytes.clone()).unwrap();
        assert_eq!(image.build(), BuildId(1_791_145_757));
        assert_eq!(image.describes.size, ImageSize(420));
        assert_eq!(
            image.describes.digest.0,
            <[u8; 32]>::from(Sha256::digest(&bytes))
        );
    }
}
