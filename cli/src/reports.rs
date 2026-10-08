//! The `reports` commands: receive the sensor reports that boards send over
//! their network, through one board that stays attached.

use std::time::Duration;

use anyhow::{Result, bail};
use postcard_rpc::host_client::MultiSubRxError;
use protocol::{Report, SensorReadResult};

use crate::board::Board;

/// Delay before looking again for a board that has disconnected.
const RETRY: Duration = Duration::from_secs(2);

/// The color sensor's channels, in the order of `Readings::color`.
const COLOR_CHANNELS: [&str; 5] = ["red", "green", "blue", "white", "infrared"];

/// Print the reports as they arrive, until interrupted.
pub async fn watch(serial: Option<&str>) -> Result<()> {
    collect(serial, |report| println!("{}", describe(&report))).await
}

/// Call `on_report` with every report received through the board that
/// `serial` selects, until interrupted. If the board disconnects (unplugged,
/// reflashed, the computer asleep), wait for it to come back.
async fn collect(serial: Option<&str>, mut on_report: impl FnMut(Report)) -> Result<()> {
    // A missing board is an error the first time, so that a mistyped serial
    // fails instead of waiting forever.
    let mut board = Board::select(serial).await?;
    let serial = board.serial().to_owned();
    loop {
        match collect_through(&board, &mut on_report).await {
            Ok(()) => return Ok(()),
            Err(e) => eprintln!("{e:#}. Waiting for it to come back."),
        }
        board = loop {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => return Ok(()),
                () = tokio::time::sleep(RETRY) => {}
            }
            if let Ok(board) = Board::select(Some(&serial)).await {
                break board;
            }
        };
    }
}

/// Returns `Ok` when interrupted, and an error when the board disconnects.
async fn collect_through(board: &Board, on_report: &mut impl FnMut(Report)) -> Result<()> {
    // Subscribe before telling the board to collect, so that no forwarded
    // report is missed.
    let mut reports = board.reports().await?;
    board.start_collecting().await?;
    eprintln!("Collecting reports through board {}.", board.serial());

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                // Best effort: a board that keeps collecting only does
                // unneeded work.
                let _ = board.stop_collecting().await;
                return Ok(());
            }
            report = reports.recv() => match report {
                Ok(report) => on_report(report),
                Err(MultiSubRxError::Lagged(n)) => eprintln!("Fell behind: skipped {n} reports."),
                Err(MultiSubRxError::IoClosed) => bail!("board {} is gone", board.serial()),
            },
        }
    }
}

/// Format a report as one line of text.
pub fn describe(report: &Report) -> String {
    let Report {
        board,
        firmware,
        sequence,
        readings,
    } = report;
    let capacitance: Vec<String> = readings.capacitance.iter().map(describe_reading).collect();
    let color: Vec<String> = COLOR_CHANNELS
        .iter()
        .zip(&readings.color)
        .map(|(channel, reading)| format!("{channel} {}", describe_reading(reading)))
        .collect();
    format!(
        "{board}  build {firmware}  #{sequence}  capacitance {}  color {}",
        capacitance.join(", "),
        color.join(", ")
    )
}

fn describe_reading(reading: &SensorReadResult) -> String {
    match reading {
        Ok(value) => value.value.to_string(),
        Err(e) => format!("failed ({e})"),
    }
}

#[cfg(test)]
mod tests {
    use protocol::{BuildId, Readings, SensorReadError, SensorValue};

    use super::*;

    #[test]
    fn report_is_described_on_one_line() {
        let report = Report {
            board: "4B0041000350475532303120".parse().unwrap(),
            firmware: BuildId(1_791_145_757),
            sequence: 7,
            readings: Readings {
                capacitance: [
                    Ok(SensorValue { value: 1234567 }),
                    Ok(SensorValue { value: 0 }),
                    Err(SensorReadError::Nack),
                    Ok(SensorValue { value: 42 }),
                ],
                color: [
                    Ok(SensorValue { value: 512 }),
                    Ok(SensorValue { value: 256 }),
                    Ok(SensorValue { value: 128 }),
                    Err(SensorReadError::OutOfRange),
                    Ok(SensorValue { value: 0 }),
                ],
            },
        };
        assert_eq!(
            describe(&report),
            "4B0041000350475532303120  build 1791145757  #7  capacitance 1234567, 0, \
             failed (I2C ACK Not Received), 42  color red 512, green 256, blue 128, white failed \
             (Color Sensor Channel Out of Range), infrared 0"
        );
    }
}
