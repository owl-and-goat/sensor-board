//! The sensor reports that boards send over their network, taken in through
//! one board that stays attached.

use std::time::Duration;

use anyhow::{Result, bail};
use postcard_rpc::host_client::MultiSubRxError;
use protocol::{Report, SensorReadResult};

use crate::board::Board;

/// How long to wait before looking again for a board that has gone.
const RETRY: Duration = Duration::from_secs(2);

/// Print the reports as they arrive, until interrupted.
pub async fn watch(serial: Option<&str>) -> Result<()> {
    collect(serial, |report| println!("{}", describe(&report))).await
}

/// Hand every report that arrives through the board `serial` picks to
/// `on_report`, until interrupted. A board that goes away (unplugged,
/// reflashed, the computer asleep) is waited for.
async fn collect(serial: Option<&str>, mut on_report: impl FnMut(Report)) -> Result<()> {
    // The first time, a board that is not there is an error: a mistyped
    // serial should not be waited for.
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

/// `Ok` when interrupted, an error when the board is lost.
async fn collect_through(board: &Board, on_report: &mut impl FnMut(Report)) -> Result<()> {
    // Subscribed before the board is told to collect, so that nothing it
    // passes on is missed.
    let mut reports = board.reports().await?;
    board.start_collecting().await?;
    eprintln!("Collecting reports through board {}.", board.serial());

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                // Best effort: a board left collecting only does work that
                // nobody looks at.
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

/// A report as a line of text.
pub fn describe(report: &Report) -> String {
    let Report {
        board,
        firmware,
        sequence,
        readings,
    } = report;
    let capacitance: Vec<String> = readings.capacitance.iter().map(describe_reading).collect();
    format!(
        "{board}  build {firmware}  #{sequence}  capacitance {}",
        capacitance.join(", ")
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
    fn a_report_is_one_line() {
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
            },
        };
        assert_eq!(
            describe(&report),
            "4B0041000350475532303120  build 1791145757  #7  capacitance 1234567, 0, \
             failed (I2C ACK Not Received), 42"
        );
    }
}
