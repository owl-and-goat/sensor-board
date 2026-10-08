//! The sensor reports that boards send over their network, taken in through
//! one board that stays attached.

use std::{
    collections::BTreeMap,
    net::SocketAddr,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result, bail};
use metrics::{counter, describe_counter, describe_gauge, gauge};
use metrics_exporter_prometheus::PrometheusBuilder;
use metrics_util::MetricKindMask;
use postcard_rpc::host_client::MultiSubRxError;
use protocol::{BoardId, Report, SensorReadResult};

use crate::board::Board;

/// How long to wait before looking again for a board that has gone.
const RETRY: Duration = Duration::from_secs(2);

/// What a report says of a board is served for this long after the last one
/// from it. A board that has gone quiet then has its counts left, rather
/// than its last readings for ever.
const STALE_AFTER: Duration = Duration::from_secs(60);

/// Print the reports as they arrive, until interrupted.
pub async fn watch(serial: Option<&str>) -> Result<()> {
    collect(serial, |report| println!("{}", describe(&report))).await
}

/// Serve the readings in the reports to Prometheus at `listen`, until
/// interrupted.
pub async fn export(serial: Option<&str>, listen: SocketAddr) -> Result<()> {
    exporter()
        .with_http_listener(listen)
        .install()
        .with_context(|| format!("could not serve metrics at {listen}"))?;
    eprintln!("Serving metrics at http://{listen}/metrics.");

    let mut metrics = Metrics::new();
    collect(serial, |report| {
        metrics.record(&report, SystemTime::now());
    })
    .await
}

/// What keeps the metrics, and renders them for Prometheus.
fn exporter() -> PrometheusBuilder {
    PrometheusBuilder::new().idle_timeout(MetricKindMask::GAUGE, Some(STALE_AFTER))
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

const REPORTS: &str = "sensor_board_reports_total";
const REPORTS_LOST: &str = "sensor_board_reports_lost_total";
const LAST_REPORT: &str = "sensor_board_last_report_timestamp_seconds";
const FIRMWARE_BUILD: &str = "sensor_board_firmware_build";
const CAPACITANCE: &str = "sensor_board_capacitance";
const CAPACITANCE_ERRORS: &str = "sensor_board_capacitance_errors_total";

/// Turns the reports into metrics, board by board.
pub struct Metrics {
    /// The sequence number of the last report from each board.
    sequences: BTreeMap<BoardId, u32>,
}

impl Metrics {
    pub fn new() -> Metrics {
        describe_counter!(REPORTS, "Reports received from a board.");
        describe_counter!(
            REPORTS_LOST,
            "Reports a board sent that were not received, going by their sequence numbers."
        );
        describe_gauge!(
            LAST_REPORT,
            "When the last report from a board was received, in Unix time."
        );
        describe_gauge!(
            FIRMWARE_BUILD,
            "The build of the firmware a board runs: when it was built, in Unix time."
        );
        describe_gauge!(
            CAPACITANCE,
            "Raw reading of a capacitance channel, from a report of the last minute."
        );
        describe_counter!(
            CAPACITANCE_ERRORS,
            "Readings of a capacitance channel that failed."
        );
        Metrics {
            sequences: BTreeMap::new(),
        }
    }

    pub fn record(&mut self, report: &Report, now: SystemTime) {
        let board = report.board.to_string();

        // Reports the board sent that never arrived, by the gap in their
        // sequence numbers. A number that has not gone up is a board that
        // restarted, which loses nothing.
        let last = self.sequences.insert(report.board, report.sequence);
        let step = last.and_then(|last| report.sequence.checked_sub(last));
        let lost = step.map_or(0, |step| step.saturating_sub(1));

        counter!(REPORTS, "board" => board.clone()).increment(1);
        counter!(REPORTS_LOST, "board" => board.clone()).increment(lost.into());
        let since_epoch = now.duration_since(UNIX_EPOCH).unwrap_or_default();
        gauge!(LAST_REPORT, "board" => board.clone()).set(since_epoch.as_secs() as f64);
        gauge!(FIRMWARE_BUILD, "board" => board.clone()).set(report.firmware.0);

        for (channel, reading) in report.readings.capacitance.iter().enumerate() {
            let labels = [("board", board.clone()), ("channel", channel.to_string())];
            counter!(CAPACITANCE_ERRORS, &labels).increment(reading.is_err().into());
            if let Ok(reading) = reading {
                gauge!(CAPACITANCE, &labels).set(reading.value);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use metrics_exporter_prometheus::PrometheusHandle;
    use protocol::{BuildId, Readings, SensorReadError, SensorValue};

    use super::*;

    const BOARD: &str = "4B0041000350475532303120";

    fn report(sequence: u32) -> Report {
        Report {
            board: BOARD.parse().unwrap(),
            firmware: BuildId(1_791_145_757),
            sequence,
            readings: Readings {
                capacitance: [
                    Ok(SensorValue { value: 1234567 }),
                    Ok(SensorValue { value: 0 }),
                    Err(SensorReadError::Nack),
                    Ok(SensorValue { value: 42 }),
                ],
            },
        }
    }

    /// What `exporter` serves once reports with these sequence numbers have
    /// arrived.
    fn metrics_of(exporter: PrometheusBuilder, sequences: &[u32]) -> PrometheusHandle {
        let recorder = exporter.build_recorder();
        metrics::with_local_recorder(&recorder, || {
            let mut metrics = Metrics::new();
            for &sequence in sequences {
                let now = UNIX_EPOCH + Duration::from_secs(1000);
                metrics.record(&report(sequence), now);
            }
        });
        recorder.handle()
    }

    /// The value of the metric `name` whose labels include `labels`.
    fn sample(rendered: &str, name: &str, labels: &[&str]) -> Option<f64> {
        let mut samples = rendered.lines().filter(|line| {
            line.strip_prefix(name)
                .is_some_and(|rest| rest.starts_with('{'))
        });
        let line = samples.find(|line| labels.iter().all(|label| line.contains(label)))?;
        line.rsplit(' ').next()?.parse().ok()
    }

    #[test]
    fn a_report_is_one_line() {
        assert_eq!(
            describe(&report(7)),
            "4B0041000350475532303120  build 1791145757  #7  capacitance 1234567, 0, \
             failed (I2C ACK Not Received), 42"
        );
    }

    #[test]
    fn a_report_becomes_metrics_of_its_board() {
        let rendered = metrics_of(exporter(), &[0]).render();
        let board = format!("board=\"{BOARD}\"");
        let of_board = |name| sample(&rendered, name, &[&board]);
        assert_eq!(of_board(REPORTS), Some(1.0));
        assert_eq!(of_board(REPORTS_LOST), Some(0.0));
        assert_eq!(of_board(LAST_REPORT), Some(1000.0));
        assert_eq!(of_board(FIRMWARE_BUILD), Some(1_791_145_757.0));

        let of_channel = |name, channel| {
            let channel = format!("channel=\"{channel}\"");
            sample(&rendered, name, &[&board, &channel])
        };
        assert_eq!(of_channel(CAPACITANCE, 0), Some(1_234_567.0));
        assert_eq!(of_channel(CAPACITANCE, 1), Some(0.0));
        assert_eq!(of_channel(CAPACITANCE, 2), None);
        assert_eq!(of_channel(CAPACITANCE, 3), Some(42.0));
        assert_eq!(of_channel(CAPACITANCE_ERRORS, 1), Some(0.0));
        assert_eq!(of_channel(CAPACITANCE_ERRORS, 2), Some(1.0));

        let described = "# HELP sensor_board_reports_total Reports received from a board.\n\
                         # TYPE sensor_board_reports_total counter\n";
        assert!(rendered.contains(described), "{rendered}");
        assert!(rendered.contains("# TYPE sensor_board_capacitance gauge\n"));
    }

    #[test]
    fn gaps_in_the_sequence_are_lost_reports_and_a_restart_is_not() {
        let board = format!("board=\"{BOARD}\"");
        let rendered = metrics_of(exporter(), &[5, 6, 9, 10]).render();
        assert_eq!(sample(&rendered, REPORTS_LOST, &[&board]), Some(2.0));

        // The board restarts, and counts from 0 again.
        let rendered = metrics_of(exporter(), &[5, 6, 9, 10, 0, 1]).render();
        assert_eq!(sample(&rendered, REPORTS_LOST, &[&board]), Some(2.0));
        assert_eq!(sample(&rendered, REPORTS, &[&board]), Some(6.0));
    }

    #[test]
    fn readings_of_a_board_that_went_quiet_are_not_served() {
        let stale_after = Duration::from_millis(20);
        let exporter = exporter().idle_timeout(MetricKindMask::GAUGE, Some(stale_after));
        let metrics = metrics_of(exporter, &[0]);

        // A metric is stale once two scrapes that far apart find it the
        // same.
        assert!(sample(&metrics.render(), CAPACITANCE, &[]).is_some());
        std::thread::sleep(3 * stale_after);
        let rendered = metrics.render();
        assert_eq!(sample(&rendered, CAPACITANCE, &[]), None);
        assert_eq!(sample(&rendered, LAST_REPORT, &[]), None);
        // What stays is how many reports there were.
        assert_eq!(sample(&rendered, REPORTS, &[]), Some(1.0));
    }
}
