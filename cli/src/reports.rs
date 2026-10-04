//! The sensor reports that boards send over their network, taken in through
//! one board that stays attached.

use std::{
    collections::BTreeMap,
    fmt::Write as _,
    net::SocketAddr,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result};
use postcard_rpc::host_client::MultiSubRxError;
use protocol::{BoardId, Report, SensorReadResult};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
};

use crate::board::Board;

/// How long to wait before looking again for a board that has gone.
const RETRY: Duration = Duration::from_secs(2);

/// Readings are served for this long after the report they came in. A board
/// that has gone quiet then has none, rather than its last ones for ever.
const STALE_AFTER: Duration = Duration::from_secs(60);

/// How long a scraper gets to send its request.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

/// Print the reports as they arrive, until interrupted.
pub async fn watch(serial: Option<&str>) -> Result<()> {
    collect(serial, |report| println!("{}", describe(&report))).await
}

/// Serve the readings in the reports to Prometheus at `listen`, until
/// interrupted.
pub async fn export(serial: Option<&str>, listen: SocketAddr) -> Result<()> {
    let listener = TcpListener::bind(listen)
        .await
        .with_context(|| format!("could not listen on {listen}"))?;
    eprintln!(
        "Serving metrics at http://{}/metrics.",
        listener.local_addr()?
    );

    let metrics = Arc::new(Mutex::new(Metrics::default()));
    tokio::spawn(serve(listener, metrics.clone()));
    collect(serial, |report| {
        metrics.lock().unwrap().record(report, SystemTime::now());
    })
    .await
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
                Err(MultiSubRxError::IoClosed) => anyhow::bail!("board {} is gone", board.serial()),
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

/// What the reports received so far come to, board by board.
#[derive(Default)]
pub struct Metrics {
    boards: BTreeMap<BoardId, BoardMetrics>,
}

struct BoardMetrics {
    last: Report,
    last_at: SystemTime,
    received: u64,
    /// Reports the board sent that never arrived, by the gaps in their
    /// sequence numbers.
    lost: u64,
    capacitance_errors: [u64; 4],
}

impl Metrics {
    pub fn record(&mut self, report: Report, now: SystemTime) {
        let board = self.boards.entry(report.board).or_insert(BoardMetrics {
            last: report,
            last_at: now,
            received: 0,
            lost: 0,
            capacitance_errors: [0; 4],
        });
        // A sequence number that has not gone up is a board that restarted,
        // which loses nothing.
        if let Some(step) = report.sequence.checked_sub(board.last.sequence) {
            board.lost += u64::from(step.saturating_sub(1));
        }
        board.received += 1;
        for (errors, reading) in board
            .capacitance_errors
            .iter_mut()
            .zip(&report.readings.capacitance)
        {
            *errors += u64::from(reading.is_err());
        }
        board.last = report;
        board.last_at = now;
    }

    /// The metrics in Prometheus's text format.
    pub fn render(&self, now: SystemTime) -> String {
        let mut received = Family::new(
            "sensor_board_reports_total",
            "counter",
            "Reports received from a board.",
        );
        let mut lost = Family::new(
            "sensor_board_reports_lost_total",
            "counter",
            "Reports a board sent that were not received, going by their sequence numbers.",
        );
        let mut last_at = Family::new(
            "sensor_board_last_report_timestamp_seconds",
            "gauge",
            "When the last report from a board was received, in Unix time.",
        );
        let mut firmware = Family::new(
            "sensor_board_firmware_build",
            "gauge",
            "The build of the firmware a board runs: when it was built, in Unix time.",
        );
        let mut capacitance = Family::new(
            "sensor_board_capacitance",
            "gauge",
            "Raw reading of a capacitance channel, from a report of the last minute.",
        );
        let mut capacitance_errors = Family::new(
            "sensor_board_capacitance_errors_total",
            "counter",
            "Readings of a capacitance channel that failed.",
        );

        for (id, board) in &self.boards {
            let labels = format!("board=\"{id}\"");
            received.sample(&labels, board.received);
            lost.sample(&labels, board.lost);
            let since_epoch = board.last_at.duration_since(UNIX_EPOCH).unwrap_or_default();
            last_at.sample(&labels, since_epoch.as_secs());
            firmware.sample(&labels, board.last.firmware.0);

            // A clock that was set back makes the report look like it is
            // from the future: fresh.
            let age = now.duration_since(board.last_at).unwrap_or_default();
            let readings = &board.last.readings;
            for (channel, reading) in readings.capacitance.iter().enumerate() {
                let labels = format!("{labels},channel=\"{channel}\"");
                capacitance_errors.sample(&labels, board.capacitance_errors[channel]);
                if let Ok(value) = reading
                    && age <= STALE_AFTER
                {
                    capacitance.sample(&labels, value.value);
                }
            }
        }

        [
            received,
            lost,
            last_at,
            firmware,
            capacitance,
            capacitance_errors,
        ]
        .iter()
        .map(Family::render)
        .collect()
    }
}

/// The samples of one metric.
struct Family {
    name: &'static str,
    kind: &'static str,
    help: &'static str,
    samples: String,
}

impl Family {
    fn new(name: &'static str, kind: &'static str, help: &'static str) -> Family {
        Family {
            name,
            kind,
            help,
            samples: String::new(),
        }
    }

    fn sample(&mut self, labels: &str, value: impl Into<u64>) {
        let (name, value) = (self.name, value.into());
        writeln!(self.samples, "{name}{{{labels}}} {value}").expect("writing to a string");
    }

    fn render(&self) -> String {
        let Family {
            name,
            kind,
            help,
            samples,
        } = self;
        format!("# HELP {name} {help}\n# TYPE {name} {kind}\n{samples}")
    }
}

/// Answer every scrape with the metrics as they are then.
async fn serve(listener: TcpListener, metrics: Arc<Mutex<Metrics>>) {
    loop {
        match listener.accept().await {
            Ok((stream, _)) => {
                let metrics = metrics.clone();
                // A scraper that hangs up early is no concern of anyone's.
                tokio::spawn(async move { respond(stream, &metrics).await.ok() });
            }
            Err(e) => {
                eprintln!("Could not accept a connection: {e}.");
                tokio::time::sleep(RETRY).await;
            }
        }
    }
}

async fn respond(mut stream: TcpStream, metrics: &Mutex<Metrics>) -> std::io::Result<()> {
    let read = tokio::time::timeout(REQUEST_TIMEOUT, request_line(&mut stream));
    let Ok(request) = read.await else {
        return Ok(());
    };
    let request = request?;

    let (status, body) = if is_for_metrics(&request) {
        let metrics = metrics.lock().unwrap().render(SystemTime::now());
        ("200 OK", metrics)
    } else {
        ("404 Not Found", "The metrics are at /metrics.\n".to_owned())
    };
    let response = format!(
        "HTTP/1.1 {status}\r\n\
         Content-Type: text/plain; version=0.0.4; charset=utf-8\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n\
         {body}",
        body.len()
    );
    stream.write_all(response.as_bytes()).await?;
    stream.shutdown().await
}

/// The first line of an HTTP request, once all of its head has arrived.
async fn request_line(stream: &mut TcpStream) -> std::io::Result<String> {
    /// More than any scraper sends before the body.
    const MAX_HEAD: usize = 8 * 1024;

    let mut head = Vec::new();
    let mut chunk = [0; 1024];
    while !head.windows(4).any(|w| w == b"\r\n\r\n") && head.len() < MAX_HEAD {
        match stream.read(&mut chunk).await? {
            0 => break,
            n => head.extend_from_slice(&chunk[..n]),
        }
    }
    let line = head.split(|&b| b == b'\r').next().unwrap_or_default();
    Ok(String::from_utf8_lossy(line).into_owned())
}

fn is_for_metrics(request_line: &str) -> bool {
    let mut parts = request_line.split(' ');
    let (method, target) = (parts.next(), parts.next().unwrap_or_default());
    let path = target.split('?').next().unwrap_or_default();
    method == Some("GET") && path == "/metrics"
}

#[cfg(test)]
mod tests {
    use protocol::{BuildId, Readings, SensorReadError, SensorValue};

    use super::*;

    fn report(sequence: u32) -> Report {
        Report {
            board: BoardId([
                0x4b, 0x00, 0x41, 0x00, 0x03, 0x50, 0x47, 0x55, 0x32, 0x30, 0x31, 0x20,
            ]),
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

    fn at(seconds: u64) -> SystemTime {
        UNIX_EPOCH + Duration::from_secs(seconds)
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
    fn metrics_are_in_prometheus_text_format() {
        let mut metrics = Metrics::default();
        metrics.record(report(0), at(1000));
        assert_eq!(
            metrics.render(at(1010)),
            "# HELP sensor_board_reports_total Reports received from a board.\n\
             # TYPE sensor_board_reports_total counter\n\
             sensor_board_reports_total{board=\"4B0041000350475532303120\"} 1\n\
             # HELP sensor_board_reports_lost_total Reports a board sent that were not \
             received, going by their sequence numbers.\n\
             # TYPE sensor_board_reports_lost_total counter\n\
             sensor_board_reports_lost_total{board=\"4B0041000350475532303120\"} 0\n\
             # HELP sensor_board_last_report_timestamp_seconds When the last report from a \
             board was received, in Unix time.\n\
             # TYPE sensor_board_last_report_timestamp_seconds gauge\n\
             sensor_board_last_report_timestamp_seconds{board=\"4B0041000350475532303120\"} 1000\n\
             # HELP sensor_board_firmware_build The build of the firmware a board runs: when it \
             was built, in Unix time.\n\
             # TYPE sensor_board_firmware_build gauge\n\
             sensor_board_firmware_build{board=\"4B0041000350475532303120\"} 1791145757\n\
             # HELP sensor_board_capacitance Raw reading of a capacitance channel, from a \
             report of the last minute.\n\
             # TYPE sensor_board_capacitance gauge\n\
             sensor_board_capacitance{board=\"4B0041000350475532303120\",channel=\"0\"} 1234567\n\
             sensor_board_capacitance{board=\"4B0041000350475532303120\",channel=\"1\"} 0\n\
             sensor_board_capacitance{board=\"4B0041000350475532303120\",channel=\"3\"} 42\n\
             # HELP sensor_board_capacitance_errors_total Readings of a capacitance channel \
             that failed.\n\
             # TYPE sensor_board_capacitance_errors_total counter\n\
             sensor_board_capacitance_errors_total{board=\"4B0041000350475532303120\",channel=\"0\"} 0\n\
             sensor_board_capacitance_errors_total{board=\"4B0041000350475532303120\",channel=\"1\"} 0\n\
             sensor_board_capacitance_errors_total{board=\"4B0041000350475532303120\",channel=\"2\"} 1\n\
             sensor_board_capacitance_errors_total{board=\"4B0041000350475532303120\",channel=\"3\"} 0\n"
        );
    }

    fn sample(metrics: &Metrics, now: SystemTime, name: &str) -> Option<u64> {
        let rendered = metrics.render(now);
        let line = rendered.lines().find(|l| l.starts_with(name))?;
        line.rsplit(' ').next()?.parse().ok()
    }

    #[test]
    fn gaps_in_the_sequence_are_lost_reports_and_a_restart_is_not() {
        let mut metrics = Metrics::default();
        for sequence in [5, 6, 9, 10] {
            metrics.record(report(sequence), at(1000));
        }
        let lost = |m: &Metrics| sample(m, at(1000), "sensor_board_reports_lost_total{");
        assert_eq!(lost(&metrics), Some(2));

        // The board restarts, and counts from 0 again.
        metrics.record(report(0), at(1000));
        metrics.record(report(1), at(1000));
        assert_eq!(lost(&metrics), Some(2));
        assert_eq!(
            sample(&metrics, at(1000), "sensor_board_reports_total{"),
            Some(6)
        );
    }

    #[test]
    fn readings_of_a_board_that_went_quiet_are_not_served() {
        let mut metrics = Metrics::default();
        metrics.record(report(0), at(1000));
        let reading = |now| sample(&metrics, now, "sensor_board_capacitance{");
        assert_eq!(reading(at(1060)), Some(1234567));
        assert_eq!(reading(at(1061)), None);
        // What stays is when it was last heard from.
        let last = "sensor_board_last_report_timestamp_seconds{";
        assert_eq!(sample(&metrics, at(1061), last), Some(1000));
    }

    #[test]
    fn only_get_metrics_is_answered_with_them() {
        assert!(is_for_metrics("GET /metrics HTTP/1.1"));
        assert!(is_for_metrics("GET /metrics?x=1 HTTP/1.1"));
        assert!(!is_for_metrics("GET / HTTP/1.1"));
        assert!(!is_for_metrics("POST /metrics HTTP/1.1"));
        assert!(!is_for_metrics(""));
    }
}
