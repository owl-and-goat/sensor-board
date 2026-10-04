//! Sensor reports. Every board sends its readings to all the boards on its
//! network at intervals. A board that the host has told to collect also
//! takes in the reports that arrive, its own among them, and passes them on
//! over USB.

use defmt::debug;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Ticker};
use protocol::{BoardId, NetworkError, NetworkResult, Readings, Report, SensorValue};
use static_cell::StaticCell;

use crate::{
    request, rpc,
    sensor::{
        self,
        capacitance::{CapacitanceSensor, Channel},
    },
    thread,
};

/// How often a board reports.
const INTERVAL: Duration = Duration::from_secs(10);

/// How long the task gets to start or stop collecting: one request of
/// [`thread::Datagrams`], which keeps to a limit of its own, after whatever
/// report it is in the middle of. Less than the host waits for an answer.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

const _: () = assert!(Report::MAX_LEN <= thread::MAX_DATAGRAM_LEN);

/// The handle asking the task to start collecting, or to stop.
struct Collect(bool);

pub struct Builder {
    pub datagrams: thread::Datagrams,
    pub capacitance: &'static sensor::Shared<CapacitanceSensor<'static>>,
}

impl Builder {
    /// Panics if called a second time: the firmware has one reporting task.
    pub fn init(self) -> (Task, Handle) {
        static REQUESTS: StaticCell<request::Channel<Collect, NetworkResult>> = StaticCell::new();
        let (client, server) = REQUESTS.init(request::Channel::new()).split();

        let task = Task {
            datagrams: self.datagrams,
            capacitance: self.capacitance,
            requests: server,
            board: BoardId(embassy_stm32::uid::uid()),
            sequence: 0,
        };
        (task, Handle { requests: client })
    }
}

/// How the rest of the firmware reaches the reporting task.
pub struct Handle {
    requests: request::Client<Collect, NetworkResult>,
}

impl Handle {
    /// Start passing the reports that arrive on to the host, or stop.
    pub async fn collect(&mut self, on: bool) -> NetworkResult {
        self.requests
            .ask(Collect(on), REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }
}

pub struct Task {
    datagrams: thread::Datagrams,
    capacitance: &'static sensor::Shared<CapacitanceSensor<'static>>,
    requests: request::Server<Collect, NetworkResult>,
    board: BoardId,
    sequence: u32,
}

impl Task {
    pub async fn run(mut self, mut publisher: rpc::Publisher) -> ! {
        let mut ticker = Ticker::every(INTERVAL);
        loop {
            let next = select3(
                ticker.next(),
                self.requests.receive(),
                self.datagrams.receive(),
            );
            match next.await {
                Either3::First(()) => self.report().await,
                Either3::Second((pending, Collect(on))) => {
                    let outcome = self.datagrams.listen(on).await;
                    self.requests.answer(pending, outcome);
                }
                Either3::Third(datagram) => match Report::decode(&datagram) {
                    Some(report) => publisher.report_received(&report).await,
                    None => debug!("report: a datagram that is no report of this firmware's"),
                },
            }
        }
    }

    /// Read the sensors and tell every board.
    async fn report(&mut self) {
        let report = Report {
            board: self.board,
            sequence: self.sequence,
            readings: self.readings().await,
        };
        self.sequence = self.sequence.wrapping_add(1);

        let mut datagram = thread::Datagram::new();
        // Cannot fail: a datagram has room for the longest report.
        let _ = datagram.resize_default(Report::MAX_LEN);
        let Some(len) = report.encode(&mut datagram).map(<[u8]>::len) else {
            debug!("report: longer than Report::MAX_LEN");
            return;
        };
        datagram.truncate(len);

        // A board that is on no network yet cannot send. The next report
        // may find it on one.
        if let Err(e) = self.datagrams.broadcast(datagram).await {
            debug!("report: not sent: {}", e);
        }
    }

    async fn readings(&mut self) -> Readings {
        let mut sensor = self.capacitance.lock().await;
        let mut capacitance = [Err(protocol::SensorReadError::Timeout); 4];
        let channels = [Channel::Ch0, Channel::Ch1, Channel::Ch2, Channel::Ch3];
        for (reading, channel) in capacitance.iter_mut().zip(channels) {
            *reading = match sensor.read_channel_capacitance(channel).await {
                Ok(value) => Ok(SensorValue { value }),
                Err(e) => Err(e.into()),
            };
        }
        Readings { capacitance }
    }
}
