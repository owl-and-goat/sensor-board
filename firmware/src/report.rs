//! Sensor reports. Every board periodically multicasts its readings to its
//! network. A board that the host has told to collect also receives the
//! reports, including its own, and forwards them over USB.

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
    thread, update,
};

/// The interval between reports.
const INTERVAL: Duration = Duration::from_secs(10);

/// Timeout for a request to start or stop collecting. It covers one
/// [`thread::Socket`] request, which has its own timeout, and the report
/// that may be in progress. Shorter than the host's own timeout.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

const _: () = assert!(Report::MAX_LEN <= thread::MAX_DATAGRAM_LEN);

/// A request from the handle to start or stop collecting.
struct Collect(bool);

pub struct Builder {
    /// The UDP socket for reports.
    pub socket: thread::Socket,
    pub capacitance: &'static sensor::Shared<CapacitanceSensor<'static>>,
}

impl Builder {
    /// Panics if called a second time: the firmware has one reporting task.
    pub fn init(self) -> (Task, Handle) {
        static REQUESTS: StaticCell<request::Channel<Collect, NetworkResult>> = StaticCell::new();
        let (client, server) = REQUESTS.init(request::Channel::new()).split();

        let task = Task {
            socket: self.socket,
            capacitance: self.capacitance,
            requests: server,
            board: BoardId(embassy_stm32::uid::uid()),
            sequence: 0,
        };
        (task, Handle { requests: client })
    }
}

/// Controls the reporting task from the rest of the firmware.
pub struct Handle {
    requests: request::Client<Collect, NetworkResult>,
}

impl Handle {
    /// Start or stop forwarding received reports to the host.
    pub async fn collect(&mut self, on: bool) -> NetworkResult {
        self.requests
            .call(Collect(on), REQUEST_TIMEOUT)
            .await
            .unwrap_or(Err(NetworkError::Unresponsive))
    }
}

pub struct Task {
    socket: thread::Socket,
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
                self.socket.receive(),
            );
            match next.await {
                Either3::First(()) => self.report().await,
                Either3::Second((pending, Collect(on))) => {
                    let outcome = self.socket.listen(on).await;
                    self.requests.respond(pending, outcome);
                }
                Either3::Third(received) => match Report::decode(&received.datagram) {
                    Some(report) => publisher.report_received(&report).await,
                    None => debug!("report: a datagram that is no report of this firmware's"),
                },
            }
        }
    }

    /// Read the sensors and send a report to every board.
    async fn report(&mut self) {
        let report = Report {
            board: self.board,
            firmware: update::build(),
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

        // Sending fails while the board is not on a network. The next report
        // may succeed.
        if let Err(e) = self.socket.broadcast(datagram).await {
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
