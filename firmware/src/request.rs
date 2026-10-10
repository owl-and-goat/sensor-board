//! A request channel between a handle and the task that serves it. It
//! carries one request at a time.
//!
//! The client waits for the response with a timeout. A timeout does not
//! cancel the request: the server still runs it to completion, and the
//! response is discarded.

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, with_timeout};

/// Matches a response to its request.
type Id = u32;

pub struct Channel<Request, Answer> {
    request: Signal<ThreadModeRawMutex, (Id, Request)>,
    answer: Signal<ThreadModeRawMutex, (Id, Answer)>,
}

/// A request that the server has received and not yet answered.
pub struct Pending(Id);

impl<Request: Send, Answer: Send> Channel<Request, Answer> {
    pub const fn new() -> Self {
        Self {
            request: Signal::new(),
            answer: Signal::new(),
        }
    }

    /// The client end, for a handle, and the server end, for its task.
    pub fn split(&'static self) -> (Client<Request, Answer>, Server<Request, Answer>) {
        let client = Client {
            channel: self,
            next: 0,
        };
        (client, Server { channel: self })
    }
}

pub struct Server<Request: 'static, Answer: 'static> {
    channel: &'static Channel<Request, Answer>,
}

impl<Request, Answer> Clone for Server<Request, Answer> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<Request, Answer> Copy for Server<Request, Answer> {}

impl<Request: Send, Answer: Send> Server<Request, Answer> {
    pub async fn receive(&self) -> (Pending, Request) {
        let (id, request) = self.channel.request.wait().await;
        (Pending(id), request)
    }

    pub fn answer(&self, pending: Pending, answer: Answer) {
        self.channel.answer.signal((pending.0, answer));
    }
}

pub struct Client<Request: 'static, Answer: 'static> {
    channel: &'static Channel<Request, Answer>,
    next: Id,
}

impl<Request: Send, Answer: Send> Client<Request, Answer> {
    /// Send `request` and wait for the response. `None` if none arrives
    /// within `timeout`.
    pub async fn ask(&mut self, request: Request, timeout: Duration) -> Option<Answer> {
        let id = self.next;
        self.next = id.wrapping_add(1);
        self.channel.request.signal((id, request));

        let answer = async {
            loop {
                let (answered, answer) = self.channel.answer.wait().await;
                // A response with another ID is for an earlier request that
                // timed out.
                if answered == id {
                    return answer;
                }
            }
        };
        with_timeout(timeout, answer).await.ok()
    }
}
