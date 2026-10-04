//! Requests from a handle to the task that serves them, one at a time.
//!
//! The asking side waits a bounded time for its answer. What it cannot do is
//! make the serving task drop what it is in the middle of: a request that was
//! given up on is still carried out to the end, and its answer thrown away.

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, with_timeout};

/// Tells one request, and its answer, from the next.
type Id = u32;

pub struct Channel<Request, Answer> {
    request: Signal<ThreadModeRawMutex, (Id, Request)>,
    answer: Signal<ThreadModeRawMutex, (Id, Answer)>,
}

/// A request the serving task has taken, and owes an answer.
pub struct Pending(Id);

impl<Request: Send, Answer: Send> Channel<Request, Answer> {
    pub const fn new() -> Self {
        Self {
            request: Signal::new(),
            answer: Signal::new(),
        }
    }

    /// The asking end, for a handle, and the serving end, for its task.
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
    /// `None` if the task has not answered within `timeout`.
    pub async fn ask(&mut self, request: Request, timeout: Duration) -> Option<Answer> {
        let id = self.next;
        self.next = id.wrapping_add(1);
        self.channel.request.signal((id, request));

        let answer = async {
            loop {
                let (answered, answer) = self.channel.answer.wait().await;
                // Under another ID it answers a request that was given up on.
                if answered == id {
                    return answer;
                }
            }
        };
        with_timeout(timeout, answer).await.ok()
    }
}
