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

pub struct Channel<Request, Response> {
    request: Signal<ThreadModeRawMutex, (Id, Request)>,
    response: Signal<ThreadModeRawMutex, (Id, Response)>,
}

/// A request that the server has received and not yet answered.
pub struct Pending(Id);

impl<Request: Send, Response: Send> Channel<Request, Response> {
    pub const fn new() -> Self {
        Self {
            request: Signal::new(),
            response: Signal::new(),
        }
    }

    /// The client end, for a handle, and the server end, for its task.
    pub fn split(&'static self) -> (Client<Request, Response>, Server<Request, Response>) {
        let client = Client {
            channel: self,
            next: 0,
        };
        (client, Server { channel: self })
    }
}

pub struct Server<Request: 'static, Response: 'static> {
    channel: &'static Channel<Request, Response>,
}

impl<Request, Response> Clone for Server<Request, Response> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<Request, Response> Copy for Server<Request, Response> {}

impl<Request: Send, Response: Send> Server<Request, Response> {
    pub async fn receive(&self) -> (Pending, Request) {
        let (id, request) = self.channel.request.wait().await;
        (Pending(id), request)
    }

    pub fn respond(&self, pending: Pending, response: Response) {
        self.channel.response.signal((pending.0, response));
    }
}

pub struct Client<Request: 'static, Response: 'static> {
    channel: &'static Channel<Request, Response>,
    next: Id,
}

impl<Request: Send, Response: Send> Client<Request, Response> {
    /// Send `request` and wait for the response. `None` if none arrives
    /// within `timeout`.
    pub async fn call(&mut self, request: Request, timeout: Duration) -> Option<Response> {
        let id = self.next;
        self.next = id.wrapping_add(1);
        self.channel.request.signal((id, request));

        let response = async {
            loop {
                let (response_id, response) = self.channel.response.wait().await;
                // A response with another ID is for an earlier request that
                // timed out.
                if response_id == id {
                    return response;
                }
            }
        };
        with_timeout(timeout, response).await.ok()
    }
}
