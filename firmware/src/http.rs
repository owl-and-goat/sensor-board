//! A minimal HTTP/1.1 implementation for Prometheus: a server for scrape
//! requests, and a client for `PUT` requests to a Pushgateway. Every request
//! uses its own connection, which is closed after the response.

use core::fmt::{self, Write as _};
use core::net::SocketAddrV6;

use crate::thread::{Tcp, TcpChunk, TcpError};

/// The `Content-Type` of Prometheus's text format, for responses and pushes.
const CONTENT_TYPE: &str = "text/plain; version=0.0.4; charset=utf-8";

/// The most bytes read while waiting for the end of a request head, or for
/// the status line of a response.
const MAX_HEAD: usize = 2048;

#[derive(Clone, Copy, defmt::Format)]
pub enum Error {
    Tcp(TcpError),
    /// The message is not valid HTTP, is cut short, or is too long.
    Malformed,
}

impl From<TcpError> for Error {
    fn from(e: TcpError) -> Self {
        Error::Tcp(e)
    }
}

/// The request that was received.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Asked {
    /// `GET /metrics`
    Metrics,
    Other,
}

#[derive(Clone, Copy)]
pub enum Status {
    Ok,
    NotFound,
    InternalServerError,
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Status::Ok => "200 OK",
            Status::NotFound => "404 Not Found",
            Status::InternalServerError => "500 Internal Server Error",
        })
    }
}

/// Incremental parser for the head of an HTTP message. It keeps the first
/// line, and detects the blank line that ends the head.
struct Head {
    first_line: heapless::String<64>,
    first_line_done: bool,
    /// How many bytes of the terminating `\r\n\r\n` have matched so far.
    ending: usize,
    len: usize,
}

impl Head {
    const ENDING: &[u8] = b"\r\n\r\n";

    fn new() -> Self {
        Head {
            first_line: heapless::String::new(),
            first_line_done: false,
            ending: 0,
            len: 0,
        }
    }

    fn is_complete(&self) -> bool {
        self.ending == Self::ENDING.len()
    }

    /// Feed received bytes to the parser.
    fn take(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.is_complete() {
                return;
            }
            self.len += 1;
            self.ending = match byte {
                _ if byte == Self::ENDING[self.ending] => self.ending + 1,
                b'\r' => 1,
                _ => 0,
            };
            self.first_line_done |= byte == b'\r' || byte == b'\n';
            if !self.first_line_done {
                // A first line that does not fit is truncated.
                let _ = self.first_line.push(char::from(byte));
            }
        }
    }
}

/// Read a request from an accepted connection.
pub async fn request(tcp: &mut Tcp) -> Result<Asked, Error> {
    let mut head = Head::new();
    while !head.is_complete() {
        let chunk = tcp.receive().await?;
        if chunk.is_empty() || head.len > MAX_HEAD {
            return Err(Error::Malformed);
        }
        head.take(&chunk);
    }

    let mut words = head.first_line.split(' ');
    let (method, target) = (words.next(), words.next().unwrap_or_default());
    let path = target.split('?').next().unwrap_or_default();
    if method == Some("GET") && path == "/metrics" {
        Ok(Asked::Metrics)
    } else {
        Ok(Asked::Other)
    }
}

/// Send a response, then wait for the peer to close the connection.
pub async fn respond(tcp: &mut Tcp, status: Status, body: &str) -> Result<(), Error> {
    let mut head: heapless::String<160> = heapless::String::new();
    let written = write!(
        head,
        "HTTP/1.1 {status}\r\n\
         Content-Type: {CONTENT_TYPE}\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n",
        body.len()
    );
    written.map_err(|_| Error::Malformed)?;
    send(tcp, &head, body).await?;
    tcp.finish().await?;

    // The peer closes its side once it has the whole response. Anything it
    // sends before that is discarded, and a receive error ends the wait.
    while matches!(tcp.receive().await, Ok(chunk) if !chunk.is_empty()) {}
    Ok(())
}

/// Send `body` to `to` in a `PUT` request for `path`. Returns the status code
/// of the response.
pub async fn put(tcp: &mut Tcp, to: SocketAddrV6, path: &str, body: &str) -> Result<u16, Error> {
    let mut head: heapless::String<256> = heapless::String::new();
    let written = write!(
        head,
        "PUT {path} HTTP/1.1\r\n\
         Host: {to}\r\n\
         Content-Type: {CONTENT_TYPE}\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n",
        body.len()
    );
    written.map_err(|_| Error::Malformed)?;

    tcp.connect(to).await?;
    send(tcp, &head, body).await?;

    // "HTTP/1.1 200 OK": the status is the second word of the first line.
    let mut answer = Head::new();
    while !answer.first_line_done {
        let chunk = tcp.receive().await?;
        if chunk.is_empty() || answer.len > MAX_HEAD {
            return Err(Error::Malformed);
        }
        answer.take(&chunk);
    }
    let status = answer.first_line.split(' ').nth(1);
    status
        .and_then(|status| status.parse().ok())
        .ok_or(Error::Malformed)
}

/// Send a head followed by a body, in as few chunks as possible.
async fn send(tcp: &mut Tcp, head: &str, body: &str) -> Result<(), TcpError> {
    // Neither `extend_from_slice` can fail: the heads are written into
    // strings shorter than a chunk, and only as much of the body is added as
    // fits after the head.
    let mut chunk = TcpChunk::new();
    let _ = chunk.extend_from_slice(head.as_bytes());
    let with_head = body.len().min(chunk.capacity() - chunk.len());
    let (first, rest) = body.as_bytes().split_at(with_head);
    let _ = chunk.extend_from_slice(first);

    tcp.send(&chunk).await?;
    tcp.send(rest).await
}
