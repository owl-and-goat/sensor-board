//! Just enough HTTP for Prometheus: the request it scrapes a board with, and
//! the one a board pushes to a Pushgateway with. Each has a connection to
//! itself, which is closed once it has been answered.

use core::fmt::{self, Write as _};
use core::net::SocketAddrV6;

use crate::thread::{Tcp, TcpChunk, TcpError};

/// What both sides say the body is: Prometheus's text format.
const CONTENT_TYPE: &str = "text/plain; version=0.0.4; charset=utf-8";

/// The most that is read of a request before its end has to have come, and
/// of an answer before its first line has to be complete.
const MAX_HEAD: usize = 2048;

#[derive(Clone, Copy, defmt::Format)]
pub enum Error {
    Tcp(TcpError),
    /// What arrived is not HTTP, or goes on for longer than it may.
    Malformed,
}

impl From<TcpError> for Error {
    fn from(e: TcpError) -> Self {
        Error::Tcp(e)
    }
}

/// What a request asks for.
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

/// The first line of what arrives on a connection, and whether the blank
/// line that ends a request's or an answer's head has arrived.
struct Head {
    first_line: heapless::String<64>,
    first_line_done: bool,
    /// How much of the `\r\n\r\n` that ends the head has just arrived.
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

    /// Take in more of what arrives.
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
                // A line that is cut short is no line this understands.
                let _ = self.first_line.push(char::from(byte));
            }
        }
    }
}

/// Take in the request that the connection which has come in brings.
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

/// Answer the request, and wait for the peer to be done with the
/// connection.
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

    // The peer has it all once it closes its side too. Whatever it sends
    // until then is of no interest, and neither is how the connection ends.
    while matches!(tcp.receive().await, Ok(chunk) if !chunk.is_empty()) {}
    Ok(())
}

/// Send `body` to `to` in a PUT request for `path`, and answer with the
/// status that comes back.
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

/// Send a head and the body after it, in as few pieces as they fit in.
async fn send(tcp: &mut Tcp, head: &str, body: &str) -> Result<(), TcpError> {
    // Neither can fail: a head is shorter than a chunk, going by the room it
    // is written in, and of the body only what fits is added.
    let mut chunk = TcpChunk::new();
    let _ = chunk.extend_from_slice(head.as_bytes());
    let with_head = body.len().min(chunk.capacity() - chunk.len());
    let (first, rest) = body.as_bytes().split_at(with_head);
    let _ = chunk.extend_from_slice(first);

    tcp.send(&chunk).await?;
    tcp.send(rest).await
}
