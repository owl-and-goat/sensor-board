//! HTTP for Prometheus over the Thread TCP connection: a server for scrape
//! requests, and a client for `PUT` requests to a Pushgateway. nanofish
//! implements HTTP/1.1, and this module runs it on a [`Tcp`] connection.
//! Every request uses its own connection, which is closed after the response.

use core::fmt::Write as _;
use core::net::SocketAddrV6;

use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use nanofish::{
    HttpClient, HttpClientOptions, HttpClientRequest, HttpHandler, HttpHeader, HttpMethod,
    HttpResponse, HttpServer, ResponseBody, StatusCode,
};

use crate::thread::{Tcp, TcpChunk, TcpError};

/// The `Content-Type` of Prometheus's text format, for responses and pushes.
const CONTENT_TYPE: &str = "text/plain; version=0.0.4; charset=utf-8";

/// The longest body of a response. nanofish copies a response into one
/// buffer before it sends it.
pub const MAX_BODY: usize = 4096;

/// Room for the head of a message that is sent: the first line and the
/// headers.
const MAX_HEAD: usize = 256;

/// The longest request that the server reads.
const MAX_REQUEST: usize = 2048;

/// The longest response that the client reads. nanofish reads the whole
/// response before it returns the status, and the Pushgateway explains a
/// rejected push in the body. A longer response counts as malformed.
const MAX_RESPONSE: usize = 4096;

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

/// A [`Tcp`] connection as the byte stream that nanofish reads and writes.
struct Stream<'a> {
    tcp: &'a mut Tcp,
    /// The chunk that was received last.
    chunk: TcpChunk,
    /// How many bytes of `chunk` have been read.
    read: usize,
    /// The error that a read or a write failed with.
    error: Option<TcpError>,
}

impl<'a> Stream<'a> {
    fn new(tcp: &'a mut Tcp) -> Self {
        Stream {
            tcp,
            chunk: TcpChunk::new(),
            read: 0,
            error: None,
        }
    }

    /// Keep `error` for [`Stream::outcome`].
    fn fail(&mut self, error: TcpError) -> ErrorKind {
        self.error = Some(error);
        ErrorKind::Other
    }

    /// The outcome of the exchange that nanofish ran on this stream. nanofish
    /// does not pass on a stream's error, so it is taken from the stream.
    fn outcome<T>(&self, exchange: Result<T, nanofish::Error>) -> Result<T, Error> {
        exchange.map_err(|_| self.error.map_or(Error::Malformed, Error::Tcp))
    }
}

impl ErrorType for Stream<'_> {
    type Error = ErrorKind;
}

impl Read for Stream<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        if self.read == self.chunk.len() {
            let received = self.tcp.receive().await;
            self.chunk = received.map_err(|e| self.fail(e))?;
            self.read = 0;
        }
        // An empty chunk means the peer has closed its side, and so does a
        // read of no bytes.
        let unread = &self.chunk[self.read..];
        let len = unread.len().min(buf.len());
        buf[..len].copy_from_slice(&unread[..len]);
        self.read += len;
        Ok(len)
    }
}

impl Write for Stream<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        let sent = self.tcp.send(buf).await;
        sent.map_err(|e| self.fail(e))?;
        Ok(buf.len())
    }

    /// Nothing to do: each write has already passed its data to the TCP
    /// stack, which sends it without being asked again.
    async fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

/// A response with `body` in Prometheus's text format.
pub fn response(status: StatusCode, body: &str) -> HttpResponse<'_> {
    let mut headers = heapless::Vec::new();
    // Neither `push` can fail: a response has room for more headers.
    let _ = headers.push(HttpHeader::content_type(CONTENT_TYPE));
    let _ = headers.push(HttpHeader::connection("close"));
    HttpResponse {
        status_code: status,
        headers,
        body: ResponseBody::Text(body),
    }
}

/// Read the request on an accepted connection and send the response that
/// `handler` returns for it. Then wait for the peer to close the connection.
pub async fn serve(tcp: &mut Tcp, handler: &mut impl HttpHandler) -> Result<(), Error> {
    let server = HttpServer::<MAX_REQUEST, { MAX_HEAD + MAX_BODY }>::new();
    let mut stream = Stream::new(tcp);
    let served = server.handle_connection(&mut stream, handler).await;
    stream.outcome(served)?;
    tcp.finish().await?;

    // The peer closes its side once it has the whole response. Anything it
    // sends before that is discarded, and a receive error ends the wait.
    while matches!(tcp.receive().await, Ok(chunk) if !chunk.is_empty()) {}
    Ok(())
}

/// Send `body` to `to` in a `PUT` request for `path`. Returns the status code
/// of the response.
pub async fn put(tcp: &mut Tcp, to: SocketAddrV6, path: &str, body: &str) -> Result<u16, Error> {
    let mut host: heapless::String<64> = heapless::String::new();
    write!(host, "{to}").map_err(|_| Error::Malformed)?;
    let request = HttpClientRequest {
        method: HttpMethod::PUT,
        host: &host,
        path,
        headers: &[HttpHeader::content_type(CONTENT_TYPE)],
        body: Some(body.as_bytes()),
    };
    // A failed read is not tried again: `Tcp` has already waited for the
    // peer.
    let client = HttpClient::<MAX_HEAD>::with_options(HttpClientOptions {
        max_retries: 1,
        ..HttpClientOptions::default()
    });

    tcp.connect(to).await?;
    let mut stream = Stream::new(tcp);
    let mut buffer = [0; MAX_RESPONSE];
    let exchange = client.request(&mut stream, request, &mut buffer).await;
    let (response, _) = stream.outcome(exchange)?;
    Ok(response.status_code.as_u16())
}
