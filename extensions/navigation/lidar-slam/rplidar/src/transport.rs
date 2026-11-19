//! Byte links to a device and the request/response exchanges over them.
//!
//! A device path `tcp:<host>:<port>` is a TCP stream (a simulated device);
//! anything else is a serial port opened 8N1 with no flow control and DTR
//! held low.

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::thread;
use std::time::{Duration, Instant};

use crate::error::RplidarError;
use crate::proto::{self, Descriptor, DeviceInfo, Health, DESCRIPTOR_LEN, HEALTH_LEN, INFO_LEN};

/// Prefix selecting a TCP link.
pub const TCP_PREFIX: &str = "tcp:";
/// How long a TCP connect may take.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
/// Quiet time after `STOP` before stale input is discarded.
const STOP_SETTLE: Duration = Duration::from_millis(10);
/// Read timeout while draining.
const DRAIN_POLL: Duration = Duration::from_millis(2);
/// Bytes a drain may discard before giving up on the line going quiet.
const DRAIN_LIMIT: usize = 64 * 1024;

/// A full-duplex byte link to a device.
pub trait Link: Read + Write + Send {
    /// Longest a `read` blocks before returning `TimedOut` / `WouldBlock`.
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    /// Discard bytes received but not yet read.
    fn discard_input(&mut self) -> io::Result<()>;
}

struct Tcp {
    stream: TcpStream,
    timeout: Duration,
}

impl Read for Tcp {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.stream.read(buf)
    }
}

impl Write for Tcp {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.stream.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

impl Link for Tcp {
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.timeout = timeout;
        self.stream.set_read_timeout(Some(timeout))
    }
    fn discard_input(&mut self) -> io::Result<()> {
        self.stream.set_read_timeout(Some(DRAIN_POLL))?;
        let drained = drain(&mut self.stream);
        self.stream.set_read_timeout(Some(self.timeout))?;
        drained
    }
}

struct Serial {
    port: Box<dyn serialport::SerialPort>,
    timeout: Duration,
}

impl Read for Serial {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.port.read(buf)
    }
}

impl Write for Serial {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.port.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.port.flush()
    }
}

impl Link for Serial {
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.timeout = timeout;
        self.port.set_timeout(timeout).map_err(io::Error::from)
    }
    fn discard_input(&mut self) -> io::Result<()> {
        self.port
            .clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::from)?;
        self.port.set_timeout(DRAIN_POLL).map_err(io::Error::from)?;
        let drained = drain(&mut self.port);
        self.port
            .set_timeout(self.timeout)
            .map_err(io::Error::from)?;
        drained
    }
}

/// Read until a read times out (the line went quiet).
fn drain(r: &mut impl Read) -> io::Result<()> {
    let mut buf = [0u8; 512];
    let mut total = 0;
    while total < DRAIN_LIMIT {
        match r.read(&mut buf) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(n) => total += n,
            Err(e) if is_timeout(&e) => return Ok(()),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

/// A read that ran out of time rather than failed.
pub fn is_timeout(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
    )
}

/// Open `path` (`tcp:<host>:<port>` or a serial port) at `baud`.
pub fn open(path: &str, baud: u32) -> Result<Box<dyn Link>, RplidarError> {
    let read_timeout = Duration::from_millis(100);
    if let Some(addr) = path.strip_prefix(TCP_PREFIX) {
        let stream = connect(addr)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(read_timeout))?;
        return Ok(Box::new(Tcp {
            stream,
            timeout: read_timeout,
        }));
    }
    let serial_err = |source| RplidarError::Serial {
        path: path.to_string(),
        source,
    };
    let mut port = serialport::new(path, baud)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(read_timeout)
        .open()
        .map_err(serial_err)?;
    port.write_data_terminal_ready(false).map_err(serial_err)?;
    Ok(Box::new(Serial {
        port,
        timeout: read_timeout,
    }))
}

fn connect(addr: &str) -> Result<TcpStream, RplidarError> {
    let bad = || RplidarError::BadAddress(format!("{TCP_PREFIX}{addr}"));
    let (host, port) = addr.rsplit_once(':').ok_or_else(bad)?;
    if host.is_empty() || port.parse::<u16>().is_err() {
        return Err(bad());
    }
    let connect_err = |source| RplidarError::Connect {
        addr: addr.to_string(),
        source,
    };
    let mut last = io::Error::new(io::ErrorKind::NotFound, "address resolved to nothing");
    for sa in addr.to_socket_addrs().map_err(connect_err)? {
        match TcpStream::connect_timeout(&sa, CONNECT_TIMEOUT) {
            Ok(s) => return Ok(s),
            Err(e) => last = e,
        }
    }
    Err(connect_err(last))
}

/// Send a payload-less request.
pub fn send(link: &mut dyn Link, request: [u8; 2]) -> io::Result<()> {
    link.write_all(&request)?;
    link.flush()
}

/// `STOP`, let the line settle, then discard whatever was in flight.
pub fn stop_and_drain(link: &mut dyn Link) -> io::Result<()> {
    send(link, proto::stop())?;
    thread::sleep(STOP_SETTLE);
    link.discard_input()
}

/// `GET_INFO` round trip.
pub fn query_info(link: &mut dyn Link, timeout: Duration) -> Result<DeviceInfo, RplidarError> {
    send(link, proto::get_info())?;
    let b: [u8; INFO_LEN] = read_response(link, "GET_INFO", proto::INFO_DESCRIPTOR, timeout)?;
    Ok(proto::parse_info(&b))
}

/// `GET_HEALTH` round trip.
pub fn query_health(link: &mut dyn Link, timeout: Duration) -> Result<Health, RplidarError> {
    send(link, proto::get_health())?;
    let b: [u8; HEALTH_LEN] = read_response(link, "GET_HEALTH", proto::HEALTH_DESCRIPTOR, timeout)?;
    Ok(proto::parse_health(&b))
}

/// `SCAN` and its descriptor; standard nodes follow on the link.
pub fn start_scan(link: &mut dyn Link, timeout: Duration) -> Result<(), RplidarError> {
    send(link, proto::scan())?;
    let deadline = Instant::now() + timeout;
    read_descriptor(link, "SCAN", proto::SCAN_DESCRIPTOR, deadline, timeout)
}

fn read_response<const N: usize>(
    link: &mut dyn Link,
    cmd: &'static str,
    want: Descriptor,
    timeout: Duration,
) -> Result<[u8; N], RplidarError> {
    let deadline = Instant::now() + timeout;
    read_descriptor(link, cmd, want, deadline, timeout)?;
    let mut payload = [0u8; N];
    read_full(link, &mut payload, deadline, cmd, timeout)?;
    Ok(payload)
}

/// Skip to the `A5 5A` sync, then check the descriptor is `want`.
fn read_descriptor(
    link: &mut dyn Link,
    cmd: &'static str,
    want: Descriptor,
    deadline: Instant,
    timeout: Duration,
) -> Result<(), RplidarError> {
    let mut d = [0u8; DESCRIPTOR_LEN];
    let mut prev = 0u8;
    loop {
        let mut b = [0u8; 1];
        read_full(link, &mut b, deadline, cmd, timeout)?;
        if prev == proto::SYNC && b[0] == proto::SYNC2 {
            break;
        }
        prev = b[0];
    }
    d[0] = proto::SYNC;
    d[1] = proto::SYNC2;
    read_full(link, &mut d[2..], deadline, cmd, timeout)?;
    let got = proto::parse_descriptor(&d).expect("sync bytes checked");
    if got != want {
        return Err(RplidarError::Descriptor { cmd, got, want });
    }
    Ok(())
}

fn read_full(
    link: &mut dyn Link,
    buf: &mut [u8],
    deadline: Instant,
    cmd: &'static str,
    timeout: Duration,
) -> Result<(), RplidarError> {
    let mut got = 0;
    while got < buf.len() {
        if Instant::now() >= deadline {
            return Err(RplidarError::Timeout {
                cmd,
                ms: timeout.as_millis() as u64,
            });
        }
        match link.read(&mut buf[got..]) {
            Ok(0) => return Err(io::Error::from(io::ErrorKind::UnexpectedEof).into()),
            Ok(n) => got += n,
            Err(e) if is_timeout(&e) || e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e.into()),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malformed_tcp_addresses_are_refused() {
        for bad in ["tcp:", "tcp:localhost", "tcp::7700", "tcp:host:notaport"] {
            assert!(
                matches!(open(bad, 460_800), Err(RplidarError::BadAddress(_))),
                "{bad}"
            );
        }
    }
}
