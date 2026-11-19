use std::io;

use crate::proto::Descriptor;

#[derive(Debug, thiserror::Error)]
pub enum RplidarError {
    #[error("bad TCP address {0:?}: expected tcp:<host>:<port>")]
    BadAddress(String),
    #[error("connect {addr}: {source}")]
    Connect { addr: String, source: io::Error },
    #[error("open serial port {path}: {source}")]
    Serial {
        path: String,
        source: serialport::Error,
    },
    #[error("{0}")]
    Io(#[from] io::Error),
    #[error("no answer to {cmd} within {ms} ms")]
    Timeout { cmd: &'static str, ms: u64 },
    #[error("{cmd} answered with descriptor {got:?}, expected {want:?}")]
    Descriptor {
        cmd: &'static str,
        got: Descriptor,
        want: Descriptor,
    },
    #[error("device health is error, code {0:#06x}")]
    Health(u16),
    #[error("no RPLIDAR answered on a 10c4:ea60 USB serial port")]
    NotFound,
    #[error("device \"auto\" needs /dev/serial/by-id (Linux); set the serial port explicitly")]
    AutoUnsupported,
}
