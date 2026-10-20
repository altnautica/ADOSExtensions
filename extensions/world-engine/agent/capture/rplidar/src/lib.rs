//! RPLIDAR 2D LiDARs (the RPLIDAR C1 and other standard-scan models) over
//! their serial protocol, as timed 360-degree revolutions in the sensor FLU
//! frame.
//!
//! - [`proto`]: request encoders, response descriptors, the info and health
//!   payloads, the 5-byte standard scan node, and the device-angle (clockwise
//!   from forward) to FLU-azimuth (counter-clockwise from +x) mirror.
//! - [`transport`]: a serial port (8N1, DTR low) or `tcp:<host>:<port>` link,
//!   and the request/response exchanges over it.
//! - [`discover`]: `device = "auto"`, USB serial bridges `10c4:ea60` probed
//!   with `GET_INFO` (Linux).
//! - [`revolution`]: received bytes to revolutions, receive-time stamped and
//!   fitted to the sampling rate.
//! - [`device`]: the handshake, reader thread and reconnection.
//!
//! Protocol reference: Slamtec "RPLIDAR S&C Series Interface Protocol"
//! (LR001).

pub mod clock;
pub mod device;
pub mod discover;
pub mod error;
pub mod proto;
pub mod revolution;
pub mod transport;

pub use device::{Rplidar, RplidarConfig, RplidarStats, AUTO_DEVICE, DEFAULT_BAUD};
pub use error::RplidarError;
pub use proto::{flu_from_device_deg, DeviceInfo, Health, HealthStatus, Node};
pub use revolution::{RawScan, RevolutionAssembler};
