//! Livox SDK2 data packets to timed LiDAR scans and IMU samples.
//!
//! - [`packet`]: the `LivoxLidarEthernetPacket` layout and its point and IMU
//!   sample types (cartesian 32-bit mm, cartesian 16-bit cm, spherical, IMU).
//! - [`crc`]: the packet CRC-32.
//! - [`clock`]: device time to host `CLOCK_MONOTONIC` with a minimum-latency
//!   offset filter.
//! - [`scan`]: points cut into fixed-period scans (10 Hz by default).
//! - [`decoder`]: the per-device pipeline joining the above.
//! - [`sdk_config`]: the JSON file the SDK is initialised from.
//! - `sdk` (feature `sdk`, Unix): the Livox SDK2 shared library loaded at run
//!   time, its packet callbacks fed through the decoder.
//!
//! Protocol references: Livox-SDK2 v1.2.5 `include/livox_lidar_def.h` and the
//! Livox wiki "Livox LiDAR Communication Protocol -- Mid360".

pub mod clock;
pub mod crc;
pub mod decoder;
pub mod packet;
pub mod scan;
pub mod sdk_config;

#[cfg(all(feature = "sdk", unix))]
pub mod sdk;

pub use clock::{ClockConfig, ClockMapper};
pub use decoder::{DecodeError, Decoder, DecoderConfig, DecoderStats, Event, ImuSample};
pub use packet::{CrcStatus, DataType, Packet, PacketError, PacketHeader, TimeType};
pub use scan::{Scan, ScanAssembler, DEFAULT_PERIOD_NS};
pub use sdk_config::{Ports, SdkNetConfig};

#[cfg(all(feature = "sdk", unix))]
pub use sdk::{DeviceEvent, SdkDriver, SdkDriverConfig, SdkError, SdkStats};

/// Livox-SDK2 release the `sdk` feature is written against and
/// `build-sdk2.sh` builds.
pub const SDK2_TAG: &str = "v1.2.5";
