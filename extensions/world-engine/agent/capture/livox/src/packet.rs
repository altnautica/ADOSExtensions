//! One Livox SDK2 point-cloud or IMU data packet.
//!
//! Layout source: Livox-SDK2 `include/livox_lidar_def.h` (tag v1.2.5),
//! `LivoxLidarEthernetPacket` under `#pragma pack(1)`, and the Livox wiki
//! "Livox LiDAR Communication Protocol -- Mid360", section 2 "Point Cloud &
//! IMU Data Protocol". Every multi-byte field is little-endian.
//!
//! | offset | size | field                                                    |
//! |-------:|-----:|----------------------------------------------------------|
//! |      0 |    1 | `version` (currently 0)                                  |
//! |      1 |    2 | `length`: whole UDP data segment, from `version` on      |
//! |      3 |    2 | `time_interval`, 0.1 us: last point time - first point   |
//! |      5 |    2 | `dot_num`: samples in `data`                             |
//! |      7 |    2 | `udp_cnt`: packet count, 0 at the start of a frame       |
//! |      9 |    1 | `frame_cnt` (invalid for non-repetitive scanning)        |
//! |     10 |    1 | `data_type` ([`DataType`])                               |
//! |     11 |    1 | `time_type` ([`TimeType`])                               |
//! |     12 |   12 | reserved                                                 |
//! |     24 |    4 | `crc32` over `timestamp` + `data` ([`crate::crc`])       |
//! |     28 |    8 | `timestamp`, ns, time of the first sample                |
//! |     36 |    - | `data`: `dot_num` samples                                |
//!
//! Sample layouts (`livox_lidar_def.h`):
//! - `LivoxLidarImuRawPoint` (type 0): gyro x/y/z rad/s, acc x/y/z in g, six
//!   `float`s, 24 bytes.
//! - `LivoxLidarCartesianHighRawPoint` (type 1): `int32` x/y/z mm,
//!   reflectivity, tag; 14 bytes.
//! - `LivoxLidarCartesianLowRawPoint` (type 2): `int16` x/y/z cm,
//!   reflectivity, tag; 8 bytes.
//! - `LivoxLidarSpherPoint` (type 3): `uint32` depth mm, `uint16` theta
//!   (zenith, 0.01 deg), `uint16` phi (azimuth, 0.01 deg), reflectivity, tag;
//!   10 bytes.
//!
//! The wiki states the samples of a packet are equally spaced over
//! `time_interval`, the first at `timestamp` and the last at
//! `timestamp + time_interval`.

use crate::crc::crc32;

/// Bytes before `data`.
pub const HEADER_LEN: usize = 36;
/// Offset of `crc32`.
const CRC_FIELD: usize = 24;
/// Offset of `timestamp`, where the CRC-covered region starts.
const CRC_START: usize = 28;

/// Standard gravity, for the IMU's accelerations in g.
pub const STANDARD_GRAVITY: f32 = 9.806_65;

/// `LivoxLidarPointDataType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Imu,
    CartesianHigh,
    CartesianLow,
    Spherical,
}

impl DataType {
    pub fn from_wire(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Imu),
            0x01 => Some(Self::CartesianHigh),
            0x02 => Some(Self::CartesianLow),
            0x03 => Some(Self::Spherical),
            _ => None,
        }
    }

    /// Bytes per sample.
    pub fn sample_len(self) -> usize {
        match self {
            Self::Imu => 24,
            Self::CartesianHigh => 14,
            Self::CartesianLow => 8,
            Self::Spherical => 10,
        }
    }
}

/// The packet `time_type` (protocol section 2.2 "Timestamp"). Every type is a
/// 64-bit nanosecond count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeType {
    /// 0: no sync source, time since the LiDAR powered on.
    PowerOn,
    /// 1: gPTP/PTP master clock time.
    Ptp,
    /// 2: GPS time.
    Gps,
    /// A value the protocol does not define.
    Other(u8),
}

impl TimeType {
    pub fn from_wire(v: u8) -> Self {
        match v {
            0 => Self::PowerOn,
            1 => Self::Ptp,
            2 => Self::Gps,
            other => Self::Other(other),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PacketError {
    #[error("packet is {0} bytes, shorter than the {HEADER_LEN}-byte header")]
    Short(usize),
    #[error("packet declares {declared} bytes but {actual} are available")]
    Truncated { declared: usize, actual: usize },
    #[error("unknown data_type {0:#04x}")]
    UnknownDataType(u8),
    #[error("{dot_num} samples need {need} data bytes, packet carries {have}")]
    DataTooShort {
        dot_num: u16,
        need: usize,
        have: usize,
    },
}

/// Result of checking the `crc32` field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcStatus {
    /// The field is zero: the sender did not fill it.
    Absent,
    Valid,
    Mismatch {
        field: u32,
        computed: u32,
    },
}

/// The fixed header fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    pub version: u8,
    pub length: u16,
    /// Last sample time minus first sample time, 0.1 us units.
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: DataType,
    pub time_type: TimeType,
    pub crc32: u32,
    /// Device time of the first sample, ns.
    pub timestamp_ns: u64,
}

impl PacketHeader {
    /// `time_interval` in nanoseconds.
    pub fn interval_ns(&self) -> u64 {
        u64::from(self.time_interval) * 100
    }

    /// Device time of sample `i`, ns after `timestamp_ns`.
    pub fn sample_offset_ns(&self, i: usize) -> u64 {
        if self.dot_num <= 1 {
            return 0;
        }
        i as u64 * self.interval_ns() / u64::from(self.dot_num - 1)
    }

    /// Device time of the last sample, ns.
    pub fn last_sample_ns(&self) -> u64 {
        self.timestamp_ns.saturating_add(self.interval_ns())
    }
}

/// A validated packet borrowing the buffer it was parsed from.
#[derive(Debug, Clone, Copy)]
pub struct Packet<'a> {
    pub header: PacketHeader,
    /// The `length` bytes of the packet (a receive buffer may be longer).
    bytes: &'a [u8],
}

/// One point, sensor frame (the LiDAR's own x forward, y left, z up).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawPoint {
    /// Metres.
    pub xyz: [f32; 3],
    pub reflectivity: u8,
    pub tag: u8,
    /// Device time of the point minus the packet `timestamp_ns`.
    pub offset_ns: u64,
    /// False for a sample with no return (zero range).
    pub valid: bool,
}

/// One IMU sample, sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawImu {
    /// rad/s.
    pub gyro: [f32; 3],
    /// m/s² (the wire carries g; converted with [`STANDARD_GRAVITY`]).
    pub accel: [f32; 3],
    /// Device time minus the packet `timestamp_ns`.
    pub offset_ns: u64,
}

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}

fn i16_at(b: &[u8], o: usize) -> i16 {
    i16::from_le_bytes([b[o], b[o + 1]])
}

fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

fn i32_at(b: &[u8], o: usize) -> i32 {
    u32_at(b, o) as i32
}

fn f32_at(b: &[u8], o: usize) -> f32 {
    f32::from_bits(u32_at(b, o))
}

fn u64_at(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}

/// The `length` field of a buffer holding at least the first three bytes.
pub fn declared_len(buf: &[u8]) -> Option<usize> {
    (buf.len() >= 3).then(|| usize::from(u16_at(buf, 1)))
}

impl<'a> Packet<'a> {
    /// Parse and bounds-check a packet. The CRC is not checked here; see
    /// [`Packet::crc_status`].
    pub fn parse(buf: &'a [u8]) -> Result<Self, PacketError> {
        if buf.len() < HEADER_LEN {
            return Err(PacketError::Short(buf.len()));
        }
        let length = u16_at(buf, 1);
        let declared = usize::from(length);
        if declared < HEADER_LEN {
            return Err(PacketError::Short(declared));
        }
        if declared > buf.len() {
            return Err(PacketError::Truncated {
                declared,
                actual: buf.len(),
            });
        }
        let data_type =
            DataType::from_wire(buf[10]).ok_or(PacketError::UnknownDataType(buf[10]))?;
        let header = PacketHeader {
            version: buf[0],
            length,
            time_interval: u16_at(buf, 3),
            dot_num: u16_at(buf, 5),
            udp_cnt: u16_at(buf, 7),
            frame_cnt: buf[9],
            data_type,
            time_type: TimeType::from_wire(buf[11]),
            crc32: u32_at(buf, CRC_FIELD),
            timestamp_ns: u64_at(buf, CRC_START),
        };
        let need = usize::from(header.dot_num) * data_type.sample_len();
        let have = declared - HEADER_LEN;
        if need > have {
            return Err(PacketError::DataTooShort {
                dot_num: header.dot_num,
                need,
                have,
            });
        }
        Ok(Self {
            header,
            bytes: &buf[..declared],
        })
    }

    pub fn crc_status(&self) -> CrcStatus {
        if self.header.crc32 == 0 {
            return CrcStatus::Absent;
        }
        let computed = crc32(&self.bytes[CRC_START..]);
        if computed == self.header.crc32 {
            CrcStatus::Valid
        } else {
            CrcStatus::Mismatch {
                field: self.header.crc32,
                computed,
            }
        }
    }

    fn sample(&self, i: usize) -> &'a [u8] {
        let n = self.header.data_type.sample_len();
        let start = HEADER_LEN + i * n;
        &self.bytes[start..start + n]
    }

    /// The points of a point-cloud packet (empty for an IMU packet).
    pub fn points(&self) -> impl Iterator<Item = RawPoint> + '_ {
        let n = match self.header.data_type {
            DataType::Imu => 0,
            _ => usize::from(self.header.dot_num),
        };
        (0..n).map(move |i| self.point(i))
    }

    fn point(&self, i: usize) -> RawPoint {
        let s = self.sample(i);
        let offset_ns = self.header.sample_offset_ns(i);
        let (xyz, valid, refl_at) = match self.header.data_type {
            DataType::CartesianHigh => {
                let (x, y, z) = (i32_at(s, 0), i32_at(s, 4), i32_at(s, 8));
                let m = |v: i32| v as f32 * 1e-3;
                ([m(x), m(y), m(z)], (x, y, z) != (0, 0, 0), 12)
            }
            DataType::CartesianLow => {
                let (x, y, z) = (i16_at(s, 0), i16_at(s, 2), i16_at(s, 4));
                let m = |v: i16| v as f32 * 1e-2;
                ([m(x), m(y), m(z)], (x, y, z) != (0, 0, 0), 6)
            }
            DataType::Spherical => {
                let depth = u32_at(s, 0);
                // theta is the zenith angle and phi the azimuth, both in
                // 0.01 degree; the conversion is the one livox_ros_driver
                // applies to this point type.
                let r = depth as f64 * 1e-3;
                let theta = (u16_at(s, 4) as f64 / 100.0).to_radians();
                let phi = (u16_at(s, 6) as f64 / 100.0).to_radians();
                let xyz = [
                    (r * theta.sin() * phi.cos()) as f32,
                    (r * theta.sin() * phi.sin()) as f32,
                    (r * theta.cos()) as f32,
                ];
                (xyz, depth != 0, 8)
            }
            DataType::Imu => unreachable!("points() yields nothing for IMU packets"),
        };
        RawPoint {
            xyz,
            reflectivity: s[refl_at],
            tag: s[refl_at + 1],
            offset_ns,
            valid,
        }
    }

    /// The samples of an IMU packet (empty for a point-cloud packet).
    pub fn imu(&self) -> impl Iterator<Item = RawImu> + '_ {
        let n = match self.header.data_type {
            DataType::Imu => usize::from(self.header.dot_num),
            _ => 0,
        };
        (0..n).map(move |i| {
            let s = self.sample(i);
            let g = STANDARD_GRAVITY;
            RawImu {
                gyro: [f32_at(s, 0), f32_at(s, 4), f32_at(s, 8)],
                accel: [f32_at(s, 12) * g, f32_at(s, 16) * g, f32_at(s, 20) * g],
                offset_ns: self.header.sample_offset_ns(i),
            }
        })
    }
}
