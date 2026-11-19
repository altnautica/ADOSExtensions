//! What Navigation streams for other processes besides its flight topics: a
//! LiDAR scan as it was measured, in parts that fit one host event
//! ([`ScanChunk`]), and every pose estimate in batches ([`PoseTrackBatch`]).
//!
//! The host refuses an event payload over 64 KiB, and a 3D LiDAR scan is far
//! larger (20,000 points at 17 bytes is 340 KB), so a scan is cut into parts
//! of at most [`SCAN_CHUNK_POINTS`] points (59.5 KB of points). A planar scan
//! is one part.
//!
//! Points are packed with no padding, [`POINT_STRIDE`] bytes each,
//! little-endian, the layout a capture session records them in:
//!
//! | name          | offset | type |
//! |---------------|-------:|------|
//! | `x`           |      0 | f32  |
//! | `y`           |      4 | f32  |
//! | `z`           |      8 | f32  |
//! | `intensity`   |     12 | u8   |
//! | `t_offset_ns` |     13 | u32  |

use serde::{Deserialize, Serialize};

use super::{PoseSource, VioHealth};

/// Bytes per packed point.
pub const POINT_STRIDE: usize = 17;

/// Most points one [`ScanChunk`] carries.
pub const SCAN_CHUNK_POINTS: usize = 3500;

/// One part of one LiDAR scan, in the sensor frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScanChunk {
    pub sensor_id: String,
    /// Host monotonic ns of the scan's first point (every part of a scan
    /// carries the same one).
    pub t_ns: u64,
    /// The scan's number, counting from the first scan the publisher saw.
    pub seq: u64,
    /// This part's index, `0..parts`.
    pub part: u16,
    pub parts: u16,
    /// Packed points ([`POINT_STRIDE`] bytes each).
    #[serde(with = "serde_bytes")]
    pub points: Vec<u8>,
}

/// One pose estimate as its provider produced it, in the provider's own world
/// frame (the one a session records poses in).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseTrackEntry {
    /// Host monotonic ns of the instant the estimate describes.
    pub t_ns: u64,
    /// World-from-body rotation, row-major.
    pub r: [f64; 9],
    /// Body origin in the world, metres.
    pub t: [f64; 3],
    /// 6x6 covariance, row-major (position then orientation).
    pub cov: Vec<f64>,
    pub health: VioHealth,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sigma_xy_m: Option<f64>,
    pub heading_ok: bool,
}

/// Every estimate a provider produced since the previous batch, oldest first.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseTrackBatch {
    /// The provider the estimates come from (what a session records as the
    /// pose source).
    pub source: PoseSource,
    pub estimates: Vec<PoseTrackEntry>,
}

impl ScanChunk {
    pub fn to_msgpack(&self) -> Result<Vec<u8>, rmp_serde::encode::Error> {
        rmp_serde::to_vec_named(self)
    }
    pub fn from_msgpack(bytes: &[u8]) -> Result<Self, rmp_serde::decode::Error> {
        rmp_serde::from_slice(bytes)
    }
    /// How many points this part carries.
    pub fn len(&self) -> usize {
        self.points.len() / POINT_STRIDE
    }
    pub fn is_empty(&self) -> bool {
        self.points.len() < POINT_STRIDE
    }
}

impl PoseTrackBatch {
    pub fn to_msgpack(&self) -> Result<Vec<u8>, rmp_serde::encode::Error> {
        rmp_serde::to_vec_named(self)
    }
    pub fn from_msgpack(bytes: &[u8]) -> Result<Self, rmp_serde::decode::Error> {
        rmp_serde::from_slice(bytes)
    }
}

/// Pack points (`xyz`, `intensity`, `t_offset_ns`); points past the shortest
/// of the three are dropped.
pub fn pack_points(points: &[[f32; 3]], intensity: &[u8], t_offset_ns: &[u32]) -> Vec<u8> {
    let n = points.len().min(intensity.len()).min(t_offset_ns.len());
    let mut out = Vec::with_capacity(n * POINT_STRIDE);
    for ((p, i), o) in points.iter().zip(intensity).zip(t_offset_ns).take(n) {
        out.extend_from_slice(&p[0].to_le_bytes());
        out.extend_from_slice(&p[1].to_le_bytes());
        out.extend_from_slice(&p[2].to_le_bytes());
        out.push(*i);
        out.extend_from_slice(&o.to_le_bytes());
    }
    out
}

/// The points, intensities and offsets of packed points; a trailing partial
/// point is ignored.
pub fn unpack_points(data: &[u8]) -> (Vec<[f32; 3]>, Vec<u8>, Vec<u32>) {
    let n = data.len() / POINT_STRIDE;
    let (mut pts, mut inten, mut off) = (
        Vec::with_capacity(n),
        Vec::with_capacity(n),
        Vec::with_capacity(n),
    );
    for c in data.as_chunks::<POINT_STRIDE>().0 {
        let f = |i: usize| f32::from_le_bytes([c[i], c[i + 1], c[i + 2], c[i + 3]]);
        pts.push([f(0), f(4), f(8)]);
        inten.push(c[12]);
        off.push(u32::from_le_bytes([c[13], c[14], c[15], c[16]]));
    }
    (pts, inten, off)
}

/// Cut one packed scan into parts of at most [`SCAN_CHUNK_POINTS`] points. An
/// empty scan is one empty part, so a subscriber still sees the scan.
pub fn chunk_scan(sensor_id: &str, t_ns: u64, seq: u64, packed: &[u8]) -> Vec<ScanChunk> {
    let per = SCAN_CHUNK_POINTS * POINT_STRIDE;
    let whole = packed.len() - packed.len() % POINT_STRIDE;
    let slices: Vec<&[u8]> = if whole == 0 {
        vec![&[][..]]
    } else {
        packed[..whole].chunks(per).collect()
    };
    let parts = slices.len() as u16;
    slices
        .into_iter()
        .enumerate()
        .map(|(i, s)| ScanChunk {
            sensor_id: sensor_id.to_string(),
            t_ns,
            seq,
            part: i as u16,
            parts,
            points: s.to_vec(),
        })
        .collect()
}

/// Reassembles parts into whole scans. Parts of one scan arrive in order on
/// one publisher; a part out of sequence drops the scan it would have
/// completed rather than join two scans.
#[derive(Debug, Default)]
pub struct ScanAssembler {
    current: Option<(String, u64, u16, u16, u64, Vec<u8>)>,
    dropped: u64,
}

impl ScanAssembler {
    /// Feed one part; returns `(sensor_id, t_ns, seq, packed points)` when it
    /// completes a scan.
    pub fn push(&mut self, c: ScanChunk) -> Option<(String, u64, u64, Vec<u8>)> {
        if c.part == 0 {
            if self.current.take().is_some() {
                self.dropped += 1;
            }
            self.current = Some((c.sensor_id, c.seq, 1, c.parts, c.t_ns, c.points));
        } else {
            match self.current.as_mut() {
                Some((sensor, seq, next, parts, _, buf))
                    if *sensor == c.sensor_id
                        && *seq == c.seq
                        && *next == c.part
                        && *parts == c.parts =>
                {
                    buf.extend_from_slice(&c.points);
                    *next += 1;
                }
                _ => {
                    if self.current.take().is_some() {
                        self.dropped += 1;
                    }
                    return None;
                }
            }
        }
        let (_, _, next, parts, _, _) = self.current.as_ref()?;
        if next < parts {
            return None;
        }
        let (sensor, seq, _, _, t_ns, buf) = self.current.take()?;
        Some((sensor, t_ns, seq, buf))
    }

    /// Scans dropped because a part was missing or out of order.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }
}
