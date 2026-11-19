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

/// One whole scan out of a [`ScanAssembler`].
#[derive(Debug, Clone, PartialEq)]
pub struct AssembledScan {
    pub sensor_id: String,
    pub t_ns: u64,
    pub seq: u64,
    /// Packed points ([`POINT_STRIDE`] bytes each), parts in index order.
    pub points: Vec<u8>,
}

/// A scan with some parts in.
#[derive(Debug)]
struct Partial {
    t_ns: u64,
    first_seen_ns: u64,
    parts: Vec<Option<Vec<u8>>>,
    have: usize,
}

/// Reassembles parts into whole scans, keyed by `(sensor_id, seq)`, in any
/// arrival order. A scan whose parts do not all arrive within the assembler's
/// window of its first part is dropped and counted ([`Self::incomplete`]).
#[derive(Debug)]
pub struct ScanAssembler {
    window_ns: u64,
    open: std::collections::HashMap<(String, u64), Partial>,
    incomplete: u64,
}

impl ScanAssembler {
    /// An assembler that gives a scan `window_ns` from its first part.
    pub fn new(window_ns: u64) -> Self {
        Self {
            window_ns,
            open: Default::default(),
            incomplete: 0,
        }
    }

    /// Feed one part received at `now_ns`; returns the scan it completes.
    /// Expired scans are dropped first. A part whose count disagrees with
    /// the scan's, or that is out of range or repeated, is ignored.
    pub fn push(&mut self, c: ScanChunk, now_ns: u64) -> Option<AssembledScan> {
        self.expire(now_ns);
        let parts = usize::from(c.parts);
        let part = usize::from(c.part);
        if part >= parts {
            return None;
        }
        let key = (c.sensor_id, c.seq);
        let p = self.open.entry(key.clone()).or_insert_with(|| Partial {
            t_ns: c.t_ns,
            first_seen_ns: now_ns,
            parts: vec![None; parts],
            have: 0,
        });
        if p.parts.len() != parts || p.parts[part].is_some() {
            return None;
        }
        p.parts[part] = Some(c.points);
        p.have += 1;
        if p.have < parts {
            return None;
        }
        let p = self.open.remove(&key)?;
        Some(AssembledScan {
            sensor_id: key.0,
            t_ns: p.t_ns,
            seq: key.1,
            points: p.parts.into_iter().flatten().flatten().collect(),
        })
    }

    /// Drop every scan whose window has passed at `now_ns`.
    pub fn expire(&mut self, now_ns: u64) {
        let window = self.window_ns;
        let before = self.open.len();
        self.open
            .retain(|_, p| now_ns.saturating_sub(p.first_seen_ns) <= window);
        self.incomplete += (before - self.open.len()) as u64;
    }

    /// Scans dropped because a part did not arrive in time.
    pub fn incomplete(&self) -> u64 {
        self.incomplete
    }
}
