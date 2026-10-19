//! Socket format between the capture service and `kiss_icp_live`
//! (`python/kiss_icp_live/kiss_icp_live/framing.py` is the other side).
//!
//! Capture -> child, one frame per scan, little-endian:
//!
//! ```text
//! u32  len            bytes after this field (= 12 + 16 * n)
//! u64  t_ns           host monotonic ns of the first point
//! u32  n              point count
//! f32  xyz[n][3]      points, metres, LiDAR frame
//! u32  t_offset[n]    per-point ns after t_ns
//! ```
//!
//! Child -> capture, one JSON object per line:
//! `{"t_ns": u64, "T_w_l": [16 f64, row-major], "eig": [3 f64, ascending], "n": u32}`.
//! `t_ns` is the scan's last point (KISS-ICP deskews to the scan end),
//! `T_w_l` the LiDAR pose in the odometry frame (the LiDAR frame at the first
//! scan), `eig` the translational ICP Hessian eigenvalues over `n`
//! correspondences.

use serde::Deserialize;

use crate::lidar_scan::LidarScan;

/// Bytes after the length field before the point data.
const HEAD_LEN: usize = 12;

/// Append the frame of `scan` to `out`; returns false (nothing written) for
/// a scan whose per-point vectors disagree in length.
pub fn encode_scan(scan: &LidarScan, out: &mut Vec<u8>) -> bool {
    let n = scan.points.len();
    if scan.t_offset_ns.len() != n || u32::try_from(n).is_err() {
        return false;
    }
    let body = HEAD_LEN + 16 * n;
    let Ok(len) = u32::try_from(body) else {
        return false;
    };
    out.reserve(4 + body);
    out.extend_from_slice(&len.to_le_bytes());
    out.extend_from_slice(&scan.t_ns.to_le_bytes());
    out.extend_from_slice(&(n as u32).to_le_bytes());
    for p in &scan.points {
        for c in p {
            out.extend_from_slice(&c.to_le_bytes());
        }
    }
    for o in &scan.t_offset_ns {
        out.extend_from_slice(&o.to_le_bytes());
    }
    true
}

/// One odometry result.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ChildPose {
    pub t_ns: u64,
    #[serde(rename = "T_w_l")]
    pub t_w_l: [f64; 16],
    pub eig: [f64; 3],
    pub n: u32,
}

impl ChildPose {
    /// `eig_min / eig_max`, or None before any correspondence.
    pub fn degeneracy_ratio(&self) -> Option<f64> {
        let max = self.eig.iter().copied().fold(0.0f64, f64::max);
        let min = self.eig.iter().copied().fold(f64::INFINITY, f64::min);
        (self.n > 0 && max > 0.0 && max.is_finite()).then(|| min.max(0.0) / max)
    }
}

pub fn parse_line(line: &str) -> Option<ChildPose> {
    let p: ChildPose = serde_json::from_str(line).ok()?;
    p.t_w_l.iter().all(|v| v.is_finite()).then_some(p)
}
