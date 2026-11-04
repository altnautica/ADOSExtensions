//! The sweep point cloud: every planar scan placed by the full 6-DoF pose
//! (SLAM x, y, yaw; the flight controller's height; the IMU's roll and pitch)
//! and merged into a voxel hash that keeps one centroid per voxel. As the
//! vehicle climbs or tilts, a planar LiDAR sweeps out 3D structure.
//!
//! Bounded: the hash may hold the sweep share of the memory budget. When the
//! next insert would pass it, the voxel edge doubles (5 → 10 → 20 cm …), the
//! hash is re-merged at the new edge, and `sweep_cloud_coarsened {voxel_m}` is
//! logged. Never allocating past the budget wins over resolution.

use std::collections::HashMap;
use std::io::Write;
use std::path::Path;

use base64::Engine as _;
use world_engine_protocol::atlas::nav::POINTCLOUD_DELTA_MAX_POINTS;

/// RAM one voxel costs: key, centroid sum, count, flag, plus hash overhead.
pub const VOXEL_BYTES: usize = 48;
/// RAM one pending-delta key costs.
const PENDING_BYTES: usize = 12;

#[derive(Debug, Clone, Copy)]
struct Voxel {
    sum: [f32; 3],
    n: u32,
}

impl Voxel {
    fn centroid(&self) -> [f32; 3] {
        let k = 1.0 / self.n as f32;
        [self.sum[0] * k, self.sum[1] * k, self.sum[2] * k]
    }
}

/// The bounded voxel-centroid cloud.
pub struct SweepCloud {
    voxel_m: f32,
    cap_bytes: usize,
    voxels: HashMap<[i32; 3], Voxel>,
    /// Voxels created since the last delta, oldest first.
    pending: Vec<[i32; 3]>,
    coarsened: u32,
}

impl SweepCloud {
    pub fn new(voxel_m: f32, cap_bytes: usize) -> Self {
        Self {
            voxel_m,
            cap_bytes,
            voxels: HashMap::new(),
            pending: Vec::new(),
            coarsened: 0,
        }
    }

    pub fn voxel_m(&self) -> f32 {
        self.voxel_m
    }

    pub fn len(&self) -> usize {
        self.voxels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.voxels.is_empty()
    }

    /// Times the edge has doubled.
    pub fn coarsened(&self) -> u32 {
        self.coarsened
    }

    pub fn bytes(&self) -> usize {
        self.voxels.len() * VOXEL_BYTES + self.pending.len() * PENDING_BYTES
    }

    fn key(&self, p: [f32; 3]) -> [i32; 3] {
        let k = 1.0 / self.voxel_m;
        [
            (p[0] * k).floor() as i32,
            (p[1] * k).floor() as i32,
            (p[2] * k).floor() as i32,
        ]
    }

    /// Merge `points` (map_ned, metres). Coarsens first whenever a new voxel
    /// would pass the cap.
    pub fn insert(&mut self, points: impl IntoIterator<Item = [f32; 3]>) {
        for p in points {
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let mut key = self.key(p);
            if !self.voxels.contains_key(&key) {
                while self.bytes() + VOXEL_BYTES + PENDING_BYTES > self.cap_bytes
                    && !self.voxels.is_empty()
                {
                    self.coarsen();
                }
                key = self.key(p);
            }
            match self.voxels.get_mut(&key) {
                Some(v) => {
                    v.sum = [v.sum[0] + p[0], v.sum[1] + p[1], v.sum[2] + p[2]];
                    v.n += 1;
                }
                None => {
                    self.voxels.insert(key, Voxel { sum: p, n: 1 });
                    self.pending.push(key);
                }
            }
        }
    }

    /// Double the voxel edge and re-merge; every voxel becomes pending again
    /// (a consumer seeing the new `voxel_m` drops what it had).
    fn coarsen(&mut self) {
        self.voxel_m *= 2.0;
        self.coarsened += 1;
        let old = std::mem::take(&mut self.voxels);
        self.pending.clear();
        for v in old.into_values() {
            let key = self.key(v.centroid());
            match self.voxels.get_mut(&key) {
                Some(m) => {
                    m.sum = [
                        m.sum[0] + v.sum[0],
                        m.sum[1] + v.sum[1],
                        m.sum[2] + v.sum[2],
                    ];
                    m.n += v.n;
                }
                None => {
                    self.voxels.insert(key, v);
                }
            }
        }
        self.pending.extend(self.voxels.keys().copied());
        self.voxels.shrink_to_fit();
        tracing::warn!(
            voxel_m = self.voxel_m,
            voxels = self.voxels.len(),
            "sweep_cloud_coarsened"
        );
    }

    /// Up to [`POINTCLOUD_DELTA_MAX_POINTS`] centroids new since the last
    /// delta, as base64 little-endian `f32` xyz, and their count. The rest
    /// wait for the next.
    pub fn take_delta(&mut self) -> (u32, String) {
        let n = self.pending.len().min(POINTCLOUD_DELTA_MAX_POINTS);
        let mut buf = Vec::with_capacity(n * 12);
        for key in self.pending.drain(..n) {
            if let Some(v) = self.voxels.get(&key) {
                for c in v.centroid() {
                    buf.extend_from_slice(&c.to_le_bytes());
                }
            }
        }
        let count = (buf.len() / 12) as u32;
        (count, base64::engine::general_purpose::STANDARD.encode(buf))
    }

    /// Write every centroid as little-endian `f32` xyz to `path`.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
        for v in self.voxels.values() {
            for c in v.centroid() {
                f.write_all(&c.to_le_bytes())?;
            }
        }
        f.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn points_in_one_voxel_merge_to_their_centroid() {
        let mut c = SweepCloud::new(0.05, usize::MAX);
        c.insert([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [1.0, 0.0, 0.0]]);
        assert_eq!(c.len(), 2);
        let (n, b64) = c.take_delta();
        assert_eq!(n, 2);
        let raw = base64::engine::general_purpose::STANDARD
            .decode(b64)
            .unwrap();
        let x0 = f32::from_le_bytes(raw[0..4].try_into().unwrap());
        assert!((x0 - 0.02).abs() < 1e-6);
        assert_eq!(c.take_delta().0, 0, "a delta is sent once");
    }

    #[test]
    fn a_full_hash_doubles_the_voxel_instead_of_growing() {
        let cap = 100 * (VOXEL_BYTES + PENDING_BYTES);
        let mut c = SweepCloud::new(0.05, cap);
        let pts: Vec<[f32; 3]> = (0..2000)
            .map(|i| [(i % 50) as f32 * 0.05, (i / 50) as f32 * 0.05, 0.0])
            .collect();
        c.insert(pts);
        assert!(c.coarsened() >= 1);
        assert!(c.voxel_m() >= 0.1);
        assert!(c.bytes() <= cap, "{} > {cap}", c.bytes());
        assert!(!c.is_empty());
    }

    #[test]
    fn a_delta_is_capped_and_the_rest_waits() {
        let mut c = SweepCloud::new(0.05, usize::MAX);
        c.insert((0..25_000).map(|i| [i as f32 * 0.1, 0.0, 0.0]));
        assert_eq!(c.take_delta().0 as usize, POINTCLOUD_DELTA_MAX_POINTS);
        assert_eq!(c.take_delta().0, 5_000);
    }
}
