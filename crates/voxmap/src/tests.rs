//! Unit tests over the map's internals.

mod esdf;
mod insert;
mod time;
mod window;

use crate::{VoxMap, VoxMapConfig};

pub(crate) const SEC: u64 = 1_000_000_000;

/// A 9.6 x 9.6 x 4.8 m window at 0.15 m, centred on the map origin.
pub(crate) fn map64() -> VoxMap {
    VoxMap::new(VoxMapConfig {
        res_m: 0.15,
        dims: [64, 64, 32],
        recentre_slack_m: 0.0,
        esdf_every: 1,
    })
    .unwrap()
}

/// Deterministic xorshift for test maps.
pub(crate) struct Rng(pub u64);

impl Rng {
    pub(crate) fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    pub(crate) fn below(&mut self, n: i32) -> i32 {
        (self.next() % n as u64) as i32
    }
}

impl VoxMap {
    /// Force absolute voxel `c` to log-odds `lo` (inside the window).
    pub(crate) fn set_cell(&mut self, c: [i32; 3], lo: i8) {
        let idx = self.idx_of_cell(c).expect("cell in window");
        self.store_lo(idx, lo);
    }

    /// Every obstacle voxel in the window, absolute coordinates, checking
    /// the obstacle set against the log-odds on the way: every occupied
    /// voxel is in it, and nothing without positive evidence is.
    pub(crate) fn obstacles(&self) -> Vec<[i32; 3]> {
        let mut out = Vec::new();
        for w in self.slab(0, 0..self.n[0]) {
            let idx = self.idx_of_window(w);
            let c = [0, 1, 2].map(|a| w[a] + self.origin[a]);
            let member = self.esdf.is_obstacle(idx);
            let lo = self.vox[idx].lo;
            assert!(!member || lo > 0, "voxel {c:?} is an obstacle at {lo}");
            assert!(
                member || lo < crate::LO_OCCUPIED,
                "voxel {c:?} occupied, no obstacle"
            );
            if member {
                out.push(c);
            }
        }
        out
    }

    /// Check every voxel's stored distance against an exhaustive search over
    /// the occupied voxels: never below it (the stored obstacle is a real
    /// one), above it by at most `slack_vox` voxels, and the published
    /// decimetres in step. Returns the largest overestimate in voxels.
    pub(crate) fn check_esdf(&self, slack_vox: f32) -> f32 {
        let obstacles = self.obstacles();
        let mut worst = 0.0f32;
        for w in self.slab(0, 0..self.n[0]) {
            let c = [0, 1, 2].map(|a| w[a] + self.origin[a]);
            let idx = self.idx_of_window(w);
            let cell = self.esdf.cells[idx];
            assert!(!cell.raise, "voxel {c:?} left marked for raise");
            let got = cell.d2;
            let want = obstacles
                .iter()
                .map(|o| (0..3).map(|a| (o[a] - c[a]).pow(2)).sum::<i32>() as u32)
                .min()
                .filter(|&d| d <= self.esdf.cap2);
            match want {
                None => assert_eq!(
                    got,
                    crate::esdf::D2_INF,
                    "voxel {c:?} has no obstacle in reach"
                ),
                Some(want) => {
                    assert_ne!(got, crate::esdf::D2_INF, "voxel {c:?} lost its obstacle");
                    let got = u32::from(got);
                    assert!(got >= want, "voxel {c:?}: {got} below the true {want}");
                    let over = (got as f32).sqrt() - (want as f32).sqrt();
                    assert!(over <= slack_vox, "voxel {c:?}: {got} vs {want}");
                    worst = worst.max(over);
                    let oi = self.idx_of_ring(crate::window::unpack(cell.obst));
                    assert!(
                        self.esdf.is_obstacle(oi),
                        "voxel {c:?} points at a non-obstacle"
                    );
                }
            }
            let dm = if got == crate::esdf::D2_INF {
                crate::ESDF_CAP_DM
            } else {
                (f32::from(got).sqrt() * self.res * 10.0).round().min(40.0) as u8
            };
            assert_eq!(self.esdf.dm_of(got), dm, "voxel {c:?} decimetres");
            assert_eq!(self.esdf.dm[idx], dm, "voxel {c:?} published decimetres");
        }
        worst
    }
}
