//! Reading the map: voxel class, clearance and line of sight.

use crate::esdf::D2_INF;
use crate::{VoxMap, ESDF_CAP_M, VOX_FREE, VOX_OCCUPIED};

/// The class of one voxel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelState {
    Unknown,
    Free,
    Occupied,
}

/// What a straight segment meets first.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RayHit {
    /// Every voxel from start to end is known free.
    Clear,
    /// An occupied voxel, entered at this map point.
    Occupied([f32; 3]),
    /// An unknown voxel (or the window's edge), entered at this map point.
    Unknown([f32; 3]),
}

impl VoxMap {
    /// The class of absolute voxel `c`; unknown outside the window.
    pub fn state(&self, c: [i32; 3]) -> VoxelState {
        match self.idx_of_cell(c).map(|i| self.state[i]) {
            Some(VOX_OCCUPIED) => VoxelState::Occupied,
            Some(VOX_FREE) => VoxelState::Free,
            _ => VoxelState::Unknown,
        }
    }

    /// The class of the voxel holding map point `p`.
    pub fn state_at(&self, p: [f32; 3]) -> VoxelState {
        self.state(self.cell_of(p))
    }

    /// The log-odds of absolute voxel `c` in units of [`crate::LO_UNIT`];
    /// zero outside the window.
    pub fn log_odds(&self, c: [i32; 3]) -> i8 {
        self.idx_of_cell(c).map_or(0, |i| self.vox[i].lo)
    }

    /// Distance from the centre of the voxel holding `p` to the centre of the
    /// nearest obstacle voxel in the window, metres, at most
    /// [`ESDF_CAP_M`]; zero inside an obstacle voxel. `None` outside the
    /// window. Reflects the last distance-field update.
    pub fn clearance_m(&self, p: [f32; 3]) -> Option<f32> {
        let idx = self.idx_of_cell(self.cell_of(p))?;
        let d2 = self.esdf.cells[idx].d2;
        Some(if d2 == D2_INF {
            ESDF_CAP_M
        } else {
            ((d2 as f32).sqrt() * self.res).min(ESDF_CAP_M)
        })
    }

    /// Walk the voxels from `from` to `to`, both included, and report the
    /// first one that is not known free.
    pub fn ray(&self, from: [f32; 3], to: [f32; 3]) -> RayHit {
        let c0 = self.cell_of(from);
        let c1 = self.cell_of(to);
        let d = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
        let mut step = [0i32; 3];
        let mut t_max = [f32::INFINITY; 3];
        let mut t_delta = [f32::INFINITY; 3];
        let mut rem = [0i32; 3];
        for a in 0..3 {
            rem[a] = (c1[a] - c0[a]).abs();
            step[a] = (c1[a] - c0[a]).signum();
            if d[a] != 0.0 {
                let edge = (c0[a] + i32::from(step[a] > 0)) as f32 * self.res;
                t_max[a] = (edge - from[a]) / d[a];
                t_delta[a] = self.res / d[a].abs();
            }
        }
        let point = |t: f32| {
            let t = t.clamp(0.0, 1.0);
            [from[0] + d[0] * t, from[1] + d[1] * t, from[2] + d[2] * t]
        };
        let mut c = c0;
        let mut t_enter = 0.0f32;
        loop {
            match self.idx_of_cell(c).map(|i| self.state[i]) {
                Some(VOX_FREE) => {}
                Some(VOX_OCCUPIED) => return RayHit::Occupied(point(t_enter)),
                _ => return RayHit::Unknown(point(t_enter)),
            }
            if rem == [0, 0, 0] {
                return RayHit::Clear;
            }
            let a = if rem[0] > 0
                && (rem[1] == 0 || t_max[0] <= t_max[1])
                && (rem[2] == 0 || t_max[0] <= t_max[2])
            {
                0
            } else if rem[1] > 0 && (rem[2] == 0 || t_max[1] <= t_max[2]) {
                1
            } else {
                2
            };
            t_enter = t_max[a];
            t_max[a] += t_delta[a];
            rem[a] -= 1;
            c[a] += step[a];
        }
    }
}
