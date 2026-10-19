//! Live coverage: which space the rig's cameras have seen, as a 0.25 m voxel
//! hash of frustum hits, its top-down projection, and the walked trajectory.
//!
//! Without depth the frustum is sampled as a fan of rays (a 5x3 grid across
//! the field of view) stepped every voxel out to [`RANGE_M`]; every voxel a
//! ray crosses is a hit. Per room, `room_pct` is the share of the top-down
//! cells inside the room's walked footprint (trajectory bounding box grown by
//! [`ROOM_MARGIN_M`]) that have been hit.

use std::collections::{HashMap, HashSet};

use world_engine_protocol::capture_ctl::{CoverageSummary, CoverageView};

use crate::pose::se3;

/// Voxel edge, metres.
pub const VOXEL_M: f64 = 0.25;
/// Ray length, metres (indoor scale).
pub const RANGE_M: f64 = 4.0;
/// Margin around a room's walked footprint, metres.
pub const ROOM_MARGIN_M: f64 = 1.0;
/// Trajectory points are kept at most this far apart, metres.
const TRAJ_STEP_M: f64 = 0.2;
/// The loop is closed once back within this of the start after walking at
/// least [`LOOP_MIN_WALK_M`].
pub const LOOP_RADIUS_M: f64 = 2.0;
pub const LOOP_MIN_WALK_M: f64 = 20.0;

type Key = (i64, i64, i64);

fn key(p: [f64; 3]) -> Key {
    (
        (p[0] / VOXEL_M).floor() as i64,
        (p[1] / VOXEL_M).floor() as i64,
        (p[2] / VOXEL_M).floor() as i64,
    )
}

#[derive(Debug, Default, Clone)]
pub struct Coverage {
    voxels: HashSet<Key>,
    cells: HashMap<(i64, i64), u32>,
    trajectory: Vec<[f64; 3]>,
    walked_m: f64,
    heading_deg: Option<f64>,
    room_start: usize,
    loop_closed: bool,
}

impl Coverage {
    pub fn new() -> Self {
        Self::default()
    }

    /// A new room begins (MARK): its footprint starts at the current position.
    pub fn mark_room(&mut self) {
        self.room_start = self.trajectory.len().saturating_sub(1);
    }

    /// Fold a camera pose (`R_world_cam` OpenCV axes, position) with half
    /// fields of view in radians.
    pub fn observe_camera(
        &mut self,
        r_world_cam: &[f64; 9],
        pos: [f64; 3],
        hfov_half: f64,
        vfov_half: f64,
    ) {
        let mut crossed: HashSet<Key> = HashSet::new();
        for iy in -1i32..=1 {
            for ix in -2i32..=2 {
                let dir_cam = [
                    (hfov_half * ix as f64 / 2.0).tan(),
                    (vfov_half * iy as f64).tan(),
                    1.0,
                ];
                let n = (dir_cam[0].powi(2) + dir_cam[1].powi(2) + 1.0).sqrt();
                let d = se3::mat_vec(r_world_cam, [dir_cam[0] / n, dir_cam[1] / n, 1.0 / n]);
                let mut s = VOXEL_M;
                while s <= RANGE_M {
                    crossed.insert(key([
                        pos[0] + d[0] * s,
                        pos[1] + d[1] * s,
                        pos[2] + d[2] * s,
                    ]));
                    s += VOXEL_M;
                }
            }
        }
        for k in crossed {
            if self.voxels.insert(k) {
                *self.cells.entry((k.0, k.1)).or_default() += 1;
            }
        }
    }

    /// Fold a body pose: extends the trajectory and the heading.
    pub fn observe_body(&mut self, r_world_body: &[f64; 9], pos: [f64; 3]) {
        let fwd = se3::mat_vec(r_world_body, [1.0, 0.0, 0.0]);
        if fwd[0].hypot(fwd[1]) > 1e-6 {
            self.heading_deg = Some(fwd[0].atan2(fwd[1]).to_degrees().rem_euclid(360.0));
        }
        match self.trajectory.last() {
            Some(last) => {
                let d = dist(*last, pos);
                if d >= TRAJ_STEP_M {
                    self.walked_m += d;
                    self.trajectory.push(pos);
                }
            }
            None => self.trajectory.push(pos),
        }
        if !self.loop_closed
            && self.walked_m >= LOOP_MIN_WALK_M
            && self
                .distance_from_start()
                .is_some_and(|d| d <= LOOP_RADIUS_M)
        {
            self.loop_closed = true;
        }
    }

    pub fn voxels(&self) -> usize {
        self.voxels.len()
    }

    pub fn distance_from_start(&self) -> Option<f64> {
        Some(dist(*self.trajectory.first()?, *self.trajectory.last()?))
    }

    /// Share of the current room's footprint cells that have been seen, %.
    pub fn room_pct(&self) -> Option<f64> {
        let room = self.trajectory.get(self.room_start..)?;
        let first = room.first()?;
        let (mut lo, mut hi) = ([first[0], first[1]], [first[0], first[1]]);
        for p in room {
            lo = [lo[0].min(p[0]), lo[1].min(p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(p[1])];
        }
        let cell = |v: f64| (v / VOXEL_M).floor() as i64;
        let (x0, x1) = (cell(lo[0] - ROOM_MARGIN_M), cell(hi[0] + ROOM_MARGIN_M));
        let (y0, y1) = (cell(lo[1] - ROOM_MARGIN_M), cell(hi[1] + ROOM_MARGIN_M));
        let total = ((x1 - x0 + 1) * (y1 - y0 + 1)) as f64;
        let seen = (x0..=x1)
            .flat_map(|x| (y0..=y1).map(move |y| (x, y)))
            .filter(|c| self.cells.contains_key(c))
            .count() as f64;
        Some(100.0 * seen / total)
    }

    pub fn summary(&self) -> CoverageSummary {
        CoverageSummary {
            voxels: self.voxels.len() as u64,
            room_pct: self.room_pct(),
            loop_closed: self.loop_closed,
            distance_from_start_m: self.distance_from_start(),
        }
    }

    /// The `GET .../coverage` body.
    pub fn view(&self) -> CoverageView {
        let mut cells: Vec<[i64; 3]> = self
            .cells
            .iter()
            .map(|(&(x, y), &n)| [x, y, n as i64])
            .collect();
        cells.sort_unstable();
        CoverageView {
            cell_m: VOXEL_M,
            cells,
            trajectory: self.trajectory.clone(),
            start: self.trajectory.first().copied(),
            current: self.trajectory.last().copied(),
            heading_deg: self.heading_deg,
        }
    }
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// OpenCV camera looking along world +x (ENU east), y down = world −z.
    const LOOK_EAST: [f64; 9] = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0];

    #[test]
    fn a_frustum_hits_voxels_ahead_and_none_behind() {
        let mut c = Coverage::new();
        c.observe_camera(&LOOK_EAST, [0.1, 0.1, 1.5], 0.6, 0.4);
        let n = c.voxels();
        assert!(n > 40, "{n}");
        let v = c.view();
        assert!(
            v.cells.iter().all(|[x, _, _]| *x >= 0),
            "nothing behind the camera"
        );
        assert!(v.cells.iter().any(|[x, _, _]| *x >= 15), "reaches ~4 m");
        // Re-observing the same view adds nothing.
        c.observe_camera(&LOOK_EAST, [0.1, 0.1, 1.5], 0.6, 0.4);
        assert_eq!(c.voxels(), n);
    }

    #[test]
    fn trajectory_heading_room_share_and_loop_closure() {
        let mut c = Coverage::new();
        let body_east = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        // Walk a 6 m x 4 m rectangle twice, looking forward.
        let corners = [[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 4.0], [0.0, 0.0]];
        for _ in 0..2 {
            for w in corners.windows(2) {
                for k in 0..=20 {
                    let s = k as f64 / 20.0;
                    let p = [
                        w[0][0] + s * (w[1][0] - w[0][0]),
                        w[0][1] + s * (w[1][1] - w[0][1]),
                        1.5,
                    ];
                    c.observe_body(&body_east, p);
                    c.observe_camera(&LOOK_EAST, p, 0.6, 0.4);
                }
            }
        }
        let s = c.summary();
        assert!(s.loop_closed);
        assert!(s.distance_from_start_m.unwrap() < 0.3);
        let pct = s.room_pct.unwrap();
        assert!(pct > 30.0 && pct <= 100.0, "{pct}");
        let v = c.view();
        assert_eq!(v.heading_deg, Some(90.0));
        assert_eq!(v.cell_m, 0.25);
        assert!(v.trajectory.len() > 20);
        c.mark_room();
        assert!(c.room_pct().is_some());
    }
}
