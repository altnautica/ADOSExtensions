//! Live coverage: which space the rig's cameras have seen, as a 0.25 m voxel
//! hash of frustum hits, its top-down projection, the walked trajectory, and
//! the loop prompt.
//!
//! The frustum is sampled as a fan of rays (a 5x3 grid across the field of
//! view) stepped every voxel out to [`RANGE_M`], or, with a fresh LiDAR scan,
//! out to the median LiDAR range around the ray's azimuth when that is
//! nearer; every voxel a ray crosses is a hit. Per room, `room_pct` is the
//! share of the top-down cells inside the room's walked footprint (trajectory
//! bounding box grown by [`ROOM_MARGIN_M`]) that have been hit.
//!
//! A revisit is the walk coming back within [`REVISIT_RADIUS_M`] of a point
//! it passed at least [`REVISIT_MIN_AGE_NS`] earlier: that closes a loop for
//! the reconstruction. Every [`LOOP_PROMPT_EVERY_M`] walked without one, the
//! close-loop prompt is raised; a revisit clears it.

use std::collections::{HashMap, HashSet};

use world_engine_protocol::capture_ctl::{CoverageSummary, CoverageView};

use super::lidar_depth::LidarDepth;
use crate::pose::se3;

/// Voxel edge, metres.
pub const VOXEL_M: f64 = 0.25;
/// Ray length, metres (indoor scale).
pub const RANGE_M: f64 = 4.0;
/// Margin around a room's walked footprint, metres.
pub const ROOM_MARGIN_M: f64 = 1.0;
/// Trajectory points are kept at most this far apart, metres.
const TRAJ_STEP_M: f64 = 0.2;
/// A revisit comes back within this of an earlier trajectory point, metres.
pub const REVISIT_RADIUS_M: f64 = 2.0;
/// ... a point passed at least this long before.
pub const REVISIT_MIN_AGE_NS: u64 = 30_000_000_000;
/// Walked distance without a revisit at which the close-loop prompt is
/// raised, and again every further this much.
pub const LOOP_PROMPT_EVERY_M: f64 = 40.0;

type Key = (i64, i64, i64);

fn key(p: [f64; 3]) -> Key {
    (
        (p[0] / VOXEL_M).floor() as i64,
        (p[1] / VOXEL_M).floor() as i64,
        (p[2] / VOXEL_M).floor() as i64,
    )
}

/// The revisit-radius cell of `p`.
fn revisit_cell(p: [f64; 3]) -> Key {
    let c = |v: f64| (v / REVISIT_RADIUS_M).floor() as i64;
    (c(p[0]), c(p[1]), c(p[2]))
}

#[derive(Debug, Default, Clone)]
pub struct Coverage {
    voxels: HashSet<Key>,
    cells: HashMap<(i64, i64), u32>,
    trajectory: Vec<[f64; 3]>,
    /// Host ns of each trajectory point.
    trajectory_t: Vec<u64>,
    /// Trajectory point indices by [`revisit_cell`].
    by_cell: HashMap<Key, Vec<usize>>,
    heading_deg: Option<f64>,
    room_start: usize,
    revisits: u64,
    since_revisit_m: f64,
    /// Close-loop prompts raised since the last revisit.
    prompts_since_revisit: u64,
    /// Close-loop prompts raised this session.
    loop_prompts: u64,
    loop_prompt: bool,
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
    /// fields of view in radians; `depth` (a fresh scan) bounds the rays.
    pub fn observe_camera(
        &mut self,
        r_world_cam: &[f64; 9],
        pos: [f64; 3],
        hfov_half: f64,
        vfov_half: f64,
        depth: Option<&LidarDepth>,
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
                let az = d[1].atan2(d[0]).to_degrees();
                let range = depth
                    .and_then(|l| l.range_at(az))
                    .map_or(RANGE_M, |r| r.min(RANGE_M));
                let mut s = VOXEL_M;
                while s <= range {
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

    /// Fold a body pose at host `t_ns`: extends the trajectory and the
    /// heading, and tracks revisits and the close-loop prompt.
    pub fn observe_body(&mut self, r_world_body: &[f64; 9], pos: [f64; 3], t_ns: u64) {
        let fwd = se3::mat_vec(r_world_body, [1.0, 0.0, 0.0]);
        if fwd[0].hypot(fwd[1]) > 1e-6 {
            self.heading_deg = Some(fwd[0].atan2(fwd[1]).to_degrees().rem_euclid(360.0));
        }
        let step = match self.trajectory.last() {
            Some(last) => dist(*last, pos),
            None => 0.0,
        };
        if !self.trajectory.is_empty() && step < TRAJ_STEP_M {
            return;
        }
        self.since_revisit_m += step;
        if self.is_revisit(pos, t_ns) {
            self.revisits += 1;
            self.since_revisit_m = 0.0;
            self.prompts_since_revisit = 0;
            self.loop_prompt = false;
        } else if self.since_revisit_m
            >= LOOP_PROMPT_EVERY_M * (self.prompts_since_revisit + 1) as f64
        {
            self.prompts_since_revisit += 1;
            self.loop_prompts += 1;
            self.loop_prompt = true;
        }
        self.by_cell
            .entry(revisit_cell(pos))
            .or_default()
            .push(self.trajectory.len());
        self.trajectory.push(pos);
        self.trajectory_t.push(t_ns);
    }

    /// `pos` is within [`REVISIT_RADIUS_M`] of a trajectory point at least
    /// [`REVISIT_MIN_AGE_NS`] older than `t_ns`.
    fn is_revisit(&self, pos: [f64; 3], t_ns: u64) -> bool {
        let (cx, cy, cz) = revisit_cell(pos);
        (-1..=1).any(|dx| {
            (-1..=1).any(|dy| {
                (-1..=1).any(|dz| {
                    self.by_cell
                        .get(&(cx + dx, cy + dy, cz + dz))
                        .is_some_and(|idx| {
                            idx.iter().any(|&i| {
                                t_ns.saturating_sub(self.trajectory_t[i]) >= REVISIT_MIN_AGE_NS
                                    && dist(self.trajectory[i], pos) <= REVISIT_RADIUS_M
                            })
                        })
                })
            })
        })
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
            loop_closed: self.revisits > 0,
            distance_from_start_m: self.distance_from_start(),
            close_loop_prompt: self.loop_prompt,
            close_loop_prompts: self.loop_prompts,
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
#[path = "coverage_tests.rs"]
mod tests;
