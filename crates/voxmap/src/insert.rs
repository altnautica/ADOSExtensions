//! Inserting returns: 3D clouds and attitude-placed planar scans.

use crate::VoxMap;

/// Returns nearer than this to the sensor are ignored (self-hits, noise).
const MIN_RANGE_M: f32 = 0.05;

/// The voxels within this many voxels (on every axis) of a ray's return are
/// only near misses: they clear unknown or free space up to the surface but
/// never lower a voxel holding positive evidence. A surface between voxel
/// boundaries, or one seen at a grazing angle, sends some rays just past the
/// voxels other rays hit; full misses there would make the surface flicker
/// and let three such rays erase it as a moving obstacle.
pub(crate) const HIT_GUARD_VOX: i32 = 1;

/// A rigid pose in the map frame: translation (north, east, down) and unit
/// quaternion (w, x, y, z) taking body (forward, right, down) vectors into the
/// map frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub t: [f32; 3],
    pub q: [f32; 4],
}

impl Pose {
    /// Level pose at `t`.
    pub fn at(t: [f32; 3]) -> Self {
        Self {
            t,
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Rotate body vector `v` into the map frame.
    #[inline]
    pub fn rotate(&self, v: [f32; 3]) -> [f32; 3] {
        let [w, x, y, z] = self.q;
        // v + 2w(u x v) + 2u x (u x v), u = (x, y, z).
        let c1 = [
            y * v[2] - z * v[1],
            z * v[0] - x * v[2],
            x * v[1] - y * v[0],
        ];
        let c2 = [
            y * c1[2] - z * c1[1],
            z * c1[0] - x * c1[2],
            x * c1[1] - y * c1[0],
        ];
        [
            v[0] + 2.0 * (w * c1[0] + c2[0]),
            v[1] + 2.0 * (w * c1[1] + c2[1]),
            v[2] + 2.0 * (w * c1[2] + c2[2]),
        ]
    }
}

/// One planar return as the trust rule sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarReturn {
    /// Beam index in the scan.
    pub index: usize,
    /// Beam angle in the sensor plane, radians, positive from forward toward
    /// right.
    pub angle_rad: f32,
    pub range_m: f32,
    /// Where the return lands in the map frame.
    pub point: [f32; 3],
}

impl VoxMap {
    /// Insert a cloud of map-frame `points` seen from sensor position
    /// `origin` at ADOS time `stamp_ns`. A point beyond `max_range` clears
    /// its ray up to that range and marks nothing occupied. The sensor must
    /// be inside the window (call [`VoxMap::recentre`] first); otherwise
    /// nothing is inserted and `false` is returned. Rays are cut where they
    /// leave the window.
    pub fn insert_cloud(
        &mut self,
        stamp_ns: u64,
        origin: [f32; 3],
        points: &[[f32; 3]],
        max_range: f32,
    ) -> bool {
        if !origin.iter().all(|v| v.is_finite()) {
            return false;
        }
        let Some(ow) = self.window_of_cell(self.cell_of(origin)) else {
            return false;
        };
        self.stamp_ns = self.stamp_ns.max(stamp_ns);
        self.now_s = (stamp_ns / 1_000_000_000) as u8;
        self.begin_scan();

        // Hits first, so a voxel hit by one return is never cleared by a ray
        // passing through it in the same insertion.
        for p in points {
            let d = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
            let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if !r2.is_finite() || r2 < MIN_RANGE_M * MIN_RANGE_M || r2 > max_range * max_range {
                continue;
            }
            if let Some(idx) = self.idx_of_cell(self.cell_of(*p)) {
                self.hit_voxel(idx);
            }
        }
        for p in points {
            let d = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
            let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if !r2.is_finite() || r2 < MIN_RANGE_M * MIN_RANGE_M {
                continue;
            }
            let (end, hit) = if r2 > max_range * max_range {
                let s = max_range / r2.sqrt();
                ([0, 1, 2].map(|a| origin[a] + d[a] * s), false)
            } else {
                (*p, true)
            };
            self.cast(origin, ow, end, hit);
        }
        true
    }

    /// Insert one planar scan taken at `sensor` (the scan plane is the
    /// sensor's forward-right plane, so a tilted vehicle places its returns
    /// at their true 3D positions). Beam `i` has `ranges[i]` at `angles[i]`.
    /// A return that is not finite, not positive or beyond `max_range` is
    /// skipped, as is any return `trust` refuses: a refused return neither
    /// marks its voxel nor clears its ray.
    pub fn insert_planar_scan(
        &mut self,
        stamp_ns: u64,
        sensor: &Pose,
        ranges: &[f32],
        angles: &[f32],
        max_range: f32,
        mut trust: impl FnMut(PlanarReturn) -> bool,
    ) -> bool {
        let mut pts = std::mem::take(&mut self.scratch);
        pts.clear();
        for (index, (&range_m, &angle_rad)) in ranges.iter().zip(angles).enumerate() {
            if !range_m.is_finite() || range_m <= 0.0 || range_m > max_range {
                continue;
            }
            let (s, c) = angle_rad.sin_cos();
            let v = sensor.rotate([c * range_m, s * range_m, 0.0]);
            let point = [sensor.t[0] + v[0], sensor.t[1] + v[1], sensor.t[2] + v[2]];
            if trust(PlanarReturn {
                index,
                angle_rad,
                range_m,
                point,
            }) {
                pts.push(point);
            }
        }
        let ok = self.insert_cloud(stamp_ns, sensor.t, &pts, max_range);
        self.scratch = pts;
        ok
    }

    /// Mark the voxels of the ray from sensor `origin` (window voxel `ow`) to
    /// `end` as passed, cut where it leaves the window. The end voxel is
    /// passed too unless `hit`; a hit ray passes the voxels within
    /// [`HIT_GUARD_VOX`] of its end only as near misses. The walk runs from
    /// the end back toward the sensor and stops at the first voxel an earlier
    /// ray of this scan already passed: every ray of the scan converges on
    /// the same sensor, so from there on that ray covered the way. A ray
    /// ending in a voxel an earlier ray ended in is skipped.
    fn cast(&mut self, origin: [f32; 3], ow: [i32; 3], end: [f32; 3], hit: bool) {
        let d = [end[0] - origin[0], end[1] - origin[1], end[2] - origin[2]];
        // Clip to the window box: the ray leaves it for good at t_exit.
        let mut t_exit = 1.0f32;
        for a in 0..3 {
            let lo = self.origin[a] as f32 * self.res;
            let hi = (self.origin[a] + self.n[a]) as f32 * self.res;
            if d[a] > 0.0 {
                t_exit = t_exit.min((hi - origin[a]) / d[a]);
            } else if d[a] < 0.0 {
                t_exit = t_exit.min((lo - origin[a]) / d[a]);
            }
        }
        let clipped = t_exit < 1.0;
        let mut ew = [0i32; 3];
        for a in 0..3 {
            let e = origin[a] + d[a] * t_exit.min(1.0);
            let c = (e * self.inv_res).floor() as i32 - self.origin[a];
            ew[a] = c.clamp(0, self.n[a] - 1);
        }
        let end_idx = self.idx_of_window(ew);
        if !self.claim_ray_end(end_idx) {
            return;
        }
        self.stats.rays_cast += 1;
        let pass_end = clipped || !hit;
        if pass_end && self.pass_voxel(end_idx, false) {
            return;
        }

        // Walk back from the (clipped) end point toward the sensor.
        let te = t_exit.min(1.0);
        let start = [0, 1, 2].map(|a| origin[a] + d[a] * te);
        let mut step = [0i32; 3];
        let mut t_max = [f32::INFINITY; 3];
        let mut t_delta = [f32::INFINITY; 3];
        let mut rem = [0i32; 3];
        for a in 0..3 {
            rem[a] = (ow[a] - ew[a]).abs();
            step[a] = (ow[a] - ew[a]).signum();
            let back = -d[a];
            if back != 0.0 {
                let edge = (self.origin[a] + ew[a] + i32::from(step[a] > 0)) as f32 * self.res;
                t_max[a] = (edge - start[a]) / back;
                t_delta[a] = self.res / back.abs();
            }
        }
        let mut r = self.ring_of_window(ew);
        let mut idx = self.idx_of_ring(r) as isize;
        let guard = if pass_end { 0 } else { HIT_GUARD_VOX };
        // Per axis: the slot step, the ring coordinate that wraps, the slot
        // step across the wrap, and the coordinate after it.
        let mut delta = [0isize; 3];
        let mut wrap_at = [0i32; 3];
        let mut wrap_delta = [0isize; 3];
        let mut wrap_to = [0i32; 3];
        for a in 0..3 {
            let stride = self.stride[a] as isize;
            let span = (self.n[a] - 1) as isize;
            delta[a] = stride * step[a] as isize;
            if step[a] > 0 {
                (wrap_at[a], wrap_to[a], wrap_delta[a]) = (self.n[a], 0, -stride * span);
            } else {
                (wrap_at[a], wrap_to[a], wrap_delta[a]) = (-1, self.n[a] - 1, stride * span);
            }
            if rem[a] == 0 {
                t_max[a] = f32::INFINITY;
            }
        }
        let steps = rem[0] + rem[1] + rem[2];
        // Steps taken per axis: the distance from the end voxel.
        let mut gone = [0i32; 3];
        for _ in 0..steps {
            let mut a = if t_max[0] <= t_max[1] {
                if t_max[0] <= t_max[2] {
                    0
                } else {
                    2
                }
            } else if t_max[1] <= t_max[2] {
                1
            } else {
                2
            };
            if rem[a] == 0 {
                // Only when every remaining axis has an infinite crossing
                // time (a direction component that rounded to zero).
                a = if rem[0] > 0 {
                    0
                } else if rem[1] > 0 {
                    1
                } else {
                    2
                };
            }
            rem[a] -= 1;
            gone[a] += 1;
            t_max[a] = if rem[a] == 0 {
                f32::INFINITY
            } else {
                t_max[a] + t_delta[a]
            };
            r[a] += step[a];
            if r[a] == wrap_at[a] {
                r[a] = wrap_to[a];
                idx += wrap_delta[a];
            } else {
                idx += delta[a];
            }
            self.stats.ray_steps += 1;
            let near = guard > 0 && gone[0].max(gone[1]).max(gone[2]) <= guard;
            if self.pass_voxel(idx as usize, near) {
                break;
            }
        }
    }
}
