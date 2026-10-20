//! Local SLAM: IMU attitude, scan matching into the active submaps, the
//! motion filter and the submap lifecycle. A plain struct; the caller owns the
//! threading.

use std::collections::VecDeque;

use crate::attitude::{Attitude, ImuSample};
use crate::config::Scan2dConfig;
use crate::matcher::{
    constraint_information, correlative, degeneracy, keep_constrained, refine, Prior, Window,
};
use crate::pose::{wrap, Pose2};
use crate::scan::{Motion, Scan2d, ScanInput};
use crate::submap::{Node, Submap};

/// Scans with fewer usable points than this are not matched.
const MIN_POINTS: usize = 20;
/// Baseline of the constant-velocity motion prior: a velocity differenced
/// over single scans would feed each match's noise straight into the next
/// prediction.
const VELOCITY_WINDOW_NS: u64 = 300_000_000;

/// One pose of the LiDAR frame in the local frame (the LiDAR frame at the
/// first scan, levelled): planar `x`, `y` and `yaw`, plus the IMU's `roll` and
/// `pitch`. Angles in radians.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PoseOut {
    /// The scan's first-point time.
    pub t_ns: u64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub roll: f64,
    pub pitch: f64,
    /// Mean interpolated submap probability of the matched scan (0 when the
    /// scan was not matched).
    pub score: f64,
    /// Translation `eig_min / eig_max` of the scan's point-to-line
    /// constraints ([`crate::matcher::degeneracy`]); small in corridors.
    pub degeneracy: f64,
}

/// Everything local SLAM built, for the offline back end.
#[derive(Debug, Clone, Default)]
pub struct LocalMap {
    /// All submaps in creation order.
    pub submaps: Vec<Submap>,
    /// Every matched scan in time order.
    pub nodes: Vec<Node>,
}

/// Live 2D LiDAR odometry and mapping. Feed IMU samples and scans in time
/// order; every scan yields its pose.
#[derive(Debug, Clone)]
pub struct LocalSlam {
    cfg: Scan2dConfig,
    att: Attitude,
    record: bool,
    active: Vec<(usize, Submap)>,
    finished: Vec<Submap>,
    next_id: usize,
    nodes: Vec<Node>,
    last: Option<(u64, Pose2, f64)>,
    /// Matched poses of the last [`VELOCITY_WINDOW_NS`], oldest first.
    history: VecDeque<(u64, Pose2)>,
    last_insert: Option<(u64, Pose2)>,
    inserted: usize,
}

impl LocalSlam {
    /// Live use: finished submaps and past scans are dropped.
    pub fn new(cfg: Scan2dConfig) -> Self {
        Self::build(cfg, false)
    }

    /// Keeps every submap and scan for [`Self::into_map`].
    pub fn recording(cfg: Scan2dConfig) -> Self {
        Self::build(cfg, true)
    }

    fn build(cfg: Scan2dConfig, record: bool) -> Self {
        LocalSlam {
            att: Attitude::new(cfg.mahony_kp, cfg.mahony_ki, cfg.r_lidar_imu),
            cfg,
            record,
            active: Vec::new(),
            finished: Vec::new(),
            next_id: 0,
            nodes: Vec::new(),
            last: None,
            history: VecDeque::new(),
            last_insert: None,
            inserted: 0,
        }
    }

    pub fn config(&self) -> &Scan2dConfig {
        &self.cfg
    }

    /// Scans inserted into the submaps so far.
    pub fn inserted_scans(&self) -> usize {
        self.inserted
    }

    pub fn feed_imu(&mut self, s: &ImuSample) {
        self.att.update(s);
    }

    /// Match one scan and return its pose.
    pub fn feed_scan(&mut self, input: &ScanInput) -> PoseOut {
        let t = input.t_ns;
        let (roll, pitch) = self.att.roll_pitch();
        let va = self.att.vertical_angle_at(t);
        let span_ns = input.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
        let (prior, velocity, yaw_rate) = self.predict(t, va, span_ns);
        let motion = Motion {
            roll,
            pitch,
            yaw_rate,
            velocity,
        };
        let scan = Scan2d::from_cloud(input, &self.cfg, &motion);
        let mut out = PoseOut {
            t_ns: t,
            x: prior.x,
            y: prior.y,
            yaw: prior.th,
            roll,
            pitch,
            score: 0.0,
            degeneracy: 0.0,
        };
        if scan.points.len() < MIN_POINTS {
            self.advance(t, prior, va);
            return out;
        }
        if self.active.is_empty() {
            self.start_submap(prior);
        }
        let (id, sm) = &self.active[0];
        let mut pose = prior;
        let mut in_submap = sm.pose.inverse() * prior;
        if sm.scans() > 0 {
            let window = Window {
                linear_m: self.cfg.linear_window_m,
                angular_rad: self.cfg.angular_window_deg.to_radians(),
            };
            let (c, _) = correlative(&sm.grid, &scan, in_submap, window);
            let pull = Prior {
                pose: in_submap,
                translation_weight: self.cfg.translation_weight,
                rotation_weight: self.cfg.rotation_weight,
            };
            let r = refine(
                &sm.grid,
                &scan,
                c,
                Some(&pull),
                self.cfg.gn_max_iters,
                self.cfg.gn_eps,
            );
            let info = constraint_information(&scan, r.pose);
            let deg = degeneracy(&info);
            in_submap = if deg < self.cfg.degenerate_ratio {
                keep_constrained(in_submap, r.pose, &info, self.cfg.degenerate_ratio)
            } else {
                r.pose
            };
            pose = sm.pose * in_submap;
            out.score = r.score;
            out.degeneracy = deg;
        }
        let home = *id;
        (out.x, out.y, out.yaw) = (pose.x, pose.y, pose.th);
        let insert = self.should_insert(t, pose);
        if insert {
            self.insert(&scan, t, pose);
        }
        if self.record {
            self.nodes.push(Node {
                t_ns: t,
                submap: home,
                in_submap,
                roll,
                pitch,
                score: out.score,
                degeneracy: out.degeneracy,
                scan: insert.then_some(scan),
            });
        }
        self.advance(t, pose, va);
        out
    }

    /// Prior pose, LiDAR-frame velocity, and the yaw rate over the scan's
    /// `span_ns`, from the IMU's vertical rotation and the mean velocity over
    /// the recent poses. The velocity is kept in the LiDAR frame (the chord
    /// over the window is expressed at its middle heading) so it turns with
    /// the walker through a corner.
    fn predict(&self, t: u64, va: f64, span_ns: u64) -> (Pose2, [f64; 2], f64) {
        let imu_rate = if span_ns > 0 {
            (self.att.vertical_angle_at(t + span_ns) - va) / (span_ns as f64 * 1e-9)
        } else {
            self.att.vertical_rate()
        };
        let Some((t1, p1, va1)) = self.last else {
            return (Pose2::IDENTITY, [0.0, 0.0], imu_rate);
        };
        let dt = t.saturating_sub(t1) as f64 * 1e-9;
        let (vel, omega) = match self.history.front() {
            Some(&(t0, p0)) if t1 > t0 => {
                let d = (t1 - t0) as f64 * 1e-9;
                let turn = wrap(p1.th - p0.th);
                let (s, c) = (p0.th + 0.5 * turn).sin_cos();
                let (cx, cy) = ((p1.x - p0.x) / d, (p1.y - p0.y) / d);
                ([c * cx + s * cy, -s * cx + c * cy], turn / d)
            }
            _ => ([0.0, 0.0], 0.0),
        };
        let (dyaw, rate) = if self.att.has_data() {
            (va - va1, imu_rate)
        } else {
            (omega * dt, omega)
        };
        let (s, c) = (p1.th + 0.5 * dyaw).sin_cos();
        let step = [
            (c * vel[0] - s * vel[1]) * dt,
            (s * vel[0] + c * vel[1]) * dt,
        ];
        (
            Pose2::new(p1.x + step[0], p1.y + step[1], p1.th + dyaw),
            vel,
            rate,
        )
    }

    fn advance(&mut self, t: u64, pose: Pose2, va: f64) {
        self.last = Some((t, pose, va));
        self.history.push_back((t, pose));
        while self.history.len() > 2 && t.saturating_sub(self.history[1].0) >= VELOCITY_WINDOW_NS {
            self.history.pop_front();
        }
    }

    fn should_insert(&self, t: u64, pose: Pose2) -> bool {
        let Some((ti, pi)) = self.last_insert else {
            return true;
        };
        pose.dist(&pi) >= self.cfg.min_translation_m
            || wrap(pose.th - pi.th).abs() >= self.cfg.min_rotation_deg.to_radians()
            || t.saturating_sub(ti) as f64 * 1e-9 >= self.cfg.max_interval_s
    }

    fn insert(&mut self, scan: &Scan2d, t: u64, pose: Pose2) {
        for (_, sm) in &mut self.active {
            sm.insert(scan, pose);
        }
        self.inserted += 1;
        self.last_insert = Some((t, pose));
        let full = self.cfg.scans_per_submap.max(2);
        if self.active[0].1.scans() >= full {
            let (_, done) = self.active.remove(0);
            if self.record {
                self.finished.push(done);
            }
        }
        let start = match self.active.as_slice() {
            [] => true,
            [(_, only)] => only.scans() >= full / 2,
            _ => false,
        };
        if start {
            self.start_submap(pose);
        }
    }

    fn start_submap(&mut self, pose: Pose2) {
        self.active
            .push((self.next_id, Submap::new(pose, &self.cfg)));
        self.next_id += 1;
    }

    /// The submaps and scans of a [`Self::recording`] run, submaps indexed by
    /// creation order (the ids in [`Node::submap`]).
    pub fn into_map(self) -> LocalMap {
        let mut submaps = self.finished;
        submaps.extend(self.active.into_iter().map(|(_, s)| s));
        LocalMap {
            submaps,
            nodes: self.nodes,
        }
    }
}
