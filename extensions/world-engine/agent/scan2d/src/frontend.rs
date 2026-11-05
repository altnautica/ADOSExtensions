//! Local SLAM: IMU attitude, scan matching into the active submaps, the
//! motion filter and the submap lifecycle. A plain struct; the caller owns the
//! threading.

use std::collections::VecDeque;

use nalgebra::{Matrix2, Rotation2, Vector2};

use crate::attitude::{Attitude, ImuSample, VelocitySample};
use crate::config::Scan2dConfig;
use crate::matcher::{
    constraint_information, correlative, degeneracy, keep_constrained, refine,
    rotation_information, unconstrained_directions, Prior, Window,
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
/// Along a direction the matches stopped constraining, the motion prior
/// coasts on the velocity last observed there, fading with this time
/// constant: a short featureless stretch keeps its motion, a long one comes
/// to rest. Without the fade the prior would carry forward whatever
/// velocity it held, unobserved, for as long as the stretch lasts.
const COAST_TAU_S: f64 = 1.0;
/// Measured velocities older than this before a scan no longer stand for its
/// motion.
const MAX_VELOCITY_AGE_NS: u64 = 250_000_000;
/// How long measured velocities are kept.
const VELOCITY_KEEP_NS: u64 = 1_000_000_000;
/// A match teaches the gyro its z bias ([`Attitude::learn_bias`]) only when
/// its points pin the rotation to this, one sigma, each point good to the
/// grid resolution: where the scene leaves the rotation loose the matched
/// heading is the prior's or noise, not the gyro's error.
const BIAS_MAX_ROTATION_SIGMA_RAD: f64 = 0.1 * std::f64::consts::PI / 180.0;
/// A match teaches the gyro its z bias only while the heading turns slower
/// than this, rad/s (20 degrees/s). Through a fast turn the heading the
/// scans and the gyro disagree on is the gyro's scale error and the IMU's
/// timing, which grow with the rate; at a bias's 0.1 degree/s they would
/// swamp it.
const BIAS_MAX_TURN_RATE: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// The translation directions the last match left unconstrained.
#[derive(Debug, Clone, Copy)]
struct Unobserved {
    /// Projector onto them, local frame.
    free: Matrix2<f64>,
    /// Local-frame prior velocity when they were last observed, and when.
    vel: Vector2<f64>,
    since_ns: u64,
}

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
    /// Translation `eig_min / eig_max` of the scan's point-to-feature
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
    /// Time, pose and IMU heading of the last scan.
    last: Option<(u64, Pose2, f64)>,
    /// Matched poses of the last [`VELOCITY_WINDOW_NS`], oldest first.
    history: VecDeque<(u64, Pose2)>,
    /// Set while the latest match is degenerate.
    unobserved: Option<Unobserved>,
    /// Measured velocities ([`Self::feed_velocity`]), levelled LiDAR frame,
    /// oldest first.
    measured: VecDeque<(u64, Vector2<f64>)>,
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
            unobserved: None,
            measured: VecDeque::new(),
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

    /// The gyro bias about the LiDAR's z axis the matches have taught the
    /// attitude so far, rad/s ([`Attitude::learn_bias`]).
    pub fn gyro_bias_z(&self) -> f64 {
        self.att.gyro_bias_z()
    }

    pub fn feed_imu(&mut self, s: &ImuSample) {
        self.att.update(s);
    }

    /// A measured ground velocity of the IMU frame (an optical-flow sensor
    /// over its range to the ground). While one is fresh it is the motion
    /// prior's velocity in every direction: the scans refine the pose where
    /// they constrain it, and along a direction they do not (a corridor's
    /// axis) the pose follows the measured motion instead of coasting.
    /// Levelled with the IMU's roll and pitch at arrival, so feed the IMU up
    /// to the sample's time first. Dropped before the first IMU sample or
    /// when tilted too far to level.
    pub fn feed_velocity(&mut self, s: &VelocitySample) {
        let Some(v) = self.att.level_velocity(s.v) else {
            return;
        };
        if self.measured.back().is_some_and(|&(t, _)| t >= s.t_ns) {
            return;
        }
        self.measured.push_back((s.t_ns, Vector2::new(v[0], v[1])));
        let horizon = s.t_ns.saturating_sub(VELOCITY_KEEP_NS);
        while self.measured.front().is_some_and(|&(t, _)| t < horizon) {
            self.measured.pop_front();
        }
    }

    /// The mean measured velocity over `(from, to]`, else the newest no more
    /// than [`MAX_VELOCITY_AGE_NS`] before `to`.
    fn measured_velocity(&self, from: u64, to: u64) -> Option<Vector2<f64>> {
        let (sum, n) = self
            .measured
            .iter()
            .filter(|&&(t, _)| t > from && t <= to)
            .fold((Vector2::zeros(), 0usize), |(s, n), &(_, v)| (s + v, n + 1));
        if n > 0 {
            return Some(sum / n as f64);
        }
        self.measured
            .iter()
            .rev()
            .find(|&&(t, _)| t <= to)
            .filter(|&&(t, _)| to - t <= MAX_VELOCITY_AGE_NS)
            .map(|&(_, v)| v)
    }

    /// Match one scan and return its pose.
    pub fn feed_scan(&mut self, input: &ScanInput) -> PoseOut {
        let t = input.t_ns;
        let (roll, pitch) = self.att.roll_pitch();
        let heading = self.att.heading_at(t);
        let span_ns = input.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
        let (prior, velocity, yaw_rate, prior_vel) = self.predict(t, heading, span_ns);
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
            self.advance(t, prior, heading);
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
            // The heading the scans turned since the last scan against the
            // gyro's, for the gyro's z bias; never the FC's fused yaw.
            let min_rotation_info = (self.cfg.resolution_m / BIAS_MAX_ROTATION_SIGMA_RAD).powi(2);
            if let Some((t1, _, _)) = self.last {
                if self.att.has_data()
                    && yaw_rate.abs() <= BIAS_MAX_TURN_RATE
                    && rotation_information(&info) >= min_rotation_info
                {
                    let dt = t.saturating_sub(t1) as f64 * 1e-9;
                    self.att.learn_bias(wrap(r.pose.th - pull.pose.th), dt);
                }
            }
            if deg < self.cfg.degenerate_ratio {
                in_submap = keep_constrained(in_submap, r.pose, &info, self.cfg.degenerate_ratio);
                // The information is in the submap frame.
                let rot = Rotation2::new(sm.pose.th);
                let free = rot.matrix()
                    * unconstrained_directions(&info, self.cfg.degenerate_ratio)
                    * rot.matrix().transpose();
                let (vel, since_ns) = self
                    .unobserved
                    .map_or((prior_vel, t), |u| (u.vel, u.since_ns));
                self.unobserved = Some(Unobserved {
                    free,
                    vel,
                    since_ns,
                });
            } else {
                in_submap = r.pose;
                self.unobserved = None;
            }
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
        self.advance(t, pose, heading);
        out
    }

    /// Prior pose, LiDAR-frame velocity, the heading rate over the scan's
    /// `span_ns`, and the local-frame velocity of the prediction, from the
    /// IMU's heading and a velocity: the measured one
    /// ([`Self::feed_velocity`]) over the interval while it is fresh, else
    /// the mean over the recent poses.
    /// The velocity is kept in the LiDAR frame (the chord over the window is
    /// expressed at its middle heading) so it turns with the walker through a
    /// corner. Without a measured velocity, along directions the last match
    /// left unconstrained the chord only replays the prior, so the velocity
    /// there is the one last observed, fading over [`COAST_TAU_S`].
    fn predict(&self, t: u64, heading: f64, span_ns: u64) -> (Pose2, [f64; 2], f64, Vector2<f64>) {
        let imu_rate = if span_ns > 0 {
            (self.att.heading_at(t + span_ns) - heading) / (span_ns as f64 * 1e-9)
        } else {
            self.att.heading_rate()
        };
        let Some((t1, p1, heading1)) = self.last else {
            return (Pose2::IDENTITY, [0.0, 0.0], imu_rate, Vector2::zeros());
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
            (heading - heading1, imu_rate)
        } else {
            (omega * dt, omega)
        };
        let heading = Rotation2::new(p1.th + 0.5 * dyaw);
        let measured = self.measured_velocity(t1, t + span_ns);
        let mut v = heading * measured.unwrap_or(Vector2::new(vel[0], vel[1]));
        if let (None, Some(u)) = (measured, &self.unobserved) {
            let fade = (-(t.saturating_sub(u.since_ns) as f64 * 1e-9) / COAST_TAU_S).exp();
            v += u.free * (u.vel * fade - v);
        }
        let vel = heading.inverse() * v;
        (
            Pose2::new(p1.x + v[0] * dt, p1.y + v[1] * dt, p1.th + dyaw),
            [vel[0], vel[1]],
            rate,
            v,
        )
    }

    fn advance(&mut self, t: u64, pose: Pose2, heading: f64) {
        self.last = Some((t, pose, heading));
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
