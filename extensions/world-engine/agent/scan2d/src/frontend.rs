//! Local SLAM: IMU attitude, scan matching into the active submaps, the
//! motion filter and the submap lifecycle. A plain struct; the caller owns the
//! threading.

use std::collections::VecDeque;
use std::f64::consts::PI;

use nalgebra::{Matrix2, Rotation2, Vector2};

use crate::attitude::{Attitude, ImuSample, VelocitySample};
use crate::config::Scan2dConfig;
use crate::backend::PYRAMID_DEPTH;
use crate::grid::Pyramid;
use crate::matcher::{
    branch_and_bound, constraint_information, correlative, degeneracy, keep_constrained, refine,
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
/// A height older than this at a scan says nothing about the scan's climb.
const MAX_HEIGHT_AGE_NS: u64 = 500_000_000;
/// A climb that has paused this long is over: the next one starts afresh.
const CLIMB_END_NS: u64 = 2_000_000_000;
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
/// Horizontal one-sigma uncertainty of a matched pose, metres
/// ([`PoseOut::sigma_xy_m`]).
pub(crate) const SIGMA_MATCHED_M: f64 = 0.05;
/// While scans ride the motion prior the uncertainty grows by this fraction
/// of the distance the prior carries the pose.
const SIGMA_GROWTH_PER_M: f64 = 0.5;
/// The uncertainty stops growing here, metres.
const SIGMA_MAX_M: f64 = 5.0;
/// The IMU covers a scan once its newest sample is no more than this before
/// the scan's time (one sample of a 50 Hz IMU).
const IMU_COVER_SLACK_NS: u64 = 20_000_000;
/// Heading consistency ([`PoseOut::heading_ok`]): over the last
/// [`HEADING_WINDOW_NS`] the matched heading may move against the integrated
/// gyro's by at most this, radians (2 degrees), ...
const HEADING_JUMP_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
/// ... plus the fastest turn in the window times this, seconds: the scan
/// and IMU clocks agree to a few tens of milliseconds, which through a 70
/// degree/s pivot moves each match against the gyro by a degree or two.
const HEADING_RATE_SLACK_S: f64 = 0.03;
const HEADING_WINDOW_NS: u64 = 1_000_000_000;
/// After a gap in the IMU (a turn across it is a guess, see
/// [`Attitude::measured_since`]), the
/// heading is unconfirmed ([`PoseOut::heading_unconfirmed`]) until
/// [`CONFIRM_SCANS`] scans in a row find the same heading against the map
/// the scans saw before it: each scan's best heading by a search over every
/// turn the gap could have hidden, agreeing within this of the first's
/// (each against the gyro-held heading, so the gyro carries them between
/// scans; 1 degree), ...
const HEADING_CONFIRM_RAD: f64 = std::f64::consts::PI / 180.0;
/// ... while the heading turns slower than this, rad/s (20 degrees/s;
/// through a faster turn the scan's motion and the clocks blur its heading).
const HEADING_CONFIRM_MAX_TURN_RATE: f64 = 20.0 * std::f64::consts::PI / 180.0;
/// Consecutive scans that must agree before the heading is confirmed.
const CONFIRM_SCANS: usize = 3;
/// The fastest yaw an unmeasured stretch is taken to hide, rad/s (180
/// degrees/s): the search spans this times the stretch, plus
/// [`CONFIRM_MARGIN_RAD`], up to the whole circle.
const MAX_UNMEASURED_YAW_RATE: f64 = std::f64::consts::PI;
const CONFIRM_MARGIN_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
/// How far the position may have moved off its prior while unconfirmed,
/// metres (the search's half-width): a scan matched at a heading off by the
/// turn the gap hid drags the position with it.
const CONFIRM_LINEAR_M: f64 = 3.0;
/// A heading is a candidate only when it scores at least
/// [`Scan2dConfig::loop_min_score`] and no heading more than
/// [`CONFIRM_RIVAL_RAD`] from it within the search scores within this of
/// it: a room whose outline repeats under a turn (a rectangle under half a
/// turn) leaves the heading ambiguous, and it stays unconfirmed.
const CONFIRM_SCORE_MARGIN: f64 = 0.1;
const CONFIRM_RIVAL_RAD: f64 = 10.0 * std::f64::consts::PI / 180.0;

/// The translation directions the last match left unconstrained.
#[derive(Debug, Clone, Copy)]
struct Unobserved {
    /// Projector onto them, local frame.
    free: Matrix2<f64>,
    /// Local-frame prior velocity when they were last observed, and when.
    vel: Vector2<f64>,
    since_ns: u64,
}

/// A climb or descent in progress ([`LocalSlam::skip_climbing`]).
#[derive(Debug, Clone, Copy)]
struct Climb {
    /// The height it began at, metres.
    from_m: f64,
    /// Its scans are matched again in the submap of the height it began at:
    /// it has gone past [`Scan2dConfig::max_unmatched_climb_m`] (a transit,
    /// whose shaft walls draw the same outline at every height).
    transit: bool,
    /// Its scans are matched again band by band, each height in a submap of
    /// its own: it has ridden the prior for
    /// [`Scan2dConfig::max_unmatched_climb_s`] without a measured velocity.
    unmeasured: bool,
    /// The first and the last scan time it was climbing at.
    start_ns: u64,
    last_ns: u64,
}

/// The height of the scan plane, up, and its rate. The submap bands and
/// the climb rule use only its changes; floor rejection
/// ([`crate::scan::Motion::agl_m`]) reads it as the height above the floor.
/// The capture service feeds the flight controller's EKF height, whose
/// origin is near the take-off floor and which leaves out the LiDAR's mount
/// height: the floor margin is then larger by that height, which drops low
/// returns from matching, never adds a false wall.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeightSample {
    /// Nanoseconds, the same clock as the scans.
    pub t_ns: u64,
    /// Metres.
    pub up_m: f64,
    /// Metres per second, positive climbing.
    pub up_rate_mps: f64,
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
    /// Horizontal one-sigma uncertainty, metres. While scans ride the motion
    /// prior unmatched it grows by half the distance the prior carries the
    /// pose (speed times interval), and while a climb without a measured
    /// velocity is matched band by band by half the height climbed; up to
    /// 5.0. A matched scan is otherwise as good as the submap it matched in:
    /// 0.05 in a submap started at a matched pose, the start pose's own
    /// sigma in one the prior placed (the submap of a new height after a
    /// climb), never less than 0.05.
    pub sigma_xy_m: f64,
    /// Whether the integrated gyro vouches for the heading. False when the
    /// IMU had not reached the scan (its stream stalled or is starved: the
    /// heading is held at the newest sample's, the scan matched for its
    /// position only and kept out of the map), while
    /// [`Self::heading_unconfirmed`], or when the matched heading moved
    /// against the gyro's by more than 2 degrees (plus 0.03 s times the turn
    /// rate) within a second: a jump the gyro did not see. With no IMU fed at
    /// all there is no gyro to hold it to, and it is true.
    pub heading_ok: bool,
    /// The heading went unmeasured (a gap in the IMU) and the scans have not
    /// yet
    /// re-confirmed it against the map: [`CONFIRM_SCANS`] scans in a row
    /// found one heading, unambiguously, over every turn the gap could have
    /// hidden. Until then the heading may be off by any turn the gap hid,
    /// and is no navigation heading. Confirming re-seeds the heading to the
    /// one the scans found.
    pub heading_unconfirmed: bool,
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
    /// The newest height ([`Self::feed_height`]).
    height: Option<HeightSample>,
    /// The climb in progress ([`Self::skip_climbing`]).
    climb: Option<Climb>,
    /// The last pose's [`PoseOut::sigma_xy_m`].
    sigma_xy_m: f64,
    /// `(t_ns, map heading − gyro heading, |turn rate|)` of the scans of the
    /// last [`HEADING_WINDOW_NS`] the gyro measured, oldest first.
    gyro_offsets: VecDeque<(u64, f64, f64)>,
    /// Set by a gap in the IMU, cleared once the scans re-confirm the
    /// heading ([`HEADING_CONFIRM_RAD`]).
    heading_suspect: bool,
    /// How long the heading went unmeasured since it was last confirmed,
    /// ns; saturating.
    unmeasured_ns: u64,
    /// The re-confirming scans so far: each one's best map heading against
    /// the gyro-held one, radians.
    confirm_offsets: Vec<f64>,
    /// The pyramid of the submap the re-confirmation searches, by submap
    /// id; the submap takes no scans while the heading is suspect.
    confirm_pyramid: Option<(usize, Pyramid)>,
    /// The first submap id started since the heading went suspect: those
    /// hold the suspect heading and confirm nothing.
    suspect_from_id: usize,
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
            height: None,
            climb: None,
            sigma_xy_m: SIGMA_MATCHED_M,
            gyro_offsets: VecDeque::new(),
            heading_suspect: false,
            unmeasured_ns: 0,
            confirm_offsets: Vec::new(),
            confirm_pyramid: None,
            suspect_from_id: 0,
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

    /// The heading went unmeasured for an unknown time, whatever the IMU
    /// samples say: they carried no yaw rate (a stand-in fed from a fused
    /// attitude). The map the scans built meanwhile placed them at a heading
    /// pinned through any turn, so it cannot re-confirm one: it is dropped,
    /// and the next scan starts a new map at the current pose. The new map's
    /// heading against the old one (and the world) is unknown by any turn
    /// the stretch hid; the caller re-anchors it. Call it in order with the
    /// samples, before the first one that measures the turn again.
    pub fn heading_unmeasured(&mut self) {
        for (_, done) in self.active.drain(..) {
            if self.record {
                self.finished.push(done);
            }
        }
        self.history.clear();
        self.gyro_offsets.clear();
        self.unobserved = None;
        self.last_insert = None;
        self.heading_suspect = false;
        self.unmeasured_ns = 0;
        self.confirm_offsets.clear();
        self.confirm_pyramid = None;
    }

    /// The heading went unmeasured for `span_ns`.
    fn suspect(&mut self, span_ns: u64) {
        if !self.heading_suspect {
            self.heading_suspect = true;
            self.suspect_from_id = self.next_id;
        }
        self.unmeasured_ns = self.unmeasured_ns.saturating_add(span_ns);
        self.confirm_offsets.clear();
    }

    /// One scan's step in re-confirming a suspect heading against the
    /// active submap, if that submap was started before the heading went
    /// unmeasured (one started since holds the suspect heading itself). The
    /// scan's best pose over every turn the stretch could have hidden
    /// ([`MAX_UNMEASURED_YAW_RATE`]), if it is unambiguous; once
    /// [`CONFIRM_SCANS`] in a row agree, the heading is confirmed and
    /// re-seeded there, and the confirming scan's pose in the submap is
    /// returned.
    fn reconfirm(&mut self, scan: &Scan2d, prior: Pose2) -> Option<Pose2> {
        let found = self.confirm_candidate(scan, prior);
        let Some((g, off)) = found else {
            self.confirm_offsets.clear();
            return None;
        };
        if self
            .confirm_offsets
            .first()
            .is_some_and(|&o| wrap(off - o).abs() > HEADING_CONFIRM_RAD)
        {
            self.confirm_offsets.clear();
        }
        self.confirm_offsets.push(off);
        if self.confirm_offsets.len() < CONFIRM_SCANS {
            return None;
        }
        self.heading_suspect = false;
        self.unmeasured_ns = 0;
        self.confirm_offsets.clear();
        self.confirm_pyramid = None;
        // The heading jumps to the confirmed one: the motion and the gyro
        // comparison restart from it.
        self.gyro_offsets.clear();
        self.history.clear();
        self.unobserved = None;
        Some(g)
    }

    /// The scan's unambiguous best pose in the active submap over the
    /// unmeasured stretch's turns, and its heading against the prior's.
    fn confirm_candidate(&mut self, scan: &Scan2d, prior: Pose2) -> Option<(Pose2, f64)> {
        let (id, sm) = self.active.first()?;
        if sm.scans() == 0 || *id >= self.suspect_from_id {
            return None;
        }
        let id = *id;
        if self.confirm_pyramid.as_ref().is_none_or(|(i, _)| *i != id) {
            self.confirm_pyramid = Some((id, Pyramid::new(&sm.grid, PYRAMID_DEPTH)));
        }
        let (_, pyr) = self.confirm_pyramid.as_ref()?;
        let p = sm.pose.inverse() * prior;
        let half = (self.unmeasured_ns as f64 * 1e-9 * MAX_UNMEASURED_YAW_RATE
            + CONFIRM_MARGIN_RAD)
            .min(PI);
        let search = |lo: f64, hi: f64, min_score: f64| {
            let window = Window {
                linear_m: CONFIRM_LINEAR_M,
                angular_rad: 0.5 * (hi - lo),
            };
            let center = Pose2::new(p.x, p.y, 0.5 * (lo + hi));
            branch_and_bound(pyr, scan, center, window, min_score)
        };
        let (g, score) = search(p.th - half, p.th + half, self.cfg.loop_min_score)?;
        // Any heading clear of the best that scores near it makes the room
        // ambiguous under the turn.
        let rival_floor = score - CONFIRM_SCORE_MARGIN;
        let d = g.th - p.th;
        let rival = if half >= PI {
            search(g.th + CONFIRM_RIVAL_RAD, g.th + 2.0 * PI - CONFIRM_RIVAL_RAD, rival_floor)
                .is_some()
        } else {
            [(-half, d - CONFIRM_RIVAL_RAD), (d + CONFIRM_RIVAL_RAD, half)]
                .into_iter()
                .filter(|(lo, hi)| hi > lo)
                .any(|(lo, hi)| search(p.th + lo, p.th + hi, rival_floor).is_some())
        };
        (!rival).then_some((g, wrap(d)))
    }

    /// The scan plane's height and climb rate. While it climbs or descends
    /// faster than [`Scan2dConfig::max_climb_rate_mps`] a scan is neither
    /// matched nor mapped: its pose is the motion prior. A submap takes scans
    /// only from within [`Scan2dConfig::submap_height_band_m`] of the height
    /// it started at: once the height leaves that band, the active submaps
    /// are finished and a new one starts at the next scan's pose, which that
    /// scan's motion prior places. Feed it in time order with the IMU;
    /// without heights neither applies. A submap started before the first
    /// height takes that height as its own (the height source, such as an
    /// EKF that waits for this pose, often comes up after the matcher).
    pub fn feed_height(&mut self, s: &HeightSample) {
        let finite = s.up_m.is_finite() && s.up_rate_mps.is_finite();
        if !finite || self.height.is_some_and(|h| h.t_ns > s.t_ns) {
            return;
        }
        self.height = Some(*s);
        for (_, sm) in &mut self.active {
            sm.height_m.get_or_insert(s.up_m);
        }
    }

    /// Finish the active submaps when the height has left the band of the
    /// one scans are matched against. All of them, oldest first, so finished
    /// submaps stay in creation order. Not inside a transit past
    /// [`Scan2dConfig::max_unmatched_climb_m`]: a shaft's walls draw the same
    /// outline at every height, and a submap a band deep holds too few scans
    /// to match against at a transit's speed.
    fn leave_other_height(&mut self) {
        if self.climb.is_some_and(|c| c.transit) {
            return;
        }
        let (Some(h), Some((_, sm))) = (self.height, self.active.first()) else {
            return;
        };
        if sm
            .height_m
            .is_none_or(|h0| (h.up_m - h0).abs() <= self.cfg.submap_height_band_m)
        {
            return;
        }
        for (_, done) in self.active.drain(..) {
            if self.record {
                self.finished.push(done);
            }
        }
    }

    /// Whether the scan plane is climbing or descending faster than
    /// [`Scan2dConfig::max_climb_rate_mps`] at scan time `t`, by a height no
    /// older than [`MAX_HEIGHT_AGE_NS`].
    fn climbing(&self, t: u64) -> bool {
        self.height.is_some_and(|h| {
            t.abs_diff(h.t_ns) <= MAX_HEIGHT_AGE_NS
                && h.up_rate_mps.abs() > self.cfg.max_climb_rate_mps
        })
    }

    /// Whether a scan at `t` rides the motion prior because the scan plane
    /// is climbing, mostly straight up or down (faster than the horizontal
    /// `speed_mps` it moves at): a take-off or a step in height, up to
    /// [`Scan2dConfig::max_unmatched_climb_m`] from where the climb began.
    /// Past that the climb is a transit (a shaft) whose walls the motion
    /// prior cannot hold the pose against, and its scans are matched until
    /// the climb has paused for [`CLIMB_END_NS`]. Without a measured
    /// velocity (`has_measured`) the prior cannot see a sideways drift, so
    /// once the climb has ridden the prior for
    /// [`Scan2dConfig::max_unmatched_climb_s`] its scans are matched too,
    /// band by band in submaps of their own height
    /// ([`Self::leave_other_height`] still applies). A leg that climbs as it
    /// travels (terrain following) is matched throughout: over its length
    /// the prior drifts further than its slope shifts the outline.
    fn skip_climbing(&mut self, t: u64, speed_mps: f64, has_measured: bool) -> bool {
        let vertical = |h: &HeightSample| h.up_rate_mps.abs() >= speed_mps;
        let Some(h) = self.height.filter(|h| self.climbing(t) && vertical(h)) else {
            if self.climb.is_some_and(|c| t.saturating_sub(c.last_ns) > CLIMB_END_NS) {
                self.climb = None;
            }
            return false;
        };
        let c = self.climb.get_or_insert(Climb {
            from_m: h.up_m,
            transit: false,
            unmeasured: false,
            start_ns: t,
            last_ns: t,
        });
        c.last_ns = t;
        c.transit |= (h.up_m - c.from_m).abs() > self.cfg.max_unmatched_climb_m;
        // Saturates: an infinite bound never ends the ride.
        let unmeasured_ns = (self.cfg.max_unmatched_climb_s * 1e9) as u64;
        c.unmeasured |= !has_measured && t.saturating_sub(c.start_ns) > unmeasured_ns;
        !(c.transit || c.unmeasured)
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
        // A scan the IMU has not reached (its stream stalled or is starved
        // while the scans keep coming) holds the heading of the newest
        // sample, never the last rate extrapolated past it without bound. It
        // is matched for its position, but unchecked, kept out of the map
        // and the bias, so the next scan the IMU covers turns from that held
        // heading by what the gyro measured.
        let gyro = self.att.has_data();
        let newest = self.att.newest_ns().filter(|_| gyro);
        let reached = newest.is_none_or(|n| n + IMU_COVER_SLACK_NS >= t);
        let heading = self.att.heading_at(match newest {
            Some(n) if !reached => n,
            _ => t,
        });
        let span_ns = input.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
        let (prior, velocity, yaw_rate, prior_vel) = self.predict(t, heading, span_ns);
        let motion = Motion {
            roll,
            pitch,
            yaw_rate,
            velocity,
            agl_m: self
                .height
                .filter(|h| t.abs_diff(h.t_ns) <= MAX_HEIGHT_AGE_NS)
                .map(|h| h.up_m),
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
            sigma_xy_m: SIGMA_MATCHED_M,
            heading_ok: false,
            heading_unconfirmed: false,
        };
        // Riding the prior: it carried the pose `speed × dt` since the last
        // scan without a match to check it.
        let dt_s = self.last.map_or(0.0, |(t1, _, _)| t.saturating_sub(t1) as f64 * 1e-9);
        let riding_sigma =
            (self.sigma_xy_m + SIGMA_GROWTH_PER_M * prior_vel.norm() * dt_s).min(SIGMA_MAX_M);
        // Whether the gyro measured the turn since the reference scan: a gap
        // in the IMU bridges it with a guess. Scans before the first sample
        // are no gap: the heading starts there, and nothing was lost.
        let measured = !gyro || self.last.is_none_or(|(t1, _, _)| self.att.measured_since(t1));
        if !measured && self.att.last_gap_ns() > 0 {
            self.suspect(self.att.last_gap_ns());
        }
        let has_measured = self
            .measured_velocity(self.last.map_or(t, |(t1, _, _)| t1), t + span_ns)
            .is_some();
        if scan.points.len() < MIN_POINTS || self.skip_climbing(t, prior_vel.norm(), has_measured)
        {
            let consistent = self.check_heading(t, prior.th, heading, yaw_rate, measured);
            out.heading_ok = reached && consistent && !self.heading_suspect;
            out.heading_unconfirmed = self.heading_suspect;
            self.advance(t, prior, heading);
            self.sigma_xy_m = riding_sigma;
            out.sigma_xy_m = riding_sigma;
            return out;
        }
        // Matched through a climb it could not ride (no measured velocity),
        // band by band: a wall that leans moves the outline with height, so
        // the match may move with each metre climbed.
        let climb_growth = match (self.climb, self.height) {
            (Some(c), Some(h)) if c.unmeasured && !c.transit && self.climbing(t) => {
                SIGMA_GROWTH_PER_M * h.up_rate_mps.abs() * dt_s
            }
            _ => 0.0,
        };
        let climbed_sigma = (self.sigma_xy_m + climb_growth).min(SIGMA_MAX_M);
        self.leave_other_height();
        if self.active.is_empty() {
            self.start_submap(prior, riding_sigma.max(climbed_sigma));
        }
        // A heading the IMU left unmeasured is re-confirmed against the map
        // the scans saw before, never taken from a match that holds to it.
        let confirmed = if self.heading_suspect
            && reached
            && yaw_rate.abs() <= HEADING_CONFIRM_MAX_TURN_RATE
        {
            self.reconfirm(&scan, prior)
        } else {
            None
        };
        let (id, sm) = &self.active[0];
        let mut pose = prior;
        let mut in_submap = confirmed.unwrap_or_else(|| sm.pose.inverse() * prior);
        if sm.scans() > 0 {
            let window = Window {
                linear_m: self.cfg.linear_window_m,
                angular_rad: self.cfg.angular_window_deg.to_radians(),
            };
            let (c, _) = correlative(&sm.grid, &scan, in_submap, window);
            // The match holds to the gyro's heading: the gyro integrates a
            // turn far better than a scan resolves one, and in a tunnel or a
            // chamber whose outline changes with height a scan can score
            // clearly better several degrees off for seconds.
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
                    && measured
                    && reached
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
            // Matched against the submap: as good as the submap's own pose,
            // or, climbing, as the climb has made it.
            out.sigma_xy_m = sm.anchor_sigma_m.max(SIGMA_MATCHED_M);
            if climb_growth > 0.0 {
                out.sigma_xy_m = out.sigma_xy_m.max(climbed_sigma);
            }
        } else {
            // A new submap's first scan is placed by the prior, unchecked.
            out.sigma_xy_m = riding_sigma.max(climbed_sigma);
        }
        let home = *id;
        (out.x, out.y, out.yaw) = (pose.x, pose.y, pose.th);
        self.sigma_xy_m = out.sigma_xy_m;
        // A new submap takes its first scan whatever the motion filter says,
        // so the next scan has something to match against. A scan on a
        // heading the IMU did not measure (not reached, or suspect since a
        // gap) stays out of the map, so the map keeps the heading the scans
        // can still confirm it against.
        let insert = (reached && !self.heading_suspect && self.should_insert(t, pose))
            || self.active[0].1.scans() == 0;
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
        let consistent = self.check_heading(t, pose.th, heading, yaw_rate, measured);
        out.heading_ok = reached && consistent && !self.heading_suspect;
        out.heading_unconfirmed = self.heading_suspect;
        self.advance(t, pose, heading);
        out
    }

    /// [`PoseOut::heading_ok`] of a scan at `t` whose map heading is `th`,
    /// the gyro's `heading` and turn `yaw_rate`; `measured` is whether the
    /// gyro measured the turn since the reference scan (a gap restarts the
    /// comparison). With no IMU at all there is no gyro to hold the heading
    /// to, and it passes.
    fn check_heading(
        &mut self,
        t: u64,
        th: f64,
        heading: f64,
        yaw_rate: f64,
        measured: bool,
    ) -> bool {
        if !self.att.has_data() {
            return true;
        }
        if !measured {
            self.gyro_offsets.clear();
        }
        let off = wrap(th - heading);
        self.gyro_offsets.push_back((t, off, yaw_rate.abs()));
        let horizon = t.saturating_sub(HEADING_WINDOW_NS);
        while self.gyro_offsets.front().is_some_and(|&(t0, _, _)| t0 < horizon) {
            self.gyro_offsets.pop_front();
        }
        let (mut lo, mut hi, mut rate) = (0.0f64, 0.0f64, 0.0f64);
        for &(_, o, r) in &self.gyro_offsets {
            let d = wrap(o - off);
            (lo, hi, rate) = (lo.min(d), hi.max(d), rate.max(r));
        }
        measured && hi - lo <= HEADING_JUMP_RAD + HEADING_RATE_SLACK_S * rate
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
            (self.att.heading_at(t + span_ns) - self.att.heading_at(t)) / (span_ns as f64 * 1e-9)
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
            self.start_submap(pose, self.sigma_xy_m);
        }
    }

    fn start_submap(&mut self, pose: Pose2, sigma_m: f64) {
        let height = self.height.map(|h| h.up_m);
        self.active
            .push((self.next_id, Submap::new(pose, height, sigma_m, &self.cfg)));
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
