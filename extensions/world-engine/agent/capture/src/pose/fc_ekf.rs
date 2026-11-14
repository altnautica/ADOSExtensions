//! [`FcEkfPose`]: the flight controller's EKF as a pose provider.
//!
//! Inputs are the raw `ATTITUDE_QUATERNION` or `ATTITUDE`,
//! `LOCAL_POSITION_NED`, `GPS_GLOBAL_ORIGIN` and `GPS_RAW_INT` frames the
//! plugin host delivers (`mavlink.read`). One estimate is produced per
//! position message, paired with the latest attitude no more than
//! [`MAX_PAIR_NS`] older on the FC clock.
//!
//! The feed also hands the planar matcher, through the sink a run sets
//! ([`FcEkfPose::set_nav_sink`]), each `RAW_IMU` sample, each ground velocity
//! of its `OPTICAL_FLOW` over its downward `DISTANCE_SENSOR` range and each
//! EKF height the moment the frame arrives ([`NavSample`]), so a scan waiting
//! for the IMU to cover it is matched as soon as the covering sample lands.
//!
//! GNSS: `GPS_RAW_INT` carries no boot time, so its fix is placed at its
//! position in the stream. An estimate counts as GNSS-referenced
//! ([`FcEkfPose::gnss_steady_at`]) once every `GPS_RAW_INT` since a time
//! [`GNSS_SETTLE_MS`] earlier on the FC clock carried a 3D fix.
//!
//! Time: the FC stamps `time_boot_ms`. The FC-to-host offset is the minimum of
//! `arrival_mono − fc_time` over the last [`OFFSET_WINDOW`] messages (the
//! fastest delivery bounds the latency), so a pose is placed at the instant
//! the FC sampled it, not when the host read it. The offset falls at once to
//! a faster delivery and rises no faster than the measured clock drift plus
//! [`MAX_OFFSET_RISE`] of the host time that passes ([`FcClock`]), so a
//! delivery stall longer than the window keeps the delayed burst at the FC's
//! own spacing, and a simulator below real time stays on the host clock.
//!
//! Frames: the EKF works in NED at its origin with an FRD body. The world is
//! the session ENU frame at the session anchor: position goes NED → ECEF (at
//! the EKF origin, ellipsoidal height = origin MSL + geoid) → ENU (anchor),
//! rotation `R_world_body = R_anchor_origin · R_ENU_NED · R_NED_FRD · R_FRD_FLU`.
//! With no anchor or no EKF origin (indoors) the world is the EKF's own frame
//! turned to ENU, and every estimate says so through `local_frame()`.

use std::collections::VecDeque;
use std::sync::Arc;

use ados_capture_contract::GeoAnchor;
use parking_lot::Mutex;
use world_engine_protocol::atlas::{PoseSource, VioHealth, POSE_COV_LEN};

use super::{se3, PoseEstimate, PoseProvider, PoseTrack};
use crate::geodesy::{EnuFrame, R_ENU_NED, R_FRD_FLU};
use crate::mavlink_msg::{
    decode, AttitudeQuaternion, FcMessage, GpsGlobalOrigin, LocalPositionNed, RawImu,
};
use crate::nav::align::GNSS_SETTLE_MS;
use crate::nav::fc_flow;

/// Largest FC-clock gap between the attitude and the position paired into one
/// estimate.
pub const MAX_PAIR_NS: u64 = 50_000_000;
/// Messages the FC→host offset is the minimum over.
pub const OFFSET_WINDOW: usize = 64;
/// Fastest the FC-to-host offset rises, per unit of host time, beyond the
/// measured drift ([`FcClock`]): 5 %, so a burst read within microseconds
/// moves it by nothing.
pub const MAX_OFFSET_RISE: f64 = 0.05;
/// Host time each drift bucket holds the least offset over.
const DRIFT_BUCKET_NS: u64 = 1_000_000_000;
/// Drift buckets kept, and the fewest the drift is measured from.
const DRIFT_BUCKETS: usize = 8;
const DRIFT_MIN_BUCKETS: usize = 4;
/// Fastest measured drift, per unit of host time: an FC clock (a
/// simulator's) running as slow as half the host's. One running faster
/// makes the offset fall, which it follows at once.
const MAX_DRIFT: f64 = 0.5;
/// An FC clock that steps back further than this has restarted: the offset
/// starts over. An offset falling further than this at once is a step of
/// the FC clock too: the drift starts over.
const FC_RESTART_NS: i128 = 1_000_000_000;
/// GNSS state changes kept for [`FcEkfPose::gnss_steady_at`].
const GNSS_CHANGES: usize = 16;

/// A sample the planar matcher takes from the FC, handed on as its frame
/// arrives ([`FcEkfPose::set_nav_sink`]). Times are host monotonic ns.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavSample {
    /// One `RAW_IMU` sample, with the FC's newest attitude
    /// `(host ns, R_ned_frd)` for its gravity ([`crate::nav::fc_imu`]).
    Imu {
        t_ns: u64,
        /// When its frame reached the host, monotonic ns.
        arrival_ns: u64,
        imu: RawImu,
        attitude: Option<(u64, [f64; 9])>,
    },
    /// The optical-flow ground velocity, body FLU m/s
    /// ([`fc_flow::body_velocity`]).
    Velocity { t_ns: u64, v: [f64; 2] },
    /// The EKF's position (`LOCAL_POSITION_NED`): north, east in its own
    /// local frame, m, and the height and climb rate, up, m and m/s.
    Position {
        t_ns: u64,
        ne_m: [f64; 2],
        up_m: f64,
        up_rate_mps: f64,
    },
}

/// Where [`NavSample`]s go: called on the MAVLink delivery thread, in
/// arrival order, with no lock held. It must not block.
pub type NavSink = Arc<dyn Fn(NavSample) + Send + Sync>;

/// `GPS_FIX_TYPE_3D_FIX`.
const FIX_3D: u8 = 3;

/// The FC-to-host clock offset: the least `arrival − fc_time` over the last
/// [`OFFSET_WINDOW`] messages, taken at once when it falls and followed at
/// no more than the measured drift plus [`MAX_OFFSET_RISE`] of the elapsed
/// host time when it rises.
///
/// The bound is what keeps a stalled delivery honest. When the frames stop
/// for longer than the window (the companion's MAVLink fan-out stalled for
/// over half a second in a bench flight) and then arrive in one burst, the
/// window holds only delayed frames, so its minimum is the newest frame's
/// own delay: a plain minimum stamps each burst frame at its arrival, the
/// FC's 10 ms `RAW_IMU` spacing collapses to microseconds, and the gyro
/// integrated from those stamps loses the turn the burst measured.
///
/// The drift is an FC clock running slower than the host's: a simulator
/// below real time (0.8 of it in a loaded bench flight). The least offset
/// then rises steadily with every frame on time, and a fixed bound would
/// leave each sample further behind the host clock, read as a stalled feed
/// and scans waiting on IMU seconds old. It is the median slope of the
/// least offset of each [`DRIFT_BUCKET_NS`] of host time over the last
/// [`DRIFT_BUCKETS`] (a stall's burst moves one bucket, not the median),
/// within `[0, MAX_DRIFT]`. Each windowed offset is carried forward by the
/// drift to the newest arrival before the minimum is taken, so the window
/// no longer lags a rising offset by its own length; the newest frame is
/// still in it, so no stamp is later than its arrival. When the drift is
/// first measured (or grows), the offset may rise once by the growth over
/// the host time it was measured across: the lag gathered while the drift
/// was not yet known.
#[derive(Debug, Default)]
struct FcClock {
    /// `(offset, arrival host ns)` of the last [`OFFSET_WINDOW`] messages.
    window: VecDeque<(i128, u64)>,
    /// The offset in use, the host time it was last moved at and the newest
    /// FC time.
    est: Option<(i128, u64, i128)>,
    /// `(bucket start host ns, least offset)`, oldest first; the last one
    /// is still filling.
    buckets: VecDeque<(u64, i128)>,
    /// The drift over the finished buckets, measured as each one finishes.
    drift: f64,
    /// One-off rise allowed beyond the rate: the lag a newly measured drift
    /// gathered before it was known.
    catch_up: i128,
}

impl FcClock {
    /// One frame the FC stamped `fc_ns`, read at host `arrival_ns`.
    fn note(&mut self, fc_ns: i128, arrival_ns: u64) {
        if self.est.is_some_and(|(_, _, f)| fc_ns + FC_RESTART_NS < f) {
            self.window.clear();
            self.est = None;
            self.restart_drift();
        }
        let offset = arrival_ns as i128 - fc_ns;
        if self.est.is_some_and(|(o, _, _)| offset + FC_RESTART_NS < o) {
            self.restart_drift();
        }
        self.note_bucket(offset, arrival_ns);
        self.window.push_back((offset, arrival_ns));
        while self.window.len() > OFFSET_WINDOW {
            self.window.pop_front();
        }
        let drift = self.drift;
        let least = self
            .window
            .iter()
            .map(|&(o, a)| o + ((arrival_ns.saturating_sub(a)) as f64 * drift) as i128)
            .min()
            .expect("one pushed");
        let rise_rate = MAX_OFFSET_RISE + drift;
        let catch_up = std::mem::take(&mut self.catch_up);
        self.est = Some(match self.est {
            Some((o, a, f)) if least > o => {
                let rise = (arrival_ns.saturating_sub(a) as f64 * rise_rate) as i128 + catch_up;
                ((o + rise).min(least), arrival_ns, fc_ns.max(f))
            }
            Some((_, _, f)) => (least, arrival_ns, fc_ns.max(f)),
            None => (least, arrival_ns, fc_ns),
        });
    }

    fn restart_drift(&mut self) {
        self.buckets.clear();
        self.drift = 0.0;
        self.catch_up = 0;
    }

    fn note_bucket(&mut self, offset: i128, arrival_ns: u64) {
        match self.buckets.back_mut() {
            Some((start, least)) if arrival_ns < *start + DRIFT_BUCKET_NS => {
                *least = (*least).min(offset);
            }
            _ => {
                let (drift, span_ns) = self.finished_drift();
                if drift > self.drift {
                    self.catch_up = ((drift - self.drift) * span_ns as f64) as i128;
                }
                self.drift = drift;
                self.buckets.push_back((arrival_ns, offset));
                while self.buckets.len() > DRIFT_BUCKETS {
                    self.buckets.pop_front();
                }
            }
        }
    }

    /// How fast the least offset rises per unit of host time over the
    /// buckets so far, all finished: the median of the pairwise slopes, 0
    /// with too few; and the host time those buckets span.
    fn finished_drift(&self) -> (f64, u64) {
        let b = &self.buckets;
        if b.len() < DRIFT_MIN_BUCKETS {
            return (0.0, 0);
        }
        let mut slopes = [0.0; DRIFT_BUCKETS * (DRIFT_BUCKETS - 1) / 2];
        let mut n = 0;
        for i in 0..b.len() {
            for j in i + 1..b.len() {
                slopes[n] = (b[j].1 - b[i].1) as f64 / (b[j].0 - b[i].0) as f64;
                n += 1;
            }
        }
        let slopes = &mut slopes[..n];
        slopes.sort_by(f64::total_cmp);
        let span_ns = b[b.len() - 1].0 - b[0].0;
        (slopes[n / 2].clamp(0.0, MAX_DRIFT), span_ns)
    }

    /// The FC time `fc_ns` on the host clock.
    fn host_ns(&self, fc_ns: i128) -> Option<u64> {
        let (offset, _, _) = self.est?;
        Some((fc_ns + offset).max(0) as u64)
    }
}

/// The process-lifetime state the MAVLink subscriptions feed.
#[derive(Default)]
struct FeedInner {
    attitude: Option<AttitudeQuaternion>,
    origin: Option<GpsGlobalOrigin>,
    clock: FcClock,
    anchor: Option<GeoAnchor>,
    cov: Option<[f64; POSE_COV_LEN]>,
    /// FC boot ms of the newest attitude or position.
    fc_ms: Option<u32>,
    /// FC boot ms since which every `GPS_RAW_INT` has carried a 3D fix.
    fix3d_since_ms: Option<u32>,
    /// `(host ns, steady)` at each estimate where GNSS steadiness changed,
    /// oldest first.
    gnss: VecDeque<(u64, bool)>,
    /// `(host ns, north, east)` of each estimate in the EKF's own local frame
    /// (`LOCAL_POSITION_NED`), the last [`super::TRACK_HISTORY_NS`].
    ned: VecDeque<(u64, [f64; 2])>,
    /// `(host ns, yaw)` of each attitude, radians clockwise from north,
    /// unwrapped (continuous across ±π), the last
    /// [`super::TRACK_HISTORY_NS`].
    yaws: VecDeque<(u64, f64)>,
    /// Newest downward ground range ([`fc_flow::ground_range`]): FC boot ms,
    /// metres.
    range: Option<(u32, f64)>,
    /// The run's matcher input ([`FcEkfPose::set_nav_sink`]).
    sink: Option<NavSink>,
}

impl FeedInner {
    /// The FC time `fc_ns` on the host clock ([`FcClock`]).
    fn host_ns(&self, fc_ns: i128) -> Option<u64> {
        self.clock.host_ns(fc_ns)
    }

    /// The newest attitude on the host clock: `(host ns, R_ned_frd)`.
    fn host_attitude(&self) -> Option<(u64, [f64; 9])> {
        let att = self.attitude?;
        let t_ns = self.host_ns(att.time_boot_ms as i128 * 1_000_000)?;
        Some((t_ns, se3::matrix_from_quat(att.q)))
    }
}

/// The FC EKF feed and provider. Cheap to clone; every clone shares state.
#[derive(Clone)]
pub struct FcEkfPose {
    inner: Arc<Mutex<FeedInner>>,
    track: Arc<PoseTrack>,
}

impl Default for FcEkfPose {
    fn default() -> Self {
        Self::new()
    }
}

impl FcEkfPose {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(FeedInner::default())),
            track: Arc::new(PoseTrack::new()),
        }
    }

    /// The world anchor estimates are placed against (`None` = local frame).
    /// Changing it starts a new track: earlier estimates are in another frame.
    pub fn set_anchor(&self, anchor: Option<GeoAnchor>) {
        let mut g = self.inner.lock();
        if g.anchor != anchor {
            g.anchor = anchor;
            self.track.reset();
        }
    }

    /// The covariance stated with every estimate.
    pub fn set_covariance(&self, cov: [f64; POSE_COV_LEN]) {
        self.inner.lock().cov = Some(cov);
    }

    /// Whether estimates are in the EKF's own frame rather than the anchor's.
    pub fn local_frame(&self) -> bool {
        let g = self.inner.lock();
        g.anchor.is_none() || g.origin.is_none()
    }

    /// The EKF origin, once the FC has reported one.
    pub fn origin(&self) -> Option<GpsGlobalOrigin> {
        self.inner.lock().origin
    }

    /// The newest FC attitude on the host clock: `(host ns, R_ned_frd)`,
    /// row-major. Known as soon as `ATTITUDE_QUATERNION` or `ATTITUDE` arrives, with or
    /// without an EKF position (a GNSS-denied FC reports none until the
    /// external navigation it waits for is fused). Its yaw is the FC's own
    /// heading source (the compass).
    pub fn attitude(&self) -> Option<(u64, [f64; 9])> {
        self.inner.lock().host_attitude()
    }

    /// Hand every `RAW_IMU` sample, flow velocity and EKF height to `sink`
    /// as its frame arrives ([`NavSample`]); `None` stops it. A run sets it
    /// while it runs: nothing is queued for a run that is not there.
    pub fn set_nav_sink(&self, sink: Option<NavSink>) {
        self.inner.lock().sink = sink;
    }

    /// Whether the EKF position at host `t_ns` was GNSS-referenced: at the
    /// newest estimate no later than `t_ns`, GNSS had held a 3D fix for
    /// [`GNSS_SETTLE_MS`] on the FC clock. `false` before any estimate.
    pub fn gnss_steady_at(&self, t_ns: u64) -> bool {
        let g = self.inner.lock();
        g.gnss
            .iter()
            .rev()
            .find(|(t, _)| *t <= t_ns)
            .is_some_and(|(_, steady)| *steady)
    }

    /// The EKF's horizontal position in its own local frame
    /// (`LOCAL_POSITION_NED` north, east) at host `t_ns`, interpolated
    /// between the estimates around it; `None` further than
    /// [`super::MAX_EXTRAPOLATION_NS`] outside them.
    pub fn ned_at(&self, t_ns: u64) -> Option<[f64; 2]> {
        let g = self.inner.lock();
        let i = g.ned.partition_point(|(t, _)| *t <= t_ns);
        let near = |(t, p): (u64, [f64; 2])| (t.abs_diff(t_ns) <= super::MAX_EXTRAPOLATION_NS).then_some(p);
        match (i.checked_sub(1).map(|j| g.ned[j]), g.ned.get(i).copied()) {
            (Some((t0, p0)), Some((t1, p1))) => {
                let a = (t_ns - t0) as f64 / (t1 - t0) as f64;
                Some([p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1])])
            }
            (Some(e), None) | (None, Some(e)) => near(e),
            (None, None) => None,
        }
    }

    /// The FC's attitude yaw (radians clockwise from north, within ±π) at
    /// host `t_ns`, interpolated between the attitudes around it; `None`
    /// further than [`super::MAX_EXTRAPOLATION_NS`] outside them.
    pub fn heading_at(&self, t_ns: u64) -> Option<f64> {
        let g = self.inner.lock();
        let i = g.yaws.partition_point(|(t, _)| *t <= t_ns);
        let near = |(t, y): (u64, f64)| (t.abs_diff(t_ns) <= super::MAX_EXTRAPOLATION_NS).then_some(y);
        let yaw = match (i.checked_sub(1).map(|j| g.yaws[j]), g.yaws.get(i).copied()) {
            (Some((t0, y0)), Some((t1, y1))) => Some(y0 + (t_ns - t0) as f64 / (t1 - t0) as f64 * (y1 - y0)),
            (Some(e), None) | (None, Some(e)) => near(e),
            (None, None) => None,
        }?;
        Some(crate::nav::wrap_pi(yaw))
    }

    /// Fold one delivered raw frame, read at host monotonic `arrival_ns`.
    pub fn on_frame(&self, frame: &[u8], arrival_ns: u64) {
        let Some(msg) = decode(frame) else {
            return;
        };
        let mut g = self.inner.lock();
        let mut estimate = None;
        let nav = match msg {
            FcMessage::Attitude(a) => {
                g.clock.note(a.time_boot_ms as i128 * 1_000_000, arrival_ns);
                g.fc_ms = Some(a.time_boot_ms);
                g.attitude = Some(a);
                if let Some((t_ns, r)) = g.host_attitude() {
                    let yaw = crate::nav::frd_yaw(&r);
                    let yaw = g.yaws.back().map_or(yaw, |&(_, y0)| {
                        y0 + (yaw - y0 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                            - std::f64::consts::PI
                    });
                    if g.yaws.back().is_none_or(|&(t, _)| t < t_ns) {
                        g.yaws.push_back((t_ns, yaw));
                    }
                    while g.yaws.front().is_some_and(|&(t, _)| t + super::TRACK_HISTORY_NS < t_ns) {
                        g.yaws.pop_front();
                    }
                }
                None
            }
            FcMessage::Origin(o) => {
                if g.origin != Some(o) {
                    g.origin = Some(o);
                    g.ned.clear();
                    self.track.reset();
                }
                None
            }
            FcMessage::Imu(s) => {
                g.clock.note(s.time_usec as i128 * 1_000, arrival_ns);
                g.host_ns(s.time_usec as i128 * 1_000).map(|t_ns| NavSample::Imu {
                    t_ns,
                    arrival_ns,
                    imu: s,
                    attitude: g.host_attitude(),
                })
            }
            FcMessage::Range(r) => {
                if let Some(m) = fc_flow::ground_range(&r) {
                    g.range = Some((r.time_boot_ms, m));
                }
                None
            }
            FcMessage::Flow(f) => g
                .fc_ms
                .and_then(|now| fc_flow::flow_time_ms(f.time_usec, now))
                .and_then(|ms| {
                    let (_, range) = g
                        .range
                        .filter(|(t, _)| t.abs_diff(ms) <= fc_flow::MAX_RANGE_SKEW_MS)?;
                    let v = fc_flow::body_velocity(&f, range)?;
                    let t_ns = g.host_ns(ms as i128 * 1_000_000)?;
                    Some(NavSample::Velocity { t_ns, v })
                }),
            FcMessage::GpsFix(fix) => {
                g.fix3d_since_ms = if fix >= FIX_3D {
                    g.fix3d_since_ms.or(g.fc_ms)
                } else {
                    None
                };
                None
            }
            FcMessage::Position(p) => {
                g.clock.note(p.time_boot_ms as i128 * 1_000_000, arrival_ns);
                g.fc_ms = Some(p.time_boot_ms);
                let position = g
                    .host_ns(p.time_boot_ms as i128 * 1_000_000)
                    .map(|t_ns| NavSample::Position {
                        t_ns,
                        ne_m: [p.ned[0], p.ned[1]],
                        up_m: -p.ned[2],
                        up_rate_mps: -p.vel[2],
                    });
                estimate = pair_estimate(&mut g, &p);
                position
            }
        };
        let sink = g.sink.clone();
        drop(g);
        if let Some(e) = estimate {
            self.track.push(e);
        }
        if let (Some(sink), Some(s)) = (sink, nav) {
            sink(s);
        }
    }
}

/// The estimate of EKF position `p` paired with the newest attitude, `None`
/// before an attitude or with one more than [`MAX_PAIR_NS`] away on the FC
/// clock. Notes the GNSS state and the EKF's own NED at it.
fn pair_estimate(g: &mut FeedInner, p: &LocalPositionNed) -> Option<PoseEstimate> {
    let att = g.attitude?;
    let fc_ns = |ms: u32| ms as i128 * 1_000_000;
    if (fc_ns(p.time_boot_ms) - fc_ns(att.time_boot_ms)).unsigned_abs() > MAX_PAIR_NS as u128 {
        return None;
    }
    let t_ns = g.host_ns(fc_ns(p.time_boot_ms)).expect("offset noted before pairing");
    let steady = g
        .fix3d_since_ms
        .is_some_and(|s| p.time_boot_ms.saturating_sub(s) >= GNSS_SETTLE_MS);
    if g.gnss.back().map(|(_, s)| *s) != Some(steady) {
        g.gnss.push_back((t_ns, steady));
        while g.gnss.len() > GNSS_CHANGES {
            g.gnss.pop_front();
        }
    }
    if g.ned.back().is_none_or(|(t, _)| *t < t_ns) {
        g.ned.push_back((t_ns, [p.ned[0], p.ned[1]]));
        let horizon = t_ns.saturating_sub(super::TRACK_HISTORY_NS);
        while g.ned.front().is_some_and(|(t, _)| *t < horizon) {
            g.ned.pop_front();
        }
    }
    let (r, t) = world_pose(&att.q, p.ned, g.origin.as_ref(), g.anchor.as_ref());
    Some(PoseEstimate {
        t_ns,
        r,
        t,
        cov: g.cov.unwrap_or_else(|| super::diagonal_cov(1.0, 0.05)),
        source: PoseSource::FcEkf,
        health: VioHealth::Good,
        sigma_xy_m: None,
    })
}

/// `R_world_body` and the body position for an EKF attitude (FRD→NED
/// quaternion) and NED position.
pub fn world_pose(
    q_ned_frd: &[f64; 4],
    ned: [f64; 3],
    origin: Option<&GpsGlobalOrigin>,
    anchor: Option<&GeoAnchor>,
) -> ([f64; 9], [f64; 3]) {
    let r_ned_frd = se3::matrix_from_quat(*q_ned_frd);
    let r_enu_flu = se3::mat_mul(&R_ENU_NED, &se3::mat_mul(&r_ned_frd, &R_FRD_FLU));
    let enu_local = se3::mat_vec(&R_ENU_NED, ned);
    match (origin, anchor) {
        (Some(o), Some(a)) => {
            let h = o.alt_msl_m + crate::geo::geoid_undulation_m(o.lat_deg, o.lon_deg);
            let ekf = EnuFrame::at(o.lat_deg, o.lon_deg, h);
            let world = EnuFrame::from_anchor(a);
            let t = world.ecef_to_enu(ekf.enu_to_ecef(enu_local));
            (se3::mat_mul(&world.r_from(&ekf), &r_enu_flu), t)
        }
        _ => (r_enu_flu, enu_local),
    }
}

impl PoseProvider for FcEkfPose {
    fn source(&self) -> PoseSource {
        PoseSource::FcEkf
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }
}

/// MAVLink frames for tests that feed an [`FcEkfPose`].
#[cfg(test)]
pub(crate) mod test_frames {
    use crate::mavlink_msg::frame_v2;

    /// `ATTITUDE_QUATERNION` at FC boot `ms`, `q` FRD→NED `[w, x, y, z]`.
    pub(crate) fn att(ms: u32, q: [f32; 4]) -> Vec<u8> {
        let mut p = ms.to_le_bytes().to_vec();
        for v in q.iter().chain([0.0f32; 3].iter()) {
            p.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(31, &p)
    }

    /// `LOCAL_POSITION_NED` at FC boot `ms`, at rest.
    pub(crate) fn pos(ms: u32, ned: [f32; 3]) -> Vec<u8> {
        let mut p = ms.to_le_bytes().to_vec();
        for v in ned.iter().chain([0.0f32; 3].iter()) {
            p.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(32, &p)
    }
}

#[cfg(test)]
mod tests {
    use super::test_frames::{att, pos};
    use super::*;
    use crate::mavlink_msg::frame_v2;
    use ados_capture_contract::{GeoAnchorDatum, GeoAnchorSource};

    #[test]
    fn level_north_facing_body_maps_to_enu_flu() {
        // Identity FRD→NED: nose north, right wing east, belly down.
        let (r, t) = world_pose(&[1.0, 0.0, 0.0, 0.0], [10.0, 5.0, -2.0], None, None);
        // Body x (forward) is world north = ENU y; body y (left) is west.
        assert_eq!(se3::mat_vec(&r, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_eq!(se3::mat_vec(&r, [0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0]);
        assert_eq!(se3::mat_vec(&r, [0.0, 0.0, 1.0]), [0.0, 0.0, 1.0]);
        assert_eq!(t, [5.0, 10.0, 2.0]);
    }

    #[test]
    fn origin_offset_from_the_anchor_is_carried_into_enu() {
        let origin = GpsGlobalOrigin {
            lat_deg: 12.9716,
            lon_deg: 77.5946,
            alt_msl_m: 900.0,
        };
        let n = crate::geo::geoid_undulation_m(12.9716, 77.5946);
        let anchor = GeoAnchor {
            lat_deg: 12.9716,
            lon_deg: 77.5946,
            h_ellipsoid_m: 900.0 + n,
            geoid_undulation_m: n,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::GnssMean,
        };
        let (_, t) = world_pose(
            &[1.0, 0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            Some(&origin),
            Some(&anchor),
        );
        assert!((t[0]).abs() < 1e-6 && (t[1] - 100.0).abs() < 1e-6, "{t:?}");
        // The earth curves away under a 100 m chord by ~0.8 mm.
        assert!(t[2] < 0.0 && t[2] > -0.002, "{t:?}");
    }

    /// A drone outside a capture session streams `ATTITUDE` and no
    /// quaternion: its heading must still reach the navigation lane, or the
    /// lidar SLAM pose is never north-aligned and never published.
    #[test]
    fn euler_attitude_alone_gives_the_heading() {
        let p = FcEkfPose::new();
        let mut b = 2_000u32.to_le_bytes().to_vec();
        for v in [0.0f32, 0.0, 3.0, 0.0, 0.0, 0.0] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        p.on_frame(
            &frame_v2(crate::mavlink_msg::MSG_ATTITUDE, &b),
            9_000_000_000,
        );
        let (t_ns, r) = p.attitude().expect("ATTITUDE sets the attitude");
        assert_eq!(t_ns, 9_000_000_000);
        assert!((crate::nav::frd_yaw(&r) - 3.0).abs() < 1e-6);
        p.on_frame(&pos(2_010, [1.0, 2.0, 0.0]), 9_010_000_000);
        assert!(
            p.latest().is_some(),
            "positions pair with the Euler attitude"
        );
    }

    #[test]
    fn positions_pair_with_attitude_and_are_stamped_on_the_fc_clock() {
        let p = FcEkfPose::new();
        p.on_frame(&pos(1_000, [0.0; 3]), 5_000_000_000);
        assert!(p.latest().is_none(), "no attitude yet");
        p.on_frame(&att(1_010, [1.0, 0.0, 0.0, 0.0]), 5_020_000_000);
        // Arrives 5 ms late; the earlier message set a lower offset.
        p.on_frame(&pos(1_020, [1.0, 0.0, 0.0]), 5_025_000_000);
        let e = p.latest().unwrap();
        // Offset = min(5.0e9 − 1.0e9, 5.02e9 − 1.01e9, 5.025e9 − 1.02e9) = 4.0e9.
        assert_eq!(e.t_ns, 5_020_000_000);
        assert_eq!(e.source, PoseSource::FcEkf);
        assert_eq!(e.t, [0.0, 1.0, 0.0]);
        assert_eq!(p.ned_at(5_020_000_000), Some([1.0, 0.0]), "the EKF's own NED");
        assert_eq!(p.ned_at(5_060_000_000), Some([1.0, 0.0]));
        assert_eq!(p.ned_at(5_100_000_000), None, "no extrapolation past 50 ms");
        // An attitude too old to pair yields nothing.
        p.on_frame(&pos(2_000, [2.0, 0.0, 0.0]), 6_000_000_000);
        assert_eq!(p.latest().unwrap().t_ns, 5_020_000_000);
        assert!(p.local_frame());
    }

    #[test]
    fn the_attitude_alone_gives_the_heading_without_a_position() {
        let p = FcEkfPose::new();
        assert!(p.attitude().is_none());
        // Yaw +90° about down: the nose faces east.
        let h = std::f32::consts::FRAC_PI_4;
        p.on_frame(&att(1_000, [h.cos(), 0.0, 0.0, h.sin()]), 5_000_000_000);
        let (t_ns, r) = p.attitude().expect("attitude without LOCAL_POSITION_NED");
        assert_eq!(t_ns, 5_000_000_000);
        assert!((r[3].atan2(r[0]) - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!(p.latest().is_none(), "no position, no pose estimate");
    }

    fn gps(fix: u8) -> Vec<u8> {
        let mut p = vec![0u8; 30];
        p[28] = fix;
        frame_v2(crate::mavlink_msg::MSG_GPS_RAW_INT, &p)
    }

    /// The EKF position counts as GNSS-referenced only after the fix has held
    /// for the settle time on the FC clock, and stops at the first report
    /// without one, whenever the host happens to read it.
    #[test]
    fn gnss_is_steady_from_the_settle_time_until_the_fix_drops() {
        let p = FcEkfPose::new();
        let host = |ms: u32| 1_000_000_000 + ms as u64 * 1_000_000;
        let step = |ms: u32| {
            p.on_frame(&att(ms, [1.0, 0.0, 0.0, 0.0]), host(ms));
            p.on_frame(&pos(ms, [0.0; 3]), host(ms));
        };
        step(0);
        assert!(!p.gnss_steady_at(host(0)), "no fix yet");
        p.on_frame(&gps(6), host(0));
        step(1_000);
        step(GNSS_SETTLE_MS - 1);
        assert!(!p.gnss_steady_at(host(GNSS_SETTLE_MS - 1)));
        step(GNSS_SETTLE_MS + 100);
        assert!(p.gnss_steady_at(host(GNSS_SETTLE_MS + 100)));
        assert!(p.gnss_steady_at(host(GNSS_SETTLE_MS + 150)), "until the next estimate");
        assert!(!p.gnss_steady_at(host(5_000)), "earlier estimates keep their state");
        p.on_frame(&gps(1), host(GNSS_SETTLE_MS + 200));
        step(GNSS_SETTLE_MS + 300);
        assert!(!p.gnss_steady_at(host(GNSS_SETTLE_MS + 300)));
        p.on_frame(&gps(3), host(GNSS_SETTLE_MS + 400));
        step(GNSS_SETTLE_MS + 500);
        assert!(!p.gnss_steady_at(host(GNSS_SETTLE_MS + 500)), "the settle time restarts");
        step(2 * GNSS_SETTLE_MS + 400);
        assert!(p.gnss_steady_at(host(2 * GNSS_SETTLE_MS + 400)));
    }

    /// The samples a sink receives, in order.
    fn collect(p: &FcEkfPose) -> Arc<Mutex<Vec<NavSample>>> {
        let got = Arc::new(Mutex::new(Vec::new()));
        let sink = got.clone();
        p.set_nav_sink(Some(Arc::new(move |s| sink.lock().push(s))));
        got
    }

    /// Each `RAW_IMU` sample reaches the sink as its frame arrives, placed
    /// on the host clock by the same least-delay offset as the attitude and
    /// carrying that attitude; with no sink nothing is kept for later.
    #[test]
    fn raw_imu_samples_reach_the_sink_on_arrival() {
        let p = FcEkfPose::new();
        let imu = |usec: u64, gz_mrad: i16| {
            let mut b = usec.to_le_bytes().to_vec();
            for v in [0i16, 0, -1000, 0, 0, gz_mrad, 0, 0, 0] {
                b.extend_from_slice(&v.to_le_bytes());
            }
            frame_v2(27, &b)
        };
        // The FC clock runs 4 s behind the host's, with 2 ms least delay.
        p.on_frame(&att(1_000, [1.0, 0.0, 0.0, 0.0]), 5_002_000_000);
        p.on_frame(&imu(1_005_000, 400), 5_008_000_000);
        let fed = collect(&p);
        p.on_frame(&imu(1_010_000, 500), 5_015_000_000);
        assert_eq!(fed.lock().len(), 1, "handed on at arrival, the earlier one never");
        p.on_frame(&imu(1_020_000, 600), 5_022_000_000);
        let got = fed.lock().clone();
        let times: Vec<u64> = got
            .iter()
            .map(|s| match s {
                NavSample::Imu { t_ns, .. } => *t_ns,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(times, [5_012_000_000, 5_022_000_000]);
        let NavSample::Imu {
            imu: raw, attitude, ..
        } = got[1]
        else {
            unreachable!()
        };
        assert!((raw.gyro[2] - 0.6).abs() < 1e-12);
        assert_eq!(attitude.map(|(t, _)| t), Some(5_002_000_000));
        p.set_nav_sink(None);
        p.on_frame(&imu(1_030_000, 700), 5_032_000_000);
        assert_eq!(fed.lock().len(), 2, "no sink, nothing handed on");
    }

    fn raw_imu_frame(usec: u64) -> Vec<u8> {
        let mut b = usec.to_le_bytes().to_vec();
        for v in [0i16, 0, -1000, 0, 0, 0, 0, 0, 0] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(27, &b)
    }

    /// The sink's `(t_ns, arrival_ns)` of each `RAW_IMU` sample.
    fn imu_times(got: &Mutex<Vec<NavSample>>) -> Vec<(u64, u64)> {
        got.lock()
            .iter()
            .filter_map(|s| match s {
                NavSample::Imu { t_ns, arrival_ns, .. } => Some((*t_ns, *arrival_ns)),
                _ => None,
            })
            .collect()
    }

    /// A simulator running at 0.8 of real time: `RAW_IMU` at 100 Hz on the
    /// FC clock arrives steadily, 12.5 ms apart on the host's. The measured
    /// drift keeps each sample at its arrival less the 2 ms delay, where a
    /// bound of 5 % alone left the newest 1.4 s behind after 10 s.
    #[test]
    fn a_simulator_below_real_time_stays_on_the_host_clock() {
        let p = FcEkfPose::new();
        let got = collect(&p);
        for k in 0..1_000u64 {
            let fc_us = 1_000_000 + k * 10_000;
            p.on_frame(&raw_imu_frame(fc_us), 5_002_000_000 + k * 12_500_000);
        }
        let times = imu_times(&got);
        let late: Vec<_> = times[500..]
            .iter()
            .map(|(t, a)| a.saturating_sub(*t) as f64 * 1e-6)
            .collect();
        let worst = late.iter().copied().fold(0.0, f64::max);
        assert!(worst < 50.0, "a sample placed {worst:.0} ms before its arrival");
        let spacing = times[999].0 - times[998].0;
        assert!((spacing as i64 - 12_500_000).abs() < 2_000_000, "spacing {spacing} ns");
    }

    /// A delivery stall that bursts keeps the FC's spacing, drift or not.
    #[test]
    fn a_stall_then_a_burst_keeps_the_fc_spacing() {
        let p = FcEkfPose::new();
        let got = collect(&p);
        let mut k = 0u64;
        let send = |k: u64, host: u64| p.on_frame(&raw_imu_frame(1_000_000 + k * 10_000), host);
        // 6 s steady, then 1 s nothing, then that second's 100 samples at once.
        while k < 600 {
            send(k, 5_002_000_000 + k * 10_000_000);
            k += 1;
        }
        for j in 0..100 {
            send(k + j, 5_002_000_000 + 7_000_000_000 + j * 1_000);
        }
        let times = imu_times(&got);
        let burst = &times[600..];
        let spacing = (burst[99].0 - burst[0].0) as f64 / 99.0;
        assert!((spacing - 10e6).abs() < 1e6, "burst spacing {spacing} ns");
    }

    /// An FC clock that steps (a reboot back to zero, or a jump forward) is
    /// taken at once, not smoothed over by the drift.
    #[test]
    fn a_clock_step_is_taken_at_once() {
        let p = FcEkfPose::new();
        let got = collect(&p);
        // 8 s at 0.8 of real time: the drift is measured.
        for k in 0..640u64 {
            p.on_frame(&raw_imu_frame(60_000_000 + k * 10_000), 5_002_000_000 + k * 12_500_000);
        }
        // Reboot: FC time back to 1 s, read at the host's 20 s. The first
        // frame after a step is the only offset known, so it is placed at
        // its own arrival; the old offset would put it 55 s out.
        p.on_frame(&raw_imu_frame(1_000_000), 20_002_000_000);
        let (t, a) = *imu_times(&got).last().expect("one");
        assert_eq!(a, t, "the reboot starts the offset over");
        // A jump forward by 30 s: the offset falls at once.
        p.on_frame(&raw_imu_frame(31_010_000), 20_012_000_000);
        let (t, a) = *imu_times(&got).last().expect("one");
        assert_eq!(a, t, "the jump is taken at once");
        // With the drift started over, a stall and burst keep the spacing:
        // 100 samples 10 ms apart on the FC clock, read within 100 µs.
        for j in 1..=100u64 {
            p.on_frame(&raw_imu_frame(31_010_000 + j * 10_000), 21_500_000_000 + j * 1_000);
        }
        let times = imu_times(&got);
        let n = times.len();
        let spacing = (times[n - 1].0 - times[n - 100].0) as f64 / 99.0;
        assert!((spacing - 10e6).abs() < 1e6, "burst spacing {spacing} ns after the step");
    }

    /// `OPTICAL_FLOW` (ArduPilot: boot ms in `time_usec`) is scaled by the
    /// downward range read within 300 ms of it and reaches the sink as a body
    /// FLU velocity on the host clock; a reading out of the rangefinder's
    /// band or a stale one scales nothing.
    #[test]
    fn flow_over_the_downward_range_reaches_the_sink_as_a_body_velocity() {
        let p = FcEkfPose::new();
        let got = collect(&p);
        let flow = |ms: u64, comp: [f32; 2]| {
            let mut b = ms.to_le_bytes().to_vec();
            for v in [comp[0], comp[1], 0.0] {
                b.extend_from_slice(&v.to_le_bytes());
            }
            b.extend_from_slice(&[0, 0, 0, 0, 0, 255]);
            frame_v2(crate::mavlink_msg::MSG_OPTICAL_FLOW, &b)
        };
        let range = |ms: u32, cm: u16| {
            let mut b = ms.to_le_bytes().to_vec();
            for v in [5u16, 1200, cm] {
                b.extend_from_slice(&v.to_le_bytes());
            }
            b.extend_from_slice(&[0, 0, fc_flow::DOWNWARD, 0]);
            frame_v2(crate::mavlink_msg::MSG_DISTANCE_SENSOR, &b)
        };
        let velocities = || -> Vec<(u64, [f64; 2])> {
            got.lock()
                .iter()
                .map(|s| match s {
                    NavSample::Velocity { t_ns, v } => (*t_ns, *v),
                    other => panic!("{other:?}"),
                })
                .collect()
        };
        // The FC clock runs 4 s behind the host's, with 2 ms least delay.
        p.on_frame(&att(1_000, [1.0, 0.0, 0.0, 0.0]), 5_002_000_000);
        p.on_frame(&flow(1_005, [0.0, 1.0]), 5_010_000_000);
        assert!(velocities().is_empty(), "no range yet");
        p.on_frame(&range(1_010, 200), 5_015_000_000);
        p.on_frame(&flow(1_020, [-0.5, 1.0]), 5_025_000_000);
        p.on_frame(&range(1_030, 1200), 5_035_000_000);
        p.on_frame(&flow(1_040, [0.0, 2.0]), 5_045_000_000);
        let v = velocities();
        assert_eq!(v.len(), 2, "{v:?}");
        assert_eq!(v[0].0, 5_022_000_000);
        assert!((v[0].1[0] - 2.0).abs() < 1e-6 && (v[0].1[1] + 1.0).abs() < 1e-6);
        assert!((v[1].1[0] - 4.0).abs() < 1e-6, "the out-of-band reading is skipped");
        p.on_frame(&att(1_400, [1.0, 0.0, 0.0, 0.0]), 5_402_000_000);
        p.on_frame(&flow(1_400, [0.0, 1.0]), 5_405_000_000);
        assert_eq!(velocities().len(), 2, "the range is 390 ms old");
    }

    /// Every `LOCAL_POSITION_NED` reaches the sink as the EKF's position,
    /// height and climb rate, up, whether or not it pairs into an estimate.
    #[test]
    fn positions_reach_the_sink() {
        let p = FcEkfPose::new();
        let got = collect(&p);
        let pos_vel = |ms: u32, ned: [f32; 3], vel: [f32; 3]| {
            let mut b = ms.to_le_bytes().to_vec();
            for v in ned.iter().chain(vel.iter()) {
                b.extend_from_slice(&v.to_le_bytes());
            }
            frame_v2(32, &b)
        };
        p.on_frame(&pos_vel(1_000, [0.0, 0.0, -2.5], [0.0, 0.0, -0.75]), 5_002_000_000);
        assert!(p.latest().is_none(), "no attitude: no estimate");
        p.on_frame(&att(1_010, [1.0, 0.0, 0.0, 0.0]), 5_012_000_000);
        p.on_frame(&pos_vel(1_020, [1.0, -3.0, -2.0], [0.0, 0.0, 0.5]), 5_022_000_000);
        assert!(p.latest().is_some());
        let got = got.lock().clone();
        assert_eq!(
            got,
            [
                NavSample::Position {
                    t_ns: 5_002_000_000,
                    ne_m: [0.0, 0.0],
                    up_m: 2.5,
                    up_rate_mps: 0.75
                },
                NavSample::Position {
                    t_ns: 5_022_000_000,
                    ne_m: [1.0, -3.0],
                    up_m: 2.0,
                    up_rate_mps: -0.5
                },
            ]
        );
    }

    /// The attitude's yaw is kept with its host time: the yaw at a past
    /// instant through a fast turn, across the ±180° seam, is the one the FC
    /// reported then, not the newest.
    #[test]
    fn the_heading_at_a_past_instant_is_the_one_reported_then() {
        let p = FcEkfPose::new();
        let yaw_q = |deg: f64| {
            let h = deg.to_radians() / 2.0;
            [h.cos() as f32, 0.0, 0.0, h.sin() as f32]
        };
        // 90°/s from 170°, every 20 ms, through 180° to -100° (260°).
        for k in 0..=50u32 {
            let ms = 1_000 + 20 * k;
            p.on_frame(&att(ms, yaw_q(170.0 + 1.8 * k as f64)), ms as u64 * 1_000_000);
        }
        let deg = |t_ms: u64| p.heading_at(t_ms * 1_000_000).map(f64::to_degrees);
        let newest = crate::nav::frd_yaw(&p.attitude().expect("attitude").1).to_degrees();
        assert!((newest + 100.0).abs() < 1e-3, "{newest}");
        let at = deg(1_510).expect("inside the history");
        assert!((at - (170.0 + 45.9 - 360.0)).abs() < 1e-3, "{at}");
        let seam = deg(1_110).expect("inside the history");
        assert!((seam - 179.9).abs() < 1e-3, "{seam}");
        assert!(deg(1_020 + 1_000).is_some(), "near the newest");
        assert!(deg(3_000).is_none(), "past the newest by more than the extrapolation");
    }
}
