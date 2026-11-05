//! [`FcEkfPose`]: the flight controller's EKF as a pose provider.
//!
//! Inputs are the raw `ATTITUDE_QUATERNION` or `ATTITUDE`,
//! `LOCAL_POSITION_NED`, `GPS_GLOBAL_ORIGIN` and `GPS_RAW_INT` frames the
//! plugin host delivers (`mavlink.read`). One estimate is produced per
//! position message, paired with the latest attitude no more than
//! [`MAX_PAIR_NS`] older on the FC clock.
//!
//! GNSS: `GPS_RAW_INT` carries no boot time, so its fix is placed at its
//! position in the stream. An estimate counts as GNSS-referenced
//! ([`FcEkfPose::gnss_steady_at`]) once every `GPS_RAW_INT` since a time
//! [`GNSS_SETTLE_MS`] earlier on the FC clock carried a 3D fix.
//!
//! Time: the FC stamps `time_boot_ms`. The FC-to-host offset is the minimum of
//! `arrival_mono − fc_time` over the last [`OFFSET_WINDOW`] messages (the
//! fastest delivery bounds the latency), so a pose is placed at the instant
//! the FC sampled it, not when the host read it.
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
use crate::mavlink_msg::{decode, AttitudeQuaternion, FcMessage, GpsGlobalOrigin, RawImu};
use crate::nav::align::GNSS_SETTLE_MS;

/// Largest FC-clock gap between the attitude and the position paired into one
/// estimate.
pub const MAX_PAIR_NS: u64 = 50_000_000;
/// Messages the FC→host offset is the minimum over.
pub const OFFSET_WINDOW: usize = 64;
/// GNSS state changes kept for [`FcEkfPose::gnss_steady_at`].
const GNSS_CHANGES: usize = 16;
/// `RAW_IMU` samples held for [`FcEkfPose::drain_imu`] (5 s at 100 Hz).
const IMU_QUEUE: usize = 512;

/// `GPS_FIX_TYPE_3D_FIX`.
const FIX_3D: u8 = 3;

/// The process-lifetime state the MAVLink subscriptions feed.
#[derive(Default)]
struct FeedInner {
    attitude: Option<AttitudeQuaternion>,
    origin: Option<GpsGlobalOrigin>,
    offsets: VecDeque<i128>,
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
    /// `RAW_IMU` samples not yet drained, oldest first.
    imu: VecDeque<RawImu>,
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
        let g = self.inner.lock();
        let att = g.attitude?;
        let offset = *g.offsets.iter().min()?;
        let t_ns = (att.time_boot_ms as i128 * 1_000_000 + offset).max(0) as u64;
        Some((t_ns, se3::matrix_from_quat(att.q)))
    }

    /// The `RAW_IMU` samples received since the last call, oldest first, on
    /// the host clock (`(host ns, sample)`). Empty until the FC streams it.
    pub fn drain_imu(&self) -> Vec<(u64, RawImu)> {
        let mut g = self.inner.lock();
        let Some(offset) = g.offsets.iter().min().copied() else {
            return Vec::new();
        };
        g.imu
            .drain(..)
            .map(|s| (((s.time_usec as i128) * 1_000 + offset).max(0) as u64, s))
            .collect()
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

    /// Fold one delivered raw frame, read at host monotonic `arrival_ns`.
    pub fn on_frame(&self, frame: &[u8], arrival_ns: u64) {
        let Some(msg) = decode(frame) else {
            return;
        };
        let mut g = self.inner.lock();
        match msg {
            FcMessage::Attitude(a) => {
                note_offset(&mut g.offsets, a.time_boot_ms, arrival_ns);
                g.fc_ms = Some(a.time_boot_ms);
                g.attitude = Some(a);
            }
            FcMessage::Origin(o) => {
                if g.origin != Some(o) {
                    g.origin = Some(o);
                    g.ned.clear();
                    self.track.reset();
                }
            }
            FcMessage::Imu(s) => {
                g.offsets.push_back(arrival_ns as i128 - s.time_usec as i128 * 1_000);
                while g.offsets.len() > OFFSET_WINDOW {
                    g.offsets.pop_front();
                }
                g.imu.push_back(s);
                while g.imu.len() > IMU_QUEUE {
                    g.imu.pop_front();
                }
            }
            FcMessage::GpsFix(fix) => {
                g.fix3d_since_ms = if fix >= FIX_3D {
                    g.fix3d_since_ms.or(g.fc_ms)
                } else {
                    None
                };
            }
            FcMessage::Position(p) => {
                note_offset(&mut g.offsets, p.time_boot_ms, arrival_ns);
                g.fc_ms = Some(p.time_boot_ms);
                let Some(att) = g.attitude else {
                    return;
                };
                let fc_ns = |ms: u32| ms as i128 * 1_000_000;
                if (fc_ns(p.time_boot_ms) - fc_ns(att.time_boot_ms)).unsigned_abs()
                    > MAX_PAIR_NS as u128
                {
                    return;
                }
                let offset = *g.offsets.iter().min().expect("noted above");
                let t_ns = (fc_ns(p.time_boot_ms) + offset).max(0) as u64;
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
                let cov = g.cov.unwrap_or_else(|| super::diagonal_cov(1.0, 0.05));
                drop(g);
                self.track.push(PoseEstimate {
                    t_ns,
                    r,
                    t,
                    cov,
                    source: PoseSource::FcEkf,
                    health: VioHealth::Good,
                });
            }
        }
    }
}

fn note_offset(offsets: &mut VecDeque<i128>, time_boot_ms: u32, arrival_ns: u64) {
    offsets.push_back(arrival_ns as i128 - time_boot_ms as i128 * 1_000_000);
    while offsets.len() > OFFSET_WINDOW {
        offsets.pop_front();
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mavlink_msg::frame_v2;
    use ados_capture_contract::{GeoAnchorDatum, GeoAnchorSource};

    fn att(ms: u32, q: [f32; 4]) -> Vec<u8> {
        let mut p = ms.to_le_bytes().to_vec();
        for v in q.iter().chain([0.0f32; 3].iter()) {
            p.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(31, &p)
    }

    fn pos(ms: u32, ned: [f32; 3]) -> Vec<u8> {
        let mut p = ms.to_le_bytes().to_vec();
        for v in ned.iter().chain([0.0f32; 3].iter()) {
            p.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(32, &p)
    }

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

    /// `RAW_IMU` samples queue on the FC clock and drain once each, placed on
    /// the host clock by the same least-delay offset as the attitude.
    #[test]
    fn raw_imu_samples_drain_on_the_host_clock() {
        let p = FcEkfPose::new();
        let imu = |usec: u64, gz_mrad: i16| {
            let mut b = usec.to_le_bytes().to_vec();
            for v in [0i16, 0, -1000, 0, 0, gz_mrad, 0, 0, 0] {
                b.extend_from_slice(&v.to_le_bytes());
            }
            frame_v2(27, &b)
        };
        assert!(p.drain_imu().is_empty());
        // The FC clock runs 4 s behind the host's, with 2 ms least delay.
        p.on_frame(&att(1_000, [1.0, 0.0, 0.0, 0.0]), 5_002_000_000);
        p.on_frame(&imu(1_010_000, 500), 5_015_000_000);
        p.on_frame(&imu(1_020_000, 600), 5_022_000_000);
        let got = p.drain_imu();
        assert_eq!(got.iter().map(|(t, _)| *t).collect::<Vec<_>>(), [5_012_000_000, 5_022_000_000]);
        assert!((got[1].1.gyro[2] - 0.6).abs() < 1e-12);
        assert!(p.drain_imu().is_empty(), "drained once");
    }
}
