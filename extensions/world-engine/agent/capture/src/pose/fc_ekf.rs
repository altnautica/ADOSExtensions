//! [`FcEkfPose`]: the flight controller's EKF as a pose provider.
//!
//! Inputs are the raw `ATTITUDE_QUATERNION`, `LOCAL_POSITION_NED` and
//! `GPS_GLOBAL_ORIGIN` frames the plugin host delivers (`mavlink.read`). One
//! estimate is produced per position message, paired with the latest
//! attitude no more than [`MAX_PAIR_NS`] older on the FC clock.
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
use crate::mavlink_msg::{decode, AttitudeQuaternion, FcMessage, GpsGlobalOrigin};

/// Largest FC-clock gap between the attitude and the position paired into one
/// estimate.
pub const MAX_PAIR_NS: u64 = 50_000_000;
/// Messages the FC→host offset is the minimum over.
pub const OFFSET_WINDOW: usize = 64;

/// The process-lifetime state the MAVLink subscriptions feed.
#[derive(Default)]
struct FeedInner {
    attitude: Option<AttitudeQuaternion>,
    origin: Option<GpsGlobalOrigin>,
    offsets: VecDeque<i128>,
    anchor: Option<GeoAnchor>,
    cov: Option<[f64; POSE_COV_LEN]>,
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

    /// Fold one delivered raw frame, read at host monotonic `arrival_ns`.
    pub fn on_frame(&self, frame: &[u8], arrival_ns: u64) {
        let Some(msg) = decode(frame) else {
            return;
        };
        let mut g = self.inner.lock();
        match msg {
            FcMessage::Attitude(a) => {
                note_offset(&mut g.offsets, a.time_boot_ms, arrival_ns);
                g.attitude = Some(a);
            }
            FcMessage::Origin(o) => {
                if g.origin != Some(o) {
                    g.origin = Some(o);
                    self.track.reset();
                }
            }
            FcMessage::Position(p) => {
                note_offset(&mut g.offsets, p.time_boot_ms, arrival_ns);
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
        // An attitude too old to pair yields nothing.
        p.on_frame(&pos(2_000, [2.0, 0.0, 0.0]), 6_000_000_000);
        assert_eq!(p.latest().unwrap().t_ns, 5_020_000_000);
        assert!(p.local_frame());
    }
}
