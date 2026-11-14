//! The IMU stream of a companion with no IMU of its own (a drone whose
//! planar LiDAR sits on the companion and whose IMU is on the FC), taken
//! from the flight controller. Set `atlas.lidar.imu = "fc"` and declare a rig
//! IMU sensor `fc` mounted at the body origin.
//!
//! The FC's `RAW_IMU` (its calibrated gyro, body FRD) is the stream's rate,
//! turned into FLU ([`from_raw`]). The planar matcher integrates the gyro
//! for its rotation prior, so the prior turns with the airframe and nothing
//! else: the EKF's yaw corrections never reach it. That matters with
//! external navigation, where the EKF fuses the matcher's own pose; a prior
//! that followed the fused yaw would feed every heading error back into the
//! matcher.
//!
//! The stream's specific force is gravity in the attitude the FC last
//! reported, not `RAW_IMU`'s accelerometer. A multirotor's accelerometer
//! reads its thrust, which leans off the vertical by whatever the airframe
//! is accelerating at: braking into and out of a turn at the end of an
//! aisle it leant 15 to 20 degrees, the matcher's attitude filter levelled
//! with it, and the heading of the levelled frame came out of the turn 2 to
//! 4 degrees off. The FC's EKF attitude has the manoeuvre taken out (it
//! fuses velocity), and gravity carries none of its yaw.
//!
//! Until `RAW_IMU` arrives (an FC that has not been asked to stream it, or
//! not yet), the stream is synthesised from consecutive FC EKF attitudes
//! ([`sample`]: `R` world-from-body, ENU world, FLU body; the specific force
//! of a vehicle not accelerating is gravity in the body, `Rᵀ·(0, 0, g)`).
//! Its body rate is the turn of the attitude with its yaw taken out: roll
//! and pitch only, never the EKF's yaw. Under external navigation the EKF's
//! yaw is the matcher's own heading fused back; a stand-in carrying it closes
//! a loop in which every degree the EKF turns towards the matcher turns the
//! matcher a degree further, and the heading spins with the drone standing
//! still. Without a measured yaw rate the matcher's heading holds still
//! between scans, so the capture reports its pose degraded until `RAW_IMU`
//! streams ([`FeedState::degraded`]).
//!
//! Delivery: [`sink`] hands each `RAW_IMU` sample, flow velocity and EKF
//! height to the pose provider on the MAVLink delivery thread as its frame
//! arrives ([`crate::pose::fc_ekf::FcEkfPose::set_nav_sink`]), not on the
//! capture loop's pose tick. The planar matcher holds a scan until the IMU
//! covers its last point, so the scan is matched one `RAW_IMU` period after
//! it ends rather than up to a tick later. [`FeedState`] keeps the count and
//! the newest sample time for the capture loop's `nav_imu_feed` log.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use ados_capture_contract::v1::{ImuBatch, ImuSample};
use ados_scan2d::{HeightSample, VelocitySample};

use crate::geodesy::{R_ENU_NED, R_FRD_FLU};
use crate::mavlink_msg::RawImu;
use crate::pose::fc_ekf::{NavSample, NavSink};
use crate::pose::{se3, PoseProvider};
use world_engine_protocol::atlas::{ImuFeedReason, ImuFeedState, VioHealth};

/// The `atlas.lidar.imu` id that selects this stream.
pub const FC_IMU_ID: &str = "fc";
const G: f64 = 9.81;
/// Attitudes further apart than this start the stream over.
const MAX_GAP_NS: u64 = 500_000_000;

/// The rotation vector (axis × angle) of a row-major rotation.
fn rotation_vector(r: &[f64; 9]) -> [f64; 3] {
    let cos = ((r[0] + r[4] + r[8] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos.acos();
    let v = [r[7] - r[5], r[2] - r[6], r[3] - r[1]];
    let s = angle.sin();
    if s.abs() < 1e-9 {
        // Small angle: half the skew part is the vector.
        return v.map(|x| 0.5 * x);
    }
    let k = angle / (2.0 * s);
    v.map(|x| k * x)
}

/// A `RAW_IMU` sample at host `t_ns` in the FLU body: `(x, -y, -z)` of the
/// FRD rate. Its specific force is gravity in `attitude` (host time and
/// `R` world-from-body, ENU world, FLU body), the FC's latest, when that is
/// no more than [`MAX_GAP_NS`] from the sample; without one, the
/// accelerometer's own reading.
pub fn from_raw(t_ns: u64, s: &RawImu, attitude: Option<(u64, [f64; 9])>) -> ImuSample {
    let flu = |v: [f64; 3]| vec![v[0], -v[1], -v[2]];
    let accel = match attitude {
        Some((t, r)) if t.abs_diff(t_ns) <= MAX_GAP_NS => {
            se3::mat_vec(&se3::transpose(&r), [0.0, 0.0, G]).to_vec()
        }
        _ => flu(s.accel),
    };
    ImuSample {
        t_ns,
        gyro: flu(s.gyro),
        accel,
        ..Default::default()
    }
}

/// The sample at `cur` given the previous attitude, `None` for the first one
/// or after a gap. Its rate is the turn between the two attitudes with
/// their yaw taken out ([`without_yaw`]): the stand-in never carries the
/// EKF's yaw.
pub fn sample(prev: Option<(u64, [f64; 9])>, cur: (u64, [f64; 9])) -> Option<ImuSample> {
    let (t0, r0) = prev?;
    let (t1, r1) = cur;
    if t1 <= t0 || t1 - t0 > MAX_GAP_NS {
        return None;
    }
    let dt = (t1 - t0) as f64 * 1e-9;
    let delta = se3::mat_mul(&se3::transpose(&without_yaw(&r0)), &without_yaw(&r1));
    let gyro = rotation_vector(&delta).map(|w| w / dt);
    let accel = se3::mat_vec(&se3::transpose(&r1), [0.0, 0.0, G]);
    Some(ImuSample {
        t_ns: t1,
        gyro: gyro.to_vec(),
        accel: accel.to_vec(),
        ..Default::default()
    })
}

/// `R` (world-from-body, z up) turned about the world's vertical by minus its
/// yaw: the tilt alone, `Ry(pitch)·Rx(roll)` of `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
fn without_yaw(r: &[f64; 9]) -> [f64; 9] {
    let (s, c) = (-r[3].atan2(r[0])).sin_cos();
    se3::mat_mul(&[c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0], r)
}

/// `R` world-from-body with an ENU world and an FLU body (the bridge's
/// frames) from the FC's `R_ned_frd`.
pub fn enu_flu(r_ned_frd: &[f64; 9]) -> [f64; 9] {
    se3::mat_mul(&R_ENU_NED, &se3::mat_mul(r_ned_frd, &R_FRD_FLU))
}

/// `RAW_IMU` older than this at the capture loop's pose tick: the stream
/// stalled.
const RAW_IMU_STALE_NS: u64 = 250_000_000;
/// Fewer `RAW_IMU` samples than this over the last second: the stream is
/// starved (the FC is asked for 50 or 100 Hz).
const RAW_IMU_MIN_PER_S: usize = 40;
const RATE_WINDOW_NS: u64 = 1_000_000_000;

/// What the FC feed has handed the pose provider, shared between the
/// MAVLink delivery thread ([`sink`]) and the capture loop (the attitude
/// stand-in, the `nav_imu_feed` log and the pose health).
#[derive(Debug, Default)]
pub struct FeedState {
    imu_samples: AtomicU64,
    /// Host ns of the newest IMU sample handed on; 0 before the first.
    newest_imu_ns: AtomicU64,
    raw_imu: AtomicBool,
    flow: AtomicBool,
    /// Host ns of the `RAW_IMU` samples of the last [`RATE_WINDOW_NS`] up to
    /// the newest, oldest first.
    raw_times: Mutex<VecDeque<u64>>,
}

impl FeedState {
    /// One IMU sample at host `t_ns` handed on.
    pub fn note_imu(&self, t_ns: u64) {
        self.imu_samples.fetch_add(1, Ordering::Relaxed);
        self.newest_imu_ns.fetch_max(t_ns, Ordering::Release);
    }

    /// One `RAW_IMU` sample at host `t_ns` handed on.
    fn note_raw_imu(&self, t_ns: u64) {
        self.note_imu(t_ns);
        let mut times = self.raw_times();
        times.push_back(t_ns);
        while times.front().is_some_and(|&t| t + RATE_WINDOW_NS < t_ns) {
            times.pop_front();
        }
        drop(times);
        if !self.raw_imu.swap(true, Ordering::AcqRel) {
            tracing::info!("nav_imu_source_raw_imu");
        }
    }

    fn raw_times(&self) -> std::sync::MutexGuard<'_, VecDeque<u64>> {
        self.raw_times
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Host ns of the newest IMU sample handed on.
    pub fn newest_imu_ns(&self) -> Option<u64> {
        Some(self.newest_imu_ns.load(Ordering::Acquire)).filter(|&t| t > 0)
    }

    /// IMU samples handed on since the last call.
    pub fn take_imu_samples(&self) -> u64 {
        self.imu_samples.swap(0, Ordering::Relaxed)
    }

    /// The FC streams `RAW_IMU`: the attitude stand-in is over.
    pub fn raw_imu(&self) -> bool {
        self.raw_imu.load(Ordering::Acquire)
    }

    /// Whether, at host `now_ns`, the feed fails to measure the turn the
    /// matcher's heading rides between scans: the attitude stand-in, or a
    /// `RAW_IMU` stream that stalled or arrives under 40 Hz. A pose the
    /// matcher reports good is then degraded.
    pub fn degraded(&self, now_ns: u64) -> Option<ImuFeedReason> {
        if !self.raw_imu() {
            return Some(ImuFeedReason::AttitudeStandIn);
        }
        let times = self.raw_times();
        let newest = *times.back()?;
        if newest + RAW_IMU_STALE_NS < now_ns {
            Some(ImuFeedReason::Stalled)
        } else if times.len() < RAW_IMU_MIN_PER_S {
            Some(ImuFeedReason::Starved)
        } else {
            None
        }
    }

    /// `RAW_IMU` samples over the second up to host `now_ns`; `None` before
    /// `RAW_IMU`.
    pub fn raw_rate_hz(&self, now_ns: u64) -> Option<f32> {
        if !self.raw_imu() {
            return None;
        }
        let times = self.raw_times();
        let n = times.iter().rev().take_while(|&&t| t + RATE_WINDOW_NS >= now_ns).count();
        Some(n as f32 * 1e9 / RATE_WINDOW_NS as f32)
    }
}

/// Holds the poses of a matcher that takes the FC as its IMU to the feed's
/// `reason` ([`FeedState::degraded`]).
/// - On the attitude stand-in or a stalled `RAW_IMU`, no yaw rate reaches
///   the matcher, whose rotation prior then pins the heading so it cannot
///   follow a yaw at all: every pose is lost. vision-nav sends no
///   `ODOMETRY` for it, and the flight controller falls back to its other
///   sources.
/// - On a live but starved `RAW_IMU`, the heading is measured, coarsely: a
///   pose the matcher reports good is degraded.
pub fn hold_poses_to_feed(poses: &mut [crate::pose::PoseEstimate], reason: Option<ImuFeedReason>) {
    match reason.map(ImuFeedReason::state) {
        Some(ImuFeedState::Lost) => poses.iter_mut().for_each(|e| e.health = VioHealth::Lost),
        Some(_) => poses
            .iter_mut()
            .filter(|e| e.health == VioHealth::Good)
            .for_each(|e| e.health = VioHealth::Degraded),
        None => {}
    }
}

/// The FC feed's sink for one run's `pose`: each EKF height, and with the
/// FC as the matcher's IMU (`fc_is_imu`) each `RAW_IMU` sample ([`from_raw`])
/// and each flow velocity, all as [`FC_IMU_ID`]'s, the moment they arrive.
pub fn sink(pose: Arc<dyn PoseProvider>, fc_is_imu: bool, state: Arc<FeedState>) -> NavSink {
    Arc::new(move |s| match s {
        NavSample::Imu { t_ns, imu, attitude } if fc_is_imu => {
            let attitude = attitude.map(|(t, r)| (t, enu_flu(&r)));
            let samples = vec![from_raw(t_ns, &imu, attitude)];
            pose.feed_imu(FC_IMU_ID, &ImuBatch { samples });
            state.note_raw_imu(t_ns);
        }
        NavSample::Velocity { t_ns, v } if fc_is_imu => {
            pose.feed_velocity(FC_IMU_ID, &[VelocitySample { t_ns, v }]);
            if !state.flow.swap(true, Ordering::Relaxed) {
                tracing::info!("nav_velocity_source_optical_flow");
            }
        }
        NavSample::Height {
            t_ns,
            up_m,
            up_rate_mps,
        } => pose.feed_height(&[HeightSample {
            t_ns,
            up_m,
            up_rate_mps,
        }]),
        NavSample::Imu { .. } | NavSample::Velocity { .. } => {}
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 9] {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        ]
    }

    #[test]
    fn a_still_tilted_vehicle_reads_gravity_in_its_body_and_no_rate() {
        let r = rpy(0.1745, 0.0, 0.3);
        assert!(sample(None, (0, r)).is_none());
        let s = sample(Some((0, r)), (20_000_000, r)).unwrap();
        assert!(s.gyro.iter().all(|w| w.abs() < 1e-9));
        // Rolled 10° right-wing-down in FLU: gravity leans onto +y.
        let roll = s.accel[1].atan2(s.accel[2]);
        assert!((roll - 0.1745).abs() < 1e-6, "{roll}");
    }

    /// The stand-in carries the attitude's roll and pitch rates and never
    /// its yaw: under external navigation the EKF's yaw is the matcher's own
    /// heading fed back.
    #[test]
    fn a_yawing_vehicle_reads_no_yaw_rate_and_a_rolling_one_its_roll_rate() {
        let a = rpy(0.0, 0.0, 0.0);
        let b = rpy(0.0, 0.0, 0.02);
        let s = sample(Some((0, a)), (20_000_000, b)).unwrap();
        assert!(s.gyro.iter().all(|w| w.abs() < 1e-9), "{:?}", s.gyro);
        // A tilted vehicle turning: still no rate.
        let s = sample(Some((0, rpy(0.2, -0.1, 1.0))), (20_000_000, rpy(0.2, -0.1, 3.0))).unwrap();
        assert!(s.gyro.iter().all(|w| w.abs() < 1e-9), "{:?}", s.gyro);
        // Rolling 0.02 rad in 20 ms while yawing: 1 rad/s about x.
        let s = sample(Some((0, rpy(0.0, 0.0, 0.5))), (20_000_000, rpy(0.02, 0.0, 0.9))).unwrap();
        assert!((s.gyro[0] - 1.0).abs() < 1e-6 && s.gyro[2].abs() < 1e-9, "{:?}", s.gyro);
        assert!(
            sample(Some((0, a)), (2_000_000_000, b)).is_none(),
            "a gap restarts"
        );
    }

    /// The feed is degraded on the attitude stand-in, and with `RAW_IMU`
    /// once it stalls for over 250 ms or arrives under 40 Hz.
    #[test]
    fn the_feed_is_degraded_until_raw_imu_streams_steadily() {
        let fed = Arc::new(Fed::default());
        let state = Arc::new(FeedState::default());
        let sink = sink(fed, true, state.clone());
        let raw = |t_ns: u64| NavSample::Imu {
            t_ns,
            imu: RawImu {
                time_usec: t_ns / 1_000,
                gyro: [0.0; 3],
                accel: [0.0, 0.0, -9.8],
            },
            attitude: None,
        };
        state.note_imu(5_000_000_000);
        assert_eq!(state.degraded(5_000_000_000), Some(ImuFeedReason::AttitudeStandIn));
        assert_eq!(state.raw_rate_hz(5_000_000_000), None);
        // 20 Hz for a second: starved.
        for k in 0..20 {
            sink(raw(6_000_000_000 + k * 50_000_000));
        }
        assert_eq!(state.degraded(6_960_000_000), Some(ImuFeedReason::Starved));
        assert_eq!(state.raw_rate_hz(6_960_000_000), Some(20.0));
        // 100 Hz for a second: steady.
        for k in 0..100 {
            sink(raw(7_000_000_000 + k * 10_000_000));
        }
        assert_eq!(state.degraded(7_995_000_000), None);
        assert_eq!(state.raw_rate_hz(7_995_000_000), Some(100.0));
        assert_eq!(state.degraded(8_300_000_000), Some(ImuFeedReason::Stalled));
        assert_eq!(state.raw_rate_hz(8_300_000_000), Some(70.0));
    }

    /// No yaw rate reaching the matcher (the stand-in, a stalled feed) loses
    /// every pose, whatever the matcher said; a starved but live feed only
    /// degrades a good one; a steady feed leaves them be.
    #[test]
    fn a_feed_without_a_yaw_rate_loses_the_pose_and_a_starved_one_degrades_it() {
        use crate::pose::PoseEstimate;
        use world_engine_protocol::atlas::{PoseSource, POSE_COV_LEN};
        let poses = || {
            [VioHealth::Good, VioHealth::Degraded, VioHealth::Lost].map(|health| PoseEstimate {
                t_ns: 0,
                r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                t: [0.0; 3],
                cov: [0.0; POSE_COV_LEN],
                source: PoseSource::Lidar,
                health,
                sigma_xy_m: None,
            })
        };
        let held = |reason| {
            let mut p = poses();
            hold_poses_to_feed(&mut p, reason);
            p.map(|e| e.health)
        };
        use VioHealth::*;
        assert_eq!(held(Some(ImuFeedReason::AttitudeStandIn)), [Lost, Lost, Lost]);
        assert_eq!(held(Some(ImuFeedReason::Stalled)), [Lost, Lost, Lost]);
        assert_eq!(held(Some(ImuFeedReason::Starved)), [Degraded, Degraded, Lost]);
        assert_eq!(held(None), [Good, Degraded, Lost]);
    }

    #[test]
    fn raw_imu_turns_into_the_flu_body() {
        let raw = RawImu {
            time_usec: 1,
            gyro: [0.1, 0.2, 0.3],
            accel: [0.5, 0.0, -9.8],
        };
        let s = from_raw(7, &raw, None);
        assert_eq!(s.t_ns, 7);
        assert_eq!(s.gyro, vec![0.1, -0.2, -0.3], "a right yaw (FRD +z) is FLU -z");
        assert_eq!(s.accel, vec![0.5, -0.0, 9.8], "level and still reads +g on FLU z");
    }

    /// Braking hard, the accelerometer leans back with the thrust; the
    /// specific force fed is gravity in the FC's attitude (rolled 10 degrees
    /// here) while that attitude is recent, the reading only without one.
    #[test]
    fn raw_imu_takes_gravity_from_the_fc_attitude() {
        let raw = RawImu {
            time_usec: 1,
            gyro: [0.0, 0.0, 0.3],
            accel: [3.0, 0.0, -9.8],
        };
        let r = rpy(0.1745, 0.0, 1.0);
        let s = from_raw(100_000_000, &raw, Some((60_000_000, r)));
        assert_eq!(s.gyro, vec![0.0, -0.0, -0.3]);
        let roll = s.accel[1].atan2(s.accel[2]);
        assert!((roll - 0.1745).abs() < 1e-6, "{roll}");
        assert!(s.accel[0].abs() < 1e-9, "no lean along x: {:?}", s.accel);
        let stale = from_raw(2_000_000_000, &raw, Some((60_000_000, r)));
        assert_eq!(stale.accel, vec![3.0, -0.0, 9.8], "a stale attitude is not used");
    }

    /// What a provider was fed, in order.
    #[derive(Debug, Clone, PartialEq)]
    enum Got {
        Imu(String, ImuSample),
        Velocity(String, VelocitySample),
        Height(HeightSample),
    }

    #[derive(Default)]
    struct Fed {
        track: crate::pose::PoseTrack,
        got: parking_lot::Mutex<Vec<Got>>,
    }

    impl PoseProvider for Fed {
        fn source(&self) -> world_engine_protocol::atlas::PoseSource {
            world_engine_protocol::atlas::PoseSource::Lidar
        }
        fn track(&self) -> &crate::pose::PoseTrack {
            &self.track
        }
        fn feed_imu(&self, src: &str, batch: &ImuBatch) {
            let mut g = self.got.lock();
            g.extend(batch.samples.iter().map(|s| Got::Imu(src.into(), s.clone())));
        }
        fn feed_velocity(&self, src: &str, samples: &[VelocitySample]) {
            let mut g = self.got.lock();
            g.extend(samples.iter().map(|s| Got::Velocity(src.into(), *s)));
        }
        fn feed_height(&self, samples: &[HeightSample]) {
            self.got.lock().extend(samples.iter().map(|s| Got::Height(*s)));
        }
    }

    fn samples() -> [NavSample; 3] {
        // Rolled 10 degrees, nose north: gravity leans onto the body's y.
        let r_ned_frd = rpy(0.1745, 0.0, 0.0);
        [
            NavSample::Imu {
                t_ns: 100_000_000,
                imu: RawImu {
                    time_usec: 1,
                    gyro: [0.0, 0.0, 0.3],
                    accel: [3.0, 0.0, -9.8],
                },
                attitude: Some((90_000_000, r_ned_frd)),
            },
            NavSample::Velocity {
                t_ns: 105_000_000,
                v: [1.5, -0.25],
            },
            NavSample::Height {
                t_ns: 110_000_000,
                up_m: 2.5,
                up_rate_mps: 0.75,
            },
        ]
    }

    /// With the FC as the matcher's IMU every sample reaches the provider in
    /// arrival order: `RAW_IMU` in the FLU body with gravity from the FC's
    /// attitude, the flow velocity, the height.
    #[test]
    fn the_sink_feeds_every_sample_as_it_comes() {
        let fed = Arc::new(Fed::default());
        let state = Arc::new(FeedState::default());
        let sink = sink(fed.clone(), true, state.clone());
        assert!(!state.raw_imu() && state.newest_imu_ns().is_none());
        for s in samples() {
            sink(s);
        }
        let got = fed.got.lock().clone();
        let [Got::Imu(src, imu), Got::Velocity(vsrc, v), Got::Height(h)] = got.as_slice() else {
            panic!("{got:?}");
        };
        assert_eq!((src.as_str(), vsrc.as_str()), (FC_IMU_ID, FC_IMU_ID));
        assert_eq!(imu.t_ns, 100_000_000);
        assert_eq!(imu.gyro, vec![0.0, -0.0, -0.3], "a right yaw (FRD +z) is FLU -z");
        let g = imu.accel.iter().map(|a| a * a).sum::<f64>().sqrt();
        let roll = imu.accel[1].atan2(imu.accel[2]).abs();
        assert!((g - G).abs() < 1e-9 && (roll - 0.1745).abs() < 1e-6, "{:?}", imu.accel);
        assert_eq!(*v, VelocitySample { t_ns: 105_000_000, v: [1.5, -0.25] });
        assert_eq!(
            *h,
            HeightSample {
                t_ns: 110_000_000,
                up_m: 2.5,
                up_rate_mps: 0.75
            }
        );
        assert!(state.raw_imu());
        assert_eq!(state.newest_imu_ns(), Some(100_000_000));
        assert_eq!(state.take_imu_samples(), 1);
        assert_eq!(state.take_imu_samples(), 0, "counted since the last call");
    }

    /// With another IMU the FC's IMU and flow are not the matcher's; its
    /// height still is.
    #[test]
    fn without_the_fc_imu_only_the_height_is_fed() {
        let fed = Arc::new(Fed::default());
        let state = Arc::new(FeedState::default());
        let sink = sink(fed.clone(), false, state.clone());
        for s in samples() {
            sink(s);
        }
        assert!(matches!(fed.got.lock().as_slice(), [Got::Height(_)]));
        assert!(!state.raw_imu() && state.newest_imu_ns().is_none());
    }
}
