//! Roll and pitch from an IMU with a Mahony complementary filter (Mahony,
//! Hamel and Pflimlin 2008): the gyro is integrated and the accelerometer's
//! gravity direction pulls the estimate back with a PI correction. Yaw is not
//! observable from gravity; the filter reports the heading of the levelled
//! frame (the yaw of its attitude, carried by the gyro alone) and its rate as
//! the scan matcher's rotation prior.
//!
//! The heading is the yaw `ψ` of the attitude `Rz(ψ)·Ry(pitch)·Rx(roll)`, the
//! angle a scan levelled by roll and pitch is turned by in the plane. It is
//! not the integral of the rate about the world vertical: that differs from
//! `ψ̇` by `roll̇·sin(pitch)`, which does not average out on a vehicle that
//! pitches and rolls as it manoeuvres, and would leave the prior turning
//! short of (or past) the scans.
//!
//! A gyro's bias about the vertical turns the heading at a steady rate the
//! gravity direction cannot see. The scan matcher can: [`Attitude::learn_bias`]
//! takes the heading the scans turned against the gyro's and integrates the
//! difference, slowly, into a bias about the LiDAR's z axis that is removed
//! from every sample before it is integrated.

use std::collections::VecDeque;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use crate::pose::wrap;

/// One IMU sample in the IMU frame (FLU).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Nanoseconds, the same clock as the scans.
    pub t_ns: u64,
    /// Angular rate, rad/s.
    pub gyro: [f64; 3],
    /// Specific force, m/s² (+9.81 on z when level and at rest).
    pub accel: [f64; 3],
}

/// A measured velocity of the IMU frame along its own x and y axes (FLU),
/// m/s, such as a downward optical-flow sensor's angular flow, compensated
/// for the body rate and scaled by its range to the ground. The velocity
/// along the IMU's z axis is not measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocitySample {
    /// Nanoseconds, the same clock as the scans.
    pub t_ns: u64,
    pub v: [f64; 2],
}

/// Tilts past this leave a ground-velocity measurement too steep to level.
const MAX_LEVEL_TILT_RAD: f64 = 1.0;

/// Samples further apart than this restart the integration instead of
/// bridging the gap.
const MAX_GAP_S: f64 = 0.5;
/// Samples further apart than this leave the rotation between them
/// unmeasured ([`Attitude::measured_since`]): the integration bridges the gap
/// with the later sample's rate, a guess, up to [`MAX_GAP_S`].
pub const MAX_SAMPLE_GAP_NS: u64 = 200_000_000;
/// How long the heading history reaches back.
const HISTORY_NS: u64 = 2_000_000_000;
/// Time constant of the gyro z-bias estimate: long against a scan's match
/// noise, short against a gyro bias that drifts with temperature (a
/// simulated one went from 0.03 to 0.23 degrees/s and back over eight
/// minutes) and the minutes a biased heading takes to pull a map off. The
/// matcher corrects only part of the prior's error on each scan, so the
/// estimate settles over a few of these.
const BIAS_TAU_S: f64 = 12.0;
/// Bound on the gyro z bias, rad/s (1 degree/s, several times a calibrated
/// MEMS gyro's): one scan's heading error can move the estimate no faster
/// than this over [`BIAS_TAU_S`].
pub const MAX_GYRO_BIAS_RAD_S: f64 = std::f64::consts::PI / 180.0;

/// Attitude of the LiDAR frame from IMU samples.
#[derive(Debug, Clone)]
pub struct Attitude {
    kp: f64,
    ki: f64,
    r_lidar_imu: Matrix3<f64>,
    q: UnitQuaternion<f64>,
    integral: Vector3<f64>,
    last_t_ns: Option<u64>,
    heading_rate: f64,
    heading: f64,
    /// `(t_ns, heading)` per sample, oldest first.
    history: VecDeque<(u64, f64)>,
    /// Estimated gyro bias about the LiDAR's z axis, rad/s.
    bias_z: f64,
    /// Time of the sample that ended the latest gap wider than
    /// [`MAX_SAMPLE_GAP_NS`] (or the first sample).
    gap_end_ns: Option<u64>,
    /// Length of that gap, ns (0 for the first sample).
    gap_ns: u64,
}

impl Attitude {
    /// `r_lidar_imu` rotates IMU-frame vectors into the LiDAR frame, so the
    /// reported roll and pitch are the LiDAR's.
    pub fn new(kp: f64, ki: f64, r_lidar_imu: [[f64; 3]; 3]) -> Self {
        let r = r_lidar_imu;
        Attitude {
            kp,
            ki,
            r_lidar_imu: Matrix3::new(
                r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
            ),
            q: UnitQuaternion::identity(),
            integral: Vector3::zeros(),
            last_t_ns: None,
            heading_rate: 0.0,
            heading: 0.0,
            history: VecDeque::new(),
            bias_z: 0.0,
            gap_end_ns: None,
            gap_ns: 0,
        }
    }

    /// Whether any sample arrived yet.
    pub fn has_data(&self) -> bool {
        self.last_t_ns.is_some()
    }

    /// Time of the newest sample, `None` before any.
    pub fn newest_ns(&self) -> Option<u64> {
        self.last_t_ns
    }

    /// Length of the latest gap wider than [`MAX_SAMPLE_GAP_NS`], ns: the
    /// time over which the rotation went unmeasured.
    pub fn last_gap_ns(&self) -> u64 {
        self.gap_ns
    }

    /// Whether the samples measured the rotation from `t0_ns` on: no gap
    /// wider than [`MAX_SAMPLE_GAP_NS`] has ended after it, so the heading
    /// change from `t0_ns` to any time up to [`Self::newest_ns`] is
    /// integrated gyro, not a guess across a gap.
    pub fn measured_since(&self, t0_ns: u64) -> bool {
        self.gap_end_ns.is_some_and(|g| g <= t0_ns)
    }

    pub fn update(&mut self, s: &ImuSample) {
        let w = self.r_lidar_imu * Vector3::from(s.gyro) - Vector3::new(0.0, 0.0, self.bias_z);
        let a = self.r_lidar_imu * Vector3::from(s.accel);
        let Some(last) = self.last_t_ns else {
            self.last_t_ns = Some(s.t_ns);
            self.gap_end_ns = Some(s.t_ns);
            if a.norm() > 1e-6 {
                let roll = a.y.atan2(a.z);
                let pitch = (-a.x).atan2(a.y.hypot(a.z));
                self.q = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
            }
            self.heading_rate = self.heading_rate_of(&w);
            self.history.push_back((s.t_ns, 0.0));
            return;
        };
        if s.t_ns <= last {
            return;
        }
        self.last_t_ns = Some(s.t_ns);
        let dt_ns = s.t_ns - last;
        if dt_ns > MAX_SAMPLE_GAP_NS {
            self.gap_end_ns = Some(s.t_ns);
            self.gap_ns = dt_ns;
        }
        let dt = dt_ns as f64 * 1e-9;
        if dt > MAX_GAP_S {
            self.heading_rate = self.heading_rate_of(&w);
            self.record(s.t_ns);
            return;
        }
        let mut wc = w;
        let an = a.norm();
        if an > 1e-6 {
            let up_body = self.q.inverse() * Vector3::z();
            let e = (a / an).cross(&up_body);
            self.integral += self.ki * e * dt;
            wc += self.kp * e + self.integral;
        }
        let yaw0 = self.yaw();
        self.q *= UnitQuaternion::from_scaled_axis(wc * dt);
        self.heading += wrap(self.yaw() - yaw0);
        self.heading_rate = self.heading_rate_of(&w);
        self.record(s.t_ns);
    }

    /// Yaw of the current attitude, `(-π, π]`.
    fn yaw(&self) -> f64 {
        self.q.euler_angles().2
    }

    /// `ψ̇` of the current attitude turning at body rate `w`:
    /// `(w_y·sin(roll) + w_z·cos(roll)) / cos(pitch)`. Pitched straight up or
    /// down the heading is undefined; the rate about the vertical stands in.
    fn heading_rate_of(&self, w: &Vector3<f64>) -> f64 {
        let (roll, pitch, _) = self.q.euler_angles();
        let cp = pitch.cos();
        if cp.abs() < 1e-6 {
            return (self.q * w).z;
        }
        (w.y * roll.sin() + w.z * roll.cos()) / cp
    }

    fn record(&mut self, t_ns: u64) {
        self.history.push_back((t_ns, self.heading));
        while self
            .history
            .front()
            .is_some_and(|&(t, _)| t + HISTORY_NS < t_ns)
        {
            self.history.pop_front();
        }
    }

    /// `(roll, pitch)` of the LiDAR frame, radians.
    pub fn roll_pitch(&self) -> (f64, f64) {
        let (r, p, _) = self.q.euler_angles();
        (r, p)
    }

    /// Latest rate of the heading, rad/s.
    pub fn heading_rate(&self) -> f64 {
        self.heading_rate
    }

    /// Heading of the levelled frame since the first sample, radians
    /// (unwrapped): the yaw of the attitude `Rz(ψ)·Ry(pitch)·Rx(roll)`.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// Heading at `t_ns`: interpolated between samples, extrapolated with the
    /// latest rate past the newest, held before the oldest kept.
    pub fn heading_at(&self, t_ns: u64) -> f64 {
        let Some(&(tl, al)) = self.history.back() else {
            return 0.0;
        };
        if t_ns >= tl {
            return al + self.heading_rate * (t_ns - tl) as f64 * 1e-9;
        }
        let k = self.history.partition_point(|&(t, _)| t <= t_ns);
        if k == 0 {
            return self.history[0].1;
        }
        let ((t0, a0), (t1, a1)) = (self.history[k - 1], self.history[k]);
        a0 + (a1 - a0) * (t_ns - t0) as f64 / (t1 - t0) as f64
    }

    /// The horizontal velocity, in the levelled LiDAR frame, of a vehicle
    /// whose velocity along the IMU's x and y axes is `v_xy` and whose height
    /// does not change: the unmeasured IMU-z component is the one that makes
    /// the vertical velocity zero, so a pitched airframe flying level is not
    /// read short by the cosine of its pitch. `None` before any sample or
    /// tilted past [`MAX_LEVEL_TILT_RAD`].
    pub fn level_velocity(&self, v_xy: [f64; 2]) -> Option<[f64; 2]> {
        if !self.has_data() {
            return None;
        }
        let (roll, pitch, _) = self.q.euler_angles();
        let r = UnitQuaternion::from_euler_angles(roll, pitch, 0.0)
            .to_rotation_matrix()
            .into_inner()
            * self.r_lidar_imu;
        let up = r.row(2);
        if up[2] < MAX_LEVEL_TILT_RAD.cos() {
            return None;
        }
        let vz = -(up[0] * v_xy[0] + up[1] * v_xy[1]) / up[2];
        let v = r * Vector3::new(v_xy[0], v_xy[1], vz);
        Some([v.x, v.y])
    }

    /// Estimated gyro bias about the LiDAR's z axis, rad/s, removed from
    /// every sample before it is integrated.
    pub fn gyro_bias_z(&self) -> f64 {
        self.bias_z
    }

    /// Learn the gyro's z bias from the heading the scans turned over `dt_s`
    /// seconds minus the heading the (bias-corrected) gyro turned over the
    /// same interval, radians. The difference, as a rate bounded by
    /// [`MAX_GYRO_BIAS_RAD_S`], moves the bias over [`BIAS_TAU_S`]: an
    /// integral on the scans' heading, so a bias the matcher only partly
    /// corrects on each scan is still removed in full. Only the scans feed
    /// it, never the flight controller's fused yaw.
    pub fn learn_bias(&mut self, scan_minus_gyro: f64, dt_s: f64) {
        if dt_s <= 0.0 || !dt_s.is_finite() || !scan_minus_gyro.is_finite() {
            return;
        }
        let rate = (scan_minus_gyro / dt_s).clamp(-MAX_GYRO_BIAS_RAD_S, MAX_GYRO_BIAS_RAD_S);
        let gain = (dt_s / BIAS_TAU_S).min(1.0);
        self.bias_z = (self.bias_z - rate * gain).clamp(-MAX_GYRO_BIAS_RAD_S, MAX_GYRO_BIAS_RAD_S);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tilted_still_imu_converges_to_its_tilt_and_integrates_yaw() {
        let mut a = Attitude::new(
            2.0,
            0.005,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        );
        let (roll, pitch) = (0.10f64, -0.05f64);
        let g = UnitQuaternion::from_euler_angles(roll, pitch, 0.0).inverse()
            * Vector3::new(0.0, 0.0, 9.81);
        // Start from a wrong level estimate and spin about the vertical.
        a.update(&ImuSample {
            t_ns: 0,
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81],
        });
        let w_body = UnitQuaternion::from_euler_angles(roll, pitch, 0.0).inverse()
            * Vector3::new(0.0, 0.0, 0.5);
        for k in 1..=2000u64 {
            a.update(&ImuSample {
                t_ns: k * 5_000_000,
                gyro: w_body.into(),
                accel: g.into(),
            });
        }
        let (r, p) = a.roll_pitch();
        assert!((r - roll).abs() < 0.01, "roll {r}");
        assert!((p - pitch).abs() < 0.01, "pitch {p}");
        assert!((a.heading_rate() - 0.5).abs() < 0.01);
        assert!((a.heading() - 5.0).abs() < 0.05, "{}", a.heading());
    }

    /// An airframe turning at 0.2 rad/s while it rocks: roll and pitch swing
    /// 0.1 rad at 2 Hz a quarter period apart about 0.15 rad nose-down. The
    /// heading follows the yaw of the attitude; the rate about the world
    /// vertical, integrated, would be over a radian off after 20 s, since it
    /// misses `∫roll̇·sin(pitch)`.
    #[test]
    fn heading_follows_the_yaw_of_a_rocking_airframe() {
        let mut a = Attitude::new(
            2.0,
            0.005,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        );
        let w = 4.0 * std::f64::consts::PI;
        let att = |t: f64| {
            UnitQuaternion::from_euler_angles(
                0.1 * (w * t).sin(),
                -0.15 + 0.1 * (w * t).cos(),
                0.2 * t,
            )
        };
        let dt_ns = 5_000_000u64;
        let mut vertical = 0.0;
        let mut prev = att(0.0);
        a.update(&ImuSample {
            t_ns: 0,
            gyro: [0.0; 3],
            accel: (prev.inverse() * Vector3::new(0.0, 0.0, 9.81)).into(),
        });
        for k in 1..=4000u64 {
            let q = att((k * dt_ns) as f64 * 1e-9);
            let gyro = (prev.inverse() * q).scaled_axis() / (dt_ns as f64 * 1e-9);
            vertical += (q * gyro).z * dt_ns as f64 * 1e-9;
            a.update(&ImuSample {
                t_ns: k * dt_ns,
                gyro: gyro.into(),
                accel: (q.inverse() * Vector3::new(0.0, 0.0, 9.81)).into(),
            });
            prev = q;
        }
        assert!((vertical - 4.0).abs() > 1.0, "vertical {vertical}");
        assert!((a.heading() - 4.0).abs() < 0.01, "heading {}", a.heading());
        let mid = a.heading_at(19_502_500_000);
        assert!((mid - 0.2 * 19.5025).abs() < 0.01, "heading at 19.5 s {mid}");
    }

    /// An attitude held still at `roll`, `pitch` for one second.
    fn still(roll: f64, pitch: f64, r_lidar_imu: [[f64; 3]; 3]) -> Attitude {
        let mut a = Attitude::new(2.0, 0.005, r_lidar_imu);
        let r = Matrix3::from_row_slice(&r_lidar_imu.concat());
        let g = r.transpose()
            * (UnitQuaternion::from_euler_angles(roll, pitch, 0.0).inverse()
                * Vector3::new(0.0, 0.0, 9.81));
        for k in 0..=200u64 {
            a.update(&ImuSample {
                t_ns: k * 5_000_000,
                gyro: [0.0; 3],
                accel: g.into(),
            });
        }
        a
    }

    /// Flying level at 5 m/s forward and 1 m/s left, nose down 20 degrees
    /// and rolled 3: a flow sensor reads the body-x component short by the
    /// cosine of the pitch. Levelled, the full ground velocity comes back,
    /// in the LiDAR's axes when the IMU is mounted turned 90 degrees.
    #[test]
    fn a_tilted_airframe_flying_level_reads_its_ground_velocity() {
        let (roll, pitch) = (0.05f64, 0.35f64);
        let ground = Vector3::new(5.0, 1.0, 0.0);
        let body = UnitQuaternion::from_euler_angles(roll, pitch, 0.0).inverse() * ground;
        assert!((body.x - 5.0).abs() > 0.2, "the body axis reads short: {}", body.x);
        let a = still(roll, pitch, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let v = a.level_velocity([body.x, body.y]).expect("levelled");
        assert!((v[0] - 5.0).abs() < 0.01 && (v[1] - 1.0).abs() < 0.01, "{v:?}");

        // IMU x along the LiDAR's y: R_lidar_imu = Rz(90°).
        let rz = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let a = still(roll, pitch, rz);
        let imu = Matrix3::from_row_slice(&rz.concat()).transpose() * body;
        let v = a.level_velocity([imu.x, imu.y]).expect("levelled");
        assert!((v[0] - 5.0).abs() < 0.01 && (v[1] - 1.0).abs() < 0.01, "{v:?}");
    }

    #[test]
    fn ground_velocity_needs_an_attitude_and_a_moderate_tilt() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert_eq!(Attitude::new(2.0, 0.005, id).level_velocity([1.0, 0.0]), None);
        assert_eq!(still(1.2, 0.0, id).level_velocity([1.0, 0.0]), None);
        assert!(still(0.8, 0.0, id).level_velocity([1.0, 0.0]).is_some());
    }

    /// A still IMU whose gyro reads 0.2 degrees/s about z: the scans see no
    /// turn, so every scan's heading against the gyro's is minus the bias
    /// times the interval. Learnt over a minute, the bias comes out of the
    /// samples and the heading stops turning.
    #[test]
    fn a_still_gyros_z_bias_is_learnt_from_the_scans() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut a = Attitude::new(2.0, 0.005, id);
        let bias = 0.2f64.to_radians();
        let mut t = 0u64;
        let feed = |a: &mut Attitude, t: &mut u64| {
            for _ in 0..20 {
                a.update(&ImuSample {
                    t_ns: *t,
                    gyro: [0.0, 0.0, bias],
                    accel: [0.0, 0.0, 9.81],
                });
                *t += 5_000_000;
            }
        };
        feed(&mut a, &mut t);
        for _ in 0..1200 {
            let before = a.heading();
            feed(&mut a, &mut t);
            a.learn_bias(-(a.heading() - before), 0.1);
        }
        let learnt = a.gyro_bias_z();
        assert!((learnt - bias).abs() < 0.01 * bias, "bias {}", learnt.to_degrees());
        let before = a.heading();
        feed(&mut a, &mut t);
        assert!((a.heading() - before).abs() < 1e-5, "still turning");
    }

    /// One wild match moves the bias by no more than the bound over the time
    /// constant, and the bias never passes the bound.
    #[test]
    fn a_single_bad_match_barely_moves_the_bias() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut a = Attitude::new(2.0, 0.005, id);
        a.learn_bias(0.5, 0.1);
        let step = MAX_GYRO_BIAS_RAD_S * 0.1 / BIAS_TAU_S;
        assert!((a.gyro_bias_z() + step).abs() < 1e-12, "{}", a.gyro_bias_z());
        for _ in 0..100_000 {
            a.learn_bias(-0.5, 0.1);
        }
        assert!((a.gyro_bias_z() - MAX_GYRO_BIAS_RAD_S).abs() < 1e-12);
        a.learn_bias(f64::NAN, 0.1);
        a.learn_bias(0.1, 0.0);
        assert!((a.gyro_bias_z() - MAX_GYRO_BIAS_RAD_S).abs() < 1e-12);
    }
}
