//! Roll and pitch from an IMU with a Mahony complementary filter (Mahony,
//! Hamel and Pflimlin 2008): the gyro is integrated and the accelerometer's
//! gravity direction pulls the estimate back with a PI correction. Yaw is not
//! observable from gravity; the filter reports the rate about the world
//! vertical, and its integral, as the scan matcher's rotation prior.

use std::collections::VecDeque;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

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

/// Samples further apart than this restart the integration instead of
/// bridging the gap.
const MAX_GAP_S: f64 = 0.5;
/// How long the vertical-angle history reaches back.
const HISTORY_NS: u64 = 2_000_000_000;

/// Attitude of the LiDAR frame from IMU samples.
#[derive(Debug, Clone)]
pub struct Attitude {
    kp: f64,
    ki: f64,
    r_lidar_imu: Matrix3<f64>,
    q: UnitQuaternion<f64>,
    integral: Vector3<f64>,
    last_t_ns: Option<u64>,
    vertical_rate: f64,
    vertical_angle: f64,
    /// `(t_ns, vertical_angle)` per sample, oldest first.
    history: VecDeque<(u64, f64)>,
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
            vertical_rate: 0.0,
            vertical_angle: 0.0,
            history: VecDeque::new(),
        }
    }

    /// Whether any sample arrived yet.
    pub fn has_data(&self) -> bool {
        self.last_t_ns.is_some()
    }

    pub fn update(&mut self, s: &ImuSample) {
        let w = self.r_lidar_imu * Vector3::from(s.gyro);
        let a = self.r_lidar_imu * Vector3::from(s.accel);
        let Some(last) = self.last_t_ns else {
            self.last_t_ns = Some(s.t_ns);
            if a.norm() > 1e-6 {
                let roll = a.y.atan2(a.z);
                let pitch = (-a.x).atan2(a.y.hypot(a.z));
                self.q = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
            }
            self.vertical_rate = (self.q * w).z;
            self.history.push_back((s.t_ns, 0.0));
            return;
        };
        if s.t_ns <= last {
            return;
        }
        self.last_t_ns = Some(s.t_ns);
        let dt = (s.t_ns - last) as f64 * 1e-9;
        if dt > MAX_GAP_S {
            self.vertical_rate = (self.q * w).z;
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
        self.q *= UnitQuaternion::from_scaled_axis(wc * dt);
        let rate = (self.q * w).z;
        self.vertical_angle += 0.5 * (self.vertical_rate + rate) * dt;
        self.vertical_rate = rate;
        self.record(s.t_ns);
    }

    fn record(&mut self, t_ns: u64) {
        self.history.push_back((t_ns, self.vertical_angle));
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

    /// Latest rotation rate about the world vertical, rad/s.
    pub fn vertical_rate(&self) -> f64 {
        self.vertical_rate
    }

    /// Rotation about the world vertical integrated since the first sample,
    /// radians (unwrapped).
    pub fn vertical_angle(&self) -> f64 {
        self.vertical_angle
    }

    /// Vertical angle at `t_ns`: interpolated between samples, extrapolated
    /// with the latest rate past the newest, held before the oldest kept.
    pub fn vertical_angle_at(&self, t_ns: u64) -> f64 {
        let Some(&(tl, al)) = self.history.back() else {
            return 0.0;
        };
        if t_ns >= tl {
            return al + self.vertical_rate * (t_ns - tl) as f64 * 1e-9;
        }
        let k = self.history.partition_point(|&(t, _)| t <= t_ns);
        if k == 0 {
            return self.history[0].1;
        }
        let ((t0, a0), (t1, a1)) = (self.history[k - 1], self.history[k]);
        a0 + (a1 - a0) * (t_ns - t0) as f64 / (t1 - t0) as f64
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
        assert!((a.vertical_rate() - 0.5).abs() < 0.01);
        assert!(
            (a.vertical_angle() - 5.0).abs() < 0.05,
            "{}",
            a.vertical_angle()
        );
    }
}
