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

/// Samples further apart than this restart the integration instead of
/// bridging the gap.
const MAX_GAP_S: f64 = 0.5;
/// How long the heading history reaches back.
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
    heading_rate: f64,
    heading: f64,
    /// `(t_ns, heading)` per sample, oldest first.
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
            heading_rate: 0.0,
            heading: 0.0,
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
            self.heading_rate = self.heading_rate_of(&w);
            self.history.push_back((s.t_ns, 0.0));
            return;
        };
        if s.t_ns <= last {
            return;
        }
        self.last_t_ns = Some(s.t_ns);
        let dt = (s.t_ns - last) as f64 * 1e-9;
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
}
