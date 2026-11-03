//! An IMU stream synthesised from the flight controller's attitude, for a
//! companion with no IMU of its own (a drone whose planar LiDAR sits on the
//! companion and whose IMU is on the FC). Set `atlas.lidar.imu = "fc"` and
//! declare a rig IMU sensor `fc` mounted at the body origin.
//!
//! From two consecutive FC EKF attitudes (`R` world-from-body, ENU world, FLU
//! body): the body rate is the rotation vector of `R_prevᵀ·R` over the gap,
//! and the specific force of a vehicle not accelerating is gravity in the
//! body, `Rᵀ·(0, 0, g)`. The planar matcher only levels scans with roll and
//! pitch and integrates yaw for its prior, which this is enough for.

use ados_capture_contract::v1::ImuSample;

use crate::pose::se3;

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

/// The sample at `cur` given the previous attitude, `None` for the first one
/// or after a gap.
pub fn sample(prev: Option<(u64, [f64; 9])>, cur: (u64, [f64; 9])) -> Option<ImuSample> {
    let (t0, r0) = prev?;
    let (t1, r1) = cur;
    if t1 <= t0 || t1 - t0 > MAX_GAP_NS {
        return None;
    }
    let dt = (t1 - t0) as f64 * 1e-9;
    let delta = se3::mat_mul(&se3::transpose(&r0), &r1);
    let gyro = rotation_vector(&delta).map(|w| w / dt);
    let accel = se3::mat_vec(&se3::transpose(&r1), [0.0, 0.0, G]);
    Some(ImuSample {
        t_ns: t1,
        gyro: gyro.to_vec(),
        accel: accel.to_vec(),
        ..Default::default()
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

    #[test]
    fn a_yawing_vehicle_reads_its_rate_about_body_z() {
        let a = rpy(0.0, 0.0, 0.0);
        let b = rpy(0.0, 0.0, 0.02);
        let s = sample(Some((0, a)), (20_000_000, b)).unwrap();
        assert!((s.gyro[2] - 1.0).abs() < 1e-6, "{:?}", s.gyro);
        assert!(sample(Some((0, a)), (2_000_000_000, b)).is_none(), "a gap restarts");
    }
}
