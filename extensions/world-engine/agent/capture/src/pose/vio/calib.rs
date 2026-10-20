//! The Basalt calibration and estimator config `basalt-ados` loads, built from
//! the rig.
//!
//! Camera models: `kb4` and `opencv_fisheye` (the same Kannala-Brandt
//! polynomial) map to Basalt `kb4`; `pinhole` maps to Basalt `pinhole` and
//! must carry no non-zero distortion; `opencv` (k1, k2, p1, p2, k3) maps to
//! Basalt `pinhole-radtan8` with k4..k6 = 0. A coefficient count that does not
//! match the model is refused. Basalt models a global shutter; a rolling
//! shutter camera (`readout_ns > 0`) is accepted with a warning.
//!
//! Extrinsics: `T_imu_cam = inv(T_body_imu) * T_body_cam`, both cameras in the
//! OpenCV camera frame. IMU noise comes from the rig's `imu_noise` (Kalibr
//! continuous-time units, which are Basalt's units too) or, when absent, from
//! [`DEFAULT_IMU_NOISE`]. Frame times reach the child already on the host
//! monotonic clock, so `cam_time_offset_ns` is 0.

use ados_capture_contract::{
    CameraModel, CameraModelModel, ImuNoise, Rig, RigSensor, RigSensorKind,
};
use serde_json::{json, Value};

use super::pairing::VioScale;
use crate::pose::se3;

/// IMU sample rate the noise densities are discretised at. The sensor bus
/// carries the flight controller's raw IMU at about this rate.
pub const IMU_RATE_HZ: f64 = 200.0;

/// Noise of an ICM-42688-class MEMS IMU: the datasheet densities (gyro
/// 2.8 mdps/sqrt(Hz), accel 70 ug/sqrt(Hz)) and typical bias instabilities,
/// each inflated by 10x for vibration and temperature, as is usual for VIO.
pub const DEFAULT_IMU_NOISE: ImuNoise = ImuNoise {
    gyro_nd: 4.9e-4,
    accel_nd: 6.9e-3,
    gyro_rw: 4.0e-5,
    accel_rw: 2.0e-3,
};

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum CalibError {
    #[error("rig has no sensor `{0}`")]
    MissingSensor(String),
    #[error("sensor `{id}` is a {kind}, expected {expected}")]
    WrongKind {
        id: String,
        kind: String,
        expected: &'static str,
    },
    #[error("camera `{0}` has no camera model")]
    NoCameraModel(String),
    #[error(
        "camera `{id}`: model {model} with {len} distortion coefficients is not supported by Basalt"
    )]
    UnsupportedDistortion {
        id: String,
        model: String,
        len: usize,
    },
    #[error("sensor `{0}` has a non-rigid T_body_sensor")]
    BadTransform(String),
}

fn sensor<'a>(rig: &'a Rig, id: &str, kind: RigSensorKind) -> Result<&'a RigSensor, CalibError> {
    let s = rig
        .sensors
        .iter()
        .find(|s| s.id == id)
        .ok_or_else(|| CalibError::MissingSensor(id.to_string()))?;
    if s.kind != kind {
        return Err(CalibError::WrongKind {
            id: id.to_string(),
            kind: s.kind.to_string(),
            expected: if kind == RigSensorKind::Imu {
                "imu"
            } else {
                "camera"
            },
        });
    }
    Ok(s)
}

/// `T` as a rotation + translation, refusing a matrix that is not rigid.
fn rigid(id: &str, m: &[f64; 16]) -> Result<([f64; 9], [f64; 3]), CalibError> {
    let (r, t) = se3::from_mat4(m);
    let rrt = se3::mat_mul(&r, &se3::transpose(&r));
    let orthonormal = rrt
        .iter()
        .zip(se3::IDENTITY.iter())
        .all(|(a, b)| (a - b).abs() < 1e-6);
    let bottom = m[12] == 0.0 && m[13] == 0.0 && m[14] == 0.0 && m[15] == 1.0;
    let det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6])
        + r[2] * (r[3] * r[7] - r[4] * r[6]);
    if !(orthonormal && bottom && det > 0.0) || t.iter().any(|v| !v.is_finite()) {
        return Err(CalibError::BadTransform(id.to_string()));
    }
    Ok((r, t))
}

/// The rotation `R_imu_body` and translation of `T_imu_body = inv(T_body_imu)`.
pub fn imu_from_body(rig: &Rig, imu_id: &str) -> Result<([f64; 9], [f64; 3]), CalibError> {
    let imu = sensor(rig, imu_id, RigSensorKind::Imu)?;
    let (r_bi, t_bi) = rigid(imu_id, &imu.t_body_sensor)?;
    let r_ib = se3::transpose(&r_bi);
    let t = se3::mat_vec(&r_ib, t_bi);
    Ok((r_ib, [-t[0], -t[1], -t[2]]))
}

fn intrinsics(id: &str, cam: &CameraModel) -> Result<Value, CalibError> {
    let refuse = || CalibError::UnsupportedDistortion {
        id: id.to_string(),
        model: cam.model.to_string(),
        len: cam.dist.len(),
    };
    let d = &cam.dist;
    let (fx, fy, cx, cy) = (cam.fx, cam.fy, cam.cx, cam.cy);
    Ok(match cam.model {
        CameraModelModel::Pinhole => {
            if d.iter().any(|v| *v != 0.0) {
                return Err(refuse());
            }
            json!({"camera_type": "pinhole",
                   "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy}})
        }
        CameraModelModel::Opencv => {
            let [k1, k2, p1, p2, k3]: [f64; 5] = d.as_slice().try_into().map_err(|_| refuse())?;
            json!({"camera_type": "pinhole-radtan8",
                   "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                                  "k1": k1, "k2": k2, "p1": p1, "p2": p2, "k3": k3,
                                  "k4": 0.0, "k5": 0.0, "k6": 0.0,
                                  // -1: Basalt computes the valid radius itself.
                                  "rpmax": -1.0}})
        }
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => {
            let [k1, k2, k3, k4]: [f64; 4] = d.as_slice().try_into().map_err(|_| refuse())?;
            json!({"camera_type": "kb4",
                   "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                                  "k1": k1, "k2": k2, "k3": k3, "k4": k4}})
        }
    })
}

/// Basalt `{px, py, pz, qx, qy, qz, qw}` of a rotation + translation.
fn se3_json(r: &[f64; 9], t: [f64; 3]) -> Value {
    let [w, x, y, z] = se3::quat_from_matrix(r);
    json!({"px": t[0], "py": t[1], "pz": t[2], "qx": x, "qy": y, "qz": z, "qw": w})
}

/// The Basalt calibration JSON for the left/right cameras and the IMU.
pub fn basalt_calib(
    rig: &Rig,
    left_id: &str,
    right_id: &str,
    imu_id: &str,
) -> Result<Value, CalibError> {
    let (r_ib, t_ib) = imu_from_body(rig, imu_id)?;
    let mut t_imu_cam = Vec::new();
    let mut intr = Vec::new();
    let mut resolution = Vec::new();
    for id in [left_id, right_id] {
        let cam = sensor(rig, id, RigSensorKind::Camera)?;
        let model = cam
            .camera
            .as_ref()
            .ok_or_else(|| CalibError::NoCameraModel(id.to_string()))?;
        if model.readout_ns > 0 {
            tracing::warn!(
                camera = id,
                readout_ns = model.readout_ns,
                "Basalt assumes a global shutter"
            );
        }
        let (r_bc, t_bc) = rigid(id, &cam.t_body_sensor)?;
        // T_imu_cam = T_imu_body * T_body_cam.
        let r = se3::mat_mul(&r_ib, &r_bc);
        let rt = se3::mat_vec(&r_ib, t_bc);
        t_imu_cam.push(se3_json(
            &r,
            [rt[0] + t_ib[0], rt[1] + t_ib[1], rt[2] + t_ib[2]],
        ));
        intr.push(intrinsics(id, model)?);
        resolution.push(json!([model.width, model.height]));
    }
    let imu = sensor(rig, imu_id, RigSensorKind::Imu)?;
    let n = imu.imu_noise.clone().unwrap_or(DEFAULT_IMU_NOISE);
    let v3 = |x: f64| json!([x, x, x]);
    Ok(json!({"value0": {
        "T_imu_cam": t_imu_cam,
        "intrinsics": intr,
        "resolution": resolution,
        "calib_accel_bias": vec![0.0; 9],
        "calib_gyro_bias": vec![0.0; 12],
        "imu_update_rate": IMU_RATE_HZ,
        "accel_noise_std": v3(n.accel_nd),
        "gyro_noise_std": v3(n.gyro_nd),
        "accel_bias_std": v3(n.accel_rw),
        "gyro_bias_std": v3(n.gyro_rw),
        "cam_time_offset_ns": 0,
        "vignette": []
    }}))
}

/// `rig` with the cameras `ids` described at the resolution VIO runs at
/// ([`VioScale`]): the image size as [`VioScale::size`], focal lengths times
/// the scale, and the principal point `c' = s·(c + ½) − ½` (pixel centres
/// move with the 2×2 average). Distortion acts on normalised coordinates and
/// is unchanged.
pub fn scaled_rig(rig: &Rig, ids: [&str; 2], scale: VioScale) -> Rig {
    let mut rig = rig.clone();
    let s = scale.factor();
    for sensor in rig
        .sensors
        .iter_mut()
        .filter(|x| ids.contains(&x.id.as_str()))
    {
        if let Some(m) = sensor.camera.as_mut() {
            (m.width, m.height) = scale.size(m.width, m.height);
            m.fx *= s;
            m.fy *= s;
            m.cx = s * (m.cx + 0.5) - 0.5;
            m.cy = s * (m.cy + 0.5) - 0.5;
        }
    }
    rig
}

/// The estimator config: upstream's EuRoC tuning, with real-time mode on so a
/// slow frame is skipped instead of queued. Every key of upstream's
/// `VioConfig` serialiser is present (cereal refuses a missing one).
pub fn basalt_config() -> Value {
    serde_json::from_str(BASALT_CONFIG).expect("BASALT_CONFIG is valid JSON")
}

/// `basalt/vio_config.json`, shared with the synthetic check there.
const BASALT_CONFIG: &str = include_str!("../../../basalt/vio_config.json");
