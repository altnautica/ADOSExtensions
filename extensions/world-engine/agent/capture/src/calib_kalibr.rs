//! Import a Kalibr camera-IMU calibration (`camchain-imucam.yaml` plus the
//! `imu.yaml` noise model) as a contract [`Rig`].
//!
//! - Body frame = the IMU frame, unless `imu.yaml` carries `T_i_b` (body to
//!   IMU), in which case `T_body_imu = T_i_b⁻¹`.
//! - Each `camN` becomes sensor `camN` with `T_body_sensor = T_body_imu ·
//!   T_cam_imu⁻¹`; a camera without `T_cam_imu` (a camera-only calibration)
//!   sits at the body origin.
//! - `pinhole` + `radtan` → `opencv` `[k1, k2, p1, p2, 0]`; `pinhole` +
//!   `equidistant` → `opencv_fisheye` `[k1..k4]`; `pinhole` + `none` →
//!   `pinhole`. Other projection models are refused.
//! - Kalibr's `timeshift_cam_imu` is `t_imu = t_cam + shift`; with the IMU
//!   clock as the session timebase it is the camera's `time_offset_ns`. The
//!   IMU's own offset is `imu.yaml` `time_offset` (0 when absent).
//! - Roles: one camera `primary`; two `front_left` / `front_right`; any
//!   further camera `nav`. The IMU is `nav`. Edit the rig afterwards when the
//!   mounting says otherwise.

use std::collections::BTreeMap;

use ados_capture_contract::{
    CameraModel, CameraModelModel, ContractId, ImuNoise, Rig, RigSensor, RigSensorKind,
    RigSensorRole, Transform,
};
use serde::Deserialize;

/// The IMU sensor id the import creates.
pub const IMU_ID: &str = "imu0";

#[derive(Debug, thiserror::Error)]
pub enum KalibrError {
    #[error("kalibr yaml: {0}")]
    Yaml(String),
    #[error("unsupported kalibr model: {0}")]
    Unsupported(String),
    #[error("invalid kalibr calibration: {0}")]
    Invalid(String),
}

#[derive(Debug, Deserialize)]
struct KalibrCamera {
    camera_model: String,
    intrinsics: Vec<f64>,
    #[serde(default)]
    distortion_model: Option<String>,
    #[serde(default)]
    distortion_coeffs: Vec<f64>,
    resolution: [u32; 2],
    #[serde(default, rename = "T_cam_imu")]
    t_cam_imu: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    timeshift_cam_imu: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct KalibrImu {
    accelerometer_noise_density: f64,
    accelerometer_random_walk: f64,
    gyroscope_noise_density: f64,
    gyroscope_random_walk: f64,
    #[serde(default, rename = "T_i_b")]
    t_i_b: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    time_offset: Option<f64>,
}

type M4 = [f64; 16];

const IDENTITY: M4 = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// A Kalibr 4x4 list-of-rows as a row-major array.
fn matrix(rows: &[Vec<f64>], what: &str) -> Result<M4, KalibrError> {
    if rows.len() != 4 || rows.iter().any(|r| r.len() != 4) {
        return Err(KalibrError::Invalid(format!("{what} is not 4x4")));
    }
    let mut m = [0.0; 16];
    for (i, row) in rows.iter().enumerate() {
        m[i * 4..i * 4 + 4].copy_from_slice(row);
    }
    if m.iter().any(|v| !v.is_finite()) {
        return Err(KalibrError::Invalid(format!("{what} is not finite")));
    }
    Ok(m)
}

/// Inverse of a rigid transform: `[Rᵀ | -Rᵀt]`.
fn invert_rigid(m: &M4) -> M4 {
    let r = |i: usize, j: usize| m[i * 4 + j];
    let t = [m[3], m[7], m[11]];
    let mut out = IDENTITY;
    for i in 0..3 {
        for j in 0..3 {
            out[i * 4 + j] = r(j, i);
        }
        out[i * 4 + 3] = -(0..3).map(|k| r(k, i) * t[k]).sum::<f64>();
    }
    out
}

fn mul(a: &M4, b: &M4) -> M4 {
    let mut out = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            out[i * 4 + j] = (0..4).map(|k| a[i * 4 + k] * b[k * 4 + j]).sum();
        }
    }
    out
}

fn seconds_to_ns(s: f64) -> i64 {
    (s * 1e9).round() as i64
}

fn camera_model(cam: &KalibrCamera, id: &str) -> Result<CameraModel, KalibrError> {
    if cam.camera_model != "pinhole" {
        return Err(KalibrError::Unsupported(format!(
            "{id}: camera_model `{}`",
            cam.camera_model
        )));
    }
    let [fx, fy, cx, cy] = <[f64; 4]>::try_from(cam.intrinsics.as_slice())
        .map_err(|_| KalibrError::Invalid(format!("{id}: intrinsics must be [fu, fv, pu, pv]")))?;
    let coeffs = &cam.distortion_coeffs;
    let (model, dist) = match cam.distortion_model.as_deref().unwrap_or("none") {
        "radtan" if coeffs.len() == 4 => (
            CameraModelModel::Opencv,
            vec![coeffs[0], coeffs[1], coeffs[2], coeffs[3], 0.0],
        ),
        "equidistant" if coeffs.len() == 4 => (CameraModelModel::OpencvFisheye, coeffs.clone()),
        "none" => (CameraModelModel::Pinhole, Vec::new()),
        "radtan" | "equidistant" => {
            return Err(KalibrError::Invalid(format!(
                "{id}: expected 4 distortion coefficients, got {}",
                coeffs.len()
            )))
        }
        other => {
            return Err(KalibrError::Unsupported(format!(
                "{id}: distortion_model `{other}`"
            )))
        }
    };
    Ok(CameraModel {
        model,
        width: cam.resolution[0],
        height: cam.resolution[1],
        fx,
        fy,
        cx,
        cy,
        dist,
        calibrated: true,
        readout_ns: 0,
    })
}

fn camera_role(index: usize, count: usize) -> RigSensorRole {
    match (count, index) {
        (1, _) => RigSensorRole::Primary,
        (_, 0) => RigSensorRole::FrontLeft,
        (_, 1) => RigSensorRole::FrontRight,
        _ => RigSensorRole::Nav,
    }
}

/// The `camN` index of a camchain key.
fn cam_index(key: &str) -> Option<u32> {
    key.strip_prefix("cam")?.parse().ok()
}

fn parse_imu(text: &str) -> Result<KalibrImu, KalibrError> {
    let doc: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(text).map_err(|e| KalibrError::Yaml(e.to_string()))?;
    // Kalibr writes the result nested under `imu0`; its input file is flat.
    let body = match doc.get(IMU_ID) {
        Some(nested) => nested.clone(),
        None => doc,
    };
    serde_yaml_ng::from_value(body).map_err(|e| KalibrError::Yaml(e.to_string()))
}

/// Build a rig from a Kalibr `camchain-imucam.yaml` and, when given, its
/// `imu.yaml`.
pub fn import_kalibr(
    rig_id: &str,
    calib_version: &str,
    camchain_yaml: &str,
    imu_yaml: Option<&str>,
) -> Result<Rig, KalibrError> {
    let chain: BTreeMap<String, KalibrCamera> =
        serde_yaml_ng::from_str(camchain_yaml).map_err(|e| KalibrError::Yaml(e.to_string()))?;
    let mut cams: Vec<(u32, String, KalibrCamera)> = chain
        .into_iter()
        .map(|(k, v)| {
            cam_index(&k)
                .map(|i| (i, k.clone(), v))
                .ok_or_else(|| KalibrError::Invalid(format!("camchain key `{k}` is not camN")))
        })
        .collect::<Result<_, _>>()?;
    if cams.is_empty() {
        return Err(KalibrError::Invalid("camchain lists no camera".into()));
    }
    cams.sort_by_key(|(i, _, _)| *i);

    let imu = imu_yaml.map(parse_imu).transpose()?;
    let t_body_imu = match imu.as_ref().and_then(|i| i.t_i_b.as_ref()) {
        Some(rows) => invert_rigid(&matrix(rows, "T_i_b")?),
        None => IDENTITY,
    };

    let count = cams.len();
    let mut sensors = Vec::with_capacity(count + 1);
    let mut has_imu_extrinsics = false;
    for (index, (_, id, cam)) in cams.iter().enumerate() {
        let t_body_cam = match &cam.t_cam_imu {
            Some(rows) => {
                has_imu_extrinsics = true;
                mul(
                    &t_body_imu,
                    &invert_rigid(&matrix(rows, &format!("{id} T_cam_imu"))?),
                )
            }
            None => IDENTITY,
        };
        sensors.push(RigSensor {
            camera: Some(camera_model(cam, id)?),
            id: id.clone(),
            imu_noise: None,
            kind: RigSensorKind::Camera,
            lever_arm_m: None,
            role: camera_role(index, count),
            t_body_sensor: Transform::from(t_body_cam),
            time_offset_ns: cam.timeshift_cam_imu.map_or(0, seconds_to_ns),
        });
    }
    if imu.is_some() || has_imu_extrinsics {
        sensors.push(RigSensor {
            camera: None,
            id: IMU_ID.to_string(),
            imu_noise: imu.as_ref().map(|i| ImuNoise {
                accel_nd: i.accelerometer_noise_density,
                accel_rw: i.accelerometer_random_walk,
                gyro_nd: i.gyroscope_noise_density,
                gyro_rw: i.gyroscope_random_walk,
            }),
            kind: RigSensorKind::Imu,
            lever_arm_m: None,
            role: RigSensorRole::Nav,
            t_body_sensor: Transform::from(t_body_imu),
            time_offset_ns: imu
                .as_ref()
                .and_then(|i| i.time_offset)
                .map_or(0, seconds_to_ns),
        });
    }
    Ok(Rig {
        calib_version: calib_version.to_string(),
        contract: ContractId::AdosCapture1,
        rig_id: rig_id.to_string(),
        sensors,
    })
}
