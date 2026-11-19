//! Kalibr YAML in both directions: the target / IMU / camera-chain files a
//! Kalibr run reads, and the camera chain, IMU file and results text it
//! writes, folded back into a rig.
//!
//! Camera `cam<i>` is the i-th `camera` sensor of the rig in rig order; the
//! bag topic is `/cam<i>/image_raw`. The exported IMU samples are already in
//! the body frame (FLU), so Kalibr's IMU frame has body axes with its origin at
//! the IMU: `T_body_kimu` is the rig IMU's translation with identity rotation
//! (identity when the rig has no IMU).

use ados_capture_contract::{CameraModel, CameraModelModel, ImuNoise, Rig, RigSensorKind};
use serde::Deserialize;

use super::geom::{inv_rigid, mul, translation_only, Mat4, IDENTITY};
use super::AprilGrid;
use crate::recon::StageError;

/// Indexes into `rig.sensors` of the cameras, in Kalibr `cam<i>` order.
pub fn rig_cameras(rig: &Rig) -> Vec<usize> {
    (0..rig.sensors.len())
        .filter(|&i| rig.sensors[i].kind == RigSensorKind::Camera)
        .collect()
}

/// Index into `rig.sensors` of the IMU a calibration uses (the first one).
pub fn rig_imu(rig: &Rig) -> Option<usize> {
    rig.sensors
        .iter()
        .position(|s| s.kind == RigSensorKind::Imu)
}

/// A float as YAML 1.1 reads it back as a float (always with a `.`).
pub fn yaml_f(v: f64) -> String {
    let s = format!("{v}");
    if s.contains('.') || !v.is_finite() {
        s
    } else {
        format!("{s}.0")
    }
}

fn yaml_list(v: &[f64]) -> String {
    let items: Vec<String> = v.iter().map(|x| yaml_f(*x)).collect();
    format!("[{}]", items.join(", "))
}

/// `target.yaml` for an AprilGrid.
pub fn target_yaml(b: &AprilGrid) -> String {
    format!(
        "target_type: 'aprilgrid'\ntagCols: {}\ntagRows: {}\ntagSize: {}\ntagSpacing: {}\n",
        b.tag_cols,
        b.tag_rows,
        yaml_f(b.tag_size_m),
        yaml_f(b.tag_spacing)
    )
}

/// `imu.yaml` for the `/imu0` topic.
pub fn imu_yaml(n: &ImuNoise, update_rate_hz: f64) -> String {
    format!(
        "rostopic: /imu0\nupdate_rate: {}\naccelerometer_noise_density: {}\naccelerometer_random_walk: {}\ngyroscope_noise_density: {}\ngyroscope_random_walk: {}\n",
        yaml_f(update_rate_hz),
        yaml_f(n.accel_nd),
        yaml_f(n.accel_rw),
        yaml_f(n.gyro_nd),
        yaml_f(n.gyro_rw)
    )
}

/// Kalibr's camera model argument (`--models`) for a rig camera model.
pub fn kalibr_model(m: CameraModelModel) -> &'static str {
    match m {
        CameraModelModel::Kb4 | CameraModelModel::OpencvFisheye => "pinhole-equi",
        CameraModelModel::Pinhole | CameraModelModel::Opencv => "pinhole-radtan",
    }
}

fn camera_of(rig: &Rig, idx: usize) -> Result<&CameraModel, StageError> {
    let s = &rig.sensors[idx];
    s.camera
        .as_ref()
        .ok_or_else(|| StageError::input(format!("camera {} has no projection model", s.id)))
}

/// The rig's cameras as a Kalibr camera chain (`--cams` input of the
/// camera-IMU calibration).
pub fn camchain_yaml(rig: &Rig) -> Result<String, StageError> {
    let cams = rig_cameras(rig);
    if cams.is_empty() {
        return Err(StageError::input("the rig has no camera"));
    }
    let mut out = String::new();
    for (i, &idx) in cams.iter().enumerate() {
        let c = camera_of(rig, idx)?;
        let (model, coeffs): (&str, Vec<f64>) = match c.model {
            CameraModelModel::Pinhole => ("none", vec![]),
            CameraModelModel::Opencv => ("radtan", c.dist.iter().take(4).copied().collect()),
            CameraModelModel::Kb4 | CameraModelModel::OpencvFisheye => {
                ("equidistant", c.dist.clone())
            }
        };
        out.push_str(&format!("cam{i}:\n"));
        if i > 0 {
            // T_cn_cnm1 maps points from camera i-1 into camera i.
            let t = mul(
                &inv_rigid(&rig.sensors[idx].t_body_sensor.0),
                &rig.sensors[cams[i - 1]].t_body_sensor.0,
            );
            out.push_str("  T_cn_cnm1:\n");
            for r in 0..4 {
                out.push_str(&format!("  - {}\n", yaml_list(&t[r * 4..r * 4 + 4])));
            }
        }
        out.push_str("  camera_model: pinhole\n");
        out.push_str(&format!(
            "  intrinsics: {}\n",
            yaml_list(&[c.fx, c.fy, c.cx, c.cy])
        ));
        out.push_str(&format!("  distortion_model: {model}\n"));
        out.push_str(&format!("  distortion_coeffs: {}\n", yaml_list(&coeffs)));
        out.push_str(&format!("  resolution: [{}, {}]\n", c.width, c.height));
        out.push_str(&format!("  rostopic: /cam{i}/image_raw\n"));
    }
    Ok(out)
}

#[derive(Debug, Deserialize)]
struct KalibrCam {
    camera_model: String,
    intrinsics: Vec<f64>,
    distortion_model: String,
    #[serde(default)]
    distortion_coeffs: Vec<f64>,
    resolution: [u32; 2],
    #[serde(default)]
    rostopic: Option<String>,
    #[serde(rename = "T_cam_imu", default)]
    t_cam_imu: Option<Vec<Vec<f64>>>,
    #[serde(rename = "T_cn_cnm1", default)]
    t_cn_cnm1: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    timeshift_cam_imu: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct KalibrImu {
    accelerometer_noise_density: f64,
    accelerometer_random_walk: f64,
    gyroscope_noise_density: f64,
    gyroscope_random_walk: f64,
}

fn yaml_err(what: &str, e: impl std::fmt::Display) -> StageError {
    StageError::input(format!("{what}: {e}"))
}

fn mat4(rows: &[Vec<f64>], what: &str) -> Result<Mat4, StageError> {
    if rows.len() != 4 || rows.iter().any(|r| r.len() != 4) {
        return Err(StageError::input(format!("{what} is not a 4x4 matrix")));
    }
    let mut m = [0.0; 16];
    for (r, row) in rows.iter().enumerate() {
        m[r * 4..r * 4 + 4].copy_from_slice(row);
    }
    Ok(m)
}

/// `cam<i>` index from the rostopic `/cam<i>/image_raw`, else from the key.
fn cam_index(key: &str, cam: &KalibrCam) -> Result<usize, StageError> {
    let from_topic = cam.rostopic.as_deref().and_then(|t| {
        t.strip_prefix("/cam")?
            .strip_suffix("/image_raw")?
            .parse()
            .ok()
    });
    from_topic
        .or_else(|| key.strip_prefix("cam")?.parse().ok())
        .ok_or_else(|| StageError::input(format!("camera chain entry `{key}` has no cam index")))
}

fn camera_model(key: &str, k: &KalibrCam, readout_ns: u64) -> Result<CameraModel, StageError> {
    if k.camera_model != "pinhole" {
        return Err(StageError::input(format!(
            "{key}: camera model `{}` is not supported",
            k.camera_model
        )));
    }
    let [fx, fy, cx, cy] = <[f64; 4]>::try_from(k.intrinsics.as_slice())
        .map_err(|_| StageError::input(format!("{key}: intrinsics need 4 values")))?;
    let need = |n: usize| {
        if k.distortion_coeffs.len() == n {
            Ok(())
        } else {
            Err(StageError::input(format!(
                "{key}: {} distortion needs {n} coefficients",
                k.distortion_model
            )))
        }
    };
    let (model, dist) = match k.distortion_model.as_str() {
        "radtan" => {
            need(4)?;
            let mut d = k.distortion_coeffs.clone();
            d.push(0.0);
            (CameraModelModel::Opencv, d)
        }
        "equidistant" => {
            need(4)?;
            (CameraModelModel::Kb4, k.distortion_coeffs.clone())
        }
        "none" => (CameraModelModel::Pinhole, vec![]),
        other => {
            return Err(StageError::input(format!(
                "{key}: distortion model `{other}` is not supported"
            )))
        }
    };
    Ok(CameraModel {
        model,
        width: k.resolution[0],
        height: k.resolution[1],
        fx,
        fy,
        cx,
        cy,
        dist,
        calibrated: true,
        readout_ns,
    })
}

/// IMU noise from a Kalibr IMU file: the flat input form or the `imu0:` form
/// Kalibr writes back.
pub fn imu_noise_from_kalibr(imu_yaml: &str) -> Result<ImuNoise, StageError> {
    let v: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(imu_yaml).map_err(|e| yaml_err("imu yaml", e))?;
    let v = v.get("imu0").cloned().unwrap_or(v);
    let k: KalibrImu = serde_yaml_ng::from_value(v).map_err(|e| yaml_err("imu yaml", e))?;
    Ok(ImuNoise {
        gyro_nd: k.gyroscope_noise_density,
        accel_nd: k.accelerometer_noise_density,
        gyro_rw: k.gyroscope_random_walk,
        accel_rw: k.accelerometer_random_walk,
    })
}

/// Fold a Kalibr camera chain (and optionally its IMU file) into `base`.
///
/// Intrinsics, distortion (`radtan` → `opencv` with k3 = 0, `equidistant` →
/// `kb4`) and resolution replace each camera's model, marked calibrated.
/// `T_cam_imu` gives `T_body_cam = T_body_kimu · T_cam_imu⁻¹`; without it a
/// camera after the first chains off its predecessor through `T_cn_cnm1`.
/// `timeshift_cam_imu` (seconds, `t_imu = t_cam + shift`) is added to the
/// camera's `time_offset_ns`. IMU noise lands on the rig's first IMU.
pub fn rig_from_kalibr(
    base: &Rig,
    camchain_yaml: &str,
    imu_yaml: Option<&str>,
) -> Result<Rig, StageError> {
    let chain: std::collections::BTreeMap<String, KalibrCam> =
        serde_yaml_ng::from_str(camchain_yaml).map_err(|e| yaml_err("camera chain", e))?;
    let mut entries = chain
        .iter()
        .map(|(k, c)| Ok((cam_index(k, c)?, k.as_str(), c)))
        .collect::<Result<Vec<_>, StageError>>()?;
    entries.sort_by_key(|e| e.0);
    let mut rig = base.clone();
    let cams = rig_cameras(&rig);
    let imu = rig_imu(&rig);
    let t_body_kimu = imu
        .map(|i| translation_only(&rig.sensors[i].t_body_sensor.0))
        .unwrap_or(IDENTITY);
    for (n, &(i, key, k)) in entries.iter().enumerate() {
        let &idx = cams.get(i).ok_or_else(|| {
            StageError::input(format!(
                "{key} has no matching rig camera (rig has {})",
                cams.len()
            ))
        })?;
        let readout = rig.sensors[idx]
            .camera
            .as_ref()
            .map(|c| c.readout_ns)
            .unwrap_or(0);
        let model = camera_model(key, k, readout)?;
        let t_body_cam = if let Some(t) = &k.t_cam_imu {
            Some(mul(&t_body_kimu, &inv_rigid(&mat4(t, "T_cam_imu")?)))
        } else if let (Some(t), true) = (&k.t_cn_cnm1, n > 0 && entries[n - 1].0 + 1 == i) {
            let prev = rig.sensors[cams[i - 1]].t_body_sensor.0;
            Some(mul(&prev, &inv_rigid(&mat4(t, "T_cn_cnm1")?)))
        } else {
            None
        };
        let sensor = &mut rig.sensors[idx];
        sensor.camera = Some(model);
        if let Some(t) = t_body_cam {
            sensor.t_body_sensor = t.into();
        }
        if let Some(shift) = k.timeshift_cam_imu {
            sensor.time_offset_ns += (shift * 1e9).round() as i64;
        }
    }
    if let Some(y) = imu_yaml {
        let noise = imu_noise_from_kalibr(y)?;
        let i =
            imu.ok_or_else(|| StageError::input("an IMU file was given but the rig has no IMU"))?;
        rig.sensors[i].imu_noise = Some(noise);
    }
    Ok(rig)
}

fn numbers(s: &str) -> Vec<f64> {
    s.split(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | 'e' | 'E' | '+')))
        .filter_map(|t| t.parse().ok())
        .collect()
}

/// Per-camera reprojection RMS (px) from a `*-results-cam.txt`
/// (`reprojection error: [mx, my] +- [sx, sy]` under `cam<i> (...)`).
pub fn rms_from_cam_results(txt: &str) -> Vec<(usize, f64)> {
    let mut out = Vec::new();
    let mut cam: Option<usize> = None;
    for line in txt.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("cam") {
            cam = rest.split(' ').next().and_then(|n| n.parse().ok());
        } else if let (Some(rest), Some(c)) = (t.strip_prefix("reprojection error:"), cam) {
            let v = numbers(rest);
            if v.len() == 4 {
                out.push((c, v.iter().map(|x| x * x).sum::<f64>().sqrt()));
            }
        }
    }
    out
}

/// Per-camera reprojection RMS (px) from a `*-results-imucam.txt`
/// (`Reprojection error (cam<i>) [px]: mean m, median d, std: s`; the RMS
/// of the per-corner error norm is `√(m² + s²)`).
pub fn rms_from_imucam_results(txt: &str) -> Vec<(usize, f64)> {
    let mut out = Vec::new();
    for line in txt.lines() {
        let Some(rest) = line.trim().strip_prefix("Reprojection error (cam") else {
            continue;
        };
        let Some((idx, tail)) = rest.split_once(") [px]:") else {
            continue;
        };
        let (Ok(c), Some(mean), Some(std)) = (
            idx.parse::<usize>(),
            tail.split("mean")
                .nth(1)
                .and_then(|s| numbers(s).first().copied()),
            tail.split("std:")
                .nth(1)
                .and_then(|s| numbers(s).first().copied()),
        ) else {
            continue;
        };
        out.push((c, (mean * mean + std * std).sqrt()));
    }
    out
}

/// Root mean square over the cameras' RMS values.
pub fn combined_rms(per_cam: &[(usize, f64)]) -> Option<f64> {
    if per_cam.is_empty() {
        return None;
    }
    let ms = per_cam.iter().map(|(_, r)| r * r).sum::<f64>() / per_cam.len() as f64;
    Some(ms.sqrt())
}

#[cfg(test)]
#[path = "kalibr_yaml_tests.rs"]
mod tests;
