//! The rig a session records with.
//!
//! The agent stores the operator's rig (`PUT /api/v1/sensors/rig`, validated
//! against the contract schema) at `/etc/ados/rig.json`; that is the rig when
//! present. A drone with no rig document gets one built from the capture
//! config: every enabled camera at the body origin with the configured or
//! derived intrinsics (`calibrated: false` unless overridden), plus every IMU
//! stream the sensor bus delivers. Its `calib_version` says it is derived.

use std::path::{Path, PathBuf};

use ados_capture_contract::{
    CameraModel, ContractId, Rig, RigSensor, RigSensorKind, RigSensorRole,
};
use world_engine_protocol::atlas::CameraRole;

use crate::runtime::AtlasRuntimeConfig;

/// Env override of the rig document path.
pub const RIG_PATH_ENV: &str = "ADOS_RIG_PATH";

/// `/etc/ados/rig.json`.
pub fn rig_path() -> PathBuf {
    std::env::var_os(RIG_PATH_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/etc/ados/rig.json"))
}

/// The stored rig, if present and parseable.
pub fn load_rig(path: &Path) -> Option<Rig> {
    let bytes = std::fs::read(path).ok()?;
    let rig: Rig = match serde_json::from_slice(&bytes) {
        Ok(r) => r,
        Err(e) => {
            tracing::warn!(path = %path.display(), error = %e, "rig_document_unreadable");
            return None;
        }
    };
    if let Err(e) = rig.check() {
        tracing::warn!(path = %path.display(), error = %e, "rig_document_unreadable");
        return None;
    }
    Some(rig)
}

const I4: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
/// OpenCV camera looking along body +x (FLU): camera x = −body y, camera y =
/// −body z, camera z = body x.
const T_BODY_FORWARD_CAM: [f64; 16] = [
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn role(r: CameraRole) -> RigSensorRole {
    match r {
        CameraRole::Primary => RigSensorRole::Primary,
        CameraRole::Aux => RigSensorRole::Texture,
        CameraRole::Down => RigSensorRole::Down,
        CameraRole::Left => RigSensorRole::Left,
        CameraRole::Right => RigSensorRole::Right,
        CameraRole::Back => RigSensorRole::Back,
        CameraRole::Up => RigSensorRole::Up,
    }
}

/// A rig derived from the capture config. `frame_size` gives each camera's
/// frame size seen so far (cameras not yet seen are left out: their model
/// needs a size); `imus` are the sensor-bus IMU stream ids.
pub fn derive_rig(
    cfg: &AtlasRuntimeConfig,
    frame_size: impl Fn(&str) -> Option<(u32, u32)>,
    imus: &[String],
) -> Rig {
    let mut sensors = Vec::new();
    for cam in cfg.capture.enabled_cameras() {
        let Some((w, h)) = frame_size(&cam.id) else {
            continue;
        };
        let model: CameraModel = cfg.intrinsics_for(&cam.id, w, h);
        let t = if cam.role == CameraRole::Primary {
            T_BODY_FORWARD_CAM
        } else {
            I4
        };
        sensors.push(RigSensor {
            id: cam.id.clone(),
            kind: RigSensorKind::Camera,
            role: role(cam.role),
            t_body_sensor: t.into(),
            camera: Some(model),
            lidar: None,
            imu_noise: None,
            time_offset_ns: 0,
            lever_arm_m: None,
        });
    }
    for id in imus {
        sensors.push(RigSensor {
            id: id.clone(),
            kind: RigSensorKind::Imu,
            role: RigSensorRole::Nav,
            t_body_sensor: I4.into(),
            camera: None,
            lidar: None,
            imu_noise: None,
            time_offset_ns: 0,
            lever_arm_m: None,
        });
    }
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: format!("{}-derived", sanitize(&cfg.device_id)),
        calib_version: "derived".into(),
        sensors,
    }
}

fn sanitize(s: &str) -> String {
    let out: String = s
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '-'
            }
        })
        .collect();
    if out.is_empty() {
        "device".into()
    } else {
        out
    }
}

/// Whether the rig carries the stereo pair and IMU VIO needs.
pub fn has_stereo_imu(rig: &Rig, left: &str, right: &str, imu: &str) -> bool {
    let has =
        |id: &str, kind: RigSensorKind| rig.sensors.iter().any(|s| s.id == id && s.kind == kind);
    has(left, RigSensorKind::Camera)
        && has(right, RigSensorKind::Camera)
        && has(imu, RigSensorKind::Imu)
}

/// The rig's LiDAR sensor ids.
pub fn lidar_ids(rig: &Rig) -> Vec<String> {
    rig.sensors
        .iter()
        .filter(|s| s.kind == RigSensorKind::Lidar)
        .map(|s| s.id.clone())
        .collect()
}

/// The rig's first GNSS sensor id, else `gnss0`.
pub fn gnss_id(rig: &Rig) -> String {
    rig.sensors
        .iter()
        .find(|s| s.kind == RigSensorKind::Gnss)
        .map_or_else(|| "gnss0".to_string(), |s| s.id.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn a_derived_rig_holds_seen_cameras_and_bus_imus() {
        let values: BTreeMap<String, serde_json::Value> = [(
            "atlas.cameras".to_string(),
            serde_json::json!([{"id": "front"}, {"id": "down", "role": "down"}]),
        )]
        .into_iter()
        .collect();
        let cfg = AtlasRuntimeConfig::from_values("drone 1", &values).unwrap();
        let rig = derive_rig(
            &cfg,
            |id| (id == "front").then_some((640, 480)),
            &["imu0".into()],
        );
        assert_eq!(rig.rig_id, "drone-1-derived");
        assert_eq!(rig.sensors.len(), 2);
        assert_eq!(rig.sensors[0].role, RigSensorRole::Primary);
        assert!(!rig.sensors[0].camera.as_ref().unwrap().calibrated);
        assert_eq!(rig.sensors[1].kind, RigSensorKind::Imu);
        assert!(!has_stereo_imu(&rig, "cam_fl", "cam_fr", "imu0"));
        assert_eq!(gnss_id(&rig), "gnss0");
        // The derived rig is a valid contract document.
        let doc = serde_json::to_value(&rig).unwrap();
        let back: Rig = serde_json::from_value(doc.clone()).unwrap();
        assert_eq!(serde_json::to_value(&back).unwrap(), doc);
    }

    #[test]
    fn a_missing_or_broken_rig_file_is_none() {
        let dir = tempfile::tempdir().unwrap();
        assert!(load_rig(&dir.path().join("rig.json")).is_none());
        std::fs::write(dir.path().join("rig.json"), b"{").unwrap();
        assert!(load_rig(&dir.path().join("rig.json")).is_none());
    }
}
