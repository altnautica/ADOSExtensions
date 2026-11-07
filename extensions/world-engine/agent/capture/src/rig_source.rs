//! The rig a session records with.
//!
//! The agent stores the operator's rig (`PUT /api/v1/sensors/rig`, validated
//! against the contract schema) at `/etc/ados/rig.json`; that is the rig when
//! present. A drone with no rig document gets one built from the capture
//! config: every enabled camera at the body origin with the configured or
//! derived intrinsics (`calibrated: false` unless overridden), plus every IMU
//! stream the sensor bus delivers. Its `calib_version` says it is derived.
//!
//! A stored rig that lacks an enabled camera gets that camera added the same
//! way ([`complete_rig`]): the session records `/cam/<id>/image` for every
//! enabled camera, and a reconstruction reads only the rig's cameras, so a
//! camera missing from the rig would be recorded and never used.

use std::path::{Path, PathBuf};

use ados_capture_contract::{
    CameraModel, ContractId, LidarModel, Rig, RigSensor, RigSensorKind, RigSensorRole,
};
use world_engine_protocol::atlas::CameraRole;

use crate::config::CameraConfig;
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

/// A camera of the capture config as a rig sensor: the configured or derived
/// intrinsics at `w` x `h`, a primary camera looking along body +x, any
/// other at the body frame.
fn camera_sensor(cfg: &AtlasRuntimeConfig, cam: &CameraConfig, w: u32, h: u32) -> RigSensor {
    let model: CameraModel = cfg.intrinsics_for(&cam.id, w, h);
    let t = if cam.role == CameraRole::Primary {
        T_BODY_FORWARD_CAM
    } else {
        I4
    };
    RigSensor {
        id: cam.id.clone(),
        kind: RigSensorKind::Camera,
        role: role(cam.role),
        t_body_sensor: t.into(),
        camera: Some(model),
        lidar: None,
        imu_noise: None,
        time_offset_ns: 0,
        lever_arm_m: None,
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
        sensors.push(camera_sensor(cfg, cam, w, h));
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

/// The stored rig plus every enabled, already-seen camera it has no sensor
/// for, built as [`derive_rig`] builds it; returns the rig and the added
/// camera ids. The stored sensors are kept as they are.
pub fn complete_rig(
    mut rig: Rig,
    cfg: &AtlasRuntimeConfig,
    frame_size: impl Fn(&str) -> Option<(u32, u32)>,
) -> (Rig, Vec<String>) {
    let mut added = Vec::new();
    for cam in cfg.capture.enabled_cameras() {
        if rig.sensors.iter().any(|s| s.id == cam.id) {
            continue;
        }
        let Some((w, h)) = frame_size(&cam.id) else {
            continue;
        };
        rig.sensors.push(camera_sensor(cfg, cam, w, h));
        added.push(cam.id.clone());
    }
    (rig, added)
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

/// The rig's LiDAR sensor `id` with its scan model.
pub fn lidar_sensor<'a>(rig: &'a Rig, id: &str) -> Option<(&'a RigSensor, &'a LidarModel)> {
    rig.sensors
        .iter()
        .find(|s| s.id == id && s.kind == RigSensorKind::Lidar)
        .and_then(|s| s.lidar.as_ref().map(|m| (s, m)))
}

/// Whether the rig carries the LiDAR (with its scan model) and the IMU LiDAR
/// odometry needs.
pub fn has_lidar_imu(rig: &Rig, lidar: &str, imu: &str) -> bool {
    lidar_sensor(rig, lidar).is_some()
        && rig
            .sensors
            .iter()
            .any(|s| s.id == imu && s.kind == RigSensorKind::Imu)
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

    #[test]
    fn a_stored_rig_gains_the_enabled_cameras_it_lacks() {
        let stored: Rig = serde_json::from_value(serde_json::json!({
            "contract": "ados.capture/1", "rig_id": "x500", "calib_version": "1",
            "sensors": [
                {"id": "fc", "kind": "imu", "role": "nav", "time_offset_ns": 0,
                 "T_body_sensor": I4},
                {"id": "belly", "kind": "camera", "role": "down", "time_offset_ns": 0,
                 "T_body_sensor": I4,
                 "camera": {"model": "pinhole", "width": 640, "height": 480, "fx": 400.0,
                            "fy": 400.0, "cx": 320.0, "cy": 240.0, "dist": [],
                            "calibrated": true, "readout_ns": 0}}
            ]
        }))
        .unwrap();
        let values: BTreeMap<String, serde_json::Value> = [
            (
                "atlas.cameras".to_string(),
                serde_json::json!([
                    {"id": "main"}, {"id": "belly", "role": "down"}, {"id": "aux", "role": "aux"}
                ]),
            ),
            ("atlas.hfov_deg".to_string(), serde_json::json!(87.0)),
        ]
        .into_iter()
        .collect();
        let cfg = AtlasRuntimeConfig::from_values("drone 1", &values).unwrap();
        // `aux` has delivered no frame yet: no size, no model, left out.
        let (rig, added) = complete_rig(stored.clone(), &cfg, |id| {
            (id != "aux").then_some((1280, 720))
        });
        assert_eq!(added, ["main"]);
        assert_eq!(rig.rig_id, "x500");
        assert_eq!(rig.calib_version, "1");
        let json = |s: &[RigSensor]| serde_json::to_value(s).unwrap();
        assert_eq!(json(&rig.sensors[..2]), json(&stored.sensors));
        let main = &rig.sensors[2];
        assert_eq!((main.kind, main.role), (RigSensorKind::Camera, RigSensorRole::Primary));
        let cam = main.camera.as_ref().unwrap();
        assert_eq!((cam.width, cam.height), (1280, 720));
        assert!((cam.fx - 640.0 / 43.5_f64.to_radians().tan()).abs() < 1e-9);
        assert!(!cam.calibrated);
        assert!(rig.check().is_ok());
        // `aux` joins once it has a frame size; then the rig is complete and
        // comes back unchanged.
        let (rig, added) = complete_rig(rig, &cfg, |_| Some((1280, 720)));
        assert_eq!(added, ["aux"]);
        let (again, none) = complete_rig(rig.clone(), &cfg, |_| Some((1280, 720)));
        assert!(none.is_empty());
        assert_eq!(json(&again.sensors), json(&rig.sensors));
    }
}
