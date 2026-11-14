//! The rig a session records with.
//!
//! The agent stores the operator's rig (`PUT /api/v1/sensors/rig`, validated
//! against the contract schema) at `/etc/ados/rig.json`; that is the rig when
//! present. A drone with no rig document gets one built from the capture
//! config and the cameras' frames: every enabled camera at the body origin,
//! pointed and given a field of view as the agent's camera roster declares
//! (`cameras[].mount_pitch_deg`, `cameras[].fov_deg`, which the vision engine
//! stamps on every frame), else as its role implies (a down camera looks
//! down), with the configured intrinsics or a pinhole derived from that field
//! of view (`calibrated: false` unless overridden), plus every IMU stream the
//! sensor bus delivers. Its `calib_version` says it is derived.
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

/// A camera's lens and mount as the agent's camera roster declares them,
/// carried on each of its frames. `None` where the roster leaves it out.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CameraGeometry {
    /// Horizontal field of view in degrees (`cameras[].fov_deg`).
    pub hfov_deg: Option<f64>,
    /// Optical-axis pitch in degrees, negative below the nose
    /// (`cameras[].mount_pitch_deg`).
    pub mount_pitch_deg: Option<f64>,
}

/// What the service has seen of a camera: its frame size and the geometry
/// its frames carry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeenCamera {
    pub width: u32,
    pub height: u32,
    pub geometry: CameraGeometry,
}

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

/// Where a camera of role `r` points, as degrees of yaw left of the nose and
/// pitch above the horizon, when the roster gives no pitch: a down camera
/// straight down (image top toward the nose), an up camera straight up, side
/// and back cameras level along their side.
fn role_mount(r: CameraRole) -> (f64, f64) {
    match r {
        CameraRole::Primary | CameraRole::Aux => (0.0, 0.0),
        CameraRole::Down => (0.0, -90.0),
        CameraRole::Up => (0.0, 90.0),
        CameraRole::Left => (90.0, 0.0),
        CameraRole::Right => (-90.0, 0.0),
        CameraRole::Back => (180.0, 0.0),
    }
}

/// `T_body_sensor` (row-major, body FLU) of an OpenCV camera (x right, y
/// down, z along the optical axis) at the body origin, yawed `yaw_deg` left
/// of the nose and pitched `pitch_deg` above the horizon. Level and forward,
/// camera x is −body y, camera y is −body z and camera z is body x.
pub fn t_body_cam(yaw_deg: f64, pitch_deg: f64) -> [f64; 16] {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    // A level axis (forward part `fx`, up part `fz`) pitched up about body
    // −y, then yawed about body z.
    let axis = |fx: f64, fz: f64| {
        let (x, z) = (fx * cp - fz * sp, fx * sp + fz * cp);
        [x * cy, x * sy, z]
    };
    let cam_x = [sy, -cy, 0.0];
    let cam_y = axis(0.0, -1.0);
    let cam_z = axis(1.0, 0.0);
    let clean = |v: f64| if v.abs() < 1e-12 { 0.0 } else { v };
    let mut t = I4;
    for row in 0..3 {
        t[row * 4] = clean(cam_x[row]);
        t[row * 4 + 1] = clean(cam_y[row]);
        t[row * 4 + 2] = clean(cam_z[row]);
    }
    t
}

/// A camera of the capture config as a rig sensor: the configured or derived
/// intrinsics at the seen frame size, pointed as the roster's pitch (else the
/// camera's role) says.
fn camera_sensor(cfg: &AtlasRuntimeConfig, cam: &CameraConfig, seen: SeenCamera) -> RigSensor {
    let model: CameraModel =
        cfg.intrinsics_for(&cam.id, seen.width, seen.height, seen.geometry.hfov_deg);
    let (yaw, role_pitch) = role_mount(cam.role);
    let pitch = seen.geometry.mount_pitch_deg.unwrap_or(role_pitch);
    RigSensor {
        id: cam.id.clone(),
        kind: RigSensorKind::Camera,
        role: role(cam.role),
        t_body_sensor: t_body_cam(yaw, pitch).into(),
        camera: Some(model),
        lidar: None,
        imu_noise: None,
        time_offset_ns: 0,
        lever_arm_m: None,
    }
}

/// A rig derived from the capture config. `seen` gives each camera's frame
/// size and geometry seen so far (cameras not yet seen are left out: their
/// model needs a size); `imus` are the sensor-bus IMU stream ids.
pub fn derive_rig(
    cfg: &AtlasRuntimeConfig,
    seen: impl Fn(&str) -> Option<SeenCamera>,
    imus: &[String],
) -> Rig {
    let mut sensors = Vec::new();
    for cam in cfg.capture.enabled_cameras() {
        let Some(s) = seen(&cam.id) else {
            continue;
        };
        sensors.push(camera_sensor(cfg, cam, s));
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
    seen: impl Fn(&str) -> Option<SeenCamera>,
) -> (Rig, Vec<String>) {
    let mut added = Vec::new();
    for cam in cfg.capture.enabled_cameras() {
        if rig.sensors.iter().any(|s| s.id == cam.id) {
            continue;
        }
        let Some(s) = seen(&cam.id) else {
            continue;
        };
        rig.sensors.push(camera_sensor(cfg, cam, s));
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
    use crate::runtime::DEFAULT_HFOV_DEG;
    use std::collections::BTreeMap;

    fn seen(width: u32, height: u32, hfov: Option<f64>, pitch: Option<f64>) -> SeenCamera {
        SeenCamera {
            width,
            height,
            geometry: CameraGeometry {
                hfov_deg: hfov,
                mount_pitch_deg: pitch,
            },
        }
    }

    fn t_of(s: &RigSensor) -> Vec<f64> {
        serde_json::from_value(serde_json::to_value(&s.t_body_sensor).unwrap()).unwrap()
    }

    fn assert_t(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), 16);
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-6, "{got:?} vs {want:?}");
        }
    }

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
            |id| (id == "front").then_some(seen(640, 480, None, None)),
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

    /// The x500-ados mounts: a front camera of 87 degrees pitched 15 degrees
    /// down and a belly camera of 90 degrees looking straight down, the same
    /// extrinsics a hand-written rig for that airframe carries.
    #[test]
    fn derived_cameras_point_as_the_roster_or_their_role_says() {
        let values: BTreeMap<String, serde_json::Value> = [(
            "atlas.cameras".to_string(),
            serde_json::json!([{"id": "main"}, {"id": "belly", "role": "down"}]),
        )]
        .into_iter()
        .collect();
        let cfg = AtlasRuntimeConfig::from_values("x500", &values).unwrap();
        let rig = derive_rig(
            &cfg,
            |id| match id {
                "main" => Some(seen(1280, 720, Some(87.0), Some(-15.0))),
                _ => Some(seen(640, 480, Some(90.0), Some(-90.0))),
            },
            &[],
        );
        let (s15, c15) = 15f64.to_radians().sin_cos();
        assert_t(
            &t_of(&rig.sensors[0]),
            &[0.0, -s15, c15, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -c15, -s15, 0.0, 0.0, 0.0, 0.0, 1.0],
        );
        let main = rig.sensors[0].camera.as_ref().unwrap();
        assert!((main.fx - 640.0 / 43.5_f64.to_radians().tan()).abs() < 1e-9);
        // Straight down, the image top toward the nose.
        let down = [0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        assert_t(&t_of(&rig.sensors[1]), &down);
        assert!((rig.sensors[1].camera.as_ref().unwrap().fx - 320.0).abs() < 1e-9);
        // No roster geometry: the down camera still looks down (not up), the
        // primary looks level along the nose, on the default field of view.
        let bare = derive_rig(&cfg, |id| Some(seen(if id == "main" { 1280 } else { 640 }, 480, None, None)), &[]);
        assert_t(&t_of(&bare.sensors[1]), &down);
        assert_t(
            &t_of(&bare.sensors[0]),
            &[0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        );
        let f = bare.sensors[0].camera.as_ref().unwrap().fx;
        assert!((f - 640.0 / (DEFAULT_HFOV_DEG / 2.0).to_radians().tan()).abs() < 1e-9);
    }

    #[test]
    fn side_back_and_up_cameras_face_their_side() {
        let axis = |t: [f64; 16]| [t[2], t[6], t[10]];
        let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9);
        assert!(near(axis(t_body_cam(90.0, 0.0)), [0.0, 1.0, 0.0]));
        assert!(near(axis(t_body_cam(-90.0, 0.0)), [0.0, -1.0, 0.0]));
        assert!(near(axis(t_body_cam(180.0, 0.0)), [-1.0, 0.0, 0.0]));
        assert!(near(axis(t_body_cam(0.0, 90.0)), [0.0, 0.0, 1.0]));
        assert_eq!(role_mount(CameraRole::Left), (90.0, 0.0));
        assert_eq!(role_mount(CameraRole::Up), (0.0, 90.0));
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
        let values: BTreeMap<String, serde_json::Value> = [(
            "atlas.cameras".to_string(),
            serde_json::json!([
                {"id": "main"}, {"id": "belly", "role": "down"}, {"id": "aux", "role": "aux"}
            ]),
        )]
        .into_iter()
        .collect();
        let cfg = AtlasRuntimeConfig::from_values("drone 1", &values).unwrap();
        let hd = seen(1280, 720, Some(87.0), None);
        // `aux` has delivered no frame yet: no size, no model, left out.
        let (rig, added) = complete_rig(stored.clone(), &cfg, |id| (id != "aux").then_some(hd));
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
        let (rig, added) = complete_rig(rig, &cfg, |_| Some(hd));
        assert_eq!(added, ["aux"]);
        let (again, none) = complete_rig(rig.clone(), &cfg, |_| Some(hd));
        assert!(none.is_empty());
        assert_eq!(json(&again.sensors), json(&rig.sensors));
    }
}
