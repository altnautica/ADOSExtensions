//! The stored rig: where the agent keeps the operator's rig document
//! (`PUT /api/v1/sensors/rig`, validated against the contract schema), and
//! the questions both capture and navigation ask of it.

use std::path::{Path, PathBuf};

use ados_capture_contract::{LidarModel, Rig, RigSensor, RigSensorKind};

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
