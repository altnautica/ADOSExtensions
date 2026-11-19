//! MCAP channel topics of the `ados.capture/1` profile.
//!
//! `<id>` is a rig sensor id (`rig.json` `sensors[].id`); `<source>` names a
//! pose producer (`truth`, `fc_ekf`, `vio`, `lidar`).

/// `/cam/<id>/image`: `foxglove.RawImage` or `foxglove.CompressedImage`.
pub fn cam_image(id: &str) -> String {
    format!("/cam/{id}/image")
}

/// `/cam/<id>/trigger`: `ados.capture.v1.TriggerRecord`.
pub fn cam_trigger(id: &str) -> String {
    format!("/cam/{id}/trigger")
}

/// `/imu/<id>`: `ados.capture.v1.ImuBatch`, body FLU.
pub fn imu(id: &str) -> String {
    format!("/imu/{id}")
}

/// `/gnss/<id>/raw`: `ados.capture.v1.GnssRaw`.
pub fn gnss_raw(id: &str) -> String {
    format!("/gnss/{id}/raw")
}

/// `/gnss/<id>/fix`: `foxglove.LocationFix`.
pub fn gnss_fix(id: &str) -> String {
    format!("/gnss/{id}/fix")
}

/// `/lidar/<id>/points`: `foxglove.PointCloud` with a per-point
/// `t_offset_ns` (UINT32) field.
pub fn lidar_points(id: &str) -> String {
    format!("/lidar/{id}/points")
}

/// `/pose/<source>`: `foxglove.PoseInFrame`, frame `world` (ENU).
pub fn pose(source: &str) -> String {
    format!("/pose/{source}")
}

/// `/pose/<source>/cov`: `ados.capture.v1.PoseCov` for the pose with the same
/// time.
pub fn pose_cov(source: &str) -> String {
    format!("/pose/{source}/cov")
}

/// `/clock`: `ados.capture.v1.ClockSample`.
pub const CLOCK: &str = "/clock";

/// `/health`: schemaless JSON health record.
pub const HEALTH: &str = "/health";

/// Attachment name of the rig in every segment.
pub const RIG_ATTACHMENT: &str = "rig.json";

/// Attachment name of the manifest snapshot taken when a segment opens.
pub const MANIFEST_ATTACHMENT: &str = "manifest.json";
