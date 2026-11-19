//! Sensor calibration jobs (`calibrate{session_id, target}`) over an uploaded
//! capture session.
//!
//! - `imu_noise`: native Allan deviation over the session's IMU samples.
//! - `intrinsics` / `camera_imu`: the session exported to a Kalibr folder and
//!   calibrated in the pinned Kalibr container.
//! - `lidar_camera`: for a 3D LiDAR, the session exported to a ROS 2 bag and
//!   calibrated in the pinned LiDAR-camera container (NID registration); for
//!   a planar LiDAR, natively from the calibration board the scan line
//!   crosses ([`lrf_camera`]).
//!
//! Each returns the session rig with the calibrated fields replaced and
//! `calib_version` = `<input>+<target>-<UTC yyyymmddThhmmssZ>`, plus a report.

pub mod allan;
pub mod board;
pub mod container;
mod dvlc;
pub mod export;
pub mod geom;
pub mod imu_noise;
mod kalibr_job;
pub mod kalibr_yaml;
pub mod lens;
pub mod lrf_camera;
pub mod lrf_solve;
pub mod ros2_bag;
#[cfg(any(test, feature = "mock"))]
mod stand_in;

#[cfg(test)]
pub(crate) mod test_util;
#[cfg(test)]
mod tests;

use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use ados_capture_contract::{Rig, SessionMessage, SessionReader};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::recon::{StageError, ToolInvocation, ToolRunner};

pub use dvlc::rig_from_dvlc;
pub use kalibr_yaml::rig_from_kalibr;
#[cfg(any(test, feature = "mock"))]
pub use stand_in::stand_in;

/// What a calibration job calibrates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibTarget {
    ImuNoise,
    Intrinsics,
    CameraImu,
    LidarCamera,
}

impl CalibTarget {
    pub fn as_str(self) -> &'static str {
        match self {
            CalibTarget::ImuNoise => "imu_noise",
            CalibTarget::Intrinsics => "intrinsics",
            CalibTarget::CameraImu => "camera_imu",
            CalibTarget::LidarCamera => "lidar_camera",
        }
    }
}

/// AprilGrid calibration board.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AprilGrid {
    pub tag_cols: u32,
    pub tag_rows: u32,
    /// Tag edge length, metres.
    pub tag_size_m: f64,
    /// Gap between tags as a fraction of the tag size.
    pub tag_spacing: f64,
}

impl Default for AprilGrid {
    fn default() -> Self {
        Self {
            tag_cols: 6,
            tag_rows: 6,
            tag_size_m: 0.088,
            tag_spacing: 0.3,
        }
    }
}

/// A calibration job request.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibRequest {
    pub session_id: String,
    pub target: CalibTarget,
    #[serde(default)]
    pub board: AprilGrid,
}

/// A finished calibration: the updated rig and the report
/// (`calib_report.json`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibOutcome {
    pub rig: Rig,
    pub report: Value,
}

/// Run one calibration over the session in `session_dir`, using `work_dir`
/// for exports and tool outputs. Container images come from the pinned tool
/// set; a missing pin or container runtime is `E_TOOL_MISSING`.
pub fn run(
    req: &CalibRequest,
    session_dir: &Path,
    work_dir: &Path,
    runner: &dyn ToolRunner,
    progress: &mut dyn FnMut(f32),
) -> Result<CalibOutcome, StageError> {
    run_with(
        req,
        session_dir,
        work_dir,
        runner,
        progress,
        &container::pinned_image,
    )
}

/// [`run`] with the container image lookup supplied.
pub(crate) fn run_with(
    req: &CalibRequest,
    session_dir: &Path,
    work_dir: &Path,
    runner: &dyn ToolRunner,
    progress: &mut dyn FnMut(f32),
    image: &dyn Fn(&str) -> Result<String, StageError>,
) -> Result<CalibOutcome, StageError> {
    let reader = SessionReader::open(session_dir)?;
    if reader.manifest().session_id != req.session_id {
        return Err(StageError::input(format!(
            "session directory holds {}, not {}",
            reader.manifest().session_id,
            req.session_id
        )));
    }
    std::fs::create_dir_all(work_dir)?;
    progress(0.0);
    let (mut rig, mut report) = match req.target {
        CalibTarget::ImuNoise => imu_noise::calibrate(&reader, progress)?,
        CalibTarget::Intrinsics => {
            kalibr_job::intrinsics(&reader, &req.board, work_dir, runner, image, progress)?
        }
        CalibTarget::CameraImu => {
            kalibr_job::camera_imu(&reader, &req.board, work_dir, runner, image, progress)?
        }
        CalibTarget::LidarCamera if lrf_camera::rig_lidar_is_planar(reader.rig()) => {
            lrf_camera::lidar_camera(&reader, &req.board, progress)?
        }
        CalibTarget::LidarCamera => dvlc::lidar_camera(&reader, work_dir, runner, image, progress)?,
    };
    rig.calib_version = format!(
        "{}+{}-{}",
        reader.rig().calib_version,
        req.target.as_str(),
        utc_stamp(SystemTime::now())
    );
    if let Value::Object(m) = &mut report {
        m.insert("target".into(), Value::from(req.target.as_str()));
        m.insert(
            "calib_version".into(),
            Value::from(rig.calib_version.clone()),
        );
    }
    progress(1.0);
    Ok(CalibOutcome { rig, report })
}

/// `yyyymmddThhmmssZ` of `t` in UTC.
pub fn utc_stamp(t: SystemTime) -> String {
    let secs = t
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0) as i64;
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil date from days since 1970-01-01 (proleptic Gregorian).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

/// Visit every session message with a closure that fails with a
/// [`StageError`].
pub(crate) fn visit(
    reader: &SessionReader,
    mut f: impl FnMut(SessionMessage<'_>) -> Result<(), StageError>,
) -> Result<(), StageError> {
    let mut failed = None;
    let r = reader.for_each_message(|m| {
        f(m).map_err(|e| {
            failed = Some(e);
            ados_capture_contract::Error::Invalid("stopped".into())
        })
    });
    match failed {
        Some(e) => Err(e),
        None => r.map_err(Into::into),
    }
}

/// Run one tool, logging its output.
pub(crate) fn run_tool(runner: &dyn ToolRunner, inv: &ToolInvocation) -> Result<(), StageError> {
    tracing::info!(tool = %inv.tool, args = ?inv.args_lossy(), "calibration tool run");
    runner.run(inv, &mut |line| tracing::debug!(target: "calib", "{line}"))
}

/// The pinned version of `tool`, for reports.
pub(crate) fn tool_version(tool: &str) -> Value {
    crate::tools::pins()
        .get(tool)
        .map(|p| Value::from(p.version.clone()))
        .unwrap_or(Value::Null)
}

/// A fresh (emptied) sub-directory of the work directory.
pub(crate) fn fresh_dir(work_dir: &Path, rel: &str) -> Result<std::path::PathBuf, StageError> {
    let d = work_dir.join(rel);
    if d.exists() {
        std::fs::remove_dir_all(&d)?;
    }
    std::fs::create_dir_all(&d)?;
    Ok(d)
}
