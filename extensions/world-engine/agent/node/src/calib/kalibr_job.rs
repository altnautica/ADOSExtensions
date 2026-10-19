//! `intrinsics` and `camera_imu`: export, bag, calibrate in the Kalibr
//! container, fold the camera chain back into the rig.

use std::fs;
use std::path::Path;

use ados_capture_contract::{Rig, SessionReader};
use serde_json::{json, Value};

use super::container::{self, KALIBR_DIR, KALIBR_FOLDER};
use super::export::{export_kalibr_folder, KalibrExport};
use super::kalibr_yaml::{self as ky, rig_cameras, rig_imu};
use super::{fresh_dir, run_tool, tool_version, AprilGrid};
use crate::recon::{StageError, ToolRunner};

const TOOL: &str = "kalibr";

/// Export the session and write `target.yaml`; refuses a camera without
/// frames.
fn prepare(
    reader: &SessionReader,
    board: &AprilGrid,
    work_dir: &Path,
) -> Result<KalibrExport, StageError> {
    let kdir = fresh_dir(work_dir, KALIBR_DIR)?;
    let export = export_kalibr_folder(reader, &kdir.join(KALIBR_FOLDER))?;
    let rig = reader.rig();
    for (i, n) in export.frames.iter().enumerate() {
        if *n == 0 {
            let id = &rig.sensors[rig_cameras(rig)[i]].id;
            return Err(StageError::input(format!(
                "camera {id} has no frames in the session"
            )));
        }
    }
    fs::write(kdir.join("target.yaml"), ky::target_yaml(board))?;
    Ok(export)
}

fn read_output(work_dir: &Path, suffix: &str) -> Result<String, StageError> {
    let rel = container::kalibr_output(suffix);
    fs::read_to_string(work_dir.join(&rel))
        .map_err(|e| StageError::tool_failed(TOOL, format!("{rel} was not written: {e}")))
}

fn report(rig: &Rig, per_cam: &[(usize, f64)], export: &KalibrExport) -> Value {
    let cams = rig_cameras(rig);
    let cameras: Vec<Value> = cams
        .iter()
        .enumerate()
        .map(|(i, &idx)| {
            json!({
                "id": rig.sensors[idx].id,
                "kalibr_cam": format!("cam{i}"),
                "frames": export.frames.get(i).copied().unwrap_or(0),
                "reprojection_rms_px": per_cam.iter().find(|c| c.0 == i).map(|c| c.1),
                "time_offset_ns": rig.sensors[idx].time_offset_ns,
            })
        })
        .collect();
    json!({
        "reprojection_rms_px": ky::combined_rms(per_cam),
        "tool": TOOL,
        "tool_version": tool_version(TOOL),
        "cameras": cameras,
        "imu_samples": export.imu_samples,
    })
}

/// Camera intrinsics (and camera-to-camera extrinsics) from AprilGrid views.
pub fn intrinsics(
    reader: &SessionReader,
    board: &AprilGrid,
    work_dir: &Path,
    runner: &dyn ToolRunner,
    image: &dyn Fn(&str) -> Result<String, StageError>,
    progress: &mut dyn FnMut(f32),
) -> Result<(Rig, Value), StageError> {
    let base = reader.rig();
    let export = prepare(reader, board, work_dir)?;
    progress(0.2);
    let models: Vec<&str> = rig_cameras(base)
        .into_iter()
        .map(|i| {
            base.sensors[i]
                .camera
                .as_ref()
                .map(|c| ky::kalibr_model(c.model))
                .unwrap_or("pinhole-radtan")
        })
        .collect();
    let img = image(TOOL)?;
    run_tool(runner, &container::kalibr_bagcreater(work_dir, &img))?;
    progress(0.3);
    run_tool(
        runner,
        &container::kalibr_calibrate_cameras(work_dir, &img, &models),
    )?;
    progress(0.9);
    let rig = ky::rig_from_kalibr(base, &read_output(work_dir, "camchain.yaml")?, None)?;
    let per_cam = fs::read_to_string(work_dir.join(container::kalibr_output("results-cam.txt")))
        .map(|t| ky::rms_from_cam_results(&t))
        .unwrap_or_default();
    let report = report(&rig, &per_cam, &export);
    Ok((rig, report))
}

/// Camera-to-IMU extrinsics and time offsets, given calibrated cameras and
/// the IMU noise already on the rig.
pub fn camera_imu(
    reader: &SessionReader,
    board: &AprilGrid,
    work_dir: &Path,
    runner: &dyn ToolRunner,
    image: &dyn Fn(&str) -> Result<String, StageError>,
    progress: &mut dyn FnMut(f32),
) -> Result<(Rig, Value), StageError> {
    let base = reader.rig();
    let imu = rig_imu(base).ok_or_else(|| StageError::input("the rig has no IMU"))?;
    let noise = base.sensors[imu].imu_noise.clone().ok_or_else(|| {
        StageError::input(format!(
            "IMU {} has no noise parameters; run the imu_noise calibration first",
            base.sensors[imu].id
        ))
    })?;
    let camchain = ky::camchain_yaml(base)?;
    let export = prepare(reader, board, work_dir)?;
    let rate = export
        .imu_rate_hz
        .ok_or_else(|| StageError::input("the session holds no IMU samples"))?;
    let kdir = work_dir.join(KALIBR_DIR);
    fs::write(kdir.join("imu.yaml"), ky::imu_yaml(&noise, rate))?;
    fs::write(kdir.join("camchain.yaml"), camchain)?;
    progress(0.2);
    let img = image(TOOL)?;
    run_tool(runner, &container::kalibr_bagcreater(work_dir, &img))?;
    progress(0.3);
    run_tool(
        runner,
        &container::kalibr_calibrate_imu_camera(work_dir, &img),
    )?;
    progress(0.9);
    let chain = read_output(work_dir, "camchain-imucam.yaml")?;
    let imu_out = fs::read_to_string(work_dir.join(container::kalibr_output("imu.yaml"))).ok();
    let rig = ky::rig_from_kalibr(base, &chain, imu_out.as_deref())?;
    let per_cam = fs::read_to_string(work_dir.join(container::kalibr_output("results-imucam.txt")))
        .map(|t| ky::rms_from_imucam_results(&t))
        .unwrap_or_default();
    let report = report(&rig, &per_cam, &export);
    Ok((rig, report))
}
