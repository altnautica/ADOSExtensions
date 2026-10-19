//! `lidar_camera`: targetless LiDAR-camera extrinsics by NID registration in
//! the pinned container.
//!
//! The session's first camera and first LiDAR go into a ROS 2 bag, the tool
//! preprocesses it (dense cloud + image), the initial guess is seeded from the
//! rig's nominal extrinsics (`results.init_T_lidar_camera` in `calib.json`,
//! the key the registration reads first; the tool's automatic initial guess
//! relies on a matcher whose weights are licensed for non-commercial use
//! only), and `calibrate` refines it. `T_lidar_camera` (`p_lidar = T ·
//! p_camera`, `[x y z qx qy qz qw]`) then places the LiDAR through the
//! camera's known pose: `T_body_lidar = T_body_cam · T_lidar_camera⁻¹`.

use std::fs;
use std::path::Path;

use ados_capture_contract::{CameraModelModel, Rig, RigSensorKind, SessionReader};
use serde_json::{json, Value};

use super::container::{self, DVLC_BAGS, DVLC_DIR, DVLC_PREPROCESSED};
use super::geom::{from_xyz_quat, inv_rigid, mul, to_xyz_quat};
use super::kalibr_yaml::rig_cameras;
use super::ros2_bag::export_ros2_bag;
use super::{fresh_dir, run_tool, tool_version};
use crate::recon::{StageError, ToolRunner};

const TOOL: &str = "dvlc";

/// Rig indexes of the camera and LiDAR the calibration pairs.
fn pair(rig: &Rig) -> Result<(usize, usize), StageError> {
    let cam = rig_cameras(rig)
        .into_iter()
        .find(|&i| rig.sensors[i].camera.is_some())
        .ok_or_else(|| StageError::input("the rig has no camera with a projection model"))?;
    let lidar = rig
        .sensors
        .iter()
        .position(|s| s.kind == RigSensorKind::Lidar)
        .ok_or_else(|| StageError::input("the rig has no LiDAR"))?;
    Ok((cam, lidar))
}

/// `T_lidar_camera` from the rig's current extrinsics.
fn rig_t_lidar_camera(rig: &Rig, cam: usize, lidar: usize) -> [f64; 7] {
    to_xyz_quat(&mul(
        &inv_rigid(&rig.sensors[lidar].t_body_sensor.0),
        &rig.sensors[cam].t_body_sensor.0,
    ))
}

fn t_lidar_camera(calib_json: &Value) -> Result<[f64; 7], StageError> {
    let v: Vec<f64> = calib_json
        .pointer("/results/T_lidar_camera")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_f64).collect())
        .unwrap_or_default();
    <[f64; 7]>::try_from(v.as_slice()).map_err(|_| {
        StageError::tool_failed(TOOL, "calib.json has no 7-value results.T_lidar_camera")
    })
}

/// Fold a finished `calib.json` into `base`: the LiDAR at index `lidar` is
/// placed from the camera at index `cam`.
pub fn rig_from_dvlc(
    base: &Rig,
    cam: usize,
    lidar: usize,
    calib_json: &Value,
) -> Result<Rig, StageError> {
    let t = from_xyz_quat(&t_lidar_camera(calib_json)?);
    let mut rig = base.clone();
    let t_body_lidar = mul(&rig.sensors[cam].t_body_sensor.0, &inv_rigid(&t));
    rig.sensors[lidar].t_body_sensor = t_body_lidar.into();
    Ok(rig)
}

fn read_calib(path: &Path) -> Result<Value, StageError> {
    let text = fs::read_to_string(path).map_err(|e| {
        StageError::tool_failed(TOOL, format!("{} was not written: {e}", path.display()))
    })?;
    Ok(serde_json::from_str(&text)?)
}

pub fn lidar_camera(
    reader: &SessionReader,
    work_dir: &Path,
    runner: &dyn ToolRunner,
    image: &dyn Fn(&str) -> Result<String, StageError>,
    progress: &mut dyn FnMut(f32),
) -> Result<(Rig, Value), StageError> {
    let base = reader.rig();
    let (cam, lidar) = pair(base)?;
    let (cam_id, lidar_id) = (&base.sensors[cam].id, &base.sensors[lidar].id);
    let ddir = fresh_dir(work_dir, DVLC_DIR)?;
    let bag = export_ros2_bag(
        reader,
        cam_id,
        lidar_id,
        &ddir.join(DVLC_BAGS).join("session"),
    )?;
    if bag.images == 0 || bag.clouds == 0 {
        return Err(StageError::input(format!(
            "the session needs frames of {cam_id} and clouds of {lidar_id} (got {} and {})",
            bag.images, bag.clouds
        )));
    }
    progress(0.2);
    let c = base.sensors[cam]
        .camera
        .as_ref()
        .expect("pair() picks a modelled camera");
    let (model, dist) = match c.model {
        CameraModelModel::Pinhole => ("plumb_bob", vec![0.0; 5]),
        CameraModelModel::Opencv => ("plumb_bob", c.dist.clone()),
        CameraModelModel::Kb4 | CameraModelModel::OpencvFisheye => ("fisheye", c.dist.clone()),
    };
    let img = image(TOOL)?;
    run_tool(
        runner,
        &container::dvlc_preprocess(work_dir, &img, model, &[c.fx, c.fy, c.cx, c.cy], &dist),
    )?;
    progress(0.5);
    let calib_path = ddir.join(DVLC_PREPROCESSED).join("calib.json");
    let mut calib = read_calib(&calib_path)?;
    let init = rig_t_lidar_camera(base, cam, lidar);
    match calib.as_object_mut() {
        Some(o) => {
            let results = o.entry("results").or_insert_with(|| json!({}));
            if let Some(r) = results.as_object_mut() {
                r.insert("init_T_lidar_camera".into(), json!(init));
            }
        }
        None => return Err(StageError::tool_failed(TOOL, "calib.json is not an object")),
    }
    fs::write(&calib_path, serde_json::to_vec_pretty(&calib)?)?;
    run_tool(runner, &container::dvlc_calibrate(work_dir, &img))?;
    progress(0.9);
    let calib = read_calib(&calib_path)?;
    let rig = rig_from_dvlc(base, cam, lidar, &calib)?;
    let report = json!({
        "reprojection_rms_px": null,
        "tool": TOOL,
        "tool_version": tool_version(TOOL),
        "camera_id": cam_id,
        "lidar_id": lidar_id,
        "images": bag.images,
        "clouds": bag.clouds,
        "init_T_lidar_camera": init,
        "T_lidar_camera": t_lidar_camera(&calib)?,
    });
    Ok((rig, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::test_util::{assert_close, two_camera_rig};

    #[test]
    fn calib_json_places_the_lidar_through_the_camera() {
        let base = two_camera_rig();
        let (cam, lidar) = pair(&base).unwrap();
        assert_eq!((cam, lidar), (0, 3));
        // The rig's own extrinsics round-trip unchanged.
        let init = rig_t_lidar_camera(&base, cam, lidar);
        let same = rig_from_dvlc(
            &base,
            cam,
            lidar,
            &json!({"results": {"T_lidar_camera": init}}),
        )
        .unwrap();
        assert_close(
            &same.sensors[3].t_body_sensor.0,
            &base.sensors[3].t_body_sensor.0,
            1e-12,
        );

        // A camera 0.2 m along LiDAR +x with the rig's camera rotation.
        let mut t = mul(
            &inv_rigid(&base.sensors[3].t_body_sensor.0),
            &base.sensors[0].t_body_sensor.0,
        );
        t[3] = 0.2;
        t[7] = 0.0;
        t[11] = 0.0;
        let calib = json!({"results": {"T_lidar_camera": to_xyz_quat(&t)}});
        let rig = rig_from_dvlc(&base, cam, lidar, &calib).unwrap();
        let tb = rig.sensors[3].t_body_sensor.0;
        // LiDAR origin = camera origin minus 0.2 m along body x.
        assert_close(&[tb[3], tb[7], tb[11]], &[0.1 - 0.2, 0.05, 0.0], 1e-9);
        assert_close(&tb[..3], &[1.0, 0.0, 0.0], 1e-9);
        assert_eq!(rig.sensors[0], base.sensors[0]);
    }

    #[test]
    fn missing_result_is_a_tool_failure() {
        let base = two_camera_rig();
        let err = rig_from_dvlc(&base, 0, 3, &json!({"results": {}})).unwrap_err();
        assert_eq!(err.code, "E_TOOL_FAILED");
    }
}
