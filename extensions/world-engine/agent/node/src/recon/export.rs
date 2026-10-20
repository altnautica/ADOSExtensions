//! The dataset export of a reconstruction (`out/export.zip`): the COLMAP
//! sparse model (BIN, with `rigs.bin` / `frames.bin` as COLMAP 4 writes them)
//! and a Nerfstudio `transforms.json` carrying the contract extension
//! (`ados_anchor`, per-frame `t_ns`, `camera_id`).

use std::collections::BTreeMap;
use std::fs;
use std::io::Write as _;
use std::path::Path;

use ados_capture_contract::{CameraModelModel, GeoAnchor};

use super::geom::Mat4;
use super::prepare::FrameRecord;
use super::StageError;

/// OpenCV camera-to-world → the OpenGL convention Nerfstudio reads (negate
/// the camera Y and Z axes), as row arrays.
fn opengl_rows(t: &Mat4) -> [[f64; 4]; 4] {
    let mut m = [[0.0; 4]; 4];
    for (r, row) in m.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            let s = if c == 1 || c == 2 { -1.0 } else { 1.0 };
            *v = if c == 3 {
                t[r * 4 + 3]
            } else {
                s * t[r * 4 + c]
            };
        }
    }
    m[3] = [0.0, 0.0, 0.0, 1.0];
    m
}

/// `transforms.json` for every frame COLMAP registered, posed by the
/// reconstructed model (`poses`: name → `T_world_cam`).
pub fn transforms_json(
    frames: &[FrameRecord],
    poses: &BTreeMap<String, Mat4>,
    anchor: Option<&GeoAnchor>,
) -> serde_json::Value {
    let fisheye = frames.iter().any(|f| {
        matches!(
            f.model,
            CameraModelModel::OpencvFisheye | CameraModelModel::Kb4
        )
    });
    let out: Vec<serde_json::Value> = frames
        .iter()
        .filter_map(|f| {
            let t = poses.get(&f.name)?;
            let d = |i: usize| f.dist.get(i).copied().unwrap_or(0.0);
            let mut v = serde_json::json!({
                "file_path": format!("images/{}", f.name),
                "transform_matrix": opengl_rows(t),
                "t_ns": f.t_ns,
                "camera_id": f.camera_id,
                "fl_x": f.fx, "fl_y": f.fy, "cx": f.cx, "cy": f.cy,
                "w": f.width, "h": f.height,
            });
            match f.model {
                CameraModelModel::Pinhole => {}
                CameraModelModel::Opencv => {
                    for (k, i) in [("k1", 0), ("k2", 1), ("p1", 2), ("p2", 3), ("k3", 4)] {
                        v[k] = d(i).into();
                    }
                }
                CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => {
                    for (k, i) in [("k1", 0), ("k2", 1), ("k3", 2), ("k4", 3)] {
                        v[k] = d(i).into();
                    }
                }
            }
            Some(v)
        })
        .collect();
    serde_json::json!({
        "contract": ados_capture_contract::CONTRACT_ID,
        "ados_anchor": anchor,
        "camera_model": if fisheye { "OPENCV_FISHEYE" } else { "OPENCV" },
        "frames": out,
    })
}

/// Write `out/export.zip` from the job's dataset and SfM text model.
pub fn write_export(
    job_dir: &Path,
    anchor: Option<&GeoAnchor>,
    out: &Path,
) -> Result<(), StageError> {
    let dataset = job_dir.join("dataset");
    let frames = super::prepare::read_frames(&dataset)?;
    let poses = super::colmap::read_image_poses(&job_dir.join("sfm").join("txt"))?;
    let transforms = transforms_json(&frames, &poses, anchor);
    let file = fs::File::create(out.join("export.zip"))?;
    let mut zip = zip::ZipWriter::new(file);
    let opts = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated);
    let model = dataset.join("sparse").join("0");
    let mut names: Vec<_> = fs::read_dir(&model)?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_file())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    for name in names {
        zip.start_file(format!("sparse/0/{name}"), opts)
            .map_err(StageError::io)?;
        zip.write_all(&fs::read(model.join(&name))?)?;
    }
    // The registered images, at the paths `transforms.json` and `images.bin`
    // name (PNG is already compressed: stored, not deflated).
    let stored = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Stored)
        .large_file(true);
    for f in frames.iter().filter(|f| poses.contains_key(&f.name)) {
        let src = dataset.join("images").join(&f.name);
        if !src.is_file() {
            return Err(StageError::io(format!(
                "export: {} is missing",
                src.display()
            )));
        }
        zip.start_file(format!("images/{}", f.name), stored)
            .map_err(StageError::io)?;
        std::io::copy(&mut fs::File::open(&src)?, &mut zip)?;
    }
    zip.start_file("transforms.json", opts)
        .map_err(StageError::io)?;
    zip.write_all(&serde_json::to_vec_pretty(&transforms)?)?;
    zip.finish().map_err(StageError::io)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transforms_follow_the_contract_and_flip_to_opengl() {
        let t = crate::recon::geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0]);
        let frame = FrameRecord {
            name: "cam0/5.png".into(),
            camera_id: "cam0".into(),
            t_ns: 5,
            t_world_cam: None,
            width: 64,
            height: 48,
            fx: 50.0,
            fy: 50.0,
            cx: 32.0,
            cy: 24.0,
            model: CameraModelModel::Opencv,
            dist: vec![0.1, 0.0, 0.0, 0.0, 0.0],
            calibrated: false,
            t_body_cam: None,
            rig_instant: None,
        };
        let mut poses = BTreeMap::new();
        poses.insert("cam0/5.png".to_string(), t);
        let v = transforms_json(
            &[
                frame.clone(),
                FrameRecord {
                    name: "x".into(),
                    ..frame
                },
            ],
            &poses,
            None,
        );
        let parsed: ados_capture_contract::TransformsJson =
            serde_json::from_value(v.clone()).unwrap();
        assert_eq!(parsed.frames.len(), 1);
        assert_eq!(v["frames"][0]["t_ns"], 5);
        assert_eq!(v["frames"][0]["transform_matrix"][1][1], -1.0);
        assert_eq!(v["frames"][0]["transform_matrix"][2][3], 3.0);
        assert!(v["ados_anchor"].is_null());
    }
}
