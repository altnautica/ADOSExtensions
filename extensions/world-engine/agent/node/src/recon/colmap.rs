//! COLMAP text-model I/O: the known-pose model the triangulator starts from,
//! and the reconstructed points and image poses read back.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use ados_capture_contract::CameraModelModel;

use super::geom::{self, Mat4};
use super::prepare::FrameRecord;
use super::StageError;

/// The COLMAP camera model and parameter list for a contract camera.
pub fn camera_params(f: &FrameRecord) -> (&'static str, Vec<f64>) {
    let base = [f.fx, f.fy, f.cx, f.cy];
    let d = |i: usize| f.dist.get(i).copied().unwrap_or(0.0);
    match f.model {
        CameraModelModel::Pinhole => ("PINHOLE", base.to_vec()),
        CameraModelModel::Opencv if d(4) == 0.0 => {
            ("OPENCV", [&base[..], &[d(0), d(1), d(2), d(3)]].concat())
        }
        CameraModelModel::Opencv => (
            "FULL_OPENCV",
            [&base[..], &[d(0), d(1), d(2), d(3), d(4), 0.0, 0.0, 0.0]].concat(),
        ),
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => (
            "OPENCV_FISHEYE",
            [&base[..], &[d(0), d(1), d(2), d(3)]].concat(),
        ),
    }
}

/// Comma-joined parameters for `--ImageReader.camera_params`.
pub fn params_arg(params: &[f64]) -> String {
    params
        .iter()
        .map(|v| format!("{v}"))
        .collect::<Vec<_>>()
        .join(",")
}

/// Image and camera ids COLMAP assigned in its database, keyed by image name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DbIds {
    pub images: BTreeMap<String, (u32, u32)>,
}

/// Read the image → (image_id, camera_id) map from a COLMAP database.
pub fn read_db_ids(db: &Path) -> Result<DbIds, StageError> {
    let conn = rusqlite::Connection::open(db).map_err(StageError::io)?;
    let mut stmt = conn
        .prepare("SELECT image_id, name, camera_id FROM images")
        .map_err(StageError::io)?;
    let rows = stmt
        .query_map([], |r| {
            Ok((
                r.get::<_, String>(1)?,
                (r.get::<_, i64>(0)? as u32, r.get::<_, i64>(2)? as u32),
            ))
        })
        .map_err(StageError::io)?;
    let mut ids = DbIds::default();
    for row in rows {
        let (name, v) = row.map_err(StageError::io)?;
        ids.images.insert(name.replace('\\', "/"), v);
    }
    Ok(ids)
}

/// Write the known-pose text model (`cameras.txt`, `images.txt`, empty
/// `points3D.txt`) for every posed frame the database knows.
pub fn write_known_model(
    dir: &Path,
    frames: &[FrameRecord],
    ids: &DbIds,
) -> Result<usize, StageError> {
    fs::create_dir_all(dir)?;
    let mut cameras = String::from("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n");
    let mut images = String::from("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n# POINTS2D[]\n");
    let mut written_cams = std::collections::BTreeSet::new();
    let mut n = 0;
    for f in frames {
        let (Some(t_wc), Some(&(image_id, camera_id))) = (f.t_world_cam, ids.images.get(&f.name))
        else {
            continue;
        };
        if written_cams.insert(camera_id) {
            let (model, params) = camera_params(f);
            let p: Vec<String> = params.iter().map(|v| format!("{v}")).collect();
            let _ = writeln!(
                cameras,
                "{camera_id} {model} {} {} {}",
                f.width,
                f.height,
                p.join(" ")
            );
        }
        let t_cw = geom::inverse_rigid(&t_wc);
        let q = geom::quat_of(&t_cw);
        let t = geom::translation(&t_cw);
        let _ = writeln!(
            images,
            "{image_id} {} {} {} {} {} {} {} {camera_id} {}\n",
            q[0], q[1], q[2], q[3], t[0], t[1], t[2], f.name
        );
        n += 1;
    }
    fs::write(dir.join("cameras.txt"), cameras)?;
    fs::write(dir.join("images.txt"), images)?;
    fs::write(dir.join("points3D.txt"), "")?;
    Ok(n)
}

/// One reconstructed point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    pub xyz: [f64; 3],
    pub rgb: [u8; 3],
    pub error: f64,
}

/// Read `points3D.txt`.
pub fn read_points(model_txt: &Path) -> Result<Vec<Point3D>, StageError> {
    let body = fs::read_to_string(model_txt.join("points3D.txt"))?;
    let mut out = Vec::new();
    for line in body.lines() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let v: Vec<&str> = line.split_whitespace().collect();
        if v.len() < 8 {
            continue;
        }
        let f = |i: usize| v[i].parse::<f64>().unwrap_or(f64::NAN);
        let u = |i: usize| v[i].parse::<u8>().unwrap_or(0);
        let p = Point3D {
            xyz: [f(1), f(2), f(3)],
            rgb: [u(4), u(5), u(6)],
            error: f(7),
        };
        if p.xyz.iter().all(|c| c.is_finite()) {
            out.push(p);
        }
    }
    Ok(out)
}

/// Registered image poses from `images.txt`: name → `T_world_cam`.
pub fn read_image_poses(model_txt: &Path) -> Result<BTreeMap<String, Mat4>, StageError> {
    let body = fs::read_to_string(model_txt.join("images.txt"))?;
    let mut out = BTreeMap::new();
    let mut header = true;
    for line in body.lines() {
        if line.starts_with('#') {
            continue;
        }
        if header {
            let v: Vec<&str> = line.split_whitespace().collect();
            if v.len() >= 10 {
                let f = |i: usize| v[i].parse::<f64>().unwrap_or(0.0);
                let t_cw = geom::from_quat_trans([f(1), f(2), f(3), f(4)], [f(5), f(6), f(7)]);
                out.insert(v[9..].join(" "), geom::inverse_rigid(&t_cw));
            }
        }
        header = !header;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(name: &str, x: f64) -> FrameRecord {
        FrameRecord {
            name: name.into(),
            camera_id: "cam0".into(),
            t_ns: 1,
            t_world_cam: Some(geom::from_quat_trans([0.9, 0.1, 0.2, 0.3], [x, 2.0, 3.0])),
            width: 64,
            height: 48,
            fx: 50.0,
            fy: 51.0,
            cx: 32.0,
            cy: 24.0,
            model: CameraModelModel::Opencv,
            dist: vec![0.1, -0.01, 0.001, 0.002, 0.0],
            calibrated: true,
        }
    }

    #[test]
    fn known_model_round_trips_poses() {
        let dir = tempfile::tempdir().unwrap();
        let frames = vec![frame("cam0/1.png", 1.0), frame("cam0/2.png", 4.0)];
        let mut ids = DbIds::default();
        ids.images.insert("cam0/1.png".into(), (7, 1));
        ids.images.insert("cam0/2.png".into(), (8, 1));
        assert_eq!(write_known_model(dir.path(), &frames, &ids).unwrap(), 2);
        let cams = fs::read_to_string(dir.path().join("cameras.txt")).unwrap();
        assert!(cams.contains("1 OPENCV 64 48 50 51 32 24 0.1 -0.01 0.001 0.002"));
        let poses = read_image_poses(dir.path()).unwrap();
        let back = poses["cam0/2.png"];
        let want = frames[1].t_world_cam.unwrap();
        assert!(back
            .iter()
            .zip(want.iter())
            .all(|(a, b)| (a - b).abs() < 1e-9));
    }

    #[test]
    fn points_parse_and_skip_comments() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(
            dir.path().join("points3D.txt"),
            "# header\n1 0.5 1.5 -2 10 20 30 0.4 1 2 3 4\n",
        )
        .unwrap();
        let p = read_points(dir.path()).unwrap();
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].xyz, [0.5, 1.5, -2.0]);
        assert_eq!(p[0].rgb, [10, 20, 30]);
    }

    #[test]
    fn camera_models_map_to_colmap_names() {
        let mut f = frame("a", 0.0);
        assert_eq!(camera_params(&f).0, "OPENCV");
        f.dist[4] = 0.01;
        assert_eq!(camera_params(&f).0, "FULL_OPENCV");
        assert_eq!(camera_params(&f).1.len(), 12);
        f.model = CameraModelModel::Kb4;
        assert_eq!(camera_params(&f).0, "OPENCV_FISHEYE");
        f.model = CameraModelModel::Pinhole;
        assert_eq!(camera_params(&f), ("PINHOLE", vec![50.0, 51.0, 32.0, 24.0]));
    }
}
