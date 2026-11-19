//! Edits to a COLMAP database between matching and mapping: verified pairs
//! whose relative pose contradicts the pose prior are invalidated, and the
//! prior camera positions are written for `pose_prior_mapper`.
//!
//! Layout as COLMAP 4.2.1 writes it: `pair_id = 2147483647 · min(id) +
//! max(id)`; `two_view_geometries.qvec` (w, x, y, z) and `tvec` are
//! `cam2_from_cam1` of the ordered pair as little-endian `f64`; a
//! `pose_priors` row ties a prior to an image through `corr_data_id` (the
//! image id) and `corr_sensor_id` (its camera, sensor type 0).

use std::collections::BTreeMap;
use std::path::Path;

use rusqlite::{params, Connection};

use super::read_db_ids;
use crate::recon::geom::{self, Mat4};
use crate::recon::prepare::FrameRecord;
use crate::recon::StageError;

/// COLMAP's image-pair id stride (`kMaxNumImages`).
const MAX_NUM_IMAGES: i64 = 2_147_483_647;
/// `SensorType::CAMERA`.
const SENSOR_CAMERA: i64 = 0;
/// `PosePrior::CoordinateSystem::CARTESIAN`.
const CARTESIAN: i64 = 1;
/// The `pose_priors` table as COLMAP creates it.
const POSE_PRIORS_DDL: &str = "CREATE TABLE IF NOT EXISTS pose_priors \
    (pose_prior_id INTEGER PRIMARY KEY NOT NULL, corr_data_id INTEGER NOT NULL, \
    corr_sensor_id INTEGER NOT NULL, corr_sensor_type INTEGER NOT NULL, position BLOB, \
    position_covariance BLOB, gravity BLOB, coordinate_system INTEGER NOT NULL)";

fn db_err(e: rusqlite::Error) -> StageError {
    StageError::io(e)
}

/// COLMAP's id of the pair of images `a` and `b`.
pub fn pair_id(a: u32, b: u32) -> i64 {
    let (lo, hi) = if a > b { (b, a) } else { (a, b) };
    MAX_NUM_IMAGES * i64::from(lo) + i64::from(hi)
}

fn image_pair(pair_id: i64) -> (u32, u32) {
    (
        (pair_id / MAX_NUM_IMAGES) as u32,
        (pair_id % MAX_NUM_IMAGES) as u32,
    )
}

fn f64s(blob: &[u8]) -> Vec<f64> {
    blob.as_chunks::<8>()
        .0
        .iter()
        .map(|c| f64::from_le_bytes(*c))
        .collect()
}

fn blob(v: &[f64]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

fn angle_deg(a: [f64; 3], b: [f64; 3]) -> f64 {
    let n = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (n(a) * n(b));
    dot.clamp(-1.0, 1.0).acos().to_degrees()
}

/// Whether a verified relative pose (`q` w, x, y, z and `t` of
/// `cam2_from_cam1`) agrees with the priors of both cameras: rotation within
/// `max_rot_deg` and, over a prior baseline of at least `min_baseline_m`,
/// translation direction within `max_dir_deg` (a pure-rotation estimate
/// carries no direction and is judged on rotation alone).
fn agrees(
    w1: &Mat4,
    w2: &Mat4,
    q: [f64; 4],
    t: [f64; 3],
    max_rot_deg: f64,
    max_dir_deg: f64,
    min_baseline_m: f64,
) -> bool {
    let prior = geom::mul(&geom::inverse_rigid(w2), w1);
    let est = geom::from_quat_trans(q, t);
    if geom::rotation_angle(&prior, &est).to_degrees() > max_rot_deg {
        return false;
    }
    let tp = geom::translation(&prior);
    let baseline = geom::dist(tp, [0.0; 3]);
    let t_norm = geom::dist(t, [0.0; 3]);
    baseline < min_baseline_m || t_norm < 1e-9 || angle_deg(tp, t) <= max_dir_deg
}

/// Invalidate every verified pair of posed images whose relative pose
/// disagrees with the priors (see [`agrees`]): its configuration becomes
/// `UNDEFINED` (0) and its inlier matches are emptied, so the mapper never
/// uses it. Pairs without a recorded relative pose, or with an unposed
/// image, are kept. Returns how many were invalidated.
pub fn invalidate_inconsistent_pairs(
    db: &Path,
    frames: &[FrameRecord],
    max_rot_deg: f64,
    max_dir_deg: f64,
    min_baseline_m: f64,
) -> Result<usize, StageError> {
    let ids = read_db_ids(db)?;
    let poses: BTreeMap<u32, Mat4> = frames
        .iter()
        .filter_map(|f| Some((ids.images.get(&f.name)?.0, f.t_world_cam?)))
        .collect();
    let mut conn = Connection::open(db).map_err(db_err)?;
    let rows: Vec<(i64, Vec<u8>, Vec<u8>)> = {
        let mut stmt = conn
            .prepare(
                "SELECT pair_id, qvec, tvec FROM two_view_geometries \
                 WHERE rows > 0 AND qvec IS NOT NULL AND tvec IS NOT NULL",
            )
            .map_err(db_err)?;
        let it = stmt
            .query_map([], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)))
            .map_err(db_err)?;
        it.collect::<Result<_, _>>().map_err(db_err)?
    };
    let mut bad = Vec::new();
    for (pid, q, t) in rows {
        let (i1, i2) = image_pair(pid);
        let (Some(w1), Some(w2)) = (poses.get(&i1), poses.get(&i2)) else {
            continue;
        };
        let (q, t) = (f64s(&q), f64s(&t));
        let (Ok(q), Ok(t)) = (<[f64; 4]>::try_from(q), <[f64; 3]>::try_from(t)) else {
            continue;
        };
        if !agrees(w1, w2, q, t, max_rot_deg, max_dir_deg, min_baseline_m) {
            bad.push(pid);
        }
    }
    let tx = conn.transaction().map_err(db_err)?;
    for pid in &bad {
        tx.execute(
            "UPDATE two_view_geometries SET config = 0, rows = 0, data = x'' WHERE pair_id = ?1",
            params![pid],
        )
        .map_err(db_err)?;
    }
    tx.commit().map_err(db_err)?;
    Ok(bad.len())
}

/// Write a `pose_priors` row for every posed frame the database knows: the
/// prior camera centre in the session's Cartesian world frame with the
/// diagonal covariance of `sigma(frame)` (metres, x y z). Existing priors of
/// those images are replaced. Returns the rows written.
pub fn write_pose_priors(
    db: &Path,
    frames: &[FrameRecord],
    sigma: &dyn Fn(&FrameRecord) -> [f64; 3],
) -> Result<usize, StageError> {
    let ids = read_db_ids(db)?;
    let mut conn = Connection::open(db).map_err(db_err)?;
    conn.execute_batch(POSE_PRIORS_DDL).map_err(db_err)?;
    let tx = conn.transaction().map_err(db_err)?;
    let mut n = 0;
    for f in frames {
        let (Some(t_wc), Some(&(image_id, camera_id))) = (f.t_world_cam, ids.images.get(&f.name))
        else {
            continue;
        };
        let s = sigma(f);
        let cov = [
            s[0] * s[0],
            0.0,
            0.0,
            0.0,
            s[1] * s[1],
            0.0,
            0.0,
            0.0,
            s[2] * s[2],
        ];
        tx.execute(
            "DELETE FROM pose_priors WHERE corr_data_id = ?1 AND corr_sensor_type = ?2",
            params![image_id, SENSOR_CAMERA],
        )
        .map_err(db_err)?;
        tx.execute(
            "INSERT INTO pose_priors (pose_prior_id, corr_data_id, corr_sensor_id, \
             corr_sensor_type, position, position_covariance, gravity, coordinate_system) \
             VALUES (?1, ?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                image_id,
                camera_id,
                SENSOR_CAMERA,
                blob(&geom::translation(&t_wc)),
                blob(&cov),
                blob(&[f64::NAN; 3]),
                CARTESIAN
            ],
        )
        .map_err(db_err)?;
        n += 1;
    }
    tx.commit().map_err(db_err)?;
    Ok(n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_capture_contract::CameraModelModel;

    /// A camera at `x` metres along the world x axis, looking +y (OpenCV).
    fn frame(name: &str, x: f64) -> FrameRecord {
        let t = [
            1.0, 0.0, 0.0, x, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0,
        ];
        FrameRecord {
            name: name.into(),
            camera_id: "cam0".into(),
            t_ns: 0,
            t_world_cam: Some(t),
            width: 64,
            height: 48,
            fx: 40.0,
            fy: 40.0,
            cx: 32.0,
            cy: 24.0,
            model: CameraModelModel::Pinhole,
            dist: vec![],
            calibrated: true,
            t_body_cam: None,
            rig_instant: None,
            prior_sigma_m: None,
        }
    }

    /// A database with `names` as images 1.. on camera 1 and COLMAP's
    /// `two_view_geometries` table.
    fn database(dir: &Path, names: &[&str]) -> std::path::PathBuf {
        let db = dir.join("database.db");
        let conn = Connection::open(&db).unwrap();
        conn.execute_batch(
            "CREATE TABLE images (image_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, \
             name TEXT NOT NULL UNIQUE, camera_id INTEGER NOT NULL);
             CREATE TABLE two_view_geometries (pair_id INTEGER PRIMARY KEY NOT NULL, \
             rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB, config INTEGER NOT NULL, \
             F BLOB, E BLOB, H BLOB, qvec BLOB, tvec BLOB, camera1 BLOB, camera2 BLOB);",
        )
        .unwrap();
        for n in names {
            conn.execute(
                "INSERT INTO images (name, camera_id) VALUES (?1, 1)",
                params![n],
            )
            .unwrap();
        }
        db
    }

    /// Store a verified pair with `cam2_from_cam1` = `rel` (unit translation,
    /// as an essential-matrix decomposition reports it) and 40 inliers.
    fn put_pair(db: &Path, a: u32, b: u32, rel: &Mat4) {
        let t = geom::translation(rel);
        let n = geom::dist(t, [0.0; 3]).max(1e-12);
        let conn = Connection::open(db).unwrap();
        conn.execute(
            "INSERT INTO two_view_geometries (pair_id, rows, cols, data, config, qvec, tvec) \
             VALUES (?1, 40, 2, ?2, 2, ?3, ?4)",
            params![
                pair_id(a, b),
                vec![0u8; 40 * 2 * 4],
                blob(&geom::quat_of(rel)),
                blob(&[t[0] / n, t[1] / n, t[2] / n]),
            ],
        )
        .unwrap();
    }

    fn row(db: &Path, a: u32, b: u32) -> (i64, i64) {
        Connection::open(db)
            .unwrap()
            .query_row(
                "SELECT config, rows FROM two_view_geometries WHERE pair_id = ?1",
                params![pair_id(a, b)],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .unwrap()
    }

    #[test]
    fn inconsistent_pairs_are_invalidated() {
        let dir = tempfile::tempdir().unwrap();
        // Bays 4 m apart: image 3 is one bay further on than image 1.
        let frames = [
            frame("cam0/1.png", 0.0),
            frame("cam0/2.png", 1.0),
            frame("cam0/3.png", 4.0),
        ];
        let db = database(dir.path(), &["cam0/1.png", "cam0/2.png", "cam0/3.png"]);
        let rel = |a: &FrameRecord, b: &FrameRecord| {
            geom::mul(
                &geom::inverse_rigid(&b.t_world_cam.unwrap()),
                &a.t_world_cam.unwrap(),
            )
        };
        // 1-2: what the priors say. 1-3: matched on the look-alike bay, the
        // verified pose puts image 3 behind image 1 instead of 4 m ahead.
        put_pair(&db, 1, 2, &rel(&frames[0], &frames[1]));
        put_pair(&db, 1, 3, &rel(&frames[2], &frames[0]));
        let n = invalidate_inconsistent_pairs(&db, &frames, 10.0, 20.0, 0.3).unwrap();
        assert_eq!(n, 1);
        assert_eq!(row(&db, 1, 2), (2, 40));
        assert_eq!(row(&db, 1, 3), (0, 0));
        // A rotation off by more than 10 degrees is refused too, even over a
        // baseline too short to judge direction.
        let close = [frame("cam0/1.png", 0.0), frame("cam0/2.png", 0.1)];
        let dir2 = tempfile::tempdir().unwrap();
        let db2 = database(dir2.path(), &["cam0/1.png", "cam0/2.png"]);
        let yawed = geom::mul(
            &geom::from_quat_trans([0.98, 0.0, 0.199, 0.0], [0.0; 3]),
            &rel(&close[0], &close[1]),
        );
        put_pair(&db2, 1, 2, &yawed);
        assert_eq!(
            invalidate_inconsistent_pairs(&db2, &close, 10.0, 20.0, 0.3).unwrap(),
            1
        );
    }

    #[test]
    fn pose_priors_rows_are_written_with_covariance() {
        let dir = tempfile::tempdir().unwrap();
        let db = database(dir.path(), &["cam0/1.png", "cam0/2.png"]);
        let mut frames = vec![
            frame("cam0/1.png", 2.0),
            frame("cam0/2.png", 3.0),
            frame("cam0/9.png", 9.0),
        ];
        frames[1].prior_sigma_m = Some([0.1, 0.2, 0.3]);
        let sigma = |f: &FrameRecord| f.prior_sigma_m.unwrap_or([0.03, 0.03, 0.25]);
        // Not in the database: no row. Written twice: still one row each.
        assert_eq!(write_pose_priors(&db, &frames, &sigma).unwrap(), 2);
        assert_eq!(write_pose_priors(&db, &frames, &sigma).unwrap(), 2);
        let conn = Connection::open(&db).unwrap();
        let mut stmt = conn
            .prepare(
                "SELECT corr_data_id, corr_sensor_id, corr_sensor_type, position, \
                 position_covariance, gravity, coordinate_system FROM pose_priors \
                 ORDER BY corr_data_id",
            )
            .unwrap();
        type Row = (i64, i64, i64, Vec<u8>, Vec<u8>, Vec<u8>, i64);
        let rows: Vec<Row> = stmt
            .query_map([], |r| {
                Ok((
                    r.get(0)?,
                    r.get(1)?,
                    r.get(2)?,
                    r.get(3)?,
                    r.get(4)?,
                    r.get(5)?,
                    r.get(6)?,
                ))
            })
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(rows.len(), 2);
        assert_eq!((rows[0].0, rows[0].1, rows[0].2, rows[0].6), (1, 1, 0, 1));
        assert_eq!(f64s(&rows[0].3), vec![2.0, 0.0, 1.5]);
        let cov = f64s(&rows[0].4);
        assert_eq!(cov.len(), 9);
        assert!((cov[0] - 0.0009).abs() < 1e-12 && (cov[4] - 0.0009).abs() < 1e-12);
        assert!((cov[8] - 0.0625).abs() < 1e-12);
        assert!(cov[1] == 0.0 && cov[3] == 0.0);
        let cov = f64s(&rows[1].4);
        assert!((cov[0] - 0.01).abs() < 1e-12 && (cov[8] - 0.09).abs() < 1e-12);
        assert!(f64s(&rows[1].5).iter().all(|g| g.is_nan()));
    }
}
