//! LiDAR sweeps registered into one world cloud: every point placed by the
//! body pose at its own time, kept once per [`LIDAR_VOXEL_M`] voxel
//! (`dataset/lidar_points.laz`), plus the scan origin of each sweep
//! (`dataset/lidar_origins.json`, see [`super::LIDAR_ORIGINS_FILE`]).
//!
//! `prepare` registers the sweeps along the recorded pose track; a planar
//! LiDAR's own trajectory (see [`super::super::scan2d`]) registers them again.

use std::collections::HashSet;
use std::fs;
use std::path::Path;

use super::super::geom::{self, Mat4};
use super::super::polar;
use super::super::session_data::PoseTrack;
use super::super::StageError;
use super::LIDAR_ORIGINS_FILE;
use crate::products::cloud::{write_laz, PointCloud as Cloud};

/// The LiDAR world cloud is kept at this voxel size.
pub const LIDAR_VOXEL_M: f64 = 0.02;
/// The registered cloud under the dataset directory.
pub const LIDAR_POINTS_FILE: &str = "lidar_points.laz";

/// Sweeps registered so far.
#[derive(Debug, Default)]
pub struct Sweeps {
    voxels: HashSet<[i64; 3]>,
    points: Vec<[f64; 3]>,
    origins: Vec<[f64; 3]>,
}

impl Sweeps {
    /// Register one sweep stamped `t_ns`: `pts` are sensor-frame points with
    /// their time offsets, `mount` is `T_body_sensor`. Points (and the origin)
    /// at times the track does not cover are skipped.
    pub fn add(&mut self, track: &PoseTrack, mount: &Mat4, t_ns: u64, pts: &[([f64; 3], u64)]) {
        if let Some(body) = track.at(t_ns) {
            self.origins
                .push(geom::apply(&geom::mul(&body, mount), [0.0; 3]));
        }
        for (p, dt) in pts {
            let Some(body) = track.at(t_ns + dt) else {
                continue;
            };
            let w = geom::apply(&geom::mul(&body, mount), *p);
            let key = w.map(|v| (v / LIDAR_VOXEL_M).floor() as i64);
            if self.voxels.insert(key) {
                self.points.push(w);
            }
        }
    }

    /// Registered points (one per voxel).
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Register a planar LiDAR's polar stream (`lidar/<sensor>.polar.zst`,
    /// which it records instead of the Cartesian channel) along `track`, each
    /// scan at its record time shifted by the sensor's `time_offset_ns`.
    /// Returns the number of scans read (0 without a stream).
    pub fn add_polar(
        &mut self,
        session_dir: &Path,
        sensor: &str,
        mount: &Mat4,
        time_offset_ns: i64,
        track: &PoseTrack,
    ) -> Result<usize, StageError> {
        let Some(scans) = polar::read(session_dir, sensor)? else {
            return Ok(0);
        };
        for s in &scans {
            let t_ns = (s.t_ns as i64 + time_offset_ns).max(0) as u64;
            let input = polar::scan_input(s, t_ns);
            let pts: Vec<([f64; 3], u64)> =
                input.points.iter().map(|p| (p.map(f64::from), 0)).collect();
            self.add(track, mount, t_ns, &pts);
        }
        Ok(scans.len())
    }

    /// Write the cloud and the origins into `dataset`, replacing earlier
    /// ones; a file with nothing to hold is removed.
    pub fn write(&self, dataset: &Path) -> Result<(), StageError> {
        let laz = dataset.join(LIDAR_POINTS_FILE);
        let origins = dataset.join(LIDAR_ORIGINS_FILE);
        match self.points.is_empty() {
            true => remove_if_present(&laz)?,
            false => write_laz(
                &laz,
                &Cloud {
                    points: self.points.clone(),
                    colors: None,
                },
            )
            .map_err(StageError::io)?,
        }
        match self.origins.is_empty() {
            true => remove_if_present(&origins)?,
            false => fs::write(&origins, serde_json::to_vec(&self.origins)?)?,
        }
        Ok(())
    }
}

fn remove_if_present(p: &Path) -> Result<(), StageError> {
    match fs::remove_file(p) {
        Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e.into()),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use ados_mapping_protocol::atlas::{PolarScan, VioHealth};

    use super::*;

    fn write_polar(dir: &Path, scans: &[PolarScan]) {
        let path = polar::polar_path(dir, "lidar0");
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        let mut raw = Vec::new();
        for s in scans {
            s.encode(&mut raw);
        }
        let mut enc = zstd::Encoder::new(fs::File::create(&path).unwrap(), 3).unwrap();
        enc.write_all(&raw).unwrap();
        enc.finish().unwrap();
    }

    fn scan(t_ns: u64) -> PolarScan {
        PolarScan {
            t_ns,
            angle_start_rad: 0.0,
            angle_inc_rad: std::f32::consts::FRAC_PI_2,
            t: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Good,
            // 1 m ahead, 2 m to the right (FRD), nothing behind or left.
            range_mm: vec![1000, 2000, 0, 0],
            quality: vec![1, 1, 0, 0],
        }
    }

    #[test]
    fn a_polar_stream_is_registered_along_the_recorded_track() {
        let dir = tempfile::tempdir().unwrap();
        let mut sweeps = Sweeps::default();
        let level = [1.0, 0.0, 0.0, 0.0];
        let track = PoseTrack {
            source: "lidar".into(),
            samples: vec![
                (0, geom::from_quat_trans(level, [10.0, 0.0, 1.0])),
                (1_000, geom::from_quat_trans(level, [20.0, 0.0, 1.0])),
            ],
        };
        let mount = geom::from_quat_trans(level, [0.0, 0.0, 0.5]);
        let none = sweeps.add_polar(dir.path(), "lidar0", &mount, 0, &track);
        assert_eq!(none.unwrap(), 0);
        write_polar(dir.path(), &[scan(100)]);
        // Recorded at 100 ns, sensed 400 ns later: the body is at x = 15.
        let n = sweeps.add_polar(dir.path(), "lidar0", &mount, 400, &track);
        assert_eq!(n.unwrap(), 1);
        assert_eq!(sweeps.len(), 2);
        let mut pts = sweeps.points.clone();
        pts.sort_by(|a, b| a[1].total_cmp(&b[1]));
        // FLU world: 2 m to the right is y = -2; height is body 1 m + mount 0.5 m.
        assert!((pts[0][0] - 15.0).abs() < 1e-5 && (pts[0][1] + 2.0).abs() < 1e-5);
        assert!((pts[1][0] - 16.0).abs() < 1e-5 && pts[1][1].abs() < 1e-5);
        assert!(pts.iter().all(|p| (p[2] - 1.5).abs() < 1e-5));
        assert_eq!(sweeps.origins.len(), 1);
    }
}
