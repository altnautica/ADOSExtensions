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
