//! A capture session's planar scans in their compact polar form:
//! `lidar/<sensor>.polar.zst`, a zstd stream of
//! [`PolarScan`](world_engine_protocol::atlas::PolarScan) records, each with
//! the sensor's pose in `map_ned` as the drone's live SLAM and the flight
//! controller's height placed it. A planar LiDAR records here instead of the
//! Cartesian `/lidar/<id>/points` channel.

use std::fs::File;
use std::path::{Path, PathBuf};

use ados_scan2d::ScanInput;
use world_engine_protocol::atlas::PolarScan;

use super::StageError;

/// The polar stream of `sensor` under `session_dir`.
pub fn polar_path(session_dir: &Path, sensor: &str) -> PathBuf {
    session_dir.join("lidar").join(format!("{sensor}.polar.zst"))
}

/// Every record of `sensor`'s polar stream, `None` when the session has none.
pub fn read(session_dir: &Path, sensor: &str) -> Result<Option<Vec<PolarScan>>, StageError> {
    let path = polar_path(session_dir, sensor);
    if !path.is_file() {
        return Ok(None);
    }
    let raw = zstd::decode_all(File::open(&path)?)?;
    PolarScan::decode_all(&raw)
        .map(Some)
        .map_err(|e| StageError::input(format!("{}: {e}", path.display())))
}

/// The record as the scan matcher's input: every return in the sensor's FLU
/// frame (FRD `y` flips), all at the record's time (the polar form keeps no
/// per-point offset).
pub fn scan_input(scan: &PolarScan, t_ns: u64) -> ScanInput {
    let mut points = Vec::with_capacity(scan.range_mm.len());
    for k in 0..scan.range_mm.len() {
        if let Some([x, y_frd]) = scan.body_point(k) {
            points.push([x, -y_frd, 0.0]);
        }
    }
    ScanInput {
        t_ns,
        t_offset_ns: vec![0; points.len()],
        points,
    }
}

/// Every return in a z-up world (`map_ned` turned z-up: `x`, `-y`,
/// `-z`), metres.
pub fn world_points_flu(scan: &PolarScan) -> impl Iterator<Item = [f64; 3]> + '_ {
    scan.world_points()
        .map(|p| [p[0] as f64, -(p[1] as f64), -(p[2] as f64)])
}

/// The sensor origin in the same z-up world.
pub fn origin_flu(scan: &PolarScan) -> [f64; 3] {
    [scan.t[0] as f64, -(scan.t[1] as f64), -(scan.t[2] as f64)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use world_engine_protocol::atlas::VioHealth;

    fn scan() -> PolarScan {
        PolarScan {
            t_ns: 9,
            angle_start_rad: 0.0,
            angle_inc_rad: std::f32::consts::FRAC_PI_2,
            t: [1.0, 2.0, -3.0],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Good,
            range_mm: vec![1000, 2000, 0, 0],
            quality: vec![1, 1, 0, 0],
        }
    }

    #[test]
    fn a_session_stream_reads_back_and_turns_to_flu() {
        let dir = tempfile::tempdir().unwrap();
        assert!(read(dir.path(), "lidar0").unwrap().is_none());
        let path = polar_path(dir.path(), "lidar0");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let mut raw = Vec::new();
        scan().encode(&mut raw);
        let mut enc = zstd::Encoder::new(File::create(&path).unwrap(), 3).unwrap();
        enc.write_all(&raw).unwrap();
        enc.finish().unwrap();
        let back = read(dir.path(), "lidar0").unwrap().unwrap();
        assert_eq!(back, vec![scan()]);

        // Bin 1 is 2 m to the right: FLU y = -2.
        let input = scan_input(&back[0], 9);
        assert_eq!(input.points.len(), 2);
        assert!((input.points[1][1] + 2.0).abs() < 1e-5);
        let w: Vec<_> = world_points_flu(&back[0]).collect();
        assert!((w[0][0] - 2.0).abs() < 1e-5 && (w[0][2] - 3.0).abs() < 1e-5);
        assert_eq!(origin_flu(&back[0]), [1.0, -2.0, 3.0]);
    }
}
