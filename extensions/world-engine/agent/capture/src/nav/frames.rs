//! Frame conversions for the published navigation payloads.
//!
//! The pose provider's world is FLU (for the planar provider, the body frame
//! at the first scan). `map_frd` is that world turned to FRD with
//! `F = diag(1, -1, -1)`: a world-from-body rotation `R` becomes `F·R·F`, a
//! position `t` becomes `F·t`, and yaw changes sign. The published frame,
//! `map_ned`, rotates `map_frd` about down to north (see `crate::nav`).

use std::f32::consts::TAU;

use world_engine_protocol::atlas::nav::SCAN_BINS;
use world_engine_protocol::atlas::{NavFrame, PolarScan, ScanDescriptor, VioHealth};

use crate::lidar_scan::LidarScan;
use crate::pose::se3;

/// Bins of a recorded polar scan (0.5°, finer than a 0.72° planar LiDAR).
pub const POLAR_BINS: usize = 720;

/// `F·R·F` with `F = diag(1, -1, -1)`: flips the sign of every element that
/// mixes x with y or z.
pub fn flu_to_frd_rot(r: &[f64; 9]) -> [f64; 9] {
    [r[0], -r[1], -r[2], -r[3], r[4], r[5], -r[6], r[7], r[8]]
}

/// `F·t`.
pub fn flu_to_frd_vec(t: [f64; 3]) -> [f64; 3] {
    [t[0], -t[1], -t[2]]
}

/// FRD azimuth of an FLU point, `[0, 2π)`: clockwise from +x toward right.
fn frd_azimuth(x: f32, y_flu: f32) -> f32 {
    (-y_flu).atan2(x).rem_euclid(TAU)
}

/// The scan binned to [`SCAN_BINS`] 1° bins in the body FRD frame, nearest
/// horizontal range per bin. `t_body_sensor` is the LiDAR mount (FLU).
pub fn body_scan(scan: &LidarScan, t_body_sensor: &[f64; 16]) -> ScanDescriptor {
    let (r, t) = se3::from_mat4(t_body_sensor);
    let mut ranges = vec![0.0f32; SCAN_BINS];
    let inc = TAU / SCAN_BINS as f32;
    for p in &scan.points {
        let b = se3::mat_vec(&r, [p[0] as f64, p[1] as f64, p[2] as f64]);
        let (x, y) = ((b[0] + t[0]) as f32, (b[1] + t[1]) as f32);
        let range = x.hypot(y);
        if !range.is_finite() || range <= 0.0 {
            continue;
        }
        let bin = ((frd_azimuth(x, y) / inc) as usize).min(SCAN_BINS - 1);
        if ranges[bin] == 0.0 || range < ranges[bin] {
            ranges[bin] = range;
        }
    }
    ScanDescriptor {
        t_ns: scan.t_ns,
        frame: NavFrame::BodyFrd,
        angle_start_rad: 0.0,
        angle_increment_rad: inc,
        ranges_m: ranges,
    }
}

/// The scan in its sensor FRD frame as [`POLAR_BINS`] polar bins (nearest
/// return, its reflectivity as quality), stamped with the sensor's pose in
/// `map_ned` (`r_world_sensor`, `t_world_sensor` already in it).
pub fn polar_scan(
    scan: &LidarScan,
    t_ns: u64,
    r_world_sensor: &[f64; 9],
    t_world_sensor: [f64; 3],
    health: VioHealth,
) -> PolarScan {
    let mut range_mm = vec![0u16; POLAR_BINS];
    let mut quality = vec![0u8; POLAR_BINS];
    let inc = TAU / POLAR_BINS as f32;
    for (i, p) in scan.points.iter().enumerate() {
        let range = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        if !range.is_finite() || range <= 0.0 {
            continue;
        }
        let mm = (range * 1000.0).round().clamp(1.0, u16::MAX as f32) as u16;
        let bin = ((frd_azimuth(p[0], p[1]) / inc) as usize).min(POLAR_BINS - 1);
        if range_mm[bin] == 0 || mm < range_mm[bin] {
            range_mm[bin] = mm;
            quality[bin] = scan.intensity.get(i).copied().unwrap_or(0);
        }
    }
    let q = se3::quat_from_matrix(r_world_sensor);
    PolarScan {
        t_ns,
        angle_start_rad: 0.0,
        angle_inc_rad: inc,
        t: t_world_sensor.map(|v| v as f32),
        q: q.map(|v| v as f32),
        health,
        range_mm,
        quality,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_flu(a: f64) -> [f64; 9] {
        let (s, c) = a.sin_cos();
        [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
    }

    #[test]
    fn a_left_turn_in_flu_is_a_negative_yaw_in_frd() {
        let r = flu_to_frd_rot(&yaw_flu(0.3));
        // FRD yaw = atan2(R[3], R[0]).
        assert!((r[3].atan2(r[0]) + 0.3).abs() < 1e-12);
        assert_eq!(flu_to_frd_vec([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
    }

    #[test]
    fn a_point_on_the_right_lands_in_the_90_degree_bin() {
        let scan = LidarScan {
            sensor_id: "lidar0".into(),
            t_ns: 5,
            // FLU: forward 2 m; right (y = -3) 3 m.
            points: vec![[2.0, 0.0, 0.0], [0.0, -3.0, 0.0]],
            t_offset_ns: vec![0, 0],
            intensity: vec![10, 20],
        };
        let d = body_scan(&scan, &se3::to_mat4(&se3::IDENTITY, [0.0; 3]));
        assert_eq!(d.ranges_m.len(), 360);
        assert!((d.ranges_m[0] - 2.0).abs() < 1e-6);
        assert!((d.ranges_m[90] - 3.0).abs() < 1e-6);
        assert_eq!(d.ranges_m[270], 0.0);

        let p = polar_scan(&scan, 5, &se3::IDENTITY, [0.0; 3], VioHealth::Good);
        assert_eq!(p.range_mm[0], 2000);
        assert_eq!(p.range_mm[180], 3000);
        assert_eq!(p.quality[180], 20);
        let right = p.body_point(180).unwrap();
        assert!((right[1] - 3.0).abs() < 1e-3, "FRD +y is right: {right:?}");
    }
}
