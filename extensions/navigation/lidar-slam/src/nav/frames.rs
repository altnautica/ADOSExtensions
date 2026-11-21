//! Frame conversions for the published navigation payloads.
//!
//! The pose provider's world is FLU (for the planar provider, the body frame
//! at the first scan). `map_frd` is that world turned to FRD with
//! `F = diag(1, -1, -1)`: a world-from-body rotation `R` becomes `F·R·F`, a
//! position `t` becomes `F·t`, and yaw changes sign. The published frame,
//! `map_ned`, rotates `map_frd` about down to north (see `crate::nav`).

use std::f32::consts::TAU;

use ados_mapping_protocol::capture::nav::SCAN_BINS;
use ados_mapping_protocol::capture::{NavFrame, PolarScan, ScanDescriptor, VioHealth};

use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::se3;

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

/// How far over the sensor, levelled, a 3D LiDAR return still counts as a
/// wall at flight height, metres: [`ados_livemap::BAND_TOP_M`] (the airframe's top plus
/// 0.3 m) plus the climb a commanded ascent covers before the pilot's next
/// look. Higher returns met something over the drone (a canopy's underside).
pub const BAND_3D_TOP_M: f32 = 1.0;

/// A return is levelled only with attitudes at most this far either side of
/// its capture, ns: a vehicle pitching at 60°/s turns 1.8° in it.
pub const ATTITUDE_BRACKET_NS: u64 = 30_000_000;

/// The world's up in the body FLU frame at host `t_ns`, interpolated
/// between the two samples of `ups` (oldest first) around it; `None` unless
/// both lie within [`ATTITUDE_BRACKET_NS`] of it.
fn up_at(ups: &[(u64, [f64; 3])], t_ns: u64) -> Option<[f64; 3]> {
    let i = ups.partition_point(|&(t, _)| t <= t_ns);
    let (ta, a) = *ups.get(i.checked_sub(1)?)?;
    if ta == t_ns {
        return Some(a);
    }
    let (tb, b) = *ups.get(i)?;
    if t_ns - ta > ATTITUDE_BRACKET_NS || tb - t_ns > ATTITUDE_BRACKET_NS {
        return None;
    }
    let w = (t_ns - ta) as f64 / (tb - ta) as f64;
    let u = [0, 1, 2].map(|k| a[k] + w * (b[k] - a[k]));
    let n = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
    (n > 1e-9).then(|| u.map(|v| v / n))
}

/// The scan binned to [`SCAN_BINS`] 1° bins in the body FRD frame, nearest
/// horizontal range per bin. Airframe, blocked sectors and floor are kept:
/// the agent pilot reads a bin with no return as free space out to its
/// horizon, so such a return must stay a bound. `ups` is the world's up in
/// the body FLU frame at host times (from the flight controller's roll and
/// pitch), oldest first; a return captured between two of them
/// ([`ATTITUDE_BRACKET_NS`]) is levelled with the attitude at its capture,
/// and left out when it lies more than [`BAND_3D_TOP_M`] over the sensor: a
/// crown flattened would read as a wall at its horizontal range. A return
/// with no attitude around it is kept: a body-frame height, or a stale
/// attitude, would drop, on a pitching drone, objects level with it a few
/// metres ahead.
/// A consumer that wants obstacles only (the navigation extension's `OBSTACLE_DISTANCE`)
/// filters them itself. `t_body_sensor` is the LiDAR mount (FLU);
/// `blocked_azimuth_deg` the rig's sensor-frame blocked sectors, published
/// as [`ScanDescriptor::blocked_bins`].
pub fn body_scan(
    scan: &LidarScan,
    t_body_sensor: &[f64; 16],
    blocked_azimuth_deg: &[[f64; 2]],
    ups: &[(u64, [f64; 3])],
) -> ScanDescriptor {
    let (r, t) = se3::from_mat4(t_body_sensor);
    let mut ranges = vec![0.0f32; SCAN_BINS];
    let inc = TAU / SCAN_BINS as f32;
    for (i, p) in scan.points.iter().enumerate() {
        let b = se3::mat_vec(&r, [p[0] as f64, p[1] as f64, p[2] as f64]);
        let at = scan.t_ns + u64::from(scan.t_offset_ns.get(i).copied().unwrap_or(0));
        if up_at(ups, at)
            .is_some_and(|u| (u[0] * b[0] + u[1] * b[1] + u[2] * b[2]) as f32 > BAND_3D_TOP_M)
        {
            continue;
        }
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
        blocked_bins: blocked_bins(&r, blocked_azimuth_deg),
    }
}

/// The body FRD bins whose centre bearing, turned into the sensor frame by
/// the mount rotation `r_body_sensor`, lies in a blocked sector.
fn blocked_bins(r_body_sensor: &[f64; 9], blocked_azimuth_deg: &[[f64; 2]]) -> Vec<u16> {
    if blocked_azimuth_deg.is_empty() {
        return Vec::new();
    }
    let r_sensor_body = se3::transpose(r_body_sensor);
    let inc = std::f64::consts::TAU / SCAN_BINS as f64;
    (0..SCAN_BINS)
        .filter(|&i| {
            let a = (i as f64 + 0.5) * inc;
            // FRD bearing `a` (clockwise) as an FLU direction.
            let s = se3::mat_vec(&r_sensor_body, [a.cos(), -a.sin(), 0.0]);
            ados_scan2d::scan::is_blocked(s[1].atan2(s[0]).to_degrees(), blocked_azimuth_deg)
        })
        .map(|i| i as u16)
        .collect()
}

/// The scan in its sensor FRD frame as [`POLAR_BINS`] polar bins (nearest
/// return, its reflectivity as quality), stamped with the sensor's pose in
/// `map_ned` (`r_world_sensor`, `t_world_sensor` already in it). Every
/// return is kept: the live map cuts on levelled height with that pose.
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
    use ados_livemap::BAND_TOP_M;

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
        let d = body_scan(&scan, &se3::to_mat4(&se3::IDENTITY, [0.0; 3]), &[], &[]);
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

    fn ring_scan(points: Vec<[f32; 3]>) -> LidarScan {
        let n = points.len();
        LidarScan {
            sensor_id: "lidar0".into(),
            t_ns: 0,
            points,
            t_offset_ns: vec![0; n],
            intensity: vec![0; n],
        }
    }

    /// The agent pilot's reading of one shared-scan bin, mirrored from
    /// ADOSDroneAgent `crates/ados-pilot/src/obstacles.rs`
    /// (`Obstacles::ingest_scan`): a range of 0.05 m or less, or not
    /// finite, is no return, and the pilot places a horizon point at
    /// `SCAN_HORIZON_M` (12 m), which its brake treats as free space up to
    /// there. Anything else bounds the free space at that range.
    const PILOT_NO_RETURN_M: f32 = 0.05;
    const PILOT_SCAN_HORIZON_M: f32 = 12.0;
    fn pilot_free_to_m(r: f32) -> f32 {
        if r <= PILOT_NO_RETURN_M || !r.is_finite() {
            PILOT_SCAN_HORIZON_M
        } else {
            r
        }
    }

    #[test]
    fn the_pilot_reads_an_empty_bin_as_free_to_its_horizon() {
        assert_eq!(pilot_free_to_m(0.0), 12.0);
        assert_eq!(pilot_free_to_m(0.05), 12.0);
        assert_eq!(pilot_free_to_m(f32::NAN), 12.0);
        assert_eq!(pilot_free_to_m(0.2), 0.2);
    }

    /// Pitched 10° nose-down 1.5 m up, a planar scan meets the floor about
    /// 9.2 m ahead (0.12 m mount); a prop sits 0.2 m to the front right and
    /// a leg 1 m behind, inside the rig's blocked sector. None of them may
    /// read to the pilot as free space past the return.
    #[test]
    fn floor_airframe_and_blocked_returns_never_read_as_free_space() {
        let mount = se3::to_mat4(&se3::IDENTITY, [0.0, 0.0, 0.12]);
        let floor_x = (1.5 + 0.12) / 10f64.to_radians().tan();
        let prop = [0.2 * 0.5f32.cos(), -0.2 * 0.5f32.sin(), 0.0];
        let scan = ring_scan(vec![[floor_x as f32, 0.0, 0.0], prop, [-1.0, 0.0, 0.0]]);
        let blocked = [[170.0, 190.0]];
        let d = body_scan(&scan, &mount, &blocked, &[]);
        let prop_bin = (0.5f64.to_degrees()) as usize;
        for (bin, want) in [(0, floor_x as f32), (prop_bin, 0.2), (180, 1.0)] {
            let free = pilot_free_to_m(d.ranges_m[bin]);
            assert!(
                (free - want).abs() < 1e-3,
                "bin {bin}: the pilot sees {free} m free, the return is at {want} m"
            );
        }
        assert!(d.blocked_bins.contains(&180), "{:?}", d.blocked_bins);
        assert!(!d.blocked_bins.contains(&0) && !d.blocked_bins.contains(&90));
    }

    #[test]
    fn blocked_bins_follow_the_mount_yaw() {
        // Sensor turned 90° left (FLU yaw +90°): its rear sector (170..190)
        // faces the body's right, FRD bearing 90°.
        let mount = se3::to_mat4(&yaw_flu(std::f64::consts::FRAC_PI_2), [0.0; 3]);
        let d = body_scan(&ring_scan(vec![]), &mount, &[[170.0, 190.0]], &[]);
        assert!(d.blocked_bins.contains(&90), "{:?}", d.blocked_bins);
        assert!(!d.blocked_bins.contains(&180));
        assert!(body_scan(&ring_scan(vec![]), &mount, &[], &[])
            .blocked_bins
            .is_empty());
    }

    /// World up in the body FLU frame, pitched `deg` nose-down.
    fn up_nose_down(deg: f64) -> [f64; 3] {
        let a = deg.to_radians();
        [-a.sin(), 0.0, a.cos()]
    }

    /// A 3D lidar under a canopy, pitched 8° nose-down: an object level with
    /// the sensor 5 m ahead (0.70 m over the body plane) is kept; a crown 2 m
    /// over the sensor is left out; the recorded polar scan keeps both.
    #[test]
    fn levelled_returns_over_the_band_are_not_flattened_into_walls() {
        let a = 8f32.to_radians();
        // Level with the sensor 5 m ahead, in the pitched body frame.
        let level = [5.0 * a.cos(), 0.0, 5.0 * a.sin()];
        // 2 m over the sensor, 3 m to the right.
        let crown = [-2.0 * a.sin(), -3.0, 2.0 * a.cos()];
        let scan = ring_scan(vec![level, crown]);
        let id = se3::to_mat4(&se3::IDENTITY, [0.0; 3]);
        assert!(
            level[2] > BAND_TOP_M,
            "level object sits over a body-frame cut"
        );
        let d = body_scan(&scan, &id, &[], &[(0, up_nose_down(8.0))]);
        assert!(
            (d.ranges_m[0] - 5.0 * a.cos()).abs() < 1e-4,
            "{}",
            d.ranges_m[0]
        );
        assert!(
            d.ranges_m.iter().filter(|r| **r > 0.0).count() == 1,
            "crown kept"
        );
        let p = polar_scan(&scan, 0, &se3::IDENTITY, [0.0; 3], VioHealth::Good);
        assert_eq!(p.range_mm.iter().filter(|r| **r > 0).count(), 2);
    }

    /// Without an attitude nothing is left out.
    #[test]
    fn without_an_attitude_no_return_is_dropped() {
        let scan = ring_scan(vec![[2.0, 0.0, 1.5], [5.0, 0.0, 0.2], [0.0, -3.0, 2.0]]);
        let d = body_scan(&scan, &se3::to_mat4(&se3::IDENTITY, [0.0; 3]), &[], &[]);
        assert!((d.ranges_m[0] - 2.0).abs() < 1e-6, "{}", d.ranges_m[0]);
        assert!((d.ranges_m[90] - 3.0).abs() < 1e-6, "{}", d.ranges_m[90]);
    }

    /// Pitching nose-up at 60°/s through a scan, attitudes every 20 ms: an
    /// object level with the sensor 8 m ahead, captured at the scan's start,
    /// is levelled with the attitude then and kept (levelled with the
    /// newest, 9.6° later, it sits 1.33 m up and would be dropped); a crown
    /// 2 m over the sensor mid-scan is left out; a return captured past the
    /// last attitude is kept.
    #[test]
    fn each_return_is_levelled_with_the_attitude_at_its_capture() {
        let t0 = 1_000_000_000u64;
        let pitch_at = |t: u64| 60f64.to_radians() * (t as f64 - t0 as f64) * 1e-9;
        let up = |th: f64| [th.sin(), 0.0, th.cos()];
        let ups: Vec<(u64, [f64; 3])> = (0..=8u64)
            .map(|k| t0 + k * 20_000_000)
            .map(|t| (t, up(pitch_at(t))))
            .collect();
        // Nose-up `th`: the world point (x, y, z) in the body FLU frame.
        let body = |w: [f64; 3], th: f64| {
            let (s, c) = th.sin_cos();
            [
                (w[0] * c + w[2] * s) as f32,
                w[1] as f32,
                (-w[0] * s + w[2] * c) as f32,
            ]
        };
        let level = body([8.0, 0.0, 0.0], 0.0);
        let crown = body([0.0, -3.0, 2.0], pitch_at(t0 + 50_000_000));
        let late = [-4.0, 0.0, 1.5];
        let newest = ups.last().unwrap().1;
        let h = newest[0] * level[0] as f64 + newest[2] * level[2] as f64;
        assert!(
            h > BAND_3D_TOP_M as f64,
            "the newest attitude lifts it {h:.2} m"
        );
        let scan = LidarScan {
            sensor_id: "lidar0".into(),
            t_ns: t0,
            points: vec![level, crown, late],
            t_offset_ns: vec![0, 50_000_000, 300_000_000],
            intensity: vec![0; 3],
        };
        let d = body_scan(&scan, &se3::to_mat4(&se3::IDENTITY, [0.0; 3]), &[], &ups);
        assert!((d.ranges_m[0] - 8.0).abs() < 1e-4, "{}", d.ranges_m[0]);
        let kept = d.ranges_m.iter().filter(|r| **r > 0.0).count();
        assert_eq!(kept, 2, "the crown is left out");
        assert!((d.ranges_m[180] - 4.0).abs() < 1e-4, "{}", d.ranges_m[180]);
    }
}
