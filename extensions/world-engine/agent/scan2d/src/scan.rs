//! From a LiDAR cloud to a levelled, motion-corrected 2D scan.

use crate::config::Scan2dConfig;

/// One LiDAR revolution as the driver delivers it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ScanInput {
    /// Nanoseconds of the first point, the same clock as the IMU.
    pub t_ns: u64,
    /// Points in the LiDAR frame (FLU), metres. A planar LiDAR has z = 0.
    pub points: Vec<[f32; 3]>,
    /// Per-point offset from `t_ns`, nanoseconds. Missing entries count as 0.
    pub t_offset_ns: Vec<u32>,
}

/// Sensor motion at a scan, used to level and deskew it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Motion {
    /// LiDAR roll and pitch, radians.
    pub roll: f64,
    pub pitch: f64,
    /// Rotation rate about the world vertical, rad/s.
    pub yaw_rate: f64,
    /// Velocity in the levelled LiDAR frame at `t_ns`, m/s.
    pub velocity: [f64; 2],
}

/// A levelled scan expressed in the LiDAR frame at its first point's time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scan2d {
    pub t_ns: u64,
    pub points: Vec<[f32; 2]>,
    pub t_offset_ns: Vec<u32>,
}

impl Scan2d {
    /// Drop points outside the range limits or inside a blocked sector,
    /// level them by roll and pitch, then move every point into the frame at
    /// `t_ns`: a point seen `t` after the first was seen from a sensor already
    /// turned by `yaw_rate·t` and moved by `velocity·t`.
    pub fn from_cloud(scan: &ScanInput, cfg: &Scan2dConfig, m: &Motion) -> Scan2d {
        let (sr, cr) = m.roll.sin_cos();
        let (sp, cp) = m.pitch.sin_cos();
        let mut out = Scan2d {
            t_ns: scan.t_ns,
            points: Vec::with_capacity(scan.points.len()),
            t_offset_ns: Vec::with_capacity(scan.points.len()),
        };
        for (i, p) in scan.points.iter().enumerate() {
            let [x, y, z] = [p[0] as f64, p[1] as f64, p[2] as f64];
            let r = (x * x + y * y + z * z).sqrt();
            if !r.is_finite() || r < cfg.min_range_m || r > cfg.max_range_m {
                continue;
            }
            if is_blocked(y.atan2(x).to_degrees(), &cfg.blocked_azimuth_deg) {
                continue;
            }
            // Ry(pitch)·Rx(roll), keeping x and y.
            let yl = cr * y - sr * z;
            let zl = sr * y + cr * z;
            let xl = cp * x + sp * zl;
            let dt_ns = scan.t_offset_ns.get(i).copied().unwrap_or(0);
            let dt = dt_ns as f64 * 1e-9;
            let (s, c) = (m.yaw_rate * dt).sin_cos();
            let xd = c * xl - s * yl + m.velocity[0] * dt;
            let yd = s * xl + c * yl + m.velocity[1] * dt;
            out.points.push([xd as f32, yd as f32]);
            out.t_offset_ns.push(dt_ns);
        }
        out
    }

    /// Largest point range, metres (0 for an empty scan).
    pub fn max_range(&self) -> f64 {
        self.points
            .iter()
            .map(|p| (p[0] as f64).hypot(p[1] as f64))
            .fold(0.0, f64::max)
    }
}

/// Whether azimuth `az_deg` (counter-clockwise from +x) lies in one of the
/// `[start, end]` sectors; a sector with `start > end` wraps through 0°.
pub fn is_blocked(az_deg: f64, sectors: &[[f64; 2]]) -> bool {
    let a = az_deg.rem_euclid(360.0);
    sectors.iter().any(|&[s, e]| {
        if (e - s).abs() >= 360.0 {
            return true;
        }
        let (s, e) = (s.rem_euclid(360.0), e.rem_euclid(360.0));
        if s <= e {
            a >= s && a <= e
        } else {
            a >= s || a <= e
        }
    })
}
