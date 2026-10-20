//! Scene depth from the latest LiDAR scan: its median range (the keyframe
//! baseline scales with it) and, once the scan's world attitude is known, the
//! median range around each world azimuth (coverage rays stop there).

use crate::lidar_scan::LidarScan;
use crate::pose::se3;

/// Points nearer than this are the rig or its operator, not the scene.
pub const MIN_RANGE_M: f64 = 0.3;
/// Only points within this of the world horizontal bound an azimuth sector,
/// so a 3D LiDAR's floor and ceiling returns do not shorten the rays.
pub const MAX_ELEVATION_DEG: f64 = 15.0;
/// Half-width of the azimuth sector a ray takes its range from, degrees.
pub const SECTOR_HALF_DEG: usize = 10;
/// A scan describes the scene for this long around its time.
pub const FRESH_NS: u64 = 1_000_000_000;
/// A dense scan is subsampled to at most this many points.
const MAX_POINTS: usize = 4096;

/// What one scan says about the scene's depth.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarDepth {
    /// Host monotonic ns of the scan.
    pub t_ns: u64,
    /// Median range over the scan's points, metres.
    pub median_m: Option<f64>,
    /// Per world azimuth degree (counter-clockwise from world +x), the median
    /// horizontal range of the near-horizontal points within
    /// ±[`SECTOR_HALF_DEG`]; `None` without the scan's world attitude or
    /// without a point in the sector.
    sectors: Option<Vec<Option<f32>>>,
}

impl LidarDepth {
    /// Summarise `scan`; `r_world_sensor` (row-major) rotates its points into
    /// the world for the azimuth sectors.
    pub fn from_scan(scan: &LidarScan, r_world_sensor: Option<&[f64; 9]>) -> Self {
        let step = scan.points.len().div_ceil(MAX_POINTS).max(1);
        let pts = scan.points.iter().step_by(step).map(|p| {
            let v = [p[0] as f64, p[1] as f64, p[2] as f64];
            (v, (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt())
        });
        let mut ranges: Vec<f64> = Vec::new();
        let mut bins: Vec<Vec<f32>> = vec![Vec::new(); 360];
        let max_tan = MAX_ELEVATION_DEG.to_radians().tan();
        for (v, r) in pts {
            if !r.is_finite() || r < MIN_RANGE_M {
                continue;
            }
            ranges.push(r);
            let Some(rot) = r_world_sensor else { continue };
            let w = se3::mat_vec(rot, v);
            let horiz = w[0].hypot(w[1]);
            if horiz <= 0.0 || w[2].abs() > max_tan * horiz {
                continue;
            }
            let az = w[1].atan2(w[0]).to_degrees().rem_euclid(360.0);
            bins[(az as usize).min(359)].push(horiz as f32);
        }
        let median_m = median(&mut ranges);
        let sectors = r_world_sensor.map(|_| {
            let mut window: Vec<f32> = Vec::new();
            (0..360)
                .map(|c: usize| {
                    window.clear();
                    for k in 0..=2 * SECTOR_HALF_DEG {
                        window.extend_from_slice(&bins[(c + 360 + k - SECTOR_HALF_DEG) % 360]);
                    }
                    if window.is_empty() {
                        return None;
                    }
                    let mid = window.len() / 2;
                    Some(*window.select_nth_unstable_by(mid, f32::total_cmp).1)
                })
                .collect()
        });
        Self {
            t_ns: scan.t_ns,
            median_m,
            sectors,
        }
    }

    /// Whether the scan describes the scene at host `t_ns`.
    pub fn fresh_at(&self, t_ns: u64) -> bool {
        self.t_ns.abs_diff(t_ns) <= FRESH_NS
    }

    /// The median range around world azimuth `az_deg` (counter-clockwise
    /// from world +x), when the scan has one there.
    pub fn range_at(&self, az_deg: f64) -> Option<f64> {
        let i = (az_deg.rem_euclid(360.0).round() as usize) % 360;
        self.sectors.as_ref()?[i].map(f64::from)
    }
}

fn median(v: &mut [f64]) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    let mid = v.len() / 2;
    Some(*v.select_nth_unstable_by(mid, f64::total_cmp).1)
}
