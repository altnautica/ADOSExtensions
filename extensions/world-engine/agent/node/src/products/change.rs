//! Cloud-to-cloud change detection: each point of the later cloud is measured
//! against its nearest neighbour in the earlier one.

use kiddo::{ImmutableKdTree, SquaredEuclidean};
use serde::{Deserialize, Serialize};

use super::cloud::PointCloud;

/// Distance at which the change colour ramp saturates (red).
pub const RAMP_CAP_M: f64 = 0.5;

/// `change_summary.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChangeSummary {
    pub threshold_m: f64,
    pub points_compared: usize,
    pub points_changed: usize,
    pub changed_pct: f64,
    /// Largest nearest-neighbour distance over the compared points.
    pub max_m: f64,
    /// Mean distance over the changed points with a finite distance (0 when
    /// none).
    pub mean_changed_m: f64,
    /// `[[minx, miny, minz], [maxx, maxy, maxz]]` of the changed points.
    pub bbox_changed: Option<[[f64; 3]; 2]>,
}

#[derive(Debug, Clone)]
pub struct ChangeResult {
    /// `b` coloured: grey below the threshold, yellow→red above, saturating
    /// at [`RAMP_CAP_M`].
    pub cloud: PointCloud,
    /// Per point of `b`: distance to the nearest point of `a` (infinite when
    /// `a` is empty).
    pub distances: Vec<f32>,
    pub summary: ChangeSummary,
}

const GREY: [u16; 3] = [128 * 257; 3];

fn ramp(d: f64, threshold: f64) -> [u16; 3] {
    if d < threshold {
        return GREY;
    }
    let span = RAMP_CAP_M - threshold;
    let t = if span > 0.0 {
        ((d.min(RAMP_CAP_M) - threshold) / span).clamp(0.0, 1.0)
    } else {
        1.0
    };
    [65535, ((1.0 - t) * 65535.0).round() as u16, 0]
}

/// Compares `b` against `a`; a point of `b` changed when its nearest point of
/// `a` is at least `threshold_m` away.
pub fn compare(a: &PointCloud, b: &PointCloud, threshold_m: f64) -> ChangeResult {
    let distances: Vec<f32> = if a.points.is_empty() {
        vec![f32::INFINITY; b.points.len()]
    } else {
        let tree: ImmutableKdTree<f64, 3> = ImmutableKdTree::new_from_slice(&a.points)
            .expect("u32 item index covers the cloud size");
        b.points
            .iter()
            .map(|p| {
                tree.query(p)
                    .nearest_one::<SquaredEuclidean<f64>>()
                    .execute()
                    .distance
                    .sqrt() as f32
            })
            .collect()
    };
    let mut colors = Vec::with_capacity(b.points.len());
    let (mut changed, mut finite, mut sum, mut max) = (0usize, 0usize, 0.0f64, 0.0f64);
    let mut bbox: Option<[[f64; 3]; 2]> = None;
    for (p, &d) in b.points.iter().zip(&distances) {
        let d = d as f64;
        colors.push(ramp(d, threshold_m));
        if d.is_finite() {
            max = max.max(d);
        }
        if d >= threshold_m {
            changed += 1;
            if d.is_finite() {
                finite += 1;
                sum += d;
            }
            let bb = bbox.get_or_insert([*p, *p]);
            for k in 0..3 {
                bb[0][k] = bb[0][k].min(p[k]);
                bb[1][k] = bb[1][k].max(p[k]);
            }
        }
    }
    let compared = b.points.len();
    let summary = ChangeSummary {
        threshold_m,
        points_compared: compared,
        points_changed: changed,
        changed_pct: if compared > 0 {
            100.0 * changed as f64 / compared as f64
        } else {
            0.0
        },
        max_m: max,
        mean_changed_m: if finite > 0 { sum / finite as f64 } else { 0.0 },
        bbox_changed: bbox,
    };
    ChangeResult {
        cloud: PointCloud {
            points: b.points.clone(),
            colors: Some(colors),
        },
        distances,
        summary,
    }
}
