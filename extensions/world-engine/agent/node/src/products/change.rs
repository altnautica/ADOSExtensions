//! Cloud-to-cloud change detection: the later cloud is first registered into
//! the earlier cloud's frame, then each of its points is measured against its
//! nearest neighbour in the earlier one.

use ados_capture_contract::GeoAnchor;
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use serde::{Deserialize, Serialize};

use super::align::{
    apply, icp, kd_tree, unaligned, voxel_representatives, yaw_search, AlignResult, ALIGN_VOXEL_M,
    IDENTITY_4X4, MAX_ITERATIONS,
};
use super::cloud::PointCloud;
use super::geo::anchor_transform;
use super::ProductError;

/// Below this fitness two clouds are never compared.
pub const MIN_FITNESS: f64 = 0.3;
/// Fitness required when only one cloud is anchored (no geo prior).
pub const MIN_FITNESS_UNANCHORED: f64 = 0.6;
/// Above this residual two clouds are never compared (m).
pub const MAX_RMSE_M: f64 = 0.15;

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
    /// How `b` was brought into `a`'s frame; absent for a plain
    /// [`compare`] of clouds already in one frame.
    #[serde(default)]
    pub alignment: Option<Alignment>,
}

/// Start of the registration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlignInit {
    /// Both clouds anchored: the exact WGS-84 relation of the anchors.
    Anchor,
    /// At least one local frame: global heading search about Z.
    YawSearch,
}

/// Registration recorded in the change summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Alignment {
    /// Row-major 4×4 `T_a_b` applied to `b` before comparing.
    pub transform: [f64; 16],
    pub fitness: f64,
    pub rmse_m: f64,
    pub init: AlignInit,
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
    measure(kd_tree(&a.points).as_ref(), b, threshold_m)
}

/// [`compare`] against a prepared index of `a` (`None` when `a` is empty).
fn measure(
    tree: Option<&ImmutableKdTree<f64, 3>>,
    b: &PointCloud,
    threshold_m: f64,
) -> ChangeResult {
    let distances: Vec<f32> = match tree {
        None => vec![f32::INFINITY; b.points.len()],
        Some(tree) => b
            .points
            .iter()
            .map(|p| {
                tree.query(p)
                    .nearest_one::<SquaredEuclidean<f64>>()
                    .execute()
                    .distance
                    .sqrt() as f32
            })
            .collect(),
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
        alignment: None,
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

fn describe(anchor: Option<&GeoAnchor>) -> String {
    match anchor {
        Some(g) => format!(
            "anchored at {:.7}°, {:.7}°, {:.2} m",
            g.lat_deg, g.lon_deg, g.h_ellipsoid_m
        ),
        None => "local frame".into(),
    }
}

/// Registers `b` into `a`'s frame, then [`compare`]s them. The registration
/// starts from the anchors' WGS-84 relation when both clouds are anchored,
/// else from a global heading search about Z (see
/// [`super::align::align_global`]). The returned cloud is `b` in `a`'s frame.
///
/// Refuses with [`ProductError::FramesUnrelated`] when the final fitness is
/// below [`MIN_FITNESS`] (or [`MIN_FITNESS_UNANCHORED`] when only one cloud
/// is anchored) or the residual exceeds [`MAX_RMSE_M`].
pub fn align_and_compare(
    a: &PointCloud,
    anchor_a: Option<&GeoAnchor>,
    b: &PointCloud,
    anchor_b: Option<&GeoAnchor>,
    threshold_m: f64,
) -> Result<(ChangeResult, AlignResult), ProductError> {
    let tree = kd_tree(&a.points);
    let fit = match (&tree, anchor_a, anchor_b) {
        (None, ..) => unaligned(IDENTITY_4X4),
        (Some(tree), Some(ga), Some(gb)) => {
            let moving = voxel_representatives(&b.points, ALIGN_VOXEL_M);
            icp(
                tree,
                &a.points,
                &moving,
                anchor_transform(ga, gb),
                MAX_ITERATIONS,
            )
        }
        (Some(tree), ..) => yaw_search(tree, &a.points, &b.points),
    };
    let init = if anchor_a.is_some() && anchor_b.is_some() {
        AlignInit::Anchor
    } else {
        AlignInit::YawSearch
    };
    let min_fitness = if anchor_a.is_some() != anchor_b.is_some() {
        MIN_FITNESS_UNANCHORED
    } else {
        MIN_FITNESS
    };
    if fit.fitness < min_fitness || fit.rmse_m.is_nan() || fit.rmse_m > MAX_RMSE_M {
        return Err(ProductError::FramesUnrelated(format!(
            "cloud a ({}) and cloud b ({}) do not align: fitness {:.3} (need ≥ {min_fitness:.2}), \
             rmse {:.3} m (limit {MAX_RMSE_M:.2} m)",
            describe(anchor_a),
            describe(anchor_b),
            fit.fitness,
            fit.rmse_m,
        )));
    }
    let moved = PointCloud {
        points: b.points.iter().map(|p| apply(&fit.transform, p)).collect(),
        colors: b.colors.clone(),
    };
    let mut result = measure(tree.as_ref(), &moved, threshold_m);
    result.summary.alignment = Some(Alignment {
        transform: fit.transform,
        fitness: fit.fitness,
        rmse_m: fit.rmse_m,
        init,
    });
    Ok((result, fit))
}
