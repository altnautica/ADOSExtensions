//! The editor's point operations on a layered cloud, in memory: crop,
//! delete, patch, layer delete and decimate, plus the deviation a
//! decimation introduced.
//!
//! A layered cloud keeps every layer's points contiguous and in layer order
//! (the base, then each patch in the order it was made): every operation
//! here filters in place or appends a new layer at the end, and decimation
//! runs layer by layer, so the order holds.

use kiddo::{ImmutableKdTree, SquaredEuclidean};

use ados_mapping_protocol::edit::{
    point_in_polygon, DecimateTarget, Deviation, Keep, PatchSource, Transform2,
};

use crate::products::cloud::{voxel_downsample, PointCloud};

/// Points processed between two cancellation checks.
const CANCEL_EVERY: usize = 1_000_000;
/// Bisection steps searching the voxel edge of a decimation.
const DECIMATE_STEPS: usize = 12;
/// A decimation is done once it is this close to its target.
const DECIMATE_TOLERANCE: f64 = 0.02;
/// Source points the deviation estimate samples.
const DEVIATION_SAMPLE: usize = 100_000;

/// The job was cancelled while an operation ran.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

/// A cloud with each point's layer id.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Layered {
    pub cloud: PointCloud,
    pub layers: Vec<u16>,
}

impl Layered {
    pub fn len(&self) -> usize {
        self.cloud.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cloud.points.is_empty()
    }

    /// Keep the points `keep` accepts (it sees each point and its layer), in
    /// order; returns how many went.
    fn retain(
        &mut self,
        keep: impl Fn([f64; 3], u16) -> bool,
        cancelled: &dyn Fn() -> bool,
    ) -> Result<u64, Cancelled> {
        let before = self.len();
        let mut flags = Vec::with_capacity(before);
        for (i, (&p, &id)) in self.cloud.points.iter().zip(&self.layers).enumerate() {
            if i % CANCEL_EVERY == 0 && i > 0 && cancelled() {
                return Err(Cancelled);
            }
            flags.push(keep(p, id));
        }
        let mut it = flags.iter();
        self.cloud.points.retain(|_| *it.next().unwrap_or(&false));
        if let Some(c) = &mut self.cloud.colors {
            let mut it = flags.iter();
            c.retain(|_| *it.next().unwrap_or(&false));
        }
        let mut it = flags.iter();
        self.layers.retain(|_| *it.next().unwrap_or(&false));
        Ok((before - self.len()) as u64)
    }

    /// `(layer id, start, end)` of each contiguous run.
    pub fn runs(&self) -> Vec<(u16, usize, usize)> {
        let mut out: Vec<(u16, usize, usize)> = Vec::new();
        for (i, &id) in self.layers.iter().enumerate() {
            match out.last_mut() {
                Some(run) if run.0 == id => run.2 = i + 1,
                _ => out.push((id, i, i + 1)),
            }
        }
        out
    }

    /// Points per layer id.
    pub fn counts(&self) -> Vec<(u16, u64)> {
        self.runs()
            .into_iter()
            .map(|(id, a, b)| (id, (b - a) as u64))
            .collect()
    }
}

/// Whether `p` lies inside the ground polygon and the optional height band.
pub fn in_region(polygon: &[[f64; 2]], z: Option<[f64; 2]>, p: [f64; 3]) -> bool {
    z.is_none_or(|[lo, hi]| p[2] >= lo && p[2] <= hi) && point_in_polygon(polygon, [p[0], p[1]])
}

/// Keep the points inside (or outside) the region; returns how many went.
pub fn crop(
    cloud: &mut Layered,
    keep: Keep,
    polygon: &[[f64; 2]],
    z: Option<[f64; 2]>,
    cancelled: &dyn Fn() -> bool,
) -> Result<u64, Cancelled> {
    cloud.retain(
        |p, _| in_region(polygon, z, p) == (keep == Keep::Inside),
        cancelled,
    )
}

/// Remove the points inside the region; returns how many went.
pub fn delete(
    cloud: &mut Layered,
    polygon: &[[f64; 2]],
    z: Option<[f64; 2]>,
    cancelled: &dyn Fn() -> bool,
) -> Result<u64, Cancelled> {
    cloud.retain(|p, _| !in_region(polygon, z, p), cancelled)
}

/// Remove every point of layer `id`; returns how many went.
pub fn delete_layer(
    cloud: &mut Layered,
    id: u16,
    cancelled: &dyn Fn() -> bool,
) -> Result<u64, Cancelled> {
    cloud.retain(|_, layer| layer != id, cancelled)
}

/// What a patch added.
#[derive(Debug, Clone, PartialEq)]
pub struct Patched {
    pub added: u64,
    /// The target placement overlaps the source box.
    pub self_overlap: bool,
    /// The layer ids the copied points came from, ascending.
    pub source_layers: Vec<u16>,
}

/// `p` rotated about the box centre by `t.rotate_deg`, then translated.
pub fn place(source: &PatchSource, t: &Transform2, p: [f64; 2]) -> [f64; 2] {
    let (lo, hi) = source.corners();
    let c = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let (s, co) = t.rotate_deg.to_radians().sin_cos();
    let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
    [
        c[0] + co * dx - s * dy + t.translate[0],
        c[1] + s * dx + co * dy + t.translate[1],
    ]
}

/// Copy the points inside the source box (every layer, any height) to the
/// target placement as layer `id`, appended after every other layer.
pub fn patch(cloud: &mut Layered, id: u16, source: &PatchSource, target: &Transform2) -> Patched {
    let picked: Vec<usize> = (0..cloud.len())
        .filter(|&i| {
            let p = cloud.cloud.points[i];
            source.contains([p[0], p[1]])
        })
        .collect();
    let mut source_layers: Vec<u16> = picked.iter().map(|&i| cloud.layers[i]).collect();
    source_layers.sort_unstable();
    source_layers.dedup();
    for &i in &picked {
        let p = cloud.cloud.points[i];
        let [x, y] = place(source, target, [p[0], p[1]]);
        cloud.cloud.points.push([x, y, p[2]]);
        if let Some(c) = &mut cloud.cloud.colors {
            c.push(c[i]);
        }
        cloud.layers.push(id);
    }
    // The placed box's bounds against the source box: any shared area is an
    // overlap.
    let (lo, hi) = source.corners();
    let corners = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]].map(|c| place(source, target, c));
    let (mut tlo, mut thi) = (corners[0], corners[0]);
    for c in &corners[1..] {
        tlo = [tlo[0].min(c[0]), tlo[1].min(c[1])];
        thi = [thi[0].max(c[0]), thi[1].max(c[1])];
    }
    let self_overlap = tlo[0] < hi[0] && thi[0] > lo[0] && tlo[1] < hi[1] && thi[1] > lo[1];
    Patched {
        added: picked.len() as u64,
        self_overlap,
        source_layers,
    }
}

/// Voxels a layer keeps at edge `v`.
fn voxel_count(points: &[[f64; 3]], v: f64) -> usize {
    let mut keys: Vec<[i64; 3]> = points
        .iter()
        .map(|p| p.map(|c| (c / v).floor() as i64))
        .collect();
    keys.sort_unstable();
    keys.dedup();
    keys.len()
}

/// Reduce the cloud to about `target` points (within 2 %), voxel-averaging
/// each layer on its own with one shared voxel edge found by bisection. When
/// the grid cannot land that close (a cloud of a few discrete heights jumps
/// past the target as the edge crosses their spacing), the finest edge above
/// the target is used and the result thinned evenly to it. Returns the
/// reduced cloud and the edge used.
pub fn decimate(
    cloud: &Layered,
    target: &DecimateTarget,
    cancelled: &dyn Fn() -> bool,
) -> Result<(Layered, f64), Cancelled> {
    let want = target.points_for(cloud.len() as u64) as f64;
    let runs = cloud.runs();
    let Some((lo, hi)) = cloud.cloud.bounds() else {
        return Ok((cloud.clone(), 0.0));
    };
    let span = (0..3)
        .map(|k| hi[k] - lo[k])
        .fold(0.0f64, f64::max)
        .max(1e-3);
    let total = |v: f64| -> usize {
        runs.iter()
            .map(|&(_, a, b)| voxel_count(&cloud.cloud.points[a..b], v))
            .sum()
    };
    // Counts fall as the edge grows: search log-space between a sub-millimetre
    // edge and the cloud's span.
    let (mut small, mut large) = ((span / 1e6).ln(), span.ln());
    let mut best = (span, f64::INFINITY);
    // The largest edge that still keeps at least the target.
    let mut above: Option<f64> = None;
    for _ in 0..DECIMATE_STEPS {
        if cancelled() {
            return Err(Cancelled);
        }
        let mid = (small + large) / 2.0;
        let n = total(mid.exp()) as f64;
        let err = (n - want).abs() / want.max(1.0);
        if err < best.1 {
            best = (mid.exp(), err);
        }
        if err <= DECIMATE_TOLERANCE {
            break;
        }
        if n > want {
            above = Some(above.map_or(mid.exp(), |a: f64| a.max(mid.exp())));
            small = mid;
        } else {
            large = mid;
        }
    }
    let v = match above {
        Some(a) if best.1 > DECIMATE_TOLERANCE => a,
        _ => best.0,
    };
    let mut out = Layered {
        cloud: PointCloud {
            points: Vec::new(),
            colors: cloud.cloud.colors.as_ref().map(|_| Vec::new()),
        },
        layers: Vec::new(),
    };
    for (id, a, b) in runs {
        let part = PointCloud {
            points: cloud.cloud.points[a..b].to_vec(),
            colors: cloud.cloud.colors.as_ref().map(|c| c[a..b].to_vec()),
        };
        let d = voxel_downsample(&part, v);
        out.layers.extend(std::iter::repeat_n(id, d.points.len()));
        out.cloud.points.extend(d.points);
        if let (Some(dst), Some(src)) = (&mut out.cloud.colors, d.colors) {
            dst.extend(src);
        }
    }
    let keep = want.round() as usize;
    if out.len() as f64 > want * (1.0 + DECIMATE_TOLERANCE) && keep > 0 {
        // Even thinning keeps order, so layers stay contiguous.
        let n = out.len();
        let picks: Vec<bool> = (0..n)
            .map(|i| (i * keep) / n != ((i + 1) * keep) / n)
            .collect();
        let mut it = picks.iter();
        out.cloud.points.retain(|_| *it.next().unwrap_or(&false));
        if let Some(c) = &mut out.cloud.colors {
            let mut it = picks.iter();
            c.retain(|_| *it.next().unwrap_or(&false));
        }
        let mut it = picks.iter();
        out.layers.retain(|_| *it.next().unwrap_or(&false));
    }
    Ok((out, v))
}

/// Distance from a sample of `source` (at most 100,000 points, evenly
/// strided) to the nearest point of `result`: mean and 95th percentile.
pub fn deviation(source: &PointCloud, result: &PointCloud) -> Option<Deviation> {
    if source.points.is_empty() || result.points.is_empty() {
        return None;
    }
    let tree: ImmutableKdTree<f64, 3> = ImmutableKdTree::new_from_slice(&result.points).ok()?;
    let stride = source.points.len().div_ceil(DEVIATION_SAMPLE).max(1);
    let mut d: Vec<f64> = source
        .points
        .iter()
        .step_by(stride)
        .map(|p| {
            tree.query(p)
                .nearest_one::<SquaredEuclidean<f64>>()
                .execute()
                .distance
                .sqrt()
        })
        .collect();
    d.sort_unstable_by(f64::total_cmp);
    let mean = d.iter().sum::<f64>() / d.len() as f64;
    let p95 = d[((d.len() as f64 * 0.95).ceil() as usize).clamp(1, d.len()) - 1];
    Some(Deviation { mean, p95 })
}

#[cfg(test)]
#[path = "edit_ops_tests.rs"]
mod tests;
