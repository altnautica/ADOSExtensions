//! Rigid registration of one point cloud onto another: trimmed point-to-point
//! ICP of a voxel-thinned moving cloud against the full reference cloud,
//! closed-form (Horn quaternion) update per iteration.
//!
//! Transforms are row-major 4×4 rigid matrices `[R t; 0 0 0 1]` mapping
//! points of the moving cloud into the reference frame: `p_a = R p_b + t`.

use std::collections::HashMap;

use kiddo::{ImmutableKdTree, SquaredEuclidean};

use super::cloud::PointCloud;

/// Voxel edge used to thin the moving cloud before registration (m).
pub const ALIGN_VOXEL_M: f64 = 0.05;
/// A moving point counts as fitting when this close to the reference (m).
pub const FIT_RADIUS_M: f64 = 0.10;
/// Correspondences longer than this are never used (m).
const MAX_PAIR_M: f64 = 0.5;
/// Correspondences longer than this multiple of the median are dropped.
const TRIM_MEDIANS: f64 = 3.0;
/// The median trim is relaxed early on: the cut never falls below
/// `MAX_PAIR_M · TRIM_DECAY^iteration`, so a surface family that starts
/// misaligned by more than 3× the median (e.g. the walls of one orientation
/// after a sideways start error) still pulls the solution in.
const TRIM_DECAY: f64 = 0.8;
/// Convergence: mean motion of the used points in one iteration (m).
const CONVERGED_M: f64 = 1e-5;
pub(super) const MAX_ITERATIONS: u32 = 50;
/// Yaw search: candidate headings about Z, the voxel edge of the heavily
/// thinned moving cloud and the ICP iterations each candidate gets.
const YAW_STEP_DEG: u32 = 10;
const COARSE_VOXEL_M: f64 = 0.5;
const COARSE_ITERATIONS: u32 = 8;

/// Row-major 4×4 identity.
pub const IDENTITY_4X4: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// Outcome of [`align`].
#[derive(Debug, Clone, PartialEq)]
pub struct AlignResult {
    /// `T_a_b`: maps points of `b` into `a`'s frame.
    pub transform: [f64; 16],
    /// Share of (voxel-thinned) `b` points within [`FIT_RADIUS_M`] of `a`
    /// after alignment, in `[0, 1]`.
    pub fitness: f64,
    /// RMS distance of the trimmed correspondences after alignment (m);
    /// infinite when no correspondence survives.
    pub rmse_m: f64,
    pub iterations: u32,
}

/// Applies a rigid transform to a point.
pub fn apply(t: &[f64; 16], p: &[f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| {
        t[4 * i] * p[0] + t[4 * i + 1] * p[1] + t[4 * i + 2] * p[2] + t[4 * i + 3]
    })
}

/// Matrix product `x · y` of two 4×4 transforms.
pub fn compose(x: &[f64; 16], y: &[f64; 16]) -> [f64; 16] {
    std::array::from_fn(|k| {
        let (i, j) = (k / 4, k % 4);
        (0..4).map(|m| x[4 * i + m] * y[4 * m + j]).sum()
    })
}

/// Inverse of a rigid transform.
pub fn invert(t: &[f64; 16]) -> [f64; 16] {
    let mut out = IDENTITY_4X4;
    for i in 0..3 {
        for j in 0..3 {
            out[4 * i + j] = t[4 * j + i];
        }
    }
    for i in 0..3 {
        out[4 * i + 3] = -(0..3).map(|j| out[4 * i + j] * t[4 * j + 3]).sum::<f64>();
    }
    out
}

/// Eigen-decomposition of a symmetric 4×4 matrix by cyclic Jacobi rotations;
/// returns the eigenvector of the largest eigenvalue.
fn principal_eigenvector(mut a: [[f64; 4]; 4]) -> [f64; 4] {
    let mut v = [[0.0; 4]; 4];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..64 {
        let off: f64 = (0..4)
            .flat_map(|i| (0..4).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        let scale: f64 = (0..4).map(|i| a[i][i] * a[i][i]).sum::<f64>().max(1e-300);
        if off <= 1e-30 * scale {
            break;
        }
        for p in 0..3 {
            for q in p + 1..4 {
                if a[p][q] == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                // `signum(0.0)` is 1, so a zero angle picks the 45° rotation.
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for row in a.iter_mut() {
                    let (akp, akq) = (row[p], row[q]);
                    row[p] = c * akp - s * akq;
                    row[q] = s * akp + c * akq;
                }
                let (rp, rq) = (a[p], a[q]);
                a[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
                a[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
                for row in v.iter_mut() {
                    let (vp, vq) = (row[p], row[q]);
                    row[p] = c * vp - s * vq;
                    row[q] = s * vp + c * vq;
                }
            }
        }
    }
    let best = (0..4)
        .max_by(|&i, &j| a[i][i].total_cmp(&a[j][j]))
        .unwrap_or(0);
    std::array::from_fn(|i| v[i][best])
}

/// Least-squares rigid transform (row-major 4×4) taking `src[i]` onto
/// `dst[i]` (Horn's closed form with unit quaternions). Needs at least three
/// non-collinear pairs; fewer give an undetermined or non-finite result.
pub fn horn(src: &[[f64; 3]], dst: &[[f64; 3]]) -> [f64; 16] {
    let n = src.len() as f64;
    let mean = |pts: &[[f64; 3]]| -> [f64; 3] {
        let mut s = [0.0; 3];
        for p in pts {
            for k in 0..3 {
                s[k] += p[k];
            }
        }
        s.map(|v| v / n)
    };
    let (cs, cd) = (mean(src), mean(dst));
    let mut m = [[0.0; 3]; 3];
    for (s, d) in src.iter().zip(dst) {
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += (s[i] - cs[i]) * (d[j] - cd[j]);
            }
        }
    }
    let [[sxx, sxy, sxz], [syx, syy, syz], [szx, szy, szz]] = m;
    let q = principal_eigenvector([
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ]);
    let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    let [w, x, y, z] = q.map(|v| v / norm);
    let r = [
        w * w + x * x - y * y - z * z,
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        w * w - x * x + y * y - z * z,
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    ];
    let t: [f64; 3] = std::array::from_fn(|i| {
        cd[i] - (r[3 * i] * cs[0] + r[3 * i + 1] * cs[1] + r[3 * i + 2] * cs[2])
    });
    [
        r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Nearest reference point and distance for every moving point under `t`.
fn correspond(
    tree: &ImmutableKdTree<f64, 3>,
    moving: &[[f64; 3]],
    t: &[f64; 16],
) -> Vec<([f64; 3], usize, f64)> {
    moving
        .iter()
        .map(|p| {
            let q = apply(t, p);
            let nn = tree
                .query(&q)
                .nearest_one::<SquaredEuclidean<f64>>()
                .execute();
            (q, nn.item as usize, nn.distance.sqrt())
        })
        .collect()
}

/// Distance cut of the correspondence set (3× the median, never below
/// `floor`, at most [`MAX_PAIR_M`]) and the plain median trim.
fn trim_cutoff(pairs: &[([f64; 3], usize, f64)], floor: f64) -> (f64, f64) {
    let mut d: Vec<f64> = pairs.iter().map(|p| p.2).collect();
    let mid = d.len() / 2;
    let trim = TRIM_MEDIANS * *d.select_nth_unstable_by(mid, f64::total_cmp).1;
    (trim.max(floor).min(MAX_PAIR_M), trim.min(MAX_PAIR_M))
}

/// Registers `b` onto `a` starting from `init` (`T_a_b`): `b` is thinned to
/// one point per [`ALIGN_VOXEL_M`] voxel ([`voxel_representatives`]), then
/// point-to-point ICP runs with pairs longer than 3× the median (relaxed in
/// the first iterations, see [`TRIM_DECAY`]) or 0.5 m rejected, until the
/// used points move less than 1e-5 m in an iteration the relaxation did not
/// widen, or 50 iterations.
///
/// The nearest neighbours come from `a` at full resolution. Thinning both
/// clouds leaves two near-regular voxel lattices, and point-to-point matching
/// of one lattice against another locks onto lattice-aligned poses
/// (measured on a synthetic room: up to 0.2° of residual rotation); a dense
/// target has no lattice to lock onto.
pub fn align(a: &PointCloud, b: &PointCloud, init: [f64; 16]) -> AlignResult {
    let moving = voxel_representatives(&b.points, ALIGN_VOXEL_M);
    match kd_tree(&a.points) {
        Some(tree) => icp(&tree, &a.points, &moving, init, MAX_ITERATIONS),
        None => unaligned(init),
    }
}

/// Registers `b` onto `a` without a prior pose, for gravity-aligned (Z up)
/// clouds whose relative heading is unknown: see [`yaw_search`].
pub fn align_global(a: &PointCloud, b: &PointCloud) -> AlignResult {
    match kd_tree(&a.points) {
        Some(tree) => yaw_search(&tree, &a.points, &b.points),
        None => unaligned(IDENTITY_4X4),
    }
}

/// Global search over the heading: from identity and from every
/// [`YAW_STEP_DEG`] rotation of `b` about Z around its centroid placed on
/// `a`'s centroid, a short ICP ([`COARSE_ITERATIONS`]) of `b` thinned to
/// [`COARSE_VOXEL_M`] voxels; the candidate with the best fitness (then the
/// lowest residual) seeds the full [`align`] ICP. `tree` indexes `a`.
pub(super) fn yaw_search(
    tree: &ImmutableKdTree<f64, 3>,
    a: &[[f64; 3]],
    b: &[[f64; 3]],
) -> AlignResult {
    let coarse = voxel_representatives(b, COARSE_VOXEL_M);
    let (ca, cb) = (centroid(a), centroid(b));
    let headings = (0..360).step_by(YAW_STEP_DEG as usize).map(|deg| {
        let (s, c) = (deg as f64).to_radians().sin_cos();
        // T = translate(ca) · Rz · translate(-cb)
        let rcb = [c * cb[0] - s * cb[1], s * cb[0] + c * cb[1], cb[2]];
        [
            c,
            -s,
            0.0,
            ca[0] - rcb[0],
            s,
            c,
            0.0,
            ca[1] - rcb[1],
            0.0,
            0.0,
            1.0,
            ca[2] - rcb[2],
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    });
    let best = std::iter::once(IDENTITY_4X4)
        .chain(headings)
        .map(|start| icp(tree, a, &coarse, start, COARSE_ITERATIONS))
        .reduce(|x, y| {
            let y_better = y.fitness > x.fitness || (y.fitness == x.fitness && y.rmse_m < x.rmse_m);
            if y_better {
                y
            } else {
                x
            }
        })
        .expect("at least the identity candidate");
    let moving = voxel_representatives(b, ALIGN_VOXEL_M);
    icp(tree, a, &moving, best.transform, MAX_ITERATIONS)
}

/// Mean of the points (origin when empty).
pub(super) fn centroid(points: &[[f64; 3]]) -> [f64; 3] {
    let mut s = [0.0; 3];
    for p in points {
        for k in 0..3 {
            s[k] += p[k];
        }
    }
    s.map(|v| v / points.len().max(1) as f64)
}

/// One input point per occupied `voxel_m` cube: the one nearest the voxel's
/// centroid, ordered by voxel index. Unlike centroids, these lie on the
/// sampled surfaces (a centroid of a voxel cut by two surfaces lies on
/// neither, which biases point-to-point registration).
pub(super) fn voxel_representatives(points: &[[f64; 3]], voxel_m: f64) -> Vec<[f64; 3]> {
    let key = |p: &[f64; 3]| p.map(|v| (v / voxel_m).floor() as i64);
    // Per voxel: coordinate sum, count, then the best squared distance to the
    // centroid and its point index.
    let mut cells: HashMap<[i64; 3], ([f64; 3], u32, f64, usize)> = HashMap::new();
    for (i, p) in points.iter().enumerate() {
        let c = cells
            .entry(key(p))
            .or_insert(([0.0; 3], 0, f64::INFINITY, i));
        for (sum, v) in c.0.iter_mut().zip(p) {
            *sum += v;
        }
        c.1 += 1;
    }
    for (i, p) in points.iter().enumerate() {
        let c = cells
            .get_mut(&key(p))
            .expect("every point's voxel was counted");
        let d2: f64 = (0..3).map(|k| (p[k] - c.0[k] / c.1 as f64).powi(2)).sum();
        if d2 < c.2 {
            (c.2, c.3) = (d2, i);
        }
    }
    let mut picked: Vec<([i64; 3], usize)> = cells.into_iter().map(|(k, c)| (k, c.3)).collect();
    picked.sort_unstable_by_key(|(k, _)| *k);
    picked.into_iter().map(|(_, i)| points[i]).collect()
}

/// Nearest-neighbour index over `points`; `None` when empty.
pub(super) fn kd_tree(points: &[[f64; 3]]) -> Option<ImmutableKdTree<f64, 3>> {
    (!points.is_empty()).then(|| {
        ImmutableKdTree::new_from_slice(points).expect("u32 item index covers the cloud size")
    })
}

/// The result of a registration that could not run.
pub(super) fn unaligned(init: [f64; 16]) -> AlignResult {
    AlignResult {
        transform: init,
        fitness: 0.0,
        rmse_m: f64::INFINITY,
        iterations: 0,
    }
}

/// [`align`] on prepared inputs: `tree` indexes `a`, `moving` is the
/// voxel-thinned `b`.
pub(super) fn icp(
    tree: &ImmutableKdTree<f64, 3>,
    a: &[[f64; 3]],
    moving: &[[f64; 3]],
    init: [f64; 16],
    max_iterations: u32,
) -> AlignResult {
    if moving.is_empty() {
        return unaligned(init);
    }
    let mut t = init;
    let mut iterations = 0;
    let (mut src, mut dst) = (Vec::new(), Vec::new());
    while iterations < max_iterations {
        let pairs = correspond(tree, moving, &t);
        let floor = MAX_PAIR_M * TRIM_DECAY.powi(iterations as i32);
        let (cut, trim) = trim_cutoff(&pairs, floor);
        let mut relaxed = false;
        src.clear();
        dst.clear();
        for (q, i, d) in &pairs {
            if *d <= cut {
                relaxed |= *d > trim;
                src.push(*q);
                dst.push(a[*i]);
            }
        }
        if src.len() < 3 {
            break;
        }
        let step = horn(&src, &dst);
        t = compose(&step, &t);
        iterations += 1;
        let moved = src
            .iter()
            .map(|p| {
                let q = apply(&step, p);
                ((q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) + (q[2] - p[2]).powi(2)).sqrt()
            })
            .sum::<f64>()
            / src.len() as f64;
        if !relaxed && moved < CONVERGED_M {
            break;
        }
    }
    let pairs = correspond(tree, moving, &t);
    let fit = pairs.iter().filter(|p| p.2 <= FIT_RADIUS_M).count();
    let (cut, _) = trim_cutoff(&pairs, 0.0);
    let (used, sq) = pairs
        .iter()
        .filter(|p| p.2 <= cut)
        .fold((0usize, 0.0), |(n, s), p| (n + 1, s + p.2 * p.2));
    AlignResult {
        transform: t,
        fitness: fit as f64 / pairs.len() as f64,
        rmse_m: if used > 0 {
            (sq / used as f64).sqrt()
        } else {
            f64::INFINITY
        },
        iterations,
    }
}
