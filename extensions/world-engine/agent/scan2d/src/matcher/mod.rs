//! Scan-to-grid matching.
//!
//! - [`correlative`]: exhaustive search of a pose window on the grid's
//!   nearest-cell probabilities (correlative scan matching, Olson 2009).
//! - [`refine`]: Gauss-Newton on `1 - M(p)` over the bilinearly interpolated
//!   grid (the Hector SLAM formulation, Kohlbrecher et al. 2011).
//! - [`branch_and_bound`]: the same exhaustive answer over a wide window,
//!   pruned with the max-pooled [`Pyramid`] (Hess et al. 2016).
//!
//! Candidate poses are the prior's translation plus whole-cell offsets and its
//! rotation plus whole angular steps of `atan(resolution / max_range)`, so a
//! step moves the farthest point by about one cell.

use nalgebra::{Matrix2, Matrix3, SymmetricEigen, Vector2, Vector3};

use crate::grid::Grid;
use crate::pose::Pose2;
use crate::scan::Scan2d;

mod bnb;

pub use bnb::branch_and_bound;

/// Search half-widths around a prior pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Window {
    pub linear_m: f64,
    pub angular_rad: f64,
}

/// A refined match.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Refined {
    pub pose: Pose2,
    /// Mean interpolated probability of the scan points at `pose`.
    pub score: f64,
    /// Gauss-Newton Hessian `JᵀJ` over `(x, y, θ)`; θ in radians, so its rows
    /// weigh a turn by the arc it sweeps at a 1 m lever.
    pub hessian: Matrix3<f64>,
}

/// Angular search step for `scan` on a grid of resolution `res`.
pub fn angular_step(res: f64, scan: &Scan2d) -> f64 {
    (res / scan.max_range().max(1.0)).atan()
}

/// Rotated scan points for each angle of the window, as cells relative to the
/// prior translation, plus the step.
pub(crate) fn rotated_cells(
    cell_of: impl Fn([f64; 2]) -> (i64, i64),
    scan: &Scan2d,
    prior: Pose2,
    window: Window,
    res: f64,
) -> (f64, i64, Vec<Vec<(i64, i64)>>) {
    let step = angular_step(res, scan);
    let na = (window.angular_rad / step).ceil() as i64;
    let sets = (-na..=na)
        .map(|a| {
            let (s, c) = (prior.th + a as f64 * step).sin_cos();
            scan.points
                .iter()
                .map(|p| {
                    let (px, py) = (p[0] as f64, p[1] as f64);
                    cell_of([c * px - s * py + prior.x, s * px + c * py + prior.y])
                })
                .collect()
        })
        .collect();
    (step, na, sets)
}

/// The best pose in `window` around `prior`, by exhaustive search, and its
/// score (mean cell probability). An empty scan returns the prior, score 0.
pub fn correlative(grid: &Grid, scan: &Scan2d, prior: Pose2, window: Window) -> (Pose2, f64) {
    if scan.points.is_empty() {
        return (prior, 0.0);
    }
    let res = grid.resolution();
    let wl = (window.linear_m / res).ceil() as i64;
    let n = (2 * wl + 1) as usize;
    let (step, na, sets) = rotated_cells(|p| grid.cell_of(p), scan, prior, window, res);
    let p_min = grid.p_min();
    let mut acc = vec![0f32; n * n];
    let mut best = (prior, f64::NEG_INFINITY, i64::MAX);
    for (ai, cells) in sets.iter().enumerate() {
        acc.fill(0.0);
        for &(ix, iy) in cells {
            let x0 = ix - wl;
            for dy in 0..n {
                let out = &mut acc[dy * n..(dy + 1) * n];
                match grid.row(iy - wl + dy as i64) {
                    Some(row) if x0 >= 0 && x0 + n as i64 <= row.len() as i64 => {
                        let src = &row[x0 as usize..x0 as usize + n];
                        for (o, v) in out.iter_mut().zip(src) {
                            *o += *v;
                        }
                    }
                    Some(_) => {
                        for (dx, o) in out.iter_mut().enumerate() {
                            *o += grid.value(x0 + dx as i64, iy - wl + dy as i64);
                        }
                    }
                    None => out.iter_mut().for_each(|o| *o += p_min),
                }
            }
        }
        let a = ai as i64 - na;
        for (k, v) in acc.iter().enumerate() {
            let (dx, dy) = ((k % n) as i64 - wl, (k / n) as i64 - wl);
            let score = *v as f64 / cells.len() as f64;
            // Ties go to the candidate nearest the prior.
            let d = dx * dx + dy * dy + a * a;
            if score > best.1 + 1e-9 || (score > best.1 - 1e-9 && d < best.2) {
                let pose = Pose2::new(
                    prior.x + dx as f64 * res,
                    prior.y + dy as f64 * res,
                    prior.th + a as f64 * step,
                );
                best = (pose, score, d);
            }
        }
    }
    (best.0, best.1)
}

/// A pull towards a predicted pose during refinement: the cost adds
/// `translation_weight²·|t - t_prior|² + rotation_weight²·(θ - θ_prior)²` to
/// the mean squared residual `Σ(1 - M)² / n`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prior {
    pub pose: Pose2,
    pub translation_weight: f64,
    pub rotation_weight: f64,
}

/// Gauss-Newton refinement of `start` on `Σ(1 - M)² / n`, plus the `prior`
/// terms when given; stops after `max_iters` steps, when a step is shorter
/// than `eps`, or when a step would raise the cost.
pub fn refine(
    grid: &Grid,
    scan: &Scan2d,
    start: Pose2,
    prior: Option<&Prior>,
    max_iters: usize,
    eps: f64,
) -> Refined {
    let mut cur = start;
    let mut ne = normal_equations(grid, scan, cur, prior);
    for _ in 0..max_iters {
        let Some(hi) = ne.h.try_inverse() else { break };
        let d = hi * ne.g;
        if !d.iter().all(|v| v.is_finite()) {
            break;
        }
        let next = Pose2::new(cur.x + d[0], cur.y + d[1], cur.th + d[2]);
        let nn = normal_equations(grid, scan, next, prior);
        if nn.cost > ne.cost {
            break;
        }
        (cur, ne) = (next, nn);
        if d.norm() < eps {
            break;
        }
    }
    Refined {
        pose: cur,
        score: ne.score,
        hessian: ne.h_scan,
    }
}

struct Normal {
    /// Normal matrix and right-hand side of the whole cost.
    h: Matrix3<f64>,
    g: Vector3<f64>,
    /// `Σ JJᵀ` of the scan residuals alone.
    h_scan: Matrix3<f64>,
    cost: f64,
    score: f64,
}

/// Normal equations for residuals `r = 1 - M` (each weighted `1/√n`) and the
/// prior terms.
fn normal_equations(grid: &Grid, scan: &Scan2d, pose: Pose2, prior: Option<&Prior>) -> Normal {
    let (s, c) = pose.th.sin_cos();
    let mut h_scan = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let (mut sum, mut sq) = (0.0, 0.0);
    for p in &scan.points {
        let (px, py) = (p[0] as f64, p[1] as f64);
        let w = [c * px - s * py + pose.x, s * px + c * py + pose.y];
        let (m, [gx, gy]) = grid.interpolate(w);
        let j = Vector3::new(gx, gy, gx * (-s * px - c * py) + gy * (c * px - s * py));
        h_scan += j * j.transpose();
        g += j * (1.0 - m);
        sum += m;
        sq += (1.0 - m) * (1.0 - m);
    }
    let n = scan.points.len().max(1) as f64;
    let mut h = h_scan / n;
    g /= n;
    let mut cost = sq / n;
    if let Some(pr) = prior {
        let (wt, wr) = (pr.translation_weight.powi(2), pr.rotation_weight.powi(2));
        let d = Vector3::new(
            pr.pose.x - pose.x,
            pr.pose.y - pose.y,
            crate::pose::wrap(pr.pose.th - pose.th),
        );
        let w = Vector3::new(wt, wt, wr);
        for k in 0..3 {
            h[(k, k)] += w[k];
            g[k] += w[k] * d[k];
            cost += w[k] * d[k] * d[k];
        }
    }
    Normal {
        h,
        g,
        h_scan,
        cost,
        score: sum / n,
    }
}

/// Neighbours either side of a point used to fit its local line.
const NORMAL_NEIGHBOURS: usize = 12;
/// Neighbours farther than this from the point are not on its surface.
const NORMAL_RADIUS_M: f64 = 0.5;
/// Fits whose spread across the line exceeds this fraction of the spread
/// along it are not lines and carry no constraint.
const LINE_RATIO: f64 = 0.25;

/// Point-to-line information `Σ JJᵀ` of `scan` at `pose` over `(x, y, θ)`:
/// each point whose beam-order neighbours fit a line constrains motion along
/// that line's normal. This is the geometry of the scan itself, free of the
/// grid's sampling: a far wall that a few beams hit sparsely still only
/// constrains across the wall.
pub fn constraint_information(scan: &Scan2d, pose: Pose2) -> Matrix3<f64> {
    let (s, c) = pose.th.sin_cos();
    let pts: Vec<Vector2<f64>> = scan
        .points
        .iter()
        .map(|p| {
            Vector2::new(
                c * p[0] as f64 - s * p[1] as f64,
                s * p[0] as f64 + c * p[1] as f64,
            )
        })
        .collect();
    let mut h = Matrix3::zeros();
    for (i, q) in pts.iter().enumerate() {
        let lo = i.saturating_sub(NORMAL_NEIGHBOURS);
        let hi = (i + NORMAL_NEIGHBOURS + 1).min(pts.len());
        let near: Vec<&Vector2<f64>> = pts[lo..hi]
            .iter()
            .filter(|o| (*o - q).norm() <= NORMAL_RADIUS_M)
            .collect();
        if near.len() < 3 {
            continue;
        }
        let mean = near.iter().fold(Vector2::zeros(), |a, o| a + **o) / near.len() as f64;
        let cov = near.iter().fold(Matrix2::zeros(), |a, o| {
            let d = **o - mean;
            a + d * d.transpose()
        });
        let eig = SymmetricEigen::new(cov);
        let (kmin, kmax) = if eig.eigenvalues[0] <= eig.eigenvalues[1] {
            (0, 1)
        } else {
            (1, 0)
        };
        if eig.eigenvalues[kmin] > LINE_RATIO * eig.eigenvalues[kmax] {
            continue;
        }
        let n = eig.eigenvectors.column(kmin);
        let j = Vector3::new(n[0], n[1], n[0] * -q[1] + n[1] * q[0]);
        h += j * j.transpose();
    }
    h
}

/// Information on translation once rotation is solved for: the Schur
/// complement of the θ block of an information matrix over `(x, y, θ)`.
pub fn translation_information(h: &Matrix3<f64>) -> Option<Matrix2<f64>> {
    let htt = Matrix2::new(h[(0, 0)], h[(0, 1)], h[(1, 0)], h[(1, 1)]);
    let hth = Vector2::new(h[(0, 2)], h[(1, 2)]);
    let s = if h[(2, 2)] > 0.0 {
        htt - hth * hth.transpose() / h[(2, 2)]
    } else {
        htt
    };
    s.iter().all(|v| v.is_finite()).then_some(s)
}

/// `eig_min / eig_max` of the translation part of an information matrix
/// over `(x, y, θ)` (rotation marginalised out), 0 when it carries none: near
/// 0 along a featureless corridor, near 1 where walls run in both directions.
/// The rotation is marginalised rather than kept as a third axis because its
/// eigenvalue grows with the square of the point ranges, which would rate a
/// plain room corner as degenerate.
pub fn degeneracy(h: &Matrix3<f64>) -> f64 {
    let Some(s) = translation_information(h) else {
        return 0.0;
    };
    let e = SymmetricEigen::new(s).eigenvalues;
    let max = e.max();
    if max <= 0.0 || !max.is_finite() {
        return 0.0;
    }
    e.min().max(0.0) / max
}

/// `matched`, with its translation offset from `prior` kept only along the
/// directions the match constrains (translation-information eigenvalue at
/// least `ratio` of the largest): along a featureless corridor the pose keeps
/// its motion prior on the corridor axis instead of sliding (solution
/// remapping, Zhang, Kaess and Singh 2016). The rotation is kept as matched.
pub fn keep_constrained(prior: Pose2, matched: Pose2, h: &Matrix3<f64>, ratio: f64) -> Pose2 {
    let Some(s) = translation_information(h) else {
        return prior;
    };
    let eig = SymmetricEigen::new(s);
    let max = eig.eigenvalues.max();
    if max <= 0.0 || !max.is_finite() {
        return Pose2::new(prior.x, prior.y, matched.th);
    }
    let d = Vector2::new(matched.x - prior.x, matched.y - prior.y);
    let mut kept = Vector2::zeros();
    for i in 0..2 {
        if eig.eigenvalues[i] >= ratio * max {
            let v = eig.eigenvectors.column(i);
            kept += v * v.dot(&d);
        }
    }
    Pose2::new(prior.x + kept[0], prior.y + kept[1], matched.th)
}
