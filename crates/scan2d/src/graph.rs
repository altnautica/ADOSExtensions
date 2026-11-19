//! Pose graph over submap poses, solved with Levenberg-Marquardt. The first
//! pose is held fixed; loop edges carry a Huber loss.

use nalgebra::{DMatrix, DVector, Matrix2, Matrix2x3, Vector2};

use crate::pose::{wrap, Pose2};

/// Relative-pose measurement `z ≈ poses[i]⁻¹ · poses[j]`. The translation
/// residual is taken in frame `i`, weighted by `whiten` (`Lᵀ` of the
/// translation information `L·Lᵀ`); the rotation residual by `1/sigma_rad`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub i: usize,
    pub j: usize,
    pub z: Pose2,
    pub whiten: Matrix2<f64>,
    pub sigma_rad: f64,
    /// Huber threshold in whitened units, or `None` for a plain square.
    pub huber: Option<f64>,
}

impl Edge {
    /// Translation information `info` (frame `i`, positive definite).
    pub fn with_information(
        i: usize,
        j: usize,
        z: Pose2,
        info: Matrix2<f64>,
        sigma_rad: f64,
        huber: Option<f64>,
    ) -> Edge {
        let whiten = info.cholesky().map_or_else(
            || Matrix2::from_diagonal(&info.diagonal().map(|v| v.max(0.0).sqrt())),
            |c| c.l().transpose(),
        );
        Edge {
            i,
            j,
            z,
            whiten,
            sigma_rad,
            huber,
        }
    }
}

const MAX_ITERS: usize = 50;

/// Whitened residual and its Jacobians with respect to pose `i` and `j`.
fn linearise(e: &Edge, pi: &Pose2, pj: &Pose2) -> ([f64; 3], [[f64; 3]; 3], [[f64; 3]; 3]) {
    let (si, ci) = pi.th.sin_cos();
    let dt = Vector2::new(pj.x - pi.x, pj.y - pi.y);
    // R_i^T and its derivative.
    let rit = Matrix2::new(ci, si, -si, ci);
    let drit = Matrix2::new(-si, ci, -ci, -si);
    let et = e.whiten * (rit * dt - Vector2::new(e.z.x, e.z.y));
    let eth = wrap(pj.th - pi.th - e.z.th);
    let a = e.whiten * rit;
    let dth = e.whiten * drit * dt;
    let mut ji = Matrix2x3::zeros();
    ji.fixed_view_mut::<2, 2>(0, 0).copy_from(&(-a));
    ji.set_column(2, &dth);
    let mut jj = Matrix2x3::zeros();
    jj.fixed_view_mut::<2, 2>(0, 0).copy_from(&a);
    let wr = 1.0 / e.sigma_rad;
    let r = [et[0], et[1], eth * wr];
    let row = |m: &Matrix2x3<f64>, k: usize| [m[(k, 0)], m[(k, 1)], m[(k, 2)]];
    let ji = [row(&ji, 0), row(&ji, 1), [0.0, 0.0, -wr]];
    let jj = [row(&jj, 0), row(&jj, 1), [0.0, 0.0, wr]];
    (r, ji, jj)
}

/// IRLS weight and robust cost of a whitened residual.
fn robust(r: &[f64; 3], huber: Option<f64>) -> (f64, f64) {
    let n2 = r.iter().map(|v| v * v).sum::<f64>();
    match huber {
        Some(d) if n2.sqrt() > d => {
            let n = n2.sqrt();
            (d / n, d * (n - 0.5 * d))
        }
        _ => (1.0, 0.5 * n2),
    }
}

fn cost(poses: &[Pose2], edges: &[Edge]) -> f64 {
    edges
        .iter()
        .map(|e| robust(&linearise(e, &poses[e.i], &poses[e.j]).0, e.huber).1)
        .sum()
}

/// Minimise the edge costs over `poses[1..]`.
pub fn solve(poses: &mut [Pose2], edges: &[Edge]) {
    let n = poses.len();
    if n < 2 || edges.is_empty() {
        return;
    }
    let dim = 3 * (n - 1);
    let mut lambda = 1e-4;
    let mut cur = cost(poses, edges);
    for _ in 0..MAX_ITERS {
        let mut h = DMatrix::<f64>::zeros(dim, dim);
        let mut b = DVector::<f64>::zeros(dim);
        for e in edges {
            let (r, ji, jj) = linearise(e, &poses[e.i], &poses[e.j]);
            let (w, _) = robust(&r, e.huber);
            let blocks = [(e.i, ji), (e.j, jj)];
            for &(a, ja) in &blocks {
                if a == 0 {
                    continue;
                }
                let oa = 3 * (a - 1);
                for p in 0..3 {
                    b[oa + p] += w * (0..3).map(|k| ja[k][p] * r[k]).sum::<f64>();
                }
                for &(c, jc) in &blocks {
                    if c == 0 {
                        continue;
                    }
                    let oc = 3 * (c - 1);
                    for p in 0..3 {
                        for q in 0..3 {
                            let mut s = 0.0;
                            for k in 0..3 {
                                s += ja[k][p] * jc[k][q];
                            }
                            h[(oa + p, oc + q)] += w * s;
                        }
                    }
                }
            }
        }
        let mut improved = false;
        while lambda < 1e10 {
            let mut a = h.clone();
            for d in 0..dim {
                a[(d, d)] += lambda * h[(d, d)].max(1e-9);
            }
            let Some(chol) = a.cholesky() else {
                lambda *= 10.0;
                continue;
            };
            let step = chol.solve(&(-&b));
            let trial: Vec<Pose2> = poses
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    if k == 0 {
                        return *p;
                    }
                    let o = 3 * (k - 1);
                    Pose2::new(p.x + step[o], p.y + step[o + 1], p.th + step[o + 2])
                })
                .collect();
            let c = cost(&trial, edges);
            if c < cur {
                let gain = cur - c;
                poses.copy_from_slice(&trial);
                cur = c;
                lambda = (lambda / 10.0).max(1e-9);
                improved = gain > 1e-9 * cur.max(1e-9);
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
}
