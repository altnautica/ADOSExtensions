//! The planar LiDAR-camera extrinsic from board observations.
//!
//! Each observation is the board plane `nᵀ p = d` in the camera frame and the
//! LiDAR points that lie on the board (the scan line across it, z = 0 in the
//! LiDAR frame). With `T_cam_lidar = (R, t)` every such point satisfies
//! `nᵀ (R p + t) = d`. Because `p` has no z, the first two columns of `R` and
//! `t` enter linearly: the stacked equations are solved by least squares, the
//! two columns are projected onto the nearest orthonormal pair (SVD) and
//! completed by their cross product, and `t` is re-solved for that rotation
//! (the linear method of Zhang and Pless, 2004). Levenberg-Marquardt then
//! refines all six parameters on the point-to-plane residuals with a Huber
//! loss.

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};

use super::board::{exp_so3, nearest_rotation};
use crate::recon::StageError;

/// Fewest board observations a solve accepts.
pub const MIN_VIEWS: usize = 10;
/// The board normals must spread at least this far about both camera axes
/// across the view (x and y): normals that vary about one axis only leave
/// the translation along the other unobserved.
pub const MIN_TILT_SPREAD_DEG: f64 = 30.0;
/// Huber threshold on the point-to-plane residual, metres.
pub const HUBER_M: f64 = 0.01;
/// A solution with a larger RMS residual fails the job, metres.
pub const MAX_RMS_M: f64 = 0.02;

/// One board view: the plane in the camera frame and the LiDAR-frame points
/// on it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlaneObs {
    /// Unit normal, camera frame.
    pub n: Vector3<f64>,
    pub d: f64,
    /// LiDAR-frame `(x, y)` (z = 0) of the scan points on the board.
    pub points: Vec<[f64; 2]>,
}

/// The solved extrinsic: `p_cam = r · p_lidar + t`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Extrinsic {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    /// RMS point-to-plane residual over every point, metres.
    pub rms_m: f64,
}

/// The circular range covered by `angles` (radians): 2π minus the largest gap.
fn circular_spread(mut angles: Vec<f64>) -> f64 {
    if angles.len() < 2 {
        return 0.0;
    }
    angles.sort_by(f64::total_cmp);
    let tau = std::f64::consts::TAU;
    let wrap_gap = angles[0] + tau - angles[angles.len() - 1];
    let max_gap = angles
        .windows(2)
        .map(|w| w[1] - w[0])
        .fold(wrap_gap, f64::max);
    tau - max_gap
}

/// The spread (degrees) of the board normals about the camera's x axis
/// (tilted up and down) and its y axis (turned left and right).
pub fn tilt_spread_deg(obs: &[PlaneObs]) -> [f64; 2] {
    [1, 0].map(|a| {
        let angles = obs.iter().map(|o| o.n[a].atan2(o.n[2])).collect();
        circular_spread(angles).to_degrees()
    })
}

/// Refuse too few views, or views whose tilts do not span both axes.
pub fn check_views(obs: &[PlaneObs]) -> Result<(), StageError> {
    let varied = tilt_spread_deg(obs)
        .iter()
        .all(|s| *s >= MIN_TILT_SPREAD_DEG);
    if obs.len() < MIN_VIEWS || !varied {
        return Err(StageError::input(format!(
            "need at least {MIN_VIEWS} board views at varied tilt; have {}",
            obs.len()
        )));
    }
    Ok(())
}

/// The point residuals `nᵀ (R p + t) − d`.
fn residuals(obs: &[PlaneObs], r: &Matrix3<f64>, t: &Vector3<f64>) -> Vec<f64> {
    obs.iter()
        .flat_map(|o| {
            o.points
                .iter()
                .map(move |p| o.n.dot(&(r * Vector3::new(p[0], p[1], 0.0) + t)) - o.d)
        })
        .collect()
}

/// `t` minimising the residuals for a fixed `r`.
fn solve_t(obs: &[PlaneObs], r: &Matrix3<f64>) -> Option<Vector3<f64>> {
    let mut ata = Matrix3::zeros();
    let mut atb = Vector3::zeros();
    for o in obs {
        for p in &o.points {
            let rhs = o.d - o.n.dot(&(r * Vector3::new(p[0], p[1], 0.0)));
            ata += o.n * o.n.transpose();
            atb += o.n * rhs;
        }
    }
    ata.try_inverse().map(|inv| inv * atb)
}

/// The linear initialisation.
pub fn linear(obs: &[PlaneObs]) -> Option<(Matrix3<f64>, Vector3<f64>)> {
    let mut ata = SMatrix::<f64, 9, 9>::zeros();
    let mut atb = SVector::<f64, 9>::zeros();
    for o in obs {
        for p in &o.points {
            let row = SVector::<f64, 9>::from_iterator(
                (o.n * p[0])
                    .iter()
                    .chain((o.n * p[1]).iter())
                    .chain(o.n.iter())
                    .copied(),
            );
            ata += row * row.transpose();
            atb += row * o.d;
        }
    }
    let x = ata.try_inverse()? * atb;
    let (c1, c2) = (
        Vector3::new(x[0], x[1], x[2]),
        Vector3::new(x[3], x[4], x[5]),
    );
    let r = nearest_rotation(&Matrix3::from_columns(&[c1, c2, c1.cross(&c2)]));
    Some((r, solve_t(obs, &r)?))
}

/// Huber IRLS weight of a residual.
fn huber(e: f64) -> f64 {
    match e.abs() <= HUBER_M {
        true => 1.0,
        false => HUBER_M / e.abs(),
    }
}

/// Weighted Huber cost.
fn cost(e: &[f64]) -> f64 {
    e.iter()
        .map(|e| match e.abs() <= HUBER_M {
            true => 0.5 * e * e,
            false => HUBER_M * (e.abs() - 0.5 * HUBER_M),
        })
        .sum()
}

/// Levenberg-Marquardt from `(r, t)` on the Huber point-to-plane cost.
pub fn refine(
    obs: &[PlaneObs],
    mut r: Matrix3<f64>,
    mut t: Vector3<f64>,
) -> (Matrix3<f64>, Vector3<f64>) {
    let mut lambda = 1e-3;
    let mut e = residuals(obs, &r, &t);
    for _ in 0..100 {
        let mut jtj = DMatrix::<f64>::zeros(6, 6);
        let mut jte = DVector::<f64>::zeros(6);
        let mut k = 0;
        for o in obs {
            for p in &o.points {
                let rp = r * Vector3::new(p[0], p[1], 0.0);
                // d/dδ of nᵀ(exp(δ) R p + t) at δ = 0 is (R p × n).
                let g = rp.cross(&o.n);
                let row = SVector::<f64, 6>::new(g.x, g.y, g.z, o.n.x, o.n.y, o.n.z);
                let w = huber(e[k]);
                jtj += row * row.transpose() * w;
                jte += row * (w * e[k]);
                k += 1;
            }
        }
        let damped = &jtj + DMatrix::from_diagonal(&(jtj.diagonal() * lambda));
        let Some(delta) = damped.try_inverse().map(|m| m * -&jte) else {
            break;
        };
        let r_new = exp_so3(&Vector3::new(delta[0], delta[1], delta[2])) * r;
        let t_new = t + Vector3::new(delta[3], delta[4], delta[5]);
        let e_new = residuals(obs, &r_new, &t_new);
        if cost(&e_new) < cost(&e) {
            (r, t, e) = (r_new, t_new, e_new);
            lambda = (lambda / 10.0).max(1e-12);
            if delta.norm() < 1e-12 {
                break;
            }
        } else {
            lambda *= 10.0;
            if lambda > 1e8 {
                break;
            }
        }
    }
    (r, t)
}

/// Check the views, initialise linearly and refine. The caller judges the
/// RMS against [`MAX_RMS_M`].
pub fn solve(obs: &[PlaneObs]) -> Result<Extrinsic, StageError> {
    check_views(obs)?;
    let (r0, t0) = linear(obs).ok_or_else(|| {
        StageError::input("the board views do not constrain the LiDAR pose (degenerate geometry)")
    })?;
    let (r, t) = refine(obs, r0, t0);
    let e = residuals(obs, &r, &t);
    let rms_m = (e.iter().map(|v| v * v).sum::<f64>() / e.len().max(1) as f64).sqrt();
    Ok(Extrinsic { r, t, rms_m })
}

#[cfg(test)]
#[path = "lrf_solve_tests.rs"]
mod tests;
