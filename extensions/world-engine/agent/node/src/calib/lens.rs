//! Pixel to normalized-ray conversion for the contract camera models
//! (OpenCV camera frame: x right, y down, z forward).
//!
//! `opencv` is the radial-tangential model (`k1 k2 p1 p2 k3`); `opencv_fisheye`
//! and `kb4` are the equidistant Kannala-Brandt model (`k1..k4` on the angle).
//! Undistortion inverts them iteratively.

use ados_capture_contract::{CameraModel, CameraModelModel};

const ITERATIONS: usize = 30;

/// The normalized image point `(x, y)` (ray `(x, y, 1)`) of pixel `(u, v)`.
pub fn unproject(cam: &CameraModel, u: f64, v: f64) -> [f64; 2] {
    let d = [(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy];
    match cam.model {
        CameraModelModel::Pinhole => d,
        CameraModelModel::Opencv => undistort_radtan(&cam.dist, d),
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => undistort_kb(&cam.dist, d),
    }
}

/// The pixel of camera-frame point `p` (in front of the camera).
pub fn project(cam: &CameraModel, p: [f64; 3]) -> [f64; 2] {
    let n = [p[0] / p[2], p[1] / p[2]];
    let d = match cam.model {
        CameraModelModel::Pinhole => n,
        CameraModelModel::Opencv => distort_radtan(&cam.dist, n),
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => distort_kb(&cam.dist, n),
    };
    [cam.fx * d[0] + cam.cx, cam.fy * d[1] + cam.cy]
}

fn coeff(dist: &[f64], i: usize) -> f64 {
    dist.get(i).copied().unwrap_or(0.0)
}

fn distort_radtan(dist: &[f64], [x, y]: [f64; 2]) -> [f64; 2] {
    let (k1, k2, p1, p2, k3) = (
        coeff(dist, 0),
        coeff(dist, 1),
        coeff(dist, 2),
        coeff(dist, 3),
        coeff(dist, 4),
    );
    let r2 = x * x + y * y;
    let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
    [
        x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
        y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
    ]
}

/// Fixed-point inversion of [`distort_radtan`].
fn undistort_radtan(dist: &[f64], d: [f64; 2]) -> [f64; 2] {
    let mut p = d;
    for _ in 0..ITERATIONS {
        let f = distort_radtan(dist, p);
        p = [p[0] + d[0] - f[0], p[1] + d[1] - f[1]];
    }
    p
}

fn kb_theta_d(dist: &[f64], theta: f64) -> f64 {
    let t2 = theta * theta;
    theta
        * (1.0
            + t2 * (coeff(dist, 0)
                + t2 * (coeff(dist, 1) + t2 * (coeff(dist, 2) + t2 * coeff(dist, 3)))))
}

fn distort_kb(dist: &[f64], [x, y]: [f64; 2]) -> [f64; 2] {
    let r = x.hypot(y);
    if r < 1e-12 {
        return [x, y];
    }
    let s = kb_theta_d(dist, r.atan()) / r;
    [x * s, y * s]
}

/// Newton inversion of the Kannala-Brandt angle polynomial.
fn undistort_kb(dist: &[f64], [xd, yd]: [f64; 2]) -> [f64; 2] {
    let theta_d = xd.hypot(yd);
    if theta_d < 1e-12 {
        return [xd, yd];
    }
    let mut theta = theta_d;
    for _ in 0..ITERATIONS {
        let (f, h) = (kb_theta_d(dist, theta) - theta_d, 1e-7);
        let df = (kb_theta_d(dist, theta + h) - kb_theta_d(dist, theta - h)) / (2.0 * h);
        if df.abs() < 1e-12 {
            break;
        }
        theta -= f / df;
    }
    let s = theta.tan() / theta_d;
    [xd * s, yd * s]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam(model: CameraModelModel, dist: Vec<f64>) -> CameraModel {
        CameraModel {
            model,
            width: 800,
            height: 600,
            fx: 610.0,
            fy: 605.0,
            cx: 401.0,
            cy: 298.0,
            dist,
            calibrated: true,
            readout_ns: 0,
        }
    }

    #[test]
    fn unproject_inverts_project_for_every_model() {
        for c in [
            cam(CameraModelModel::Pinhole, vec![]),
            cam(
                CameraModelModel::Opencv,
                vec![-0.28, 0.07, 0.001, -0.0005, 0.0],
            ),
            cam(CameraModelModel::Kb4, vec![-0.01, 0.02, -0.005, 0.001]),
            cam(
                CameraModelModel::OpencvFisheye,
                vec![0.03, -0.01, 0.002, 0.0],
            ),
        ] {
            for p in [[0.3, -0.2, 1.0], [-0.5, 0.35, 1.2], [0.0, 0.0, 2.0]] {
                let px = project(&c, p);
                let n = unproject(&c, px[0], px[1]);
                assert!(
                    (n[0] - p[0] / p[2]).abs() < 1e-9 && (n[1] - p[1] / p[2]).abs() < 1e-9,
                    "{:?} {p:?} -> {n:?}",
                    c.model
                );
            }
        }
    }
}
