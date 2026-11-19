//! The calibration board's pose in one camera image.
//!
//! The board is the AprilGrid the camera calibrations use ([`AprilGrid`]:
//! `tag_cols x tag_rows` tag36h11 tags of `tag_size_m`, `tag_spacing` gaps,
//! ids row-major from tag 0). Tags are found by the `aprilgrid` detector; each
//! tag contributes its centre (the intersection of its undistorted diagonals),
//! which does not depend on the detector's corner order or on which way the
//! printed rows run. The board pose comes from the centres' homography,
//! refined by Gauss-Newton on their reprojection.

use ados_capture_contract::CameraModel;
use aprilgrid::detector::{DetectorParams, TagDetector};
use aprilgrid::TagFamily;
use image::DynamicImage;
use nalgebra::{Matrix3, SMatrix, SVector, SymmetricEigen, Vector3};

use super::lens::unproject;
use super::AprilGrid;
use crate::recon::session_data::DecodedImage;

/// Fewest decoded tags a pose is computed from.
pub const MIN_TAGS: usize = 6;

/// The board in the camera frame: `p_cam = r · p_board + t`, with the board
/// on its own z = 0 plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoardPose {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    /// Tags the pose was computed from.
    pub tags: usize,
    /// RMS reprojection error of the tag centres, normalized image units.
    pub rms: f64,
}

impl BoardPose {
    /// The board plane `nᵀ p = d` in the camera frame, `n` pointing away from
    /// the camera (`d > 0`).
    pub fn plane(&self) -> (Vector3<f64>, f64) {
        let n = self.r.column(2).into_owned();
        let d = n.dot(&self.t);
        if d < 0.0 {
            (-n, -d)
        } else {
            (n, d)
        }
    }

    /// Board point `(x, y)` in the camera frame.
    pub fn to_camera(&self, xy: [f64; 2]) -> Vector3<f64> {
        self.r * Vector3::new(xy[0], xy[1], 0.0) + self.t
    }
}

/// An AprilGrid board and its detector.
pub struct Board {
    grid: AprilGrid,
    detector: TagDetector,
}

impl Board {
    pub fn new(grid: &AprilGrid) -> Self {
        let params = DetectorParams {
            tag_spacing_ratio: grid.tag_spacing as f32,
            ..DetectorParams::default_params()
        };
        Self {
            grid: grid.clone(),
            detector: TagDetector::new(&TagFamily::T36H11, Some(params)),
        }
    }

    fn pitch(&self) -> f64 {
        self.grid.tag_size_m * (1.0 + self.grid.tag_spacing)
    }

    /// Width and height of the tag area, metres.
    pub fn extent(&self) -> [f64; 2] {
        let gap = self.grid.tag_size_m * self.grid.tag_spacing;
        [
            self.grid.tag_cols as f64 * self.pitch() - gap,
            self.grid.tag_rows as f64 * self.pitch() - gap,
        ]
    }

    /// The board-frame centre of tag `id`.
    pub fn tag_center(&self, id: u32) -> Option<[f64; 2]> {
        let (cols, rows) = (self.grid.tag_cols, self.grid.tag_rows);
        (id < cols * rows).then(|| {
            let half = self.grid.tag_size_m / 2.0;
            [
                (id % cols) as f64 * self.pitch() + half,
                (id / cols) as f64 * self.pitch() + half,
            ]
        })
    }

    /// The board pose in `img`, `None` when fewer than [`MIN_TAGS`] tags
    /// spanning two rows and two columns decode.
    pub fn detect(&self, img: &DecodedImage, cam: &CameraModel) -> Option<BoardPose> {
        let dynimg = match img.channels {
            1 => DynamicImage::ImageLuma8(image::GrayImage::from_raw(
                img.width,
                img.height,
                img.data.clone(),
            )?),
            _ => DynamicImage::ImageRgb8(image::RgbImage::from_raw(
                img.width,
                img.height,
                img.data.clone(),
            )?),
        };
        let mut board = Vec::new();
        let mut rays = Vec::new();
        for (id, corners) in self.detector.detect(&dynimg) {
            let (Some(c), Some(ray)) = (self.tag_center(id), center_ray(cam, &corners)) else {
                continue;
            };
            board.push(c);
            rays.push(ray);
        }
        let distinct = |k: usize| {
            let mut v: Vec<i64> = board.iter().map(|p| (p[k] * 1e3).round() as i64).collect();
            v.sort_unstable();
            v.dedup();
            v.len()
        };
        if board.len() < MIN_TAGS || distinct(0) < 2 || distinct(1) < 2 {
            return None;
        }
        pose_from_points(&board, &rays)
    }
}

/// The normalized ray through a tag's centre: the intersection of its two
/// diagonals after undistortion (`corners` in cyclic order).
fn center_ray(cam: &CameraModel, corners: &[(f32, f32); 4]) -> Option<[f64; 2]> {
    let p: Vec<Vector3<f64>> = corners
        .iter()
        .map(|&(u, v)| {
            let n = unproject(cam, u as f64, v as f64);
            Vector3::new(n[0], n[1], 1.0)
        })
        .collect();
    let x = p[0].cross(&p[2]).cross(&p[1].cross(&p[3]));
    (x.z.abs() > 1e-12).then(|| [x.x / x.z, x.y / x.z])
}

/// Similarity normalisation (centroid to the origin, mean distance √2).
fn normalizer(pts: &[[f64; 2]]) -> Matrix3<f64> {
    let n = pts.len() as f64;
    let (mx, my) = pts
        .iter()
        .fold((0.0, 0.0), |a, p| (a.0 + p[0] / n, a.1 + p[1] / n));
    let mean_d = pts
        .iter()
        .map(|p| (p[0] - mx).hypot(p[1] - my))
        .sum::<f64>()
        / n;
    let s = std::f64::consts::SQRT_2 / mean_d.max(1e-12);
    Matrix3::new(s, 0.0, -s * mx, 0.0, s, -s * my, 0.0, 0.0, 1.0)
}

/// The homography taking board points to normalized image points (DLT).
fn homography(board: &[[f64; 2]], img: &[[f64; 2]]) -> Matrix3<f64> {
    let (tb, ti) = (normalizer(board), normalizer(img));
    let mut ata = SMatrix::<f64, 9, 9>::zeros();
    for (b, m) in board.iter().zip(img) {
        let b = tb * Vector3::new(b[0], b[1], 1.0);
        let m = ti * Vector3::new(m[0], m[1], 1.0);
        let rows = [
            SVector::<f64, 9>::from_column_slice(&[
                b.x,
                b.y,
                1.0,
                0.0,
                0.0,
                0.0,
                -m.x * b.x,
                -m.x * b.y,
                -m.x,
            ]),
            SVector::<f64, 9>::from_column_slice(&[
                0.0,
                0.0,
                0.0,
                b.x,
                b.y,
                1.0,
                -m.y * b.x,
                -m.y * b.y,
                -m.y,
            ]),
        ];
        for r in rows {
            ata += r * r.transpose();
        }
    }
    let eig = SymmetricEigen::new(ata);
    let k = eig.eigenvalues.imin();
    let h = eig.eigenvectors.column(k);
    let hn = Matrix3::new(h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
    ti.try_inverse().unwrap_or_else(Matrix3::identity) * hn * tb
}

/// Nearest rotation (SVD projection, determinant +1).
pub fn nearest_rotation(m: &Matrix3<f64>) -> Matrix3<f64> {
    let svd = m.svd(true, true);
    let (u, vt) = (svd.u.expect("u"), svd.v_t.expect("v_t"));
    let mut r = u * vt;
    if r.determinant() < 0.0 {
        let mut u = u;
        u.column_mut(2).neg_mut();
        r = u * vt;
    }
    r
}

/// Rotation of the axis-angle vector `w` (Rodrigues).
pub fn exp_so3(w: &Vector3<f64>) -> Matrix3<f64> {
    nalgebra::Rotation3::new(*w).into_inner()
}

fn reprojection(
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    board: &[[f64; 2]],
    img: &[[f64; 2]],
) -> Vec<f64> {
    board
        .iter()
        .zip(img)
        .flat_map(|(b, m)| {
            let p = r * Vector3::new(b[0], b[1], 0.0) + t;
            [p.x / p.z - m[0], p.y / p.z - m[1]]
        })
        .collect()
}

/// Board pose from board points and their normalized image points: the
/// homography's decomposition, then Gauss-Newton on the reprojection.
pub fn pose_from_points(board: &[[f64; 2]], img: &[[f64; 2]]) -> Option<BoardPose> {
    let h = homography(board, img);
    let (h1, h2, h3) = (h.column(0), h.column(1), h.column(2));
    let mut s = 2.0 / (h1.norm() + h2.norm());
    if (h3 * s).z < 0.0 {
        s = -s;
    }
    let (r1, r2) = (h1 * s, h2 * s);
    let mut r = nearest_rotation(&Matrix3::from_columns(&[r1, r2, r1.cross(&r2)]));
    let mut t: Vector3<f64> = h3 * s;
    for _ in 0..10 {
        let e0 = reprojection(&r, &t, board, img);
        let mut jac = nalgebra::DMatrix::<f64>::zeros(e0.len(), 6);
        let step = 1e-7;
        for k in 0..6 {
            let mut d = SVector::<f64, 6>::zeros();
            d[k] = step;
            let (rk, tk) = perturb(&r, &t, &d);
            for (i, e) in reprojection(&rk, &tk, board, img).iter().enumerate() {
                jac[(i, k)] = (e - e0[i]) / step;
            }
        }
        let e = nalgebra::DVector::from_vec(e0);
        let delta = (jac.transpose() * &jac).try_inverse()? * (jac.transpose() * -e);
        let d = SVector::<f64, 6>::from_iterator(delta.iter().copied());
        (r, t) = perturb(&r, &t, &d);
        if d.norm() < 1e-12 {
            break;
        }
    }
    let e = reprojection(&r, &t, board, img);
    let rms = (e.iter().map(|v| v * v).sum::<f64>() / board.len() as f64).sqrt();
    (t.z > 0.0 && rms.is_finite()).then_some(BoardPose {
        r,
        t,
        tags: board.len(),
        rms,
    })
}

fn perturb(
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    d: &SVector<f64, 6>,
) -> (Matrix3<f64>, Vector3<f64>) {
    (
        exp_so3(&Vector3::new(d[0], d[1], d[2])) * r,
        t + Vector3::new(d[3], d[4], d[5]),
    )
}

#[cfg(test)]
#[path = "board_tests.rs"]
mod tests;
