//! Rigid-transform helpers for the pipeline: row-major 4x4 `T_<to>_<from>`
//! matrices (the contract convention) and unit quaternions `[w, x, y, z]`.

/// Row-major 4x4 transform.
pub type Mat4 = [f64; 16];

pub const IDENTITY: Mat4 = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// `a · b`.
pub fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = (0..4).map(|k| a[r * 4 + k] * b[k * 4 + c]).sum();
        }
    }
    out
}

/// Inverse of a rigid transform (rotation + translation).
pub fn inverse_rigid(t: &Mat4) -> Mat4 {
    let r = [t[0], t[1], t[2], t[4], t[5], t[6], t[8], t[9], t[10]];
    let p = [t[3], t[7], t[11]];
    // R^T and -R^T p.
    let rt = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]];
    let q = [
        -(rt[0] * p[0] + rt[1] * p[1] + rt[2] * p[2]),
        -(rt[3] * p[0] + rt[4] * p[1] + rt[5] * p[2]),
        -(rt[6] * p[0] + rt[7] * p[1] + rt[8] * p[2]),
    ];
    [
        rt[0], rt[1], rt[2], q[0], rt[3], rt[4], rt[5], q[1], rt[6], rt[7], rt[8], q[2], 0.0, 0.0,
        0.0, 1.0,
    ]
}

/// Apply `t` to a point.
pub fn apply(t: &Mat4, p: [f64; 3]) -> [f64; 3] {
    [
        t[0] * p[0] + t[1] * p[1] + t[2] * p[2] + t[3],
        t[4] * p[0] + t[5] * p[1] + t[6] * p[2] + t[7],
        t[8] * p[0] + t[9] * p[1] + t[10] * p[2] + t[11],
    ]
}

/// The translation of `t`.
pub fn translation(t: &Mat4) -> [f64; 3] {
    [t[3], t[7], t[11]]
}

/// Build a transform from a unit quaternion `[w, x, y, z]` and a translation.
pub fn from_quat_trans(q: [f64; 4], p: [f64; 3]) -> Mat4 {
    let [w, x, y, z] = normalize(q);
    [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        p[0],
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        p[1],
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
        p[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// The rotation of `t` as a unit quaternion `[w, x, y, z]` with `w >= 0`.
pub fn quat_of(t: &Mat4) -> [f64; 4] {
    let (m00, m01, m02) = (t[0], t[1], t[2]);
    let (m10, m11, m12) = (t[4], t[5], t[6]);
    let (m20, m21, m22) = (t[8], t[9], t[10]);
    let tr = m00 + m11 + m22;
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    };
    let q = normalize(q);
    if q[0] < 0.0 {
        [-q[0], -q[1], -q[2], -q[3]]
    } else {
        q
    }
}

fn normalize(q: [f64; 4]) -> [f64; 4] {
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    if n <= f64::EPSILON {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

/// Spherical interpolation between unit quaternions, `u` in `[0, 1]`.
pub fn slerp(a: [f64; 4], b: [f64; 4], u: f64) -> [f64; 4] {
    let mut b = b;
    let mut dot: f64 = a.iter().zip(&b).map(|(x, y)| x * y).sum();
    if dot < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
        dot = -dot;
    }
    if dot > 0.9995 {
        return normalize([
            a[0] + u * (b[0] - a[0]),
            a[1] + u * (b[1] - a[1]),
            a[2] + u * (b[2] - a[2]),
            a[3] + u * (b[3] - a[3]),
        ]);
    }
    let theta = dot.clamp(-1.0, 1.0).acos();
    let s = theta.sin();
    let wa = ((1.0 - u) * theta).sin() / s;
    let wb = (u * theta).sin() / s;
    normalize([
        wa * a[0] + wb * b[0],
        wa * a[1] + wb * b[1],
        wa * a[2] + wb * b[2],
        wa * a[3] + wb * b[3],
    ])
}

/// Interpolate between two rigid transforms (slerp rotation, lerp
/// translation), `u` in `[0, 1]`.
pub fn interpolate(a: &Mat4, b: &Mat4, u: f64) -> Mat4 {
    let q = slerp(quat_of(a), quat_of(b), u);
    let (pa, pb) = (translation(a), translation(b));
    from_quat_trans(
        q,
        [
            pa[0] + u * (pb[0] - pa[0]),
            pa[1] + u * (pb[1] - pa[1]),
            pa[2] + u * (pb[2] - pa[2]),
        ],
    )
}

/// The rotation angle between two transforms' rotations, radians.
pub fn rotation_angle(a: &Mat4, b: &Mat4) -> f64 {
    let (qa, qb) = (quat_of(a), quat_of(b));
    let dot: f64 = qa.iter().zip(&qb).map(|(x, y)| x * y).sum::<f64>().abs();
    2.0 * dot.clamp(0.0, 1.0).acos()
}

/// Euclidean distance.
pub fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &[f64], b: &[f64]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn quaternion_round_trips_and_inverse_undoes() {
        let q = normalize([0.9, 0.1, -0.3, 0.2]);
        let t = from_quat_trans(q, [1.0, -2.0, 3.0]);
        assert!(close(&quat_of(&t), &q));
        assert!(close(&mul(&t, &inverse_rigid(&t)), &IDENTITY));
        let p = apply(&t, [0.5, 0.25, -1.0]);
        assert!(close(&apply(&inverse_rigid(&t), p), &[0.5, 0.25, -1.0]));
    }

    #[test]
    fn interpolation_is_halfway_in_angle_and_position() {
        let a = from_quat_trans([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let half = std::f64::consts::FRAC_PI_4;
        let b = from_quat_trans([half.cos(), 0.0, 0.0, half.sin()], [2.0, 0.0, 0.0]);
        let m = interpolate(&a, &b, 0.5);
        assert!((rotation_angle(&a, &m) - std::f64::consts::FRAC_PI_4).abs() < 1e-9);
        assert!(close(&translation(&m), &[1.0, 0.0, 0.0]));
    }
}
