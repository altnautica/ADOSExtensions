//! Rigid-transform helpers: row-major rotation matrices, unit quaternions
//! `[w, x, y, z]`, and SE(3) interpolation (quaternion slerp for rotation,
//! linear for translation).

/// Row-major 3x3 identity.
pub const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

/// Unit quaternion `[w, x, y, z]` of a row-major rotation matrix (Shepperd's
/// method, numerically stable for every rotation).
pub fn quat_from_matrix(r: &[f64; 9]) -> [f64; 4] {
    let (m00, m01, m02) = (r[0], r[1], r[2]);
    let (m10, m11, m12) = (r[3], r[4], r[5]);
    let (m20, m21, m22) = (r[6], r[7], r[8]);
    let trace = m00 + m11 + m22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
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
    normalize(q)
}

/// Row-major rotation matrix of a quaternion `[w, x, y, z]` (normalised first).
pub fn matrix_from_quat(q: [f64; 4]) -> [f64; 9] {
    let [w, x, y, z] = normalize(q);
    [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    ]
}

fn normalize(q: [f64; 4]) -> [f64; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n == 0.0 || !n.is_finite() {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// Spherical linear interpolation along the shorter arc, `s` in `0..=1`.
pub fn slerp(a: [f64; 4], b: [f64; 4], s: f64) -> [f64; 4] {
    let a = normalize(a);
    let mut b = normalize(b);
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if dot < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
        dot = -dot;
    }
    if dot > 0.9995 {
        // Nearly parallel: normalised lerp is exact to float precision.
        return normalize([
            a[0] + s * (b[0] - a[0]),
            a[1] + s * (b[1] - a[1]),
            a[2] + s * (b[2] - a[2]),
            a[3] + s * (b[3] - a[3]),
        ]);
    }
    let theta = dot.clamp(-1.0, 1.0).acos();
    let sin = theta.sin();
    let wa = ((1.0 - s) * theta).sin() / sin;
    let wb = (s * theta).sin() / sin;
    [
        wa * a[0] + wb * b[0],
        wa * a[1] + wb * b[1],
        wa * a[2] + wb * b[2],
        wa * a[3] + wb * b[3],
    ]
}

/// Product of two row-major 3x3 matrices, `a * b`.
pub fn mat_mul(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut out = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            out[i * 3 + j] = (0..3).map(|k| a[i * 3 + k] * b[k * 3 + j]).sum();
        }
    }
    out
}

/// `R * v`.
pub fn mat_vec(r: &[f64; 9], v: [f64; 3]) -> [f64; 3] {
    [
        r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
        r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
        r[6] * v[0] + r[7] * v[1] + r[8] * v[2],
    ]
}

/// Transpose of a row-major 3x3 (the inverse of a rotation).
pub fn transpose(r: &[f64; 9]) -> [f64; 9] {
    [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
}

/// Angle of the relative rotation between two rotations, radians.
pub fn rotation_angle_between(a: &[f64; 9], b: &[f64; 9]) -> f64 {
    let rel = mat_mul(&transpose(a), b);
    let c = ((rel[0] + rel[4] + rel[8] - 1.0) / 2.0).clamp(-1.0, 1.0);
    c.acos()
}

/// Row-major 4x4 `T` of a rotation and a translation.
pub fn to_mat4(r: &[f64; 9], t: [f64; 3]) -> [f64; 16] {
    [
        r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Rotation and translation of a row-major 4x4 rigid transform.
pub fn from_mat4(m: &[f64; 16]) -> ([f64; 9], [f64; 3]) {
    (
        [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]],
        [m[3], m[7], m[11]],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rz(a: f64) -> [f64; 9] {
        let (s, c) = a.sin_cos();
        [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
    }

    #[test]
    fn quaternion_round_trips_every_branch() {
        let cases = [
            IDENTITY,
            rz(3.0),
            [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
        ];
        for r in cases {
            let back = matrix_from_quat(quat_from_matrix(&r));
            for (a, b) in r.iter().zip(back.iter()) {
                assert!((a - b).abs() < 1e-12, "{r:?} -> {back:?}");
            }
        }
    }

    #[test]
    fn slerp_halfway_is_half_the_angle() {
        let a = quat_from_matrix(&rz(0.0));
        let b = quat_from_matrix(&rz(1.0));
        let mid = matrix_from_quat(slerp(a, b, 0.5));
        assert!(rotation_angle_between(&mid, &rz(0.5)) < 1e-12);
        // The shorter arc is taken across the ±π seam.
        let c = quat_from_matrix(&rz(3.1));
        let d = quat_from_matrix(&rz(-3.1));
        let m = matrix_from_quat(slerp(c, d, 0.5));
        assert!(rotation_angle_between(&m, &rz(std::f64::consts::PI)) < 1e-9);
    }

    #[test]
    fn mat4_round_trips() {
        let (r, t) = from_mat4(&to_mat4(&rz(0.3), [1.0, 2.0, 3.0]));
        assert_eq!(t, [1.0, 2.0, 3.0]);
        assert!(rotation_angle_between(&r, &rz(0.3)) < 1e-15);
    }
}
