//! Row-major 4x4 rigid transforms (`T_<to>_<from>`, metres) as the rig stores
//! them, plus the `[x y z qx qy qz qw]` form the LiDAR-camera tool uses.

/// Row-major 4x4 homogeneous transform.
pub type Mat4 = [f64; 16];

/// The identity transform.
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

/// Inverse of a rigid transform: `[Rᵀ | -Rᵀ t]`.
pub fn inv_rigid(t: &Mat4) -> Mat4 {
    let mut out = IDENTITY;
    for r in 0..3 {
        for c in 0..3 {
            out[r * 4 + c] = t[c * 4 + r];
        }
    }
    for r in 0..3 {
        out[r * 4 + 3] = -(0..3).map(|k| out[r * 4 + k] * t[k * 4 + 3]).sum::<f64>();
    }
    out
}

/// The transform keeping only the translation of `t` (identity rotation).
pub fn translation_only(t: &Mat4) -> Mat4 {
    let mut out = IDENTITY;
    out[3] = t[3];
    out[7] = t[7];
    out[11] = t[11];
    out
}

/// `[x y z qx qy qz qw]` (Hamilton, unit quaternion) to a transform.
pub fn from_xyz_quat(v: &[f64; 7]) -> Mat4 {
    let [x, y, z, qx, qy, qz, qw] = *v;
    let n = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    let (qx, qy, qz, qw) = (qx / n, qy / n, qz / n, qw / n);
    [
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw),
        x,
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw),
        y,
        2.0 * (qx * qz - qy * qw),
        2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
        z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// A transform to `[x y z qx qy qz qw]` with `qw >= 0`.
pub fn to_xyz_quat(t: &Mat4) -> [f64; 7] {
    let (m00, m01, m02) = (t[0], t[1], t[2]);
    let (m10, m11, m12) = (t[4], t[5], t[6]);
    let (m20, m21, m22) = (t[8], t[9], t[10]);
    let trace = m00 + m11 + m22;
    let (qx, qy, qz, qw) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    };
    let sign = if qw < 0.0 { -1.0 } else { 1.0 };
    [
        t[3],
        t[7],
        t[11],
        sign * qx,
        sign * qy,
        sign * qz,
        sign * qw,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &[f64], b: &[f64]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12)
    }

    #[test]
    fn inverse_and_quaternion_round_trip() {
        let t = from_xyz_quat(&[0.1, -0.2, 0.3, 0.1, 0.7, 0.05, 0.7]);
        assert!(close(&mul(&t, &inv_rigid(&t)), &IDENTITY));
        let back = from_xyz_quat(&to_xyz_quat(&t));
        assert!(close(&back, &t));
        // A 180 degree turn about x exercises the non-trace branch.
        let flip = [
            1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        assert!(close(&from_xyz_quat(&to_xyz_quat(&flip)), &flip));
    }
}
