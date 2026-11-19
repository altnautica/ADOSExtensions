//! Rigid 2D transforms.

use std::f64::consts::PI;
use std::ops::Mul;

/// A rigid transform in the plane: rotate by `th` (radians, counter-clockwise),
/// then translate by `(x, y)`. `a * b` applies `b` first, then `a`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub th: f64,
}

impl Pose2 {
    pub const IDENTITY: Pose2 = Pose2 {
        x: 0.0,
        y: 0.0,
        th: 0.0,
    };

    pub fn new(x: f64, y: f64, th: f64) -> Self {
        Pose2 { x, y, th: wrap(th) }
    }

    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.th.sin_cos();
        Pose2::new(
            -(c * self.x + s * self.y),
            s * self.x - c * self.y,
            -self.th,
        )
    }

    /// The point `p` (in this pose's frame) expressed in the parent frame.
    pub fn apply(&self, p: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.th.sin_cos();
        [c * p[0] - s * p[1] + self.x, s * p[0] + c * p[1] + self.y]
    }

    /// Distance between the two translations, metres.
    pub fn dist(&self, o: &Pose2) -> f64 {
        (self.x - o.x).hypot(self.y - o.y)
    }
}

impl Mul for Pose2 {
    type Output = Pose2;
    // Composing planar rotations adds their angles.
    #[allow(clippy::suspicious_arithmetic_impl)]
    fn mul(self, b: Pose2) -> Pose2 {
        let [x, y] = self.apply([b.x, b.y]);
        Pose2::new(x, y, self.th + b.th)
    }
}

/// `a` wrapped into `(-π, π]`.
pub fn wrap(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a <= -PI {
        a += 2.0 * PI;
    }
    a
}
