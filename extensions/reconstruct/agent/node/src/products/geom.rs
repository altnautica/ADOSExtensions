//! Small geometry helpers shared by the indoor products: a deterministic RNG,
//! horizontal-plane RANSAC, 2-D line fitting and polygon utilities.

use std::collections::HashMap;

/// SplitMix64: deterministic, seedable, good enough for RANSAC sampling.
pub(crate) struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform index in `0..n` (`n > 0`).
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

/// Plane `n·p + d = 0` with a unit normal pointing up (`n.z > 0`).
#[derive(Debug, Clone, Copy)]
pub(crate) struct Plane {
    pub n: [f64; 3],
    pub d: f64,
}

impl Plane {
    pub fn horizontal(z: f64) -> Self {
        Self {
            n: [0.0, 0.0, 1.0],
            d: -z,
        }
    }

    /// Signed height of `p` above the plane.
    pub fn height(&self, p: [f64; 3]) -> f64 {
        self.n[0] * p[0] + self.n[1] * p[1] + self.n[2] * p[2] + self.d
    }

    /// Plane z at `(x, y)`.
    pub fn z_at(&self, x: f64, y: f64) -> f64 {
        -(self.n[0] * x + self.n[1] * y + self.d) / self.n[2]
    }

    fn from_points(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<Self> {
        let n = cross3(sub3(b, a), sub3(c, a));
        let len = norm3(n);
        if len < 1e-9 {
            return None;
        }
        let s = if n[2] < 0.0 { -1.0 / len } else { 1.0 / len };
        let n = [n[0] * s, n[1] * s, n[2] * s];
        Some(Self {
            n,
            d: -(n[0] * a[0] + n[1] * a[1] + n[2] * a[2]),
        })
    }

    /// Least-squares `z = a x + b y + c` fit; `None` when degenerate.
    fn fit(points: &[[f64; 3]]) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let k = points.len() as f64;
        let (mx, my, mz) = points.iter().fold((0.0, 0.0, 0.0), |s, p| {
            (s.0 + p[0] / k, s.1 + p[1] / k, s.2 + p[2] / k)
        });
        let (mut sxx, mut sxy, mut syy, mut sxz, mut syz) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for p in points {
            let (x, y, z) = (p[0] - mx, p[1] - my, p[2] - mz);
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sxz += x * z;
            syz += y * z;
        }
        let det = sxx * syy - sxy * sxy;
        if det.abs() < 1e-12 {
            return None;
        }
        let a = (sxz * syy - syz * sxy) / det;
        let b = (syz * sxx - sxz * sxy) / det;
        let len = (a * a + b * b + 1.0).sqrt();
        let n = [-a / len, -b / len, 1.0 / len];
        Some(Self {
            n,
            d: -(n[0] * mx + n[1] * my + n[2] * mz),
        })
    }
}

/// RANSAC for the dominant near-horizontal plane (normal within ~15° of Z),
/// refined by least squares on its inliers.
pub(crate) fn ransac_horizontal_plane(
    points: &[[f64; 3]],
    tol: f64,
    iters: usize,
    rng: &mut Rng,
) -> Option<(Plane, usize)> {
    if points.len() < 3 {
        return None;
    }
    let count = |pl: &Plane| points.iter().filter(|p| pl.height(**p).abs() < tol).count();
    let mut best: Option<(Plane, usize)> = None;
    for _ in 0..iters {
        let a = points[rng.below(points.len())];
        let b = points[rng.below(points.len())];
        let c = points[rng.below(points.len())];
        let Some(pl) = Plane::from_points(a, b, c) else {
            continue;
        };
        if pl.n[2] < 0.966 {
            continue;
        }
        let k = count(&pl);
        if best.is_none_or(|(_, bk)| k > bk) {
            best = Some((pl, k));
        }
    }
    let (pl, _) = best?;
    let inliers: Vec<[f64; 3]> = points
        .iter()
        .copied()
        .filter(|p| pl.height(*p).abs() < tol)
        .collect();
    let refined = Plane::fit(&inliers)
        .filter(|r| r.n[2] >= 0.966)
        .unwrap_or(pl);
    Some((refined, count(&refined)))
}

/// Total-least-squares line through `pts`: (direction angle in `[0, π)`, offset
/// along the normal `(-sin, cos)`).
pub(crate) fn fit_line(pts: &[[f64; 2]]) -> Option<(f64, f64)> {
    if pts.len() < 2 {
        return None;
    }
    let k = pts.len() as f64;
    let (mx, my) = pts
        .iter()
        .fold((0.0, 0.0), |s, p| (s.0 + p[0] / k, s.1 + p[1] / k));
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in pts {
        let (x, y) = (p[0] - mx, p[1] - my);
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    let angle = norm_angle(0.5 * (2.0 * sxy).atan2(sxx - syy));
    let n = normal(angle);
    Some((angle, n[0] * mx + n[1] * my))
}

/// Angle folded into `[0, π)`.
pub(crate) fn norm_angle(a: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let r = a.rem_euclid(pi);
    if r >= pi - 1e-12 {
        0.0
    } else {
        r
    }
}

/// Smallest difference between two undirected line angles, in `[0, π/2]`.
pub(crate) fn angle_diff(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::PI);
    d.min(std::f64::consts::PI - d)
}

pub(crate) fn dir(angle: f64) -> [f64; 2] {
    [angle.cos(), angle.sin()]
}

pub(crate) fn normal(angle: f64) -> [f64; 2] {
    [-angle.sin(), angle.cos()]
}

pub(crate) fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub(crate) fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub(crate) fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

pub(crate) fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn norm3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

/// Mean of the points falling into each `cell`-sized square, in a stable
/// (sorted cell) order.
pub(crate) fn dedupe2d(pts: impl Iterator<Item = [f64; 2]>, cell: f64) -> Vec<[f64; 2]> {
    let mut acc: HashMap<(i64, i64), (f64, f64, u32)> = HashMap::new();
    for p in pts {
        let k = ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
        let e = acc.entry(k).or_insert((0.0, 0.0, 0));
        e.0 += p[0];
        e.1 += p[1];
        e.2 += 1;
    }
    let mut cells: Vec<_> = acc.into_iter().collect();
    cells.sort_unstable_by_key(|(k, _)| *k);
    cells
        .into_iter()
        .map(|(_, (x, y, n))| [x / n as f64, y / n as f64])
        .collect()
}

/// Signed shoelace area (positive for counter-clockwise rings).
pub(crate) fn signed_area(poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|i| cross2(poly[i], poly[(i + 1) % n]))
        .sum::<f64>()
        * 0.5
}

/// Area centroid of a simple polygon (vertex mean when degenerate).
pub(crate) fn centroid(poly: &[[f64; 2]]) -> [f64; 2] {
    let a = signed_area(poly);
    let n = poly.len();
    if a.abs() < 1e-12 {
        let k = n.max(1) as f64;
        return poly
            .iter()
            .fold([0.0, 0.0], |s, p| [s[0] + p[0] / k, s[1] + p[1] / k]);
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let (p, q) = (poly[i], poly[(i + 1) % n]);
        let c = cross2(p, q);
        cx += (p[0] + q[0]) * c;
        cy += (p[1] + q[1]) * c;
    }
    [cx / (6.0 * a), cy / (6.0 * a)]
}

/// Even-odd point-in-polygon test.
pub(crate) fn point_in_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    let mut inside = false;
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0];
            if p[0] < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Douglas–Peucker simplification of a closed ring.
pub(crate) fn simplify_ring(ring: &[[f64; 2]], tol: f64) -> Vec<[f64; 2]> {
    let n = ring.len();
    if n <= 4 {
        return ring.to_vec();
    }
    // Split the ring at vertex 0 and the vertex farthest from it.
    let far = (1..n)
        .max_by(|&a, &b| dist2(ring[0], ring[a]).total_cmp(&dist2(ring[0], ring[b])))
        .unwrap_or(n / 2);
    let mut first: Vec<[f64; 2]> = ring[..=far].to_vec();
    let mut second: Vec<[f64; 2]> = ring[far..].to_vec();
    second.push(ring[0]);
    first = simplify_open(&first, tol);
    second = simplify_open(&second, tol);
    first.pop();
    second.pop();
    first.extend(second);
    first
}

fn simplify_open(pts: &[[f64; 2]], tol: f64) -> Vec<[f64; 2]> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((s, e)) = stack.pop() {
        let (a, b) = (pts[s], pts[e]);
        let ab = [b[0] - a[0], b[1] - a[1]];
        let len = dot2(ab, ab).sqrt();
        let mut best = (0.0, s);
        for (i, p) in pts.iter().enumerate().take(e).skip(s + 1) {
            let ap = [p[0] - a[0], p[1] - a[1]];
            let d = if len < 1e-12 {
                dot2(ap, ap).sqrt()
            } else {
                cross2(ab, ap).abs() / len
            };
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > tol {
            keep[best.1] = true;
            stack.push((s, best.1));
            stack.push((best.1, e));
        }
    }
    pts.iter()
        .zip(keep)
        .filter_map(|(p, k)| k.then_some(*p))
        .collect()
}
