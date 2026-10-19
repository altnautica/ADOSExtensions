//! Distance measurement on the reconstructed mesh: a bounding-volume
//! hierarchy over the triangles for nearest-surface and ray queries.

use serde::{Deserialize, Serialize};

use super::cloud::TriMesh;
use super::geom::{add3, cross3, dot3, norm3, scale3, sub3};

/// Snap radius of point-mode measurement.
pub const SNAP_RADIUS_M: f64 = 0.25;

const LEAF: usize = 4;

#[derive(Clone, Copy)]
struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

impl Aabb {
    fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }

    fn grow(&mut self, p: [f64; 3]) {
        for (k, v) in p.into_iter().enumerate() {
            self.min[k] = self.min[k].min(v);
            self.max[k] = self.max[k].max(v);
        }
    }

    fn union(&mut self, o: &Aabb) {
        self.grow(o.min);
        self.grow(o.max);
    }

    /// Squared distance from `p` to the box (0 inside).
    fn dist2(&self, p: [f64; 3]) -> f64 {
        (0..3)
            .map(|k| {
                let d = (self.min[k] - p[k]).max(0.0).max(p[k] - self.max[k]);
                d * d
            })
            .sum()
    }

    /// Slab test; entry distance when the ray hits within `t_max`.
    fn ray(&self, o: [f64; 3], inv: [f64; 3], t_max: f64) -> Option<f64> {
        let (mut t0, mut t1) = (0.0f64, t_max);
        for k in 0..3 {
            let (a, b) = ((self.min[k] - o[k]) * inv[k], (self.max[k] - o[k]) * inv[k]);
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            // NaN (0 * inf on a slab face) leaves the bounds unchanged.
            t0 = t0.max(a);
            t1 = t1.min(b);
            if t0 > t1 {
                return None;
            }
        }
        Some(t0)
    }
}

enum Node {
    Leaf { bb: Aabb, start: usize, end: usize },
    Inner { bb: Aabb, left: usize, right: usize },
}

impl Node {
    fn bb(&self) -> &Aabb {
        match self {
            Node::Leaf { bb, .. } | Node::Inner { bb, .. } => bb,
        }
    }
}

/// Spatial index over a triangle mesh.
pub struct MeshIndex {
    tris: Vec<[[f64; 3]; 3]>,
    nodes: Vec<Node>,
}

impl MeshIndex {
    pub fn new(mesh: &TriMesh) -> Self {
        let mut tris: Vec<[[f64; 3]; 3]> = mesh
            .triangles
            .iter()
            .filter_map(|t| {
                Some([
                    *mesh.vertices.get(t[0] as usize)?,
                    *mesh.vertices.get(t[1] as usize)?,
                    *mesh.vertices.get(t[2] as usize)?,
                ])
            })
            .collect();
        let mut nodes = Vec::new();
        if !tris.is_empty() {
            let n = tris.len();
            build(&mut tris, 0, n, &mut nodes);
        }
        Self { tris, nodes }
    }

    /// Closest surface point to `p` within `max_dist`.
    pub fn nearest(&self, p: [f64; 3], max_dist: f64) -> Option<[f64; 3]> {
        let root = self.nodes.last()?;
        let mut best = (max_dist * max_dist, None);
        let mut stack = vec![root];
        while let Some(node) = stack.pop() {
            if node.bb().dist2(p) > best.0 {
                continue;
            }
            match node {
                Node::Leaf { start, end, .. } => {
                    for t in &self.tris[*start..*end] {
                        let q = closest_on_triangle(p, t);
                        let d = sub3(q, p);
                        let d2 = dot3(d, d);
                        if d2 <= best.0 {
                            best = (d2, Some(q));
                        }
                    }
                }
                Node::Inner { left, right, .. } => {
                    let (l, r) = (&self.nodes[*left], &self.nodes[*right]);
                    // Visit the nearer child first (pushed last).
                    if l.bb().dist2(p) < r.bb().dist2(p) {
                        stack.push(r);
                        stack.push(l);
                    } else {
                        stack.push(l);
                        stack.push(r);
                    }
                }
            }
        }
        best.1
    }

    /// First hit of the ray `origin + t * dir` (`t > 0`): `(t, point)` with `t`
    /// in metres along the normalised direction.
    pub fn raycast(&self, origin: [f64; 3], dir: [f64; 3]) -> Option<(f64, [f64; 3])> {
        let len = norm3(dir);
        if len.is_nan() || len <= 0.0 {
            return None;
        }
        let d = scale3(dir, 1.0 / len);
        let inv = d.map(|v| 1.0 / v);
        let root = self.nodes.last()?;
        let mut best = f64::INFINITY;
        let mut stack = vec![root];
        while let Some(node) = stack.pop() {
            if node.bb().ray(origin, inv, best).is_none() {
                continue;
            }
            match node {
                Node::Leaf { start, end, .. } => {
                    for t in &self.tris[*start..*end] {
                        if let Some(h) = ray_triangle(origin, d, t) {
                            best = best.min(h);
                        }
                    }
                }
                Node::Inner { left, right, .. } => {
                    stack.push(&self.nodes[*left]);
                    stack.push(&self.nodes[*right]);
                }
            }
        }
        best.is_finite()
            .then(|| (best, add3(origin, scale3(d, best))))
    }
}

/// Builds the subtree over `tris[start..end]`; returns its node index.
fn build(tris: &mut [[[f64; 3]; 3]], start: usize, end: usize, nodes: &mut Vec<Node>) -> usize {
    let mut bb = Aabb::empty();
    let mut cb = Aabb::empty();
    for t in &tris[start..end] {
        for v in t {
            bb.grow(*v);
        }
        cb.grow(centre(t));
    }
    if end - start <= LEAF {
        nodes.push(Node::Leaf { bb, start, end });
        return nodes.len() - 1;
    }
    let ext = sub3(cb.max, cb.min);
    let axis = (0..3)
        .max_by(|&a, &b| ext[a].total_cmp(&ext[b]))
        .unwrap_or(0);
    let mid = start + (end - start) / 2;
    tris[start..end].select_nth_unstable_by(mid - start, |a, b| {
        centre(a)[axis].total_cmp(&centre(b)[axis])
    });
    let left = build(tris, start, mid, nodes);
    let right = build(tris, mid, end, nodes);
    let mut full = *nodes[left].bb();
    full.union(nodes[right].bb());
    nodes.push(Node::Inner {
        bb: full,
        left,
        right,
    });
    nodes.len() - 1
}

fn centre(t: &[[f64; 3]; 3]) -> [f64; 3] {
    scale3(add3(add3(t[0], t[1]), t[2]), 1.0 / 3.0)
}

/// Closest point on a triangle (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(p: [f64; 3], t: &[[f64; 3]; 3]) -> [f64; 3] {
    let [a, b, c] = *t;
    let (ab, ac, ap) = (sub3(b, a), sub3(c, a), sub3(p, a));
    let (d1, d2) = (dot3(ab, ap), dot3(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub3(p, b);
    let (d3, d4) = (dot3(ab, bp), dot3(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add3(a, scale3(ab, d1 / (d1 - d3)));
    }
    let cp = sub3(p, c);
    let (d5, d6) = (dot3(ab, cp), dot3(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add3(a, scale3(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return add3(b, scale3(sub3(c, b), w));
    }
    let denom = va + vb + vc;
    if denom.abs() < 1e-30 {
        return a;
    }
    let (v, w) = (vb / denom, vc / denom);
    add3(a, add3(scale3(ab, v), scale3(ac, w)))
}

/// Möller–Trumbore; distance along the unit direction `d` for `t > 1e-9`.
fn ray_triangle(o: [f64; 3], d: [f64; 3], t: &[[f64; 3]; 3]) -> Option<f64> {
    let (e1, e2) = (sub3(t[1], t[0]), sub3(t[2], t[0]));
    let p = cross3(d, e2);
    let det = dot3(e1, p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub3(o, t[0]);
    let u = dot3(s, p) * inv;
    if !(-1e-12..=1.0 + 1e-12).contains(&u) {
        return None;
    }
    let q = cross3(s, e1);
    let v = dot3(d, q) * inv;
    if v < -1e-12 || u + v > 1.0 + 1e-12 {
        return None;
    }
    let h = dot3(e2, q) * inv;
    (h > 1e-9).then_some(h)
}

/// Measurement mode of [`MeasureRequest`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MeasureMode {
    /// `a` and `b` are picked points, each snapped to the surface.
    #[default]
    Point,
    /// `a` is a ray origin and `b` its direction.
    Ray,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeasureRequest {
    pub job_id: String,
    pub a: [f64; 3],
    pub b: [f64; 3],
    #[serde(default)]
    pub mode: MeasureMode,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeasureResponse {
    pub distance_m: f64,
    pub a_hit: Option<[f64; 3]>,
    pub b_hit: Option<[f64; 3]>,
}

/// Point mode: each end snaps to the nearest surface within
/// [`SNAP_RADIUS_M`] (the raw point is used when nothing is in reach) and the
/// distance is between the two. Ray mode: distance from `a` to the first hit
/// along `b`; `None` when the ray misses.
pub fn measure(idx: &MeshIndex, req: &MeasureRequest) -> Option<MeasureResponse> {
    match req.mode {
        MeasureMode::Point => {
            let a_hit = idx.nearest(req.a, SNAP_RADIUS_M);
            let b_hit = idx.nearest(req.b, SNAP_RADIUS_M);
            let (a, b) = (a_hit.unwrap_or(req.a), b_hit.unwrap_or(req.b));
            Some(MeasureResponse {
                distance_m: norm3(sub3(b, a)),
                a_hit,
                b_hit,
            })
        }
        MeasureMode::Ray => {
            let (t, hit) = idx.raycast(req.a, req.b)?;
            Some(MeasureResponse {
                distance_m: t,
                a_hit: Some(req.a),
                b_hit: Some(hit),
            })
        }
    }
}
