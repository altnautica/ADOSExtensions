//! Room segmentation on an occupancy grid: walls are rasterised as blocking
//! cells, free space is split into 4-connected components, and a component's
//! outer boundary becomes a polygon snapped back onto the wall lines.

use std::collections::{HashMap, VecDeque};

use super::geom::{cross2, dot2, signed_area, simplify_ring};
use super::walls::WallLine;

pub(crate) struct Grid {
    pub origin: [f64; 2],
    pub cell: f64,
    pub w: usize,
    pub h: usize,
    wall: Vec<bool>,
    walk: Vec<bool>,
}

impl Grid {
    pub fn new(min: [f64; 2], max: [f64; 2], cell: f64) -> Self {
        let w = (((max[0] - min[0]) / cell).ceil() as usize).max(1) + 1;
        let h = (((max[1] - min[1]) / cell).ceil() as usize).max(1) + 1;
        Self {
            origin: min,
            cell,
            w,
            h,
            wall: vec![false; w * h],
            walk: vec![true; w * h],
        }
    }

    pub fn cell_of(&self, p: [f64; 2]) -> Option<(usize, usize)> {
        let i = ((p[0] - self.origin[0]) / self.cell).floor();
        let j = ((p[1] - self.origin[1]) / self.cell).floor();
        (i >= 0.0 && j >= 0.0 && (i as usize) < self.w && (j as usize) < self.h)
            .then_some((i as usize, j as usize))
    }

    fn corner(&self, i: i64, j: i64) -> [f64; 2] {
        [
            self.origin[0] + i as f64 * self.cell,
            self.origin[1] + j as f64 * self.cell,
        ]
    }

    /// Marks every cell the segment `a–b` passes through as wall.
    pub fn raster_segment(&mut self, a: [f64; 2], b: [f64; 2]) {
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        let steps = (len / (self.cell / 3.0)).ceil() as usize + 1;
        for s in 0..=steps {
            let t = s as f64 / steps as f64;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            if let Some((i, j)) = self.cell_of(p) {
                self.wall[j * self.w + i] = true;
            }
        }
    }

    /// Restricts free space to cells within `dilate` cells of an observed
    /// floor point.
    pub fn set_walkable(&mut self, floor: &[[f64; 2]], dilate: usize) {
        let mut seen = vec![false; self.w * self.h];
        for p in floor {
            if let Some((i, j)) = self.cell_of(*p) {
                seen[j * self.w + i] = true;
            }
        }
        let r = dilate as i64;
        for j in 0..self.h as i64 {
            for i in 0..self.w as i64 {
                let mut any = false;
                'n: for dj in -r..=r {
                    for di in -r..=r {
                        let (x, y) = (i + di, j + dj);
                        if x >= 0
                            && y >= 0
                            && (x as usize) < self.w
                            && (y as usize) < self.h
                            && seen[y as usize * self.w + x as usize]
                        {
                            any = true;
                            break 'n;
                        }
                    }
                }
                self.walk[j as usize * self.w + i as usize] = any;
            }
        }
    }

    fn free(&self, k: usize) -> bool {
        !self.wall[k] && self.walk[k]
    }

    /// Per-cell free flags (row-major, `w × h`).
    pub fn free_mask(&self) -> Vec<bool> {
        (0..self.w * self.h).map(|k| self.free(k)).collect()
    }

    /// 4-connected component labels of free cells (0 = blocked).
    pub fn label(&self) -> Vec<u32> {
        let mut labels = vec![0u32; self.w * self.h];
        let mut next = 0u32;
        let mut queue = VecDeque::new();
        for start in 0..labels.len() {
            if labels[start] != 0 || !self.free(start) {
                continue;
            }
            next += 1;
            labels[start] = next;
            queue.push_back(start);
            while let Some(k) = queue.pop_front() {
                let (i, j) = (k % self.w, k / self.w);
                let mut visit = |n: usize| {
                    if labels[n] == 0 && self.free(n) {
                        labels[n] = next;
                        queue.push_back(n);
                    }
                };
                if i > 0 {
                    visit(k - 1);
                }
                if i + 1 < self.w {
                    visit(k + 1);
                }
                if j > 0 {
                    visit(k - self.w);
                }
                if j + 1 < self.h {
                    visit(k + self.w);
                }
            }
        }
        labels
    }

    /// Label under `p`, searching up to `reach` cells around a blocked cell.
    pub fn label_at(&self, labels: &[u32], p: [f64; 2], reach: usize) -> Option<u32> {
        let (ci, cj) = self.cell_of(p)?;
        for r in 0..=reach as i64 {
            let mut best: Option<(i64, u32)> = None;
            for dj in -r..=r {
                for di in -r..=r {
                    if di.abs().max(dj.abs()) != r {
                        continue;
                    }
                    let (x, y) = (ci as i64 + di, cj as i64 + dj);
                    if x < 0 || y < 0 || x as usize >= self.w || y as usize >= self.h {
                        continue;
                    }
                    let l = labels[y as usize * self.w + x as usize];
                    let d = di * di + dj * dj;
                    if l != 0 && best.is_none_or(|b| d < b.0) {
                        best = Some((d, l));
                    }
                }
            }
            if let Some((_, l)) = best {
                return Some(l);
            }
        }
        None
    }

    /// Outer boundary (counter-clockwise, on cell corners) of the cells with
    /// `label`.
    pub fn outer_boundary(&self, labels: &[u32], label: u32) -> Vec<[f64; 2]> {
        let inside = |i: i64, j: i64| {
            i >= 0
                && j >= 0
                && (i as usize) < self.w
                && (j as usize) < self.h
                && labels[j as usize * self.w + i as usize] == label
        };
        // Directed boundary edges keep the region on their left.
        let mut out: HashMap<(i64, i64), Vec<(i64, i64)>> = HashMap::new();
        let mut edges = 0usize;
        for j in 0..self.h as i64 {
            for i in 0..self.w as i64 {
                if !inside(i, j) {
                    continue;
                }
                let mut add = |a: (i64, i64), b: (i64, i64)| {
                    out.entry(a).or_default().push(b);
                    edges += 1;
                };
                if !inside(i, j - 1) {
                    add((i, j), (i + 1, j));
                }
                if !inside(i + 1, j) {
                    add((i + 1, j), (i + 1, j + 1));
                }
                if !inside(i, j + 1) {
                    add((i + 1, j + 1), (i, j + 1));
                }
                if !inside(i - 1, j) {
                    add((i, j + 1), (i, j));
                }
            }
        }
        let mut best: Vec<[f64; 2]> = Vec::new();
        let mut best_area = 0.0;
        let mut starts: Vec<(i64, i64)> = out.keys().copied().collect();
        starts.sort_unstable();
        for s in starts {
            while let Some(first) = out.get_mut(&s).and_then(Vec::pop) {
                let mut ring = vec![s];
                let (mut prev, mut cur) = (s, first);
                let mut guard = 0;
                while cur != s && guard <= edges {
                    guard += 1;
                    ring.push(cur);
                    let Some(nexts) = out.get_mut(&cur) else {
                        break;
                    };
                    if nexts.is_empty() {
                        break;
                    }
                    // At a pinch vertex take the sharpest left turn.
                    let din = (cur.0 - prev.0, cur.1 - prev.1);
                    let pick = (0..nexts.len())
                        .max_by_key(|&k| {
                            let d = (nexts[k].0 - cur.0, nexts[k].1 - cur.1);
                            din.0 * d.1 - din.1 * d.0
                        })
                        .unwrap_or(0);
                    let nxt = nexts.swap_remove(pick);
                    prev = cur;
                    cur = nxt;
                }
                let poly: Vec<[f64; 2]> = ring.iter().map(|&(i, j)| self.corner(i, j)).collect();
                let a = signed_area(&poly);
                if a > best_area {
                    best_area = a;
                    best = poly;
                }
            }
        }
        drop_collinear(&best)
    }
}

fn drop_collinear(ring: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let n = ring.len();
    if n < 4 {
        return ring.to_vec();
    }
    (0..n)
        .filter(|&i| {
            let (a, b, c) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
            cross2([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]).abs() > 1e-12
        })
        .map(|i| ring[i])
        .collect()
}

/// A polygon edge's supporting line: a point on it, a unit direction and the
/// wall line it was snapped to.
struct EdgeLine {
    q: [f64; 2],
    u: [f64; 2],
    wall: Option<usize>,
}

/// Simplifies a raster boundary and moves each edge onto the nearest parallel
/// wall line within `snap` metres; vertices become edge-line intersections.
/// Falls back to the simplified raster ring when snapping distorts the area by
/// more than 25 %.
pub(crate) fn polygon_from_boundary(
    ring: &[[f64; 2]],
    lines: &[WallLine],
    simplify: f64,
    snap: f64,
) -> Vec<[f64; 2]> {
    let simple = simplify_ring(ring, simplify);
    let n = simple.len();
    if n < 3 {
        return simple;
    }
    let max_sin = 10f64.to_radians().sin();
    let mut edges: Vec<EdgeLine> = Vec::with_capacity(n);
    for i in 0..n {
        let (a, b) = (simple[i], simple[(i + 1) % n]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let len = dot2(d, d).sqrt().max(1e-12);
        let u = [d[0] / len, d[1] / len];
        let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        let wall = lines
            .iter()
            .enumerate()
            .filter(|(_, l)| {
                let Some([t0, t1]) = l.extent() else {
                    return false;
                };
                let (ta, tb) = (l.t_of(a), l.t_of(b));
                cross2(u, l.dir()).abs() < max_sin
                    && l.dist(mid) < snap
                    && ta.max(tb) > t0 - 0.5
                    && ta.min(tb) < t1 + 0.5
            })
            .min_by(|x, y| x.1.dist(mid).total_cmp(&y.1.dist(mid)))
            .map(|(k, _)| k);
        edges.push(match wall {
            Some(k) => {
                let l = &lines[k];
                let ld = l.dir();
                let s = if dot2(ld, u) < 0.0 { -1.0 } else { 1.0 };
                let off = dot2(l.normal(), mid) - l.offset;
                let nn = l.normal();
                EdgeLine {
                    q: [mid[0] - nn[0] * off, mid[1] - nn[1] * off],
                    u: [ld[0] * s, ld[1] * s],
                    wall: Some(k),
                }
            }
            None => EdgeLine {
                q: a,
                u,
                wall: None,
            },
        });
    }
    let proj = |e: &EdgeLine, p: [f64; 2]| {
        let t = dot2([p[0] - e.q[0], p[1] - e.q[1]], e.u);
        [e.q[0] + e.u[0] * t, e.q[1] + e.u[1] * t]
    };
    let min_sin = 20f64.to_radians().sin();
    let mut out: Vec<[f64; 2]> = Vec::new();
    for i in 0..n {
        let (e0, e1) = (&edges[(i + n - 1) % n], &edges[i]);
        if e0.wall.is_some() && e0.wall == e1.wall {
            continue;
        }
        let c = cross2(e0.u, e1.u);
        if c.abs() > min_sin {
            let w = [e1.q[0] - e0.q[0], e1.q[1] - e0.q[1]];
            let t = cross2(w, e1.u) / c;
            out.push([e0.q[0] + e0.u[0] * t, e0.q[1] + e0.u[1] * t]);
        } else {
            out.push(proj(e0, simple[i]));
            out.push(proj(e1, simple[i]));
        }
    }
    out.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-4 && (a[1] - b[1]).abs() < 1e-4);
    let raw = signed_area(&simple).abs();
    let snapped = signed_area(&out);
    if out.len() < 3 || snapped <= 0.0 || (snapped - raw).abs() > 0.25 * raw {
        return simple;
    }
    out
}
