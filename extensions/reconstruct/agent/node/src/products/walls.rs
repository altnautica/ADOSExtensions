//! Wall extraction from a horizontal slice: sequential local RANSAC lines,
//! collinear merging, Manhattan snapping, corner completion and door gaps.

use std::collections::HashMap;

use super::geom::{angle_diff, cross2, dir, dist2, dot2, fit_line, norm_angle, normal, Rng};

/// One infinite wall line with the points it explains and the occupied
/// intervals (runs) along it.
#[derive(Debug, Clone)]
pub(crate) struct WallLine {
    /// Direction angle in `[0, π)`.
    pub angle: f64,
    /// Offset along the normal `(-sin, cos)`.
    pub offset: f64,
    pub points: Vec<[f64; 2]>,
    /// Sorted `[t0, t1]` intervals along the direction.
    pub runs: Vec<[f64; 2]>,
}

impl WallLine {
    pub fn dir(&self) -> [f64; 2] {
        dir(self.angle)
    }

    pub fn normal(&self) -> [f64; 2] {
        normal(self.angle)
    }

    pub fn at(&self, t: f64) -> [f64; 2] {
        let (d, n) = (self.dir(), self.normal());
        [d[0] * t + n[0] * self.offset, d[1] * t + n[1] * self.offset]
    }

    pub fn t_of(&self, p: [f64; 2]) -> f64 {
        dot2(self.dir(), p)
    }

    pub fn dist(&self, p: [f64; 2]) -> f64 {
        (dot2(self.normal(), p) - self.offset).abs()
    }

    pub fn extent(&self) -> Option<[f64; 2]> {
        Some([self.runs.first()?[0], self.runs.last()?[1]])
    }

    pub fn length(&self) -> f64 {
        self.runs.iter().map(|r| r[1] - r[0]).sum()
    }

    /// Re-fits the offset for the current angle and recomputes the runs.
    fn refit_offset(&mut self, prm: &WallParams) {
        let n = self.normal();
        let k = self.points.len().max(1) as f64;
        self.offset = self.points.iter().map(|p| dot2(n, *p)).sum::<f64>() / k;
        self.recompute_runs(prm);
    }

    fn recompute_runs(&mut self, prm: &WallParams) {
        let d = self.dir();
        let mut ts: Vec<f64> = self.points.iter().map(|p| dot2(d, *p)).collect();
        ts.sort_unstable_by(f64::total_cmp);
        self.runs.clear();
        let mut start = 0;
        for i in 1..=ts.len() {
            if i == ts.len() || ts[i] - ts[i - 1] > prm.run_gap {
                let (a, b) = (ts[start], ts[i - 1]);
                if i - start >= prm.min_run_points && b - a >= prm.min_run_len {
                    self.runs.push([a, b]);
                }
                start = i;
            }
        }
    }
}

/// Tunables of the wall extractor.
#[derive(Debug, Clone)]
pub(crate) struct WallParams {
    pub tol: f64,
    pub min_inliers: usize,
    pub min_wall_len: f64,
    pub run_gap: f64,
    pub min_run_len: f64,
    pub min_run_points: usize,
    pub iters: usize,
    pub local_radius: f64,
}

impl WallParams {
    pub fn new(tol: f64, min_wall_len: f64) -> Self {
        Self {
            tol,
            min_inliers: 25,
            min_wall_len,
            run_gap: 0.25,
            min_run_len: 0.1,
            min_run_points: 5,
            iters: 300,
            local_radius: 3.0,
        }
    }
}

const BUCKET: f64 = 0.5;

fn bucket_of(p: [f64; 2]) -> (i64, i64) {
    (
        (p[0] / BUCKET).floor() as i64,
        (p[1] / BUCKET).floor() as i64,
    )
}

/// Sequential RANSAC: the best locally-scored line is refined by total least
/// squares, its inliers removed, and the loop repeats until no line explains
/// `min_inliers` points.
pub(crate) fn extract_lines(pts: &[[f64; 2]], prm: &WallParams, rng: &mut Rng) -> Vec<WallLine> {
    let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, p) in pts.iter().enumerate() {
        buckets.entry(bucket_of(*p)).or_default().push(i);
    }
    let mut alive = vec![true; pts.len()];
    let mut lines = Vec::new();
    let reach = (prm.local_radius / BUCKET).ceil() as i64;
    let near = |c: (i64, i64), r: i64, alive: &[bool], out: &mut Vec<usize>| {
        out.clear();
        for dx in -r..=r {
            for dy in -r..=r {
                if let Some(v) = buckets.get(&(c.0 + dx, c.1 + dy)) {
                    out.extend(v.iter().copied().filter(|&i| alive[i]));
                }
            }
        }
    };
    let mut local = Vec::new();
    let mut pair = Vec::new();
    loop {
        let live: Vec<usize> = (0..pts.len()).filter(|&i| alive[i]).collect();
        if live.len() < prm.min_inliers {
            break;
        }
        let mut best: Option<(usize, f64, f64)> = None;
        for _ in 0..prm.iters {
            let i = live[rng.below(live.len())];
            near(bucket_of(pts[i]), 2, &alive, &mut pair);
            let j = pair[rng.below(pair.len())];
            if dist2(pts[i], pts[j]) < 0.15 {
                continue;
            }
            let angle = norm_angle((pts[j][1] - pts[i][1]).atan2(pts[j][0] - pts[i][0]));
            let n = normal(angle);
            let c = dot2(n, pts[i]);
            near(bucket_of(pts[i]), reach, &alive, &mut local);
            let score = local
                .iter()
                .filter(|&&k| (dot2(n, pts[k]) - c).abs() < prm.tol)
                .count();
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, angle, c));
            }
        }
        let Some((score, mut angle, mut c)) = best else {
            break;
        };
        if score < prm.min_inliers {
            break;
        }
        let mut inliers: Vec<usize> = Vec::new();
        for _ in 0..2 {
            let n = normal(angle);
            inliers = live
                .iter()
                .copied()
                .filter(|&k| (dot2(n, pts[k]) - c).abs() < prm.tol)
                .collect();
            let sel: Vec<[f64; 2]> = inliers.iter().map(|&k| pts[k]).collect();
            if let Some((a, o)) = fit_line(&sel) {
                angle = a;
                c = o;
            }
        }
        let n = normal(angle);
        inliers.retain(|&k| (dot2(n, pts[k]) - c).abs() < prm.tol);
        if inliers.is_empty() {
            break;
        }
        for &k in &inliers {
            alive[k] = false;
        }
        let mut line = WallLine {
            angle,
            offset: c,
            points: inliers.iter().map(|&k| pts[k]).collect(),
            runs: Vec::new(),
        };
        line.recompute_runs(prm);
        if line.runs.iter().any(|r| r[1] - r[0] >= prm.min_wall_len) {
            lines.push(line);
        }
    }
    lines
}

/// Merges near-collinear lines (angle within `angle_tol`, offset within
/// `offset_tol`); the longer line keeps its angle.
pub(crate) fn merge_collinear(
    mut lines: Vec<WallLine>,
    angle_tol: f64,
    offset_tol: f64,
    prm: &WallParams,
) -> Vec<WallLine> {
    lines.sort_by(|a, b| b.length().total_cmp(&a.length()));
    let mut out: Vec<WallLine> = Vec::new();
    for l in lines {
        let mid = l.points.iter().fold([0.0, 0.0], |s, p| {
            [
                s[0] + p[0] / l.points.len() as f64,
                s[1] + p[1] / l.points.len() as f64,
            ]
        });
        let target = out
            .iter_mut()
            .find(|o| angle_diff(o.angle, l.angle) < angle_tol && o.dist(mid) < offset_tol);
        match target {
            Some(o) => {
                o.points.extend(l.points);
                o.refit_offset(prm);
            }
            None => out.push(l),
        }
    }
    out
}

/// Snaps lines within `max_dev` of the dominant orthogonal frame onto it.
/// Returns the dominant angle.
pub(crate) fn manhattan_snap(
    lines: &mut [WallLine],
    max_dev: f64,
    prm: &WallParams,
) -> Option<f64> {
    let (mut s, mut c) = (0.0, 0.0);
    for l in lines.iter() {
        let w = l.length();
        s += w * (4.0 * l.angle).sin();
        c += w * (4.0 * l.angle).cos();
    }
    if s == 0.0 && c == 0.0 {
        return None;
    }
    let quarter = std::f64::consts::FRAC_PI_2;
    let dominant = s.atan2(c) / 4.0;
    for l in lines.iter_mut() {
        let k = ((l.angle - dominant) / quarter).round();
        let snapped = dominant + k * quarter;
        if (l.angle - snapped).abs() < max_dev {
            l.angle = norm_angle(snapped);
            l.refit_offset(prm);
        }
    }
    Some(norm_angle(dominant))
}

/// Intersection of two lines, `None` when near-parallel.
pub(crate) fn intersect(a: &WallLine, b: &WallLine) -> Option<[f64; 2]> {
    let (na, nb) = (a.normal(), b.normal());
    let det = cross2(na, nb);
    if det.abs() < 0.5 {
        return None;
    }
    Some([
        (a.offset * nb[1] - b.offset * na[1]) / det,
        (na[0] * b.offset - nb[0] * a.offset) / det,
    ])
}

/// Extends the outer ends of each line to a crossing line when the corner
/// lies within `reach` of both.
pub(crate) fn complete_corners(lines: &mut [WallLine], reach: f64) {
    let snapshot: Vec<WallLine> = lines.to_vec();
    for (i, l) in lines.iter_mut().enumerate() {
        let Some([t0, t1]) = l.extent() else {
            continue;
        };
        let (mut new0, mut new1) = (t0, t1);
        for (j, m) in snapshot.iter().enumerate() {
            let (Some(x), Some([m0, m1])) = (intersect(l, m), m.extent()) else {
                continue;
            };
            if i == j {
                continue;
            }
            let tm = m.t_of(x);
            if tm < m0 - reach || tm > m1 + reach {
                continue;
            }
            let t = l.t_of(x);
            if t < t0 && t0 - t <= reach {
                new0 = new0.min(t);
            }
            if t > t1 && t - t1 <= reach {
                new1 = new1.max(t);
            }
        }
        if let Some(first) = l.runs.first_mut() {
            first[0] = new0;
        }
        if let Some(last) = l.runs.last_mut() {
            last[1] = new1;
        }
    }
}

/// Door candidates: gaps of `door_min..=door_max` between consecutive runs of
/// one line, rejected when the low band (`low` points, wall below a window)
/// fills the gap. Parallel candidates on the two faces of one wall merge.
pub(crate) fn doors(
    lines: &[WallLine],
    low: &[[f64; 2]],
    door_min: f64,
    door_max: f64,
    tol: f64,
) -> Vec<[[f64; 2]; 2]> {
    let mut found: Vec<[[f64; 2]; 2]> = Vec::new();
    for l in lines {
        for w in l.runs.windows(2) {
            let (a, b) = (w[0][1], w[1][0]);
            let gap = b - a;
            if gap < door_min || gap > door_max {
                continue;
            }
            let (lo, hi) = (a + 0.1, b - 0.1);
            let blocked = low
                .iter()
                .filter(|p| l.dist(**p) < 2.0 * tol && (lo..=hi).contains(&l.t_of(**p)))
                .count();
            // A wall below the gap (a window) fills it at slice density.
            if blocked as f64 > 0.3 * (hi - lo) / 0.02 {
                continue;
            }
            found.push([l.at(a), l.at(b)]);
        }
    }
    let mut merged: Vec<([[f64; 2]; 2], u32)> = Vec::new();
    for d in found {
        let mid = |s: &[[f64; 2]; 2]| [(s[0][0] + s[1][0]) / 2.0, (s[0][1] + s[1][1]) / 2.0];
        let m = mid(&d);
        let hit = merged.iter_mut().find(|(o, _)| dist2(mid(o), m) < 0.4);
        match hit {
            Some((o, k)) => {
                let d = if dist2(o[0], d[0]) > dist2(o[0], d[1]) {
                    [d[1], d[0]]
                } else {
                    d
                };
                let w = *k as f64;
                for e in 0..2 {
                    for c in 0..2 {
                        o[e][c] = (o[e][c] * w + d[e][c]) / (w + 1.0);
                    }
                }
                *k += 1;
            }
            None => merged.push((d, 1)),
        }
    }
    merged.into_iter().map(|(d, _)| d).collect()
}
