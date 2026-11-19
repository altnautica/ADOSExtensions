//! Free-space carving of a wall slice. Rays from the sensor positions (the
//! keyframe camera centres and LiDAR scan origins) cross the slice; a cell a
//! ray passes through on its way to a wall was seen empty. A cell is kept as
//! wall when it holds a hit and was seen empty at most three times per hit:
//! a person walking by or a sparse clutter return is crossed far more often
//! than it is hit and drops out, a wall stops the rays and stays.

use std::collections::HashSet;

/// Carving cell, metres.
pub(crate) const CELL_M: f64 = 0.05;
/// A cell stays wall while `free <= FREE_PER_HIT * hits`.
const FREE_PER_HIT: u32 = 3;
/// Rays reach this far from their origin.
const RANGE_M: f64 = 15.0;
/// Rays cast per origin, evenly over the full turn.
const RAYS: usize = 1440;
/// Origins closer than this to a kept one add no rays.
const ORIGIN_SPACING_M: f64 = 0.25;

/// Hit and free-pass counts on the slice's bounding grid.
struct Counts {
    min: [i64; 2],
    w: i64,
    h: i64,
    hits: Vec<u32>,
    free: Vec<u32>,
}

impl Counts {
    fn index(&self, c: (i64, i64)) -> Option<usize> {
        let (i, j) = (c.0 - self.min[0], c.1 - self.min[1]);
        (i >= 0 && j >= 0 && i < self.w && j < self.h).then(|| (j * self.w + i) as usize)
    }
}

fn cell_of(p: [f64; 2]) -> (i64, i64) {
    (
        (p[0] / CELL_M).floor() as i64,
        (p[1] / CELL_M).floor() as i64,
    )
}

/// For each slice point, whether its cell is a wall cell. Without origins
/// nothing can be carved and every point is kept.
///
/// A ray ends at the first strongly hit cell (as many hits as the median hit
/// cell): that is the surface it observed, and nothing behind it was seen. A
/// ray that meets no such cell within range observed nothing and counts for
/// nothing; the weakly hit cells a finished ray crossed count one free pass.
pub(crate) fn wall_mask(points: &[[f64; 2]], origins: &[[f64; 2]]) -> Vec<bool> {
    if origins.is_empty() || points.is_empty() {
        return vec![true; points.len()];
    }
    let cells: Vec<(i64, i64)> = points.iter().map(|p| cell_of(*p)).collect();
    let (mut min, mut max) = ([i64::MAX; 2], [i64::MIN; 2]);
    for c in &cells {
        min = [min[0].min(c.0), min[1].min(c.1)];
        max = [max[0].max(c.0), max[1].max(c.1)];
    }
    let (w, h) = (max[0] - min[0] + 1, max[1] - min[1] + 1);
    let mut g = Counts {
        min,
        w,
        h,
        hits: vec![0; (w * h) as usize],
        free: vec![0; (w * h) as usize],
    };
    for c in &cells {
        if let Some(k) = g.index(*c) {
            g.hits[k] += 1;
        }
    }
    let mut counts: Vec<u32> = g.hits.iter().copied().filter(|&n| n > 0).collect();
    counts.sort_unstable();
    let strong = counts[counts.len() / 2].max(2);

    let spacing = (ORIGIN_SPACING_M / CELL_M).round() as i64;
    let mut seen_origin = HashSet::new();
    let mut crossed: Vec<usize> = Vec::new();
    let step = CELL_M * 0.5;
    let steps = (RANGE_M / step) as usize;
    let lo = [min[0] as f64 * CELL_M, min[1] as f64 * CELL_M];
    let hi = [(max[0] + 1) as f64 * CELL_M, (max[1] + 1) as f64 * CELL_M];
    for o in origins {
        let oc = cell_of(*o);
        if !seen_origin.insert((oc.0.div_euclid(spacing), oc.1.div_euclid(spacing))) {
            continue;
        }
        for r in 0..RAYS {
            let a = r as f64 / RAYS as f64 * std::f64::consts::TAU;
            let (s, c) = a.sin_cos();
            crossed.clear();
            let mut last = oc;
            let mut ended = false;
            for k in 1..=steps {
                let t = k as f64 * step;
                let p = [o[0] + c * t, o[1] + s * t];
                // Once outside the slice's box and moving away, nothing is left
                // to meet.
                let leaving = (p[0] < lo[0] && c <= 0.0)
                    || (p[0] > hi[0] && c >= 0.0)
                    || (p[1] < lo[1] && s <= 0.0)
                    || (p[1] > hi[1] && s >= 0.0);
                if leaving {
                    break;
                }
                let cell = cell_of(p);
                if cell == last {
                    continue;
                }
                last = cell;
                let Some(idx) = g.index(cell) else {
                    continue;
                };
                match g.hits[idx] {
                    0 => {}
                    n if n >= strong => {
                        ended = true;
                        break;
                    }
                    _ => crossed.push(idx),
                }
            }
            if ended {
                for &idx in &crossed {
                    g.free[idx] += 1;
                }
            }
        }
    }
    cells
        .iter()
        .map(|c| {
            g.index(*c)
                .is_none_or(|k| g.free[k] <= FREE_PER_HIT * g.hits[k])
        })
        .collect()
}
