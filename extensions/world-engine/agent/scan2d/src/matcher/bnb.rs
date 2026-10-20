//! Branch-and-bound search over a max-pooled pyramid (Hess et al. 2016).

use super::{rotated_cells, Window};
use crate::grid::Pyramid;
use crate::pose::Pose2;
use crate::scan::Scan2d;

/// The best-scoring pose in `window` around `prior` whose score (mean cell
/// probability) is above `min_score`, found by depth-first branch and bound
/// over the pyramid. Returns the same pose as exhaustive search would.
pub fn branch_and_bound(
    pyr: &Pyramid,
    scan: &Scan2d,
    prior: Pose2,
    window: Window,
    min_score: f64,
) -> Option<(Pose2, f64)> {
    if scan.points.is_empty() {
        return None;
    }
    let res = pyr.resolution();
    let wl = (window.linear_m / res).ceil() as i64;
    let (step, na, sets) = rotated_cells(|p| pyr.cell_of(p), scan, prior, window, res);
    let top = pyr.depth() - 1;
    let size = 1i64 << top;
    let mut cands = Vec::new();
    for (a, cells) in sets.iter().enumerate() {
        let mut oy = -wl;
        while oy <= wl {
            let mut ox = -wl;
            while ox <= wl {
                let c = Cand {
                    a,
                    ox,
                    oy,
                    score: 0.0,
                };
                cands.push(Cand {
                    score: bound(pyr, cells, top, &c),
                    ..c
                });
                ox += size;
            }
            oy += size;
        }
    }
    let mut best: Option<Cand> = None;
    let mut best_score = min_score;
    descend(pyr, &sets, wl, top, cands, &mut best, &mut best_score);
    best.map(|c| {
        let pose = Pose2::new(
            prior.x + c.ox as f64 * res,
            prior.y + c.oy as f64 * res,
            prior.th + (c.a as i64 - na) as f64 * step,
        );
        (pose, c.score)
    })
}

#[derive(Debug, Clone, Copy)]
struct Cand {
    a: usize,
    ox: i64,
    oy: i64,
    score: f64,
}

fn bound(pyr: &Pyramid, cells: &[(i64, i64)], level: usize, c: &Cand) -> f64 {
    let sum: f64 = cells
        .iter()
        .map(|&(ix, iy)| pyr.value(level, ix + c.ox, iy + c.oy) as f64)
        .sum();
    sum / cells.len() as f64
}

fn descend(
    pyr: &Pyramid,
    sets: &[Vec<(i64, i64)>],
    wl: i64,
    level: usize,
    mut cands: Vec<Cand>,
    best: &mut Option<Cand>,
    best_score: &mut f64,
) {
    cands.sort_by(|a, b| b.score.total_cmp(&a.score));
    for c in cands {
        if c.score <= *best_score {
            break;
        }
        if level == 0 {
            *best_score = c.score;
            *best = Some(c);
            break;
        }
        let half = 1i64 << (level - 1);
        let mut children = Vec::with_capacity(4);
        for (dx, dy) in [(0, 0), (half, 0), (0, half), (half, half)] {
            let (ox, oy) = (c.ox + dx, c.oy + dy);
            if ox > wl || oy > wl {
                continue;
            }
            let k = Cand { ox, oy, ..c };
            children.push(Cand {
                score: bound(pyr, &sets[c.a], level - 1, &k),
                ..k
            });
        }
        descend(pyr, sets, wl, level - 1, children, best, best_score);
    }
}
