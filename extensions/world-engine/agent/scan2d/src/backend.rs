//! Offline back end: loop-constraint search against finished submaps and
//! pose-graph optimisation of the submap poses.
//!
//! Inserted scans are taken in time order; every `loop_every_n_nodes`-th is
//! matched by branch and bound (on the submap's occupied-or-not pyramid)
//! against each submap at least `loop_min_submap_gap` older than its own
//! that holds a scan taken within `loop_search_radius_m`, around the pose the
//! current graph predicts, scoring only the part of the scan that submap
//! observed from close by. A match scoring at least `loop_min_score` is
//! refined. It is counted as rejected when its optimum sits on the window's
//! border or when it moves the scan from the prediction by more than the
//! drift the walked distance allows (`MIN_CORRECTION_M` plus
//! `DRIFT_PER_METRE` of the path, at most `loop_max_correction_m`; at most
//! `loop_max_correction_deg`), dropped when its translation is constrained
//! along one direction only, and otherwise becomes a loop edge. The graph is
//! re-solved after every scan that added an edge, so later predictions carry
//! the corrections already found. Every scan pose is finally re-expressed
//! through its corrected submap.

use std::collections::HashSet;

use crate::config::Scan2dConfig;
use crate::frontend::{LocalMap, PoseOut};
use crate::graph::{solve, Edge};
use crate::grid::Pyramid;
use nalgebra::{Matrix2, Matrix3, SymmetricEigen};

use crate::matcher::{
    branch_and_bound, constraint_information, degeneracy, refine, translation_information, Window,
};
use crate::pose::{wrap, Pose2};
use crate::scan::Scan2d;
use crate::submap::Submap;

/// Pyramid levels for loop search (windows of 1 to 64 cells).
const PYRAMID_DEPTH: usize = 7;
/// Loop matching scores only scan points that land, at the prior, within
/// this distance of a position the submap's own scans were taken from.
const OBSERVED_RADIUS_M: f64 = 4.0;
/// Largest believable loop correction: `MIN_CORRECTION_M` plus
/// `DRIFT_PER_METRE` of the path walked between the submap and the scan,
/// capped at `loop_max_correction_m`. Local SLAM drifts with distance, so a
/// submap passed a few metres ago can not be a metre off; a match that far
/// out slid along a corridor or locked onto a repeated feature.
const DRIFT_PER_METRE: f64 = 0.02;
const MIN_CORRECTION_M: f64 = 0.1;

/// Whether the search optimum `m` lies on the border of `window` around
/// `prior`: the true optimum may then lie outside, so the match proves
/// nothing.
fn on_window_edge(prior: Pose2, m: Pose2, window: Window, res: f64) -> bool {
    (m.x - prior.x).abs() > window.linear_m - res
        || (m.y - prior.y).abs() > window.linear_m - res
        || wrap(m.th - prior.th).abs() > window.angular_rad * 0.99
}

/// Distance walked up to each scan along the local trajectory.
fn path_positions(map: &LocalMap) -> Vec<f64> {
    let mut s = 0.0;
    let mut last: Option<Pose2> = None;
    map.nodes
        .iter()
        .map(|nd| {
            let p = map.submaps[nd.submap].pose * nd.in_submap;
            if let Some(q) = last {
                s += p.dist(&q);
            }
            last = Some(p);
            s
        })
        .collect()
}

/// Loop matches whose translation information is more lopsided than this
/// (`eig_min / eig_max`) are dropped: a scan that can slide along a corridor
/// would bind two submaps at whatever offset the slide stopped at.
const LOOP_MIN_DEGENERACY: f64 = 0.1;

/// Loop edges trust no direction less than this share of their best one.
const MIN_LOOP_INFORMATION_SHARE: f64 = 1e-3;

/// Translation information of a loop edge: `1/sigma²` along the direction
/// the match constrains best, scaled down along weaker directions by the
/// match geometry (`h`, point-to-line information), so a match that can
/// slide along a corridor binds the graph only across it.
fn loop_information(h: &Matrix3<f64>, sigma_m: f64) -> Matrix2<f64> {
    let iso = Matrix2::identity() / (sigma_m * sigma_m);
    let Some(s) = translation_information(h) else {
        return iso;
    };
    let e = SymmetricEigen::new(s);
    let max = e.eigenvalues.max();
    if max <= 0.0 || !max.is_finite() {
        return iso;
    }
    let shares = e
        .eigenvalues
        .map(|v| (v / max).clamp(MIN_LOOP_INFORMATION_SHARE, 1.0));
    e.eigenvectors * Matrix2::from_diagonal(&shares) * e.eigenvectors.transpose()
        / (sigma_m * sigma_m)
}

/// Translation information of the odometry edge leaving submap `from`:
/// `1/sigma²` along the best-constrained direction of the scans matched
/// into it, less along weaker ones. Local SLAM drifts where its matches are
/// weak, along a corridor, so loop corrections are taken up there.
fn odometry_information(map: &LocalMap, from: usize, sigma_m: f64) -> Matrix2<f64> {
    let mut h = Matrix3::zeros();
    for nd in map.nodes.iter().filter(|nd| nd.submap == from) {
        if let Some(scan) = &nd.scan {
            h += constraint_information(scan, nd.in_submap);
        }
    }
    loop_information(&h, sigma_m)
}

/// A candidate needs at least this share of the scan in that zone.
const MIN_OBSERVED_SHARE: f64 = 0.3;

/// The part of `scan` (at `prior`, submap frame) that `submap` observed from
/// close by. Far from its scan positions a submap holds only sparse hits, so
/// points landing there score low even when aligned and pull the match
/// towards where the submap was built; scoring the closely observed part
/// only keeps true overlaps scoring like local matches.
fn observed_part(scan: &Scan2d, prior: Pose2, submap: &Submap) -> Option<Scan2d> {
    let keep: Vec<usize> = (0..scan.points.len())
        .filter(|&i| {
            let p = prior.apply([scan.points[i][0] as f64, scan.points[i][1] as f64]);
            submap.distance_to_scans(p) <= OBSERVED_RADIUS_M
        })
        .collect();
    if (keep.len() as f64) < MIN_OBSERVED_SHARE * scan.points.len() as f64 {
        return None;
    }
    Some(Scan2d {
        t_ns: scan.t_ns,
        points: keep.iter().map(|&i| scan.points[i]).collect(),
        t_offset_ns: keep.iter().map(|&i| scan.t_offset_ns[i]).collect(),
    })
}

/// The optimised result.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Trajectory {
    /// One pose per matched scan, time order, in the frame of the first scan.
    pub poses: Vec<PoseOut>,
    /// Submaps built.
    pub submaps: usize,
    /// Loop edges added to the graph.
    pub loops_accepted: usize,
    /// Loop matches found but refused for moving the scan too far from the
    /// graph's prediction.
    pub loops_rejected: usize,
    /// The accepted loop matches.
    pub loops: Vec<LoopClosure>,
    /// Inserted scan points in the optimised frame, one per grid cell.
    pub map_points: Vec<[f32; 2]>,
}

/// One accepted loop match: the scan at `t_ns` lies at `pose` in the frame of
/// submap `submap` (see [`crate::frontend::LocalMap::submaps`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopClosure {
    pub t_ns: u64,
    pub submap: usize,
    pub pose: Pose2,
    /// Branch-and-bound score of the match.
    pub score: f64,
}

/// Search loop constraints in `map`, optimise, and return every scan's pose.
pub fn optimize(map: &LocalMap, cfg: &Scan2dConfig) -> Trajectory {
    let n = map.submaps.len();
    let mut poses: Vec<Pose2> = map.submaps.iter().map(|s| s.pose).collect();
    let mut edges: Vec<Edge> = (1..n)
        .map(|k| {
            Edge::with_information(
                k - 1,
                k,
                poses[k - 1].inverse() * poses[k],
                odometry_information(map, k - 1, cfg.odom_sigma_m),
                cfg.odom_sigma_deg.to_radians(),
                None,
            )
        })
        .collect();
    let window = Window {
        linear_m: cfg.loop_linear_window_m,
        angular_rad: cfg.loop_angular_window_deg.to_radians(),
    };
    let travelled = path_positions(map);
    let mut built_until = vec![0.0f64; n];
    for (nd, s) in map.nodes.iter().zip(&travelled) {
        for b in [nd.submap, nd.submap + 1] {
            if b < n && nd.scan.is_some() {
                built_until[b] = built_until[b].max(*s);
            }
        }
    }
    let mut pyramids: Vec<Option<Pyramid>> = vec![None; n];
    let (mut accepted, mut rejected) = (0, 0);
    let mut loops = Vec::new();
    let every = cfg.loop_every_n_nodes.max(1);
    let inserted = map
        .nodes
        .iter()
        .zip(&travelled)
        .filter_map(|(nd, s)| nd.scan.as_ref().map(|sc| (nd, sc, *s)));
    for (k, (node, full, s_node)) in inserted.enumerate() {
        if k % every != 0 {
            continue;
        }
        let global = poses[node.submap] * node.in_submap;
        let mut added = false;
        for b in 0..n {
            if b + cfg.loop_min_submap_gap > node.submap {
                break;
            }
            let sm = &map.submaps[b];
            let prior = poses[b].inverse() * global;
            if sm.distance_to_scans([prior.x, prior.y]) > cfg.loop_search_radius_m {
                continue;
            }
            let Some(scan) = observed_part(full, prior, sm) else {
                continue;
            };
            let pyr = pyramids[b].get_or_insert_with(|| Pyramid::new(sm.grid(), PYRAMID_DEPTH));
            let Some((m, score)) = branch_and_bound(pyr, &scan, prior, window, cfg.loop_min_score)
            else {
                continue;
            };
            if on_window_edge(prior, m, window, cfg.resolution_m) {
                rejected += 1;
                continue;
            }
            let r = refine(sm.grid(), &scan, m, None, cfg.gn_max_iters, cfg.gn_eps);
            let info = constraint_information(&scan, r.pose);
            if degeneracy(&info) < LOOP_MIN_DEGENERACY {
                continue;
            }
            let corr = prior.inverse() * r.pose;
            let max_m = (MIN_CORRECTION_M + DRIFT_PER_METRE * (s_node - built_until[b]).max(0.0))
                .min(cfg.loop_max_correction_m);
            if corr.x.hypot(corr.y) > max_m
                || corr.th.abs() > cfg.loop_max_correction_deg.to_radians()
            {
                rejected += 1;
                continue;
            }
            accepted += 1;
            added = true;
            loops.push(LoopClosure {
                t_ns: node.t_ns,
                submap: b,
                pose: r.pose,
                score,
            });
            edges.push(Edge::with_information(
                b,
                node.submap,
                r.pose * node.in_submap.inverse(),
                loop_information(&info, cfg.loop_sigma_m),
                cfg.loop_sigma_deg.to_radians(),
                Some(cfg.huber_delta / cfg.loop_sigma_m),
            ));
        }
        if added {
            solve(&mut poses, &edges);
        }
    }
    Trajectory {
        poses: map
            .nodes
            .iter()
            .map(|nd| {
                let p = poses[nd.submap] * nd.in_submap;
                PoseOut {
                    t_ns: nd.t_ns,
                    x: p.x,
                    y: p.y,
                    yaw: p.th,
                    roll: nd.roll,
                    pitch: nd.pitch,
                    score: nd.score,
                    degeneracy: nd.degeneracy,
                }
            })
            .collect(),
        submaps: n,
        loops_accepted: accepted,
        loops_rejected: rejected,
        loops,
        map_points: map_points(map, &poses, cfg.resolution_m),
    }
}

fn map_points(map: &LocalMap, poses: &[Pose2], res: f64) -> Vec<[f32; 2]> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for nd in &map.nodes {
        let Some(scan) = &nd.scan else { continue };
        let p = poses[nd.submap] * nd.in_submap;
        for q in &scan.points {
            let [x, y] = p.apply([q[0] as f64, q[1] as f64]);
            if seen.insert(((x / res).floor() as i64, (y / res).floor() as i64)) {
                out.push([x as f32, y as f32]);
            }
        }
    }
    out
}
