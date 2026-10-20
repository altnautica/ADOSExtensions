//! Aisle and cross-aisle detection for warehouse scans. The floor between
//! 0.2 m and 2.0 m is rasterised; a cell is free where the floor was seen and
//! nothing stands. The free space is thinned to its medial axis (Zhang–Suen),
//! the axis is split into straight runs where its direction turns by more than
//! 20°, and a run of at least 2 m is a corridor. A corridor crossing at least
//! two others at more than 60°, more of them than run parallel to it, is a
//! cross-aisle; the rest are aisles.

use serde_json::{json, Value};

use super::cloud::PointCloud;
use super::geom::{angle_diff, Rng};
use super::grid2d::{distance_to_blocked, square_filter};
use super::skeleton::thin;
use super::walls::{extract_lines, merge_collinear, WallParams};

/// Whether a corridor runs along the racks or across them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AisleKind {
    Aisle,
    CrossAisle,
}

impl AisleKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Aisle => "aisle",
            Self::CrossAisle => "cross_aisle",
        }
    }
}

/// One corridor in the local frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Aisle {
    pub id: u32,
    pub kind: AisleKind,
    pub centerline: [[f64; 2]; 2],
    pub width_m: f64,
    pub length_m: f64,
}

/// Obstacle band above the floor, metres.
const BAND: (f64, f64) = (0.2, 2.0);
/// Raster cell, metres.
const CELL: f64 = 0.1;
/// Floor points count within this height of the floor, metres.
const FLOOR_TOL: f64 = 0.1;
/// Gaps up to twice this many cells between floor samples close.
const FLOOR_CLOSE: usize = 3;
/// An axis cell stays when its clearance is at least this share of the
/// largest axis clearance within `PRUNE_RADIUS_M`: the spurs thinning grows
/// into corners thin out toward them, a corridor's axis does not.
const PRUNE_SHARE: f64 = 0.5;
const PRUNE_RADIUS_M: f64 = 1.5;
/// Shortest corridor reported, metres.
const MIN_RUN_M: f64 = 2.0;
/// The axis bends into a new run beyond this turn.
const MAX_TURN_DEG: f64 = 20.0;
/// Narrowest and widest corridor reported, metres.
const WIDTH_RANGE: (f64, f64) = (0.6, 8.0);
/// Two corridors cross when their angle exceeds this.
const CROSS_DEG: f64 = 60.0;
/// Corridor ends are extended by this much when testing a crossing, metres.
const CROSS_REACH_M: f64 = 1.0;

/// The free-space raster of one floor.
struct FreeGrid {
    origin: [f64; 2],
    w: usize,
    h: usize,
    free: Vec<bool>,
}

fn free_grid(cloud: &PointCloud, floor_z: f64) -> Option<FreeGrid> {
    let (mut band, mut floor) = (Vec::new(), Vec::new());
    for p in &cloud.points {
        let h = p[2] - floor_z;
        if (BAND.0..=BAND.1).contains(&h) {
            band.push([p[0], p[1]]);
        } else if h.abs() <= FLOOR_TOL {
            floor.push([p[0], p[1]]);
        }
    }
    if band.len() < 100 || floor.is_empty() {
        return None;
    }
    let (mut min, mut max) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in band.iter().chain(&floor) {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    }
    let pad = (FLOOR_CLOSE + 1) as f64 * CELL;
    let origin = [min[0] - pad, min[1] - pad];
    let w = ((max[0] - min[0] + 2.0 * pad) / CELL).ceil() as usize + 1;
    let h = ((max[1] - min[1] + 2.0 * pad) / CELL).ceil() as usize + 1;
    let index = |p: &[f64; 2]| {
        let i = ((p[0] - origin[0]) / CELL).floor() as usize;
        let j = ((p[1] - origin[1]) / CELL).floor() as usize;
        (i.min(w - 1), j.min(h - 1))
    };
    let mut floor_seen = vec![false; w * h];
    for p in &floor {
        let (i, j) = index(p);
        floor_seen[j * w + i] = true;
    }
    let mut occupied = vec![false; w * h];
    for p in &band {
        let (i, j) = index(p);
        occupied[j * w + i] = true;
    }
    // Closings (dilate, then erode): the seen floor fills the gaps between
    // its samples without spreading past its edge, and an obstacle fills the
    // one-cell gaps a sparse scan leaves inside a rack (which would otherwise
    // thin into branches through it); both keep their outlines.
    let close =
        |m: &[bool], r: usize| square_filter(&square_filter(m, w, h, r, false), w, h, r, true);
    let support = close(&floor_seen, FLOOR_CLOSE);
    let blocked = close(&occupied, 1);
    let free = (0..w * h)
        .map(|k| support[k] && !blocked[k] && !occupied[k])
        .collect();
    Some(FreeGrid { origin, w, h, free })
}

/// The axis without its corner spurs (see [`PRUNE_SHARE`]).
fn prune(axis: &[bool], clearance: &[f64], w: usize, h: usize) -> Vec<bool> {
    let r = (PRUNE_RADIUS_M / CELL).round() as i64;
    (0..w * h)
        .map(|k| {
            if !axis[k] {
                return false;
            }
            let (i, j) = ((k % w) as i64, (k / w) as i64);
            let mut best = 0.0f64;
            for y in (j - r).max(0)..=(j + r).min(h as i64 - 1) {
                for x in (i - r).max(0)..=(i + r).min(w as i64 - 1) {
                    let n = y as usize * w + x as usize;
                    if axis[n] && (x - i).pow(2) + (y - j).pow(2) <= r * r {
                        best = best.max(clearance[n]);
                    }
                }
            }
            clearance[k] >= PRUNE_SHARE * best
        })
        .collect()
}

/// Straight runs `(a, b, width)` of the medial axis.
fn runs(g: &FreeGrid) -> Vec<([f64; 2], [f64; 2], f64)> {
    let clearance = distance_to_blocked(&g.free, g.w, g.h);
    let axis = prune(&thin(&g.free, g.w, g.h), &clearance, g.w, g.h);
    let centre = |k: usize| {
        [
            g.origin[0] + ((k % g.w) as f64 + 0.5) * CELL,
            g.origin[1] + ((k / g.w) as f64 + 0.5) * CELL,
        ]
    };
    let pts: Vec<[f64; 2]> = (0..axis.len()).filter(|&k| axis[k]).map(centre).collect();
    // A straight-line fit with a 3-cell tolerance, which absorbs the axis's
    // wiggle where side corridors join; a turn of more than 20° leaves it
    // within a metre, so the bend starts a new line. Along one line, the axis
    // may break across a junction for up to twice that before a run ends.
    let tol = 3.0 * CELL;
    let prm = WallParams {
        tol,
        min_inliers: (MIN_RUN_M / CELL * 0.75) as usize,
        min_wall_len: MIN_RUN_M,
        run_gap: 2.0 * tol / MAX_TURN_DEG.to_radians().sin(),
        min_run_len: MIN_RUN_M,
        min_run_points: (MIN_RUN_M / CELL * 0.5) as usize,
        iters: 300,
        local_radius: 3.0,
    };
    // Pieces of one corridor found as separate lines join.
    let lines = merge_collinear(
        extract_lines(&pts, &prm, &mut Rng::new(17)),
        5f64.to_radians(),
        tol,
        &prm,
    );
    let clear_at = |p: [f64; 2]| {
        let i = ((p[0] - g.origin[0]) / CELL).floor() as usize;
        let j = ((p[1] - g.origin[1]) / CELL).floor() as usize;
        clearance[j.min(g.h - 1) * g.w + i.min(g.w - 1)] * CELL
    };
    let mut out = Vec::new();
    for l in &lines {
        for r in &l.runs {
            if r[1] - r[0] < MIN_RUN_M {
                continue;
            }
            let mut c: Vec<f64> = l
                .points
                .iter()
                .filter(|p| (r[0]..=r[1]).contains(&l.t_of(**p)))
                .map(|p| clear_at(*p))
                .collect();
            if c.is_empty() {
                continue;
            }
            c.sort_unstable_by(f64::total_cmp);
            out.push((l.at(r[0]), l.at(r[1]), 2.0 * c[c.len() / 2]));
        }
    }
    out
}

fn angle_of(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[1] - a[1]).atan2(b[0] - a[0])
}

/// Whether segments `p` and `q`, each extended by `reach` at both ends,
/// intersect.
fn crosses(p: ([f64; 2], [f64; 2]), q: ([f64; 2], [f64; 2]), reach: f64) -> bool {
    let ext = |(a, b): ([f64; 2], [f64; 2])| {
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2))
            .sqrt()
            .max(1e-9);
        let d = [(b[0] - a[0]) / len * reach, (b[1] - a[1]) / len * reach];
        ([a[0] - d[0], a[1] - d[1]], [b[0] + d[0], b[1] + d[1]])
    };
    let ((a, b), (c, d)) = (ext(p), ext(q));
    let side = |o: [f64; 2], u: [f64; 2], v: [f64; 2]| {
        (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])
    };
    side(a, b, c) * side(a, b, d) < 0.0 && side(c, d, a) * side(c, d, b) < 0.0
}

/// Detects aisles and cross-aisles on the floor at `floor_z`.
pub fn detect(cloud: &PointCloud, floor_z: f64) -> Vec<Aisle> {
    let Some(grid) = free_grid(cloud, floor_z) else {
        return Vec::new();
    };
    let corridors: Vec<_> = runs(&grid)
        .into_iter()
        .filter(|r| (WIDTH_RANGE.0..=WIDTH_RANGE.1).contains(&r.2))
        .collect();
    let angle: Vec<f64> = corridors.iter().map(|c| angle_of(c.0, c.1)).collect();
    let mut found: Vec<Aisle> = corridors
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let others = || (0..corridors.len()).filter(move |&j| j != i);
            let crossing = others()
                .filter(|&j| {
                    angle_diff(angle[i], angle[j]) > CROSS_DEG.to_radians()
                        && crosses((c.0, c.1), (corridors[j].0, corridors[j].1), CROSS_REACH_M)
                })
                .count();
            let parallel = others()
                .filter(|&j| angle_diff(angle[i], angle[j]) <= MAX_TURN_DEG.to_radians())
                .count();
            let kind = if crossing >= 2 && crossing > parallel {
                AisleKind::CrossAisle
            } else {
                AisleKind::Aisle
            };
            Aisle {
                id: 0,
                kind,
                centerline: [c.0, c.1],
                width_m: c.2,
                length_m: ((c.1[0] - c.0[0]).powi(2) + (c.1[1] - c.0[1]).powi(2)).sqrt(),
            }
        })
        .collect();
    // Aisles first, then cross-aisles; each in order of their midpoints.
    let key = |a: &Aisle| {
        let m = [
            (a.centerline[0][0] + a.centerline[1][0]) / 2.0,
            (a.centerline[0][1] + a.centerline[1][1]) / 2.0,
        ];
        (
            a.kind == AisleKind::CrossAisle,
            (m[0] * 10.0).round() as i64,
            (m[1] * 10.0).round() as i64,
        )
    };
    found.sort_by_key(key);
    for (k, a) in found.iter_mut().enumerate() {
        a.id = k as u32 + 1;
    }
    found
}

/// GeoJSON FeatureCollection of corridor centrelines in the local ENU frame.
pub fn to_geojson(aisles: &[Aisle]) -> Value {
    let r = |v: f64| (v * 1000.0).round() / 1000.0;
    let features: Vec<Value> = aisles
        .iter()
        .map(|a| {
            json!({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": a.centerline.iter().map(|p| [r(p[0]), r(p[1])]).collect::<Vec<_>>(),
                },
                "properties": {
                    "id": a.id,
                    "kind": a.kind.as_str(),
                    "width_m": r(a.width_m),
                    "length_m": r(a.length_m),
                },
            })
        })
        .collect();
    json!({
        "type": "FeatureCollection",
        "frame": "enu_local",
        "crs": null,
        "features": features,
    })
}
