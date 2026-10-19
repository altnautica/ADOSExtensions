//! Rack-row and aisle detection for warehouse scans. Rack rows are dense
//! vertical slabs (points 0.2–6 m above the floor) sharing one orientation;
//! an aisle is the free corridor between two consecutive rows.

use serde_json::{json, Value};

use super::cloud::PointCloud;
use super::geom::dedupe2d;

/// One aisle in the local frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Aisle {
    pub id: u32,
    pub centerline: [[f64; 2]; 2],
    pub width_m: f64,
    pub length_m: f64,
}

/// Slab height band above the floor.
const BAND: (f64, f64) = (0.2, 6.0);
/// Across-row profile bin.
const BIN: f64 = 0.05;
/// Along-row coverage bin.
const ALONG: f64 = 0.25;
/// Thinnest slab accepted as a rack row (a single wall face is thinner).
const MIN_ROW_DEPTH: f64 = 0.25;
/// Narrowest and widest corridor reported as an aisle.
const AISLE_RANGE: (f64, f64) = (0.6, 8.0);
/// Shortest shared row length for an aisle.
const MIN_AISLE_LEN: f64 = 1.0;
/// Gaps inside one row (flue space between back-to-back racks) up to this
/// width do not split it.
const JOIN_GAP: f64 = 0.5;

/// Orientation (radians, `[0, π)`) of the row direction that makes the
/// across-row profile sharpest.
fn row_angle(cells: &[[f64; 2]]) -> f64 {
    let score = |a: f64| {
        let (s, c) = a.sin_cos();
        let mut hist: std::collections::HashMap<i64, u32> = std::collections::HashMap::new();
        for p in cells {
            let v = -s * p[0] + c * p[1];
            *hist.entry((v / 0.1).floor() as i64).or_default() += 1;
        }
        hist.values().map(|&n| (n as f64).powi(2)).sum::<f64>()
    };
    let best_of = |angles: &mut dyn Iterator<Item = f64>| {
        angles
            .map(|a| (a, score(a)))
            .max_by(|x, y| x.1.total_cmp(&y.1))
            .map_or(0.0, |(a, _)| a)
    };
    let coarse = best_of(&mut (0..180).map(|d| (d as f64).to_radians()));
    best_of(&mut (-10..=10).map(|k| coarse + (k as f64 * 0.1).to_radians()))
}

/// Slab across-row interval `[v0, v1]` and along-row interval `[u0, u1]`.
struct Row {
    v: [f64; 2],
    u: [f64; 2],
}

fn percentile(v: &mut [f64], q: f64) -> f64 {
    v.sort_unstable_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * q).round() as usize]
}

/// Detects aisles between rack rows standing on the floor at `floor_z`.
pub fn detect(cloud: &PointCloud, floor_z: f64) -> Vec<Aisle> {
    let band: Vec<[f64; 2]> = cloud
        .points
        .iter()
        .filter(|p| (BAND.0..=BAND.1).contains(&(p[2] - floor_z)))
        .map(|p| [p[0], p[1]])
        .collect();
    if band.len() < 100 {
        return Vec::new();
    }
    let cells = dedupe2d(band.iter().copied(), 0.1);
    let angle = row_angle(&cells);
    let (s, c) = angle.sin_cos();
    let uv: Vec<[f64; 2]> = band
        .iter()
        .map(|p| [c * p[0] + s * p[1], -s * p[0] + c * p[1]])
        .collect();

    // Coverage of each across-row bin: distinct along-row bins occupied.
    let vmin = uv.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let vmax = uv.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    let nbins = ((vmax - vmin) / BIN) as usize + 1;
    let mut occupied: Vec<std::collections::HashSet<i64>> = vec![Default::default(); nbins];
    for p in &uv {
        occupied[((p[1] - vmin) / BIN) as usize].insert((p[0] / ALONG).floor() as i64);
    }
    let coverage: Vec<usize> = occupied.iter().map(|s| s.len()).collect();
    let peak = coverage.iter().copied().max().unwrap_or(0);
    if peak == 0 {
        return Vec::new();
    }
    let dense = |b: usize| coverage[b] as f64 >= 0.4 * peak as f64;

    // Contiguous dense bins, joined across gaps narrower than an aisle.
    let mut spans: Vec<[usize; 2]> = Vec::new();
    for b in 0..nbins {
        if !dense(b) {
            continue;
        }
        match spans.last_mut() {
            Some(last) if (b - last[1] - 1) as f64 * BIN <= JOIN_GAP => last[1] = b,
            _ => spans.push([b, b]),
        }
    }
    let mut rows: Vec<Row> = Vec::new();
    for [b0, b1] in spans {
        let (lo, hi) = (vmin + b0 as f64 * BIN, vmin + (b1 + 1) as f64 * BIN);
        let members: Vec<&[f64; 2]> = uv.iter().filter(|p| p[1] >= lo && p[1] < hi).collect();
        let mut vs: Vec<f64> = members.iter().map(|p| p[1]).collect();
        let mut us: Vec<f64> = members.iter().map(|p| p[0]).collect();
        let v = [percentile(&mut vs, 0.005), percentile(&mut vs, 0.995)];
        if v[1] - v[0] < MIN_ROW_DEPTH {
            continue;
        }
        rows.push(Row {
            v,
            u: [percentile(&mut us, 0.01), percentile(&mut us, 0.99)],
        });
    }

    let to_xy = |u: f64, v: f64| [c * u - s * v, s * u + c * v];
    let mut aisles = Vec::new();
    for pair in rows.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let width = b.v[0] - a.v[1];
        let (u0, u1) = (a.u[0].max(b.u[0]), a.u[1].min(b.u[1]));
        if !(AISLE_RANGE.0..=AISLE_RANGE.1).contains(&width) || u1 - u0 < MIN_AISLE_LEN {
            continue;
        }
        let vm = (a.v[1] + b.v[0]) / 2.0;
        aisles.push(Aisle {
            id: aisles.len() as u32 + 1,
            centerline: [to_xy(u0, vm), to_xy(u1, vm)],
            width_m: width,
            length_m: u1 - u0,
        });
    }
    aisles
}

/// GeoJSON FeatureCollection of aisle centerlines in the local ENU frame.
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
