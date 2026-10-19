use ados_capture_contract::{GeoAnchor, GeoAnchorDatum, GeoAnchorSource};

use super::super::align::{align, apply, compose, invert, voxel_representatives, IDENTITY_4X4};
use super::super::change::{align_and_compare, AlignInit};
use super::super::geo::{anchor_transform, enu_offset, enu_rotation};
use super::super::geom::Rng;
use super::super::{PointCloud, ProductError};
use super::jitter;

/// Stratified sampling cell: one random point per cell, so any surface
/// point has a sample within the cell diagonal (4.2 cm).
const SPACING: f64 = 0.03;
const BOX_SIZE: f64 = 0.5;

fn anchor(lat_deg: f64, lon_deg: f64, h_ellipsoid_m: f64) -> GeoAnchor {
    GeoAnchor {
        lat_deg,
        lon_deg,
        h_ellipsoid_m,
        geoid_undulation_m: 0.0,
        datum: GeoAnchorDatum::Wgs84,
        source: GeoAnchorSource::GnssMean,
    }
}

/// Stratified random samples of the rectangle `o + s·u + t·v`,
/// `s ∈ [0, lu]`, `t ∈ [0, lv]` (an irregular pattern, like a real scan).
fn patch(
    out: &mut Vec<[f64; 3]>,
    rng: &mut Rng,
    o: [f64; 3],
    u: [f64; 3],
    v: [f64; 3],
    lu: f64,
    lv: f64,
) {
    let (nu, nv) = (
        (lu / SPACING).round().max(1.0),
        (lv / SPACING).round().max(1.0),
    );
    for i in 0..nu as usize {
        for j in 0..nv as usize {
            let s = (i as f64 + 0.5 + jitter(rng, 0.5)) * lu / nu;
            let t = (j as f64 + 0.5 + jitter(rng, 0.5)) * lv / nv;
            out.push(std::array::from_fn(|k| o[k] + s * u[k] + t * v[k]));
        }
    }
}

const X: [f64; 3] = [1.0, 0.0, 0.0];
const Y: [f64; 3] = [0.0, 1.0, 0.0];
const Z: [f64; 3] = [0.0, 0.0, 1.0];

/// Sides and top of a box standing on the floor at `min`.
fn open_box(out: &mut Vec<[f64; 3]>, rng: &mut Rng, min: [f64; 3], size: [f64; 3]) {
    let [x, y, z] = min;
    let [sx, sy, sz] = size;
    patch(out, rng, [x, y, z], X, Z, sx, sz);
    patch(out, rng, [x, y + sy, z], X, Z, sx, sz);
    patch(out, rng, [x, y, z], Y, Z, sy, sz);
    patch(out, rng, [x + sx, y, z], Y, Z, sy, sz);
    patch(out, rng, [x, y, z + sz], X, Y, sx, sy);
}

/// A 3.5 × 2.5 m room (floor and 1.2 m of wall), a cabinet, a table top and
/// a 0.5 m box at `box_xy`, sampled with `seed`; returns the cloud and the
/// box's point count.
fn room(box_xy: [f64; 2], seed: u64) -> (Vec<[f64; 3]>, usize) {
    let rng = &mut Rng::new(seed);
    let mut p = Vec::new();
    patch(&mut p, rng, [0.0; 3], X, Y, 3.5, 2.5);
    patch(&mut p, rng, [0.0; 3], X, Z, 3.5, 1.2);
    patch(&mut p, rng, [0.0, 2.5, 0.0], X, Z, 3.5, 1.2);
    patch(&mut p, rng, [0.0; 3], Y, Z, 2.5, 1.2);
    patch(&mut p, rng, [3.5, 0.0, 0.0], Y, Z, 2.5, 1.2);
    open_box(&mut p, rng, [2.5, 0.2, 0.0], [0.7, 0.4, 1.0]);
    patch(&mut p, rng, [1.4, 1.5, 0.75], X, Y, 0.8, 0.5);
    let before = p.len();
    open_box(&mut p, rng, [box_xy[0], box_xy[1], 0.0], [BOX_SIZE; 3]);
    let n_box = p.len() - before;
    (p, n_box)
}

fn rz(deg: f64, t: [f64; 3]) -> [f64; 16] {
    let (s, c) = deg.to_radians().sin_cos();
    [
        c, -s, 0.0, t[0], s, c, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

fn cloud(points: Vec<[f64; 3]>, t: &[f64; 16]) -> PointCloud {
    PointCloud {
        points: points.iter().map(|p| apply(t, p)).collect(),
        colors: None,
    }
}

/// Rotation angle (deg) between two transforms and the distance (m) between
/// their images of the `b`-frame point `at`.
fn error(x: &[f64; 16], y: &[f64; 16], at: [f64; 3]) -> (f64, f64) {
    let d = compose(x, &invert(y));
    let tr = d[0] + d[5] + d[10];
    let angle = ((tr - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees();
    let (px, py) = (apply(x, &at), apply(y, &at));
    let dt = ((px[0] - py[0]).powi(2) + (px[1] - py[1]).powi(2) + (px[2] - py[2]).powi(2)).sqrt();
    (angle, dt)
}

const OLD_BOX: [f64; 2] = [0.5, 0.5];
const NEW_BOX: [f64; 2] = [0.4, 1.6];

#[test]
fn enu_offset_matches_independent_fixture() {
    // Reference: PROJ 9.8 `cart` + `topocentric` pipeline (WGS-84).
    let a = anchor(12.9716, 77.5946, 834.6);
    let b = anchor(12.9781, 77.6012, 871.25);
    let want = [716.1599136427963, 719.2048684236452, 36.569009654630264];
    let got = enu_offset(&a, &b);
    for k in 0..3 {
        assert!((got[k] - want[k]).abs() < 1e-3, "{got:?} vs {want:?}");
    }
    assert!(enu_offset(&a, &a).iter().all(|v| v.abs() < 1e-9));
}

#[test]
fn enu_rotation_relates_both_frames() {
    let a = anchor(47.3769, 8.5417, 408.0);
    let b = anchor(47.3801, 8.5552, 431.0);
    let m = enu_rotation(&a, &b);
    // Orthonormal.
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| m[3 * k + i] * m[3 * k + j]).sum();
            let want = if i == j { 1.0 } else { 0.0 };
            assert!((dot - want).abs() < 1e-12);
        }
    }
    // The reverse offset is the forward one seen from b: o_ba = -Mᵀ o_ab.
    let (ab, ba) = (enu_offset(&a, &b), enu_offset(&b, &a));
    for i in 0..3 {
        let back: f64 = -(0..3).map(|k| m[3 * k + i] * ab[k]).sum::<f64>();
        assert!((ba[i] - back).abs() < 1e-6, "{ba:?} vs {back}");
    }
    // Meridian convergence: rotation about up by about Δλ·sin φ.
    let gamma = (b.lon_deg - a.lon_deg).to_radians() * a.lat_deg.to_radians().sin();
    assert!(
        (m[1].abs() - gamma.abs()).abs() < 1e-6,
        "{} vs {gamma}",
        m[1]
    );
    let t = anchor_transform(&a, &b);
    assert_eq!([t[3], t[7], t[11]], ab);
}

#[test]
fn identical_clouds_align_to_identity() {
    let (pts, _) = room(OLD_BOX, 1);
    let a = cloud(pts, &IDENTITY_4X4);
    let res = align(&a, &a, IDENTITY_4X4);
    let (angle, dt) = error(&res.transform, &IDENTITY_4X4, [0.0; 3]);
    assert!(angle < 1e-6 && dt < 1e-6, "{angle} {dt}");
    assert_eq!(res.fitness, 1.0);
    assert!(res.rmse_m < 1e-9);
    assert!(res.iterations <= 2);
}

#[test]
fn voxel_representatives_are_input_points() {
    let pts = vec![
        [0.01, 0.01, 0.0],
        [0.04, 0.04, 0.0],
        [0.02, 0.03, 0.0],
        [0.07, 0.01, 0.0],
    ];
    // Voxel (0,0,0) centroid (0.0233, 0.0267): nearest input is the third.
    assert_eq!(
        voxel_representatives(&pts, 0.05),
        vec![[0.02, 0.03, 0.0], [0.07, 0.01, 0.0]]
    );
}

#[test]
fn local_clouds_register_and_isolate_the_moved_box() {
    let (pa, _) = room(OLD_BOX, 1);
    let a = cloud(pa, &IDENTITY_4X4);
    let t_b_a = rz(4.0, [3.2, -1.1, 0.05]);
    let (pb, n_box) = room(NEW_BOX, 2);
    let b = cloud(pb, &t_b_a);
    let (res, fit) = align_and_compare(&a, None, &b, None, 0.05).unwrap();
    let truth = invert(&t_b_a);
    // The transform's own translation (b's origin is metres from the room).
    let (angle, dt) = error(&fit.transform, &truth, [0.0; 3]);
    assert!(
        angle < 0.1 && dt < 0.01,
        "angle {angle}° offset {dt} m {fit:?}"
    );
    assert!(fit.fitness > 0.9 && fit.rmse_m < 0.03, "{fit:?}");

    let s = &res.summary;
    let al = s.alignment.as_ref().expect("alignment recorded");
    assert_eq!(al.init, AlignInit::Identity);
    assert_eq!(al.transform, fit.transform);
    // Changed points sit on the moved box (in a's frame) and cover its
    // faces clear of the floor.
    let (lo, hi) = (
        [NEW_BOX[0] - 0.05, NEW_BOX[1] - 0.05, 0.0],
        [
            NEW_BOX[0] + BOX_SIZE + 0.05,
            NEW_BOX[1] + BOX_SIZE + 0.05,
            BOX_SIZE + 0.05,
        ],
    );
    let mut changed = 0;
    for (p, &d) in res.cloud.points.iter().zip(&res.distances) {
        if d >= 0.05 {
            changed += 1;
            assert!(
                (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k]),
                "{p:?} at {d}"
            );
        }
    }
    assert_eq!(changed, s.points_changed);
    assert!(changed as f64 >= 0.8 * n_box as f64, "{changed} of {n_box}");

    let json = serde_json::to_value(s).unwrap();
    assert_eq!(json["alignment"]["init"], "identity");
    assert_eq!(json["alignment"]["transform"].as_array().unwrap().len(), 16);
}

#[test]
fn anchored_clouds_start_from_the_geo_offset() {
    let ga = anchor(47.3769, 8.5417, 408.0);
    // 50 m of arc due east along the parallel at the same height (radius
    // (N + h)·cos φ); the chord is shorter by well under a micrometre.
    let n =
        6_378_137.0 / (1.0 - 0.006_694_379_990_14 * ga.lat_deg.to_radians().sin().powi(2)).sqrt();
    let dlon = (50.0 / ((n + ga.h_ellipsoid_m) * ga.lat_deg.to_radians().cos())).to_degrees();
    let gb = anchor(ga.lat_deg, ga.lon_deg + dlon, ga.h_ellipsoid_m);
    let off = enu_offset(&ga, &gb);
    assert!(
        (off[0] - 50.0).abs() < 1e-3 && off[1].abs() < 0.01,
        "{off:?}"
    );

    let (pa, _) = room(OLD_BOX, 1);
    let a = cloud(pa, &IDENTITY_4X4);
    // b's local frame from the anchors, plus a small anchor error.
    let geo = anchor_transform(&ga, &gb);
    let err = rz(0.3, [0.03, -0.02, 0.01]);
    let (pb, _) = room(NEW_BOX, 3);
    let b = cloud(pb, &compose(&err, &invert(&geo)));
    let (res, fit) = align_and_compare(&a, Some(&ga), &b, Some(&gb), 0.05).unwrap();
    let truth = compose(&geo, &invert(&err));
    // b's origin is 50 m away, so the offset is judged at the room centre.
    let centre = apply(&invert(&truth), &[1.75, 1.25, 0.5]);
    let (angle, dt) = error(&fit.transform, &truth, centre);
    assert!(
        angle < 0.1 && dt < 0.01,
        "angle {angle}° offset {dt} m {fit:?}"
    );
    assert_eq!(res.summary.alignment.unwrap().init, AlignInit::Anchor);
}

#[test]
fn unrelated_local_cloud_is_refused() {
    let (pa, _) = room(OLD_BOX, 1);
    let a = cloud(pa, &IDENTITY_4X4);
    let mut rng = Rng::new(7);
    let scatter: Vec<[f64; 3]> = (0..5_000)
        .map(|_| {
            [
                jitter(&mut rng, 5.0),
                jitter(&mut rng, 5.0),
                jitter(&mut rng, 5.0),
            ]
        })
        .collect();
    let b = cloud(scatter, &rz(0.0, [40.0, 40.0, 0.0]));
    let ga = anchor(47.3769, 8.5417, 408.0);
    let err = align_and_compare(&a, Some(&ga), &b, None, 0.05).unwrap_err();
    match err {
        ProductError::FramesUnrelated(msg) => {
            assert!(
                msg.contains("anchored") && msg.contains("local frame"),
                "{msg}"
            );
            assert!(msg.contains("fitness"), "{msg}");
        }
        other => panic!("unexpected {other:?}"),
    }
}
