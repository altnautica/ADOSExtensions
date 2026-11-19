use ados_capture_contract::{GeoAnchor, GeoAnchorDatum, GeoAnchorSource};

use super::super::align::{
    align, align_global, apply, compose, invert, voxel_representatives, IDENTITY_4X4,
};
use super::super::change::{align_and_compare, AlignInit, ChangeResult};
use super::super::geo::{anchor_transform, enu_offset, enu_rotation};
use super::super::geom::Rng;
use super::super::{PointCloud, ProductError};
use super::jitter;

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

const X: [f64; 3] = [1.0, 0.0, 0.0];
const Y: [f64; 3] = [0.0, 1.0, 0.0];
const Z: [f64; 3] = [0.0, 0.0, 1.0];

/// Synthetic scan: stratified random samples (one per `spacing` cell, an
/// irregular pattern like a real scan) of planar surfaces.
struct Scan {
    rng: Rng,
    spacing: f64,
    points: Vec<[f64; 3]>,
}

impl Scan {
    fn new(seed: u64, spacing: f64) -> Self {
        Self {
            rng: Rng::new(seed),
            spacing,
            points: Vec::new(),
        }
    }

    /// The rectangle `o + s·u + t·v`, `s ∈ [0, lu]`, `t ∈ [0, lv]`.
    fn patch(&mut self, o: [f64; 3], u: [f64; 3], v: [f64; 3], lu: f64, lv: f64) {
        let n = |l: f64| (l / self.spacing).round().max(1.0);
        let (nu, nv) = (n(lu), n(lv));
        for i in 0..nu as usize {
            for j in 0..nv as usize {
                let s = (i as f64 + 0.5 + jitter(&mut self.rng, 0.5)) * lu / nu;
                let t = (j as f64 + 0.5 + jitter(&mut self.rng, 0.5)) * lv / nv;
                self.points
                    .push(std::array::from_fn(|k| o[k] + s * u[k] + t * v[k]));
            }
        }
    }

    /// Sides and top of a box standing on the floor at `min`; returns its
    /// point count.
    fn open_box(&mut self, min: [f64; 3], size: [f64; 3]) -> usize {
        let before = self.points.len();
        let [x, y, z] = min;
        let [sx, sy, sz] = size;
        self.patch([x, y, z], X, Z, sx, sz);
        self.patch([x, y + sy, z], X, Z, sx, sz);
        self.patch([x, y, z], Y, Z, sy, sz);
        self.patch([x + sx, y, z], Y, Z, sy, sz);
        self.patch([x, y, z + sz], X, Y, sx, sy);
        self.points.len() - before
    }
}

/// A 3.5 × 2.5 m room (floor and 1.2 m of wall), a cabinet, a table top and
/// a 0.5 m box at `box_xy`, sampled every 3 cm with `seed`; returns the cloud
/// and the box's point count.
fn room(box_xy: [f64; 2], seed: u64) -> (Vec<[f64; 3]>, usize) {
    let mut s = Scan::new(seed, 0.03);
    s.patch([0.0; 3], X, Y, 3.5, 2.5);
    s.patch([0.0; 3], X, Z, 3.5, 1.2);
    s.patch([0.0, 2.5, 0.0], X, Z, 3.5, 1.2);
    s.patch([0.0; 3], Y, Z, 2.5, 1.2);
    s.patch([3.5, 0.0, 0.0], Y, Z, 2.5, 1.2);
    s.open_box([2.5, 0.2, 0.0], [0.7, 0.4, 1.0]);
    s.patch([1.4, 1.5, 0.75], X, Y, 0.8, 0.5);
    let n_box = s.open_box([box_xy[0], box_xy[1], 0.0], [BOX_SIZE; 3]);
    (s.points, n_box)
}

/// A pallet with goods stacked 1.5 m high.
const PALLET: [f64; 3] = [1.2, 1.0, 1.5];

/// A 14 × 9 m warehouse sampled every 15 cm: floor, 1.5 m walls with a
/// doorway in the east wall, four rack rows of different lengths, an office
/// block and a pallet at `pallet_xy`; returns the cloud and the pallet's
/// point count.
fn warehouse(pallet_xy: [f64; 2], seed: u64) -> (Vec<[f64; 3]>, usize) {
    let mut s = Scan::new(seed, 0.15);
    s.patch([0.0; 3], X, Y, 14.0, 9.0);
    s.patch([0.0; 3], X, Z, 14.0, 1.5);
    s.patch([0.0, 9.0, 0.0], X, Z, 14.0, 1.5);
    s.patch([0.0; 3], Y, Z, 9.0, 1.5);
    s.patch([14.0, 0.0, 0.0], Y, Z, 3.0, 1.5);
    s.patch([14.0, 6.0, 0.0], Y, Z, 3.0, 1.5);
    for (y, x0, len) in [
        (1.5, 2.0, 8.0),
        (3.5, 2.0, 10.0),
        (5.5, 3.0, 6.0),
        (7.0, 2.0, 9.0),
    ] {
        s.open_box([x0, y, 0.0], [len, 1.0, 2.0]);
    }
    s.open_box([11.5, 7.0, 0.0], [2.0, 1.8, 2.5]);
    let n_pallet = s.open_box([pallet_xy[0], pallet_xy[1], 0.0], PALLET);
    (s.points, n_pallet)
}

/// A 12 × 8 m hall sampled every 15 cm whose layout is point-symmetric under
/// a half turn about its centre (6, 4): closed 1.5 m walls, two 8 m rack rows
/// mirroring each other and a centred 6 m row. Nothing tells the two
/// headings apart.
fn symmetric_hall(seed: u64) -> Vec<[f64; 3]> {
    let mut s = Scan::new(seed, 0.15);
    s.patch([0.0; 3], X, Y, 12.0, 8.0);
    s.patch([0.0; 3], X, Z, 12.0, 1.5);
    s.patch([0.0, 8.0, 0.0], X, Z, 12.0, 1.5);
    s.patch([0.0; 3], Y, Z, 8.0, 1.5);
    s.patch([12.0, 0.0, 0.0], Y, Z, 8.0, 1.5);
    s.open_box([2.0, 1.5, 0.0], [8.0, 1.0, 2.0]);
    s.open_box([2.0, 5.5, 0.0], [8.0, 1.0, 2.0]);
    s.open_box([3.0, 3.5, 0.0], [6.0, 1.0, 2.0]);
    s.points
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
    assert_eq!(al.init, AlignInit::YawSearch);
    assert_eq!(al.transform, fit.transform);
    assert_change_is_the_box(&res, NEW_BOX, [BOX_SIZE; 3], n_box, 0.05);

    let json = serde_json::to_value(s).unwrap();
    assert_eq!(json["alignment"]["init"], "yaw_search");
    assert_eq!(json["alignment"]["transform"].as_array().unwrap().len(), 16);
}

/// Every changed point (at least `threshold` from `a`) sits on the box at
/// `xy` of `size` (in `a`'s frame), and they cover most of its `n` points
/// (all but the band near the floor).
fn assert_change_is_the_box(
    res: &ChangeResult,
    xy: [f64; 2],
    size: [f64; 3],
    n: usize,
    threshold: f64,
) {
    let m = 0.05;
    let lo = [xy[0] - m, xy[1] - m, 0.0];
    let hi = [xy[0] + size[0] + m, xy[1] + size[1] + m, size[2] + m];
    let mut changed = 0;
    for (p, &d) in res.cloud.points.iter().zip(&res.distances) {
        if d as f64 >= threshold {
            changed += 1;
            assert!(
                (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k]),
                "{p:?} at {d}"
            );
        }
    }
    assert_eq!(changed, res.summary.points_changed);
    assert!(changed as f64 >= 0.8 * n as f64, "{changed} of {n}");
}

#[test]
fn warehouse_with_large_yaw_is_found_by_the_heading_search() {
    // Both pallet spots stand clear (≥ 0.5 m) of every other surface.
    let (old, new) = ([12.5, 5.0], [10.0, 5.0]);
    let (pa, _) = warehouse(old, 1);
    let a = cloud(pa, &IDENTITY_4X4);
    let t_b_a = rz(37.0, [5.0, -2.0, 0.0]);
    let (pb, n_pallet) = warehouse(new, 2);
    let b = cloud(pb, &t_b_a);
    let started = std::time::Instant::now();
    let (res, fit) = align_and_compare(&a, None, &b, None, 0.25).unwrap();
    let elapsed = started.elapsed();
    let truth = invert(&t_b_a);
    // Offset judged at the warehouse centre.
    let centre = apply(&t_b_a, &[7.0, 4.5, 1.0]);
    let (angle, dt) = error(&fit.transform, &truth, centre);
    assert!(
        angle < 0.2 && dt < 0.02,
        "angle {angle}° offset {dt} m {fit:?} in {elapsed:?}"
    );
    // Sparse sampling: a share of the points sit just beyond the 10 cm
    // fitness radius from the other scan.
    assert!(fit.fitness > 0.8, "{fit:?}");
    assert_eq!(
        res.summary.alignment.as_ref().unwrap().init,
        AlignInit::YawSearch
    );
    // 15 cm sampling: two scans of one surface sit up to ~20 cm apart, so
    // the change threshold is 25 cm.
    assert_change_is_the_box(&res, new, PALLET, n_pallet, 0.25);
}

#[test]
fn half_turn_symmetric_hall_is_ambiguous() {
    let a = cloud(symmetric_hall(1), &IDENTITY_4X4);
    let b = cloud(symmetric_hall(2), &rz(183.0, [4.0, -1.0, 0.0]));
    match align_and_compare(&a, None, &b, None, 0.25) {
        Err(ProductError::AlignAmbiguous(msg)) => {
            assert!(msg.contains("yaw") && msg.contains("fitness"), "{msg}");
        }
        Err(other) => panic!("unexpected {other:?}"),
        Ok((_, fit)) => panic!("aligned despite the symmetry: {fit:?}"),
    }
    // The same through the standalone entry point.
    assert!(matches!(
        align_global(&a, &b),
        Err(ProductError::AlignAmbiguous(_))
    ));
}

#[test]
fn asymmetric_warehouse_has_one_heading() {
    let (pa, _) = warehouse([12.5, 5.0], 1);
    let a = cloud(pa, &IDENTITY_4X4);
    let t_b_a = rz(183.0, [4.0, -1.0, 0.0]);
    let (pb, _) = warehouse([12.5, 5.0], 2);
    let b = cloud(pb, &t_b_a);
    let fit = align_global(&a, &b).expect("doorway and office block break the symmetry");
    let centre = apply(&t_b_a, &[7.0, 4.5, 1.0]);
    let (angle, dt) = error(&fit.transform, &invert(&t_b_a), centre);
    assert!(
        angle < 0.2 && dt < 0.02,
        "angle {angle}° offset {dt} m {fit:?}"
    );
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
