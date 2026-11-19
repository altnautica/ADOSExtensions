//! Exact WGS-84 relations between two local ENU frames: geodetic → ECEF →
//! ENU with ellipsoidal heights.

use ados_capture_contract::GeoAnchor;

/// WGS-84 semi-major axis (m).
const WGS84_A: f64 = 6_378_137.0;
/// WGS-84 flattening.
const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// Earth-centred, earth-fixed position of the anchor origin.
fn ecef(anchor: &GeoAnchor) -> [f64; 3] {
    let e2 = WGS84_F * (2.0 - WGS84_F);
    let (sl, cl) = anchor.lat_deg.to_radians().sin_cos();
    let (so, co) = anchor.lon_deg.to_radians().sin_cos();
    let n = WGS84_A / (1.0 - e2 * sl * sl).sqrt();
    let h = anchor.h_ellipsoid_m;
    [
        (n + h) * cl * co,
        (n + h) * cl * so,
        (n * (1.0 - e2) + h) * sl,
    ]
}

/// Row-major rotation taking ECEF vectors into the anchor's ENU axes (rows
/// are the east, north and up unit vectors in ECEF).
fn enu_from_ecef(anchor: &GeoAnchor) -> [f64; 9] {
    let (sl, cl) = anchor.lat_deg.to_radians().sin_cos();
    let (so, co) = anchor.lon_deg.to_radians().sin_cos();
    [-so, co, 0.0, -sl * co, -sl * so, cl, cl * co, cl * so, sl]
}

/// Position of `b`'s anchor origin in `a`'s local ENU frame (m).
pub fn enu_offset(a: &GeoAnchor, b: &GeoAnchor) -> [f64; 3] {
    let (pa, pb) = (ecef(a), ecef(b));
    let d = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    let r = enu_from_ecef(a);
    std::array::from_fn(|i| r[3 * i] * d[0] + r[3 * i + 1] * d[1] + r[3 * i + 2] * d[2])
}

/// Row-major rotation taking `b`-ENU vectors into `a`-ENU: its columns are
/// `b`'s east, north and up axes expressed in `a`. Differs from identity by
/// the meridian convergence and the change of vertical between the anchors.
pub fn enu_rotation(a: &GeoAnchor, b: &GeoAnchor) -> [f64; 9] {
    let (ra, rb) = (enu_from_ecef(a), enu_from_ecef(b));
    // ra · rbᵀ
    std::array::from_fn(|k| {
        let (i, j) = (k / 3, k % 3);
        (0..3).map(|m| ra[3 * i + m] * rb[3 * j + m]).sum()
    })
}

/// Row-major 4×4 rigid transform mapping `b`-ENU points into `a`-ENU
/// (`T_a_b`).
pub fn anchor_transform(a: &GeoAnchor, b: &GeoAnchor) -> [f64; 16] {
    let r = enu_rotation(a, b);
    let t = enu_offset(a, b);
    [
        r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}
