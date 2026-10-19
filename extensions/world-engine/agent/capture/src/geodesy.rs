//! Exact WGS-84 geodesy: geodetic → ECEF → local ENU at a [`GeoAnchor`], and
//! the NED frame of a flight controller's EKF origin.
//!
//! Heights are ellipsoidal throughout; an MSL height becomes ellipsoidal by
//! adding the geoid undulation ([`crate::geo::geoid_undulation_m`]).

use ados_capture_contract::GeoAnchor;

/// WGS-84 semi-major axis, metres.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS-84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// First eccentricity squared.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// ECEF metres of a geodetic position (degrees, ellipsoidal height).
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, h_m: f64) -> [f64; 3] {
    let (sl, cl) = lat_deg.to_radians().sin_cos();
    let (so, co) = lon_deg.to_radians().sin_cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sl * sl).sqrt();
    [
        (n + h_m) * cl * co,
        (n + h_m) * cl * so,
        (n * (1.0 - WGS84_E2) + h_m) * sl,
    ]
}

/// Row-major rotation ENU-from-ECEF at a geodetic origin.
pub fn r_enu_ecef(lat_deg: f64, lon_deg: f64) -> [f64; 9] {
    let (sl, cl) = lat_deg.to_radians().sin_cos();
    let (so, co) = lon_deg.to_radians().sin_cos();
    [-so, co, 0.0, -sl * co, -sl * so, cl, cl * co, cl * so, sl]
}

/// The local ENU frame at an anchor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnuFrame {
    origin_ecef: [f64; 3],
    r: [f64; 9],
}

impl EnuFrame {
    pub fn at(lat_deg: f64, lon_deg: f64, h_m: f64) -> Self {
        Self {
            origin_ecef: geodetic_to_ecef(lat_deg, lon_deg, h_m),
            r: r_enu_ecef(lat_deg, lon_deg),
        }
    }

    pub fn from_anchor(a: &GeoAnchor) -> Self {
        Self::at(a.lat_deg, a.lon_deg, a.h_ellipsoid_m)
    }

    /// ENU metres of an ECEF point.
    pub fn ecef_to_enu(&self, p: [f64; 3]) -> [f64; 3] {
        let d = [
            p[0] - self.origin_ecef[0],
            p[1] - self.origin_ecef[1],
            p[2] - self.origin_ecef[2],
        ];
        crate::pose::se3::mat_vec(&self.r, d)
    }

    /// ENU metres of a geodetic point.
    pub fn geodetic_to_enu(&self, lat_deg: f64, lon_deg: f64, h_m: f64) -> [f64; 3] {
        self.ecef_to_enu(geodetic_to_ecef(lat_deg, lon_deg, h_m))
    }

    /// ECEF of an ENU offset from this frame's origin.
    pub fn enu_to_ecef(&self, enu: [f64; 3]) -> [f64; 3] {
        let d = crate::pose::se3::mat_vec(&crate::pose::se3::transpose(&self.r), enu);
        [
            self.origin_ecef[0] + d[0],
            self.origin_ecef[1] + d[1],
            self.origin_ecef[2] + d[2],
        ]
    }

    /// Rotation from another ENU frame's axes to this one's (the two differ
    /// by the convergence of the meridians between their origins).
    pub fn r_from(&self, other: &EnuFrame) -> [f64; 9] {
        crate::pose::se3::mat_mul(&self.r, &crate::pose::se3::transpose(&other.r))
    }
}

/// Row-major rotation ENU-from-NED (swap north/east, negate down).
pub const R_ENU_NED: [f64; 9] = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0];

/// Row-major rotation FRD-from-FLU (and its own inverse).
pub const R_FRD_FLU: [f64; 9] = [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0];

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 3], b: [f64; 3], tol: f64) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < tol, "{a:?} vs {b:?}");
        }
    }

    // Fixture values computed independently with the closed-form WGS-84
    // formulas in double precision (Python), not with this module.
    #[test]
    fn geodetic_to_ecef_matches_fixtures() {
        close(geodetic_to_ecef(0.0, 0.0, 0.0), [WGS84_A, 0.0, 0.0], 1e-6);
        close(
            geodetic_to_ecef(90.0, 0.0, 0.0),
            [0.0, 0.0, 6_356_752.314_245],
            1e-5,
        );
        close(
            geodetic_to_ecef(45.0, 45.0, 0.0),
            [3_194_419.145_061, 3_194_419.145_061, 4_487_348.408_866],
            1e-5,
        );
        close(
            geodetic_to_ecef(12.9716, 77.5946, 920.4),
            [1_335_650.877_933, 6_072_159.781_338, 1_422_550.258_559],
            1e-5,
        );
        close(
            geodetic_to_ecef(-33.8688, 151.2093, 58.0),
            [-4_646_093.477_288, 2_553_229.535_817, -3_534_404.710_910],
            1e-5,
        );
    }

    #[test]
    fn enu_offset_matches_fixture_and_round_trips() {
        let f = EnuFrame::at(12.9716, 77.5946, 920.4);
        let enu = f.geodetic_to_enu(12.9725, 77.5960, 925.0);
        close(enu, [151.917_389, 99.582_170, 4.597_409], 1e-5);
        let back = f.ecef_to_enu(f.enu_to_ecef([10.0, -20.0, 3.0]));
        close(back, [10.0, -20.0, 3.0], 1e-8);
        close(f.geodetic_to_enu(12.9716, 77.5946, 920.4), [0.0; 3], 1e-8);
    }

    #[test]
    fn frames_at_one_origin_share_axes() {
        let a = EnuFrame::at(10.0, 20.0, 0.0);
        let r = a.r_from(&EnuFrame::at(10.0, 20.0, 50.0));
        for (i, v) in r.iter().enumerate() {
            let want = if i % 4 == 0 { 1.0 } else { 0.0 };
            assert!((v - want).abs() < 1e-12);
        }
    }
}
