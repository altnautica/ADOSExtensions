//! Geoid undulation N (EGM2008), so an anchor built from a flight
//! controller's MSL altitude states a true ellipsoidal height:
//! `h_ellipsoid = H_msl + N`.
//!
//! Data: EGM2008 geoid heights, NGA, public domain — the 1° nodes of the
//! 5-arc-minute EGM2008 grid as distributed for GeographicLib, stored as
//! little-endian `i16` centimetres in `egm2008_1deg.bin` (181 rows from 90°N
//! to 90°S, 360 columns from 0°E eastward; 130 KB). Bilinear interpolation on
//! this grid is within 1 m of EGM2008 over 95 % of the globe (median 0.12 m);
//! the worst cells (the steepest geoid gradients) reach
//! several metres, which an anchor height tolerates.

use ados_capture_contract::{GeoAnchor, GeoAnchorDatum, GeoAnchorSource};

/// 3D fixes averaged into a session anchor: about 10 s of the ~10 Hz vehicle
/// state, so the anchor is a mean position, not one fix's noise.
pub const ANCHOR_FIXES: u32 = 100;

/// Accumulates 3D fixes and latches the session anchor at their mean once
/// [`ANCHOR_FIXES`] have arrived. Longitudes are averaged as offsets from the
/// first fix so a session across the antimeridian does not average to 0°.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct AnchorLatch {
    first: Option<(f64, f64)>,
    sum: [f64; 3],
    n: u32,
    anchor: Option<GeoAnchor>,
}

fn wrap180(deg: f64) -> f64 {
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

impl AnchorLatch {
    /// Fold in one 3D fix (`alt_msl_m` above mean sea level) and return the
    /// anchor once latched.
    pub fn observe(&mut self, lat_deg: f64, lon_deg: f64, alt_msl_m: f64) -> Option<GeoAnchor> {
        if self.anchor.is_none() {
            let (lat0, lon0) = *self.first.get_or_insert((lat_deg, lon_deg));
            self.sum[0] += lat_deg - lat0;
            self.sum[1] += wrap180(lon_deg - lon0);
            self.sum[2] += alt_msl_m;
            self.n += 1;
            if self.n >= ANCHOR_FIXES {
                let n = f64::from(self.n);
                let lat = lat0 + self.sum[0] / n;
                let lon = lon0 + self.sum[1] / n;
                // Normalise only when the mean crossed ±180°, so a session
                // that never moved keeps its exact longitude.
                let lon = if (-180.0..=180.0).contains(&lon) {
                    lon
                } else {
                    wrap180(lon)
                };
                let undulation = geoid_undulation_m(lat, lon);
                self.anchor = Some(GeoAnchor {
                    lat_deg: lat,
                    lon_deg: lon,
                    h_ellipsoid_m: self.sum[2] / n + undulation,
                    geoid_undulation_m: undulation,
                    datum: GeoAnchorDatum::Wgs84,
                    source: GeoAnchorSource::GnssMean,
                });
            }
        }
        self.anchor.clone()
    }

    /// The latched anchor, if any.
    pub fn anchor(&self) -> Option<GeoAnchor> {
        self.anchor.clone()
    }
}

const ROWS: usize = 181;
const COLS: usize = 360;
const GRID: &[u8; ROWS * COLS * 2] = include_bytes!("egm2008_1deg.bin");

/// The grid node at `row` (0 = 90°N) and `col` (0 = 0°E), metres.
fn node(row: usize, col: usize) -> f64 {
    let i = (row * COLS + col % COLS) * 2;
    f64::from(i16::from_le_bytes([GRID[i], GRID[i + 1]])) / 100.0
}

/// EGM2008 geoid undulation at a WGS84 position, metres (bilinear).
pub fn geoid_undulation_m(lat_deg: f64, lon_deg: f64) -> f64 {
    let y = (90.0 - lat_deg.clamp(-90.0, 90.0)).min(179.999_999);
    let x = lon_deg.rem_euclid(360.0);
    let (r, c) = (y.floor() as usize, x.floor() as usize);
    let (fy, fx) = (y - r as f64, x - c as f64);
    let top = node(r, c) * (1.0 - fx) + node(r, c + 1) * fx;
    let bottom = node(r + 1, c) * (1.0 - fx) + node(r + 1, c + 1) * fx;
    top * (1.0 - fy) + bottom * fy
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference undulations from GeographicLib GeoidEval on the EGM2008
    /// 5-arc-minute grid; this table must agree within 1 m.
    #[test]
    fn matches_reference_undulations_within_a_metre() {
        for (lat, lon, n) in [
            (12.9716, 77.5946, -86.16),  // Bengaluru
            (28.6139, 77.2090, -52.55),  // New Delhi
            (51.4779, -0.0015, 45.90),   // Greenwich
            (40.7128, -74.0060, -32.74), // New York
            (-33.8688, 151.2093, 22.34), // Sydney
        ] {
            let got = geoid_undulation_m(lat, lon);
            assert!((got - n).abs() < 1.0, "({lat}, {lon}): {got} vs {n}");
        }
    }

    #[test]
    fn wraps_longitude_and_clamps_the_poles() {
        let a = geoid_undulation_m(12.9716, 77.5946);
        assert!((a - geoid_undulation_m(12.9716, 77.5946 - 360.0)).abs() < 1e-9);
        assert!(geoid_undulation_m(90.0, 10.0).is_finite());
        assert!(geoid_undulation_m(-90.0, 10.0).is_finite());
    }

    #[test]
    fn the_anchor_is_the_mean_of_the_first_fixes_with_ellipsoidal_height() {
        let mut latch = AnchorLatch::default();
        for i in 0..ANCHOR_FIXES - 1 {
            let jitter = if i % 2 == 0 { 1e-5 } else { -1e-5 };
            assert!(latch
                .observe(12.9716 + jitter, 77.5946 - jitter, 900.0)
                .is_none());
        }
        let a = latch.observe(12.9716, 77.5946, 900.0).expect("latched");
        assert!((a.lat_deg - 12.9716).abs() < 1e-6 && (a.lon_deg - 77.5946).abs() < 1e-6);
        assert!((a.geoid_undulation_m - (-86.16)).abs() < 1.0);
        assert!((a.h_ellipsoid_m - (900.0 + a.geoid_undulation_m)).abs() < 1e-9);
        assert_eq!(a.source, GeoAnchorSource::GnssMean);
        // Later fixes never move a latched anchor.
        assert_eq!(latch.observe(13.5, 78.0, 1000.0), Some(a));
    }

    #[test]
    fn longitudes_across_the_antimeridian_average_correctly() {
        let mut latch = AnchorLatch::default();
        for i in 0..ANCHOR_FIXES {
            latch.observe(0.0, if i % 2 == 0 { 179.9999 } else { -179.9999 }, 0.0);
        }
        assert!(latch.anchor().unwrap().lon_deg.abs() > 179.99);
    }
}
