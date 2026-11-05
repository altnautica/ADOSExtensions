//! The map's north from GNSS-referenced displacement.
//!
//! `map_ned` must have the axes of the flight controller's `LOCAL_POSITION_NED`.
//! While the EKF navigates on GNSS, ArduPilot (`VISO_TYPE` 1) keeps shifting
//! the external-navigation position onto its own (`align_inactive_sources`), but
//! it never rotates it. When GNSS fails and the EKF switches to external
//! navigation, any heading error in the map frame therefore turns into a
//! position error that grows with distance flown: 3° is 7 m after 137 m. The
//! compass heading the map starts from is typically a few degrees off on the
//! ground (magnetic interference, declination, calibration), so it cannot
//! be the final word.
//!
//! [`HeadingAlign`] fits the yaw that maps the SLAM track (`map_frd`) onto the
//! EKF's GNSS-referenced track over the same instants: the 2D Procrustes
//! rotation of the centred tracks, `ψ = atan2(Σ s×e, Σ s·e)`. Only
//! displacement enters, so neither track's origin nor the compass matters.
//! Samples are taken every [`SAMPLE_STEP_M`] of SLAM travel and weighted
//! down with distance ([`WINDOW_M`]), so the fit follows the SLAM's own slow
//! heading drift. A fit replaces the heading once the track spans
//! [`MIN_BASELINE_M`], both tracks agree in scale and the fit's own yaw
//! uncertainty is below [`MAX_YAW_SIGMA_RAD`].
//!
//! When GNSS stops being usable the window is dropped and the last fit is
//! frozen: from then on the EKF's position comes from this map, so its track
//! says nothing about north. A step where the two tracks disagree by more
//! than [`STEP_JUMP_M`] (an EKF position reset, a SLAM relocalisation, a
//! world-frame change) restarts the window too.

/// SLAM travel between samples, metres.
pub const SAMPLE_STEP_M: f64 = 1.0;
/// Distance over which a sample's weight falls by `e`, metres.
pub const WINDOW_M: f64 = 200.0;
/// Smallest RMS spread of the SLAM track about its centroid for a fit, metres
/// (a straight leg of about 35 m).
pub const MIN_BASELINE_M: f64 = 10.0;
/// Largest relative difference between the two tracks' spreads.
pub const MAX_SCALE_ERR: f64 = 0.2;
/// Largest yaw uncertainty of an accepted fit, radians (0.5°).
pub const MAX_YAW_SIGMA_RAD: f64 = 0.5 * std::f64::consts::PI / 180.0;
/// Largest disagreement of one step's length between the tracks, metres.
pub const STEP_JUMP_M: f64 = 3.0;
/// How long GNSS must have held a 3D fix before the EKF's track is taken as
/// GNSS-referenced, ns. Longer than a companion's switch back from external
/// navigation after GNSS returns (vision-nav: 10 s), so the EKF's own reset
/// back onto GNSS is behind it.
pub const GNSS_SETTLE_NS: u64 = 15_000_000_000;

/// One accepted heading fit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadingFit {
    /// `map_ned = Rz(yaw) · map_frd`, radians.
    pub yaw: f64,
    /// RMS spread of the SLAM track about its centroid, metres.
    pub baseline_m: f64,
    /// RMS residual of the fitted tracks, metres.
    pub residual_m: f64,
    /// The fit's yaw uncertainty, radians.
    pub sigma_rad: f64,
}

/// Weighted sums of the paired tracks: `s` SLAM (`map_frd` x, y), `e` EKF
/// (north, east).
#[derive(Debug, Default, Clone, Copy)]
struct Sums {
    w: f64,
    s: [f64; 2],
    e: [f64; 2],
    dot: f64,
    cross: f64,
    ss: f64,
    ee: f64,
}

impl Sums {
    fn scale(&mut self, k: f64) {
        self.w *= k;
        self.s = [self.s[0] * k, self.s[1] * k];
        self.e = [self.e[0] * k, self.e[1] * k];
        self.dot *= k;
        self.cross *= k;
        self.ss *= k;
        self.ee *= k;
    }

    fn add(&mut self, s: [f64; 2], e: [f64; 2]) {
        self.w += 1.0;
        self.s = [self.s[0] + s[0], self.s[1] + s[1]];
        self.e = [self.e[0] + e[0], self.e[1] + e[1]];
        self.dot += s[0] * e[0] + s[1] * e[1];
        self.cross += s[0] * e[1] - s[1] * e[0];
        self.ss += s[0] * s[0] + s[1] * s[1];
        self.ee += e[0] * e[0] + e[1] * e[1];
    }

    /// The fit of the centred tracks, when it passes every gate.
    fn fit(&self) -> Option<HeadingFit> {
        let w = self.w;
        if w < 3.0 {
            return None;
        }
        let dot = self.dot - (self.s[0] * self.e[0] + self.s[1] * self.e[1]) / w;
        let cross = self.cross - (self.s[0] * self.e[1] - self.s[1] * self.e[0]) / w;
        let vs = self.ss - (self.s[0] * self.s[0] + self.s[1] * self.s[1]) / w;
        let ve = self.ee - (self.e[0] * self.e[0] + self.e[1] * self.e[1]) / w;
        let baseline_m = (vs.max(0.0) / w).sqrt();
        if baseline_m < MIN_BASELINE_M {
            return None;
        }
        let spread_e = (ve.max(0.0) / w).sqrt();
        if (spread_e / baseline_m - 1.0).abs() > MAX_SCALE_ERR {
            return None;
        }
        let residual_m = ((ve + vs - 2.0 * dot.hypot(cross)).max(0.0) / w).sqrt();
        let sigma_rad = residual_m / (baseline_m * w.sqrt());
        if sigma_rad > MAX_YAW_SIGMA_RAD {
            return None;
        }
        Some(HeadingFit {
            yaw: cross.atan2(dot),
            baseline_m,
            residual_m,
            sigma_rad,
        })
    }
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The GNSS heading fit; see the module docs.
#[derive(Debug, Default)]
pub struct HeadingAlign {
    sums: Sums,
    /// The last sample, `(slam, ekf)`; `None` while no window is open.
    last: Option<([f64; 2], [f64; 2])>,
    fit: Option<HeadingFit>,
}

impl HeadingAlign {
    /// The newest accepted fit, frozen while GNSS is unusable.
    pub fn fit(&self) -> Option<HeadingFit> {
        self.fit
    }

    /// Fold one pair taken at the same instant: `s` the SLAM position in
    /// `map_frd` (x, y), `e` the EKF's GNSS-referenced position (north,
    /// east). Returns the fit when this sample produced a new accepted one.
    pub fn observe(&mut self, s: [f64; 2], e: [f64; 2]) -> Option<HeadingFit> {
        let Some((s0, e0)) = self.last else {
            self.restart(s, e);
            return None;
        };
        let ds = dist(s, s0);
        if (ds - dist(e, e0)).abs() > STEP_JUMP_M {
            self.restart(s, e);
            return None;
        }
        if ds < SAMPLE_STEP_M {
            return None;
        }
        self.sums.scale((-ds / WINDOW_M).exp());
        self.sums.add(s, e);
        self.last = Some((s, e));
        let fit = self.sums.fit()?;
        self.fit = Some(fit);
        Some(fit)
    }

    /// GNSS is not usable: drop the window, keep the last fit. Returns
    /// whether a window was open (the first call after GNSS went).
    pub fn freeze(&mut self) -> bool {
        self.sums = Sums::default();
        self.last.take().is_some()
    }

    fn restart(&mut self, s: [f64; 2], e: [f64; 2]) {
        self.sums = Sums::default();
        self.sums.add(s, e);
        self.last = Some((s, e));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Rz(yaw) · s` in the x, y plane.
    fn rot(yaw: f64, s: [f64; 2]) -> [f64; 2] {
        let (sn, c) = yaw.sin_cos();
        [c * s[0] - sn * s[1], sn * s[0] + c * s[1]]
    }

    fn straight(n: usize) -> impl Iterator<Item = [f64; 2]> {
        (0..n).map(|k| [k as f64 * 1.5, 0.2 * k as f64])
    }

    #[test]
    fn the_fit_is_the_rotation_between_the_tracks_whatever_their_origins() {
        let mut h = HeadingAlign::default();
        let yaw = 0.7;
        let mut fit = None;
        for s in straight(60) {
            let e = rot(yaw, s);
            fit = h.observe(s, [e[0] + 400.0, e[1] - 250.0]).or(fit);
        }
        let fit = fit.expect("a 90 m leg is a baseline");
        assert!((fit.yaw - yaw).abs() < 1e-9, "yaw {}", fit.yaw);
        assert!(fit.residual_m < 1e-3, "residual {}", fit.residual_m);
    }

    #[test]
    fn no_fit_before_the_baseline() {
        let mut h = HeadingAlign::default();
        for s in straight(15) {
            assert!(h.observe(s, rot(0.1, s)).is_none());
        }
        assert!(h.fit().is_none(), "a 21 m leg is too short");
    }

    #[test]
    fn gnss_loss_freezes_the_fit_and_a_new_window_needs_its_own_baseline() {
        let mut h = HeadingAlign::default();
        for s in straight(60) {
            h.observe(s, rot(0.05, s));
        }
        let frozen = h.fit().unwrap();
        assert!(h.freeze(), "a window was open");
        assert!(!h.freeze(), "only once");
        // GNSS back with the EKF's track now offset and turned: nothing
        // changes until the new window has a baseline of its own.
        for s in straight(20) {
            h.observe(s, rot(0.3, [s[0] + 5.0, s[1]]));
        }
        assert_eq!(h.fit(), Some(frozen));
    }

    #[test]
    fn a_position_jump_restarts_the_window() {
        let mut h = HeadingAlign::default();
        for s in straight(60) {
            h.observe(s, rot(0.05, s));
        }
        let before = h.fit().unwrap();
        // The EKF's track jumps 20 m (a reset onto another source) and the
        // rest of it is turned: the samples across the jump never mix.
        for s in (60..80).map(|k| [k as f64 * 1.5, 0.2 * k as f64]) {
            let e = rot(-0.4, s);
            h.observe(s, [e[0] + 20.0, e[1]]);
        }
        assert_eq!(h.fit(), Some(before), "20 samples after the jump are under the baseline");
    }

    #[test]
    fn a_track_at_another_scale_is_refused() {
        let mut h = HeadingAlign::default();
        for s in straight(80) {
            h.observe(s, [s[0] * 1.3, s[1] * 1.3]);
        }
        assert!(h.fit().is_none());
    }

    #[test]
    fn hovering_adds_no_samples() {
        let mut h = HeadingAlign::default();
        for s in straight(60) {
            h.observe(s, rot(0.05, s));
        }
        let fit = h.fit().unwrap();
        let last = [59.0 * 1.5, 0.2 * 59.0];
        for k in 0..1000 {
            let wobble = [last[0] + 0.3 * (k as f64).sin(), last[1]];
            assert!(h.observe(wobble, rot(0.05, last)).is_none());
        }
        assert_eq!(h.fit(), Some(fit));
    }
}
