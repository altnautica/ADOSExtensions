//! `map_ned` held on the flight controller's GNSS-referenced local frame.
//!
//! ArduPilot fuses external navigation (`VISO_TYPE` 1, MAVLink `ODOMETRY`)
//! exactly as sent: that backend applies no yaw trim and no position
//! correction, and when the EKF switches its position source to external
//! navigation it resets its position to the external one
//! (`AP_NavEKF3_PosVelFusion.cpp`, `posxy_source_reset`). So when GNSS fails,
//! the drone's position becomes `map_ned`'s, and any difference between
//! `map_ned` and `LOCAL_POSITION_NED` at that moment is a jump: a heading
//! error turns into a position error proportional to the distance from the
//! map's origin (3° at 137 m is 7 m), and an origin offset carries over as is.
//! The compass heading the map starts from is typically a few degrees off on
//! the ground (magnetic interference, declination, calibration).
//!
//! [`MapAlign`] estimates the rigid 2D transform `e ≈ Rz(yaw)·s + offset` from
//! the SLAM track `s` (`map_frd`) to the EKF's GNSS-referenced track `e`
//! (north, east) over the same instants:
//!
//! - yaw: the Procrustes rotation of the centred tracks, `atan2(Σ s×e, Σ s·e)`,
//!   over samples taken every [`SAMPLE_STEP_M`] of SLAM travel and weighted
//!   down with distance ([`WINDOW_M`]), so it follows the SLAM's own slow
//!   heading drift. A fit is accepted once the track spans
//!   [`MIN_BASELINE_M`], both tracks agree in scale and its yaw uncertainty is
//!   below [`MAX_YAW_SIGMA_RAD`]; until then the compass heading stands;
//! - offset: the mean of `e − Rz(yaw)·s` over the last [`NEAR_SAMPLES`]
//!   pairs, so the map sits on the EKF where the drone is now.
//!
//! When GNSS stops being usable the windows are dropped and the transform is
//! frozen: from then on the EKF's position is the map's. A step where the two
//! tracks disagree by more than [`STEP_JUMP_M`] (an EKF position reset, a SLAM
//! relocalisation, a world-frame change) restarts the windows.

/// SLAM travel between yaw samples, metres.
pub const SAMPLE_STEP_M: f64 = 1.0;
/// Distance over which a yaw sample's weight falls by `e`, metres.
pub const WINDOW_M: f64 = 200.0;
/// Smallest RMS spread of the SLAM track about its centroid for a yaw fit,
/// metres (a straight leg of about 35 m).
pub const MIN_BASELINE_M: f64 = 10.0;
/// Largest relative difference between the two tracks' spreads.
pub const MAX_SCALE_ERR: f64 = 0.2;
/// Largest yaw uncertainty of an accepted fit, radians (0.5°).
pub const MAX_YAW_SIGMA_RAD: f64 = 0.5 * std::f64::consts::PI / 180.0;
/// Pairs over which a pair's weight in the offset falls by `e` (about 5 s at
/// the 10 Hz scan rate).
pub const NEAR_SAMPLES: f64 = 50.0;
/// Largest disagreement of one step's length between the tracks, metres.
pub const STEP_JUMP_M: f64 = 3.0;
/// How long GNSS must have held a 3D fix before the EKF's position is taken as
/// GNSS-referenced, ms on the FC clock. Longer than a companion's switch back
/// from external navigation after GNSS returns (vision-nav: 10 s), so the
/// EKF's own reset back onto GNSS is behind it.
pub const GNSS_SETTLE_MS: u32 = 15_000;

/// One accepted yaw fit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawFit {
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

    /// The yaw of the centred tracks, when it passes every gate.
    fn fit(&self) -> Option<YawFit> {
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
        Some(YawFit {
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

/// `Rz(yaw) · s` in the x, y plane.
pub fn rot2(yaw: f64, s: [f64; 2]) -> [f64; 2] {
    let (sn, c) = yaw.sin_cos();
    [c * s[0] - sn * s[1], sn * s[0] + c * s[1]]
}

/// The GNSS alignment of the map; see the module docs.
#[derive(Debug, Default)]
pub struct MapAlign {
    /// The yaw window.
    sums: Sums,
    /// The offset window: weight, `Σ s`, `Σ e`.
    near: (f64, [f64; 2], [f64; 2]),
    /// The last yaw sample, `(slam, ekf)`.
    last: Option<([f64; 2], [f64; 2])>,
    /// The last pair observed; `None` while no window is open.
    prev: Option<([f64; 2], [f64; 2])>,
    fit: Option<YawFit>,
}

impl MapAlign {
    /// The newest accepted yaw fit, frozen while GNSS is unusable.
    pub fn fit(&self) -> Option<YawFit> {
        self.fit
    }

    /// Fold one pair taken at the same instant: `s` the SLAM position in
    /// `map_frd` (x, y), `e` the EKF's GNSS-referenced position (north,
    /// east). Returns the yaw fit when this pair produced a new accepted one.
    pub fn observe(&mut self, s: [f64; 2], e: [f64; 2]) -> Option<YawFit> {
        if let Some((s0, e0)) = self.prev {
            if (dist(s, s0) - dist(e, e0)).abs() > STEP_JUMP_M {
                self.drop_windows();
            }
        }
        self.prev = Some((s, e));
        let k = (-1.0 / NEAR_SAMPLES).exp();
        let (w, ns, ne) = self.near;
        self.near = (
            w * k + 1.0,
            [ns[0] * k + s[0], ns[1] * k + s[1]],
            [ne[0] * k + e[0], ne[1] * k + e[1]],
        );
        let Some((s0, _)) = self.last else {
            self.sums.add(s, e);
            self.last = Some((s, e));
            return None;
        };
        let ds = dist(s, s0);
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

    /// The offset `e − Rz(yaw)·s` over the recent pairs, `None` while no
    /// window is open.
    pub fn offset(&self, yaw: f64) -> Option<[f64; 2]> {
        self.prev?;
        let (w, s, e) = self.near;
        let rs = rot2(yaw, s);
        Some([(e[0] - rs[0]) / w, (e[1] - rs[1]) / w])
    }

    /// GNSS is not usable: drop the windows, keep the last fit. Returns
    /// whether a window was open (the first call after GNSS went).
    pub fn freeze(&mut self) -> bool {
        let open = self.prev.is_some();
        self.drop_windows();
        open
    }

    fn drop_windows(&mut self) {
        self.sums = Sums::default();
        self.near = (0.0, [0.0; 2], [0.0; 2]);
        self.last = None;
        self.prev = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(n: usize) -> impl Iterator<Item = [f64; 2]> {
        (0..n).map(|k| [k as f64 * 1.5, 0.2 * k as f64])
    }

    fn shifted(yaw: f64, s: [f64; 2], c: [f64; 2]) -> [f64; 2] {
        let r = rot2(yaw, s);
        [r[0] + c[0], r[1] + c[1]]
    }

    #[test]
    fn the_fit_is_the_transform_between_the_tracks() {
        let mut h = MapAlign::default();
        let (yaw, c) = (0.7, [400.0, -250.0]);
        let mut fit = None;
        for s in straight(60) {
            fit = h.observe(s, shifted(yaw, s, c)).or(fit);
        }
        let fit = fit.expect("a 90 m leg is a baseline");
        assert!((fit.yaw - yaw).abs() < 1e-9, "yaw {}", fit.yaw);
        assert!(fit.residual_m < 1e-3, "residual {}", fit.residual_m);
        let off = h.offset(fit.yaw).unwrap();
        assert!((off[0] - c[0]).abs() < 1e-6 && (off[1] - c[1]).abs() < 1e-6, "{off:?}");
    }

    #[test]
    fn no_yaw_fit_before_the_baseline_but_an_offset_at_once() {
        let mut h = MapAlign::default();
        assert!(h.offset(0.0).is_none());
        for s in straight(15) {
            assert!(h.observe(s, shifted(0.0, s, [3.0, 4.0])).is_none());
        }
        assert!(h.fit().is_none(), "a 21 m leg is too short");
        let off = h.offset(0.0).unwrap();
        assert!((off[0] - 3.0).abs() < 1e-9 && (off[1] - 4.0).abs() < 1e-9);
    }

    #[test]
    fn the_offset_follows_the_recent_track() {
        let mut h = MapAlign::default();
        // The SLAM track drifts 3 cm a sample off the EKF's: the offset
        // trails where the drone is now by the offset window (about 50
        // samples, 1.5 m here), far closer than the whole track's mean.
        for (k, s) in straight(200).enumerate() {
            h.observe(s, [s[0] + 0.03 * k as f64, s[1]]);
        }
        let off = h.offset(0.0).unwrap()[0];
        let now = 0.03 * 199.0;
        assert!(now - off < 1.1 * 0.03 * NEAR_SAMPLES && off > 0.7 * now, "{off} vs {now}");
    }

    #[test]
    fn gnss_loss_freezes_the_fit_and_a_new_window_needs_its_own_baseline() {
        let mut h = MapAlign::default();
        for s in straight(60) {
            h.observe(s, rot2(0.05, s));
        }
        let frozen = h.fit().unwrap();
        assert!(h.freeze(), "a window was open");
        assert!(!h.freeze(), "only once");
        assert!(h.offset(frozen.yaw).is_none());
        // GNSS back with the EKF's track now offset and turned: the yaw stays
        // until the new window has a baseline of its own.
        for s in straight(20) {
            h.observe(s, rot2(0.3, [s[0] + 5.0, s[1]]));
        }
        assert_eq!(h.fit(), Some(frozen));
    }

    #[test]
    fn a_position_jump_restarts_the_windows() {
        let mut h = MapAlign::default();
        for s in straight(60) {
            h.observe(s, rot2(0.05, s));
        }
        let before = h.fit().unwrap();
        // The EKF's track jumps 20 m (a reset onto another source) and the
        // rest of it is turned: the samples across the jump never mix.
        for s in (60..80).map(|k| [k as f64 * 1.5, 0.2 * k as f64]) {
            h.observe(s, shifted(-0.4, s, [20.0, 0.0]));
        }
        assert_eq!(h.fit(), Some(before), "20 samples after the jump are under the baseline");
        let off = h.offset(-0.4).unwrap();
        assert!((off[0] - 20.0).abs() < 1e-6 && off[1].abs() < 1e-6, "{off:?}");
    }

    #[test]
    fn a_track_at_another_scale_is_refused() {
        let mut h = MapAlign::default();
        for s in straight(80) {
            h.observe(s, [s[0] * 1.3, s[1] * 1.3]);
        }
        assert!(h.fit().is_none());
    }

    #[test]
    fn hovering_adds_no_yaw_samples() {
        let mut h = MapAlign::default();
        for s in straight(60) {
            h.observe(s, rot2(0.05, s));
        }
        let fit = h.fit().unwrap();
        let last = [59.0 * 1.5, 0.2 * 59.0];
        for k in 0..1000 {
            let wobble = [last[0] + 0.3 * (k as f64).sin(), last[1]];
            assert!(h.observe(wobble, rot2(0.05, last)).is_none());
        }
        assert_eq!(h.fit(), Some(fit));
    }
}
