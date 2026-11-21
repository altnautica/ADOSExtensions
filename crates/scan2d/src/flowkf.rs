//! The measured ground velocity (a downward flow sensor) along a direction
//! the scans leave loose, judged against the accelerometer.
//!
//! Along a corridor's axis nothing but the flow sensor measures the motion,
//! and a sensor losing the ground does not say so: it stalls, reads noise
//! around zero, or fades. Two Kalman filters ride the accelerometer's
//! levelled specific force ([`FlowGuard::feed_force`]), state
//! `[position, velocity, accelerometer bias]`, each in the plane, in the
//! attitude's levelled frame:
//!
//! - the *fused* filter also takes every flow sample while the flow is
//!   trusted;
//! - the *reference* filter takes only what a failing flow cannot corrupt:
//!   the velocity the scans match along the directions they pin
//!   ([`FlowGuard::scan`]) and zero velocity at a declared rest, on the
//!   ground before take-off ([`FlowGuard::feed_rest`]). Those make the bias
//!   observable; between them it is a random walk ([`BIAS_WALK`]).
//!
//! The flow is judged by windowed innovation tests along the loose
//! direction ([`FlowGuard::judge`]): its mean over the last
//! [`WINDOWS_NS`] against the fused filter's state at the window's start
//! carried on by the accelerometer (a stall, a jump), and its mean over the
//! longest window against the reference filter (a fade the fused filter
//! would follow), each normalised by its own variance and gated
//! ([`GATE_WINDOW`], [`GATE_REFERENCE`]). A flow that fails is a stall: it
//! is no longer fused and the corridor rides the reference velocity. It is
//! trusted again only on evidence of life: it follows a change of over
//! [`LIFE_MPS`] the reference shows, or, moving, agrees with a reference
//! sure of itself ([`REACCEPT_SIGMA`]). Agreeing at rest is no evidence; a
//! dead sensor reads zero too.
//!
//! The position's sigma along the corridor ([`FlowGuard::sigma_along`]) is
//! how far the pose has moved from where the accelerometer alone puts it
//! since the scans last pinned every direction, added to the reference's
//! own error since, never under the fused filter's: a flow fading too
//! slowly for the gates leaves the pose off by about that much, so the
//! sigma shows what the gates cannot yet call.
//!
//! Limits.
//! - Along a bare corridor a step in the accelerometer's bias and a flow
//!   fading at the same rate look alike (the flow says constant, the
//!   accelerometer says accelerating, or the mirror); only the bias random
//!   walk sets where one ends and the other begins. Steps up to 0.01 m/s²
//!   are absorbed; from about 0.02 m/s² held for 10 s or more they may be
//!   judged a flow fault and the corridor flagged lost (fail-safe); at
//!   0.05 m/s² within 10 s.
//! - At 0.85 m/s noise a sample from a 50 Hz flow, the slope of a fade
//!   cannot be told from noise in less than about 8 s at 0.06 m/s²
//!   (σ_slope = 0.85·√(12 / (50·τ³)) m/s²). Fades of 0.1 m/s² or more
//!   (1.5 m/s to nothing in 15 s or less) are caught within 2 m of error;
//!   slower ones are caught later, the pose within two of its sigma.
//! - Without specific force nothing here judges the flow. Without a rest or
//!   a pinned stretch the bias keeps its prior ([`BIAS_PRIOR`]) and the
//!   reference test is too loose to catch a fade.

use std::collections::VecDeque;

use nalgebra::{Matrix2, SMatrix, SVector, Vector2};

type V6 = SVector<f64, 6>;
type M6 = SMatrix<f64, 6, 6>;
type R6 = SMatrix<f64, 1, 6>;

/// White noise of the levelled specific force, m/s² per √Hz.
pub const ACCEL_NOISE: f64 = 0.01;
/// Random walk of the accelerometer's bias (with the levelling's error),
/// m/s² per √s: about 0.012 m/s² a minute.
pub const BIAS_WALK: f64 = 0.0015;
/// The bias before a rest or a pinned stretch teaches it, m/s² one sigma (a
/// levelling 1.7° off).
pub const BIAS_PRIOR: f64 = 0.3;
/// The velocity before anything measures it, m/s one sigma.
const VELOCITY_PRIOR: f64 = 5.0;
/// Zero velocity at a declared rest, m/s one sigma.
const REST_SIGMA: f64 = 0.01;
/// The velocity the scans match along a direction they pin, m/s one sigma
/// (a chord over a few scans, a scan interval off the filter's time).
const MATCH_VELOCITY_SIGMA: f64 = 0.1;
/// The position the scans hold along a direction they pin, m one sigma.
const MATCH_POSITION_SIGMA: f64 = 0.05;
/// The flow's noise a sample, m/s one sigma, before its innovations teach
/// it, the least it is taken for, and the share a sample teaches.
const FLOW_SIGMA_START: f64 = 1.0;
const FLOW_SIGMA_MIN: f64 = 0.02;
const FLOW_NOISE_GAIN: f64 = 0.01;
/// The windows the flow is judged over, ns; the longest also against the
/// reference filter.
pub const WINDOWS_NS: [u64; 2] = [500_000_000, 2_000_000_000];
/// The fewest flow samples a window is judged on.
const MIN_SAMPLES: usize = 5;
/// Normalised innovation squared over which a window against the fused
/// filter fails (chi-square, one degree, p = 0.0002: the windows are judged
/// every scan, overlapping).
pub const GATE_WINDOW: f64 = 13.8;
/// ... and the longest window against the reference filter (p = 0.0027).
pub const GATE_REFERENCE: f64 = 9.0;
/// A stalled flow is alive again once the reference velocity changes by
/// over this between the longest window's halves, m/s, ...
pub const LIFE_MPS: f64 = 0.5;
/// ... and the flow's change follows it within this share of it.
pub const LIFE_FOLLOW: f64 = 0.3;
/// A stalled flow that agrees with the reference (normalised innovation
/// squared under [`GATE_AGREE`]) while the reference is this sure of
/// itself, m/s one sigma, and moving over [`LIFE_MPS`], is alive again too.
pub const REACCEPT_SIGMA: f64 = 0.25;
/// One sigma: agreement, not mere consistency.
pub const GATE_AGREE: f64 = 1.0;
/// Flow samples whose noise must be learnt before the flow is judged (5 s
/// of a 50 Hz sensor).
const MIN_LEARNT: usize = 250;
/// Specific force older than this is not held across a gap, ns.
const MAX_HOLD_NS: u64 = 50_000_000;

/// One Kalman filter: `[position, velocity, bias]`, each in the plane.
#[derive(Debug, Clone)]
struct Kf {
    x: V6,
    p: M6,
}

impl Kf {
    fn new() -> Self {
        let mut p = M6::zeros();
        for i in 2..4 {
            p[(i, i)] = VELOCITY_PRIOR * VELOCITY_PRIOR;
        }
        for i in 4..6 {
            p[(i, i)] = BIAS_PRIOR * BIAS_PRIOR;
        }
        Kf { x: V6::zeros(), p }
    }

    fn position(&self) -> Vector2<f64> {
        Vector2::new(self.x[0], self.x[1])
    }

    fn velocity(&self) -> Vector2<f64> {
        Vector2::new(self.x[2], self.x[3])
    }

    fn bias(&self) -> Vector2<f64> {
        Vector2::new(self.x[4], self.x[5])
    }

    /// The `(k, k)` block from row and column `k`, along `u`.
    fn var_along(&self, k: usize, u: &Vector2<f64>) -> f64 {
        let b = self.p.fixed_view::<2, 2>(k, k);
        u.dot(&(b * u))
    }

    fn predict(&mut self, a: Vector2<f64>, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let acc = a - self.bias();
        for i in 0..2 {
            self.x[i] += self.x[2 + i] * dt + 0.5 * acc[i] * dt * dt;
            self.x[2 + i] += acc[i] * dt;
        }
        let mut f = M6::identity();
        for i in 0..2 {
            f[(i, 2 + i)] = dt;
            f[(i, 4 + i)] = -0.5 * dt * dt;
            f[(2 + i, 4 + i)] = -dt;
        }
        let (qa, qb) = (ACCEL_NOISE * ACCEL_NOISE, BIAS_WALK * BIAS_WALK);
        let mut q = M6::zeros();
        for i in 0..2 {
            q[(i, i)] = qa * dt * dt * dt / 3.0;
            q[(i, 2 + i)] = qa * dt * dt / 2.0;
            q[(2 + i, i)] = qa * dt * dt / 2.0;
            q[(2 + i, 2 + i)] = qa * dt;
            q[(4 + i, 4 + i)] = qb * dt;
        }
        self.p = f * self.p * f.transpose() + q;
    }

    /// A scalar measurement `z` of `h · x` with variance `r`.
    fn update(&mut self, h: R6, z: f64, r: f64) {
        let s = (h * self.p * h.transpose())[(0, 0)] + r;
        if s <= 0.0 {
            return;
        }
        let k = self.p * h.transpose() / s;
        let innov = z - (h * self.x)[(0, 0)];
        self.x += k * innov;
        let i_kh = M6::identity() - k * h;
        // Joseph form keeps the covariance symmetric and positive.
        self.p = i_kh * self.p * i_kh.transpose() + k * k.transpose() * r;
    }

    fn row(k: usize, u: &Vector2<f64>) -> R6 {
        let mut h = R6::zeros();
        h[(0, k)] = u.x;
        h[(0, k + 1)] = u.y;
        h
    }

    fn update_velocity(&mut self, u: &Vector2<f64>, z: f64, r: f64) {
        self.update(Self::row(2, u), z, r);
    }

    /// A measurement `z` of the velocity with covariance `r`.
    fn update_velocity2(&mut self, z: Vector2<f64>, r: &Matrix2<f64>) {
        let s = self.p.fixed_view::<2, 2>(2, 2) + r;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };
        let k: SMatrix<f64, 6, 2> = self.p.fixed_view::<6, 2>(0, 2) * s_inv;
        let mut h = SMatrix::<f64, 2, 6>::zeros();
        h[(0, 2)] = 1.0;
        h[(1, 3)] = 1.0;
        self.x += k * (z - self.velocity());
        let i_kh = M6::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }

    /// The scans hold the position along `u`: its variance shrinks, its
    /// value (the matcher's, not this filter's) is not read.
    fn hold_position(&mut self, u: &Vector2<f64>) {
        let h = Self::row(0, u);
        let z = (h * self.x)[(0, 0)];
        self.update(h, z, MATCH_POSITION_SIGMA * MATCH_POSITION_SIGMA);
    }
}

/// One flow sample with both filters' state at its time.
#[derive(Debug, Clone, Copy)]
struct Sample {
    t_ns: u64,
    flow: Vector2<f64>,
    /// The reference filter's velocity and its covariance.
    v_ref: Vector2<f64>,
    p_ref: Matrix2<f64>,
    /// The fused filter's velocity before this sample, its covariance, its
    /// bias's covariance, and its specific force integrated (bias taken
    /// out) so far.
    v_fused: Vector2<f64>,
    p_fused: Matrix2<f64>,
    p_bias: Matrix2<f64>,
    carried: Vector2<f64>,
}

/// What the flow along a loose direction shows ([`FlowGuard::judge`]).
#[derive(Debug, Clone, Copy, Default)]
pub struct Judgement {
    /// A window failed its gate: the flow is not to be followed.
    pub fails: bool,
    /// The flow follows a change of over [`LIFE_MPS`] the reference shows.
    pub alive: bool,
}

/// The two filters and the recent flow ([`crate::frontend`]'s check of a
/// measured velocity along a corridor).
#[derive(Debug, Clone)]
pub struct FlowGuard {
    fused: Kf,
    reference: Kf,
    t_ns: Option<u64>,
    /// The newest specific force, levelled, and when.
    accel: Option<(u64, Vector2<f64>)>,
    /// The fused filter's specific force integrated, bias taken out.
    carried: Vector2<f64>,
    /// The flow's noise a sample, covariance, as its innovations teach it,
    /// and how many have.
    flow_cov: Matrix2<f64>,
    flow_learnt: usize,
    samples: VecDeque<Sample>,
    stalled: bool,
    /// The pose, summed from the moves the matcher reports
    /// ([`Self::moved`]), attitude frame.
    pose: Vector2<f64>,
    /// The pose less the reference's position when the scans last pinned
    /// every direction (or at the start, or a rebase).
    pos_offset: Vector2<f64>,
    /// The next placed pose sets [`Self::pos_offset`].
    rebase: bool,
}

impl Default for FlowGuard {
    fn default() -> Self {
        FlowGuard {
            fused: Kf::new(),
            reference: Kf::new(),
            t_ns: None,
            accel: None,
            carried: Vector2::zeros(),
            flow_cov: Matrix2::identity() * FLOW_SIGMA_START * FLOW_SIGMA_START,
            flow_learnt: 0,
            samples: VecDeque::new(),
            stalled: false,
            pose: Vector2::zeros(),
            pos_offset: Vector2::zeros(),
            rebase: true,
        }
    }
}

impl FlowGuard {
    /// Whether specific force has been fed: without it nothing is judged.
    pub fn active(&self) -> bool {
        self.accel.is_some()
    }

    /// Both filters carried to `t_ns` on the newest specific force.
    fn advance(&mut self, t_ns: u64) {
        let Some(t0) = self.t_ns else {
            self.t_ns = Some(t_ns);
            return;
        };
        if t_ns <= t0 {
            return;
        }
        let dt = (t_ns - t0) as f64 * 1e-9;
        let a = self
            .accel
            .filter(|&(ta, _)| t_ns.saturating_sub(ta) <= MAX_HOLD_NS)
            .map_or_else(|| self.fused.bias(), |(_, a)| a);
        self.carried += (a - self.fused.bias()) * dt;
        self.fused.predict(a, dt);
        self.reference.predict(a, dt);
        self.t_ns = Some(t_ns);
    }

    /// The accelerometer's levelled specific force at `t_ns`, its horizontal
    /// part, the attitude's levelled frame, m/s².
    pub fn feed_force(&mut self, t_ns: u64, a: Vector2<f64>) {
        self.advance(t_ns);
        self.accel = Some((t_ns, a));
    }

    /// The vehicle is at rest at `t_ns` (on the ground before take-off).
    pub fn feed_rest(&mut self, t_ns: u64) {
        if !self.active() {
            return;
        }
        self.advance(t_ns);
        let r = REST_SIGMA * REST_SIGMA;
        for u in [Vector2::x(), Vector2::y()] {
            self.fused.update_velocity(&u, 0.0, r);
            self.reference.update_velocity(&u, 0.0, r);
        }
    }

    /// One flow sample at `t_ns`, the attitude's levelled frame: kept for
    /// the windows, and fused while the flow is trusted.
    pub fn feed_flow(&mut self, t_ns: u64, flow: Vector2<f64>) {
        if !self.active() {
            return;
        }
        self.advance(t_ns);
        let block = |kf: &Kf, k: usize| kf.p.fixed_view::<2, 2>(k, k).into_owned();
        self.samples.push_back(Sample {
            t_ns,
            flow,
            v_ref: self.reference.velocity(),
            p_ref: block(&self.reference, 2),
            v_fused: self.fused.velocity(),
            p_fused: block(&self.fused, 2),
            p_bias: block(&self.fused, 4),
            carried: self.carried,
        });
        let keep = WINDOWS_NS[WINDOWS_NS.len() - 1] + 500_000_000;
        while self.samples.front().is_some_and(|s| s.t_ns + keep < t_ns) {
            self.samples.pop_front();
        }
        if self.stalled {
            return;
        }
        let e = flow - self.fused.velocity();
        let p = block(&self.fused, 2);
        let seen = e * e.transpose() - p;
        let cov = self.flow_cov + (seen - self.flow_cov) * FLOW_NOISE_GAIN;
        // Symmetric, and no direction quieter than the floor.
        let eig = (0.5 * (cov + cov.transpose())).symmetric_eigen();
        let floor = FLOW_SIGMA_MIN * FLOW_SIGMA_MIN;
        let vals = eig.eigenvalues.map(|v| v.max(floor));
        self.flow_cov =
            eig.eigenvectors * Matrix2::from_diagonal(&vals) * eig.eigenvectors.transpose();
        self.flow_learnt += 1;
        self.fused.update_velocity2(flow, &self.flow_cov);
    }

    /// The scans pin every direction at a scan: the velocity they matched
    /// (`chord`, the attitude's levelled frame, about `t_ns`; the filters may
    /// be a scan interval on, which [`MATCH_VELOCITY_SIGMA`] allows for), and
    /// the position they hold. Both filters take it. Along a corridor the
    /// scans' share across is not taken: any direction a little off the
    /// axis would carry the motion prior (the flow) along it into the
    /// reference.
    pub fn scan(&mut self, t_ns: u64, chord: Vector2<f64>) {
        if !self.active() {
            return;
        }
        self.advance(t_ns);
        let r = MATCH_VELOCITY_SIGMA * MATCH_VELOCITY_SIGMA;
        for (i, u) in [Vector2::x(), Vector2::y()].iter().enumerate() {
            self.fused.update_velocity(u, chord[i], r);
            self.reference.update_velocity(u, chord[i], r);
            self.fused.hold_position(u);
            self.reference.hold_position(u);
        }
        self.rebase = true;
    }

    /// The flow judged along the unit direction `u` (attitude frame) up to
    /// `t_ns`.
    pub fn judge(&self, t_ns: u64, u: &Vector2<f64>) -> Judgement {
        let mut j = Judgement::default();
        if !self.active() || self.flow_learnt < MIN_LEARNT {
            return j;
        }
        let qa = ACCEL_NOISE * ACCEL_NOISE;
        let r_u = u.dot(&(self.flow_cov * u));
        let end = self.samples.partition_point(|s| s.t_ns <= t_ns);
        for (k, &w) in WINDOWS_NS.iter().enumerate() {
            let start = self.samples.partition_point(|s| s.t_ns + w <= t_ns);
            let n = end.saturating_sub(start);
            if n < MIN_SAMPLES {
                continue;
            }
            let (s0, last) = (self.samples[start], self.samples[end - 1]);
            let (mut flow, mut carried, mut off_ref, mut p_ref) =
                (Vector2::zeros(), Vector2::zeros(), Vector2::zeros(), 0.0);
            let (mut off_fused, mut p_fused) = (Vector2::zeros(), 0.0);
            // The window's first half, for the change across it.
            let half = start + n / 2;
            let (mut flow_1, mut ref_1, mut ref_2) =
                (Vector2::zeros(), Vector2::zeros(), Vector2::zeros());
            for (i, s) in self.samples.range(start..end).enumerate() {
                flow += s.flow;
                carried += s.carried;
                off_ref += s.flow - s.v_ref;
                p_ref += u.dot(&(s.p_ref * u));
                off_fused += s.flow - s.v_fused;
                p_fused += u.dot(&(s.p_fused * u));
                if start + i < half {
                    flow_1 += s.flow;
                    ref_1 += s.v_ref;
                } else {
                    ref_2 += s.v_ref;
                }
            }
            let k_n = n as f64;
            let noise = r_u / k_n;
            // Against the fused filter at the window's start, carried on by
            // the accelerometer.
            let mean = (flow - carried) / k_n - (s0.v_fused - s0.carried);
            let span = (last.t_ns - s0.t_ns) as f64 * 1e-9;
            let var = u.dot(&(s0.p_fused * u))
                + qa * span / 2.0
                + u.dot(&(s0.p_bias * u)) * span * span / 4.0
                + noise;
            j.fails |= u.dot(&mean).powi(2) / var > GATE_WINDOW;
            if k + 1 < WINDOWS_NS.len() {
                continue;
            }
            // The longest window against the reference filter.
            let ref_var = p_ref / k_n;
            let nis = u.dot(&(off_ref / k_n)).powi(2) / (ref_var + noise);
            j.fails |= nis > GATE_REFERENCE;
            // Alive: across the window's halves the reference changes by
            // over LIFE_MPS and the flow follows that change; or, moving
            // over LIFE_MPS, the flow agrees with the reference when that is
            // sure of itself, else with the fused filter, which while the
            // flow is stalled rides the accelerometer alone from the stall
            // (agreeing at rest is no evidence: a dead sensor reads zero).
            let (n1, n2) = ((half - start) as f64, (end - half) as f64);
            let d_ref = u.dot(&(ref_2 / n2 - ref_1 / n1));
            let d_flow = u.dot(&((flow - flow_1) / n2 - flow_1 / n1));
            let follows =
                d_ref.abs() > LIFE_MPS && (d_flow - d_ref).abs() < LIFE_FOLLOW * d_ref.abs();
            let fused_var = p_fused / k_n;
            let agrees = if ref_var.sqrt() < REACCEPT_SIGMA {
                nis <= GATE_AGREE && u.dot(&last.v_ref).abs() > LIFE_MPS
            } else {
                let nis_fused = u.dot(&(off_fused / k_n)).powi(2) / (fused_var + noise);
                fused_var.sqrt() < REACCEPT_SIGMA
                    && nis_fused <= GATE_AGREE
                    && u.dot(&last.v_fused).abs() > LIFE_MPS
            };
            // Nothing revives a flow the reference still objects to.
            j.alive = (follows || agrees) && nis <= GATE_REFERENCE;
        }
        j
    }

    /// Whether the flow is not followed. A flow trusted again starts the
    /// fused filter over from the reference when that knows the velocity
    /// better: whatever the fused one followed while failing does not stay.
    pub fn set_stalled(&mut self, stalled: bool) {
        if self.stalled && !stalled {
            let trace = |kf: &Kf| kf.p[(2, 2)] + kf.p[(3, 3)];
            if trace(&self.reference) < trace(&self.fused) {
                self.fused = self.reference.clone();
            }
        }
        self.stalled = stalled;
    }

    /// The pose the matcher reports moved by `d`, attitude frame, at a scan.
    pub fn moved(&mut self, d: Vector2<f64>) {
        self.pose += d;
        if self.rebase {
            self.pos_offset = self.pose - self.reference.position();
            self.rebase = false;
        }
    }

    /// The velocity the corridor rides while the flow is stalled: the
    /// reference filter's, attitude frame.
    pub fn reference_velocity(&self) -> Vector2<f64> {
        self.reference.velocity()
    }

    /// The position's one sigma along `u`, m: how far the pose has moved
    /// from where the reference (the accelerometer alone) puts it since the
    /// scans last pinned every direction, plus the reference's own error
    /// since, and never under the fused filter's. A flow fading too slowly
    /// for the gates ([`GATE_REFERENCE`]) leaves the pose about that far
    /// off.
    pub fn sigma_along(&self, u: &Vector2<f64>) -> f64 {
        let drift = u.dot(&(self.pose - self.reference.position() - self.pos_offset));
        let reference = self.reference.var_along(0, u).max(0.0) + drift * drift;
        reference.max(self.fused.var_along(0, u)).sqrt()
    }

    /// The reference filter's bias, attitude frame, and its one sigma along
    /// the axes, m/s².
    pub fn bias(&self) -> (Vector2<f64>, Vector2<f64>) {
        let p = &self.reference.p;
        (
            self.reference.bias(),
            Vector2::new(p[(4, 4)].sqrt(), p[(5, 5)].sqrt()),
        )
    }

    /// The flow's noise a sample as learnt, m/s one sigma (the mean of the
    /// two directions').
    #[cfg(test)]
    fn flow_sigma(&self) -> f64 {
        (0.5 * self.flow_cov.trace()).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// At rest for 6 s with a 0.1 m/s² accelerometer bias, the rest teaches
    /// it to within a few mm/s²; with no rest it keeps its prior.
    #[test]
    fn a_rest_teaches_the_bias_and_no_rest_leaves_it_uncertain() {
        let mut g = FlowGuard::default();
        for k in 0..1200u64 {
            let t = k * 5_000_000;
            g.feed_force(t, Vector2::new(0.1, 0.0));
            g.feed_rest(t);
        }
        let (b, s) = g.bias();
        assert!((b.x - 0.1).abs() < 0.01, "{b:?}");
        assert!(s.x < 0.01, "{s:?}");
        let mut g = FlowGuard::default();
        for k in 0..1200u64 {
            g.feed_force(k * 5_000_000, Vector2::new(0.1, 0.0));
        }
        let (_, s) = g.bias();
        assert!(s.x > 0.25, "no rest: {s:?}");
    }

    /// Sound flow at 0.85 m/s noise a sample, still: its noise is learnt,
    /// and fewer than 1 % of windows fail.
    #[test]
    fn sound_noisy_flow_rarely_fails_a_window() {
        let mut g = FlowGuard::default();
        let mut rng = 0x2545_f491_4f6c_dd1du64;
        let mut gauss = || {
            let mut u = || {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                (rng >> 11) as f64 / (1u64 << 53) as f64
            };
            let (a, b) = (u().max(1e-12), u());
            (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
        };
        let (mut fails, mut judged) = (0, 0);
        for k in 0..60_000u64 {
            let t = k * 5_000_000;
            g.feed_force(t, Vector2::zeros());
            if t < 5_000_000_000 {
                g.feed_rest(t);
            }
            if k % 4 == 0 {
                g.feed_flow(t, Vector2::new(0.85 * gauss(), 0.85 * gauss()));
            }
            if k % 20 == 0 && t > 10_000_000_000 {
                judged += 1;
                fails += usize::from(g.judge(t, &Vector2::x()).fails);
            }
        }
        assert!((g.flow_sigma() - 0.85).abs() < 0.15, "{}", g.flow_sigma());
        let rate = fails as f64 / judged as f64;
        assert!(rate < 0.01, "{fails} of {judged} windows failed");
    }
}
