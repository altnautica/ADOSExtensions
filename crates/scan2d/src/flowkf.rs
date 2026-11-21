//! The measured ground velocity (a downward flow sensor) along a direction
//! the scans leave loose, judged against the accelerometer.
//!
//! Along a corridor's axis nothing but the flow sensor measures the motion,
//! and a sensor losing the ground does not say so: it stalls, reads noise
//! around zero, or fades; and a sound one may read a few percent off (a
//! height or texture error). Two extended Kalman filters ride the
//! accelerometer's levelled specific force ([`FlowGuard::feed_force`]),
//! state `[position, velocity, accelerometer bias]`, each in the plane, in
//! the attitude's levelled frame, and the flow's scale `k` (the flow reads
//! `k` times the velocity; prior [`SCALE_PRIOR`], a random walk
//! [`SCALE_WALK`] and [`SCALE_PER_HEIGHT_M`] a metre climbed):
//!
//! - the *fused* filter also takes every flow sample while the flow is
//!   trusted, linearised about its prediction. Where the scans pin every
//!   direction, or the accelerometer shows an acceleration, the velocity is
//!   known apart from the flow and the scale is learnt; the pose rides the
//!   flow over it ([`FlowGuard::flow_scale`]). Without specific force the
//!   filters guess the motion between samples ([`FREE_ACCEL_NOISE`]) and the
//!   scale is only considered: its uncertainty counts, nothing moves it;
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
//! would follow, into its velocity or its scale: against the reference the
//! flow is taken over the scale learnt when the scans last pinned every
//! direction), each normalised by its own variance and gated
//! ([`GATE_WINDOW`], [`GATE_REFERENCE`]). A flow that fails is a stall: it
//! is no longer fused and the corridor rides the reference velocity. It is
//! trusted again only on evidence of life: it follows a change of over
//! [`LIFE_MPS`] the reference shows, or, moving, agrees with a reference
//! sure of itself ([`REACCEPT_SIGMA`]). Agreeing at rest is no evidence; a
//! dead sensor reads zero too.
//!
//! The position's sigma along the corridor ([`FlowGuard::sigma_along`]):
//! while the flow is fused, the fused filter's, whose position carries the
//! scale's uncertainty through their covariance, so it grows with the way
//! flown on a scale no pinned stretch or acceleration has taught, plus how
//! far the pose has moved from the fused filter since the scans last
//! pinned; while it is stalled, the reference's own error plus how far the
//! pose had moved from it on the flow.
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
//!   slower ones are caught later, up to about 3.3 m off and four of the
//!   sigma for a 25 s fade: until it is caught nothing tells the fade from
//!   sound flow, whose sigma it then shares.
//! - At constant speed a step in the flow's scale is a fade: it shows only
//!   against the reference, with the same floor.
//! - A scale is learnt only as well as the flow's noise allows: at 0.85 m/s
//!   a sample, a 10 m pinned walk at 1.5 m/s teaches it to about 2 %.
//! - Without specific force nothing here judges the flow (the pose says
//!   so) and the scale is not learnt. Without a rest or a pinned stretch
//!   the bias keeps its prior ([`BIAS_PRIOR`]) and the reference test is
//!   too loose to catch a fade.

use std::collections::VecDeque;

use nalgebra::{Matrix2, SMatrix, SVector, Vector2};

type V7 = SVector<f64, 7>;
type M7 = SMatrix<f64, 7, 7>;
type R7 = SMatrix<f64, 1, 7>;

/// The flow scale's index in the state.
const K: usize = 6;

/// White noise of the levelled specific force, m/s² per √Hz.
pub const ACCEL_NOISE: f64 = 0.01;
/// The acceleration's white noise while no specific force drives the
/// filters (none fed, or the newest too old): the motion is a guess the
/// flow corrects, m/s² per √Hz.
pub const FREE_ACCEL_NOISE: f64 = 1.0;
/// Random walk of the accelerometer's bias (with the levelling's error),
/// m/s² per √s: about 0.012 m/s² a minute.
pub const BIAS_WALK: f64 = 0.0015;
/// The bias before a rest or a pinned stretch teaches it, m/s² one sigma (a
/// levelling 1.7° off).
pub const BIAS_PRIOR: f64 = 0.3;
/// The flow's scale (its speed over the true speed) before a pinned stretch
/// or an acceleration teaches it, one sigma about 1: a height or texture
/// error. Its uncertainty never grows past this.
pub const SCALE_PRIOR: f64 = 0.03;
/// Random walk of the flow's scale, per √s.
pub const SCALE_WALK: f64 = 0.001;
/// The flow's scale changes with the height, one sigma a metre climbed or
/// descended.
pub const SCALE_PER_HEIGHT_M: f64 = 0.01;
/// The least sigma along a loose direction once flow has been fused, m:
/// the pose then rides the flow's recent mean, and is never sure to better
/// than this (the first metres after a take-off lag the flow's window by
/// up to about 0.28 m).
pub const SIGMA_FLOOR_M: f64 = 0.15;
/// The velocity before anything measures it, m/s one sigma.
const VELOCITY_PRIOR: f64 = 5.0;
/// Zero velocity at a declared rest, m/s one sigma.
const REST_SIGMA: f64 = 0.01;
/// The velocity the scans match along a direction they pin, m/s one sigma
/// (a chord over a few scans, a scan interval off the filter's time).
const MATCH_VELOCITY_SIGMA: f64 = 0.1;
/// The position the scans hold along a direction they pin, m one sigma: a
/// room's far walls hold it less well than a scan's own match.
const MATCH_POSITION_SIGMA: f64 = 0.3;
/// The flow's noise a sample, m/s one sigma, before its innovations teach
/// it, the least it is taken for, and the share a sample teaches.
const FLOW_SIGMA_START: f64 = 1.0;
const FLOW_SIGMA_MIN: f64 = 0.02;
const FLOW_NOISE_GAIN: f64 = 0.01;
/// The flow's scale the pose divides it by is held within this share of 1.
const SCALE_CLAMP: f64 = 0.2;
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

/// One extended Kalman filter: `[position, velocity, bias]`, each in the
/// plane, and the flow's scale `k` (the flow reads `k` times the velocity).
#[derive(Debug, Clone)]
struct Kf {
    x: V7,
    p: M7,
    /// Whether specific force drove the last step: without it the scale
    /// cannot be told from the velocity, and is a considered state
    /// (Schmidt): its uncertainty counts, no measurement moves it.
    learn_scale: bool,
}

impl Kf {
    fn new() -> Self {
        let mut p = M7::zeros();
        for i in 2..4 {
            p[(i, i)] = VELOCITY_PRIOR * VELOCITY_PRIOR;
        }
        for i in 4..6 {
            p[(i, i)] = BIAS_PRIOR * BIAS_PRIOR;
        }
        p[(K, K)] = SCALE_PRIOR * SCALE_PRIOR;
        let mut x = V7::zeros();
        x[K] = 1.0;
        Kf {
            x,
            p,
            learn_scale: false,
        }
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

    /// The scale's variance grows by `q`, never past its prior.
    fn widen_scale(&mut self, q: f64) {
        let room = SCALE_PRIOR * SCALE_PRIOR - self.p[(K, K)];
        self.p[(K, K)] += q.min(room).max(0.0);
    }

    /// Carried `dt` on the specific force `a`; without one the
    /// acceleration is zero and loose ([`FREE_ACCEL_NOISE`]).
    fn predict(&mut self, a: Option<Vector2<f64>>, dt: f64) {
        self.learn_scale = a.is_some();
        if dt <= 0.0 {
            return;
        }
        let acc = a.map_or_else(Vector2::zeros, |a| a - self.bias());
        for i in 0..2 {
            self.x[i] += self.x[2 + i] * dt + 0.5 * acc[i] * dt * dt;
            self.x[2 + i] += acc[i] * dt;
        }
        let mut f = M7::identity();
        for i in 0..2 {
            f[(i, 2 + i)] = dt;
            if a.is_some() {
                f[(i, 4 + i)] = -0.5 * dt * dt;
                f[(2 + i, 4 + i)] = -dt;
            }
        }
        let sa = if a.is_some() {
            ACCEL_NOISE
        } else {
            FREE_ACCEL_NOISE
        };
        let (qa, qb) = (sa * sa, BIAS_WALK * BIAS_WALK);
        let mut q = M7::zeros();
        for i in 0..2 {
            q[(i, i)] = qa * dt * dt * dt / 3.0;
            q[(i, 2 + i)] = qa * dt * dt / 2.0;
            q[(2 + i, i)] = qa * dt * dt / 2.0;
            q[(2 + i, 2 + i)] = qa * dt;
            q[(4 + i, 4 + i)] = qb * dt;
        }
        self.p = f * self.p * f.transpose() + q;
        self.widen_scale(SCALE_WALK * SCALE_WALK * dt);
    }

    /// A measurement with innovation `innov`, Jacobian `h` and covariance
    /// `r` (Joseph form, which keeps the covariance symmetric and positive
    /// and holds for the considered scale's gain too).
    fn update2(&mut self, h: &SMatrix<f64, 2, 7>, innov: Vector2<f64>, r: &Matrix2<f64>) {
        let s = h * self.p * h.transpose() + r;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };
        let mut k = self.p * h.transpose() * s_inv;
        if !self.learn_scale {
            k.row_mut(K).fill(0.0);
        }
        self.x += k * innov;
        let i_kh = M7::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }

    /// A scalar measurement `z` of `h · x` with variance `r`.
    fn update_linear(&mut self, h: R7, z: f64, r: f64) {
        let mut h2 = SMatrix::<f64, 2, 7>::zeros();
        h2.set_row(0, &h);
        // The second row measures nothing, with unit noise.
        let innov = Vector2::new(z - (h * self.x)[(0, 0)], 0.0);
        self.update2(&h2, innov, &Matrix2::new(r, 0.0, 0.0, 1.0));
    }

    fn row(k: usize, u: &Vector2<f64>) -> R7 {
        let mut h = R7::zeros();
        h[(0, k)] = u.x;
        h[(0, k + 1)] = u.y;
        h
    }

    fn update_velocity(&mut self, u: &Vector2<f64>, z: f64, r: f64) {
        self.update_linear(Self::row(2, u), z, r);
    }

    /// The flow measurement's Jacobian about the prediction, and the flow
    /// predicted: `k` times the velocity.
    fn flow_jacobian(&self) -> (SMatrix<f64, 2, 7>, Vector2<f64>) {
        let (k, v) = (self.x[K], self.velocity());
        let mut h = SMatrix::<f64, 2, 7>::zeros();
        h[(0, 2)] = k;
        h[(1, 3)] = k;
        h[(0, K)] = v.x;
        h[(1, K)] = v.y;
        (h, k * v)
    }

    /// A flow sample `z` with covariance `r`, linearised about the
    /// prediction (an extended Kalman filter's step).
    fn update_flow(&mut self, z: Vector2<f64>, r: &Matrix2<f64>) {
        let (h, pred) = self.flow_jacobian();
        self.update2(&h, z - pred, r);
    }

    /// The scans hold the position along `u`: its variance is brought down
    /// to [`MATCH_POSITION_SIGMA`]'s, no further (scan after scan the
    /// matcher's error is the same error, not fresh evidence); its value
    /// (the matcher's, not this filter's) is not read.
    fn hold_position(&mut self, u: &Vector2<f64>) {
        let h = Self::row(0, u);
        let var = (h * self.p * h.transpose())[(0, 0)];
        let held = MATCH_POSITION_SIGMA * MATCH_POSITION_SIGMA;
        if var <= held * 1.0001 {
            return;
        }
        let z = (h * self.x)[(0, 0)];
        self.update_linear(h, z, var * held / (var - held));
    }

    /// The flow's scale, held within [`SCALE_CLAMP`] of 1.
    fn scale(&self) -> f64 {
        self.x[K].clamp(1.0 - SCALE_CLAMP, 1.0 + SCALE_CLAMP)
    }
}

/// One flow sample with both filters' state at its time.
#[derive(Debug, Clone, Copy)]
struct Sample {
    t_ns: u64,
    /// The flow over the scale the scans last taught ([`FlowGuard::pin`]),
    /// for the tests against the reference, and over the fused filter's
    /// scale before this sample.
    flow: Vector2<f64>,
    flow_fused: Vector2<f64>,
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
    /// The fused filter's scale and its variance when the scans last
    /// pinned every direction (else the prior): what the reference tests
    /// take the flow's scale for, as a fade the fused filter follows into
    /// its scale must still show against the reference.
    pin: (f64, f64),
    /// The newest height, m.
    height_m: Option<f64>,
    /// The pose, summed from the moves the matcher reports
    /// ([`Self::moved`]), attitude frame.
    pose: Vector2<f64>,
    /// The pose less the reference's position when the scans last pinned
    /// every direction (or at the start, or a rebase).
    pos_offset: Vector2<f64>,
    /// The same for the fused filter.
    fused_offset: Vector2<f64>,
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
            pin: (1.0, SCALE_PRIOR * SCALE_PRIOR),
            height_m: None,
            pose: Vector2::zeros(),
            pos_offset: Vector2::zeros(),
            fused_offset: Vector2::zeros(),
            rebase: true,
        }
    }
}

impl FlowGuard {
    /// Whether specific force has been fed: without it nothing is judged.
    pub fn active(&self) -> bool {
        self.accel.is_some()
    }

    /// Both filters carried to `t_ns` on the newest specific force, or on
    /// none ([`Kf::predict`]).
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
            .map(|(_, a)| a);
        if let Some(a) = a {
            self.carried += (a - self.fused.bias()) * dt;
        }
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

    /// The height over the ground, m: the flow's scale may change with it
    /// ([`SCALE_PER_HEIGHT_M`]).
    pub fn feed_height(&mut self, up_m: f64) {
        if let Some(h) = self.height_m.replace(up_m) {
            let q = (SCALE_PER_HEIGHT_M * (up_m - h)).powi(2);
            self.fused.widen_scale(q);
        }
    }

    /// The vehicle is at rest at `t_ns` (on the ground before take-off).
    pub fn feed_rest(&mut self, t_ns: u64) {
        self.advance(t_ns);
        let r = REST_SIGMA * REST_SIGMA;
        for u in [Vector2::x(), Vector2::y()] {
            self.fused.update_velocity(&u, 0.0, r);
            self.reference.update_velocity(&u, 0.0, r);
        }
    }

    /// The flow's scale the fused filter has learnt: the flow over it is
    /// the velocity.
    pub fn flow_scale(&self) -> f64 {
        self.fused.scale()
    }

    /// One flow sample at `t_ns`, the attitude's levelled frame, as the
    /// sensor reads it (not yet over [`Self::flow_scale`]): kept for the
    /// windows, and fused while the flow is trusted.
    pub fn feed_flow(&mut self, t_ns: u64, flow: Vector2<f64>) {
        self.advance(t_ns);
        let block = |kf: &Kf, k: usize| kf.p.fixed_view::<2, 2>(k, k).into_owned();
        self.samples.push_back(Sample {
            t_ns,
            flow: flow / self.pin.0,
            flow_fused: flow / self.fused.scale(),
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
        let (h, pred) = self.fused.flow_jacobian();
        let e = flow - pred;
        let seen = e * e.transpose() - h * self.fused.p * h.transpose();
        let cov = self.flow_cov + (seen - self.flow_cov) * FLOW_NOISE_GAIN;
        // Symmetric, and no direction quieter than the floor.
        let eig = (0.5 * (cov + cov.transpose())).symmetric_eigen();
        let floor = FLOW_SIGMA_MIN * FLOW_SIGMA_MIN;
        let vals = eig.eigenvalues.map(|v| v.max(floor));
        self.flow_cov =
            eig.eigenvectors * Matrix2::from_diagonal(&vals) * eig.eigenvectors.transpose();
        self.flow_learnt += 1;
        self.fused.update_flow(flow, &self.flow_cov);
    }

    /// The scans pin every direction at a scan: the velocity they matched
    /// (`chord`, the attitude's levelled frame, about `t_ns`; the filters may
    /// be a scan interval on, which [`MATCH_VELOCITY_SIGMA`] allows for), and
    /// the position they hold. Both filters take it. Along a corridor the
    /// scans' share across is not taken: any direction a little off the
    /// axis would carry the motion prior (the flow) along it into the
    /// reference.
    pub fn scan(&mut self, t_ns: u64, chord: Vector2<f64>) {
        self.advance(t_ns);
        let r = MATCH_VELOCITY_SIGMA * MATCH_VELOCITY_SIGMA;
        for (i, u) in [Vector2::x(), Vector2::y()].iter().enumerate() {
            self.fused.update_velocity(u, chord[i], r);
            self.reference.update_velocity(u, chord[i], r);
            self.fused.hold_position(u);
            self.reference.hold_position(u);
        }
        self.pin = (self.fused.scale(), self.fused.p[(K, K)]);
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
            let (mut flow_f, mut off_fused, mut p_fused) =
                (Vector2::zeros(), Vector2::zeros(), 0.0);
            // The window's first half, for the change across it.
            let half = start + n / 2;
            let (mut flow_1, mut ref_1, mut ref_2) =
                (Vector2::zeros(), Vector2::zeros(), Vector2::zeros());
            for (i, s) in self.samples.range(start..end).enumerate() {
                flow += s.flow;
                carried += s.carried;
                off_ref += s.flow - s.v_ref;
                p_ref += u.dot(&(s.p_ref * u));
                flow_f += s.flow_fused;
                off_fused += s.flow_fused - s.v_fused;
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
            // the accelerometer, the flow over the fused filter's scale.
            let mean = (flow_f - carried) / k_n - (s0.v_fused - s0.carried);
            let span = (last.t_ns - s0.t_ns) as f64 * 1e-9;
            let var = u.dot(&(s0.p_fused * u))
                + qa * span / 2.0
                + u.dot(&(s0.p_bias * u)) * span * span / 4.0
                + noise;
            j.fails |= u.dot(&mean).powi(2) / var > GATE_WINDOW;
            if k + 1 < WINDOWS_NS.len() {
                continue;
            }
            // The longest window against the reference filter, the flow over
            // the scale the scans last taught, whose own error reads as an
            // offset as large as the speed's share of it.
            let v_mean = u.dot(&((ref_1 + ref_2) / k_n));
            let ref_var = p_ref / k_n;
            let scale_var = v_mean * v_mean * self.pin.1 / (self.pin.0 * self.pin.0);
            let nis = u.dot(&(off_ref / k_n)).powi(2) / (ref_var + scale_var + noise);
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
                // The scale the flow was last fused with stays: the
                // reference never learns one.
                let (k, var) = (self.fused.x[K], self.fused.p[(K, K)]);
                self.fused = self.reference.clone();
                self.fused_offset = self.pos_offset;
                self.fused.x[K] = k;
                self.fused.p.row_mut(K).fill(0.0);
                self.fused.p.column_mut(K).fill(0.0);
                self.fused.p[(K, K)] = var;
            }
        }
        self.stalled = stalled;
    }

    /// The pose the matcher reports moved by `d`, attitude frame, at a scan.
    pub fn moved(&mut self, d: Vector2<f64>) {
        self.pose += d;
        if self.rebase {
            self.pos_offset = self.pose - self.reference.position();
            self.fused_offset = self.pose - self.fused.position();
            self.rebase = false;
        }
    }

    /// The velocity the corridor rides while the flow is stalled: the
    /// reference filter's, attitude frame.
    pub fn reference_velocity(&self) -> Vector2<f64> {
        self.reference.velocity()
    }

    /// The position's one sigma along `u`, m. While the flow is fused, the
    /// fused filter's (its scale's uncertainty enters through the
    /// position's covariance with it, so a flow reading a few percent off
    /// adds up with the way flown until an acceleration or a pinned stretch
    /// teaches the scale), plus how far the pose has moved from the fused
    /// filter since the scans last pinned every direction (the pose rides
    /// the flow's recent mean, the filter the accelerometer between
    /// samples). While the flow is stalled, the reference's own error plus
    /// how far the pose moved from it on the flow before.
    pub fn sigma_along(&self, u: &Vector2<f64>) -> f64 {
        if self.stalled {
            let drift = u.dot(&(self.pose - self.reference.position() - self.pos_offset));
            return (self.reference.var_along(0, u).max(0.0) + drift * drift).sqrt();
        }
        let drift = u.dot(&(self.pose - self.fused.position() - self.fused_offset));
        (self.fused.var_along(0, u).max(0.0) + drift * drift)
            .sqrt()
            .max(if self.flow_learnt > 0 {
                SIGMA_FLOOR_M
            } else {
                0.0
            })
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
