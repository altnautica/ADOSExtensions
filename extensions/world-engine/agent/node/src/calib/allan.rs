//! Overlapping Allan deviation and the IEEE 952 slope fits that turn it into
//! continuous-time IMU noise parameters (Kalibr units).
//!
//! A white-noise density `N` shows as `σ(τ) = N / √τ` (slope −1/2, read at
//! τ = 1 s); a random walk `K` as `σ(τ) = K √(τ / 3)` (slope +1/2, read at
//! τ = 3 s). Each fit takes every curve point whose local log-log slope is
//! near the model slope and uses the median intercept, so a bias-instability
//! plateau or quantization noise at short τ does not pull the estimate.

/// An Allan deviation curve over log-spaced cluster times.
#[derive(Debug, Clone, PartialEq)]
pub struct AllanCurve {
    /// Cluster times τ, seconds, increasing.
    pub taus: Vec<f64>,
    /// Allan deviation at each τ, in the units of the input samples.
    pub adev: Vec<f64>,
}

/// Points per decade of τ.
const PER_DECADE: f64 = 12.0;

/// Half-width, in curve points, of the window a local slope is measured over.
const SLOPE_HALF_WINDOW: usize = 3;

/// How far a local slope may sit from the model slope and still be fitted.
const SLOPE_TOLERANCE: f64 = 0.1;

/// Cluster sizes `m` (in samples), log-spaced from 1 to `max_m`.
fn cluster_sizes(max_m: usize) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    if max_m == 0 {
        return out;
    }
    let decades = (max_m as f64).log10();
    let steps = (decades * PER_DECADE).ceil() as usize;
    for i in 0..=steps {
        let m = 10f64.powf(i as f64 / PER_DECADE).round() as usize;
        let m = m.clamp(1, max_m);
        if out.last() != Some(&m) {
            out.push(m);
        }
    }
    out
}

/// Overlapping Allan deviation of uniformly spaced samples `x` taken every
/// `tau0` seconds. Cluster times run up to half the record length.
pub fn overlapping_adev(x: &[f64], tau0: f64) -> AllanCurve {
    let n = x.len();
    // Prefix sums: s[k] = x[0] + ... + x[k-1].
    let mut s = Vec::with_capacity(n + 1);
    let mut acc = 0.0f64;
    s.push(0.0);
    for v in x {
        acc += v;
        s.push(acc);
    }
    let mut taus = Vec::new();
    let mut adev = Vec::new();
    for m in cluster_sizes(n.saturating_sub(1) / 2) {
        let terms = n + 1 - 2 * m;
        let mut sum = 0.0;
        for k in 0..terms {
            let d = s[k + 2 * m] - 2.0 * s[k + m] + s[k];
            sum += d * d;
        }
        let mf = m as f64;
        let avar = sum / (2.0 * mf * mf * terms as f64);
        taus.push(mf * tau0);
        adev.push(avar.sqrt());
    }
    AllanCurve { taus, adev }
}

impl AllanCurve {
    /// Local log-log slope at every point, over a few neighbours each side.
    fn slopes(&self) -> Vec<f64> {
        let n = self.taus.len();
        (0..n)
            .map(|i| {
                let lo = i.saturating_sub(SLOPE_HALF_WINDOW);
                let hi = (i + SLOPE_HALF_WINDOW).min(n.saturating_sub(1));
                if hi <= lo || self.adev[lo] <= 0.0 || self.adev[hi] <= 0.0 {
                    return f64::NAN;
                }
                (self.adev[hi].ln() - self.adev[lo].ln())
                    / (self.taus[hi].ln() - self.taus[lo].ln())
            })
            .collect()
    }

    /// Median of `ln σ − slope·ln τ` over the points near `slope` with
    /// `τ <= tau_max`; when none qualify, the single point whose slope is
    /// closest. `None` for an empty or all-zero curve.
    fn intercept(&self, slope: f64, tau_max: f64) -> Option<f64> {
        let slopes = self.slopes();
        let usable: Vec<usize> = (0..self.taus.len())
            .filter(|&i| self.taus[i] <= tau_max && self.adev[i] > 0.0 && slopes[i].is_finite())
            .collect();
        let mut picked: Vec<f64> = usable
            .iter()
            .filter(|&&i| (slopes[i] - slope).abs() <= SLOPE_TOLERANCE)
            .map(|&i| self.adev[i].ln() - slope * self.taus[i].ln())
            .collect();
        if picked.is_empty() {
            let best = usable.iter().copied().min_by(|&a, &b| {
                (slopes[a] - slope)
                    .abs()
                    .total_cmp(&(slopes[b] - slope).abs())
            })?;
            picked.push(self.adev[best].ln() - slope * self.taus[best].ln());
        }
        picked.sort_by(f64::total_cmp);
        let mid = picked.len() / 2;
        Some(if picked.len() % 2 == 1 {
            picked[mid]
        } else {
            0.5 * (picked[mid - 1] + picked[mid])
        })
    }

    /// White-noise density: the −1/2 slope line read at τ = 1 s.
    pub fn white_noise_density(&self, tau_max: f64) -> Option<f64> {
        self.intercept(-0.5, tau_max).map(f64::exp)
    }

    /// Random-walk density: the +1/2 slope line read at τ = 3 s.
    pub fn random_walk(&self, tau_max: f64) -> Option<f64> {
        self.intercept(0.5, tau_max)
            .map(|b| (b + 0.5 * 3f64.ln()).exp())
    }
}

/// Noise of one sensor axis set (three axes, averaged).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisNoise {
    /// White-noise density (units/√Hz).
    pub nd: f64,
    /// Random-walk density (units/s/√Hz).
    pub rw: f64,
}

/// Noise of three axes: mean of the per-axis fits. `tau_max` bounds the
/// cluster times trusted for the fits (a tenth of the record is customary).
pub fn axes_noise(axes: [&[f64]; 3], tau0: f64, tau_max: f64) -> Option<AxisNoise> {
    let mut nd = 0.0;
    let mut rw = 0.0;
    for x in axes {
        let curve = overlapping_adev(x, tau0);
        nd += curve.white_noise_density(tau_max)?;
        rw += curve.random_walk(tau_max)?;
    }
    Some(AxisNoise {
        nd: nd / 3.0,
        rw: rw / 3.0,
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Deterministic standard-normal source (xorshift64* + Box-Muller).
    pub(crate) struct Gauss(u64, Option<f64>);

    impl Gauss {
        pub(crate) fn new(seed: u64) -> Self {
            Self(seed.max(1), None)
        }

        fn uniform(&mut self) -> f64 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            let v = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
            (v as f64 + 0.5) / (1u64 << 53) as f64
        }

        pub(crate) fn next(&mut self) -> f64 {
            if let Some(v) = self.1.take() {
                return v;
            }
            let (u1, u2) = (self.uniform(), self.uniform());
            let r = (-2.0 * u1.ln()).sqrt();
            let a = 2.0 * std::f64::consts::PI * u2;
            self.1 = Some(r * a.sin());
            r * a.cos()
        }
    }

    /// `n` samples at `dt` of white noise density `nd` plus a random walk of
    /// density `rw`.
    pub(crate) fn synth(g: &mut Gauss, n: usize, dt: f64, nd: f64, rw: f64) -> Vec<f64> {
        let mut bias = 0.0;
        (0..n)
            .map(|_| {
                bias += rw * dt.sqrt() * g.next();
                bias + nd / dt.sqrt() * g.next()
            })
            .collect()
    }

    #[test]
    fn pure_white_noise_reads_its_density_at_one_second() {
        let mut g = Gauss::new(7);
        let x = synth(&mut g, 200_000, 0.01, 0.02, 0.0);
        let c = overlapping_adev(&x, 0.01);
        let nd = c.white_noise_density(200.0).unwrap();
        assert!((nd / 0.02 - 1.0).abs() < 0.05, "nd {nd}");
        assert!(c.taus.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn white_noise_and_random_walk_are_recovered() {
        let mut g = Gauss::new(42);
        let dt = 0.1;
        let n = 8 * 3600 * 10;
        let (nd, rw) = (0.005, 0.002);
        let ax: Vec<Vec<f64>> = (0..3).map(|_| synth(&mut g, n, dt, nd, rw)).collect();
        let tau_max = n as f64 * dt / 10.0;
        let got = axes_noise([&ax[0], &ax[1], &ax[2]], dt, tau_max).unwrap();
        assert!((got.nd / nd - 1.0).abs() < 0.1, "nd {}", got.nd);
        assert!((got.rw / rw - 1.0).abs() < 0.1, "rw {}", got.rw);
    }
}
