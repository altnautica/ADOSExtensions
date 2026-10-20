use super::*;

/// Deterministic normal samples (LCG + Box-Muller).
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn normal(&mut self, sigma: f64) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        sigma * (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// A LiDAR (FLU) 10 cm below and slightly behind a forward camera (OpenCV
/// axes), turned 2° off nominal.
fn truth() -> (Matrix3<f64>, Vector3<f64>) {
    let nominal = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
    let off = exp_so3(&Vector3::new(0.02, -0.025, 0.015));
    (off * nominal, Vector3::new(0.02, 0.10, -0.03))
}

/// Board views: each board centred on the scan plane 0.8-2.05 m ahead and up
/// to `2 · lateral_step` to the side, tilted from facing the camera by
/// `(about x, about y)` degrees; the scan points on it every 0.72° with range
/// noise `sigma`.
fn views(tilts: &[(f64, f64)], lateral_step: f64, sigma: f64, seed: u64) -> Vec<PlaneObs> {
    let (r, t) = truth();
    let mut noise = Noise(seed);
    tilts
        .iter()
        .enumerate()
        .map(|(k, &(ax, ay))| {
            let dist = 0.8 + 0.25 * (k % 6) as f64;
            let lateral = lateral_step * ((k % 5) as f64 - 2.0);
            let c = r * Vector3::new(dist, lateral, 0.0) + t;
            let facing = -c.normalize();
            let tilt = exp_so3(&Vector3::new(ax.to_radians(), ay.to_radians(), 0.0));
            let n = -(tilt * facing);
            let d = n.dot(&c);
            // The board's trace in the scan plane: a·x + b·y = e (LiDAR frame).
            let m = r.transpose() * n;
            let e = d - n.dot(&t);
            let dir = Vector3::new(-m.y, m.x, 0.0).normalize();
            let foot = Vector3::new(m.x, m.y, 0.0) * (e / (m.x * m.x + m.y * m.y));
            let center = foot + dir * dir.dot(&(Vector3::new(dist, lateral, 0.0) - foot));
            let step = (0.72f64).to_radians() * dist;
            let points = (-40..=40)
                .map(|i| center + dir * (i as f64 * step))
                .filter(|p| (p - center).norm() <= 0.3)
                .map(|p| {
                    let range = p.norm();
                    let q = p * ((range + noise.normal(sigma)) / range);
                    [q.x, q.y]
                })
                .collect();
            PlaneObs { n, d, points }
        })
        .collect()
}

/// Twenty views tilted up to 40° about both camera axes, as an operator
/// sweeping the board through its range would give.
fn varied() -> Vec<(f64, f64)> {
    (0..20)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / 20.0 * 3.0;
            let mag = 15.0 + 25.0 * ((k % 4) as f64 / 3.0);
            (mag * a.cos(), mag * a.sin())
        })
        .collect()
}

fn rotation_error_deg(a: &Matrix3<f64>, b: &Matrix3<f64>) -> f64 {
    let c = ((a.transpose() * b).trace() - 1.0) / 2.0;
    c.clamp(-1.0, 1.0).acos().to_degrees()
}

/// Twenty board poses, each held for five scans (half a second of a 10 Hz
/// LiDAR), with 1 cm range noise.
#[test]
fn synthetic_boards_recover_the_extrinsic() {
    let (r_true, t_true) = truth();
    for seed in [1, 2, 3, 4, 5] {
        let obs: Vec<PlaneObs> = (0..5)
            .flat_map(|j| views(&varied(), 0.15, 0.01, seed * 10 + j))
            .collect();
        assert!(obs.iter().all(|o| o.points.len() >= 20));
        let x = solve(&obs).unwrap();
        let dt = (x.t - t_true).norm();
        let dr = rotation_error_deg(&x.r, &r_true);
        assert!(dt < 0.01, "seed {seed}: translation off by {dt} m");
        assert!(dr < 0.5, "seed {seed}: rotation off by {dr} deg");
        assert!(x.rms_m < MAX_RMS_M && x.rms_m > 0.005, "rms {}", x.rms_m);
    }
}

#[test]
fn noise_free_boards_are_exact() {
    let (r_true, t_true) = truth();
    let x = solve(&views(&varied(), 0.15, 0.0, 1)).unwrap();
    assert!((x.t - t_true).norm() < 1e-6);
    assert!(rotation_error_deg(&x.r, &r_true) < 1e-4);
    assert!(x.rms_m < 1e-6);
}

#[test]
fn too_few_or_untilted_views_are_refused() {
    let few = views(&varied()[..9], 0.15, 0.01, 1);
    assert_eq!(
        solve(&few).unwrap_err().message,
        "need at least 10 board views at varied tilt; have 9"
    );
    // Twelve views tilted about one axis only.
    let one_axis: Vec<(f64, f64)> = (0..12).map(|i| (-25.0 + 5.0 * i as f64, 0.0)).collect();
    let spread = tilt_spread_deg(&views(&one_axis, 0.08, 0.0, 1));
    assert!(spread[0] >= MIN_TILT_SPREAD_DEG, "{spread:?}");
    assert_eq!(
        solve(&views(&one_axis, 0.08, 0.01, 1)).unwrap_err().message,
        "need at least 10 board views at varied tilt; have 12"
    );
}
