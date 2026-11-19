//! Synthetic planar LiDAR: line-segment worlds, a walker path, a ray caster
//! with range noise, and an IMU, all deterministic.

#![allow(dead_code)]

use std::f64::consts::PI;

use ados_scan2d::{ImuSample, Pose2, ScanInput};

/// Beams per revolution and revolution period of the simulated sensor.
pub const BEAMS: usize = 500;
pub const PERIOD_NS: u64 = 100_000_000;

pub type Seg = ([f64; 2], [f64; 2]);

/// Splitmix64 with Box-Muller normals.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn gauss(&mut self) -> f64 {
        let u = self.uniform().max(1e-300);
        let v = self.uniform();
        (-2.0 * u.ln()).sqrt() * (2.0 * PI * v).cos()
    }
}

/// Closed outline of the axis-aligned box `(x0, y0)`-`(x1, y1)`.
pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Seg> {
    vec![
        ([x0, y0], [x1, y0]),
        ([x1, y0], [x1, y1]),
        ([x1, y1], [x0, y1]),
        ([x0, y1], [x0, y0]),
    ]
}

/// Distance along the ray from `o` at heading `a` to the nearest segment.
pub fn cast(world: &[Seg], o: [f64; 2], a: f64) -> Option<f64> {
    let (dx, dy) = (a.cos(), a.sin());
    let mut best: Option<f64> = None;
    for &(p, q) in world {
        let (ex, ey) = (q[0] - p[0], q[1] - p[1]);
        let den = dx * ey - dy * ex;
        if den.abs() < 1e-12 {
            continue;
        }
        let (wx, wy) = (p[0] - o[0], p[1] - o[1]);
        let t = (wx * ey - wy * ex) / den;
        let u = (wx * dy - wy * dx) / den;
        if t > 1e-9 && (-1e-12..=1.0 + 1e-12).contains(&u) && best.is_none_or(|b| t < b) {
            best = Some(t);
        }
    }
    best
}

/// A walker path built from straight runs and arcs, walked at constant speed.
#[derive(Clone)]
pub struct Path {
    start: Pose2,
    pieces: Vec<Piece>,
    speed: f64,
}

/// `len` metres at curvature `k`, speed changing linearly from `v0` to `v1`
/// over `dur` seconds.
#[derive(Clone, Copy)]
struct Piece {
    len: f64,
    k: f64,
    v0: f64,
    v1: f64,
    dur: f64,
}

impl Path {
    /// A walker standing at `(x, y)` facing `heading`.
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Path {
            start: Pose2::new(x, y, heading),
            pieces: Vec::new(),
            speed: 0.0,
        }
    }

    /// Stand still for `s` seconds (speed must be 0).
    pub fn wait(mut self, s: f64) -> Self {
        assert!(self.speed == 0.0);
        self.pieces.push(Piece {
            len: 0.0,
            k: 0.0,
            v0: 0.0,
            v1: 0.0,
            dur: s,
        });
        self
    }

    /// Straight for `len` metres while the speed changes evenly to `v`.
    pub fn speed_to(mut self, v: f64, len: f64) -> Self {
        let dur = 2.0 * len / (self.speed + v);
        self.pieces.push(Piece {
            len,
            k: 0.0,
            v0: self.speed,
            v1: v,
            dur,
        });
        self.speed = v;
        self
    }

    /// Straight for `len` metres at the current speed.
    pub fn line(mut self, len: f64) -> Self {
        let v = self.speed;
        self.pieces.push(Piece {
            len,
            k: 0.0,
            v0: v,
            v1: v,
            dur: len / v,
        });
        self
    }

    /// Arc of `radius` turning by `turn` radians (positive = left) at the
    /// current speed.
    pub fn arc(mut self, radius: f64, turn: f64) -> Self {
        let (v, len) = (self.speed, radius * turn.abs());
        self.pieces.push(Piece {
            len,
            k: turn.signum() / radius,
            v0: v,
            v1: v,
            dur: len / v,
        });
        self
    }

    pub fn duration_s(&self) -> f64 {
        self.pieces.iter().map(|p| p.dur).sum()
    }

    pub fn at_time(&self, t_s: f64) -> Pose2 {
        let mut p = self.start;
        let mut t = t_s.max(0.0);
        for pc in &self.pieces {
            let dt = t.min(pc.dur);
            let a = (pc.v1 - pc.v0) / pc.dur;
            let d = (pc.v0 * dt + 0.5 * a * dt * dt).min(pc.len);
            p = advance(p, d, pc.k);
            t -= dt;
            if t <= 0.0 {
                break;
            }
        }
        p
    }
}
/// Move `d` metres along an arc of curvature `k` from `p`.
fn advance(p: Pose2, d: f64, k: f64) -> Pose2 {
    if k.abs() < 1e-12 {
        return Pose2::new(p.x + d * p.th.cos(), p.y + d * p.th.sin(), p.th);
    }
    let th = p.th + k * d;
    Pose2::new(
        p.x + (th.sin() - p.th.sin()) / k,
        p.y - (th.cos() - p.th.cos()) / k,
        th,
    )
}

/// Sensor parameters of a simulated revolution.
#[derive(Clone, Copy)]
pub struct Sensor {
    pub min_range: f64,
    pub max_range: f64,
    pub sigma: f64,
}

pub const C1: Sensor = Sensor {
    min_range: 0.05,
    max_range: 12.0,
    sigma: 0.03,
};

/// One revolution starting at `t_s`, each beam cast from the pose at its own
/// firing time (`pose_at`), points in the sensor frame at that time.
pub fn revolution(
    world: &[Seg],
    pose_at: &dyn Fn(f64) -> Pose2,
    t_s: f64,
    sensor: Sensor,
    rng: &mut Rng,
) -> ScanInput {
    let mut scan = ScanInput {
        t_ns: (t_s * 1e9).round() as u64,
        ..Default::default()
    };
    for k in 0..BEAMS {
        let off_ns = k as u64 * PERIOD_NS / BEAMS as u64;
        let p = pose_at(t_s + off_ns as f64 * 1e-9);
        let phi = 2.0 * PI * k as f64 / BEAMS as f64;
        let Some(d) = cast(world, [p.x, p.y], p.th + phi) else {
            continue;
        };
        if d < sensor.min_range || d > sensor.max_range {
            continue;
        }
        let r = d + sensor.sigma * rng.gauss();
        scan.points
            .push([(r * phi.cos()) as f32, (r * phi.sin()) as f32, 0.0]);
        scan.t_offset_ns.push(off_ns as u32);
    }
    scan
}

/// Scans every period over the path's duration, with a level IMU at 200 Hz
/// reporting the path's turn rate.
pub fn walk(
    world: &[Seg],
    path: &Path,
    sensor: Sensor,
    seed: u64,
) -> (Vec<ScanInput>, Vec<ImuSample>, Vec<Pose2>) {
    let mut rng = Rng::new(seed);
    let pose_at = |t: f64| path.at_time(t);
    let n = (path.duration_s() / (PERIOD_NS as f64 * 1e-9)).floor() as usize;
    let mut scans = Vec::with_capacity(n);
    let mut truth = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 * PERIOD_NS as f64 * 1e-9;
        scans.push(revolution(world, &pose_at, t, sensor, &mut rng));
        truth.push(pose_at(t));
    }
    let dt = 0.005;
    let imu = (0..=(path.duration_s() / dt) as usize)
        .map(|i| {
            let t = i as f64 * dt;
            let h = 1e-4;
            let rate = ados_scan2d::pose::wrap(pose_at(t + h).th - pose_at((t - h).max(0.0)).th)
                / (t + h - (t - h).max(0.0));
            ImuSample {
                t_ns: (t * 1e9).round() as u64,
                gyro: [0.0, 0.0, rate],
                accel: [0.0, 0.0, 9.81],
            }
        })
        .collect();
    (scans, imu, truth)
}

/// `b` expressed in the frame of `a`.
pub fn relative(a: &Pose2, b: &Pose2) -> Pose2 {
    a.inverse() * *b
}
/// Planar absolute trajectory error: RMS distance between estimated and true
/// positions after the rigid 2D alignment that minimises it (Umeyama 1991,
/// without scale).
pub fn ate_xy(est: &[Pose2], truth: &[Pose2]) -> f64 {
    let n = est.len() as f64;
    let mean = |v: &[Pose2]| {
        let (sx, sy) = v.iter().fold((0.0, 0.0), |(a, b), p| (a + p.x, b + p.y));
        (sx / n, sy / n)
    };
    let ((ex, ey), (tx, ty)) = (mean(est), mean(truth));
    let (mut sxx, mut sxy, mut syx, mut syy) = (0.0, 0.0, 0.0, 0.0);
    for (e, t) in est.iter().zip(truth) {
        let (a, b) = ((e.x - ex, e.y - ey), (t.x - tx, t.y - ty));
        sxx += a.0 * b.0;
        sxy += a.0 * b.1;
        syx += a.1 * b.0;
        syy += a.1 * b.1;
    }
    let th = (sxy - syx).atan2(sxx + syy);
    let (s, c) = th.sin_cos();
    let sum: f64 = est
        .iter()
        .zip(truth)
        .map(|(e, t)| {
            let (a, b) = (e.x - ex, e.y - ey);
            let (x, y) = (c * a - s * b + tx, s * a + c * b + ty);
            (x - t.x).powi(2) + (y - t.y).powi(2)
        })
        .sum();
    (sum / n).sqrt()
}
