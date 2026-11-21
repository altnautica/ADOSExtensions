//! Frame-cost benchmark: one 10 Hz frame of a 3D lidar (20 000 returns over
//! 360 x -7..52 degrees, 40 m) flying north at 3 m/s, 2 m above the ground,
//! through a Poisson forest of 1 trunk per 25 m^2.
//!
//! Times are the bench thread's CPU time (`CLOCK_THREAD_CPUTIME_ID`), so
//! time spent waiting for a core on a busy host does not count. Run in
//! release on one core:
//!
//! ```text
//! cargo test --release -p ados-voxmap --test bench_update -- --ignored --nocapture
//! ```

use std::time::Duration;

use ados_protocol::voxmap::VoxmapWriter;
use ados_voxmap::{Pose, VoxMap, VoxMapConfig, VoxTier};

const POINTS: usize = 20_000;
const MAX_RANGE: f32 = 40.0;
const HEIGHT: f32 = 2.0;
const SPEED: f32 = 3.0;
const FRAME_NS: u64 = 100_000_000;

struct Rng(u64);

impl Rng {
    fn f(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// Trunks (north, east, radius) at 1 per 25 m^2 over the flight area, kept
/// 1.5 m clear of the flight line so the scene stays flyable.
fn forest(rng: &mut Rng) -> Vec<[f32; 3]> {
    let (len, width) = (140.0f32, 100.0f32);
    let count = (len * width / 25.0) as usize;
    (0..count)
        .map(|_| {
            let n = rng.f() * len - 40.0;
            let mut e = rng.f() * width - width / 2.0;
            if e.abs() < 1.5 {
                e += 3.0f32.copysign(e);
            }
            [n, e, 0.1 + 0.2 * rng.f()]
        })
        .collect()
}

/// One scan from `at`: the nearest of the ground (down = 0) and every trunk
/// within range, else nothing within `MAX_RANGE` (the return is dropped).
fn scan(at: [f32; 3], trunks: &[[f32; 3]], rng: &mut Rng, out: &mut Vec<[f32; 3]>) {
    out.clear();
    let near: Vec<[f32; 3]> = trunks
        .iter()
        .copied()
        .filter(|t| (t[0] - at[0]).hypot(t[1] - at[1]) < MAX_RANGE + 0.3)
        .collect();
    for _ in 0..POINTS {
        let az = rng.f() * std::f32::consts::TAU;
        let el = (-7.0 + 59.0 * rng.f()).to_radians();
        let d = [el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()];
        let mut t_hit = MAX_RANGE + 1.0;
        if d[2] > 1e-4 {
            t_hit = t_hit.min(-at[2] / d[2]);
        }
        let h = d[0].hypot(d[1]);
        for tr in &near {
            // Ray against a vertical cylinder in the horizontal plane.
            let (px, py) = (at[0] - tr[0], at[1] - tr[1]);
            let a = h * h;
            let b = px * d[0] + py * d[1];
            let c = px * px + py * py - tr[2] * tr[2];
            let disc = b * b - a * c;
            if a < 1e-9 || disc < 0.0 {
                continue;
            }
            let t = (-b - disc.sqrt()) / a;
            if t > 0.0 && t < t_hit {
                t_hit = t;
            }
        }
        if t_hit <= MAX_RANGE {
            out.push([
                at[0] + d[0] * t_hit,
                at[1] + d[1] * t_hit,
                at[2] + d[2] * t_hit,
            ]);
        }
    }
}

/// This thread's CPU time.
fn cpu_now() -> Duration {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, writable timespec for the call's duration.
    let rc = unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
    assert_eq!(rc, 0, "thread CPU clock");
    Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
}

fn pct(sorted: &[Duration], p: f64) -> Duration {
    sorted[((sorted.len() as f64 - 1.0) * p).round() as usize]
}

const WARM: usize = 20;
const FRAMES: usize = 120;
/// The flight is deterministic, so each frame does the same work on every
/// repeat; the per-phase minimum over repeats filters out a busy host.
const REPEATS: usize = 3;

/// Per frame: recentre, insert, decay, distance field, seam publish.
type Split = [Duration; 5];

/// Fly the precomputed `scans` once; per-frame phase times and the steady
/// frames' work counters.
fn fly(
    cfg: VoxMapConfig,
    scans: &[Vec<[f32; 3]>],
) -> (Vec<Split>, ados_voxmap::WorkStats, ados_voxmap::WorkStats) {
    let mut map = VoxMap::new(cfg).unwrap();
    let dir = tempfile::tempdir().unwrap();
    // The seam writer publishes only into a directory nobody else may write.
    std::fs::set_permissions(
        dir.path(),
        std::os::unix::fs::PermissionsExt::from_mode(0o700),
    )
    .unwrap();
    let mut writer = VoxmapWriter::create(&dir.path().join("vox"), map.dims(), None).unwrap();
    let mut splits = Vec::with_capacity(scans.len());
    let mut work = ados_voxmap::WorkStats::default();
    let mut moved = ados_voxmap::WorkStats::default();
    for (k, cloud) in scans.iter().enumerate() {
        let t_ns = k as u64 * FRAME_NS;
        let at = [SPEED * k as f32 * 0.1, 0.0, -HEIGHT];
        let t0 = cpu_now();
        map.recentre(at);
        let recentre_work = map.take_stats();
        let t_r = cpu_now();
        map.insert_cloud(t_ns, at, cloud, MAX_RANGE);
        let t_i = cpu_now();
        map.decay(t_ns);
        let t_d = cpu_now();
        map.tick_esdf();
        let t_e = cpu_now();
        map.to_seam(&mut writer, &Pose::at(at), VoxTier::Lidar3d)
            .unwrap();
        let t_s = cpu_now();
        splits.push([t_r - t0, t_i - t_r, t_d - t_i, t_e - t_d, t_s - t_e]);
        let s = map.take_stats();
        if k >= WARM {
            work.voxels_updated += s.voxels_updated;
            work.rays_cast += s.rays_cast;
            work.ray_steps += s.ray_steps;
            work.obstacles_added += s.obstacles_added;
            work.obstacles_removed += s.obstacles_removed + recentre_work.obstacles_removed;
            work.esdf_visits += s.esdf_visits + recentre_work.esdf_visits;
            moved.esdf_visits += recentre_work.esdf_visits;
        }
    }
    (splits, work, moved)
}

fn mean(v: &[Duration]) -> Duration {
    v.iter().sum::<Duration>() / v.len() as u32
}

/// Fly the scene with `cfg` and print the frame costs against the budget.
fn run(cfg: VoxMapConfig, label: &str) {
    let trunks = forest(&mut Rng(0x2545_f491_4f6c_dd1d));
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let scans: Vec<Vec<[f32; 3]>> = (0..WARM + FRAMES)
        .map(|k| {
            let mut cloud = Vec::with_capacity(POINTS);
            let at = [SPEED * k as f32 * 0.1, 0.0, -HEIGHT];
            scan(at, &trunks, &mut rng, &mut cloud);
            cloud
        })
        .collect();

    let (mut best, work, moved) = fly(cfg, &scans);
    for _ in 1..REPEATS {
        let (again, _, _) = fly(cfg, &scans);
        for (b, a) in best.iter_mut().zip(again) {
            for (bp, ap) in b.iter_mut().zip(a) {
                *bp = (*bp).min(ap);
            }
        }
    }
    let first: Duration = best[0][..4].iter().sum();
    let steady = &best[WARM..];
    let mut update: Vec<Duration> = steady.iter().map(|s| s[..4].iter().sum()).collect();
    let mut total: Vec<Duration> = steady.iter().map(|s| s.iter().sum()).collect();
    let phase = [0, 1, 2, 3, 4].map(|p| mean(&steady.iter().map(|s| s[p]).collect::<Vec<_>>()));
    update.sort();
    total.sort();
    let f = FRAMES as u64;
    eprintln!(
        "[{label}] {:?} voxels at {} m, ~{} returns/frame",
        cfg.dims,
        cfg.res_m,
        scans[WARM].len()
    );
    eprintln!(
        "  update: first frame {first:.2?}; steady mean {:.2?} p50 {:.2?} p95 {:.2?} max {:.2?}",
        mean(&update),
        pct(&update, 0.5),
        pct(&update, 0.95),
        update[update.len() - 1]
    );
    eprintln!(
        "  update + seam publish: mean {:.2?} p95 {:.2?}",
        mean(&total),
        pct(&total, 0.95)
    );
    eprintln!(
        "  phase means: recentre {:.2?}, insert {:.2?}, decay {:.2?}, distance field {:.2?}, seam {:.2?}",
        phase[0], phase[1], phase[2], phase[3], phase[4]
    );
    eprintln!(
        "  work per frame: rays {} stepping {} voxels, voxels updated {}, obstacles +{} -{}, distance visits {} ({} moving the window)",
        work.rays_cast / f,
        work.ray_steps / f,
        work.voxels_updated / f,
        work.obstacles_added / f,
        work.obstacles_removed / f,
        work.esdf_visits / f,
        moved.esdf_visits / f
    );
    let p95 = pct(&update, 0.95);
    let verdict = if p95 <= Duration::from_millis(15) {
        "within"
    } else {
        "over"
    };
    eprintln!("  steady update p95 {p95:.2?}, {verdict} the 15 ms frame budget");
}

#[test]
#[ignore = "benchmark; run in release with --ignored --nocapture"]
fn frame_cost_on_the_default_window() {
    run(VoxMapConfig::default(), "default: 0.2 m, 24 x 24 x 10 m");
}

#[test]
#[ignore = "benchmark; run in release with --ignored --nocapture"]
fn frame_cost_on_the_fine_window() {
    run(VoxMapConfig::fine_window(), "fine: 0.15 m, 30 x 30 x 12 m");
}

#[test]
#[ignore = "benchmark; run in release with --ignored --nocapture"]
fn frame_cost_on_the_fine_window_with_a_5_hz_distance_field() {
    let cfg = VoxMapConfig {
        esdf_every: 2,
        ..VoxMapConfig::fine_window()
    };
    run(cfg, "fine: 0.15 m, 30 x 30 x 12 m, distance field at 5 Hz");
}

/// The moving-window cost against the recentre slack, on both windows.
#[test]
#[ignore = "benchmark; run in release with --ignored --nocapture"]
fn frame_cost_by_recentre_slack() {
    for slack in [0.3, 1.0, 2.0] {
        let default = VoxMapConfig {
            recentre_slack_m: slack,
            ..VoxMapConfig::default()
        };
        run(default, &format!("default window, slack {slack} m"));
        let fine = VoxMapConfig {
            recentre_slack_m: slack,
            ..VoxMapConfig::fine_window()
        };
        run(fine, &format!("fine window, slack {slack} m"));
    }
}
