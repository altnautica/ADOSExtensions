//! Local SLAM with a measured ground velocity (a downward optical-flow
//! sensor over its range to the ground): along a direction the scans do not
//! constrain, the pose follows the measured motion.

mod common;

use std::f64::consts::PI;

use ados_scan2d::{ImuSample, LocalSlam, PoseOut, Scan2dConfig, ScanInput, VelocitySample};
use common::{walk, Path, Rng, BEAMS, C1, PERIOD_NS};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// Feed `imu` and `vel` (each in time order) up to every scan's last return,
/// as the capture service does, then the scan; one pose per scan.
fn replay(scans: &[ScanInput], imu: &[ImuSample], vel: &[VelocitySample]) -> Vec<PoseOut> {
    let mut slam = LocalSlam::new(cfg());
    let (mut i, mut v) = (imu.iter().peekable(), vel.iter().peekable());
    scans
        .iter()
        .map(|s| {
            let end = s.t_ns + s.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
            loop {
                let ti = i.peek().map(|x| x.t_ns).filter(|&t| t <= end);
                let tv = v.peek().map(|x| x.t_ns).filter(|&t| t <= end);
                match (ti, tv) {
                    (Some(a), Some(b)) if b < a => slam.feed_velocity(v.next().unwrap()),
                    (Some(_), _) => slam.feed_imu(i.next().unwrap()),
                    (None, Some(_)) => slam.feed_velocity(v.next().unwrap()),
                    (None, None) => break,
                }
            }
            slam.feed_scan(s)
        })
        .collect()
}

/// A walker goes 14 m down a bare 2 m wide corridor from its closed end and
/// stops. The scans pin the walker across the corridor only; a velocity
/// measured at 50 Hz with 5 % noise carries the pose along it, where the
/// motion prior alone lags the walker.
#[test]
fn bare_corridor_follows_the_measured_velocity() {
    let world = vec![
        ([-2.0, -1.0], [200.0, -1.0]),
        ([-2.0, 1.0], [200.0, 1.0]),
        ([-2.0, -1.0], [-2.0, 1.0]),
    ];
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 0.75)
        .line(13.0)
        .speed_to(0.0, 0.75)
        .wait(2.0);
    let (scans, imu, truth) = walk(&world, &path, C1, 7);
    let mut rng = Rng::new(3);
    let vel: Vec<VelocitySample> = (0..=(path.duration_s() * 50.0) as u64)
        .map(|k| {
            let t = k as f64 * 0.02;
            let (a, b) = (path.at_time((t - 0.01).max(0.0)), path.at_time(t + 0.01));
            let d = (b.x - a.x).hypot(b.y - a.y) / (t + 0.01 - (t - 0.01).max(0.0));
            VelocitySample {
                t_ns: (t * 1e9).round() as u64,
                v: [d * (1.0 + 0.05 * rng.gauss()), 0.02 * rng.gauss()],
            }
        })
        .collect();
    let walked = truth.last().unwrap().x;
    let with = replay(&scans, &imu, &vel);
    let without = replay(&scans, &imu, &[]);
    let degenerate = with
        .iter()
        .filter(|p| p.score > 0.0 && p.degeneracy < cfg().degenerate_ratio)
        .count();
    let (end, end_without) = (with.last().unwrap(), without.last().unwrap());
    let across = with.iter().map(|p| p.y.abs()).fold(0.0, f64::max);
    eprintln!(
        "corridor: walked {walked:.2} m; with the velocity {:.2} m, without {:.2} m; \
         {degenerate} of {} scans degenerate, {across:.3} m across",
        end.x,
        end_without.x,
        with.len()
    );
    assert!(degenerate > with.len() / 2, "{degenerate} degenerate scans");
    assert!((end.x - walked).abs() < 0.2, "ended at {:.3} m of {walked:.3}", end.x);
    assert!(across < 0.05, "{across} m across the corridor");
}

/// One recording of `tests/data/mine_drift_flow.txt`: IMU samples, measured
/// velocities, and revolutions with their true position and yaw.
struct Recording {
    imu: Vec<ImuSample>,
    vel: Vec<VelocitySample>,
    scans: Vec<(ScanInput, [f64; 2], f64)>,
}

/// Parse a `tests/data` flight (format in the file's header).
fn recorded(text: &str) -> Recording {
    let mut rec = Recording {
        imu: Vec::new(),
        vel: Vec::new(),
        scans: Vec::new(),
    };
    for l in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let mut f = l.split_whitespace();
        let kind = f.next().expect("a line kind");
        let v: Vec<i64> = f.map(|x| x.parse().expect("an integer")).collect();
        let t_ns = v[0] as u64 * 1_000;
        match kind {
            "g" => rec.imu.push(ImuSample {
                t_ns,
                gyro: [v[1] as f64 * 1e-6, v[2] as f64 * 1e-6, v[3] as f64 * 1e-6],
                accel: [v[4] as f64 * 1e-3, v[5] as f64 * 1e-3, v[6] as f64 * 1e-3],
            }),
            "v" => rec.vel.push(VelocitySample {
                t_ns,
                v: [v[1] as f64 * 1e-3, v[2] as f64 * 1e-3],
            }),
            "s" => {
                let n = v[4] as usize;
                assert_eq!(v.len(), 5 + 2 * n, "return count");
                let (mut k, mut points, mut t_offset_ns) = (0u64, Vec::new(), Vec::new());
                for p in v[5..].chunks(2) {
                    k += p[0] as u64;
                    let az = k as f64 * 2.0 * PI / BEAMS as f64;
                    let r = p[1] as f64 * 1e-3;
                    points.push([(r * az.cos()) as f32, (r * az.sin()) as f32, 0.0]);
                    t_offset_ns.push((k * PERIOD_NS / BEAMS as u64) as u32);
                }
                let scan = ScanInput {
                    t_ns,
                    points,
                    t_offset_ns,
                };
                rec.scans.push((scan, [v[1] as f64 * 1e-3, v[2] as f64 * 1e-3], v[3] as f64 * 1e-6));
            }
            other => panic!("unknown line kind `{other}`"),
        }
    }
    rec
}

/// Down a mine drift at up to 8 m/s: rough rock walls 5 m apart pin the
/// pose across the drift and barely along it. With the motion prior alone
/// the matched distance fell to 40 % of the true 27.9 m and the pose ended
/// 16.7 m off (the flight controller, fusing that pose, sped up until its
/// EKF failed). With the flight controller's flow velocity it is 5 % short
/// (the flow itself reads 3 % long here; the matches in the weakly
/// constrained scans pull back), and the scans the drift leaves degenerate
/// move with the measured motion.
#[test]
fn mine_drift_keeps_its_length_on_the_flow_velocity() {
    let rec = recorded(include_str!("data/mine_drift_flow.txt"));
    assert_eq!(rec.scans.len(), 107);
    let scans: Vec<ScanInput> = rec.scans.iter().map(|s| s.0.clone()).collect();
    let est = replay(&scans, &rec.imu, &rec.vel);
    // Truth in the local frame: the LiDAR frame at the first scan, levelled.
    let (p0, yaw0) = (rec.scans[0].1, rec.scans[0].2);
    let (s, c) = yaw0.sin_cos();
    let truth: Vec<[f64; 2]> = rec
        .scans
        .iter()
        .map(|(_, p, _)| {
            let (dx, dy) = (p[0] - p0[0], p[1] - p0[1]);
            [c * dx + s * dy, -s * dx + c * dy]
        })
        .collect();
    let (t_end, e_end) = (truth.last().unwrap(), est.last().unwrap());
    let length = t_end[0].hypot(t_end[1]);
    let axis = [t_end[0] / length, t_end[1] / length];
    let along = |x: f64, y: f64| x * axis[0] + y * axis[1];
    let matched = along(e_end.x, e_end.y);
    let end_err = (e_end.x - t_end[0]).hypot(e_end.y - t_end[1]);
    // Along-drift steps of the scans rated degenerate.
    let ratio = cfg().degenerate_ratio;
    let (mut deg_truth, mut deg_est, mut degenerate) = (0.0, 0.0, 0);
    for k in 1..est.len() {
        if est[k].score > 0.0 && est[k].degeneracy < ratio {
            degenerate += 1;
            deg_truth += along(truth[k][0] - truth[k - 1][0], truth[k][1] - truth[k - 1][1]);
            deg_est += along(est[k].x - est[k - 1].x, est[k].y - est[k - 1].y);
        }
    }
    eprintln!(
        "mine drift: {length:.2} m flown, {matched:.2} m matched along it ({:+.1} %), \
         ends {end_err:.2} m off; {degenerate} degenerate scans moved {deg_est:.2} m of {deg_truth:.2}",
        100.0 * (matched / length - 1.0)
    );
    assert!(degenerate >= 5, "{degenerate} degenerate scans");
    assert!(
        (matched / length - 1.0).abs() < 0.08,
        "matched {matched:.2} m of {length:.2}"
    );
    assert!(end_err < 2.5, "ends {end_err:.2} m off");
    assert!(
        (deg_est / deg_truth - 1.0).abs() < 0.05,
        "degenerate scans moved {deg_est:.2} m of {deg_truth:.2}"
    );
}
