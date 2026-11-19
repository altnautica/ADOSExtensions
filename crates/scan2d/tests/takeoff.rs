//! Recorded take-offs in a dark mine tunnel on external navigation, with
//! no ground velocity: the downward flow sensor reads nothing without
//! light. The drone climbs 1.05 m in 1.8 s, the scans ride the prior
//! through the climb, and the first scan matched after it starts the new
//! height's submap at the pose the ride left: whatever offset the pose
//! holds then, the map keeps, and every survey map placed on this pose
//! moves with it.

mod common;

use std::f64::consts::PI;

use ados_scan2d::{HeightSample, ImuSample, LocalSlam, PoseOut, Scan2dConfig, ScanInput};
use common::{BEAMS, PERIOD_NS};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// One recording: IMU samples, heights, and revolutions with their true
/// position and yaw.
struct Recording {
    imu: Vec<ImuSample>,
    heights: Vec<HeightSample>,
    scans: Vec<(ScanInput, [f64; 2], f64)>,
}

/// Parse a `tests/data` flight (format in the file's header).
fn recorded(text: &str) -> Recording {
    let mut rec = Recording {
        imu: Vec::new(),
        heights: Vec::new(),
        scans: Vec::new(),
    };
    for l in text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
    {
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
            "h" => rec.heights.push(HeightSample {
                t_ns,
                up_m: v[1] as f64 * 1e-3,
                up_rate_mps: v[2] as f64 * 1e-3,
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
                rec.scans.push((
                    scan,
                    [v[1] as f64 * 1e-3, v[2] as f64 * 1e-3],
                    v[3] as f64 * 1e-6,
                ));
            }
            other => panic!("unknown line kind `{other}`"),
        }
    }
    rec
}

/// Feed the IMU and the heights (in time order) up to every scan's last
/// return, as the capture service does, then the scan; one pose per scan.
fn replay(slam: &mut LocalSlam, rec: &Recording) -> Vec<PoseOut> {
    let (mut i, mut h) = (rec.imu.iter().peekable(), rec.heights.iter().peekable());
    rec.scans
        .iter()
        .map(|(s, _, _)| {
            let end = s.t_ns + s.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
            loop {
                let ti = i.peek().map(|x| x.t_ns).filter(|&t| t <= end);
                let th = h.peek().map(|x| x.t_ns).filter(|&t| t <= end);
                match (ti, th) {
                    (Some(a), Some(b)) if b < a => slam.feed_height(h.next().unwrap()),
                    (Some(_), _) => slam.feed_imu(i.next().unwrap()),
                    (None, Some(_)) => slam.feed_height(h.next().unwrap()),
                    (None, None) => break,
                }
            }
            slam.feed_scan(s)
        })
        .collect()
}

/// The horizontal error of every pose against the truth, in the local
/// frame (the LiDAR frame at the first scan, levelled), with the scan's
/// time from the first scan, s.
fn errors(rec: &Recording, est: &[PoseOut]) -> Vec<(f64, f64, f64)> {
    let (p0, yaw0) = (rec.scans[0].1, rec.scans[0].2);
    let (s, c) = yaw0.sin_cos();
    let t0 = rec.scans[0].0.t_ns;
    rec.scans
        .iter()
        .zip(est)
        .map(|((scan, p, _), e)| {
            let (dx, dy) = (p[0] - p0[0], p[1] - p0[1]);
            let (tx, ty) = (c * dx + s * dy, -s * dx + c * dy);
            ((scan.t_ns - t0) as f64 * 1e-9, e.x - tx, e.y - ty)
        })
        .collect()
}

/// The take-off of a recording: the error at the first scan matched after
/// the climb (the pose a new height's submap starts at) and its sigma, the
/// truth's and the pose's horizontal move over the climb, and the mean
/// error from then to the end (the offset the bore's map keeps; one scan's
/// match is good to a few centimetres either side of it). The climb is the
/// run of scans ridden unmatched after the first `ground_s` seconds.
struct TakeOff {
    err_after_climb: f64,
    sigma_after_climb: f64,
    truth_moved: f64,
    pose_moved: f64,
    mean_after: f64,
}

fn take_off(text: &str, ground_s: f64) -> TakeOff {
    take_off_from(text, ground_s, 0.0)
}

/// [`take_off`] with every height raised by `lift_m`: the same climb begun
/// in flight, `lift_m` over the height origin.
fn take_off_from(text: &str, ground_s: f64, lift_m: f64) -> TakeOff {
    let mut rec = recorded(text);
    for h in &mut rec.heights {
        h.up_m += lift_m;
    }
    let est = replay(&mut LocalSlam::new(cfg()), &rec);
    let err = errors(&rec, &est);
    let first_ridden = err
        .iter()
        .zip(&est)
        .position(|((t, _, _), p)| *t > ground_s && p.score == 0.0)
        .expect("a ridden scan");
    let after = first_ridden
        + est[first_ridden..]
            .iter()
            .position(|p| p.score > 0.0)
            .expect("a matched scan after the climb");
    let before = first_ridden - 1;
    let truth = |k: usize| {
        let (p0, yaw0) = (rec.scans[0].1, rec.scans[0].2);
        let (s, c) = yaw0.sin_cos();
        let (dx, dy) = (rec.scans[k].1[0] - p0[0], rec.scans[k].1[1] - p0[1]);
        [c * dx + s * dy, -s * dx + c * dy]
    };
    let (tb, ta) = (truth(before), truth(after));
    let (_, ex, ey) = err[after];
    TakeOff {
        err_after_climb: ex.hypot(ey),
        sigma_after_climb: est[after].sigma_xy_m,
        truth_moved: (ta[0] - tb[0]).hypot(ta[1] - tb[1]),
        pose_moved: (est[after].x - est[before].x).hypot(est[after].y - est[before].y),
        mean_after: err[after..]
            .iter()
            .map(|(_, x, y)| x.hypot(*y))
            .sum::<f64>()
            / (err.len() - after) as f64,
    }
}

/// A take-off whose flight controller lifted off with its velocity settled:
/// the truth moves 0.015 m over the climb. Coasting through the climb on
/// the velocity the floor's matches last showed (their noise) carried the
/// pose 0.4 m off here, and the bore's map kept the offset; holding the
/// position, the pose leaves the climb within 0.05 m of the truth and stays
/// there down the bore.
#[test]
fn a_still_take_off_without_flow_keeps_the_pose() {
    let r = take_off(include_str!("data/tunnel_takeoff_still.txt"), 3.9);
    eprintln!(
        "still: after the climb {:.3} m off (sigma {:.3}), truth moved {:.3}, pose {:.3}; mean after {:.3}",
        r.err_after_climb, r.sigma_after_climb, r.truth_moved, r.pose_moved, r.mean_after
    );
    assert!(
        r.err_after_climb < 0.05,
        "{:.3} m off after the climb",
        r.err_after_climb
    );
    assert!(
        r.mean_after < 0.05,
        "{:.3} m off down the bore",
        r.mean_after
    );
}

/// A take-off whose flight controller lifted off with a 0.03 m/s velocity
/// error and drifted 0.09 m over the climb: nothing on board measures that
/// in the dark. The pose holds its position through the climb rather than
/// coasting, and its sigma grows by 0.03 m/s for the time held, enough to
/// keep the drift it could not see within two sigma without claiming more
/// than 0.114 m after a 1.8 s climb.
#[test]
fn a_drifting_take_off_without_flow_holds_and_says_so() {
    let r = take_off(include_str!("data/tunnel_takeoff_drift.txt"), 3.9);
    eprintln!(
        "drift: after the climb {:.3} m off (sigma {:.3}), truth moved {:.3}, pose {:.3}; mean after {:.3}",
        r.err_after_climb, r.sigma_after_climb, r.truth_moved, r.pose_moved, r.mean_after
    );
    assert!(
        r.pose_moved < 0.02,
        "the pose moved {:.3} m through the climb",
        r.pose_moved
    );
    assert!(
        r.err_after_climb <= 2.0 * r.sigma_after_climb,
        "{:.3} m off and says {:.3}",
        r.err_after_climb,
        r.sigma_after_climb
    );
    assert!(
        r.sigma_after_climb <= 0.114,
        "sigma {:.3}",
        r.sigma_after_climb
    );
}

/// The same climb begun 1.5 m up, in flight: only a take-off starts from
/// rest, so a climb that begins in the air keeps riding the velocity its
/// matches last showed (here their noise, which carries the pose 0.4 m),
/// since in flight that velocity may be the drone's real sideways motion.
#[test]
fn a_climb_begun_in_flight_is_not_held() {
    let r = take_off_from(include_str!("data/tunnel_takeoff_still.txt"), 3.9, 1.5);
    eprintln!(
        "in flight: after the climb {:.3} m off (sigma {:.3}), pose moved {:.3}",
        r.err_after_climb, r.sigma_after_climb, r.pose_moved
    );
    assert!(
        r.pose_moved > 0.1,
        "the pose moved only {:.3} m: the climb was held",
        r.pose_moved
    );
}
