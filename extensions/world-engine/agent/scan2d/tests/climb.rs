//! A planar scan cuts the scene at its own height. In a room whose walls are
//! not vertical (rock leaning in over the floor), a take-off draws a
//! different outline every revolution of the climb; matched against the
//! outline the submap holds from the floor, the pose slides by the
//! difference and keeps the offset for the rest of the flight. Fed the
//! height and climb rate, the matcher rides the motion prior through the
//! climb and maps the new height in a submap of its own. A long climb or
//! descent (a shaft) is matched again once it has gone past
//! `max_unmatched_climb_m`: the prior alone would carry the pose into the
//! shaft's walls.

mod common;

use ados_scan2d::{
    HeightSample, ImuSample, LocalSlam, Pose2, PoseOut, Scan2dConfig, ScanInput, VelocitySample,
};
use common::{cast, revolution, Rng, Seg, BEAMS, C1, PERIOD_NS};

/// The take-off: on the floor, a 1 m/s climb, a hover. Seconds and metres.
const GROUND_S: f64 = 3.0;
const FLOOR_H: f64 = 0.35;
const TOP_H: f64 = 2.5;
const CLIMB_RATE: f64 = 1.0;
const CLIMB_S: f64 = (TOP_H - FLOOR_H) / CLIMB_RATE;
const HOVER_S: f64 = 3.0;

/// A 9 x 7 m chamber whose east wall leans in 0.3 m per metre of height and
/// whose south wall 0.15: its outline at height `h`. Within the matcher's
/// search window of the floor's outline at any height of the climb.
fn chamber(h: f64) -> Vec<Seg> {
    let (x1, y0) = (9.0 - 0.3 * h, 0.15 * h);
    vec![
        ([0.0, y0], [x1, y0]),
        ([x1, y0], [x1, 7.0]),
        ([x1, 7.0], [0.0, 7.0]),
        ([0.0, 7.0], [0.0, y0]),
    ]
}

/// Scan-plane height and climb rate at `t`.
fn height(t: f64) -> (f64, f64) {
    if t < GROUND_S {
        (FLOOR_H, 0.0)
    } else if t < GROUND_S + CLIMB_S {
        (FLOOR_H + CLIMB_RATE * (t - GROUND_S), CLIMB_RATE)
    } else {
        (TOP_H, 0.0)
    }
}

/// The take-off fed to `slam` in time order: IMU at 200 Hz, the measured
/// (zero) ground velocity at 50 Hz with 3 cm/s noise, the height at 20 Hz
/// from `heights_from` seconds on (never with `None`), and a scan every
/// period cast at the scan plane's height. The rig holds (4, 3) facing +x
/// throughout. One `(time, pose)` per scan.
fn take_off(slam: &mut LocalSlam, heights_from: Option<f64>) -> Vec<(f64, PoseOut)> {
    let mut rng = Rng::new(11);
    let pose_at = |_t: f64| Pose2::new(4.0, 3.0, 0.0);
    let period_s = PERIOD_NS as f64 * 1e-9;
    let n = ((GROUND_S + CLIMB_S + HOVER_S) / period_s) as u64;
    let mut out = Vec::with_capacity(n as usize);
    let mut k = 0u64;
    for i in 0..n {
        let t = i as f64 * period_s;
        let scan: ScanInput = revolution(&chamber(height(t).0), &pose_at, t, C1, &mut rng);
        let end_ms = ((t + (BEAMS - 1) as f64 / BEAMS as f64 * period_s) * 1000.0) as u64;
        // Every input up to the scan's last return, in 1 ms steps.
        while k <= end_ms {
            let (t_ns, t_s) = (k * 1_000_000, k as f64 * 1e-3);
            if k.is_multiple_of(5) {
                slam.feed_imu(&ImuSample {
                    t_ns,
                    gyro: [0.0, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                });
            }
            if k.is_multiple_of(20) {
                slam.feed_velocity(&VelocitySample {
                    t_ns,
                    v: [0.03 * rng.gauss(), 0.03 * rng.gauss()],
                });
            }
            if k.is_multiple_of(50) && heights_from.is_some_and(|h| t_s >= h) {
                let (up_m, up_rate_mps) = height(t_s);
                slam.feed_height(&HeightSample {
                    t_ns,
                    up_m,
                    up_rate_mps,
                });
            }
            k += 1;
        }
        out.push((t, slam.feed_scan(&scan)));
    }
    out
}

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// The pose error at the end of the hover (the local frame starts at the
/// rig, which never moves).
fn final_error(poses: &[(f64, PoseOut)]) -> f64 {
    let p = poses.last().expect("poses").1;
    p.x.hypot(p.y)
}

fn in_climb(t: f64) -> bool {
    t > GROUND_S + 0.1 && t < GROUND_S + CLIMB_S - 0.1
}

#[test]
fn the_chamber_outline_moves_with_height() {
    // The east wall straight ahead of (4, 3): 4.895 m away from the floor,
    // 4.25 m from 2.5 m up.
    let at = |h: f64| cast(&chamber(h), [4.0, 3.0], 0.0).unwrap();
    assert!((at(FLOOR_H) - 4.895).abs() < 1e-9 && (at(TOP_H) - 4.25).abs() < 1e-9);
}

/// The failure: with no height, the climb's scans and the hover are matched
/// against the floor's outline and the pose slides off the rig.
#[test]
fn a_take_off_without_heights_slides_the_pose() {
    let err = final_error(&take_off(&mut LocalSlam::new(cfg()), None));
    assert!(err > 0.2, "the pose stayed within {err:.3} m");
}

/// Fed the height, the climb's scans are neither matched nor mapped, the
/// pose rides the measured velocity through it, and the hover is matched in
/// a submap of its own height.
#[test]
fn a_take_off_with_heights_keeps_the_pose() {
    let poses = take_off(&mut LocalSlam::new(cfg()), Some(0.0));
    let err = final_error(&poses);
    assert!(err < 0.05, "the pose slid {err:.3} m");
    assert!(
        poses.iter().filter(|(t, _)| in_climb(*t)).all(|(_, p)| p.score == 0.0),
        "a scan of the climb was matched"
    );
    assert!(
        poses.iter().rev().take(10).all(|(_, p)| p.score > 0.0),
        "the hover is not matched"
    );
}

/// Riding the prior through the climb is not enough: matched against the
/// floor's submap, the hover slides as far.
#[test]
fn the_hover_needs_a_submap_of_its_own_height() {
    let mut one_band = cfg();
    one_band.submap_height_band_m = f64::INFINITY;
    let err = final_error(&take_off(&mut LocalSlam::new(one_band), Some(0.0)));
    assert!(err > 0.2, "the pose stayed within {err:.3} m");
}

/// Nor is a submap per height band: matched band by band through the climb,
/// each band's outline drifts under the scans and the slides add up.
#[test]
fn the_climb_rides_the_prior() {
    let mut matched = cfg();
    matched.max_climb_rate_mps = f64::INFINITY;
    let err = final_error(&take_off(&mut LocalSlam::new(matched), Some(0.0)));
    assert!(err > 0.2, "the pose stayed within {err:.3} m");
}

/// A submap started before the first height takes that height: heights
/// that come up a second into the session (an EKF that waits for this pose)
/// still split the floor from the hover. No scan of the climb is mapped.
#[test]
fn submaps_carry_their_height() {
    let mut slam = LocalSlam::recording(cfg());
    let poses = take_off(&mut slam, Some(1.0));
    assert!(final_error(&poses) < 0.08);
    let map = slam.into_map();
    let height_of = |t: f64| {
        let t_ns = (t * 1e9).round() as u64;
        map.nodes
            .iter()
            .find(|n| n.t_ns == t_ns)
            .map(|n| map.submaps[n.submap].height_m)
    };
    assert_eq!(height_of(0.5), Some(Some(FLOOR_H)));
    let last = poses.last().unwrap().0;
    assert_eq!(height_of(last), Some(Some(TOP_H)));
    assert!(
        poses
            .iter()
            .filter(|(t, _)| in_climb(*t))
            .all(|(t, _)| height_of(*t).is_none()),
        "a scan of the climb was mapped"
    );
}

/// A 5 x 4 m shaft with vertical walls: the same outline at every height.
fn shaft() -> Vec<Seg> {
    vec![
        ([0.0, 0.0], [5.0, 0.0]),
        ([5.0, 0.0], [5.0, 4.0]),
        ([5.0, 4.0], [0.0, 4.0]),
        ([0.0, 4.0], [0.0, 0.0]),
    ]
}

/// The shaft descent: a hover, then 12 m down at 1 m/s, then a hover, the
/// rig at (2.5, 2) facing +x. The measured ground velocity reads 0.1 m/s
/// along x that is not there (a flow sensor over a floor 40 m below), so a
/// pose that rides it through the whole descent drifts 1.2 m.
fn descend(slam: &mut LocalSlam) -> Vec<(f64, PoseOut)> {
    const HOLD_S: f64 = 3.0;
    const DOWN_S: f64 = 12.0;
    let height = |t: f64| {
        let down = (t - HOLD_S).clamp(0.0, DOWN_S);
        let rate = if (HOLD_S..HOLD_S + DOWN_S).contains(&t) { -1.0 } else { 0.0 };
        (40.0 - down, rate)
    };
    let mut rng = Rng::new(7);
    let pose_at = |_t: f64| Pose2::new(2.5, 2.0, 0.0);
    let period_s = PERIOD_NS as f64 * 1e-9;
    let n = ((2.0 * HOLD_S + DOWN_S) / period_s) as u64;
    let mut out = Vec::with_capacity(n as usize);
    let mut k = 0u64;
    for i in 0..n {
        let t = i as f64 * period_s;
        let scan: ScanInput = revolution(&shaft(), &pose_at, t, C1, &mut rng);
        let end_ms = ((t + (BEAMS - 1) as f64 / BEAMS as f64 * period_s) * 1000.0) as u64;
        while k <= end_ms {
            let (t_ns, t_s) = (k * 1_000_000, k as f64 * 1e-3);
            if k.is_multiple_of(5) {
                slam.feed_imu(&ImuSample {
                    t_ns,
                    gyro: [0.0, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                });
            }
            if k.is_multiple_of(20) {
                slam.feed_velocity(&VelocitySample {
                    t_ns,
                    v: [0.1 + 0.03 * rng.gauss(), 0.03 * rng.gauss()],
                });
            }
            if k.is_multiple_of(50) {
                let (up_m, up_rate_mps) = height(t_s);
                slam.feed_height(&HeightSample {
                    t_ns,
                    up_m,
                    up_rate_mps,
                });
            }
            k += 1;
        }
        out.push((t, slam.feed_scan(&scan)));
    }
    out
}

/// The pose's worst distance from the rig over a run.
fn max_error(poses: &[(f64, PoseOut)], rig: [f64; 2]) -> f64 {
    poses
        .iter()
        .map(|(_, p)| (p.x - rig[0]).hypot(p.y - rig[1]))
        .fold(0.0, f64::max)
}

/// Past `max_unmatched_climb_m` a descent is a transit: its scans are
/// matched against the shaft's walls and the pose stays on the rig.
#[test]
fn a_shaft_descent_is_matched_past_the_unmatched_climb() {
    let poses = descend(&mut LocalSlam::new(cfg()));
    // The local frame starts at the rig.
    let err = max_error(&poses, [0.0, 0.0]);
    assert!(err < 0.5, "the pose drifted {err:.3} m");
    let late = poses.iter().filter(|(t, _)| (8.0..14.0).contains(t));
    assert!(
        late.clone().count() > 0 && late.clone().all(|(_, p)| p.score > 0.0),
        "a scan of the transit was not matched"
    );
}

/// The failure it fixes: riding the prior down the whole shaft, the pose
/// follows the flow's bias into the wall.
#[test]
fn a_shaft_descent_on_the_prior_alone_drifts() {
    let mut unmatched = cfg();
    unmatched.max_unmatched_climb_m = f64::INFINITY;
    let err = max_error(&descend(&mut LocalSlam::new(unmatched)), [0.0, 0.0]);
    assert!(err > 0.8, "the pose stayed within {err:.3} m");
}

/// A room with vertical walls and two boxes, so a match pins x and y.
fn room() -> Vec<Seg> {
    let mut w = common::rect(-5.0, -4.0, 6.0, 4.0);
    w.extend(common::rect(1.5, 1.0, 2.3, 1.6));
    w.extend(common::rect(-3.0, -2.5, -2.4, -1.5));
    w
}

/// A slow descent with no measured ground velocity: a hover at 6 m, then
/// 0.3 m/s down for 16 s while the wind carries the rig 0.1 m/s along +x,
/// then a hover. The prior alone cannot see the drift. One
/// `(time, truth x, pose)` per scan; the rig starts at (0, 0) facing +x.
fn windy_descent(slam: &mut LocalSlam) -> Vec<(f64, f64, PoseOut)> {
    const HOLD_S: f64 = 2.0;
    const DOWN_S: f64 = 16.0;
    let down = |t: f64| (t - HOLD_S).clamp(0.0, DOWN_S);
    let truth_x = move |t: f64| 0.1 * down(t);
    let mut rng = Rng::new(3);
    let period_s = PERIOD_NS as f64 * 1e-9;
    let n = ((2.0 * HOLD_S + DOWN_S) / period_s) as u64;
    let mut out = Vec::with_capacity(n as usize);
    let mut k = 0u64;
    for i in 0..n {
        let t = i as f64 * period_s;
        let pose_at = |s: f64| Pose2::new(truth_x(s), 0.0, 0.0);
        let scan: ScanInput = revolution(&room(), &pose_at, t, C1, &mut rng);
        let end_ms = ((t + (BEAMS - 1) as f64 / BEAMS as f64 * period_s) * 1000.0) as u64;
        while k <= end_ms {
            let (t_ns, t_s) = (k * 1_000_000, k as f64 * 1e-3);
            if k.is_multiple_of(5) {
                slam.feed_imu(&ImuSample {
                    t_ns,
                    gyro: [0.0, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                });
            }
            if k.is_multiple_of(50) {
                let rate = if (HOLD_S..HOLD_S + DOWN_S).contains(&t_s) { -0.3 } else { 0.0 };
                slam.feed_height(&HeightSample {
                    t_ns,
                    up_m: 6.0 - 0.3 * down(t_s),
                    up_rate_mps: rate,
                });
            }
            k += 1;
        }
        out.push((t, truth_x(t), slam.feed_scan(&scan)));
    }
    out
}

/// Without a measured velocity the descent rides the prior for at most
/// `max_unmatched_climb_s`, then its scans are matched again and the pose
/// follows the drift.
#[test]
fn an_unmeasured_descent_is_matched_again_after_the_time_bound() {
    let poses = windy_descent(&mut LocalSlam::new(cfg()));
    let (_, truth_x, last) = poses.last().unwrap();
    assert!((last.x - truth_x).abs() < 0.15, "pose {} vs truth {truth_x}", last.x);
    let late = poses.iter().filter(|(t, _, _)| (5.0..17.0).contains(t));
    assert!(
        late.clone().count() > 0 && late.clone().all(|(_, _, p)| p.score > 0.0),
        "a scan of the descent past the time bound was not matched"
    );
}

/// The failure it fixes: riding the prior for the first
/// `max_unmatched_climb_m` of the descent, the pose misses the drift.
#[test]
fn an_unmeasured_descent_on_the_prior_alone_misses_the_drift() {
    let mut unbounded = cfg();
    unbounded.max_unmatched_climb_s = f64::INFINITY;
    let poses = windy_descent(&mut LocalSlam::new(unbounded));
    let (_, truth_x, last) = poses.last().unwrap();
    assert!((last.x - truth_x).abs() > 0.5, "pose {} vs truth {truth_x}", last.x);
}
