//! The heading held to the integrated gyro (`PoseOut::heading_ok`): a scan
//! the IMU has not reached rides the extrapolated prior unchecked and never
//! becomes the next scan's reference, and a gap in the IMU leaves the turn
//! across it unchecked.

mod common;

use ados_scan2d::pose::wrap;
use ados_scan2d::{ImuSample, LocalSlam, Pose2, PoseOut, Scan2dConfig};
use common::{rect, revolution, Rng, C1};

const PERIOD_S: f64 = 0.1;
const IMU_DT_S: f64 = 0.005;

/// In place at (3, 2) of a 6 x 4 m room, turning at `rate` rad/s over
/// `[from_s, to_s)`.
fn pivot(rate: f64, from_s: f64, to_s: f64) -> impl Fn(f64) -> Pose2 + Copy {
    move |t: f64| Pose2::new(3.0, 2.0, rate * (t.clamp(from_s, to_s) - from_s))
}

/// A level IMU reporting `rate_at(t)`, every [`IMU_DT_S`] over `(from, to]`.
fn feed_imu(slam: &mut LocalSlam, from: &mut f64, to: f64, rate_at: &dyn Fn(f64) -> f64) {
    while *from + IMU_DT_S <= to + 1e-9 {
        *from += IMU_DT_S;
        slam.feed_imu(&ImuSample {
            t_ns: (*from * 1e9).round() as u64,
            gyro: [0.0, 0.0, rate_at(*from)],
            accel: [0.0, 0.0, 9.81],
        });
    }
}

/// Every revolution of `n` from t = 0, the IMU fed up to each scan's end
/// except where `stalled(t)`; returns `(t, pose)`.
fn fly(
    pose_at: impl Fn(f64) -> Pose2 + Copy,
    rate_at: &dyn Fn(f64) -> f64,
    stalled: &dyn Fn(f64) -> bool,
    n: usize,
) -> Vec<(f64, PoseOut)> {
    let world = rect(0.0, 0.0, 6.0, 4.0);
    let mut rng = Rng::new(11);
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu_t = -IMU_DT_S;
    (0..n)
        .map(|k| {
            let t = k as f64 * PERIOD_S;
            if !stalled(t) {
                feed_imu(&mut slam, &mut imu_t, t + PERIOD_S, rate_at);
            }
            (
                t,
                slam.feed_scan(&revolution(&world, &pose_at, t, C1, &mut rng)),
            )
        })
        .collect()
}

fn err_deg(pose_at: impl Fn(f64) -> Pose2, t: f64, p: &PoseOut) -> f64 {
    wrap(p.yaw - pose_at(t).th).to_degrees()
}

/// The FC's IMU stream stalls after the scan at 2.0 s (fed to 2.1 s) until
/// 2.7 s while the scans keep coming, and the drone turns 30 degrees in
/// between. The scans the IMU has not reached hold the newest sample's
/// heading, unchecked, and are still matched for their position; the first
/// scan it covers again turns by the whole 30 degrees the gyro measured
/// since.
#[test]
fn a_scan_the_imu_has_not_reached_rides_unchecked_and_the_turn_comes_back() {
    let rate = 60f64.to_radians();
    let pose_at = pivot(rate, 2.15, 2.65);
    let rate_at = |t: f64| if (2.15..2.65).contains(&t) { rate } else { 0.0 };
    let out = fly(pose_at, &rate_at, &|t| (2.05..2.65).contains(&t), 50);
    for (t, p) in &out {
        let e = err_deg(pose_at, *t, p);
        if (2.15..2.65).contains(t) {
            assert!(
                !p.heading_ok,
                "t {t:.1}: an unmeasured heading reported checked"
            );
            assert!(p.score > 0.0, "t {t:.1}: an unreached scan was not matched");
        } else if *t >= 0.3 {
            assert!(p.heading_ok, "t {t:.1}: unchecked (err {e:.2} deg)");
            assert!(e.abs() < 1.5, "t {t:.1}: heading err {e:.2} deg");
        }
    }
}

/// The FC's IMU stream stops for good at 4 s, as the drone, walking at
/// 0.5 m/s, stops. The scans keep being matched for their position on the
/// newest sample's heading, every one unchecked: the pose stays on the
/// drone instead of running on at the last velocity.
#[test]
fn a_stopped_imu_keeps_the_scans_matched_and_unchecked() {
    let pose_at = |t: f64| Pose2::new(1.0 + 0.5 * t.min(4.0), 2.0, 0.0);
    let out = fly(pose_at, &|_| 0.0, &|t| t >= 3.95, 60);
    for (t, p) in out.iter().filter(|(t, _)| *t >= 4.05) {
        assert!(!p.heading_ok, "t {t:.1}: checked without an IMU");
        // The local frame is the first scan's pose, (1, 2) heading 0.
        let truth = pose_at(*t);
        let d = (p.x - (truth.x - 1.0)).hypot(p.y - (truth.y - 2.0));
        assert!(d < 0.1, "t {t:.1}: {d:.2} m off the drone");
        assert!(err_deg(pose_at, *t, p).abs() < 1.0, "t {t:.1}: heading off");
    }
}

/// The IMU drops out from 2.0 s to 2.3 s in a turn that runs on past it:
/// 90 degrees/s from 1.9 s to 2.2 s, then 30 degrees/s to 3.0 s. The sample
/// after the gap bridges it with its own 30 degrees/s, so 12 degrees are
/// lost, and the match holds to the gyro. Wherever the heading ends up off
/// by more than 2 degrees, it is reported unchecked.
#[test]
fn a_turn_running_on_past_an_imu_gap_is_unchecked_while_off() {
    let (fast, slow) = (90f64.to_radians(), 30f64.to_radians());
    let pose_at = move |t: f64| {
        let th = fast * (t.clamp(1.9, 2.2) - 1.9) + slow * (t.clamp(2.2, 3.0) - 2.2);
        Pose2::new(3.0, 2.0, th)
    };
    let rate_at = move |t: f64| match t {
        t if (1.9..2.2).contains(&t) => fast,
        t if (2.2..3.0).contains(&t) => slow,
        _ => 0.0,
    };
    let world = rect(0.0, 0.0, 6.0, 4.0);
    let mut rng = Rng::new(11);
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu_t = -IMU_DT_S;
    let mut off = 0;
    for k in 0..50 {
        let t = k as f64 * PERIOD_S;
        if k == 20 {
            // Samples up to 2.0 s, then none until 2.3 s.
            imu_t = 2.3 - IMU_DT_S;
        }
        feed_imu(&mut slam, &mut imu_t, t + PERIOD_S, &rate_at);
        let p = slam.feed_scan(&revolution(&world, &pose_at, t, C1, &mut rng));
        let e = err_deg(pose_at, t, &p);
        if t > 2.0 && e.abs() > 2.0 {
            off += 1;
            assert!(!p.heading_ok, "t {t:.1}: heading err {e:.2} deg reported checked");
        }
    }
    assert!(off > 10, "the gap lost no turn ({off} scans off)");
}

/// The IMU drops out from 2.0 s to 2.3 s with no turn: the scan it has not
/// reached and the scans after it stay unchecked until enough of them agree
/// with the map on one heading; the rest are checked.
#[test]
fn a_gap_in_the_imu_leaves_the_next_scan_unchecked() {
    let pose_at = pivot(0.0, 0.0, 0.0);
    let world = rect(0.0, 0.0, 6.0, 4.0);
    let mut rng = Rng::new(5);
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu_t = -IMU_DT_S;
    let mut flagged = Vec::new();
    for k in 0..40 {
        let t = k as f64 * PERIOD_S;
        if k == 20 {
            // Samples up to 2.0 s (for the scan at 1.9 s), then none until 2.3 s.
            imu_t = 2.3 - IMU_DT_S;
        }
        feed_imu(&mut slam, &mut imu_t, t + PERIOD_S, &|_| 0.0);
        let p = slam.feed_scan(&revolution(&world, &pose_at, t, C1, &mut rng));
        if !p.heading_ok {
            flagged.push(k);
        }
    }
    // Three scans from the first after the gap agree, and the third
    // re-confirms.
    assert_eq!(flagged, vec![21, 22, 23], "only the scans across the gap and its re-confirm");
}
