//! The heading held to the integrated gyro (`PoseOut::heading_ok`): a scan
//! the IMU has not reached rides the extrapolated prior unchecked and never
//! becomes the next scan's reference, a gap in the IMU leaves the turn
//! across it unchecked, and a turn lost in such a gap is reported unchecked
//! while the scans disagree with it, the match following the scans once
//! they have disagreed for three scans in a row.

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
/// between. The scans the IMU has not reached go out on the last rate
/// extrapolated (none), unchecked; the first scan it covers again turns by
/// the whole 30 degrees the gyro measured since the last checked scan.
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
                "t {t:.1}: an extrapolated heading reported checked"
            );
            assert_eq!(p.score, 0.0, "t {t:.1}: matched on an extrapolated heading");
        } else if *t >= 0.3 {
            assert!(p.heading_ok, "t {t:.1}: unchecked (err {e:.2} deg)");
            assert!(e.abs() < 1.5, "t {t:.1}: heading err {e:.2} deg");
        }
    }
}

/// The IMU drops out from 2.0 s to 2.3 s while the drone snaps round 8
/// degrees (2.0 to 2.1 s): the samples after the gap read no turn, so the
/// integration bridges it with none. Held to the gyro by the rotation prior,
/// the heading used to stay 8 degrees off for good, every new submap
/// anchored on it. After the gap the heading is suspect; the scans clearly
/// disagree with it, unchecked, and after three in a row the match follows
/// them back to the true heading: a jump the gyro did not see, unchecked
/// until the heading has held to the gyro for a second.
#[test]
fn a_turn_lost_in_an_imu_gap_is_taken_from_the_scans() {
    let rate = 80f64.to_radians();
    let pose_at = pivot(rate, 2.0, 2.1);
    let rate_at = |t: f64| if (2.0..2.1).contains(&t) { rate } else { 0.0 };
    let world = rect(0.0, 0.0, 6.0, 4.0);
    let mut rng = Rng::new(11);
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu_t = -IMU_DT_S;
    let mut out = Vec::new();
    for k in 0..50 {
        let t = k as f64 * PERIOD_S;
        if k == 20 {
            // Samples up to 2.0 s (for the scan at 1.9 s), then none until 2.3 s.
            imu_t = 2.3 - IMU_DT_S;
        }
        feed_imu(&mut slam, &mut imu_t, t + PERIOD_S, &rate_at);
        out.push((
            t,
            slam.feed_scan(&revolution(&world, &pose_at, t, C1, &mut rng)),
        ));
    }
    for (t, p) in out.iter().filter(|(t, _)| *t >= 2.65) {
        let e = err_deg(pose_at, *t, p);
        assert!(e.abs() < 1.5, "t {t:.1}: heading err {e:.2} deg");
    }
    let flagged: Vec<f64> = out
        .iter()
        .filter(|(_, p)| !p.heading_ok)
        .map(|(t, _)| *t)
        .collect();
    assert!(
        flagged.iter().any(|t| (2.05..2.5).contains(t)),
        "no scan flagged after the gap: {flagged:?}"
    );
    assert!(
        flagged.iter().all(|t| (2.05..3.75).contains(t)),
        "flagged outside the gap and the jump: {flagged:?}"
    );
}

/// The IMU drops out from 2.0 s to 2.3 s with no turn: the scan it has not
/// reached and the two whose turn the next sample bridges with a guess are
/// unchecked; the rest are checked.
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
    assert_eq!(flagged, vec![21, 22, 23], "only the scans across the gap");
}
