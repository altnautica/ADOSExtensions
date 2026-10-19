use super::replay::ReplayPose;
use super::*;

fn rz(a: f64) -> [f64; 9] {
    let (s, c) = a.sin_cos();
    [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
}

fn est(t_ns: u64, yaw: f64, x: f64) -> PoseEstimate {
    PoseEstimate {
        t_ns,
        r: rz(yaw),
        t: [x, 0.0, 0.0],
        cov: diagonal_cov(0.1, 0.01),
        source: PoseSource::Vio,
        health: VioHealth::Good,
    }
}

#[test]
fn interpolation_is_slerp_plus_linear() {
    let p = ReplayPose::new(vec![
        est(1_000_000_000, 0.0, 0.0),
        est(1_100_000_000, 1.0, 2.0),
    ]);
    let mid = p.pose_at(1_025_000_000).unwrap();
    assert!((mid.t[0] - 0.5).abs() < 1e-12);
    assert!(se3::rotation_angle_between(&mid.r, &rz(0.25)) < 1e-12);
    assert_eq!(mid.t_ns, 1_025_000_000);
    assert_eq!(mid.source, PoseSource::Replay);
    // Exact sample times return the samples.
    let a = p.pose_at(1_100_000_000).unwrap();
    assert!((a.t[0] - 2.0).abs() < 1e-12);
}

#[test]
fn extrapolation_beyond_50_ms_is_refused() {
    let p = ReplayPose::new(vec![
        est(1_000_000_000, 0.0, 0.0),
        est(1_100_000_000, 0.0, 1.0),
    ]);
    // Within 50 ms of either end: the end estimate, held, never extrapolated.
    let late = p.pose_at(1_150_000_000).unwrap();
    assert_eq!(late.t, [1.0, 0.0, 0.0]);
    assert_eq!(p.pose_at(950_000_000).unwrap().t, [0.0, 0.0, 0.0]);
    assert!(p.pose_at(1_150_000_001).is_none());
    assert!(p.pose_at(949_999_999).is_none());
    assert!(ReplayPose::new(vec![]).pose_at(0).is_none());
}

#[test]
fn health_and_covariance_take_the_worse_side() {
    let mut b = est(2_000, 0.0, 1.0);
    b.health = VioHealth::Degraded;
    b.cov = diagonal_cov(0.5, 0.01);
    let p = ReplayPose::new(vec![est(1_000, 0.0, 0.0), b]);
    let m = p.pose_at(1_500).unwrap();
    assert_eq!(m.health, VioHealth::Degraded);
    assert!((m.cov[0] - 0.25).abs() < 1e-12);
}

#[test]
fn the_track_drains_each_estimate_once_and_trims_history() {
    let t = PoseTrack::new();
    t.push(est(1, 0.0, 0.0));
    t.push(est(2, 0.0, 0.0));
    // Out of order: dropped.
    t.push(est(2, 0.0, 9.0));
    let mut out = Vec::new();
    t.drain_new(&mut out);
    assert_eq!(out.len(), 2);
    t.drain_new(&mut out);
    assert_eq!(out.len(), 2);
    t.push(est(3 + TRACK_HISTORY_NS, 0.0, 0.0));
    t.drain_new(&mut out);
    assert_eq!(out.len(), 3);
    assert!(t.pose_at(2).is_none(), "history older than 30 s is gone");
    t.set_health(VioHealth::Lost);
    assert_eq!(t.health(), VioHealth::Lost);
    t.reset();
    assert!(t.latest().is_none());
}
