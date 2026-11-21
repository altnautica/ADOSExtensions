use super::*;

const MS: i64 = 1_000_000;

/// Replay a recorded flight (`f t cx cy q gd`, `v t vn ve`, `y t yaw`; t in
/// sim milliseconds) through the monitor: the first time it disagrees and
/// goes lost, seconds.
fn replay(rec: &str) -> (Option<f64>, Option<f64>) {
    let mut m = FlowEkfMonitor::default();
    let (mut disagree, mut lost) = (None, None);
    for line in rec.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        let n = |i: usize| f[i].parse::<f32>().unwrap();
        let t = f[1].parse::<i64>().unwrap() * MS;
        match f[0] {
            "v" => m.on_velocity(n(2), n(3), t),
            "y" => m.on_yaw(n(2), t),
            "f" => {
                m.on_flow([n(2), n(3)], f[4].parse().unwrap(), n(5), t);
                let s = t as f64 * 1e-9;
                match m.state() {
                    FlowEkf::Disagree => {
                        disagree.get_or_insert(s);
                    }
                    FlowEkf::Lost => {
                        disagree.get_or_insert(s);
                        lost.get_or_insert(s);
                    }
                    FlowEkf::Agree => {}
                }
            }
            _ => unreachable!("{line}"),
        }
    }
    (disagree, lost)
}

/// Mine drift and shaft over dusty floor (flow quality 18-68), with the
/// external-nav pose frozen along the tunnel: the EKF followed it while the
/// vehicle reached 5-6 m/s. Recorded EKF error at the alarm: 3.9 m (drift,
/// 65.8 s) and 4.3 m (shaft, 39.2 s); 45-50 m by the end without it.
#[test]
fn fc_flow_and_the_ekf_velocity_disagreeing_degrades_then_loses_the_pose() {
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_drift.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((65.0..66.5).contains(&d), "drift disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "drift lost at {l}");
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_shaft.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((38.5..40.0).contains(&d), "shaft disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "shaft lost at {l}");
}

/// A goto down a mine drift over a dusty floor (flow at quality 64 and
/// below in flight, some at 0), a planar LiDAR pose frozen along the drift from
/// about 30 s while the vehicle flew on at 1 to 2 m/s: the EKF followed
/// the frozen pose. The monitor parts the two within 3 s of the freeze and
/// loses the pose 3 s later.
#[test]
fn a_drift_route_on_a_frozen_pose_degrades_then_loses_it() {
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_drift_route.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((32.0..34.0).contains(&d), "disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "lost at {l}");
}

/// A whole warehouse ext-nav flight and a whole wall-inspection flight,
/// the EKF tracking: never an alarm (their largest 3 s gap is 0.37 m).
#[test]
fn a_tracking_flight_raises_no_flow_disagreement() {
    for rec in [
        include_str!("../tests/data/flow_ekf_warehouse.txt"),
        include_str!("../tests/data/flow_ekf_wall.txt"),
    ] {
        assert_eq!(replay(rec), (None, None));
    }
}

fn steady(m: &mut FlowEkfMonitor, from_s: f64, to_s: f64, flow_fwd: f32, ekf_n: f32, q: u8) {
    let mut t = (from_s * 1e9) as i64;
    while t < (to_s * 1e9) as i64 {
        m.on_yaw(0.0, t);
        m.on_velocity(ekf_n, 0.0, t);
        // Forward (north at yaw 0) at `flow_fwd` over 2 m of ground.
        m.on_flow([0.0, flow_fwd / 2.0], q, 2.0, t);
        t += 100 * MS;
    }
}

#[test]
fn a_disagreement_clears_only_after_agreeing_for_a_while() {
    let mut m = FlowEkfMonitor::default();
    steady(&mut m, 0.0, 4.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Agree);
    // The EKF stalls at 0.5 m/s while flow reads 2 m/s: 1.5 m/s apart, so
    // past 1.5 m one second into a full window.
    steady(&mut m, 4.0, 6.0, 2.0, 0.5, 40);
    assert_eq!(m.state(), FlowEkf::Disagree);
    steady(&mut m, 6.0, 9.5, 2.0, 0.5, 40);
    assert_eq!(m.state(), FlowEkf::Lost);
    // Agreeing again: the window empties of the stall, then it holds 3 s.
    steady(&mut m, 9.5, 14.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Lost, "not yet clear");
    steady(&mut m, 14.0, 16.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Agree);
}

#[test]
fn flow_of_quality_zero_or_without_ground_is_not_used() {
    let mut m = FlowEkfMonitor::default();
    steady(&mut m, 0.0, 8.0, 3.0, 0.0, 0);
    assert_eq!(m.state(), FlowEkf::Agree);
    let mut t = 0;
    while t < 8_000 * MS {
        m.on_yaw(0.0, t);
        m.on_velocity(0.0, 0.0, t);
        m.on_flow([0.0, 1.5], 200, 0.0, t);
        t += 100 * MS;
    }
    assert_eq!(m.state(), FlowEkf::Agree);
}

/// The one reason the tick reports: the monitor first, then the publisher's
/// estimator, then its FC IMU feed. A pose degraded by `RAW_IMU` under
/// 40 Hz says so (it said nothing before), and every key is one the tick
/// clears again.
#[test]
fn the_reason_falls_back_to_the_imu_feed() {
    let corridor = Some(PoseReason::CorridorUnmeasured);
    let starved = Some(ImuFeed::Starved);
    assert_eq!(
        health_reason(FlowEkf::Agree, None, starved),
        Some("imu_feed_starved")
    );
    assert_eq!(
        health_reason(FlowEkf::Agree, None, Some(ImuFeed::HeadingUnconfirmed)),
        Some("heading_unconfirmed")
    );
    assert_eq!(
        health_reason(FlowEkf::Agree, corridor, starved),
        Some("corridor_unmeasured")
    );
    assert_eq!(
        health_reason(FlowEkf::Disagree, corridor, starved),
        Some(FlowEkf::REASON)
    );
    assert_eq!(health_reason(FlowEkf::Agree, None, None), None);
    for feed in [
        ImuFeed::AttitudeStandIn,
        ImuFeed::Stalled,
        ImuFeed::Starved,
        ImuFeed::HeadingUnconfirmed,
        ImuFeed::NorthUnknown,
    ] {
        assert!(NAV_REASONS.contains(&feed.reason_key()), "{feed:?}");
    }
}
