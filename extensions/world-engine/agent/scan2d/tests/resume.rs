//! Recovery when the IMU's yaw rate comes back after the drone yawed
//! without it: the samples of the stretch dropped, a stand-in that carried
//! no yaw rate, or a delivery stall that bursts. A heading a gap left
//! unmeasured is never reported checked off the drone's: the scans
//! re-confirm it against the map they saw before (near the flight
//! controller's own turn across a short gap), or the map starts over; after
//! a stand-in the map starts over.

mod common;

use ados_scan2d::pose::wrap;
use ados_scan2d::{ImuSample, LocalSlam, Pose2, Scan2dConfig};
use common::{rect, revolution, Rng, Seg, C1};

/// A 12 x 6 m room with two boxes: no turn maps its outline onto itself.
fn room() -> Vec<Seg> {
    let mut w = rect(0.0, 0.0, 12.0, 6.0);
    w.extend(rect(2.0, 1.0, 7.0, 1.6));
    w.extend(rect(8.5, 3.5, 9.5, 5.2));
    w
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Feed {
    /// 100 Hz IMU, the samples in [stop, resume) never delivered.
    Dropped,
    /// 20 Hz zero-yaw-rate stand-in until `resume`, then 100 Hz IMU.
    StandIn,
    /// 100 Hz IMU whose delivery stops at `stop` and bursts at `resume`.
    Burst,
}

struct Run {
    /// Scans after `resume` reported checked more than 1.5 degrees off,
    /// before any restart.
    checked_off: Vec<(f64, f64)>,
    /// The first scan after `resume` reported checked, before any restart.
    first_ok: Option<f64>,
    /// The heading error of the last scan, degrees.
    final_err: f64,
    /// The scan that started the map over ([`PoseOut::map_restarted`]),
    /// its heading error then, and the scans after it whose error moved
    /// from that by more than 1.5 degrees (the caller re-anchors the new
    /// map once).
    restart: Option<(f64, f64)>,
    restart_drift: Vec<(f64, f64)>,
}

/// The drone yaws `deg` at `rate_dps` from 2 s on, in place at (4, 3) of
/// `world`, while the IMU's yaw rate is missing over [stop, resume). With
/// `fc_heading` the flight controller's heading rides each delivered
/// sample, as the capture feeds it (its attitude streams on through a
/// `RAW_IMU` gap), off the truth by that many degrees from `resume` on.
fn run(
    world: &[Seg],
    deg: f64,
    rate_dps: f64,
    feed: Feed,
    stop: f64,
    resume: f64,
    fc_heading: Option<f64>,
) -> Run {
    let (t0, t1) = (2.0, 2.0 + deg / rate_dps);
    let rate = rate_dps.to_radians();
    let yaw = move |t: f64| rate * (t.clamp(t0, t1) - t0);
    let pose_at = move |t: f64| Pose2::new(4.0, 3.0, 0.4 + yaw(t));
    let mut rng = Rng::new(3);
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu_t = 0.0f64;
    let mut out = Run {
        checked_off: Vec::new(),
        first_ok: None,
        final_err: 0.0,
        restart: None,
        restart_drift: Vec::new(),
    };
    for n in 0..200 {
        let t = n as f64 * 0.1;
        loop {
            let stand_in = feed == Feed::StandIn && imu_t < resume;
            let dt = if stand_in { 0.05 } else { 0.01 };
            let next = imu_t + dt;
            if next > t + 0.1 + 1e-9 {
                break;
            }
            if feed == Feed::Burst && next >= stop && t < resume {
                break;
            }
            if feed == Feed::StandIn && stand_in && next >= resume {
                // The first sample that measures the turn again.
                slam.heading_unmeasured();
            }
            imu_t = next;
            if feed == Feed::Dropped && (stop..resume).contains(&imu_t) {
                continue;
            }
            let t_ns = (imu_t * 1e9).round() as u64;
            if let Some(bias_deg) = fc_heading {
                let bias = if imu_t >= resume { bias_deg.to_radians() } else { 0.0 };
                slam.feed_fc_heading(t_ns, 1.0 + yaw(imu_t) + bias);
            }
            let wz = if stand_in || !(t0..t1).contains(&imu_t) { 0.0 } else { rate };
            slam.feed_imu(&ImuSample {
                t_ns,
                gyro: [0.0, 0.0, wz],
                accel: [0.0, 0.0, 9.81],
            });
        }
        let p = slam.feed_scan(&revolution(world, &pose_at, t, C1, &mut rng));
        let e = wrap(p.yaw - yaw(t)).to_degrees();
        out.final_err = e;
        if p.map_restarted {
            out.restart.get_or_insert((t, e));
        }
        match out.restart {
            Some((_, e0)) => {
                if wrap((e - e0).to_radians()).to_degrees().abs() > 1.5 {
                    out.restart_drift.push((t, e));
                }
            }
            None if t >= resume && p.heading_ok => {
                out.first_ok.get_or_insert(t);
                if e.abs() > 1.5 {
                    out.checked_off.push((t, e));
                }
            }
            None => {}
        }
        if p.heading_ok {
            assert!(!p.heading_unconfirmed, "t {t:.1}: checked yet unconfirmed");
        }
    }
    out
}

fn assert_recovers(r: &Run, what: &str) {
    assert!(r.checked_off.is_empty(), "{what}: reported checked off the drone at {:?}", r.checked_off);
    assert_eq!(r.restart, None, "{what}: started the map over");
    assert!(r.first_ok.is_some(), "{what}: never re-confirmed");
    assert!(r.final_err.abs() < 1.5, "{what}: ends {:.2} deg off", r.final_err);
}

#[test]
fn a_yaw_whose_samples_dropped_is_re_confirmed_against_the_map() {
    let w = room();
    for (deg, rate) in [(30.0, 15.0), (90.0, 45.0)] {
        let r = run(&w, deg, rate, Feed::Dropped, 1.0, 5.0, None);
        assert_recovers(&r, &format!("dropped {deg} deg"));
    }
}

/// A short gap inside a fast turn: the flight controller's own heading
/// change across it is the turn it hid, so the scans confirm near it
/// through the turn, the gap counted once, rather than waiting it out with
/// a search of every turn.
#[test]
fn a_short_gap_in_a_fast_turn_is_re_confirmed_on_the_flight_controllers_turn() {
    let w = room();
    for (deg, rate) in [(90.0, 60.0), (150.0, 80.0)] {
        let r = run(&w, deg, rate, Feed::Dropped, 2.5, 3.0, Some(0.0));
        let what = format!("short gap {deg} deg");
        assert_recovers(&r, &what);
        let first = r.first_ok.expect("checked");
        assert!(first <= 3.6, "{what}: re-confirmed only at {first:.1} s");
    }
}

/// The flight controller's heading stepping across a short gap in a fast
/// turn (an EKF lane switch or yaw reset) puts the turn it hid outside the
/// window searched around it: the best heading inside is on the window's
/// edge, and it is not confirmed there. The map starts over instead.
#[test]
fn a_turn_the_flight_controller_misreports_is_not_confirmed_on_the_window_edge() {
    let w = room();
    for bias in [10.0, -10.0] {
        let r = run(&w, 90.0, 90.0, Feed::Dropped, 2.3, 2.8, Some(bias));
        let what = format!("FC turn {bias:+} deg off");
        assert!(r.checked_off.is_empty(), "{what}: confirmed off the drone at {:?}", r.checked_off);
        assert_eq!(r.first_ok, None, "{what}: confirmed before the map started over");
        assert!(r.restart.is_some(), "{what}: never started the map over");
    }
}

#[test]
fn a_yaw_through_a_delivery_stall_recovers_exactly() {
    let w = room();
    for (deg, rate) in [(30.0, 15.0), (90.0, 45.0)] {
        let r = run(&w, deg, rate, Feed::Burst, 1.0, 5.0, None);
        assert_recovers(&r, &format!("burst {deg} deg"));
        assert_eq!(r.first_ok, Some(5.0), "burst {deg} deg: the burst measured the turn");
    }
}

/// The stand-in pinned the heading through the turn while its scans built
/// the map, so that map cannot re-confirm the heading: the matcher starts a
/// new one where the stand-in ends, and tracks from there. The new map is
/// off the old one by the turn the stand-in hid, steadily (the caller
/// re-anchors it to the world).
#[test]
fn a_stand_in_without_yaw_rate_starts_a_new_map() {
    let w = room();
    for (deg, rate) in [(30.0, 15.0), (90.0, 30.0)] {
        let r = run(&w, deg, rate, Feed::StandIn, 0.0, 6.0, None);
        let what = format!("stand-in {deg} deg");
        assert_eq!(r.first_ok, Some(6.0), "{what}: the new map tracks at once");
        let off: Vec<_> = r.checked_off.iter().filter(|(_, e)| (e + deg).abs() > 1.5).collect();
        assert!(off.is_empty(), "{what}: the new map moved against the old at {off:?}");
        assert!((r.final_err + deg).abs() < 1.5, "{what}: ends {:.2} deg off", r.final_err);
    }
}

/// In the middle of a bare square a quarter turn maps the outline onto
/// itself: after a stretch long enough to hide a quarter turn the heading
/// cannot be told from the others. It is never confirmed; once the bounds
/// pass the map starts over, and the new one tracks steadily from there.
#[test]
fn an_ambiguous_room_starts_the_map_over() {
    let r = run(&rect(1.0, 0.0, 7.0, 6.0), 0.0, 15.0, Feed::Dropped, 1.0, 5.0, None);
    assert_eq!(r.first_ok, None, "re-confirmed a heading the room leaves ambiguous");
    let (at, _) = r.restart.expect("the map starts over");
    assert!(at <= 7.6, "started over only at {at:.1} s");
    assert!(r.restart_drift.is_empty(), "the new map moved at {:?}", r.restart_drift);
}
