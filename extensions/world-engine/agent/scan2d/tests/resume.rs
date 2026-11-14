//! Recovery when the IMU's yaw rate comes back after the drone yawed
//! without it: the samples of the stretch dropped, a stand-in that carried
//! no yaw rate, or a delivery stall that bursts. A heading a gap left
//! unmeasured is never reported checked off the drone's, and the scans
//! re-confirm it against the map they saw before; after a stand-in the map
//! starts over.

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
    /// Scans after `resume` reported checked more than 1.5 degrees off.
    checked_off: Vec<(f64, f64)>,
    /// The first scan after `resume` reported checked.
    first_ok: Option<f64>,
    /// The heading error of the last scan, degrees.
    final_err: f64,
}

/// The drone yaws `deg` at `rate_dps` from 2 s on, in place at (4, 3) of
/// `world`, while the IMU's yaw rate is missing over [stop, resume).
fn run(world: &[Seg], deg: f64, rate_dps: f64, feed: Feed, stop: f64, resume: f64) -> Run {
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
            let wz = if stand_in || !(t0..t1).contains(&imu_t) { 0.0 } else { rate };
            slam.feed_imu(&ImuSample {
                t_ns: (imu_t * 1e9).round() as u64,
                gyro: [0.0, 0.0, wz],
                accel: [0.0, 0.0, 9.81],
            });
        }
        let p = slam.feed_scan(&revolution(world, &pose_at, t, C1, &mut rng));
        let e = wrap(p.yaw - yaw(t)).to_degrees();
        out.final_err = e;
        if t >= resume && p.heading_ok {
            out.first_ok.get_or_insert(t);
            if e.abs() > 1.5 {
                out.checked_off.push((t, e));
            }
        }
        if p.heading_ok {
            assert!(!p.heading_unconfirmed, "t {t:.1}: checked yet unconfirmed");
        }
    }
    out
}

fn assert_recovers(r: &Run, what: &str) {
    assert!(r.checked_off.is_empty(), "{what}: reported checked off the drone at {:?}", r.checked_off);
    assert!(r.first_ok.is_some(), "{what}: never re-confirmed");
    assert!(r.final_err.abs() < 1.5, "{what}: ends {:.2} deg off", r.final_err);
}

#[test]
fn a_yaw_whose_samples_dropped_is_re_confirmed_against_the_map() {
    let w = room();
    for (deg, rate) in [(30.0, 15.0), (90.0, 45.0)] {
        let r = run(&w, deg, rate, Feed::Dropped, 1.0, 5.0);
        assert_recovers(&r, &format!("dropped {deg} deg"));
    }
}

#[test]
fn a_yaw_through_a_delivery_stall_recovers_exactly() {
    let w = room();
    for (deg, rate) in [(30.0, 15.0), (90.0, 45.0)] {
        let r = run(&w, deg, rate, Feed::Burst, 1.0, 5.0);
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
        let r = run(&w, deg, rate, Feed::StandIn, 0.0, 6.0);
        let what = format!("stand-in {deg} deg");
        assert_eq!(r.first_ok, Some(6.0), "{what}: the new map tracks at once");
        let off: Vec<_> = r.checked_off.iter().filter(|(_, e)| (e + deg).abs() > 1.5).collect();
        assert!(off.is_empty(), "{what}: the new map moved against the old at {off:?}");
        assert!((r.final_err + deg).abs() < 1.5, "{what}: ends {:.2} deg off", r.final_err);
    }
}

/// In the middle of a bare square a quarter turn maps the outline onto
/// itself: after a stretch long enough to hide a quarter turn the heading
/// cannot be told from the others, and it stays unconfirmed.
#[test]
fn an_ambiguous_room_leaves_the_heading_unconfirmed() {
    let r = run(&rect(1.0, 0.0, 7.0, 6.0), 0.0, 15.0, Feed::Dropped, 1.0, 5.0);
    assert_eq!(r.first_ok, None, "re-confirmed a heading the room leaves ambiguous");
}
