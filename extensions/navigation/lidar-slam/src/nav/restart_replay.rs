//! A recorded warehouse flight replayed through the shipped path: the
//! planar provider ([`PlanarLidarPose`]) fed the flight's scans, IMU and
//! the flight controller's heading and position as the capture feeds it,
//! an [`FcEkfPose`] fed the FC's attitude (with its heading rate) and
//! `LOCAL_POSITION_NED`, and each drained batch handed to the navigation
//! map the way the capture's tick does ([`NavMap::on_matcher_poses`], then
//! [`NavMap::on_pose`] with the newest pose). With a hole in the IMU inside
//! a turn, the published `map_ned` pose is scored against the same flight
//! replayed without the hole, in the map's own north from before the gap.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use ados_capture_contract::v1::{ImuBatch, ImuSample as BusImu};
use ados_capture_contract::Rig;
use ados_mapping_protocol::capture::ComputePlacement;
use serde_json::json;

use super::*;
use crate::pose::planar::{PlanarLidarConfig, PlanarLidarPose};
use ados_mapping_pose::fc_ekf::test_frames::{att_turning, pos};
use ados_mapping_pose::pose::PoseProvider;

/// Beams per revolution and revolution period of the recorded LiDAR.
const BEAMS: u64 = 500;
const PERIOD_NS: u64 = 100_000_000;

/// The flight: the IMU the capture feeds the matcher (the FC's gyro, and
/// gravity in the attitude the FC last reported as the specific force),
/// the FC's own attitude `(t_ns, FLU yaw)` and each revolution.
struct Flight {
    imu: Vec<BusImu>,
    fc_yaw: Vec<(u64, f64)>,
    scans: Vec<LidarScan>,
}

/// Parse a scan2d `tests/data` flight carrying the FC's IMU (`g`) and
/// attitude (`f`), format in the file's header.
fn flight(text: &str) -> Flight {
    let urad = |v: i64| v as f64 * 1e-6;
    let mut level = Vec::new();
    let mut f = Flight {
        imu: Vec::new(),
        fc_yaw: Vec::new(),
        scans: Vec::new(),
    };
    for l in text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
    {
        let mut it = l.split_whitespace();
        let kind = it.next().expect("a line kind");
        let v: Vec<i64> = it.map(|x| x.parse().expect("an integer")).collect();
        let t_ns = v[0] as u64 * 1_000;
        match kind {
            "g" => f.imu.push(BusImu {
                t_ns,
                gyro: vec![urad(v[1]), urad(v[2]), urad(v[3])],
                accel: vec![v[4] as f64 * 1e-3, v[5] as f64 * 1e-3, v[6] as f64 * 1e-3],
                ..Default::default()
            }),
            "f" => {
                f.fc_yaw.push((t_ns, urad(v[3])));
                level.push((t_ns, urad(v[1]), urad(v[2])));
            }
            "s" => {
                let n = v[2] as usize;
                let mut s = LidarScan {
                    sensor_id: "lidar0".into(),
                    t_ns,
                    ..LidarScan::default()
                };
                let mut k = 0u64;
                for p in v[3..3 + 2 * n].chunks(2) {
                    k += p[0] as u64;
                    let az = k as f64 * 2.0 * PI / BEAMS as f64;
                    let r = p[1] as f64 * 1e-3;
                    s.points
                        .push([(r * az.cos()) as f32, (r * az.sin()) as f32, 0.0]);
                    s.t_offset_ns.push((k * PERIOD_NS / BEAMS) as u32);
                    s.intensity.push(50);
                }
                f.scans.push(s);
            }
            "a" => {}
            other => panic!("unknown line kind `{other}`"),
        }
    }
    for s in &mut f.imu {
        let k = level.partition_point(|(t, _, _)| *t <= s.t_ns);
        if let Some(&(t, roll, pitch)) = k.checked_sub(1).map(|k| &level[k]) {
            if s.t_ns - t <= 500_000_000 {
                let ((sr, cr), (sp, cp)) = (roll.sin_cos(), pitch.sin_cos());
                s.accel = vec![-9.81 * sp, 9.81 * cp * sr, 9.81 * cp * cr];
            }
        }
    }
    f
}

/// The planar LiDAR at the body origin, level, 12 m, nothing blocked.
fn rig() -> Rig {
    serde_json::from_value(json!({
        "contract": "ados.capture/1",
        "rig_id": "planar",
        "calib_version": "1",
        "sensors": [
            { "id": "lidar0", "kind": "lidar", "role": "front_left", "time_offset_ns": 0,
              "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
              "lidar": { "model": "rplidar-c1", "pattern": "planar", "h_fov_deg": 360.0,
                         "v_min_deg": 0.0, "v_max_deg": 0.0, "rate_hz": 10.0,
                         "points_per_s": 5000.0, "min_range_m": 0.05, "max_range_m": 12.0,
                         "range_sigma_m": 0.03, "blocked_azimuth_deg": [] } },
            { "id": "imu0", "kind": "imu", "role": "nav", "time_offset_ns": 0,
              "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1] }
        ]
    }))
    .expect("rig")
}

/// The flight controller's dead reckoning while the pose is lost: its
/// position runs off by this, m/s north and east, from the hole's start.
const FC_DRIFT_MPS: [f64; 2] = [0.08, -0.06];
/// How long after the pose is first good again it is scored.
const SCORED_NS: u64 = 10_000_000_000;

/// One published pose: its time, health and `map_ned` position and heading.
type PublishedPose = (u64, VioHealth, [f64; 2], f64);

/// What one replay published: per scan, the newest pose after that scan,
/// whether its heading was measured, the restarts and where each was
/// placed from.
struct Published {
    poses: Vec<Option<PublishedPose>>,
    heading_ok: Vec<bool>,
    restarts: u32,
    seeds: Vec<Seed>,
}

/// Where a restart was placed from, printed with its case: the newest
/// pose and the scan that brought the restart, the FC's move and the time
/// since the hole began then (against the matcher's 3 m and 5 s restart
/// bounds), the last good and last measured pose (time, heading) the map
/// then held, and the north it took; times in s, angles in degrees.
#[derive(Clone, Copy)]
struct Seed {
    t_s: f64,
    scan_t_s: f64,
    fc_moved_m: f64,
    since_hole_s: f64,
    good: Option<(f64, f64)>,
    measured: Option<(f64, f64)>,
    north_deg: Option<f64>,
}

/// Replay with the IMU samples in `hole` (host ns, open) dropped. The FC
/// streams on: its attitude at the recorded yaw (its own error against the
/// map included), turning at `rate(t)`, and its position `fc_ned(t)`.
fn replay(
    f: &Flight,
    hole: (u64, u64),
    rate: &dyn Fn(u64) -> f64,
    fc_ned: &dyn Fn(u64) -> [f64; 2],
) -> Published {
    let p = PlanarLidarPose::spawn(PlanarLidarConfig {
        rig: rig(),
        lidar_id: "lidar0".into(),
        imu_id: "imu0".into(),
    })
    .expect("spawn");
    let fc = FcEkfPose::new();
    let mut nav = NavMap::new(NavSettings {
        placement: ComputePlacement::Onboard,
        budget: Budget::from_mb(64),
        pose_interval_ms: 33,
        takeoff_heading_rad: None,
    });
    let mut seen = 0u32;
    let mut atts = f.fc_yaw.iter().peekable();
    let mut imu = f
        .imu
        .iter()
        .filter(|s| !(hole.0 < s.t_ns && s.t_ns < hole.1))
        .peekable();
    let mut newest_imu = 0u64;
    let mut out = Vec::new();
    let mut poses = Vec::new();
    let mut heading_ok = Vec::new();
    let mut seeds = Vec::new();
    for (i, scan) in f.scans.iter().enumerate() {
        let until = scan.t_ns + PERIOD_NS;
        while let Some(&(t, yaw)) = atts.next_if(|(t, _)| *t <= until) {
            let ms = (t / 1_000_000) as u32;
            let h = -yaw / 2.0;
            let q = [h.cos() as f32, 0.0, 0.0, h.sin() as f32];
            fc.on_frame(&att_turning(ms, q, rate(t) as f32), t);
            let ne = fc_ned(t);
            fc.on_frame(&pos(ms, [ne[0] as f32, ne[1] as f32, -2.0]), t);
            p.feed_fc_position(t, ne);
        }
        let samples: Vec<BusImu> =
            std::iter::from_fn(|| imu.next_if(|s| s.t_ns <= until).cloned()).collect();
        if let Some(last) = samples.last() {
            newest_imu = last.t_ns;
            // The capture feeds the FC's heading with each `RAW_IMU` sample.
            let k = f.fc_yaw.partition_point(|(t, _)| *t <= last.t_ns).max(1);
            p.feed_fc_heading(last.t_ns, f.fc_yaw[k - 1].1);
            p.feed_imu("imu0", &ImuBatch { samples });
        }
        p.feed_lidar(scan);
        // A scan the IMU does not reach yet waits for it (or the next scan).
        let want = if newest_imu + 10_000_000 >= until {
            i + 1
        } else {
            i
        } as u64;
        let deadline = Instant::now() + Duration::from_secs(60);
        while p.stats().scans_processed < want {
            assert!(Instant::now() < deadline, "scan {i} not matched in 60 s");
            std::thread::sleep(Duration::from_millis(1));
        }
        out.clear();
        p.drain_new(&mut out);
        let q = p.quality();
        let restarted = q.map_restarts > seen;
        nav.on_matcher_poses(&out, q.map_restarts, &mut seen, Some(0.0), Some(&fc));
        if let (true, Some(e)) = (restarted, out.last()) {
            let at = |s: Option<Trusted>| s.map(|(t, _, h)| (t as f64 * 1e-9, h.to_degrees()));
            let (a, b) = (fc_ned(hole.0), fc_ned(scan.t_ns));
            seeds.push(Seed {
                t_s: e.t_ns as f64 * 1e-9,
                scan_t_s: scan.t_ns as f64 * 1e-9,
                fc_moved_m: (b[0] - a[0]).hypot(b[1] - a[1]),
                since_hole_s: scan.t_ns.saturating_sub(hole.0) as f64 * 1e-9,
                good: at(nav.good),
                measured: at(nav.measured),
                north_deg: nav.map_yaw().map(f64::to_degrees),
            });
        }
        let published = out.last().and_then(|e| {
            let heading = fc.heading_at(e.t_ns);
            let _ = nav.on_pose(e, Some(0.0), heading, (e.t_ns / 1_000_000) as i64, 0);
            nav.latest
                .map(|(r, t)| (e.t_ns, e.health, [t[0], t[1]], frd_yaw(&r)))
        });
        poses.push(published);
        heading_ok.push(out.last().is_some_and(|e| e.heading_ok));
    }
    Published {
        poses,
        heading_ok,
        restarts: p.quality().map_restarts,
        seeds,
    }
}

/// How the pose came back after a hole ([`holed`]).
#[derive(Debug)]
struct Scored {
    restarts: u32,
    /// The worst `map_ned` position error against the unholed replay, m,
    /// from the first good pose after the hole for [`SCORED_NS`].
    worst_m: f64,
    /// The worst heading error against the map's own north before the gap,
    /// degrees, over the same span.
    worst_deg: f64,
    /// The flight controller's own drift over the outage, m.
    fc_drift_m: f64,
    /// The FC's heading off the map's north at the first good pose after
    /// the hole, degrees: what a north taken from it would carry.
    fc_north_deg: f64,
    /// When the worst heading error was, s.
    worst_deg_at_s: f64,
    /// When the pose was first good after the hole, host ns.
    good_at_ns: Option<u64>,
}

/// The unholed replay's published poses: time, `map_ned` position and
/// heading.
type Refs = Vec<(u64, [f64; 2], f64)>;

/// Replay `f` without a hole: what [`holed`] scores against.
fn reference(f: &Flight) -> Refs {
    let run = replay(f, (0, 0), &|_| 0.0, &|_| [0.0; 2]);
    run.poses
        .iter()
        .flatten()
        .map(|&(t, _, p, h)| (t, p, h))
        .collect()
}

/// Replay `f` with a hole of `len_ns` from `at_ns` after the first scan,
/// and score it against `refs`, the unholed replay ([`reference`]). The
/// FC's position is the unholed replay's, in the FC's own north (its
/// heading's error against the map turns its dead reckoning), off by
/// [`FC_DRIFT_MPS`] from the hole's start; its heading rate is the unholed
/// replay's turn rate.
fn holed(f: &Flight, refs: &Refs, at_ns: u64, len_ns: u64) -> (Scored, Published) {
    let t0 = f.scans[0].t_ns;
    let hole = (t0 + at_ns, t0 + at_ns + len_ns);
    let k_at = |t: u64| refs.partition_point(|r| r.0 <= t).clamp(1, refs.len() - 1);
    let lerp = |t: u64| {
        let k = k_at(t);
        let (a, b) = (refs[k - 1], refs[k]);
        let w = ((t as f64 - a.0 as f64) / (b.0 - a.0) as f64).clamp(0.0, 1.0);
        let p = [
            a.1[0] + w * (b.1[0] - a.1[0]),
            a.1[1] + w * (b.1[1] - a.1[1]),
        ];
        (p, a.2 + w * wrap_pi(b.2 - a.2))
    };
    let rate = |t: u64| {
        let k = k_at(t);
        let (a, b) = (refs[k - 1], refs[k]);
        wrap_pi(b.2 - a.2) / ((b.0 - a.0) as f64 * 1e-9)
    };
    let fc_heading = |t: u64| {
        let k = f.fc_yaw.partition_point(|(ft, _)| *ft <= t).max(1);
        -f.fc_yaw[k - 1].1
    };
    // The FC's dead reckoning in its own north: the unholed path's steps,
    // each turned by the FC's heading against the map's then.
    let mut fc_path: Vec<(u64, [f64; 2])> = Vec::with_capacity(refs.len());
    for (k, r) in refs.iter().enumerate() {
        let p = match k.checked_sub(1).map(|j| (fc_path[j].1, refs[j].1)) {
            None => r.1,
            Some((prev, from)) => {
                let off = fc_heading(r.0) - r.2;
                let (s, c) = off.sin_cos();
                let (dn, de) = (r.1[0] - from[0], r.1[1] - from[1]);
                [prev[0] + c * dn - s * de, prev[1] + s * dn + c * de]
            }
        };
        fc_path.push((r.0, p));
    }
    let drift = |t: u64| {
        let s = t.saturating_sub(hole.0) as f64 * 1e-9;
        [FC_DRIFT_MPS[0] * s, FC_DRIFT_MPS[1] * s]
    };
    let fc_ned = |t: u64| {
        let k = fc_path
            .partition_point(|r| r.0 <= t)
            .clamp(1, fc_path.len() - 1);
        let (a, b) = (fc_path[k - 1], fc_path[k]);
        let w = ((t as f64 - a.0 as f64) / (b.0 - a.0) as f64).clamp(0.0, 1.0);
        let d = drift(t);
        [
            a.1[0] + w * (b.1[0] - a.1[0]) + d[0],
            a.1[1] + w * (b.1[1] - a.1[1]) + d[1],
        ]
    };
    let run = replay(f, hole, &rate, &fc_ned);
    for s in &run.seeds {
        let deg = |p: Option<(f64, f64)>| {
            p.map_or("-".into(), |(t, h)| format!("{t:.3} s at {h:.3} deg"))
        };
        eprintln!(
            "  restart at {:.3} s (scan {:.3} s): FC moved {:.2} m in {:.2} s since the hole; last good {}, last measured {}; north {}",
            s.t_s,
            s.scan_t_s,
            s.fc_moved_m,
            s.since_hole_s,
            deg(s.good),
            deg(s.measured),
            s.north_deg.map_or("-".into(), |n| format!("{n:.3} deg")),
        );
    }
    // Each published pose from 0.5 s before the hole until 2 s after it is
    // first good again: time, health, measured heading, error against the
    // unholed replay's heading (degrees).
    let mut first_good = None;
    for (pose, ok) in run.poses.iter().zip(&run.heading_ok) {
        let Some(&(t, health, _, h)) = pose.as_ref() else {
            continue;
        };
        if t + 500_000_000 < hole.0 {
            continue;
        }
        if t > hole.1 && health != VioHealth::Lost {
            first_good.get_or_insert(t);
        }
        if first_good.is_some_and(|g| t > g + 2_000_000_000) {
            break;
        }
        let err = wrap_pi(h - lerp(t).1).to_degrees();
        eprintln!(
            "  trace t {:.3} s {health:?} heading_ok {ok} err {err:+.3} deg",
            t as f64 * 1e-9
        );
    }
    let mut s = Scored {
        restarts: run.restarts,
        worst_m: 0.0,
        worst_deg: 0.0,
        fc_drift_m: 0.0,
        fc_north_deg: 0.0,
        worst_deg_at_s: 0.0,
        good_at_ns: None,
    };
    let mut good_at = None;
    for &(t, health, p, h) in run.poses.iter().flatten() {
        if t <= hole.1 || health == VioHealth::Lost {
            continue;
        }
        let at = *good_at.get_or_insert_with(|| {
            let d = drift(t);
            s.fc_drift_m = d[0].hypot(d[1]);
            s.fc_north_deg = wrap_pi(fc_heading(t) - lerp(t).1).to_degrees();
            t
        });
        if t > at + SCORED_NS {
            break;
        }
        let (want, heading) = lerp(t);
        s.worst_m = s.worst_m.max((p[0] - want[0]).hypot(p[1] - want[1]));
        let err = wrap_pi(h - heading).to_degrees().abs();
        if err > s.worst_deg {
            s.worst_deg = err;
            s.worst_deg_at_s = t as f64 * 1e-9;
        }
    }
    s.good_at_ns = good_at;
    (s, run)
}

/// `f` with every LiDAR point of the scans from `from_ns` to `to_ns` moved
/// one ulp away from zero in x.
fn nudged(f: &Flight, from_ns: u64, to_ns: u64) -> Flight {
    let scans = f.scans.iter().map(|s| {
        let mut s = s.clone();
        if (from_ns..to_ns).contains(&s.t_ns) {
            s.points
                .iter_mut()
                .for_each(|p| p[0] = f32::from_bits(p[0].to_bits() + 1));
        }
        s
    });
    Flight {
        imu: f.imu.clone(),
        fc_yaw: f.fc_yaw.clone(),
        scans: scans.collect(),
    }
}

/// How far apart two replays of one flight publish over the span [`holed`]
/// scores from `from_ns`: the largest heading difference, degrees, and
/// position difference, m, and the poses compared.
fn apart(a: &Published, b: &Published, from_ns: u64) -> (f64, f64, usize) {
    let mut worst = (0.0f64, 0.0f64, 0);
    for (pa, pb) in a.poses.iter().zip(&b.poses) {
        let (Some((ta, _, xa, ha)), Some((tb, _, xb, hb))) = (pa, pb) else {
            continue;
        };
        if *ta < from_ns || *ta > from_ns + SCORED_NS {
            continue;
        }
        assert_eq!(ta, tb, "the replays published different scans");
        let deg = wrap_pi(ha - hb).to_degrees().abs();
        let m = (xa[0] - xb[0]).hypot(xa[1] - xb[1]);
        worst = (worst.0.max(deg), worst.1.max(m), worst.2 + 1);
    }
    worst
}

/// Holes in the warehouse's turns: 1.5 s at 22 and 30 s and 5 s at 30 s,
/// where the drone flies on 3 to 10 m and the map starts over, and 0.3 s at
/// 22 and 30 s, re-confirmed. For 10 s from the first good pose after the
/// hole, the published pose is within the FC's own drift plus [`SLACK_M`]
/// of the unholed replay's, and its heading within 2° of it, both in the
/// map's north from before the gap: there the FC's own heading is degrees
/// off that north (`fc_north_deg`), and a restarted map does not take it.
///
/// The 22 s, 1.5 s hole is flown again with every LiDAR point through the
/// hole moved by one ulp. Matched on a stale prior, the scans through the
/// hole grow a last-bit difference to centimetres (aarch64's `hypot`,
/// which rounds a last bit differently from x86's, did), and the map
/// started over after it must not take that on: both replays publish
/// within [`NUDGE_DEG`] and [`NUDGE_M`] of each other over the scored span.
#[test]
fn a_map_restarted_in_flight_is_published_where_the_drone_is() {
    let f = flight(include_str!(
        "../../../../../crates/scan2d/tests/data/warehouse_return_leg.txt"
    ));
    let refs = reference(&f);
    let (s, ms) = (1_000_000_000u64, 1_000_000u64);
    let mut scored: Vec<_> = [
        (22 * s, 300 * ms, false),
        (30 * s, 300 * ms, false),
        (22 * s, 1_500 * ms, true),
        (30 * s, 1_500 * ms, true),
        (30 * s, 5_000 * ms, true),
    ]
    .into_iter()
    .map(|(at, len, restarts)| {
        let (r, run) = holed(&f, &refs, at, len);
        let name = format!("{} s, {} s hole", at / s, len as f64 * 1e-9);
        eprintln!("{name}: {r:?}");
        (name, r, restarts, run)
    })
    .collect();
    let (at, len) = (22 * s, 1_500 * ms);
    let t0 = f.scans[0].t_ns;
    let (r, run) = holed(&nudged(&f, t0 + at, t0 + at + len), &refs, at, len);
    let name = "22 s, 1.5 s hole, nudged".to_string();
    eprintln!("{name}: {r:?}");
    let (_, plain, _, plain_run) = &scored[2];
    let from = plain
        .good_at_ns
        .expect("the 22 s, 1.5 s hole's pose is good again");
    let (deg, m, n) = apart(plain_run, &run, from);
    eprintln!("{name}: {deg:.6} deg and {m:.6} m apart from the unnudged replay over {n} poses");
    scored.push((name, r, true, run));
    // Every case is scored before any is asserted, so a failure shows them all.
    for (name, r, restarts, _) in &scored {
        assert_eq!(r.restarts > 0, *restarts, "{name}: restarts {}", r.restarts);
        let bound = r.fc_drift_m + SLACK_M;
        assert!(
            r.worst_m <= bound,
            "{name}: {:.2} m off (bound {bound:.2} m)",
            r.worst_m
        );
        assert!(
            r.worst_deg <= 2.0,
            "{name}: {:.2} deg off the map's north",
            r.worst_deg
        );
    }
    assert!(n > 0, "no poses compared");
    assert!(
        deg <= NUDGE_DEG && m <= NUDGE_M,
        "a one-ulp nudge moved the restarted map {deg:.3} deg, {m:.3} m"
    );
}

/// The position slack over the FC's drift: the matcher's own error.
const SLACK_M: f64 = 1.0;
/// How far a one-ulp nudge to the scans through the hole may move the map
/// published after it ([`a_map_restarted_in_flight_is_published_where_the_drone_is`]).
const NUDGE_DEG: f64 = 0.01;
const NUDGE_M: f64 = 0.01;
