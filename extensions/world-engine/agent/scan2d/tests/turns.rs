//! Heading through turns, on revolutions recorded from simulated flights:
//! the matched yaw must follow the true yaw of the LiDAR, not drift with
//! the turns or lag a fast yaw slew.
//!
//! Each recording carries either the LiDAR's true attitude at the rate a
//! flight controller reports it (20 Hz), turned into an IMU stream the way
//! the capture service does when the FC streams no IMU (body rate from
//! consecutive attitudes, gravity as the specific force), or the FC's own
//! IMU (`RAW_IMU`) together with the attitude the FC itself reported.

use std::f64::consts::PI;

use ados_scan2d::{ImuSample, LocalSlam, Scan2dConfig, ScanInput};
use nalgebra::{Rotation3, Vector3};

/// Beams per revolution and revolution period of the recorded LiDAR.
const BEAMS: u64 = 500;
const PERIOD_NS: u64 = 100_000_000;

/// One recording: the matcher's IMU stream, the stream synthesised from the
/// flight controller's own attitude (empty when not recorded) and that
/// attitude, the heading the flight controller reports (its own attitude,
/// else the true one at its rate), and `(revolution, true yaw)` in time
/// order.
struct Recording {
    imu: Vec<ImuSample>,
    fc_attitude_imu: Vec<ImuSample>,
    fc_attitudes: Vec<(u64, Rotation3<f64>)>,
    fc_headings: Vec<(u64, f64)>,
    scans: Vec<(ScanInput, f64)>,
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

/// Parse a `tests/data` flight (format in each file's header).
fn recorded(text: &str) -> Recording {
    let urad = |v: i64| v as f64 * 1e-6;
    let mut atts: Vec<(u64, Rotation3<f64>)> = Vec::new();
    let mut fc_atts: Vec<(u64, Rotation3<f64>)> = Vec::new();
    let mut gyro = Vec::new();
    let mut scans = Vec::new();
    for l in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let mut f = l.split_whitespace();
        let kind = f.next().expect("a line kind");
        let v: Vec<i64> = f.map(|x| x.parse().expect("an integer")).collect();
        let t_ns = v[0] as u64 * 1_000;
        match kind {
            "a" => atts.push((t_ns, Rotation3::from_euler_angles(urad(v[1]), urad(v[2]), urad(v[3])))),
            "f" => fc_atts.push((t_ns, Rotation3::from_euler_angles(urad(v[1]), urad(v[2]), urad(v[3])))),
            "g" => gyro.push(ImuSample {
                t_ns,
                gyro: [urad(v[1]), urad(v[2]), urad(v[3])],
                accel: [v[4] as f64 * 1e-3, v[5] as f64 * 1e-3, v[6] as f64 * 1e-3],
            }),
            "s" => {
                let n = v[2] as usize;
                assert_eq!(v.len(), 3 + 2 * n, "return count");
                let (mut k, mut points, mut t_offset_ns) = (0u64, Vec::new(), Vec::new());
                for p in v[3..].chunks(2) {
                    k += p[0] as u64;
                    let az = k as f64 * 2.0 * PI / BEAMS as f64;
                    let r = p[1] as f64 * 1e-3;
                    points.push([(r * az.cos()) as f32, (r * az.sin()) as f32, 0.0]);
                    t_offset_ns.push((k * PERIOD_NS / BEAMS) as u32);
                }
                let scan = ScanInput {
                    t_ns,
                    points,
                    t_offset_ns,
                };
                scans.push((scan, urad(v[1])));
            }
            other => panic!("unknown line kind `{other}`"),
        }
    }
    let imu = if gyro.is_empty() { from_attitudes(&atts) } else { gyro };
    let heading_src = if fc_atts.is_empty() { &atts } else { &fc_atts };
    Recording {
        imu,
        fc_attitude_imu: from_attitudes(&fc_atts),
        fc_headings: heading_src.iter().map(|(t, r)| (*t, r.euler_angles().2)).collect(),
        fc_attitudes: fc_atts,
        scans,
    }
}

/// IMU samples from consecutive attitudes, as the capture service makes them.
fn from_attitudes(atts: &[(u64, Rotation3<f64>)]) -> Vec<ImuSample> {
    atts.windows(2)
        .map(|w| {
            let ((t0, r0), (t1, r1)) = (w[0], w[1]);
            let dt = (t1 - t0) as f64 * 1e-9;
            let gyro = (r0.inverse() * r1).scaled_axis() / dt;
            let accel = r1.inverse() * Vector3::new(0.0, 0.0, 9.81);
            ImuSample {
                t_ns: t1,
                gyro: gyro.into(),
                accel: accel.into(),
            }
        })
        .collect()
}

/// Replay as the capture service feeds the matcher (each revolution after
/// the IMU up to its last return) and return the heading error of every
/// scan against the truth, relative to the first scan, radians, and the
/// matcher.
fn replay(rec: &Recording, imu: &[ImuSample]) -> (Vec<f64>, LocalSlam) {
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu = imu.iter().peekable();
    let mut first = None;
    let errs = rec
        .scans
        .iter()
        .map(|(scan, yaw)| {
            while let Some(s) = imu.next_if(|s| s.t_ns <= scan.t_ns + PERIOD_NS) {
                slam.feed_imu(s);
            }
            let out = slam.feed_scan(scan);
            let (y0, t0) = *first.get_or_insert((out.yaw, *yaw));
            wrap((out.yaw - y0) - (yaw - t0))
        })
        .collect();
    (errs, slam)
}

fn heading_errors(rec: &Recording, imu: &[ImuSample]) -> Vec<f64> {
    replay(rec, imu).0
}

fn worst_deg(errs: &[f64]) -> f64 {
    errs.iter().map(|e| e.abs().to_degrees()).fold(0.0, f64::max)
}

fn mean_deg(errs: &[f64]) -> f64 {
    errs.iter().sum::<f64>().to_degrees() / errs.len() as f64
}

/// The recording's IMU with the specific force the capture service feeds
/// beside the FC's gyro: gravity in the attitude the FC last reported, not
/// the accelerometer, which on a multirotor reads the thrust.
fn with_fc_gravity(rec: &Recording) -> Vec<ImuSample> {
    rec.imu
        .iter()
        .map(|s| {
            let k = rec.fc_attitudes.partition_point(|(t, _)| *t <= s.t_ns);
            let accel = match k.checked_sub(1).map(|k| rec.fc_attitudes[k]) {
                Some((t, r)) if s.t_ns - t <= 500_000_000 => {
                    (r.inverse() * Vector3::new(0.0, 0.0, 9.81)).into()
                }
                _ => s.accel,
            };
            ImuSample { accel, ..*s }
        })
        .collect()
}

/// 25 s and 66 m of a forest trail, 56 degrees of bends, the airframe
/// pitching and rolling throughout. The rotation prior used to integrate
/// the rate about the world vertical, which differs from the levelled
/// scan's heading change by `∫roll̇·sin(pitch)`: the matched heading ended
/// 0.65 degrees off. It now stays within 0.3 degrees.
#[test]
fn forest_trail_bends_keep_the_heading() {
    let rec = recorded(include_str!("data/forest_trail.txt"));
    assert_eq!(rec.scans.len(), 250);
    let errs = heading_errors(&rec, &rec.imu);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("forest: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 0.3, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 0.2, "last {last:+.3} deg");
}

/// A take-off in a 12 m room yawing 190 degrees at up to 65 degrees/s while
/// the airframe tilts as it climbs: the heading follows the slew (it fell
/// 5.4 degrees behind with the vertical-rate prior).
#[test]
fn room_take_off_yaw_slew_is_tracked() {
    let rec = recorded(include_str!("data/room_takeoff_turn.txt"));
    assert_eq!(rec.scans.len(), 80);
    let errs = heading_errors(&rec, &rec.imu);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("room: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 1.5, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 1.0, "last {last:+.3} deg");
}

/// Waypoint turns in a warehouse aisle: 450 degrees of yaw in 8 s, slewing
/// at up to 160 degrees/s while the airframe rocks (23 degrees lost with the
/// vertical-rate prior).
#[test]
fn aisle_waypoint_turns_are_tracked() {
    let rec = recorded(include_str!("data/aisle_waypoint_turns.txt"));
    assert_eq!(rec.scans.len(), 80);
    let errs = heading_errors(&rec, &rec.imu);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("aisle: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 2.5, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 1.0, "last {last:+.3} deg");
}

/// Down a warehouse aisle with the airframe wobbling up to 7 degrees in
/// yaw. The flight controller here fused the lidar pose back into its EKF,
/// and its reported yaw ends 10 degrees off. With a prior from that
/// attitude the matched heading follows it; integrating the 100 Hz gyro
/// (`RAW_IMU`) it stays on the scans, also when that gyro reads 0.155
/// degrees/s high.
#[test]
fn aisle_yaw_wobble_holds_the_heading_on_the_gyro() {
    let rec = recorded(include_str!("data/aisle_yaw_wobble.txt"));
    assert_eq!(rec.scans.len(), 100);
    let errs = heading_errors(&rec, &rec.imu);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("wobble, gyro: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 0.5, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 0.3, "last {last:+.3} deg");

    // The same gyro reading 0.155 degrees/s high about its body z axis
    // (FRD; FLU -0.155): the heading stays on the scans.
    let bias = 0.155f64.to_radians();
    let biased: Vec<ImuSample> = rec
        .imu
        .iter()
        .map(|s| ImuSample {
            gyro: [s.gyro[0], s.gyro[1], s.gyro[2] - bias],
            ..*s
        })
        .collect();
    let errs = heading_errors(&rec, &biased);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("wobble, biased gyro: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 0.5, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 0.3, "last {last:+.3} deg");

    let fc = heading_errors(&rec, &rec.fc_attitude_imu);
    let fc_last = fc.last().unwrap().to_degrees();
    eprintln!("wobble, FC attitude: last {fc_last:+.3} deg");
    assert!(fc_last.abs() > 5.0, "the FC attitude's yaw error reaches the match: {fc_last:+.3} deg");
}

/// 70 s of a forest trail on the flight controller's own IMU, whose gyro
/// bias about body z rises from 0.08 to 0.13 degrees/s (FRD): 8 degrees of
/// heading over the slice. The matcher keeps most of its rotation prior on
/// every scan, so it adopted part of that drift, its map turning with it:
/// 0.35 degrees off on average and 0.63 by the end (worst 0.74). It now
/// learns the bias from the heading the scans turned against the gyro's,
/// tracks it as it rises, and takes it out of the prior.
#[test]
fn forest_gyro_bias_is_learnt_from_the_scans() {
    let rec = recorded(include_str!("data/forest_gyro_bias.txt"));
    assert_eq!(rec.scans.len(), 700);
    let (errs, slam) = replay(&rec, &with_fc_gravity(&rec));
    let (last, mean) = (errs.last().unwrap().to_degrees(), mean_deg(&errs));
    let bias = slam.gyro_bias_z().to_degrees();
    eprintln!(
        "forest, FC gyro: worst {:.3} deg, mean {mean:+.3} deg, last {last:+.3} deg, bias {bias:+.4} deg/s",
        worst_deg(&errs)
    );
    // The gyro's bias over the last 15 s, FLU.
    assert!((bias + 0.132).abs() < 0.04, "learnt bias {bias:+.4} deg/s");
    assert!(mean.abs() < 0.2, "mean {mean:+.3} deg");
    assert!(last.abs() < 0.25, "last {last:+.3} deg");
    assert!(worst_deg(&errs) < 0.5, "worst {:.3} deg", worst_deg(&errs));
}

/// Down a warehouse aisle, two fast turns at its end and back up the next
/// aisle on the flight controller's IMU. Braking into and out of the turns
/// the accelerometer leans with the thrust, up to 20 degrees off gravity;
/// levelled by it, the heading came out of the turns 4 degrees off and held
/// that up the return aisle. With gravity from the attitude the FC reported
/// (what the capture service feeds) the return leg stays on the scans.
#[test]
fn warehouse_return_leg_holds_the_heading() {
    let rec = recorded(include_str!("data/warehouse_return_leg.txt"));
    assert_eq!(rec.scans.len(), 750);
    // The return aisle: after the second turn, 41 s in.
    let leg = rec.scans.iter().position(|(s, _)| s.t_ns >= 41_000_000_000).unwrap();

    let errs = heading_errors(&rec, &with_fc_gravity(&rec));
    let (mean, worst) = (mean_deg(&errs[leg..]), worst_deg(&errs[leg..]));
    eprintln!("warehouse return leg, FC gravity: mean {mean:+.3} deg, worst {worst:.3} deg");
    assert!(mean.abs() < 0.3, "mean {mean:+.3} deg");
    assert!(worst < 0.5, "worst {worst:.3} deg");

    let raw = heading_errors(&rec, &rec.imu);
    let raw_mean = mean_deg(&raw[leg..]);
    eprintln!("warehouse return leg, accelerometer: mean {raw_mean:+.3} deg");
    assert!(raw_mean.abs() > 1.0, "the accelerometer's lean reaches the heading: {raw_mean:+.3} deg");
}

/// Replay with a 0.3 s hole in the IMU from `hole_ns` after the first scan
/// (its samples dropped), feeding the flight controller's heading with each
/// sample as the capture service does. Returns the seconds from the hole's
/// end until the pose is usable again (re-confirmed, or a restarted map
/// holding 10 scans), the worst heading error while usable after it (degrees,
/// re-based at a restart, where the capture re-anchors the north) and the
/// restarts.
fn holed_replay(rec: &Recording, imu: &[ImuSample], hole_ns: Option<u64>) -> (Option<f64>, f64, usize) {
    let t0 = rec.scans[0].0.t_ns;
    let hole = hole_ns.map_or((0, 0), |h| (t0 + h, t0 + h + 300_000_000));
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut it = imu.iter().filter(|s| !(hole.0 < s.t_ns && s.t_ns < hole.1)).peekable();
    let mut fed = 0;
    let mut base = None;
    let (mut usable_at, mut worst, mut restarts) = (None, 0.0f64, 0);
    for (scan, yaw) in &rec.scans {
        while let Some(s) = it.next_if(|s| s.t_ns <= scan.t_ns + PERIOD_NS) {
            let k = rec.fc_headings.partition_point(|(t, _)| *t <= s.t_ns);
            if k > fed {
                let (t, h) = rec.fc_headings[k - 1];
                slam.feed_fc_heading(t, h);
                fed = k;
            }
            slam.feed_imu(s);
        }
        let out = slam.feed_scan(scan);
        if out.map_restarted {
            restarts += 1;
            base = None;
        }
        let (y0, ty0) = *base.get_or_insert((out.yaw, *yaw));
        let e = wrap((out.yaw - y0) - (yaw - ty0)).to_degrees();
        if scan.t_ns > hole.1 && !out.heading_unconfirmed && slam.inserted_scans() >= 10 {
            usable_at.get_or_insert((scan.t_ns - hole.1) as f64 * 1e-9);
            worst = worst.max(e.abs());
        }
    }
    (usable_at, worst, restarts)
}

/// A 0.3 s hole whose samples dropped inside a turn, at the marks where the
/// pose used to stay lost for the rest of the recording (the aisle at 2 to
/// 5 s, the room take-off at 2 s, the warehouse at 30 s) and one where it
/// re-confirmed (the warehouse at 22 s). The pose is usable again within
/// seconds, on a heading within 2° of the truth (the aisle's own unholed
/// replay is up to 1.74° off); unholed, nothing restarts.
#[test]
fn a_short_imu_hole_in_a_turn_is_usable_again_within_seconds() {
    let aisle = recorded(include_str!("data/aisle_waypoint_turns.txt"));
    let room = recorded(include_str!("data/room_takeoff_turn.txt"));
    let warehouse = recorded(include_str!("data/warehouse_return_leg.txt"));
    let wh_imu = with_fc_gravity(&warehouse);
    let s = 1_000_000_000u64;
    let cases: [(&str, &Recording, &[ImuSample], u64); 7] = [
        ("aisle 2 s", &aisle, &aisle.imu, 2 * s),
        ("aisle 3 s", &aisle, &aisle.imu, 3 * s),
        ("aisle 4 s", &aisle, &aisle.imu, 4 * s),
        ("aisle 5 s", &aisle, &aisle.imu, 5 * s),
        ("room 2 s", &room, &room.imu, 2 * s),
        ("warehouse 22 s", &warehouse, &wh_imu, 22 * s),
        ("warehouse 30 s", &warehouse, &wh_imu, 30 * s),
    ];
    for (name, rec, imu, hole) in cases {
        let (usable, worst, restarts) = holed_replay(rec, imu, Some(hole));
        eprintln!("{name}: usable after {usable:?} s, worst {worst:.2} deg, restarts {restarts}");
        let usable = usable.unwrap_or_else(|| panic!("{name}: lost for the rest of the recording"));
        assert!(usable <= HOLE_USABLE_S, "{name}: usable only after {usable:.1} s");
        assert!(worst <= 2.0, "{name}: {worst:.2} deg off while usable");
    }
    for (name, rec, imu) in [("aisle", &aisle, &aisle.imu), ("room", &room, &room.imu), ("warehouse", &warehouse, &wh_imu)] {
        assert_eq!(holed_replay(rec, imu, None).2, 0, "{name}: restarted with no hole");
    }
}

/// The bound on [`a_short_imu_hole_in_a_turn_is_usable_again_within_seconds`].
const HOLE_USABLE_S: f64 = 4.0;
