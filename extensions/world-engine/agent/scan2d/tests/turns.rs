//! Heading through turns, on revolutions recorded from simulated flights:
//! the matched yaw must follow the true yaw of the LiDAR, not drift with
//! the turns or lag a fast yaw slew.
//!
//! Each recording carries the LiDAR's true attitude at the rate a flight
//! controller reports it (20 Hz); the IMU stream is synthesised from it the
//! way the capture service does for a companion with no IMU of its own (body
//! rate from consecutive attitudes, gravity as the specific force).

use std::f64::consts::PI;

use ados_scan2d::{ImuSample, LocalSlam, Scan2dConfig, ScanInput};
use nalgebra::{Rotation3, Vector3};

/// Beams per revolution and revolution period of the recorded LiDAR.
const BEAMS: u64 = 500;
const PERIOD_NS: u64 = 100_000_000;

/// One recording: IMU samples and `(revolution, true yaw)` in time order.
struct Recording {
    imu: Vec<ImuSample>,
    scans: Vec<(ScanInput, f64)>,
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

/// Parse a `tests/data` flight (format in each file's header).
fn recorded(text: &str) -> Recording {
    let urad = |v: i64| v as f64 * 1e-6;
    let mut atts: Vec<(u64, Rotation3<f64>)> = Vec::new();
    let mut scans = Vec::new();
    for l in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let mut f = l.split_whitespace();
        let kind = f.next().expect("a line kind");
        let v: Vec<i64> = f.map(|x| x.parse().expect("an integer")).collect();
        let t_ns = v[0] as u64 * 1_000;
        match kind {
            "a" => atts.push((t_ns, Rotation3::from_euler_angles(urad(v[1]), urad(v[2]), urad(v[3])))),
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
    let imu = atts
        .windows(2)
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
        .collect();
    Recording { imu, scans }
}

/// Replay as the capture service feeds the matcher (each revolution after
/// the IMU up to its last return) and return the heading error of every
/// scan against the truth, relative to the first scan, radians.
fn heading_errors(rec: &Recording) -> Vec<f64> {
    let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut imu = rec.imu.iter().peekable();
    let mut first = None;
    rec.scans
        .iter()
        .map(|(scan, yaw)| {
            while let Some(s) = imu.next_if(|s| s.t_ns <= scan.t_ns + PERIOD_NS) {
                slam.feed_imu(s);
            }
            let out = slam.feed_scan(scan);
            let (y0, t0) = *first.get_or_insert((out.yaw, *yaw));
            wrap((out.yaw - y0) - (yaw - t0))
        })
        .collect()
}

fn worst_deg(errs: &[f64]) -> f64 {
    errs.iter().map(|e| e.abs().to_degrees()).fold(0.0, f64::max)
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
    let errs = heading_errors(&rec);
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
    let errs = heading_errors(&rec);
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
    let errs = heading_errors(&rec);
    let last = errs.last().unwrap().to_degrees();
    eprintln!("aisle: worst {:.3} deg, last {last:+.3} deg", worst_deg(&errs));
    assert!(worst_deg(&errs) < 2.5, "worst {:.3} deg", worst_deg(&errs));
    assert!(last.abs() < 1.0, "last {last:+.3} deg");
}
