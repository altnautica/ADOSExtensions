//! A planar LiDAR tilted 10° in a room of vertical walls: the tilted scan
//! plane meets each wall farther away than the level plane would, and
//! levelling by the IMU's roll and pitch must put every point back on the
//! wall's footprint.

mod common;

use std::f64::consts::PI;

use ados_scan2d::scan::Motion;
use ados_scan2d::{ImuSample, LocalSlam, Scan2d, Scan2dConfig, ScanInput};
use common::{cast, rect, Seg, BEAMS, PERIOD_NS};
use nalgebra::{UnitQuaternion, Vector3};

const TILT: f64 = 10.0 * PI / 180.0;

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// The level-frame direction of a sensor-plane beam at `phi`, as
/// `Ry(pitch)·Rx(roll)` (the levelling `Scan2d::from_cloud` applies).
fn world_dir(phi: f64, roll: f64, pitch: f64) -> Vector3<f64> {
    let q = UnitQuaternion::from_euler_angles(0.0, pitch, 0.0)
        * UnitQuaternion::from_euler_angles(roll, 0.0, 0.0);
    q * Vector3::new(phi.cos(), phi.sin(), 0.0)
}

/// One noiseless revolution of a sensor at the origin tilted by `roll` and
/// `pitch` inside `world`: each beam's 3D range runs to the wall its
/// horizontal footprint meets.
fn tilted_scan(world: &[Seg], t_ns: u64, roll: f64, pitch: f64) -> ScanInput {
    let mut scan = ScanInput {
        t_ns,
        ..Default::default()
    };
    for k in 0..BEAMS {
        let phi = 2.0 * PI * k as f64 / BEAMS as f64;
        let d = world_dir(phi, roll, pitch);
        let horiz = d.x.hypot(d.y);
        let Some(h) = cast(world, [0.0, 0.0], d.y.atan2(d.x)) else {
            continue;
        };
        let r = h / horiz;
        scan.points
            .push([(r * phi.cos()) as f32, (r * phi.sin()) as f32, 0.0]);
        scan.t_offset_ns.push(0);
    }
    scan
}

/// Whether `p` lies on one of the room's walls within `tol`.
fn on_wall(p: [f32; 2], half: f64, tol: f64) -> bool {
    let (x, y) = (p[0] as f64, p[1] as f64);
    let near = |v: f64| (v.abs() - half).abs() < tol;
    (near(x) && y.abs() <= half + tol) || (near(y) && x.abs() <= half + tol)
}

#[test]
fn levelling_a_10_degree_roll_puts_points_on_the_wall_footprint() {
    let half = 4.0;
    let world = rect(-half, -half, half, half);
    for (roll, pitch) in [(TILT, 0.0), (0.0, TILT), (TILT, -TILT)] {
        let scan = tilted_scan(&world, 0, roll, pitch);
        // Unlevelled, the tilted plane reads the walls too far away.
        let raw = Scan2d::from_cloud(&scan, &cfg(), &Motion::default());
        let off = raw
            .points
            .iter()
            .filter(|p| !on_wall(**p, half, 0.02))
            .count();
        assert!(
            off > raw.points.len() / 4,
            "a 10° tilt must distort the raw scan"
        );

        let levelled = Scan2d::from_cloud(
            &scan,
            &cfg(),
            &Motion {
                roll,
                pitch,
                ..Motion::default()
            },
        );
        assert!(levelled.points.len() > BEAMS * 9 / 10);
        for p in &levelled.points {
            assert!(
                on_wall(*p, half, 1e-3),
                "{p:?} off the wall at roll {roll} pitch {pitch}"
            );
        }
    }
}

#[test]
fn a_still_sensor_rolled_10_degrees_holds_its_pose() {
    let world = {
        let mut w = rect(-5.0, -3.0, 6.0, 4.0);
        // Asymmetric clutter so the match is well conditioned.
        w.extend(rect(1.0, 1.0, 2.0, 1.6));
        w.extend(rect(-3.0, -2.0, -2.4, -0.8));
        w
    };
    let g =
        UnitQuaternion::from_euler_angles(TILT, 0.0, 0.0).inverse() * Vector3::new(0.0, 0.0, 9.81);
    let mut slam = LocalSlam::new(cfg());
    let mut imu_t = 0u64;
    let mut last = None;
    for i in 0..40u64 {
        let t = i * PERIOD_NS;
        while imu_t <= t {
            slam.feed_imu(&ImuSample {
                t_ns: imu_t,
                gyro: [0.0; 3],
                accel: g.into(),
            });
            imu_t += 5_000_000;
        }
        last = Some(slam.feed_scan(&tilted_scan(&world, t, TILT, 0.0)));
    }
    let p = last.expect("a pose");
    assert!((p.roll - TILT).abs() < 0.01, "roll {}", p.roll);
    assert!(p.x.hypot(p.y) < 0.03, "drifted to ({}, {})", p.x, p.y);
    assert!(p.yaw.abs() < 0.01, "yaw {}", p.yaw);
}
