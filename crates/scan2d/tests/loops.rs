mod common;

use std::f64::consts::PI;
use std::time::Instant;

use ados_scan2d::backend::optimize;
use ados_scan2d::{offline, LocalMap, Pose2, Scan2dConfig, Trajectory};
use common::{ate_xy, rect, relative, walk, Path, Seg, C1, PERIOD_NS};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

fn poses(t: &Trajectory) -> Vec<Pose2> {
    t.poses
        .iter()
        .map(|p| Pose2::new(p.x, p.y, p.yaw))
        .collect()
}

/// Scan poses as the local map places them, before optimisation.
fn local_poses(map: &LocalMap) -> Vec<Pose2> {
    map.nodes
        .iter()
        .map(|n| map.submaps[n.submap].pose * n.in_submap)
        .collect()
}

fn end_to_start(est: &[Pose2], truth: &[Pose2]) -> f64 {
    let e = relative(&est[0], est.last().unwrap());
    let t = relative(&truth[0], truth.last().unwrap());
    e.dist(&t)
}

/// A 2 m wide corridor around a 18 × 6 m block, the walker's centre line a
/// 20 × 8 m rectangle. Like an office corridor, both walls carry fittings
/// (pillars, cabinets, alcove ends) of assorted sizes at irregular spacing.
fn corridor_ring() -> Vec<Seg> {
    let mut w = rect(-2.0, -1.0, 20.0, 9.0);
    w.extend(rect(0.0, 1.0, 18.0, 7.0));
    // Fittings on the long walls: (x0, x1, depth) off the outer wall (y = -1,
    // y = 9) and the block (y = 1, y = 7).
    for (x0, x1, d) in [
        (1.2, 1.5, 0.2),
        (3.0, 3.3, 0.25),
        (5.6, 6.6, 0.15),
        (7.7, 8.0, 0.25),
        (10.1, 10.5, 0.3),
        (12.2, 12.6, 0.25),
        (14.9, 15.7, 0.12),
        (16.4, 16.6, 0.25),
    ] {
        w.extend(rect(x0, -1.0, x1, -1.0 + d));
    }
    for (x0, x1, d) in [
        (2.2, 2.5, 0.25),
        (4.3, 5.1, 0.12),
        (6.0, 6.4, 0.25),
        (8.8, 9.0, 0.3),
        (10.9, 11.1, 0.25),
        (13.4, 14.4, 0.15),
        (15.5, 15.9, 0.25),
        (17.3, 17.6, 0.2),
    ] {
        w.extend(rect(x0, 9.0 - d, x1, 9.0));
    }
    for (x0, x1, d) in [
        (0.8, 1.1, 0.2),
        (3.9, 4.9, 0.12),
        (7.1, 7.3, 0.3),
        (9.6, 10.0, 0.18),
        (12.9, 13.2, 0.25),
        (15.8, 16.9, 0.1),
    ] {
        w.extend(rect(x0, 1.0 - d, x1, 1.0));
    }
    for (x0, x1, d) in [
        (1.6, 2.6, 0.12),
        (5.2, 5.5, 0.25),
        (8.1, 8.4, 0.2),
        (11.3, 12.4, 0.1),
        (14.2, 14.4, 0.3),
        (16.7, 17.0, 0.18),
    ] {
        w.extend(rect(x0, 7.0, x1, 7.0 + d));
    }
    // Short walls: (y0, y1, depth) off x = 20, x = -2, the block's x = 18 and
    // x = 0.
    for (y0, y1, d) in [(1.4, 1.7, 0.2), (3.0, 3.4, 0.25), (5.6, 5.8, 0.2)] {
        w.extend(rect(20.0 - d, y0, 20.0, y1));
    }
    for (y0, y1, d) in [(2.5, 2.8, 0.25), (4.1, 4.9, 0.12), (5.9, 6.3, 0.25)] {
        w.extend(rect(-2.0, y0, -2.0 + d, y1));
    }
    for (y0, y1, d) in [(2.2, 2.5, 0.2), (4.6, 5.4, 0.12)] {
        w.extend(rect(18.0, y0, 18.0 + d, y1));
    }
    for (y0, y1, d) in [(1.9, 2.9, 0.12), (5.1, 5.4, 0.25)] {
        w.extend(rect(-d, y0, 0.0, y1));
    }
    w
}

#[test]
fn closes_a_rectangular_loop() {
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.2, 1.0)
        .line(17.0)
        .arc(1.0, PI / 2.0)
        .line(6.0)
        .arc(1.0, PI / 2.0)
        .line(18.0)
        .arc(1.0, PI / 2.0)
        .line(6.0)
        .arc(1.0, PI / 2.0)
        .line(8.0);
    let (scans, imu, truth) = walk(&corridor_ring(), &path, C1, 7);
    let c = cfg();
    let started = Instant::now();
    let mut map = offline::local(&scans, &imu, &c);
    assert_eq!(map.nodes.len(), truth.len());
    let local_ate = ate_xy(&local_poses(&map), &truth);

    // Odometry scale drift growing to 2 % by the last submap: every
    // submap-to-submap step is stretched by its share.
    let old: Vec<Pose2> = map.submaps.iter().map(|s| s.pose).collect();
    let n = old.len();
    for k in 1..n {
        let rel = old[k - 1].inverse() * old[k];
        let s = 1.0 + 0.02 * k as f64 / (n - 1) as f64;
        map.submaps[k].pose = map.submaps[k - 1].pose * Pose2::new(rel.x * s, rel.y * s, rel.th);
    }
    let drifted = local_poses(&map);
    let traj = optimize(&map, &c);
    let elapsed = started.elapsed();
    let est = poses(&traj);
    let (e2s, ate) = (end_to_start(&est, &truth), ate_xy(&est, &truth));
    eprintln!(
        "rectangle: {} scans, {} submaps, loops {} accepted / {} rejected, {:.2?}",
        truth.len(),
        traj.submaps,
        traj.loops_accepted,
        traj.loops_rejected,
        elapsed
    );
    let errs = loop_errors(&map, &traj, &truth);
    eprintln!(
        "rectangle: local ATE {local_ate:.4} m; drifted end-to-start {:.4} m, ATE {:.4} m; optimised end-to-start {e2s:.4} m, ATE {ate:.4} m; loop error mean {:.4} max {:.4} m",
        end_to_start(&drifted, &truth),
        ate_xy(&drifted, &truth),
        errs.iter().sum::<f64>() / errs.len().max(1) as f64,
        errs.iter().copied().fold(0.0, f64::max)
    );
    assert!(traj.loops_accepted > 0);
    assert!(
        end_to_start(&drifted, &truth) > 0.1,
        "the injected drift must open the loop"
    );
    assert!(e2s < 0.05, "end-to-start {e2s} m");
    assert!(ate < 0.05, "ATE {ate} m");
}

/// Bay pitch of the racking.
const BAY_M: f64 = 4.0;

/// A 40 m aisle between two racks of identical 4 m bays (uprights, pallet
/// faces with gaps to the rack back), closed by walls at both ends.
fn racked_aisle() -> Vec<Seg> {
    let mut w = vec![
        ([-3.0, -2.8], [43.0, -2.8]),
        ([-3.0, 2.8], [43.0, 2.8]),
        ([-3.0, -2.8], [-3.0, 2.8]),
        ([43.0, -2.8], [43.0, 2.8]),
    ];
    for b in 0..=10 {
        let x0 = b as f64 * BAY_M;
        w.extend(rect(x0 - 0.05, 1.5, x0 + 0.05, 1.6));
        w.extend(rect(x0 - 0.05, -1.6, x0 + 0.05, -1.5));
        if b < 10 {
            for y in [1.75f64, -1.75] {
                let back = 2.6 * y.signum();
                w.push(([x0 + 0.3, y], [x0 + 3.7, y]));
                w.push(([x0 + 0.3, y], [x0 + 0.3, back]));
                w.push(([x0 + 3.7, y], [x0 + 3.7, back]));
            }
        }
    }
    w
}

#[test]
fn refuses_one_bay_aliasing() {
    let path = Path::new(1.0, -0.4, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(36.0)
        .arc(0.7, PI)
        .line(37.0);
    let (scans, imu, truth) = walk(&racked_aisle(), &path, C1, 11);
    let c = cfg();
    let started = Instant::now();
    let mut map = offline::local(&scans, &imu, &c);
    assert_eq!(map.nodes.len(), truth.len());

    // Drift on the walk back: 0.5 m along the aisle for its first half, then
    // a further jump of 2.5 m, so that past the middle the prior sits between
    // the true bay and the next one.
    for sm in &mut map.submaps {
        if sm.pose.th.abs() > PI / 2.0 {
            sm.pose.x += if sm.pose.x > 20.0 { 0.5 } else { 3.0 };
        }
    }
    let traj = optimize(&map, &c);
    let elapsed = started.elapsed();
    let errs = loop_errors(&map, &traj, &truth);
    let worst = errs.iter().copied().fold(0.0, f64::max);
    for (l, err) in traj.loops.iter().zip(&errs) {
        assert!(
            *err < 0.2,
            "loop at t {} to submap {} is off by {err:.3} m",
            l.t_ns,
            l.submap
        );
    }
    eprintln!(
        "aisle: {} scans, {} submaps, loops {} accepted / {} rejected, worst accepted error {worst:.4} m, {:.2?}",
        truth.len(),
        traj.submaps,
        traj.loops_accepted,
        traj.loops_rejected,
        elapsed
    );
    assert!(traj.loops_accepted > 0);
    assert!(traj.loops_rejected > 0, "no aliased candidate was refused");
}

/// Translation error of each accepted loop against ground truth: the loop
/// places its scan in the submap's frame; the submap's scan nearest to that
/// spot ties the frame to the truth.
fn loop_errors(map: &LocalMap, traj: &Trajectory, truth: &[Pose2]) -> Vec<f64> {
    let truth_at = |t_ns: u64| truth[(t_ns / PERIOD_NS) as usize];
    traj.loops
        .iter()
        .map(|l| {
            let reference = map
                .nodes
                .iter()
                .filter(|n| n.submap == l.submap)
                .min_by(|a, b| {
                    a.in_submap
                        .dist(&l.pose)
                        .total_cmp(&b.in_submap.dist(&l.pose))
                })
                .expect("a scan in the submap");
            let est = reference.in_submap.inverse() * l.pose;
            est.dist(&relative(&truth_at(reference.t_ns), &truth_at(l.t_ns)))
        })
        .collect()
}
