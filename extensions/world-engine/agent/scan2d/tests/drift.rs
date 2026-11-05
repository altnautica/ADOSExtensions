//! Local SLAM where the scans constrain one direction weakly or not at all:
//! the pose must not slide along it.

mod common;

use std::f64::consts::PI;

use ados_scan2d::{offline, LocalMap, LocalSlam, Pose2, Scan2dConfig, ScanInput};
use common::{ate_xy, rect, walk, Path, Seg, C1};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// Scan poses as the local map places them.
fn local_poses(map: &LocalMap) -> Vec<Pose2> {
    map.nodes
        .iter()
        .map(|n| map.submaps[n.submap].pose * n.in_submap)
        .collect()
}

/// The revolutions of `tests/data/racked_aisle_still.txt`.
fn recorded(text: &str) -> Vec<ScanInput> {
    text.lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let f: Vec<i64> = l
                .split_whitespace()
                .map(|v| v.parse().expect("an integer"))
                .collect();
            let n = f[1] as usize;
            assert_eq!(f.len(), 2 + 3 * n, "point count");
            let pts = f[2..].chunks(3);
            let m = |v: i64| (v as f64 * 1e-4) as f32;
            ScanInput {
                t_ns: f[0] as u64 * 1_000,
                points: pts.clone().map(|p| [m(p[0]), m(p[1]), 0.0]).collect(),
                t_offset_ns: pts.map(|p| p[2] as u32 * 1_000).collect(),
            }
        })
        .collect()
}

/// A still LiDAR in a simulated racked aisle sees only isolated uprights
/// along the aisle (one or two returns each) and two short crossbars across
/// it. The uprights pin the aisle axis. Rated by the crossbars alone, the
/// scans were degenerate along it and the constant-velocity prior carried
/// the pose 3 m down the aisle in 20 s. The bound is one grid cell: the
/// uprights stand on cell boundaries, and the second scan, matched against
/// a submap of one, takes the neighbouring cell.
#[test]
fn still_among_rack_uprights_holds_its_pose() {
    let scans = recorded(include_str!("data/racked_aisle_still.txt"));
    assert_eq!(scans.len(), 200);
    let mut slam = LocalSlam::new(cfg());
    let (mut worst, mut last, mut min_deg) = (0.0f64, 0.0, f64::INFINITY);
    for s in &scans {
        let p = slam.feed_scan(s);
        last = p.x.hypot(p.y);
        worst = worst.max(last);
        if p.score > 0.0 {
            min_deg = min_deg.min(p.degeneracy);
        }
    }
    eprintln!(
        "still among uprights: worst offset {worst:.4} m, last {last:.4} m, lowest degeneracy {min_deg:.3}"
    );
    assert!(
        min_deg >= cfg().degenerate_ratio,
        "rated degenerate ({min_deg})"
    );
    assert!(worst <= 0.05, "drifted {worst} m");
}

/// A walker goes 15 m down a 2 m wide corridor from its closed end and
/// stands for 10 s. A few metres in, the end wall is too small a share of
/// the scan to pin the corridor axis and the match is degenerate: the pose
/// may lag the walker from there, but it must come to rest, not run on at
/// the walking speed after the walker stopped.
#[test]
fn bare_corridor_does_not_run_away() {
    let world = vec![
        ([-2.0, -1.0], [200.0, -1.0]),
        ([-2.0, 1.0], [200.0, 1.0]),
        ([-2.0, -1.0], [-2.0, 1.0]),
    ];
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.0, 0.5)
        .line(14.0)
        .speed_to(0.0, 0.5)
        .wait(10.0);
    let (scans, imu, truth) = walk(&world, &path, C1, 21);
    let est = local_poses(&offline::local(&scans, &imu, &cfg()));
    assert_eq!(est.len(), truth.len());
    // The last 5 s: the walker has stood still for 5 s by then.
    let tail = &est[est.len() - 50..];
    let ran_on = tail.last().unwrap().x - tail[0].x;
    let ahead = est
        .iter()
        .zip(&truth)
        .map(|(e, t)| e.x - t.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let across = est.iter().map(|e| e.y.abs()).fold(0.0, f64::max);
    eprintln!(
        "bare corridor: walked {:.2} m, estimate {:.2} m; {ran_on:.4} m in the last 5 s, \
         at most {ahead:.3} m ahead, {across:.3} m across",
        truth.last().unwrap().x,
        tail.last().unwrap().x
    );
    assert!(ran_on.abs() < 0.01, "ran on {ran_on} m in the last 5 s");
    assert!(ahead < 0.1, "ran {ahead} m ahead of the walker");
    assert!(across < 0.05, "{across} m across the corridor");
}

/// Rack uprights only, 8 cm square: two rows either side of the aisle at
/// a 2.7 m bay pitch, nothing else.
fn upright_aisle() -> Vec<Seg> {
    let mut w = Vec::new();
    for b in -2..=16 {
        let x = b as f64 * 2.7;
        for y in [-8.6, -6.3, 6.3, 8.6] {
            w.extend(rect(x - 0.04, y - 0.04, x + 0.04, y + 0.04));
        }
    }
    w
}

/// Down an aisle of bare uprights and back: every scan is matched on the
/// uprights alone, along the aisle too.
#[test]
fn walking_past_rack_uprights_tracks_the_aisle() {
    let path = Path::new(1.0, -0.4, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(36.0)
        .arc(0.7, PI)
        .line(37.0);
    let (scans, imu, truth) = walk(&upright_aisle(), &path, C1, 11);
    let c = cfg();
    let map = offline::local(&scans, &imu, &c);
    let est = local_poses(&map);
    let degenerate = map
        .nodes
        .iter()
        .filter(|n| n.score > 0.0 && n.degeneracy < c.degenerate_ratio)
        .count();
    let local: Vec<Pose2> = truth.iter().map(|t| truth[0].inverse() * *t).collect();
    let ate = ate_xy(&est, &local);
    eprintln!(
        "upright aisle: {} scans, {degenerate} degenerate, local ATE {ate:.4} m",
        est.len()
    );
    assert_eq!(degenerate, 0);
    assert!(ate < 0.3, "local ATE {ate} m");
}
