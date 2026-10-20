mod common;

use ados_scan2d::grid::{Grid, Pyramid};
use ados_scan2d::matcher::{
    branch_and_bound, constraint_information, correlative, degeneracy, refine, Window,
};
use ados_scan2d::scan::Motion;
use ados_scan2d::{Pose2, Scan2d, Scan2dConfig, ScanInput};
use common::{rect, revolution, Rng, Seg, Sensor, C1};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// A furnished 8 × 6 m room.
fn room() -> Vec<Seg> {
    let mut w = rect(0.0, 0.0, 8.0, 6.0);
    w.extend(rect(1.0, 1.0, 1.8, 1.6));
    w.extend(rect(5.5, 3.8, 6.5, 4.4));
    w.extend(rect(3.5, 4.5, 3.8, 4.8));
    w.push(([6.0, 0.5], [7.5, 2.0]));
    w
}

fn still_scan(world: &[Seg], at: Pose2, sensor: Sensor, seed: u64) -> Scan2d {
    let raw = revolution(world, &|_| at, 0.0, sensor, &mut Rng::new(seed));
    Scan2d::from_cloud(&raw, &cfg(), &Motion::default())
}

fn grid_of(scans: &[(Scan2d, Pose2)]) -> Grid {
    let mut g = Grid::new(&cfg());
    for (s, at) in scans {
        let pts: Vec<[f64; 2]> = s
            .points
            .iter()
            .map(|p| at.apply([p[0] as f64, p[1] as f64]))
            .collect();
        g.insert([at.x, at.y], &pts);
    }
    g
}

fn local_window(c: &Scan2dConfig) -> Window {
    Window {
        linear_m: c.linear_window_m,
        angular_rad: c.angular_window_deg.to_radians(),
    }
}

#[test]
fn matches_a_known_offset_within_one_cell() {
    let w = room();
    let a = Pose2::new(3.0, 2.5, 0.3);
    let offset = Pose2::new(0.2, 0.0, 5f64.to_radians());
    let sa = still_scan(&w, a, C1, 1);
    let sb = still_scan(&w, a * offset, C1, 2);
    let grid = grid_of(&[(sa, Pose2::IDENTITY)]);
    let c = cfg();
    let (coarse, _) = correlative(&grid, &sb, Pose2::IDENTITY, local_window(&c));
    let r = refine(&grid, &sb, coarse, None, c.gn_max_iters, c.gn_eps);
    let (dt, dth) = (
        r.pose.dist(&offset),
        (r.pose.th - offset.th).abs().to_degrees(),
    );
    eprintln!(
        "offset error {dt:.4} m, {dth:.3} deg (coarse {:.4} m)",
        coarse.dist(&offset)
    );
    assert!(dt < 0.05, "translation error {dt} m");
    assert!(dth < 0.5, "rotation error {dth} deg");
}

#[test]
fn branch_and_bound_finds_the_exhaustive_optimum() {
    let w = room();
    let a = Pose2::new(4.0, 3.0, -0.4);
    let truth = Pose2::new(-0.6, 0.45, 0.12);
    let sa = still_scan(&w, a, C1, 3);
    let sb = still_scan(&w, a * truth, C1, 4);
    let grid = grid_of(&[(sa, Pose2::IDENTITY)]);
    let window = Window {
        linear_m: 0.8,
        angular_rad: 9f64.to_radians(),
    };
    // One level: every candidate is scored at full resolution.
    let flat = Pyramid::new(&grid, 1);
    let (_, ex_score) =
        branch_and_bound(&flat, &sb, Pose2::IDENTITY, window, 0.0).expect("a match");
    let pyr = Pyramid::new(&grid, 7);
    let (bb, bb_score) =
        branch_and_bound(&pyr, &sb, Pose2::IDENTITY, window, 0.0).expect("a match");
    assert!(
        (ex_score - bb_score).abs() < 1e-9,
        "exhaustive {ex_score} vs pruned {bb_score}"
    );
    assert!(bb.dist(&truth) < 0.1, "{bb:?} vs {truth:?}");
    assert!(branch_and_bound(&pyr, &sb, Pose2::IDENTITY, window, bb_score + 1e-6).is_none());
}

/// Degeneracy of a scan at `at`, refined into a grid built from scans taken
/// along `step` either side of it, as a submap would hold.
fn degeneracy_at(world: &[Seg], at: Pose2, step: [f64; 2]) -> f64 {
    let built: Vec<(Scan2d, Pose2)> = (-6..=6)
        .map(|k| {
            let p = Pose2::new(at.x + k as f64 * step[0], at.y + k as f64 * step[1], at.th);
            (still_scan(world, p, C1, (106 + k) as u64), p)
        })
        .collect();
    let grid = grid_of(&built);
    let c = cfg();
    let scan = still_scan(world, at, C1, 6);
    let r = refine(&grid, &scan, at, None, c.gn_max_iters, c.gn_eps);
    degeneracy(&constraint_information(&scan, r.pose))
}

#[test]
fn corridor_is_reported_degenerate() {
    let corridor = vec![([-60.0, -1.0], [60.0, -1.0]), ([-60.0, 1.0], [60.0, 1.0])];
    let corner = vec![([1.5, -6.0], [1.5, 1.5]), ([-6.0, 1.5], [1.5, 1.5])];
    let d_corridor = degeneracy_at(&corridor, Pose2::new(0.0, 0.2, 0.05), [0.5, 0.0]);
    let d_corner = degeneracy_at(&corner, Pose2::new(0.0, 0.0, 0.3), [0.08, -0.08]);
    eprintln!("degeneracy corridor {d_corridor:.4}, corner {d_corner:.4}");
    assert!(d_corridor < 0.05, "corridor {d_corridor}");
    assert!(d_corner > 0.2, "corner {d_corner}");
}

#[test]
fn blocked_sector_points_are_dropped() {
    let c = Scan2dConfig::for_sensor(0.05, 12.0, &[[130.0, 230.0], [350.0, 10.0]]);
    let mut raw = ScanInput::default();
    for k in 0..360 {
        let a = (k as f64 + 0.5).to_radians();
        raw.points
            .push([(3.0 * a.cos()) as f32, (3.0 * a.sin()) as f32, 0.0]);
        raw.t_offset_ns.push(k);
    }
    // Inside the 0.35 m floor and beyond the 12 m ceiling, in open sectors.
    raw.points.push([0.2, 0.1, 0.0]);
    raw.points.push([0.0, 13.0, 0.0]);
    let s = Scan2d::from_cloud(&raw, &c, &Motion::default());
    assert_eq!(s.points.len(), 240);
    for p in &s.points {
        let az = (p[1] as f64)
            .atan2(p[0] as f64)
            .to_degrees()
            .rem_euclid(360.0);
        assert!(!(130.0..=230.0).contains(&az), "{az} in the rear sector");
        assert!(!(az >= 350.0 || az <= 10.0), "{az} in the wrapping sector");
        let r = (p[0] as f64).hypot(p[1] as f64);
        assert!((r - 3.0).abs() < 1e-5, "range {r}");
    }
}
