use super::{Rng, SEC};
use crate::{VoxMap, VoxMapConfig, ESDF_CAP_M};

/// A 6 x 6 x 3 m window, inside the distance cap, so every voxel has a
/// distance to check.
fn map40() -> VoxMap {
    VoxMap::new(VoxMapConfig {
        res_m: 0.15,
        dims: [40, 40, 20],
        recentre_slack_m: 0.0,
        esdf_every: 1,
    })
    .unwrap()
}

fn random_cell(m: &VoxMap, rng: &mut Rng) -> [i32; 3] {
    let o = m.origin_cell();
    let n = m.dims().map(i32::from);
    [0, 1, 2].map(|a| o[a] + rng.below(n[a]))
}

/// The incremental field matches an exhaustive transform on a random map,
/// after building it, after removing and adding obstacles, and after the
/// window moves.
#[test]
fn esdf_matches_brute_force_on_a_random_map() {
    let mut m = map40();
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut placed = Vec::new();
    for _ in 0..160 {
        let c = random_cell(&m, &mut rng);
        m.set_cell(c, 40);
        placed.push(c);
    }
    m.update_esdf();
    let built = m.check_esdf(0.5);

    for (i, c) in placed.iter().enumerate() {
        if i % 2 == 0 {
            m.set_cell(*c, 0);
        }
    }
    for _ in 0..40 {
        let c = random_cell(&m, &mut rng);
        m.set_cell(c, 40);
    }
    m.update_esdf();
    let changed = m.check_esdf(0.5);

    assert!(m.recentre([0.9, -0.45, 0.3]));
    assert_ne!(m.om, [20, 20, 10], "the ring is offset");
    let moved = m.check_esdf(0.5);

    // Remove every obstacle: the field empties.
    for c in m.obstacles() {
        m.set_cell(c, 0);
    }
    m.update_esdf();
    m.check_esdf(0.0);
    eprintln!("largest overestimate (voxels): built {built}, changed {changed}, moved {moved}");
}

/// One obstacle: every distance is exact, capped at 4 m, and clearance
/// reads it in metres.
#[test]
fn a_single_obstacle_gives_exact_capped_distances() {
    let mut m = VoxMap::new(VoxMapConfig {
        res_m: 0.15,
        dims: [80, 16, 8],
        recentre_slack_m: 0.0,
        esdf_every: 1,
    })
    .unwrap();
    let o = [0.07, 0.07, 0.07];
    m.insert_cloud(SEC, o, &[[0.07, 0.07, 0.07 + 0.3]], 10.0);
    m.update_esdf();
    assert_eq!(m.check_esdf(0.0), 0.0);
    let obstacle = m.centre_of(m.cell_of([0.07, 0.07, 0.37]));
    let p = [obstacle[0] + 1.5, obstacle[1], obstacle[2]];
    assert!((m.clearance_m(p).unwrap() - 1.5).abs() < 1e-4);
    assert_eq!(m.clearance_m(obstacle), Some(0.0));
    let far = [obstacle[0] + 5.0, obstacle[1], obstacle[2]];
    assert_eq!(m.clearance_m(far), Some(ESDF_CAP_M));
    assert_eq!(m.clearance_m([50.0, 0.0, 0.0]), None);
}

/// Long-range churn on the default window: obstacles appear, vanish and the
/// window follows a moving vehicle; the field stays consistent with an
/// exhaustive check on every voxel near the obstacles.
#[test]
fn esdf_stays_consistent_through_churn_and_moves() {
    let mut m = map40();
    let mut rng = Rng(42);
    for step in 0..12 {
        for _ in 0..25 {
            let c = random_cell(&m, &mut rng);
            m.set_cell(c, 40);
        }
        for c in m.obstacles() {
            if rng.below(4) == 0 {
                m.set_cell(c, 0);
            }
        }
        m.update_esdf();
        let shift = step as f32 * 0.4;
        m.recentre([shift, -shift * 0.5, (step % 3) as f32 * 0.3]);
    }
    m.check_esdf(0.5);
}

/// A voxel hit once and then passed once reads unknown but stays an
/// obstacle to the distance field until its positive evidence is gone.
#[test]
fn obstacles_leave_the_field_only_without_positive_evidence() {
    let mut m = map40();
    let c = [3, 0, 0];
    m.set_cell(c, crate::LO_HIT);
    m.update_esdf();
    let p = m.centre_of(c);
    assert_eq!(m.clearance_m(p), Some(0.0));
    m.set_cell(c, crate::LO_HIT + crate::LO_MISS);
    m.update_esdf();
    assert_eq!(m.state(c), crate::VoxelState::Unknown);
    assert_eq!(m.clearance_m(p), Some(0.0), "still an obstacle");
    m.set_cell(c, 0);
    m.update_esdf();
    assert_eq!(m.clearance_m(p), Some(ESDF_CAP_M));
    m.check_esdf(0.0);
}

/// With `esdf_every` 2 the field updates on every second tick; changes
/// queue in between.
#[test]
fn the_field_can_run_every_other_frame() {
    let mut m = VoxMap::new(VoxMapConfig {
        res_m: 0.15,
        dims: [40, 40, 20],
        recentre_slack_m: 0.0,
        esdf_every: 2,
    })
    .unwrap();
    let c = [2, 1, 0];
    let next = m.centre_of([3, 1, 0]);
    m.set_cell(c, 40);
    assert!(!m.tick_esdf());
    assert_eq!(m.clearance_m(next), Some(ESDF_CAP_M), "not yet updated");
    assert!(m.tick_esdf());
    assert!((m.clearance_m(next).unwrap() - 0.15).abs() < 1e-5);
    m.check_esdf(0.0);
}
