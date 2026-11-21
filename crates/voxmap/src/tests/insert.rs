use super::{map64, SEC};
use crate::{Pose, RayHit, VoxelState, LO_HIT, LO_MAX, LO_MIN, LO_MISS, VOX_FREE, VOX_OCCUPIED};

/// One return 2 m ahead: its voxel is occupied, the voxels between are free,
/// the voxels behind it are unknown, and repeats accumulate to the clamps.
#[test]
fn hits_misses_and_unknown_classify() {
    let mut m = map64();
    let o = [0.07, 0.07, 0.07];
    let p = [2.07, 0.07, 0.07];
    assert!(m.insert_cloud(SEC, o, &[p], 30.0));
    let hit = m.cell_of(p);
    assert_eq!(m.log_odds(hit), LO_HIT);
    assert_eq!(m.state(hit), VoxelState::Occupied);
    for x in 0..hit[0] {
        assert_eq!(m.log_odds([x, 0, 0]), LO_MISS, "x {x}");
        assert_eq!(m.state([x, 0, 0]), VoxelState::Free);
    }
    assert_eq!(m.state([hit[0] + 1, 0, 0]), VoxelState::Unknown);
    assert_eq!(m.state([0, 5, 0]), VoxelState::Unknown);

    for k in 0..10 {
        m.insert_cloud(SEC + k, o, &[p], 30.0);
    }
    assert_eq!(m.log_odds(hit), LO_MAX);
    assert_eq!(m.log_odds([3, 0, 0]), LO_MIN);
    assert_eq!(m.state[m.idx_of_cell(hit).unwrap()], VOX_OCCUPIED);
    assert_eq!(m.state[m.idx_of_cell([3, 0, 0]).unwrap()], VOX_FREE);
}

/// Many returns in one voxel and a ray through a hit voxel update each voxel
/// once per insertion, and the hit wins.
#[test]
fn one_update_per_voxel_per_insertion_and_hits_win() {
    let mut m = map64();
    let o = [0.07, 0.07, 0.07];
    let near = [1.07, 0.07, 0.07];
    let far = [2.07, 0.07, 0.07];
    m.insert_cloud(SEC, o, &[near, near, far, far, far], 30.0);
    assert_eq!(m.log_odds(m.cell_of(near)), LO_HIT);
    assert_eq!(m.log_odds(m.cell_of(far)), LO_HIT);
    assert_eq!(m.log_odds([2, 0, 0]), LO_MISS);
}

/// A return beyond the sensor's range clears its ray up to that range and
/// marks nothing; the sensor outside the window inserts nothing.
#[test]
fn out_of_range_returns_only_clear_and_outside_sensors_are_refused() {
    let mut m = map64();
    let o = [0.07, 0.07, 0.07];
    m.insert_cloud(SEC, o, &[[4.07, 0.07, 0.07]], 1.5);
    assert_eq!(m.state([9, 0, 0]), VoxelState::Free);
    assert_eq!(m.state([11, 0, 0]), VoxelState::Unknown);
    assert_eq!(m.state([27, 0, 0]), VoxelState::Unknown);
    assert!(!m.insert_cloud(SEC, [50.0, 0.0, 0.0], &[[51.0, 0.0, 0.0]], 30.0));
}

/// A ray to a point outside the window marks voxels up to the window's edge.
#[test]
fn rays_stop_at_the_window_edge() {
    let mut m = map64();
    m.insert_cloud(SEC, [0.07, 0.07, 0.07], &[[20.0, 0.07, 0.07]], 30.0);
    let last = m.origin_cell()[0] + 63;
    assert_eq!(m.state([last, 0, 0]), VoxelState::Free);
    assert_eq!(m.state([last - 5, 0, 0]), VoxelState::Free);
}

/// A wall 3 m ahead built from a fan of returns: a ray through it meets it
/// at its face, a ray short of it is clear, a ray into unseen space is not.
#[test]
fn ray_cast_meets_a_known_wall() {
    let mut m = map64();
    let o = [0.07, 0.07, 0.07];
    let mut wall = Vec::new();
    for j in -10..=10 {
        for k in -5..=5 {
            wall.push([3.07, j as f32 * 0.1 + 0.07, k as f32 * 0.1 + 0.07]);
        }
    }
    for t in 0..3 {
        m.insert_cloud(SEC + t, o, &wall, 30.0);
    }
    match m.ray(o, [4.5, 0.07, 0.07]) {
        RayHit::Occupied(p) => assert!((p[0] - 3.0).abs() < 1e-3, "{p:?}"),
        other => panic!("{other:?}"),
    }
    // Along the ray to the wall return (3.07, 0.37, 0.17), stopping short.
    assert_eq!(m.ray(o, [2.47, 0.31, 0.15]), RayHit::Clear);
    assert!(matches!(m.ray(o, [0.07, 3.0, 0.07]), RayHit::Unknown(_)));
    assert!(matches!(m.ray(o, [-20.0, 0.07, 0.07]), RayHit::Unknown(_)));
}

/// A planar scan on a vehicle pitched 30 degrees nose down places a forward
/// return 4 m out at its true height; a refused return changes nothing.
#[test]
fn planar_returns_land_at_their_true_height() {
    let mut m = map64();
    let half = (-30.0f32).to_radians() / 2.0;
    let pose = Pose {
        t: [0.07, 0.07, 0.07],
        q: [half.cos(), 0.0, half.sin(), 0.0],
    };
    let ranges = [4.0, 4.0];
    let angles = [0.0, std::f32::consts::FRAC_PI_2];
    let mut seen = Vec::new();
    m.insert_planar_scan(SEC, &pose, &ranges, &angles, 12.0, |r| {
        seen.push(r);
        r.index == 0
    });
    assert_eq!(seen.len(), 2);
    let p = seen[0].point;
    assert!((p[0] - (0.07 + 4.0 * 30f32.to_radians().cos())).abs() < 1e-4);
    assert!((p[2] - (0.07 + 2.0)).abs() < 1e-4, "{p:?}");
    assert_eq!(m.state_at(p), VoxelState::Occupied);
    // The level beam was refused: neither its voxel nor its ray changed.
    assert_eq!(m.state_at(seen[1].point), VoxelState::Unknown);
    assert_eq!(m.state_at([0.07, 2.0, 0.07]), VoxelState::Unknown);
    // Pose::rotate keeps lengths.
    let v = pose.rotate([1.0, 2.0, 3.0]);
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    assert!((len - 14f32.sqrt()).abs() < 1e-5);
}

/// A return just past a known surface voxel does not erode it: the voxels
/// touching a return only take misses while they hold no positive evidence.
#[test]
fn returns_just_behind_a_surface_do_not_erode_it() {
    let mut m = map64();
    let o = [0.07, 0.07, 0.07];
    let surface = [2.07, 0.07, 0.07];
    let behind = [2.22, 0.07, 0.07];
    m.insert_cloud(SEC, o, &[surface], 30.0);
    let c = m.cell_of(surface);
    for k in 0..5 {
        m.insert_cloud(SEC + 1 + k, o, &[behind], 30.0);
    }
    assert_eq!(m.log_odds(c), LO_HIT);
    assert_eq!(m.state(c), VoxelState::Occupied);
    // Two voxels behind is a real pass: three of them clear it.
    let beyond = [2.37, 0.07, 0.07];
    for k in 0..3 {
        m.insert_cloud(SEC + 10 + k, o, &[beyond], 30.0);
    }
    assert_eq!(m.state(c), VoxelState::Free);
}
