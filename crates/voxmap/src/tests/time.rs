use super::{map64, SEC};
use crate::{VoxelState, LO_FREE, LO_HIT, LO_MISS};

const O: [f32; 3] = [0.07, 0.07, 0.07];
const P: [f32; 3] = [2.07, 0.07, 0.07];
const BEYOND: [f32; 3] = [3.07, 0.07, 0.07];

/// Positive evidence holds for the grace time, then falls 0.2 log-odds
/// (4 units) per second to unknown and stops at zero; a new hit restarts it.
#[test]
fn unseen_obstacles_decay_to_unknown_after_the_grace() {
    let mut m = map64();
    for k in 0..4 {
        m.insert_cloud(k, O, &[P], 30.0);
    }
    let c = m.cell_of(P);
    assert_eq!(m.log_odds(c), 4 * LO_HIT);
    let free = [3, 0, 0];
    assert_eq!(m.log_odds(free), 4 * LO_MISS);

    let at = |m: &mut crate::VoxMap, t_ds: u64| {
        m.decay(t_ds * SEC / 10);
    };
    for t in 0..=99 {
        at(&mut m, t);
    }
    assert_eq!(m.log_odds(c), 4 * LO_HIT, "inside the grace");
    for t in 100..=120 {
        at(&mut m, t);
    }
    // Decay runs in quarter seconds from 9.75 s, the last quarter the grace
    // covered: 2.25 s by 12 s, 9 units.
    assert_eq!(m.log_odds(c), 4 * LO_HIT - 9);
    assert_eq!(m.state(c), VoxelState::Occupied);
    for t in 121..=220 {
        at(&mut m, t);
    }
    assert_eq!(m.log_odds(c), 4 * LO_HIT - 49);
    assert_eq!(m.state(c), VoxelState::Occupied);
    for t in 221..=230 {
        at(&mut m, t);
    }
    assert_eq!(m.state(c), VoxelState::Unknown, "below the occupied level");
    for t in 231..=400 {
        at(&mut m, t);
    }
    assert_eq!(m.log_odds(c), 0);
    // Free evidence does not decay.
    assert_eq!(m.log_odds(free), 4 * LO_MISS);

    // A hit restarts the grace.
    m.insert_cloud(40 * SEC, O, &[P], 30.0);
    for t in 401..=490 {
        at(&mut m, t);
    }
    assert_eq!(m.log_odds(c), LO_HIT);
}

/// An obstacle that rays pass through on three insertions in a row is
/// cleared to free at the third, however confident it was; a hit in between
/// restarts the count.
#[test]
fn three_passes_in_a_row_clear_a_moved_obstacle() {
    let mut m = map64();
    for k in 0..6 {
        m.insert_cloud(k, O, &[P], 30.0);
    }
    let c = m.cell_of(P);
    let full = m.log_odds(c);
    m.insert_cloud(10, O, &[BEYOND], 30.0);
    m.insert_cloud(11, O, &[BEYOND], 30.0);
    assert_eq!(m.log_odds(c), full + 2 * LO_MISS);
    // Seen again: the count restarts.
    m.insert_cloud(12, O, &[P], 30.0);
    m.insert_cloud(13, O, &[BEYOND], 30.0);
    m.insert_cloud(14, O, &[BEYOND], 30.0);
    assert_eq!(m.state(c), VoxelState::Occupied);
    m.insert_cloud(15, O, &[BEYOND], 30.0);
    assert_eq!(m.log_odds(c), LO_FREE);
    assert_eq!(m.state(c), VoxelState::Free);
    // Its distance-field entry went with it.
    assert_eq!(m.state(m.cell_of(BEYOND)), VoxelState::Occupied);
    m.update_esdf();
    m.check_esdf(0.5);
}
