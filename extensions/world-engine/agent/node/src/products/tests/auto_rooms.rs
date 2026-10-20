//! Room segmentation of a free map: two rooms joined by an open doorway.

use super::super::auto_rooms::segment;

const CELL: f64 = 0.05;

/// Cells of a free map: room A is 8 × 6 m (x 0..8), room B 6 × 6 m (x 8.05..
/// 14.05), a one-cell wall between them with a 0.9 m open doorway, a one-cell
/// outer wall, and two cells of free space outside it (the exterior).
fn plan() -> (Vec<bool>, usize, usize) {
    let (a, b, h_in) = (160usize, 120usize, 120usize);
    let margin = 2;
    let w = margin + 1 + a + 1 + b + 1 + margin;
    let h = margin + 1 + h_in + 1 + margin;
    let mut free = vec![true; w * h];
    let wall_x = [margin, margin + 1 + a, margin + 1 + a + 1 + b];
    for j in margin..h - margin {
        for &x in &wall_x {
            free[j * w + x] = false;
        }
    }
    for i in margin..w - margin {
        free[margin * w + i] = false;
        free[(h - margin - 1) * w + i] = false;
    }
    // The doorway: 18 cells (0.9 m) open in the middle wall.
    for j in 0..18 {
        free[(margin + 1 + 51 + j) * w + wall_x[1]] = true;
    }
    (free, w, h)
}

#[test]
fn auto_rooms_split_two_rooms_by_a_door() {
    let (free, w, h) = plan();
    let seg = segment(&free, w, h, CELL);
    assert_eq!(seg.rooms, 2);
    let label_at = |x: f64, y: f64| {
        let (i, j) = (3 + (x / CELL) as usize, 3 + (y / CELL) as usize);
        seg.labels[j * w + i]
    };
    // Largest first: A is room 1, B room 2, each whole.
    assert_eq!(label_at(4.0, 3.0), 1);
    assert_eq!(label_at(0.1, 0.1), 1);
    assert_eq!(label_at(7.9, 5.9), 1);
    assert_eq!(label_at(11.0, 3.0), 2);
    assert_eq!(label_at(13.9, 0.1), 2);
    let area = |l: u32| seg.labels.iter().filter(|&&v| v == l).count() as f64 * CELL * CELL;
    assert!((area(1) - 48.0).abs() < 0.5, "A {}", area(1));
    assert!((area(2) - 36.0).abs() < 0.5, "B {}", area(2));
    // The exterior is never a room.
    assert_eq!(seg.labels[0], 0);
    assert_eq!(seg.labels[w * h - 1], 0);
}

#[test]
fn a_hall_too_small_to_hold_a_shoulder_width_is_no_room() {
    // A 0.4 m wide, 10 m long closed strip.
    let (w, h) = (204usize, 12usize);
    let mut free = vec![false; w * h];
    for j in 2..10 {
        for i in 2..202 {
            free[j * w + i] = true;
        }
    }
    assert_eq!(segment(&free, w, h, CELL).rooms, 0);
}
