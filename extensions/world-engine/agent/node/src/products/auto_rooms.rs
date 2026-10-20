//! Rooms found without the operator marking any: morphological segmentation
//! of the free map (Bormann et al. 2016). The free map's distance transform is
//! thresholded at a growing erosion radius; a part that splits off with a
//! room-sized area becomes a seed, and the seeds grow back over the free map
//! by a watershed on the distance transform, so doorways become the borders.

use super::floorplan::{mm, FloorplanOptions, FloorplanRoom, FT2_PER_M2};
use super::geom::{signed_area, Plane};
use super::grid2d::{components, distance_to_blocked, neighbours4};
use super::rooms::{polygon_from_boundary, Grid};
use super::walls::WallLine;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Room-sized area of an eroded part, square metres.
pub(crate) const ROOM_AREA_M2: (f64, f64) = (0.8, 47.0);
/// Erosion step, metres.
const STEP_M: f64 = 0.05;
/// First erosion radius: a part narrower than a shoulder width is no room.
const FIRST_RADIUS_M: f64 = 0.25;

/// Rooms on a free mask.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Segmentation {
    /// Room label per cell (0 = no room), rooms numbered 1.. by area,
    /// largest first.
    pub labels: Vec<u32>,
    pub rooms: u32,
    /// Distance to the nearest blocked cell, metres.
    pub clearance_m: Vec<f64>,
}

/// Segments the free cells of a `w × h` grid of `cell` metres into rooms.
/// Free space connected to the grid border is outside the building and never
/// a room.
pub(crate) fn segment(free: &[bool], w: usize, h: usize, cell: f64) -> Segmentation {
    // Exterior: free components touching the border.
    let (comp, n) = components(free, w, h);
    let mut exterior = vec![false; n as usize + 1];
    for k in 0..w * h {
        let (i, j) = (k % w, k / w);
        if i == 0 || j == 0 || i + 1 == w || j + 1 == h {
            exterior[comp[k] as usize] = true;
        }
    }
    let inside: Vec<bool> = (0..w * h)
        .map(|k| free[k] && !exterior[comp[k] as usize])
        .collect();
    let clearance_m: Vec<f64> = distance_to_blocked(&inside, w, h)
        .into_iter()
        .map(|d| d * cell)
        .collect();
    let max_clear = clearance_m.iter().copied().fold(0.0, f64::max);

    // Erosion: room-sized parts become seeds and leave the map.
    let cell_area = cell * cell;
    let mut labels = vec![0u32; w * h];
    let mut active = inside.clone();
    let mut seeds = 0u32;
    let mut r = FIRST_RADIUS_M;
    while r < max_clear {
        let eroded: Vec<bool> = (0..w * h)
            .map(|k| active[k] && clearance_m[k] > r)
            .collect();
        let (parts, count) = components(&eroded, w, h);
        let mut sizes = vec![0usize; count as usize + 1];
        for &p in &parts {
            sizes[p as usize] += 1;
        }
        let mut seed_of = vec![0u32; count as usize + 1];
        for p in 1..=count as usize {
            let area = sizes[p] as f64 * cell_area;
            if (ROOM_AREA_M2.0..=ROOM_AREA_M2.1).contains(&area) {
                seeds += 1;
                seed_of[p] = seeds;
            }
        }
        for k in 0..w * h {
            let s = seed_of[parts[k] as usize];
            if s != 0 {
                labels[k] = s;
                active[k] = false;
            }
        }
        r += STEP_M;
    }

    // Watershed: seeds grow over the free map, the clearest cells first.
    let key = |k: usize| (clearance_m[k] * 1e4) as u64;
    let mut heap: BinaryHeap<(u64, Reverse<usize>)> = (0..w * h)
        .filter(|&k| labels[k] != 0)
        .map(|k| (key(k), Reverse(k)))
        .collect();
    while let Some((_, Reverse(k))) = heap.pop() {
        for n in neighbours4(k, w, h).into_iter().flatten() {
            if inside[n] && labels[n] == 0 {
                labels[n] = labels[k];
                heap.push((key(n), Reverse(n)));
            }
        }
    }

    // Number by area, largest first (ties by first cell).
    let mut area = vec![(0usize, usize::MAX); seeds as usize + 1];
    for (k, &l) in labels.iter().enumerate() {
        if l != 0 {
            area[l as usize].0 += 1;
            area[l as usize].1 = area[l as usize].1.min(k);
        }
    }
    let mut order: Vec<u32> = (1..=seeds).filter(|&l| area[l as usize].0 > 0).collect();
    order.sort_by_key(|&l| (Reverse(area[l as usize].0), area[l as usize].1));
    let mut rename = vec![0u32; seeds as usize + 1];
    for (i, &l) in order.iter().enumerate() {
        rename[l as usize] = i as u32 + 1;
    }
    for l in &mut labels {
        *l = rename[*l as usize];
    }
    Segmentation {
        labels,
        rooms: order.len() as u32,
        clearance_m,
    }
}

/// The rooms of a floor nobody marked rooms on, named `Room 1..n` by area
/// (largest first); the hotspot sits at the room's clearest cell, at eye
/// height.
pub(crate) fn rooms(
    grid: &Grid,
    lines: &[WallLine],
    plane: &Plane,
    opts: &FloorplanOptions,
) -> Vec<FloorplanRoom> {
    let seg = segment(&grid.free_mask(), grid.w, grid.h, grid.cell);
    let mut out = Vec::new();
    for label in 1..=seg.rooms {
        let Some(best) = (0..seg.labels.len())
            .filter(|&k| seg.labels[k] == label)
            .max_by(|&a, &b| seg.clearance_m[a].total_cmp(&seg.clearance_m[b]))
        else {
            continue;
        };
        let ring = grid.outer_boundary(&seg.labels, label);
        let polygon = polygon_from_boundary(&ring, lines, opts.simplify_m, 3.0 * opts.cell_m);
        if polygon.len() < 3 {
            continue;
        }
        let area = signed_area(&polygon).abs();
        let c = [
            grid.origin[0] + ((best % grid.w) as f64 + 0.5) * grid.cell,
            grid.origin[1] + ((best / grid.w) as f64 + 0.5) * grid.cell,
        ];
        out.push(FloorplanRoom {
            name: format!("Room {}", out.len() + 1),
            polygon: polygon.iter().map(|p| [mm(p[0]), mm(p[1])]).collect(),
            area_m2: mm(area),
            area_ft2: mm(area * FT2_PER_M2),
            hotspot: [c[0], c[1], plane.z_at(c[0], c[1]) + 1.6].map(mm),
        });
    }
    out
}
