//! 2-D floor plans from an indoor cloud and the operator's room-marked
//! trajectory: per-floor ground plane, a wall slice (body height, plus under
//! the ceiling when there is one) carved by the sensors' free-space rays,
//! RANSAC wall lines, door gaps, and room polygons from an occupancy-grid
//! flood fill (marked rooms) or a morphological segmentation (no marks).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::auto_rooms;
use super::carve::wall_mask;
use super::cloud::PointCloud;
use super::geom::{centroid, dedupe2d, point_in_polygon, ransac_horizontal_plane, Plane, Rng};
use super::rooms::{polygon_from_boundary, Grid};
use super::slice::{ceiling_height, SliceBands};
use super::walls::{
    complete_corners, doors, extract_lines, manhattan_snap, merge_collinear, WallParams,
};

pub use super::floorplan_svg::to_svg;

/// Square feet per square metre.
pub(super) const FT2_PER_M2: f64 = 10.763_910_416_709_722;

/// The operator's trajectory while inside one marked room.
#[derive(Debug, Clone, PartialEq)]
pub struct RoomTrack {
    pub name: String,
    pub floor: i32,
    pub positions: Vec<[f64; 3]>,
}

/// Tunables of [`build`].
#[derive(Debug, Clone, PartialEq)]
pub struct FloorplanOptions {
    /// Wall slice band, metres above the floor plane.
    pub slice_min_m: f64,
    pub slice_max_m: f64,
    /// Snap near-orthogonal walls onto the dominant frame.
    pub manhattan: bool,
    /// Collinear gap widths reported as doors.
    pub door_min_m: f64,
    pub door_max_m: f64,
    /// Occupancy grid cell size.
    pub cell_m: f64,
    /// Point-to-line inlier distance for wall extraction.
    pub wall_tol_m: f64,
    /// Shortest wall run reported.
    pub min_wall_len_m: f64,
    /// Douglas–Peucker tolerance of the room boundary.
    pub simplify_m: f64,
    /// RANSAC seed (results are deterministic per seed).
    pub seed: u64,
}

impl Default for FloorplanOptions {
    fn default() -> Self {
        Self {
            slice_min_m: 1.0,
            slice_max_m: 1.5,
            manhattan: true,
            door_min_m: 0.7,
            door_max_m: 1.2,
            cell_m: 0.05,
            wall_tol_m: 0.03,
            min_wall_len_m: 0.4,
            simplify_m: 0.08,
            seed: 7,
        }
    }
}

/// `floorplan.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Floorplan {
    pub contract: String,
    pub units: String,
    pub frame: String,
    pub floors: Vec<FloorLevel>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FloorLevel {
    pub floor: i32,
    pub z_m: f64,
    /// `[x0, y0, x1, y1]`.
    pub walls: Vec<[f64; 4]>,
    pub doors: Vec<[f64; 4]>,
    pub rooms: Vec<FloorplanRoom>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FloorplanRoom {
    pub name: String,
    pub polygon: Vec<[f64; 2]>,
    pub area_m2: f64,
    pub area_ft2: f64,
    pub hotspot: [f64; 3],
}

/// Rounds to millimetres (and folds `-0.0` into `0.0`).
pub(super) fn mm(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0 + 0.0
}

fn median(mut v: Vec<f64>) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    v.sort_unstable_by(f64::total_cmp);
    Some(v[v.len() / 2])
}

/// Ground plane of one floor. With a trajectory, the search band sits 0.5–2.3 m
/// below its median height (body height above the floor); without one, around
/// the densest z layer of the lower half of the cloud.
fn floor_plane(cloud: &PointCloud, track_z: Option<f64>, rng: &mut Rng) -> Option<Plane> {
    let (lo, hi) = match track_z {
        Some(z) => (z - 2.3, z - 0.5),
        None => {
            let (min, max) = cloud.bounds()?;
            let bins = (((max[2] - min[2]) / 0.05).ceil() as usize).max(1);
            let mut hist = vec![0usize; bins + 1];
            for p in &cloud.points {
                hist[((p[2] - min[2]) / 0.05) as usize] += 1;
            }
            let half = hist.len().div_ceil(2);
            let peak = (0..half).max_by_key(|&b| hist[b])?;
            let z = min[2] + (peak as f64 + 0.5) * 0.05;
            (z - 0.3, z + 0.3)
        }
    };
    let cand: Vec<[f64; 3]> = cloud
        .points
        .iter()
        .copied()
        .filter(|p| p[2] >= lo && p[2] <= hi)
        .collect();
    // Plane RANSAC on a bounded sample keeps large clouds cheap.
    let stride = (cand.len() / 60_000).max(1);
    let sample: Vec<[f64; 3]> = cand.iter().step_by(stride).copied().collect();
    let (plane, inliers) = ransac_horizontal_plane(&sample, 0.03, 200, rng)?;
    (inliers >= 50).then_some(plane)
}

/// Builds the floor plan. Floors are the distinct `RoomTrack::floor` values
/// (one floor 0 without tracks). Marked rooms are reported by name; a floor
/// without marked rooms gets its rooms found automatically. `origins` are the
/// sensor positions (keyframe camera centres, LiDAR scan origins) whose rays
/// carve free space out of the wall slice.
pub fn build(
    cloud: &PointCloud,
    rooms: &[RoomTrack],
    origins: &[[f64; 3]],
    opts: &FloorplanOptions,
) -> Floorplan {
    let mut by_floor: BTreeMap<i32, Vec<&RoomTrack>> = BTreeMap::new();
    for r in rooms {
        by_floor.entry(r.floor).or_default().push(r);
    }
    if by_floor.is_empty() {
        by_floor.insert(0, Vec::new());
    }
    let track_z: BTreeMap<i32, Option<f64>> = by_floor
        .iter()
        .map(|(f, tracks)| (*f, track_median_z(tracks)))
        .collect();
    let mut rng = Rng::new(opts.seed);
    let floors = by_floor
        .iter()
        .filter_map(|(&floor, tracks)| {
            // The next floor up bounds this floor's ceiling search: just under
            // its walking surface (body height 1.5 m below its track).
            let above = track_z
                .range(floor + 1..)
                .find_map(|(_, z)| *z)
                .map(|z| z - 1.5 - 0.15);
            let ctx = FloorCtx {
                floor,
                tracks,
                origins,
                ceiling_limit_z: above,
            };
            build_floor(cloud, &ctx, opts, &mut rng)
        })
        .collect();
    Floorplan {
        contract: ados_capture_contract::CONTRACT_ID.to_string(),
        units: "m".to_string(),
        frame: "enu_local".to_string(),
        floors,
    }
}

fn track_median_z(tracks: &[&RoomTrack]) -> Option<f64> {
    median(
        tracks
            .iter()
            .flat_map(|t| t.positions.iter().map(|p| p[2]))
            .collect(),
    )
}

/// What one floor is built from besides the cloud.
struct FloorCtx<'a> {
    floor: i32,
    tracks: &'a [&'a RoomTrack],
    origins: &'a [[f64; 3]],
    /// Absolute height the ceiling search stays below.
    ceiling_limit_z: Option<f64>,
}

fn build_floor(
    cloud: &PointCloud,
    ctx: &FloorCtx<'_>,
    opts: &FloorplanOptions,
    rng: &mut Rng,
) -> Option<FloorLevel> {
    let (floor, tracks) = (ctx.floor, ctx.tracks);
    let track_z = track_median_z(tracks);
    let plane = match floor_plane(cloud, track_z, rng) {
        Some(p) => p,
        None => {
            tracing::warn!(floor, "no ground plane found; assuming 1.5 m body height");
            Plane::horizontal(track_z? - 1.5)
        }
    };
    let ceiling = ceiling_height(
        cloud.points.iter().map(|p| plane.height(*p)),
        ctx.ceiling_limit_z.map(|z| z - plane.z_at(0.0, 0.0)),
    );
    let bands = SliceBands::new((opts.slice_min_m, opts.slice_max_m), ceiling);
    let (mut slice, mut low, mut ground) = (Vec::new(), Vec::new(), Vec::new());
    for p in &cloud.points {
        let h = plane.height(*p);
        let xy = [p[0], p[1]];
        if bands.contains(h) {
            slice.push(xy);
        } else if (0.2..=0.9).contains(&h) {
            low.push(xy);
        } else if h.abs() < 0.05 {
            ground.push(xy);
        }
    }
    let origins: Vec<[f64; 2]> = ctx
        .origins
        .iter()
        .filter(|o| (0.3..=2.5).contains(&plane.height(**o)))
        .map(|o| [o[0], o[1]])
        .collect();
    let keep = wall_mask(&slice, &origins);
    let slice = dedupe2d(
        slice
            .into_iter()
            .zip(keep)
            .filter_map(|(p, k)| k.then_some(p)),
        0.02,
    );
    let low = dedupe2d(low.into_iter(), 0.02);
    let ground = dedupe2d(ground.into_iter(), opts.cell_m);

    let prm = WallParams::new(opts.wall_tol_m, opts.min_wall_len_m);
    let mut lines = extract_lines(&slice, &prm, rng);
    lines = merge_collinear(lines, 3f64.to_radians(), 0.04, &prm);
    if opts.manhattan {
        manhattan_snap(&mut lines, 10f64.to_radians(), &prm);
        lines = merge_collinear(lines, 0.5f64.to_radians(), 0.04, &prm);
    }
    complete_corners(&mut lines, 0.3);
    let door_segs = doors(
        &lines,
        &low,
        opts.door_min_m,
        opts.door_max_m,
        opts.wall_tol_m,
    );

    let centre = [
        tracks
            .iter()
            .flat_map(|t| &t.positions)
            .map(|p| p[0])
            .sum::<f64>(),
        tracks
            .iter()
            .flat_map(|t| &t.positions)
            .map(|p| p[1])
            .sum::<f64>(),
    ];
    let n_pos = tracks.iter().map(|t| t.positions.len()).sum::<usize>();
    let z_m = if n_pos > 0 {
        plane.z_at(centre[0] / n_pos as f64, centre[1] / n_pos as f64)
    } else {
        let k = ground.len().max(1) as f64;
        let c = ground
            .iter()
            .fold([0.0, 0.0], |s, p| [s[0] + p[0] / k, s[1] + p[1] / k]);
        plane.z_at(c[0], c[1])
    };

    let walls = lines
        .iter()
        .flat_map(|l| {
            l.runs
                .iter()
                .filter(|r| r[1] - r[0] >= opts.min_wall_len_m)
                .map(|r| {
                    let (a, b) = (l.at(r[0]), l.at(r[1]));
                    [mm(a[0]), mm(a[1]), mm(b[0]), mm(b[1])]
                })
        })
        .collect();
    let doors_out = door_segs
        .iter()
        .map(|d| [mm(d[0][0]), mm(d[0][1]), mm(d[1][0]), mm(d[1][1])])
        .collect();
    let rooms = room_polygons(&slice, &ground, &lines, tracks, &plane, opts);
    Some(FloorLevel {
        floor,
        z_m: mm(z_m),
        walls,
        doors: doors_out,
        rooms,
    })
}

fn room_polygons(
    slice: &[[f64; 2]],
    ground: &[[f64; 2]],
    lines: &[super::walls::WallLine],
    tracks: &[&RoomTrack],
    plane: &Plane,
    opts: &FloorplanOptions,
) -> Vec<FloorplanRoom> {
    let positions = tracks
        .iter()
        .flat_map(|t| t.positions.iter().map(|p| [p[0], p[1]]));
    let all: Vec<[f64; 2]> = slice.iter().copied().chain(positions).collect();
    let Some(first) = all.first() else {
        return Vec::new();
    };
    let (mut min, mut max) = (*first, *first);
    for p in &all {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    }
    let margin = 0.5;
    let mut grid = Grid::new(
        [min[0] - margin, min[1] - margin],
        [max[0] + margin, max[1] + margin],
        opts.cell_m,
    );
    for l in lines {
        // Runs extend a little so corners close; gaps up to door width close
        // so rooms separate at their doorways.
        let ext = 2.0 * opts.cell_m;
        for r in &l.runs {
            grid.raster_segment(l.at(r[0] - ext), l.at(r[1] + ext));
        }
        for w in l.runs.windows(2) {
            if w[1][0] - w[0][1] <= opts.door_max_m {
                grid.raster_segment(l.at(w[0][1]), l.at(w[1][0]));
            }
        }
    }
    if ground.len() >= 100 {
        grid.set_walkable(ground, 3);
    }
    if tracks.is_empty() {
        return auto_rooms::rooms(&grid, lines, plane, opts);
    }
    let labels = grid.label();

    let mut names: Vec<&str> = Vec::new();
    for t in tracks {
        if !names.contains(&t.name.as_str()) {
            names.push(&t.name);
        }
    }
    let mut out = Vec::new();
    for name in names {
        let pos: Vec<[f64; 3]> = tracks
            .iter()
            .filter(|t| t.name == name)
            .flat_map(|t| t.positions.iter().copied())
            .collect();
        // The component holding most of the room's track wins; positions in
        // a doorway or on a wall cell do not pull in the neighbour.
        let mut votes: BTreeMap<u32, usize> = BTreeMap::new();
        for p in &pos {
            if let Some(l) = grid.label_at(&labels, [p[0], p[1]], 4) {
                *votes.entry(l).or_default() += 1;
            }
        }
        let Some((&label, _)) = votes.iter().max_by_key(|(_, v)| **v) else {
            tracing::warn!(room = name, "room track has no free cell; room skipped");
            continue;
        };
        let ring = grid.outer_boundary(&labels, label);
        let polygon = polygon_from_boundary(&ring, lines, opts.simplify_m, 3.0 * opts.cell_m);
        if polygon.len() < 3 {
            continue;
        }
        let area = super::geom::signed_area(&polygon).abs();
        let k = pos.len() as f64;
        let mean = pos.iter().fold([0.0; 3], |s, p| {
            [s[0] + p[0] / k, s[1] + p[1] / k, s[2] + p[2] / k]
        });
        let hotspot = if point_in_polygon([mean[0], mean[1]], &polygon) {
            mean
        } else {
            let c = centroid(&polygon);
            [c[0], c[1], plane.z_at(c[0], c[1]) + 1.6]
        };
        out.push(FloorplanRoom {
            name: name.to_string(),
            polygon: polygon.iter().map(|p| [mm(p[0]), mm(p[1])]).collect(),
            area_m2: mm(area),
            area_ft2: mm(area * FT2_PER_M2),
            hotspot: hotspot.map(mm),
        });
    }
    out
}
