//! The live 2D occupancy grid in `map_ned`, held as 64×64-cell tiles at 5 cm.
//!
//! Each scan return is ray-traced from the sensor origin: the cells it
//! crosses lose evidence (free), the end cell gains it (occupied). A cell
//! holds clamped log-odds in tenths as `i8`; `0` means never observed, so an
//! observed cell that sums back to zero is stored as `±1` and stays known.
//!
//! Tiles farther than [`EVICT_RADIUS_M`] from the vehicle are written to disk
//! and dropped from RAM; past the occupancy share of the memory budget the
//! farthest tiles go first, whatever their distance.

use std::collections::{HashMap, HashSet};
use std::io::Cursor;
use std::path::Path;

use base64::Engine as _;
use image::codecs::png::PngEncoder;
use image::{ExtendedColorType, ImageEncoder};
use world_engine_protocol::atlas::nav::{
    OCCUPANCY_FREE, OCCUPANCY_OCCUPIED, OCCUPANCY_TILE_CELLS, OCCUPANCY_UNKNOWN,
};
use world_engine_protocol::atlas::OccupancyTile;

/// Cell edge, metres.
pub const RES_M: f32 = 0.05;
/// Tiles farther than this from the vehicle leave RAM.
pub const EVICT_RADIUS_M: f32 = 200.0;
const CELLS: i32 = OCCUPANCY_TILE_CELLS as i32;
const TILE_LEN: usize = (CELLS * CELLS) as usize;
/// Evidence a hit adds and a pass-through removes, tenths of log-odds.
const HIT: i16 = 9;
const MISS: i16 = -4;
const CLAMP: i16 = 100;
/// Above this the cell is drawn occupied; below `FREE_BELOW` free.
const OCCUPIED_ABOVE: i8 = 20;
const FREE_BELOW: i8 = -10;
/// RAM one tile costs (cells plus map overhead).
pub const TILE_BYTES: usize = TILE_LEN + 64;

struct Tile {
    cells: Box<[i8; TILE_LEN]>,
    known: u32,
}

impl Tile {
    fn new() -> Self {
        Self {
            cells: Box::new([0; TILE_LEN]),
            known: 0,
        }
    }
}

/// The live grid.
#[derive(Default)]
pub struct OccupancyGrid {
    tiles: HashMap<(i32, i32), Tile>,
    changed: HashSet<(i32, i32)>,
    /// Known cells in tiles already evicted to disk.
    evicted_known: u64,
}

/// The tile and in-tile cell of world cell `(cx, cy)`.
fn split(cx: i32, cy: i32) -> ((i32, i32), usize) {
    let (tx, ty) = (cx.div_euclid(CELLS), cy.div_euclid(CELLS));
    let (lx, ly) = (cx.rem_euclid(CELLS), cy.rem_euclid(CELLS));
    ((tx, ty), (lx * CELLS + ly) as usize)
}

fn cell_of(v: f32) -> i32 {
    (v / RES_M).floor() as i32
}

impl OccupancyGrid {
    pub fn new() -> Self {
        Self::default()
    }

    fn bump(&mut self, cx: i32, cy: i32, delta: i16) {
        let (key, i) = split(cx, cy);
        let tile = self.tiles.entry(key).or_insert_with(Tile::new);
        let old = tile.cells[i];
        let mut v = (old as i16 + delta).clamp(-CLAMP, CLAMP) as i8;
        if v == 0 {
            v = delta.signum() as i8;
        }
        if old == 0 {
            tile.known += 1;
        }
        if v != old {
            tile.cells[i] = v;
            self.changed.insert(key);
        }
    }

    /// Ray-trace one return from `origin` to `hit` (map_ned x, y, metres).
    pub fn insert_ray(&mut self, origin: [f32; 2], hit: [f32; 2]) {
        let (x0, y0) = (cell_of(origin[0]), cell_of(origin[1]));
        let (x1, y1) = (cell_of(hit[0]), cell_of(hit[1]));
        // Bresenham over the cells strictly before the hit.
        let (dx, dy) = ((x1 - x0).abs(), -(y1 - y0).abs());
        let (sx, sy) = (if x0 < x1 { 1 } else { -1 }, if y0 < y1 { 1 } else { -1 });
        let (mut x, mut y, mut err) = (x0, y0, dx + dy);
        while (x, y) != (x1, y1) {
            self.bump(x, y, MISS);
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
        self.bump(x1, y1, HIT);
    }

    /// The drawn value of world point `(x, y)`.
    pub fn value_at(&self, x: f32, y: f32) -> u8 {
        let (key, i) = split(cell_of(x), cell_of(y));
        self.tiles
            .get(&key)
            .map_or(OCCUPANCY_UNKNOWN, |t| draw(t.cells[i]))
    }

    /// Tiles held in RAM.
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// RAM the tiles hold, bytes.
    pub fn bytes(&self) -> usize {
        self.tiles.len() * TILE_BYTES
    }

    /// Observed area, m², evicted tiles included.
    pub fn area_m2(&self) -> f32 {
        let known: u64 = self.tiles.values().map(|t| t.known as u64).sum::<u64>() + self.evicted_known;
        known as f32 * RES_M * RES_M
    }

    /// The tiles changed since the last call, as PNG.
    pub fn take_changed(&mut self) -> Vec<OccupancyTile> {
        let mut keys: Vec<_> = self.changed.drain().collect();
        keys.sort_unstable();
        keys.into_iter()
            .filter_map(|k| {
                let t = self.tiles.get(&k)?;
                Some(OccupancyTile {
                    ix: k.0,
                    iy: k.1,
                    png_b64: base64::engine::general_purpose::STANDARD.encode(tile_png(t)),
                })
            })
            .collect()
    }

    /// Drop tiles farther than [`EVICT_RADIUS_M`] from `vehicle`, then the
    /// farthest until the tiles fit `cap_bytes`, writing each to
    /// `dir/<ix>_<iy>.png` first when `dir` is given. Returns how many went.
    pub fn evict(&mut self, vehicle: [f32; 2], cap_bytes: usize, dir: Option<&Path>) -> usize {
        let tile_m = CELLS as f32 * RES_M;
        let dist = |k: &(i32, i32)| {
            let cx = (k.0 as f32 + 0.5) * tile_m - vehicle[0];
            let cy = (k.1 as f32 + 0.5) * tile_m - vehicle[1];
            cx.hypot(cy)
        };
        let mut keys: Vec<((i32, i32), f32)> = self.tiles.keys().map(|k| (*k, dist(k))).collect();
        keys.sort_by(|a, b| b.1.total_cmp(&a.1));
        let max_tiles = cap_bytes / TILE_BYTES;
        let mut n = 0;
        for (k, d) in keys {
            if d <= EVICT_RADIUS_M && self.tiles.len() <= max_tiles {
                break;
            }
            let Some(t) = self.tiles.remove(&k) else { continue };
            self.changed.remove(&k);
            self.evicted_known += t.known as u64;
            if let Some(dir) = dir {
                write_tile(dir, k, &t);
            }
            n += 1;
        }
        n
    }

    /// Write every tile in RAM to `dir` (a session closing).
    pub fn save_all(&self, dir: &Path) {
        for (k, t) in &self.tiles {
            write_tile(dir, *k, t);
        }
    }
}

fn write_tile(dir: &Path, k: (i32, i32), t: &Tile) {
    let path = dir.join(format!("{}_{}.png", k.0, k.1));
    let r = std::fs::create_dir_all(dir).and_then(|()| std::fs::write(&path, tile_png(t)));
    if let Err(e) = r {
        tracing::warn!(path = %path.display(), error = %e, "occupancy_tile_write_failed");
    }
}

fn draw(v: i8) -> u8 {
    match v {
        0 => OCCUPANCY_UNKNOWN,
        v if v > OCCUPIED_ABOVE => OCCUPANCY_OCCUPIED,
        v if v < FREE_BELOW => OCCUPANCY_FREE,
        _ => OCCUPANCY_UNKNOWN,
    }
}

/// The tile as 8-bit grey PNG: row 0 is the largest x (north up), column 0
/// the smallest y (west left).
fn tile_png(t: &Tile) -> Vec<u8> {
    let mut px = Vec::with_capacity(TILE_LEN);
    for row in 0..CELLS {
        let lx = CELLS - 1 - row;
        for ly in 0..CELLS {
            px.push(draw(t.cells[(lx * CELLS + ly) as usize]));
        }
    }
    let mut out = Cursor::new(Vec::new());
    PngEncoder::new(&mut out)
        .write_image(&px, CELLS as u32, CELLS as u32, ExtendedColorType::L8)
        .expect("an in-memory PNG encode cannot fail");
    out.into_inner()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ray_frees_its_path_and_occupies_its_end() {
        let mut g = OccupancyGrid::new();
        for _ in 0..5 {
            g.insert_ray([0.01, 0.01], [2.01, 0.01]);
        }
        assert_eq!(g.value_at(2.01, 0.01), OCCUPANCY_OCCUPIED);
        assert_eq!(g.value_at(1.0, 0.01), OCCUPANCY_FREE);
        assert_eq!(g.value_at(0.0, 3.0), OCCUPANCY_UNKNOWN);
        assert!((g.area_m2() - 41.0 * RES_M * RES_M).abs() < 1e-4);
    }

    #[test]
    fn changed_tiles_come_out_once_as_png_with_north_up() {
        let mut g = OccupancyGrid::new();
        for _ in 0..5 {
            g.insert_ray([0.01, 0.01], [3.16, 0.01]);
        }
        let tiles = g.take_changed();
        assert_eq!(tiles.iter().map(|t| (t.ix, t.iy)).collect::<Vec<_>>(), vec![(0, 0)]);
        assert!(g.take_changed().is_empty(), "unchanged tiles are not re-sent");
        let png = base64::engine::general_purpose::STANDARD
            .decode(&tiles[0].png_b64)
            .unwrap();
        let img = image::load_from_memory(&png).unwrap().to_luma8();
        assert_eq!(img.dimensions(), (64, 64));
        // x = 3.16 m is cell 63, the top row; y = 0 is column 0.
        assert_eq!(img.get_pixel(0, 0).0[0], OCCUPANCY_OCCUPIED);
        assert_eq!(img.get_pixel(0, 63).0[0], OCCUPANCY_FREE);
    }

    #[test]
    fn far_tiles_leave_ram_for_disk_and_a_tight_cap_evicts_the_farthest() {
        let dir = tempfile::tempdir().unwrap();
        let mut g = OccupancyGrid::new();
        g.insert_ray([0.0, 0.0], [1.0, 0.0]);
        g.insert_ray([300.0, 0.0], [300.5, 0.0]);
        let before = g.area_m2();
        assert_eq!(g.evict([0.0, 0.0], usize::MAX, Some(dir.path())), 1);
        assert_eq!(g.tile_count(), 1);
        assert!(dir.path().join("93_0.png").exists());
        assert!((g.area_m2() - before).abs() < 1e-4, "evicted area still counts");

        g.insert_ray([0.0, 0.0], [0.0, 5.0]);
        let n = g.tile_count();
        assert!(n >= 2);
        g.evict([0.0, 0.0], TILE_BYTES, None);
        assert_eq!(g.tile_count(), 1);
        let left: Vec<_> = g.take_changed().iter().map(|t| (t.ix, t.iy)).collect();
        assert_eq!(left, vec![(0, 0)], "the nearest tile stays");
    }
}
