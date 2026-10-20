//! Occupancy grid with log-odds updates, and the max-pooled pyramid the
//! branch-and-bound search bounds its candidates with.

use crate::config::Scan2dConfig;

/// Cells added beyond the needed extent when a grid grows.
const GROW_MARGIN: i64 = 32;

#[derive(Debug, Clone, Copy, Default)]
struct Cell {
    logodds: f32,
    /// Id of the last scan that updated the cell (0: never observed).
    stamp: u32,
}

/// Occupancy log-odds on a square lattice that grows as scans arrive.
/// Cell `(ix, iy)` covers `origin + [ix, ix+1)·res × [iy, iy+1)·res`.
///
/// Matching reads a cell's *value*: its occupancy probability when the cell
/// is more likely occupied than free, and `p_min` otherwise (free, unknown
/// and outside the grid alike). A young submap's free cells sit just below
/// 0.5 after a few misses while unknown cells are far lower; scoring them
/// apart would pull scans off the walls into observed free space, so only
/// occupied evidence scores.
#[derive(Debug, Clone)]
pub struct Grid {
    res: f64,
    origin: [f64; 2],
    w: i64,
    h: i64,
    cells: Vec<Cell>,
    value: Vec<f32>,
    scan_id: u32,
    l_hit: f32,
    l_miss: f32,
    l_min: f32,
    l_max: f32,
    p_min: f32,
    p_max: f32,
}

fn logit(p: f64) -> f32 {
    (p / (1.0 - p)).ln() as f32
}

impl Grid {
    /// An empty grid with the configuration's resolution and update model.
    pub fn new(cfg: &Scan2dConfig) -> Grid {
        Grid {
            res: cfg.resolution_m,
            origin: [0.0, 0.0],
            w: 0,
            h: 0,
            cells: Vec::new(),
            value: Vec::new(),
            scan_id: 0,
            l_hit: logit(cfg.hit_p),
            l_miss: logit(cfg.miss_p),
            l_min: logit(cfg.p_min),
            l_max: logit(cfg.p_max),
            p_min: cfg.p_min as f32,
            p_max: cfg.p_max as f32,
        }
    }

    pub fn resolution(&self) -> f64 {
        self.res
    }

    /// Value of free, unknown and out-of-grid cells.
    pub fn p_min(&self) -> f32 {
        self.p_min
    }

    /// The cell holding grid-frame point `p`.
    pub fn cell_of(&self, p: [f64; 2]) -> (i64, i64) {
        (
            ((p[0] - self.origin[0]) / self.res).floor() as i64,
            ((p[1] - self.origin[1]) / self.res).floor() as i64,
        )
    }

    fn index(&self, ix: i64, iy: i64) -> Option<usize> {
        (ix >= 0 && iy >= 0 && ix < self.w && iy < self.h).then(|| (iy * self.w + ix) as usize)
    }

    /// Matching value of cell `(ix, iy)`.
    pub fn value(&self, ix: i64, iy: i64) -> f32 {
        self.index(ix, iy).map_or(self.p_min, |k| self.value[k])
    }

    /// Row `iy` of matching values, when it exists.
    pub fn row(&self, iy: i64) -> Option<&[f32]> {
        (iy >= 0 && iy < self.h).then(|| {
            let s = (iy * self.w) as usize;
            &self.value[s..s + self.w as usize]
        })
    }

    /// Bilinear matching value at grid-frame point `p`, interpolating cell
    /// centres, with its gradient per metre.
    pub fn interpolate(&self, p: [f64; 2]) -> (f64, [f64; 2]) {
        let u = (p[0] - self.origin[0]) / self.res - 0.5;
        let v = (p[1] - self.origin[1]) / self.res - 0.5;
        let (i, j) = (u.floor(), v.floor());
        let (fx, fy) = (u - i, v - j);
        let (i, j) = (i as i64, j as i64);
        let v00 = self.value(i, j) as f64;
        let v10 = self.value(i + 1, j) as f64;
        let v01 = self.value(i, j + 1) as f64;
        let v11 = self.value(i + 1, j + 1) as f64;
        let m = (1.0 - fy) * ((1.0 - fx) * v00 + fx * v10) + fy * ((1.0 - fx) * v01 + fx * v11);
        let gx = ((1.0 - fy) * (v10 - v00) + fy * (v11 - v01)) / self.res;
        let gy = ((1.0 - fx) * (v01 - v00) + fx * (v11 - v10)) / self.res;
        (m, [gx, gy])
    }

    /// Fold one scan in: every endpoint cell is a hit, every cell its ray
    /// crosses before the endpoint a miss, each cell updated once per scan
    /// (a hit wins over a miss). `sensor` and `points` are grid-frame metres.
    pub fn insert(&mut self, sensor: [f64; 2], points: &[[f64; 2]]) {
        if points.is_empty() {
            return;
        }
        let mut lo = [sensor[0], sensor[1]];
        let mut hi = lo;
        for p in points {
            lo = [lo[0].min(p[0]), lo[1].min(p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(p[1])];
        }
        self.ensure(lo, hi);
        self.scan_id = self.scan_id.wrapping_add(1).max(1);
        let id = self.scan_id;
        let ends: Vec<(i64, i64)> = points.iter().map(|p| self.cell_of(*p)).collect();
        for &(ix, iy) in &ends {
            self.update(ix, iy, id, true);
        }
        let (sx, sy) = self.cell_of(sensor);
        for &(ex, ey) in &ends {
            self.trace_misses(sx, sy, ex, ey, id);
        }
    }

    fn trace_misses(&mut self, x0: i64, y0: i64, x1: i64, y1: i64, id: u32) {
        // Bresenham from the sensor cell up to, not including, the endpoint.
        let (dx, dy) = ((x1 - x0).abs(), -(y1 - y0).abs());
        let (stx, sty) = ((x1 - x0).signum(), (y1 - y0).signum());
        let (mut x, mut y, mut err) = (x0, y0, dx + dy);
        while (x, y) != (x1, y1) {
            self.update(x, y, id, false);
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += stx;
            }
            if e2 <= dx {
                err += dx;
                y += sty;
            }
        }
    }

    fn update(&mut self, ix: i64, iy: i64, id: u32, hit: bool) {
        let k = (iy * self.w + ix) as usize;
        let c = &mut self.cells[k];
        if c.stamp == id {
            return;
        }
        c.stamp = id;
        let l = if hit { self.l_hit } else { self.l_miss };
        c.logodds = (c.logodds + l).clamp(self.l_min, self.l_max);
        self.value[k] = if c.logodds > 0.0 {
            1.0 - 1.0 / (1.0 + c.logodds.exp())
        } else {
            self.p_min
        };
    }

    /// Grow so the grid-frame box `lo..hi` is inside.
    fn ensure(&mut self, lo: [f64; 2], hi: [f64; 2]) {
        if self.w == 0 {
            self.origin = [
                (lo[0] / self.res).floor() * self.res - GROW_MARGIN as f64 * self.res,
                (lo[1] / self.res).floor() * self.res - GROW_MARGIN as f64 * self.res,
            ];
        }
        let (ax, ay) = self.cell_of(lo);
        let (bx, by) = self.cell_of(hi);
        if ax >= 0 && ay >= 0 && bx < self.w && by < self.h {
            return;
        }
        let add_lx = if ax < 0 { -ax + GROW_MARGIN } else { 0 };
        let add_ly = if ay < 0 { -ay + GROW_MARGIN } else { 0 };
        let add_hx = if bx >= self.w {
            bx - self.w + 1 + GROW_MARGIN
        } else {
            0
        };
        let add_hy = if by >= self.h {
            by - self.h + 1 + GROW_MARGIN
        } else {
            0
        };
        let (nw, nh) = (self.w + add_lx + add_hx, self.h + add_ly + add_hy);
        let n = (nw * nh) as usize;
        let mut cells = vec![Cell::default(); n];
        let mut value = vec![self.p_min; n];
        for iy in 0..self.h {
            let src = (iy * self.w) as usize;
            let dst = ((iy + add_ly) * nw + add_lx) as usize;
            let len = self.w as usize;
            cells[dst..dst + len].copy_from_slice(&self.cells[src..src + len]);
            value[dst..dst + len].copy_from_slice(&self.value[src..src + len]);
        }
        self.origin = [
            self.origin[0] - add_lx as f64 * self.res,
            self.origin[1] - add_ly as f64 * self.res,
        ];
        (self.w, self.h) = (nw, nh);
        (self.cells, self.value) = (cells, value);
    }
}

/// Max-pooled occupancy of a grid for wide searches. Level 0 is `p_max` on
/// cells more likely occupied than free and `p_min` elsewhere: occupied or
/// not, however often seen, so a search over a whole submap is not drawn to
/// where the submap was built (walls seen at every scan have higher
/// probability than walls seen once from afar). Level `h` holds, at
/// `(ix, iy)`, the largest level-0 value over cells `[ix, ix+2^h) ×
/// [iy, iy+2^h)`, so a scan scored on level `h` at offset `o` bounds every
/// offset in the `2^h`-square at `o`.
#[derive(Debug, Clone)]
pub struct Pyramid {
    levels: Vec<Level>,
    p_min: f32,
    res: f64,
    origin: [f64; 2],
}

#[derive(Debug, Clone)]
struct Level {
    /// Cell index of the first stored column and row (`-(2^h - 1)`).
    lo: i64,
    w: i64,
    h: i64,
    data: Vec<f32>,
}

impl Level {
    fn get(&self, ix: i64, iy: i64, p_min: f32) -> f32 {
        let (x, y) = (ix - self.lo, iy - self.lo);
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return p_min;
        }
        self.data[(y * self.w + x) as usize]
    }
}

impl Pyramid {
    /// `depth` levels (window 1, 2, 4, ... `2^(depth-1)` cells).
    pub fn new(grid: &Grid, depth: usize) -> Pyramid {
        let p_min = grid.p_min;
        let mut levels = vec![Level {
            lo: 0,
            w: grid.w,
            h: grid.h,
            data: grid
                .cells
                .iter()
                .map(|c| if c.logodds > 0.0 { grid.p_max } else { p_min })
                .collect(),
        }];
        for h in 1..depth.max(1) {
            let s = 1i64 << (h - 1);
            let prev = &levels[h - 1];
            let lo = -((1i64 << h) - 1);
            let (w, hh) = (grid.w - lo, grid.h - lo);
            let mut data = vec![p_min; (w * hh).max(0) as usize];
            for y in 0..hh {
                let iy = y + lo;
                for x in 0..w {
                    let ix = x + lo;
                    let v = prev
                        .get(ix, iy, p_min)
                        .max(prev.get(ix + s, iy, p_min))
                        .max(prev.get(ix, iy + s, p_min))
                        .max(prev.get(ix + s, iy + s, p_min));
                    data[(y * w + x) as usize] = v;
                }
            }
            levels.push(Level { lo, w, h: hh, data });
        }
        Pyramid {
            levels,
            p_min,
            res: grid.res,
            origin: grid.origin,
        }
    }

    pub fn depth(&self) -> usize {
        self.levels.len()
    }

    pub fn resolution(&self) -> f64 {
        self.res
    }

    /// The cell holding grid-frame point `p` (the same lattice as the grid).
    pub fn cell_of(&self, p: [f64; 2]) -> (i64, i64) {
        (
            ((p[0] - self.origin[0]) / self.res).floor() as i64,
            ((p[1] - self.origin[1]) / self.res).floor() as i64,
        )
    }

    /// Max over the `2^level`-square of cells starting at `(ix, iy)`.
    pub fn value(&self, level: usize, ix: i64, iy: i64) -> f32 {
        self.levels[level].get(ix, iy, self.p_min)
    }
}
