//! Ring-buffer addressing and moving the window.

use std::ops::Range;

use crate::esdf::NONE;
use crate::{Vox, VoxMap, VOX_UNKNOWN};

/// Pack three coordinates below [`crate::MAX_AXIS`] into one word.
#[inline]
pub(crate) fn pack(c: [i32; 3]) -> u32 {
    (c[0] as u32) | ((c[1] as u32) << 10) | ((c[2] as u32) << 20)
}

#[inline]
pub(crate) fn unpack(p: u32) -> [i32; 3] {
    [
        (p & 0x3ff) as i32,
        ((p >> 10) & 0x3ff) as i32,
        ((p >> 20) & 0x3ff) as i32,
    ]
}

impl VoxMap {
    /// Ring coordinates of window coordinates `w` (each in `0..n`).
    #[inline]
    pub(crate) fn ring_of_window(&self, w: [i32; 3]) -> [i32; 3] {
        let mut r = [0; 3];
        for a in 0..3 {
            let v = w[a] + self.om[a];
            r[a] = if v >= self.n[a] { v - self.n[a] } else { v };
        }
        r
    }

    /// Window coordinates of ring coordinates `r`.
    #[inline]
    pub(crate) fn window_of_ring(&self, r: [i32; 3]) -> [i32; 3] {
        let mut w = [0; 3];
        for a in 0..3 {
            let v = r[a] - self.om[a];
            w[a] = if v < 0 { v + self.n[a] } else { v };
        }
        w
    }

    #[inline]
    pub(crate) fn idx_of_ring(&self, r: [i32; 3]) -> usize {
        (r[0] + r[1] * self.stride[1] + r[2] * self.stride[2]) as usize
    }

    #[inline]
    pub(crate) fn idx_of_window(&self, w: [i32; 3]) -> usize {
        self.idx_of_ring(self.ring_of_window(w))
    }

    /// Ring coordinates of ring slot `idx`.
    #[inline]
    pub(crate) fn ring_of_idx(&self, idx: usize) -> [i32; 3] {
        let i = idx as i32;
        [
            i % self.n[0],
            (i / self.n[0]) % self.n[1],
            i / self.stride[2],
        ]
    }

    /// Window coordinates of absolute voxel `c`, or `None` outside.
    #[inline]
    pub(crate) fn window_of_cell(&self, c: [i32; 3]) -> Option<[i32; 3]> {
        let w = [0, 1, 2].map(|a| c[a] - self.origin[a]);
        (0..3).all(|a| w[a] >= 0 && w[a] < self.n[a]).then_some(w)
    }

    /// Ring slot of absolute voxel `c`, or `None` outside the window.
    #[inline]
    pub(crate) fn idx_of_cell(&self, c: [i32; 3]) -> Option<usize> {
        self.window_of_cell(c).map(|w| self.idx_of_window(w))
    }

    /// Move the window so the voxel holding `body` is at its centre, once the
    /// body has drifted more than the configured slack from the centre on any
    /// axis. Voxels that scroll out are forgotten, and every voxel that kept
    /// a distance to an obstacle among them takes a new one; voxels that
    /// scroll in start unknown, with distances propagated from the voxels
    /// already in the window. Returns whether the window moved.
    pub fn recentre(&mut self, body: [f32; 3]) -> bool {
        if !body.iter().all(|v| v.is_finite()) {
            return false;
        }
        let c = self.cell_of(body);
        let half = self.n.map(|v| v / 2);
        if (0..3).all(|a| (c[a] - (self.origin[a] + half[a])).abs() <= self.slack) {
            return false;
        }
        let new_origin = [0, 1, 2].map(|a| c[a] - half[a]);
        let shift = [0, 1, 2].map(|a| new_origin[a] - self.origin[a]);
        if (0..3).any(|a| shift[a].abs() >= self.n[a]) {
            self.reset_all(new_origin);
            return true;
        }

        // Settle anything queued while the old window still addresses it.
        self.update_esdf();

        self.origin = new_origin;
        self.om = [0, 1, 2].map(|a| new_origin[a].rem_euclid(self.n[a]));
        for (a, &s) in shift.iter().enumerate() {
            if s == 0 {
                continue;
            }
            let entering = if s > 0 {
                self.n[a] - s..self.n[a]
            } else {
                0..-s
            };
            for w in self.slab(a, entering) {
                let idx = self.idx_of_window(w);
                self.reset_slot(idx);
            }
        }
        // A voxel that kept a distance to an obstacle in the leaving slabs
        // now points at a recycled (reset, so non-obstacle) slot. They all
        // lie within the cap of the trailing edge: raise them there.
        let reach = (self.esdf.cap2 as f32).sqrt().ceil() as i32 + 1;
        for (a, &s) in shift.iter().enumerate() {
            if s == 0 {
                continue;
            }
            let n = self.n[a];
            let band = if s > 0 {
                0..reach.min(n - s)
            } else {
                (n - reach).max(-s)..n
            };
            for w in self.slab(a, band) {
                let idx = self.idx_of_window(w);
                let cell = self.esdf.cells[idx];
                if cell.obst == NONE || cell.raise || cell.d2 == 0 {
                    continue;
                }
                if !self.esdf.is_obstacle(self.idx_of_ring(unpack(cell.obst))) {
                    self.esdf.mark_raise(idx);
                    self.esdf.push(u32::from(cell.d2), pack(w));
                }
            }
        }
        // Distances flow into the new slabs from the layer beside each.
        for (a, &s) in shift.iter().enumerate() {
            if s == 0 {
                continue;
            }
            let layer = if s > 0 { self.n[a] - s - 1 } else { -s };
            for w in self.slab(a, layer..layer + 1) {
                let idx = self.idx_of_window(w);
                let cell = self.esdf.cells[idx];
                if cell.obst != NONE {
                    self.esdf.push(u32::from(cell.d2), pack(w));
                }
            }
        }
        self.update_esdf();
        true
    }

    /// Every window coordinate with axis `a` in `range` and the other axes
    /// over the whole window.
    pub(crate) fn slab(&self, a: usize, range: Range<i32>) -> impl Iterator<Item = [i32; 3]> {
        let mut r: [Range<i32>; 3] = [0..self.n[0], 0..self.n[1], 0..self.n[2]];
        r[a] = range;
        let [rx, ry, rz] = r;
        rz.flat_map(move |z| {
            let rx = rx.clone();
            ry.clone()
                .flat_map(move |y| rx.clone().map(move |x| [x, y, z]))
        })
    }

    fn reset_slot(&mut self, idx: usize) {
        self.vox[idx] = Vox::default();
        self.state[idx] = VOX_UNKNOWN;
        self.esdf.reset(idx);
    }

    fn reset_all(&mut self, origin: [i32; 3]) {
        self.origin = origin;
        self.om = [0, 1, 2].map(|a| origin[a].rem_euclid(self.n[a]));
        self.vox.fill(Vox::default());
        self.state.fill(VOX_UNKNOWN);
        self.esdf.clear();
    }
}
