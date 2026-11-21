//! The incremental distance field.
//!
//! Every voxel within [`crate::ESDF_CAP_M`] of an obstacle voxel stores that
//! obstacle (its ring coordinates, which stay put while the window moves) and
//! the squared distance to it in voxels. Changes run as waves over a queue
//! bucketed by squared distance, nearest first:
//!
//! * a new obstacle starts a lower wave: each neighbour that is closer to the
//!   obstacle than to its current one takes it and passes it on;
//! * a removed obstacle starts a raise wave: every voxel that pointed at it is
//!   reset, and the voxels around that region that still point at a live
//!   obstacle start lower waves back into it.
//!
//! Waves only travel within the cap of the obstacle they carry, so their cost
//! follows what changed. They step between face neighbours and carry the
//! obstacle itself, so the distance stored is the true distance to a real
//! obstacle; propagation can hand a voxel an obstacle that is not quite the
//! nearest, a small overestimate the tests bound against a brute-force
//! transform. Each voxel's obstacle, distance and raise mark sit together in
//! one 8-byte cell, so a neighbour costs one cache line; the published
//! decimetre plane is kept beside it as each distance changes.

use crate::window::{pack, unpack};
use crate::{VoxMap, ESDF_CAP_DM, ESDF_CAP_M};

/// No obstacle within the cap.
pub(crate) const NONE: u32 = u32::MAX;
/// Squared distance of a voxel with no obstacle within the cap.
pub(crate) const D2_INF: u16 = u16::MAX;

/// One voxel's distance-field entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub(crate) struct Cell {
    /// Packed ring coordinates of the nearest obstacle, or [`NONE`].
    pub(crate) obst: u32,
    /// Squared distance to it in voxels, or [`D2_INF`].
    pub(crate) d2: u16,
    /// Queued for the raise wave.
    pub(crate) raise: bool,
}

pub(crate) const EMPTY: Cell = Cell {
    obst: NONE,
    d2: D2_INF,
    raise: false,
};

pub(crate) struct Esdf {
    /// One cell per ring slot.
    pub(crate) cells: Vec<Cell>,
    /// Decimetre distance per ring slot (the published plane, ring order).
    pub(crate) dm: Vec<u8>,
    /// Packed window coordinates queued per squared distance.
    buckets: Vec<Vec<u32>>,
    /// Lowest bucket that may hold an entry.
    cur: usize,
    /// Largest stored squared distance.
    pub(crate) cap2: u32,
    /// Decimetres per squared distance, `0..=cap2`.
    lut: Vec<u8>,
}

impl Esdf {
    pub(crate) fn new(count: usize, res_m: f32) -> Self {
        let cap_vox = ESDF_CAP_M / res_m;
        let cap2 = (cap_vox * cap_vox).floor().min(f32::from(D2_INF - 1)) as u32;
        let lut = (0..=cap2)
            .map(|d2| {
                let dm = ((d2 as f32).sqrt() * res_m * 10.0).round();
                dm.min(f32::from(ESDF_CAP_DM)) as u8
            })
            .collect();
        Self {
            cells: vec![EMPTY; count],
            dm: vec![ESDF_CAP_DM; count],
            buckets: (0..=cap2).map(|_| Vec::with_capacity(1024)).collect(),
            cur: cap2 as usize + 1,
            cap2,
            lut,
        }
    }

    /// The published decimetre value of squared distance `d2`.
    #[inline]
    pub(crate) fn dm_of(&self, d2: u16) -> u8 {
        if d2 == D2_INF {
            ESDF_CAP_DM
        } else {
            self.lut[d2 as usize]
        }
    }

    #[inline]
    fn put(&mut self, idx: usize, cell: Cell) {
        self.dm[idx] = self.dm_of(cell.d2);
        self.cells[idx] = cell;
    }

    /// Whether slot `idx` is in the obstacle set (only obstacles sit at
    /// distance zero).
    #[inline]
    pub(crate) fn is_obstacle(&self, idx: usize) -> bool {
        self.cells[idx].d2 == 0
    }

    /// Reset slot `idx` and mark it for the raise wave.
    #[inline]
    pub(crate) fn mark_raise(&mut self, idx: usize) {
        self.put(
            idx,
            Cell {
                raise: true,
                ..EMPTY
            },
        );
    }

    /// Forget slot `idx` (it scrolled into the window).
    pub(crate) fn reset(&mut self, idx: usize) {
        self.put(idx, EMPTY);
    }

    #[inline]
    pub(crate) fn push(&mut self, prio: u32, w: u32) {
        let p = prio as usize;
        self.buckets[p].push(w);
        if p < self.cur {
            self.cur = p;
        }
    }

    /// The nearest queued entry and the priority it was queued at.
    #[inline]
    fn pop(&mut self) -> Option<(u32, u32)> {
        while self.cur < self.buckets.len() {
            if let Some(w) = self.buckets[self.cur].pop() {
                return Some((self.cur as u32, w));
            }
            self.cur += 1;
        }
        None
    }

    pub(crate) fn clear(&mut self) {
        self.cells.fill(EMPTY);
        self.dm.fill(ESDF_CAP_DM);
        for b in &mut self.buckets {
            b.clear();
        }
        self.cur = self.buckets.len();
    }
}

impl VoxMap {
    /// Ring slot `idx` became occupied.
    pub(crate) fn esdf_set_obstacle(&mut self, idx: usize) {
        self.stats.obstacles_added += 1;
        let r = self.ring_of_idx(idx);
        self.esdf.put(
            idx,
            Cell {
                obst: pack(r),
                d2: 0,
                raise: false,
            },
        );
        self.esdf.push(0, pack(self.window_of_ring(r)));
    }

    /// Ring slot `idx` stopped being occupied.
    pub(crate) fn esdf_remove_obstacle(&mut self, idx: usize) {
        self.stats.obstacles_removed += 1;
        let w = self.window_of_ring(self.ring_of_idx(idx));
        self.esdf.put(
            idx,
            Cell {
                raise: true,
                ..EMPTY
            },
        );
        self.esdf.push(0, pack(w));
    }

    /// Run every queued wave to completion.
    pub fn update_esdf(&mut self) {
        while let Some((prio, wp)) = self.esdf.pop() {
            let w = unpack(wp);
            let r = self.ring_of_window(w);
            let idx = self.idx_of_ring(r);
            let cell = self.esdf.cells[idx];
            if cell.raise {
                self.stats.esdf_visits += 1;
                self.raise(wp, w, r, idx);
            } else if u32::from(cell.d2) == prio
                && cell.obst != NONE
                && self.esdf.is_obstacle(self.idx_of_ring(unpack(cell.obst)))
            {
                // An entry whose voxel has since taken a nearer obstacle is
                // stale: that nearer one was queued and spreads instead.
                self.stats.esdf_visits += 1;
                self.lower(wp, w, r, cell.obst);
            }
        }
    }

    fn raise(&mut self, wp: u32, w: [i32; 3], r: [i32; 3], idx: usize) {
        for a in 0..3 {
            for up in [false, true] {
                let Some((wnp, ni)) = self.step(wp, w, r, idx, a, up) else {
                    continue;
                };
                let c = self.esdf.cells[ni];
                if c.raise || c.obst == NONE {
                    continue;
                }
                if !self.esdf.is_obstacle(self.idx_of_ring(unpack(c.obst))) {
                    self.esdf.put(
                        ni,
                        Cell {
                            raise: true,
                            ..EMPTY
                        },
                    );
                }
                // A reset voxel is queued at its old distance so the raise
                // spreads in order; a live one at its own, to lower back in.
                self.esdf.push(u32::from(c.d2), wnp);
            }
        }
        self.esdf.cells[idx].raise = false;
    }

    fn lower(&mut self, wp: u32, w: [i32; 3], r: [i32; 3], o: u32) {
        let idx = self.idx_of_ring(r);
        let wo = self.window_of_ring(unpack(o));
        let base = [w[0] - wo[0], w[1] - wo[1], w[2] - wo[2]];
        let cap2 = self.esdf.cap2;
        for a in 0..3 {
            for up in [false, true] {
                let mut dd = base;
                dd[a] += if up { 1 } else { -1 };
                let d = (dd[0] * dd[0] + dd[1] * dd[1] + dd[2] * dd[2]) as u32;
                if d > cap2 {
                    continue;
                }
                let Some((wnp, ni)) = self.step(wp, w, r, idx, a, up) else {
                    continue;
                };
                let c = self.esdf.cells[ni];
                if c.raise || d >= u32::from(c.d2) {
                    continue;
                }
                self.esdf.put(
                    ni,
                    Cell {
                        obst: o,
                        d2: d as u16,
                        raise: false,
                    },
                );
                self.esdf.push(d, wnp);
            }
        }
    }

    /// The face neighbour of window voxel `w` (packed `wp`, ring `r`, slot
    /// `idx`) one step along axis `a`: its packed window coordinates and ring
    /// slot, or `None` past the window's edge.
    #[inline]
    fn step(
        &self,
        wp: u32,
        w: [i32; 3],
        r: [i32; 3],
        idx: usize,
        a: usize,
        up: bool,
    ) -> Option<(u32, usize)> {
        let n = self.n[a];
        let stride = self.stride[a] as usize;
        let unit = 1u32 << (10 * a);
        if up {
            if w[a] + 1 >= n {
                return None;
            }
            let ni = if r[a] + 1 == n {
                idx - stride * (n as usize - 1)
            } else {
                idx + stride
            };
            Some((wp + unit, ni))
        } else {
            if w[a] == 0 {
                return None;
            }
            let ni = if r[a] == 0 {
                idx + stride * (n as usize - 1)
            } else {
                idx - stride
            };
            Some((wp - unit, ni))
        }
    }
}
