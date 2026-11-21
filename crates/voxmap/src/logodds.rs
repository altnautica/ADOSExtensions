//! Fixed-point log-odds and the per-voxel update rules.
//!
//! Each insertion is one scan. A voxel takes at most one update per scan,
//! applied the moment the scan first reaches it: a hit, a full miss, or a
//! near miss (a voxel touching the ray's return). The scan's marks live in
//! the flag byte and count only while the voxel's epoch byte equals the
//! scan's, so a new scan starts with no clearing pass.

use crate::{VoxMap, VOX_FREE, VOX_OCCUPIED, VOX_UNKNOWN};

/// Log-odds of one `i8` unit.
pub const LO_UNIT: f32 = 0.05;
/// A return in the voxel: +0.85.
pub const LO_HIT: i8 = 17;
/// A ray through the voxel: -0.40.
pub const LO_MISS: i8 = -8;
/// Lower clamp: -2.0.
pub const LO_MIN: i8 = -40;
/// Upper clamp: +3.5.
pub const LO_MAX: i8 = 70;
/// Occupied at or above +0.85.
pub const LO_OCCUPIED: i8 = 17;
/// Free at or below -0.40.
pub const LO_FREE: i8 = -8;
/// Seconds without a hit before positive evidence starts to decay.
pub const DECAY_GRACE_S: u8 = 10;
/// Decay rate toward unknown, log-odds per second (0.2).
pub const DECAY_PER_S: f32 = 0.2;
/// Consecutive passing insertions that clear a voxel with positive evidence.
pub const DYNAMIC_PASSES: u8 = 3;

/// Flag byte: the dynamic pass streak (kept across scans) ...
const F_STREAK: u8 = 0b11;
/// ... and this scan's marks: what the voxel took, and whether a ray ended
/// in it.
const KIND_SHIFT: u8 = 2;
const KIND: u8 = 0b11 << KIND_SHIFT;
const K_NEAR: u8 = 1 << KIND_SHIFT;
const K_MISS: u8 = 2 << KIND_SHIFT;
const K_HIT: u8 = 3 << KIND_SHIFT;
const F_RAY_END: u8 = 1 << 4;

/// The occupancy class of a log-odds value.
#[inline]
pub(crate) fn class(lo: i8) -> u8 {
    if lo >= LO_OCCUPIED {
        VOX_OCCUPIED
    } else if lo <= LO_FREE {
        VOX_FREE
    } else {
        VOX_UNKNOWN
    }
}

impl VoxMap {
    /// Store a new log-odds value in ring slot `idx`, keeping the class plane
    /// and the distance field's obstacle set in step. A voxel joins the
    /// obstacle set when it becomes occupied and leaves it only when its
    /// positive evidence is gone (log-odds at or below zero): a sparsely
    /// seen surface that one miss drops back to unknown stays an obstacle to
    /// the distance field instead of flickering in and out of it.
    #[inline]
    pub(crate) fn store_lo(&mut self, idx: usize, lo: i8) {
        let old = self.vox[idx].lo;
        self.vox[idx].lo = lo;
        let class_now = class(lo);
        if class_now != class(old) {
            self.state[idx] = class_now;
        }
        // Membership can only change on joining (occupied) or on losing the
        // last positive evidence; free-space updates never look it up.
        if lo >= LO_OCCUPIED {
            if !self.esdf.is_obstacle(idx) {
                self.esdf_set_obstacle(idx);
            }
        } else if lo <= 0 && old > 0 && self.esdf.is_obstacle(idx) {
            self.esdf_remove_obstacle(idx);
        }
    }

    /// Start a scan: every voxel's marks lapse.
    pub(crate) fn begin_scan(&mut self) {
        self.epoch_now = self.epoch_now.wrapping_add(1);
        if self.epoch_now == 0 {
            for v in &mut self.vox {
                v.epoch = 0;
            }
            self.epoch_now = 1;
        }
    }

    /// This scan's marks and the streak of slot `idx`.
    #[inline]
    fn marks(&self, idx: usize) -> (u8, u8) {
        let v = self.vox[idx];
        let marks = if v.epoch == self.epoch_now {
            v.flags & (KIND | F_RAY_END)
        } else {
            0
        };
        (marks, v.flags & F_STREAK)
    }

    #[inline]
    fn set_marks(&mut self, idx: usize, marks: u8, streak: u8) {
        let v = &mut self.vox[idx];
        v.epoch = self.epoch_now;
        v.flags = marks | streak;
    }

    /// A return in slot `idx`.
    #[inline]
    pub(crate) fn hit_voxel(&mut self, idx: usize) {
        let (m, _) = self.marks(idx);
        if m & KIND == K_HIT {
            return;
        }
        self.set_marks(idx, (m & F_RAY_END) | K_HIT, 0);
        self.stats.voxels_updated += 1;
        self.vox[idx].last_hit = self.now_s;
        let lo = self.vox[idx].lo.saturating_add(LO_HIT).min(LO_MAX);
        self.store_lo(idx, lo);
    }

    /// Record that a ray ends in slot `idx`; false when one already did.
    #[inline]
    pub(crate) fn claim_ray_end(&mut self, idx: usize) -> bool {
        let (m, streak) = self.marks(idx);
        if m & F_RAY_END != 0 {
            return false;
        }
        self.set_marks(idx, m | F_RAY_END, streak);
        true
    }

    /// A ray passes slot `idx`, as a near miss when `near`. Returns true when
    /// a ray already passed it in this scan (so the rest of the way back to
    /// the sensor is covered too).
    #[inline]
    pub(crate) fn pass_voxel(&mut self, idx: usize, near: bool) -> bool {
        let (m, streak) = self.marks(idx);
        match m & KIND {
            K_HIT => false,
            K_MISS => true,
            K_NEAR => {
                if !near {
                    self.set_marks(idx, (m & F_RAY_END) | K_MISS, streak);
                    // The near miss left positive evidence alone; a full
                    // miss does not.
                    if self.vox[idx].lo > 0 {
                        self.full_miss(idx, streak);
                    }
                }
                true
            }
            _ => {
                self.stats.voxels_updated += 1;
                let lo = self.vox[idx].lo;
                if near {
                    self.set_marks(idx, m | K_NEAR, streak);
                    // Next to a return: clears space up to a surface, never
                    // erodes one (the return is evidence of a surface here).
                    if lo <= 0 {
                        self.store_lo(idx, lo.saturating_add(LO_MISS).max(LO_MIN));
                    }
                } else {
                    self.set_marks(idx, m | K_MISS, streak);
                    self.full_miss(idx, streak);
                }
                false
            }
        }
    }

    /// A full miss: positive evidence drops, and the third pass in a row
    /// clears it outright.
    fn full_miss(&mut self, idx: usize, streak: u8) {
        let v = &mut self.vox[idx];
        let lo = v.lo;
        let marks = v.flags & !F_STREAK;
        if lo > 0 {
            let streak = streak + 1;
            if streak >= DYNAMIC_PASSES {
                v.flags = marks;
                self.store_lo(idx, LO_FREE);
            } else {
                v.flags = marks | streak;
                self.store_lo(idx, lo.saturating_add(LO_MISS).max(LO_MIN));
            }
        } else {
            v.flags = marks;
            self.store_lo(idx, lo.saturating_add(LO_MISS).max(LO_MIN));
        }
    }
}
