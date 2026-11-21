//! Forgetting positive evidence that is no longer seen.

use crate::logodds::{DECAY_GRACE_S, DECAY_PER_S, LO_UNIT};
use crate::VoxMap;

/// Decay is applied in whole log-odds units; one unit every this many ns.
const NS_PER_UNIT: u64 = (LO_UNIT as f64 / DECAY_PER_S as f64 * 1e9) as u64;

impl VoxMap {
    /// Bring decay up to ADOS time `now_ns`: every voxel with positive
    /// evidence and no hit for [`DECAY_GRACE_S`] seconds moves toward
    /// unknown (zero) at [`DECAY_PER_S`] log-odds per second, never past
    /// zero. The first call only starts the clock. Returns the units applied.
    pub fn decay(&mut self, now_ns: u64) -> u32 {
        let Some(clock) = self.decay_clock_ns else {
            self.decay_clock_ns = Some(now_ns);
            return 0;
        };
        if now_ns <= clock {
            return 0;
        }
        let units = (now_ns - clock) / NS_PER_UNIT;
        if units == 0 {
            return 0;
        }
        self.decay_clock_ns = Some(clock + units * NS_PER_UNIT);
        let dec = units.min(i8::MAX as u64) as i8;
        let now_s = (now_ns / 1_000_000_000) as u8;
        for idx in 0..self.vox.len() {
            let v = self.vox[idx];
            if v.lo <= 0 || now_s.wrapping_sub(v.last_hit) < DECAY_GRACE_S {
                continue;
            }
            self.store_lo(idx, (v.lo - dec).max(0));
        }
        units as u32
    }
}
