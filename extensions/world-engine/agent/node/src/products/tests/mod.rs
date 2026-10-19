//! Unit tests of the indoor products on synthetic scenes.

mod aisles;
mod align;
mod change;
mod cloud_io;
mod floorplan;
mod measure;
mod tour;

use super::geom::Rng;

/// Uniform noise in `[-amp, amp]`.
pub(super) fn jitter(rng: &mut Rng, amp: f64) -> f64 {
    (rng.next_u64() as f64 / u64::MAX as f64 * 2.0 - 1.0) * amp
}
