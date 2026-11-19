//! The wall slice of one floor: a band at body height above the floor plus,
//! when a ceiling is found, a band just under it, where walls are seldom
//! hidden behind furniture.

/// Histogram bin of the ceiling search, metres.
const BIN_M: f64 = 0.02;
/// The ceiling is searched this far above the floor and higher.
const CEILING_MIN_M: f64 = 2.0;
/// A bin counts toward the ceiling at this share of the fullest bin.
const CEILING_SHARE: f64 = 0.5;
/// Fewest points in the fullest bin for a ceiling to be reported.
const CEILING_MIN_POINTS: usize = 50;
/// The upper band, metres below the ceiling.
const UPPER_BAND: (f64, f64) = (0.4, 0.1);

/// Ceiling height above the floor plane: the highest 2 cm bin above 2 m whose
/// count is at least half the fullest bin above 2 m. Heights at or above
/// `limit_m` (the next floor up) are ignored. `None` without such a bin.
pub(crate) fn ceiling_height(
    heights: impl Iterator<Item = f64>,
    limit_m: Option<f64>,
) -> Option<f64> {
    let limit = limit_m.unwrap_or(f64::INFINITY);
    let mut bins: Vec<usize> = Vec::new();
    for h in heights {
        if !(h > CEILING_MIN_M && h < limit) || !h.is_finite() {
            continue;
        }
        let b = ((h - CEILING_MIN_M) / BIN_M) as usize;
        if b >= bins.len() {
            bins.resize(b + 1, 0);
        }
        bins[b] += 1;
    }
    let peak = bins.iter().copied().max()?;
    if peak < CEILING_MIN_POINTS {
        return None;
    }
    let top = bins
        .iter()
        .rposition(|&n| n as f64 >= CEILING_SHARE * peak as f64)?;
    Some(CEILING_MIN_M + (top as f64 + 0.5) * BIN_M)
}

/// The height bands (metres above the floor) whose points form the slice.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SliceBands {
    pub lower: (f64, f64),
    pub upper: Option<(f64, f64)>,
}

impl SliceBands {
    /// `lower` always; the band 0.4–0.1 m under the ceiling when there is one.
    pub fn new(lower: (f64, f64), ceiling_m: Option<f64>) -> Self {
        Self {
            lower,
            upper: ceiling_m.map(|c| (c - UPPER_BAND.0, c - UPPER_BAND.1)),
        }
    }

    pub fn contains(&self, h: f64) -> bool {
        let inside = |b: (f64, f64)| h >= b.0 && h <= b.1;
        inside(self.lower) || self.upper.is_some_and(inside)
    }
}
