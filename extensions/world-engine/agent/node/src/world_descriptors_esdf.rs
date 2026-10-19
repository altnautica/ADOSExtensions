//! The truncated Euclidean signed distance field behind the occupancy lane.
//!
//! # The occupancy lane is an ESDF, not a voxel dump
//!
//! A planner does not want a binary occupancy grid. Every planner that receives
//! one immediately runs its own distance transform over it, because what a
//! trajectory optimiser needs is a distance AND a gradient — something to push
//! away from, not merely something to test collisions against. So the producer
//! does that work once, here, and publishes a truncated Euclidean signed
//! distance field: `f32` metres to the nearest occupied surface, clamped at a
//! truncation radius beyond which the exact value stops mattering for planning.
//!
//! The distance transform is Felzenszwalb & Huttenlocher's exact separable
//! algorithm, run per axis. It is O(n) per axis in the number of voxels and
//! EXACT — a chamfer or multi-pass approximation would be cheaper to write and
//! wrong by up to ~10% near diagonals, which is not an acceptable error on a
//! clearance figure an aircraft flies against.

/// Default voxel edge for a derived occupancy / ESDF grid, in metres. 20 cm is
/// the coarse local-costmap resolution a planner uses for an aerial vehicle:
/// fine enough to resolve a mast or a cable run, coarse enough that a 200 m
/// capture is a grid a drone can hold in memory.
pub const DEFAULT_ESDF_RESOLUTION_M: f32 = 0.20;

/// Default truncation radius for the ESDF, in metres. Beyond a few metres of
/// clearance the exact distance stops changing a planning decision, and
/// truncating keeps the field's dynamic range small enough to be useful at
/// `f32` and cheap to compress.
pub const DEFAULT_ESDF_TRUNCATION_M: f32 = 4.0;

/// Hard ceiling on the derived grid's voxel count, so a large or mis-scaled
/// reconstruction cannot allocate an unbounded field on the node. At the
/// default 20 cm resolution this is a ~1.3 km-per-side volume; a capture larger
/// than this yields no occupancy descriptor rather than a truncated,
/// silently-wrong one.
pub const MAX_ESDF_VOXELS: usize = 64_000_000;

/// A derived occupancy / distance grid.
#[derive(Debug, Clone, PartialEq)]
pub struct EsdfGrid {
    /// World-frame position of voxel `(0,0,0)`'s centre.
    pub origin: [f64; 3],
    pub resolution_m: f32,
    /// `[nx, ny, nz]`.
    pub dims: [u32; 3],
    /// Truncation radius in metres.
    pub truncation_m: f32,
    /// Row-major `nx * ny * nz` distances in metres, x fastest.
    pub distances: Vec<f32>,
}

impl EsdfGrid {
    /// `nx * ny * nz`.
    pub fn voxel_count(&self) -> usize {
        self.dims[0] as usize * self.dims[1] as usize * self.dims[2] as usize
    }

    /// The distance at a voxel index, for tests and consumers that hold the grid
    /// in memory.
    pub fn at(&self, x: u32, y: u32, z: u32) -> Option<f32> {
        if x >= self.dims[0] || y >= self.dims[1] || z >= self.dims[2] {
            return None;
        }
        let idx =
            (z as usize * self.dims[1] as usize + y as usize) * self.dims[0] as usize + x as usize;
        self.distances.get(idx).copied()
    }

    /// The grid as little-endian `f32` bytes, the layout
    /// [`OccupancyDescriptor::url`](world_engine_protocol::atlas::OccupancyDescriptor::url) promises.
    pub fn to_le_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.distances.len() * 4);
        for d in &self.distances {
            out.extend_from_slice(&d.to_le_bytes());
        }
        out
    }
}

/// Why an ESDF could not be derived. Every variant is a case where publishing
/// something anyway would mean publishing a planning input that is wrong.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum EsdfError {
    /// The cloud had no points, so there is no surface to measure against.
    #[error("no points in the reconstruction")]
    NoPoints,
    /// A non-finite coordinate would make every derived bound meaningless.
    #[error("reconstruction contains non-finite coordinates")]
    NonFinite,
    /// The requested resolution is not a usable voxel edge.
    #[error("resolution {0} m is not a positive finite length")]
    BadResolution(f32),
    /// The grid the capture's extent implies is larger than
    /// [`MAX_ESDF_VOXELS`].
    #[error("grid of {got} voxels exceeds the {max} ceiling")]
    TooLarge { got: usize, max: usize },
}

/// Build a truncated ESDF from a reconstruction's points.
///
/// The grid is sized to the cloud's axis-aligned bounds plus one truncation
/// radius of padding on every side, so a planner has real clearance values in
/// the free space AROUND the structure rather than a field that stops at the
/// surface.
pub fn esdf_from_points(
    positions: &[[f32; 3]],
    resolution_m: f32,
    truncation_m: f32,
) -> Result<EsdfGrid, EsdfError> {
    if !resolution_m.is_finite() || resolution_m <= 0.0 {
        return Err(EsdfError::BadResolution(resolution_m));
    }
    if positions.is_empty() {
        return Err(EsdfError::NoPoints);
    }
    let truncation_m = if truncation_m.is_finite() && truncation_m > 0.0 {
        truncation_m
    } else {
        DEFAULT_ESDF_TRUNCATION_M
    };

    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for p in positions {
        for a in 0..3 {
            if !p[a].is_finite() {
                return Err(EsdfError::NonFinite);
            }
            min[a] = min[a].min(p[a]);
            max[a] = max[a].max(p[a]);
        }
    }

    // Pad by a truncation radius so free space around the structure is covered.
    let pad = truncation_m;
    let mut dims = [0u32; 3];
    let mut origin = [0.0f64; 3];
    for a in 0..3 {
        let lo = min[a] - pad;
        let hi = max[a] + pad;
        origin[a] = lo as f64;
        // At least one voxel per axis: a planar cloud is legitimate input.
        let n = ((hi - lo) / resolution_m).ceil() as i64 + 1;
        dims[a] = n.clamp(1, u32::MAX as i64) as u32;
    }
    let count = dims[0] as usize * dims[1] as usize * dims[2] as usize;
    if count > MAX_ESDF_VOXELS {
        return Err(EsdfError::TooLarge {
            got: count,
            max: MAX_ESDF_VOXELS,
        });
    }

    // Squared-distance field in VOXEL units. `INF` marks free space; an occupied
    // voxel starts at 0 and the transform grows outward from it.
    const INF: f32 = 1e20;
    let mut sq = vec![INF; count];
    let (nx, ny, nz) = (dims[0] as usize, dims[1] as usize, dims[2] as usize);
    let idx = |x: usize, y: usize, z: usize| (z * ny + y) * nx + x;
    for p in positions {
        let vx = (((p[0] as f64 - origin[0]) / resolution_m as f64).round() as i64)
            .clamp(0, nx as i64 - 1) as usize;
        let vy = (((p[1] as f64 - origin[1]) / resolution_m as f64).round() as i64)
            .clamp(0, ny as i64 - 1) as usize;
        let vz = (((p[2] as f64 - origin[2]) / resolution_m as f64).round() as i64)
            .clamp(0, nz as i64 - 1) as usize;
        sq[idx(vx, vy, vz)] = 0.0;
    }

    // Exact separable EDT, one axis at a time (Felzenszwalb & Huttenlocher).
    let mut line = Vec::with_capacity(nx.max(ny).max(nz));
    let mut out = vec![0.0f32; nx.max(ny).max(nz)];
    // X
    for z in 0..nz {
        for y in 0..ny {
            line.clear();
            line.extend((0..nx).map(|x| sq[idx(x, y, z)]));
            transform_1d(&line, &mut out[..nx]);
            for x in 0..nx {
                sq[idx(x, y, z)] = out[x];
            }
        }
    }
    // Y
    for z in 0..nz {
        for x in 0..nx {
            line.clear();
            line.extend((0..ny).map(|y| sq[idx(x, y, z)]));
            transform_1d(&line, &mut out[..ny]);
            for y in 0..ny {
                sq[idx(x, y, z)] = out[y];
            }
        }
    }
    // Z
    for y in 0..ny {
        for x in 0..nx {
            line.clear();
            line.extend((0..nz).map(|z| sq[idx(x, y, z)]));
            transform_1d(&line, &mut out[..nz]);
            for z in 0..nz {
                sq[idx(x, y, z)] = out[z];
            }
        }
    }

    // Voxel-space squared distance -> truncated metres.
    let distances = sq
        .into_iter()
        .map(|s| (s.max(0.0).sqrt() * resolution_m).min(truncation_m))
        .collect();

    Ok(EsdfGrid {
        origin,
        resolution_m,
        dims,
        truncation_m,
        distances,
    })
}

/// The 1-D exact squared-distance transform of `f`, written into `out`.
///
/// Felzenszwalb & Huttenlocher's lower-envelope-of-parabolas method: each cell
/// contributes the parabola `(q - p)^2 + f[p]`, the algorithm walks the
/// envelope's breakpoints in one forward pass and samples it in one backward
/// pass, so the whole line is O(n) rather than O(n^2).
pub(super) fn transform_1d(f: &[f32], out: &mut [f32]) {
    let n = f.len();
    debug_assert_eq!(out.len(), n);
    if n == 0 {
        return;
    }
    if n == 1 {
        out[0] = f[0];
        return;
    }
    // `v[k]` is the index of the k-th parabola in the lower envelope; `z[k]` is
    // the boundary between parabola k-1 and k.
    let mut v = vec![0usize; n];
    let mut z = vec![0.0f32; n + 1];
    let mut k = 0usize;
    v[0] = 0;
    z[0] = f32::NEG_INFINITY;
    z[1] = f32::INFINITY;
    for q in 1..n {
        // Intersection of the parabola from q with the current last one.
        let mut s = intersection(f, v[k], q);
        while s <= z[k] {
            // The new parabola hides the last one entirely; pop it.
            if k == 0 {
                break;
            }
            k -= 1;
            s = intersection(f, v[k], q);
        }
        if s <= z[k] && k == 0 {
            v[0] = q;
            z[0] = f32::NEG_INFINITY;
            z[1] = f32::INFINITY;
            continue;
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f32::INFINITY;
    }
    let mut k = 0usize;
    for (q, slot) in out.iter_mut().enumerate().take(n) {
        // Advance to the envelope segment covering q; each segment is visited
        // once across the whole pass, which is what keeps this O(n).
        while z[k + 1] < q as f32 {
            k += 1;
        }
        let d = q as f32 - v[k] as f32;
        *slot = d * d + f[v[k]];
    }
}

/// The q-coordinate where the parabolas rooted at `p` and `q` intersect.
fn intersection(f: &[f32], p: usize, q: usize) -> f32 {
    let (fp, fq) = (f[p], f[q]);
    let (pf, qf) = (p as f32, q as f32);
    let num = (fq + qf * qf) - (fp + pf * pf);
    let den = 2.0 * (qf - pf);
    if den == 0.0 {
        f32::INFINITY
    } else {
        num / den
    }
}
