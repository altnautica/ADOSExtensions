//! Shared-data world-model artifact descriptors. Each carries the capture
//! contract's [`Placement`]: where its coordinates sit in the world (the ENU
//! frame at the session anchor, or a local frame when the anchor is `null`).

use ados_capture_contract::Placement;
use serde::{Deserialize, Serialize};

/// The monotonic generation counter every world-model artifact set is stamped
/// with. A reconstruction is a JOB producing an immutable artifact set, not a
/// stream: generation N's splat, cloud, mesh and occupancy all describe the
/// same capture state, so a consumer can diff generations and a viewer can
/// fetch coarse chunks of N while N-1 is still on screen.
pub type Generation = u64;

/// Shared-data descriptor on [`super::PLUGIN_ATLAS_POINTCLOUD_TOPIC`]. The heavy buffer
/// rides the shm ring (`shm_name`/`slot`/`seq`) or a
/// stream-lane `url`; this carries the summary only.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointCloudDescriptor {
    /// The capture session this cloud was reconstructed from.
    #[serde(default)]
    pub session_id: String,
    /// Monotonic artifact generation (see [`Generation`]).
    #[serde(default)]
    pub generation: Generation,
    /// Where the cloud's coordinates sit in the world.
    pub placement: Placement,
    pub point_count: u64,
    /// Axis-aligned bounds: `[min_x, min_y, min_z, max_x, max_y, max_z]`.
    pub bounds: [f64; 6],
    pub shm_name: Option<String>,
    pub slot: Option<u32>,
    pub seq: Option<u64>,
    pub url: Option<String>,
}

/// What an occupancy buffer's voxels hold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OccupancyField {
    /// Per-voxel occupancy probability, quantised to `u8`.
    #[default]
    Occupancy,
    /// Euclidean signed distance to the nearest surface, `f32` metres,
    /// truncated at `truncation_m`. This is what a planner actually consumes:
    /// a binary voxel dump forces every planner to run its own distance
    /// transform, and a gradient is what lets a trajectory optimiser push away
    /// from an obstacle instead of only testing collisions.
    Esdf,
}

/// Shared-data descriptor on [`super::PLUGIN_ATLAS_OCCUPANCY_TOPIC`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OccupancyDescriptor {
    /// The capture session this grid was derived from.
    #[serde(default)]
    pub session_id: String,
    /// Monotonic artifact generation (see [`Generation`]).
    #[serde(default)]
    pub generation: Generation,
    /// Where the grid's coordinates (`origin`) sit in the world.
    pub placement: Placement,
    /// World-frame origin of voxel `(0,0,0)`.
    pub origin: [f64; 3],
    pub resolution_m: f32,
    /// Grid dimensions in voxels `[nx, ny, nz]`.
    pub dims: [u32; 3],
    /// What the voxels hold. `#[serde(default)]` reads an older producer's
    /// buffer as plain occupancy, never as a distance field a planner would
    /// then mis-read as metres.
    #[serde(default)]
    pub field: OccupancyField,
    /// The distance at which an [`OccupancyField::Esdf`] buffer is truncated,
    /// in metres. Meaningless (and zero) for a plain occupancy buffer.
    #[serde(default)]
    pub truncation_m: f32,
    pub shm_name: Option<String>,
    pub slot: Option<u32>,
    pub seq: Option<u64>,
    /// Where the buffer can be fetched when it is NOT in shared memory. The
    /// on-drone producer writes into the shm ring; a compute node writes a file
    /// and serves it, exactly like the sibling artifact descriptors. Row-major
    /// `nx * ny * nz`, little-endian: `u8` occupancy probability for
    /// [`OccupancyField::Occupancy`], `f32` metres for
    /// [`OccupancyField::Esdf`].
    #[serde(default)]
    pub url: Option<String>,
}

/// Shared-data descriptor on [`super::PLUGIN_ATLAS_SPLAT_TOPIC`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SplatDescriptor {
    /// The capture session this splat was trained from.
    #[serde(default)]
    pub session_id: String,
    /// Monotonic artifact generation (see [`Generation`]).
    #[serde(default)]
    pub generation: Generation,
    /// Where the splat's coordinates sit in the world.
    pub placement: Placement,
    pub gaussian_count: u64,
    /// Training step this descriptor reflects (monotonic for live sessions).
    pub step: u64,
    pub url: Option<String>,
    pub handle: Option<String>,
    /// A chunk manifest for level-of-detail streaming, when the producer wrote
    /// one. The viewer fetches the coarsest level first and refines, so first
    /// pixels arrive in well under a second on a scene whose full transfer
    /// takes minutes.
    ///
    /// This is what replaced the specified "SPZ delta" lane. SPZ and SOG are
    /// whole-scene containers with global quantisation and Morton ordering, and
    /// NEITHER the formats NOR the Khronos glTF extensions define an
    /// incremental append or delta codec — so a delta lane would have meant
    /// inventing and maintaining a codec no other tool reads. Generation-
    /// versioned LOD chunks get the same "see it grow" behaviour out of formats
    /// that already exist.
    #[serde(default)]
    pub manifest_url: Option<String>,
    /// Number of LOD levels behind `manifest_url` (0 when there is no manifest).
    #[serde(default)]
    pub lod_levels: u8,
}

/// Shared-data descriptor on [`super::PLUGIN_ATLAS_MESH_TOPIC`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshDescriptor {
    /// The capture session this mesh was extracted from.
    #[serde(default)]
    pub session_id: String,
    /// Monotonic artifact generation (see [`Generation`]).
    #[serde(default)]
    pub generation: Generation,
    /// Where the mesh's coordinates sit in the world.
    pub placement: Placement,
    pub vertex_count: u64,
    pub face_count: u64,
    pub url: Option<String>,
    pub handle: Option<String>,
}
