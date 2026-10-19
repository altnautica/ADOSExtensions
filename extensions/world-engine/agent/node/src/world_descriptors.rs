//! Turning a finished reconstruction into shared world-model DATA.
//!
//! Without this module Atlas is a private pipeline ending in a viewer: the
//! compute node writes a `.ply` and a `.rrd`, the GCS renders them, and nothing
//! else in the system can consume the world the aircraft just mapped. The four
//! `plugin.atlas.*` shared-data topics existed as constants with no publisher
//! and no subscriber anywhere in the tree.
//!
//! This is the publisher. When a reconstruct job completes, its outputs are
//! mapped to descriptors stamped with the capture session and a monotonically
//! increasing `generation`, and published on the per-device world stream. A
//! descriptor is small and carries no pixels — it says what exists, how big it
//! is, and where to fetch it — so it rides any bearer and a consumer pulls only
//! what it needs.
//!
//! The occupancy lane's distance field lives in `world_descriptors_esdf.rs`.

use std::path::{Path, PathBuf};

use ados_capture_contract::Placement;
use world_engine_protocol::atlas::{
    Generation, MeshDescriptor, OccupancyDescriptor, OccupancyField, PointCloudDescriptor,
    SplatDescriptor,
};

use crate::rerun_world::parse_ply;
use crate::Output;

#[path = "world_descriptors_esdf.rs"]
mod esdf;
pub use esdf::*;

/// The world-model descriptors derived from one reconstruction generation. Each
/// is `None` when the generation produced no artifact of that kind — never a
/// placeholder, so a consumer cannot mistake an absent mesh for an empty one.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WorldDescriptorSet {
    pub splat: Option<SplatDescriptor>,
    pub pointcloud: Option<PointCloudDescriptor>,
    pub mesh: Option<MeshDescriptor>,
    pub occupancy: Option<OccupancyDescriptor>,
}

impl WorldDescriptorSet {
    /// Whether this generation produced anything at all to publish.
    pub fn is_empty(&self) -> bool {
        self.splat.is_none()
            && self.pointcloud.is_none()
            && self.mesh.is_none()
            && self.occupancy.is_none()
    }
}

/// Map one completed job's outputs to shared-data descriptors, each stating
/// `placement` (where the generation's coordinates sit in the world).
///
/// Every count and bound is read from the real artifact — a `gaussian_count`
/// comes from the backend's own metadata or from parsing the `.ply`, never from
/// a guess — so a descriptor either states a measured fact or omits the field.
pub fn derive_descriptors(
    session_id: &str,
    generation: Generation,
    placement: &Placement,
    outputs: &[Output],
    work_root: &Path,
) -> WorldDescriptorSet {
    let mut set = WorldDescriptorSet::default();
    for out in outputs {
        match out.kind.as_str() {
            "splat" => {
                set.splat = Some(SplatDescriptor {
                    session_id: session_id.to_string(),
                    generation,
                    placement: placement.clone(),
                    gaussian_count: meta_u64(out, "gaussian_count")
                        .or_else(|| ply_point_count(out, work_root))
                        .unwrap_or(0),
                    step: meta_u64(out, "step").unwrap_or(0),
                    url: Some(out.uri.clone()),
                    handle: None,
                    manifest_url: meta_str(out, "manifest_url"),
                    lod_levels: meta_u64(out, "lod_levels").unwrap_or(0).min(255) as u8,
                });
            }
            "pointcloud" => {
                let points = local_ply_points(out, work_root);
                set.pointcloud = Some(PointCloudDescriptor {
                    session_id: session_id.to_string(),
                    generation,
                    placement: placement.clone(),
                    point_count: meta_u64(out, "point_count")
                        .unwrap_or_else(|| points.as_ref().map(|p| p.len() as u64).unwrap_or(0)),
                    bounds: points.as_deref().map(bounds_of).unwrap_or([0.0; 6]),
                    shm_name: None,
                    slot: None,
                    seq: None,
                    url: Some(out.uri.clone()),
                });
            }
            "mesh" => {
                set.mesh = Some(MeshDescriptor {
                    session_id: session_id.to_string(),
                    generation,
                    placement: placement.clone(),
                    vertex_count: meta_u64(out, "vertex_count").unwrap_or(0),
                    face_count: meta_u64(out, "face_count").unwrap_or(0),
                    url: Some(out.uri.clone()),
                    handle: None,
                });
            }
            _ => {}
        }
    }
    set
}

/// Derive the ESDF for a generation from whichever output carries geometry,
/// write it under the work root, and return its descriptor.
///
/// Returns `Ok(None)` when the generation carries no readable geometry — an
/// honest "no planning input this generation" rather than an empty grid a
/// planner would read as wide-open free space.
pub fn derive_occupancy(
    session_id: &str,
    generation: Generation,
    placement: &Placement,
    outputs: &[Output],
    work_root: &Path,
    job_id: &str,
    public_base: &str,
) -> Result<Option<(OccupancyDescriptor, EsdfGrid)>, EsdfError> {
    // Prefer an explicit point cloud; a splat `.ply`'s gaussian centres are the
    // next best surface sample and are what the viewer already renders.
    let geometry = outputs
        .iter()
        .find(|o| o.kind == "pointcloud")
        .or_else(|| outputs.iter().find(|o| o.kind == "splat"));
    let Some(out) = geometry else {
        return Ok(None);
    };
    let Some(points) = local_ply_points(out, work_root) else {
        return Ok(None);
    };
    let grid = esdf_from_points(
        &points,
        DEFAULT_ESDF_RESOLUTION_M,
        DEFAULT_ESDF_TRUNCATION_M,
    )?;

    let rel = PathBuf::from(job_id).join(format!("esdf-g{generation}.f32"));
    let abs = work_root.join(&rel);
    if let Some(parent) = abs.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let url = match std::fs::write(&abs, grid.to_le_bytes()) {
        Ok(()) => Some(format!(
            "{}/artifacts/{}",
            public_base.trim_end_matches('/'),
            rel.to_string_lossy().replace('\\', "/")
        )),
        Err(e) => {
            // The grid is still valid in memory and the descriptor still states
            // real geometry, so publish it without a fetch URL rather than
            // dropping the whole planning lane on a disk fault.
            tracing::warn!(error = %e, path = %abs.display(), "atlas_esdf_write_failed");
            None
        }
    };

    Ok(Some((
        OccupancyDescriptor {
            session_id: session_id.to_string(),
            generation,
            placement: placement.clone(),
            origin: grid.origin,
            resolution_m: grid.resolution_m,
            dims: grid.dims,
            field: OccupancyField::Esdf,
            truncation_m: grid.truncation_m,
            shm_name: None,
            slot: None,
            seq: None,
            url,
        },
        grid,
    )))
}

fn meta_u64(out: &Output, key: &str) -> Option<u64> {
    out.meta.get(key)?.as_u64()
}

fn meta_str(out: &Output, key: &str) -> Option<String> {
    Some(out.meta.get(key)?.as_str()?.to_string())
}

/// Resolve an output's local `.ply` and parse its positions, or `None` when the
/// output is not a readable local `.ply` (a `mock://` placeholder, an artifact
/// URL whose file is gone, an unsupported layout).
fn local_ply_points(out: &Output, work_root: &Path) -> Option<Vec<[f32; 3]>> {
    let path = local_ply_path(out, work_root)?;
    parse_ply(&path).ok().flatten().map(|p| p.positions)
}

fn ply_point_count(out: &Output, work_root: &Path) -> Option<u64> {
    local_ply_points(out, work_root).map(|p| p.len() as u64)
}

/// The on-disk path of an output's `.ply`, from either its original `file://`
/// URI or the `local_uri` the artifact rewriter preserves.
fn local_ply_path(out: &Output, work_root: &Path) -> Option<PathBuf> {
    let candidates = [
        out.meta.get("local_uri").and_then(|v| v.as_str()),
        Some(out.uri.as_str()),
    ];
    for uri in candidates.into_iter().flatten() {
        if let Some(rest) = uri.strip_prefix("file://") {
            let p = PathBuf::from(rest);
            if p.extension().is_some_and(|e| e == "ply") && p.is_file() {
                return Some(p);
            }
        }
        // An already-rewritten artifact URL: recover the work-root-relative path.
        if let Some((_, rel)) = uri.split_once("/artifacts/") {
            let p = work_root.join(rel);
            if p.extension().is_some_and(|e| e == "ply") && p.is_file() {
                return Some(p);
            }
        }
    }
    None
}

/// Axis-aligned bounds `[min_x, min_y, min_z, max_x, max_y, max_z]`.
fn bounds_of(points: &[[f32; 3]]) -> [f64; 6] {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in points {
        for a in 0..3 {
            let v = p[a] as f64;
            if v < min[a] {
                min[a] = v;
            }
            if v > max[a] {
                max[a] = v;
            }
        }
    }
    if points.is_empty() {
        return [0.0; 6];
    }
    [min[0], min[1], min[2], max[0], max[1], max[2]]
}

#[cfg(test)]
#[path = "world_descriptors_tests.rs"]
mod tests;
