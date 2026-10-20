//! The staged reconstruction pipeline over an uploaded capture session.
//!
//! A `reconstruct` job with a `preset`, a `change` job and a `calibrate` job
//! are *staged*: the worker runs them stage by stage ([`pipeline`]), each
//! stage persisted in the job store so a restarted node resumes at the stage
//! that was interrupted. Every third-party program runs through a
//! [`ToolRunner`] resolved from the pinned tool set.

pub mod chunks;
pub mod chunks_merge;
pub mod colmap;
pub mod export;
pub mod geom;
pub mod glim;
pub mod glim_bag;
pub mod mesh;
pub mod pipeline;
pub mod prepare;
pub mod preset;
pub mod process;
pub mod products_stage;
pub mod products_tour;
pub mod runner;
pub mod session_data;
pub mod sfm;
#[cfg(any(test, feature = "mock"))]
pub mod stand_in;
pub mod train;

use std::path::PathBuf;

pub use pipeline::{run_staged, StageSink};
pub use preset::Preset;
pub use process::ProcessRunner;
pub use products_stage::ArtifactOut;
pub use runner::{RecordingRunner, ResolvedTool, StageError, ToolInvocation, ToolRunner};
#[cfg(any(test, feature = "mock"))]
pub use stand_in::StandInRunner;

use crate::{ComputeJobKind, JobRecord, JobStore};

/// A claimed staged job with its inputs resolved.
#[derive(Debug, Clone, PartialEq)]
pub enum StagedJob {
    Reconstruct {
        session_id: String,
        session_dir: PathBuf,
        preset: Preset,
        /// Placement generation of this reconstruction (1 for the first).
        generation: u32,
        /// Frames per spatial chunk of a large posed session (job param
        /// `chunk_frames`; [`chunks::CHUNK_FRAMES`] when absent).
        chunk_frames: Option<usize>,
        /// GLIM odometry of a LiDAR session (job param `glim_odometry`,
        /// `cpu` when absent).
        glim_odometry: glim::GlimOdometry,
        /// Optional products switched off for this job (job param `products`).
        products: preset::ProductSwitches,
        /// gsplat's per-image appearance model (job param `appearance`,
        /// `affine` when absent).
        appearance: train::Appearance,
        /// How the mesh is built (job param `mesh_method`, `tsdf` when absent).
        mesh_method: mesh::MeshMethod,
    },
    Change {
        a: String,
        b: String,
        /// The jobs whose `out/cloud.laz` are compared.
        a_job: String,
        b_job: String,
    },
    Calibrate {
        session_id: String,
        session_dir: PathBuf,
        request: crate::calib::CalibRequest,
    },
}

/// Whether a job runs on the staged pipeline.
pub fn is_staged(job: &JobRecord) -> bool {
    match job.kind {
        ComputeJobKind::Reconstruct => job.params.get("preset").is_some_and(|p| p.is_string()),
        ComputeJobKind::Change | ComputeJobKind::Calibrate => true,
        ComputeJobKind::PerceptionOffload => false,
    }
}

fn param<'a>(job: &'a JobRecord, key: &str) -> Result<&'a str, StageError> {
    job.params
        .get(key)
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .ok_or_else(|| StageError::input(format!("missing `{key}`")))
}

/// An optional string job param with a fixed set of values: `None` when
/// absent, an input error naming the key when it is not one `parse` takes.
fn enum_param<T>(
    job: &JobRecord,
    key: &str,
    parse: fn(&str) -> Option<T>,
) -> Result<Option<T>, StageError> {
    job.params
        .get(key)
        .map(|v| {
            v.as_str()
                .and_then(parse)
                .ok_or_else(|| StageError::input(format!("unknown `{key}` value {v}")))
        })
        .transpose()
}

/// The directory of a session committed through the upload lane under
/// `sessions_root`, or `E_NOT_UPLOADED`.
pub fn committed_session_dir(
    sessions_root: Option<&std::path::Path>,
    session_id: &str,
) -> Result<PathBuf, StageError> {
    let store = sessions_root.map(|r| crate::bulk::SessionStore::new(r.to_path_buf()));
    match store {
        Some(s) if s.is_committed(session_id) => Ok(s.session_dir(session_id)),
        _ => Err(StageError::new(
            "E_NOT_UPLOADED",
            format!("session {session_id} is not committed on this node"),
        )),
    }
}

/// The newest completed session reconstruction of `session_id`, or
/// `E_NOT_RECONSTRUCTED`.
fn latest_reconstruction(store: &JobStore, session_id: &str) -> Result<String, StageError> {
    store
        .latest_session_outputs(session_id)
        .map_err(StageError::io)?
        .into_iter()
        .find(|o| o.kind == "pointcloud")
        .map(|o| o.job_id)
        .ok_or_else(|| {
            StageError::new(
                "E_NOT_RECONSTRUCTED",
                format!("session {session_id} has no reconstructed point cloud"),
            )
        })
}

/// Resolve a staged job's inputs (under the engine lock).
pub fn prepare_staged(
    store: &JobStore,
    sessions_root: Option<&std::path::Path>,
    job: &JobRecord,
) -> Result<StagedJob, StageError> {
    match job.kind {
        ComputeJobKind::Reconstruct => {
            let session_id = param(job, "session_id")?.to_string();
            let preset_s = param(job, "preset")?;
            let preset = Preset::parse(preset_s)
                .ok_or_else(|| StageError::input(format!("unknown preset `{preset_s}`")))?;
            let session_dir = committed_session_dir(sessions_root, &session_id)?;
            // Fixed at submit; computed only for a job submitted without it.
            let generation = match job.params.get("generation").and_then(|g| g.as_u64()) {
                Some(g) => {
                    u32::try_from(g).map_err(|_| StageError::input("generation out of range"))?
                }
                None => {
                    store
                        .session_generation(&session_id)
                        .map_err(StageError::io)?
                        + 1
                }
            };
            let chunk_frames = job
                .params
                .get("chunk_frames")
                .map(|v| {
                    v.as_u64()
                        .filter(|&n| n > 0)
                        .map(|n| n as usize)
                        .ok_or_else(|| {
                            StageError::input("`chunk_frames` must be a positive integer")
                        })
                })
                .transpose()?;
            let glim_odometry = match job.params.get("glim_odometry") {
                None => glim::GlimOdometry::default(),
                Some(v) => v
                    .as_str()
                    .and_then(glim::GlimOdometry::parse)
                    .ok_or_else(|| {
                        StageError::input("`glim_odometry` must be \"cpu\" or \"gpu\"")
                    })?,
            };
            let products = preset::ProductSwitches::parse(job.params.get("products"))
                .map_err(StageError::input)?;
            let appearance =
                enum_param(job, "appearance", train::Appearance::parse)?.unwrap_or_default();
            let mesh_method =
                enum_param(job, "mesh_method", mesh::MeshMethod::parse)?.unwrap_or_default();
            Ok(StagedJob::Reconstruct {
                session_id,
                session_dir,
                preset,
                generation,
                chunk_frames,
                glim_odometry,
                products,
                appearance,
                mesh_method,
            })
        }
        ComputeJobKind::Change => {
            let (a, b) = (param(job, "a")?.to_string(), param(job, "b")?.to_string());
            Ok(StagedJob::Change {
                a_job: latest_reconstruction(store, &a)?,
                b_job: latest_reconstruction(store, &b)?,
                a,
                b,
            })
        }
        ComputeJobKind::Calibrate => {
            let request: crate::calib::CalibRequest = serde_json::from_value(job.params.clone())
                .map_err(|e| StageError::input(format!("calibrate params: {e}")))?;
            let session_dir = committed_session_dir(sessions_root, &request.session_id)?;
            Ok(StagedJob::Calibrate {
                session_id: request.session_id.clone(),
                session_dir,
                request,
            })
        }
        ComputeJobKind::PerceptionOffload => Err(StageError::input("not a staged job")),
    }
}

/// The pinned tools a staged job needs before it may start.
pub fn required_tools(job: &JobRecord) -> Vec<&'static str> {
    match job.kind {
        ComputeJobKind::Reconstruct => job
            .params
            .get("preset")
            .and_then(|p| p.as_str())
            .and_then(Preset::parse)
            .map(Preset::required_tools)
            .unwrap_or_default(),
        _ => Vec::new(),
    }
}

/// `E_TOOL_MISSING` for the first required tool `runner` cannot resolve.
pub fn check_tools(runner: &dyn ToolRunner, tools: &[&str]) -> Result<(), StageError> {
    for t in tools {
        runner.resolve(t)?;
    }
    Ok(())
}

/// The submit-time tool check of a resolved staged job: the preset's tools,
/// plus GLIM (and, for GPU odometry, the NVIDIA runtime) when the session's
/// rig has a LiDAR.
pub fn check_staged_tools(
    runner: &dyn ToolRunner,
    job: &JobRecord,
    staged: &StagedJob,
) -> Result<(), StageError> {
    check_tools(runner, &required_tools(job))?;
    if let StagedJob::Reconstruct {
        session_dir,
        glim_odometry,
        ..
    } = staged
    {
        let rig: ados_capture_contract::Rig =
            serde_json::from_slice(&std::fs::read(session_dir.join("rig.json"))?)
                .map_err(|e| StageError::input(format!("rig.json: {e}")))?;
        if glim::rig_has_lidar(&rig) {
            glim::check_available(runner, session_dir, *glim_odometry)?;
        }
    }
    Ok(())
}

/// Whether `id` is usable as one plain path component: ASCII letters,
/// digits, `-`, `_` and `.`, not starting with `.`, at most 128 bytes.
pub fn valid_job_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && !id.starts_with('.')
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.'))
}

/// Where a job's working tree lives; refuses an id that is not one plain
/// path component.
pub fn job_dir(work_root: &std::path::Path, job_id: &str) -> Result<PathBuf, StageError> {
    if !valid_job_id(job_id) {
        return Err(StageError::input(format!("invalid job id {job_id:?}")));
    }
    Ok(work_root.join("jobs").join(job_id))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn job_dirs_refuse_anything_but_one_plain_component() {
        let root = std::path::Path::new("/w");
        assert_eq!(
            job_dir(root, "job-1.a_b").unwrap(),
            root.join("jobs").join("job-1.a_b")
        );
        for bad in ["", "..", "../../x", "a/b", "a\\b", ".hidden", "a\nb", "C:x"] {
            assert!(job_dir(root, bad).is_err(), "{bad:?}");
        }
        assert!(job_dir(root, &"a".repeat(129)).is_err());
    }
}
