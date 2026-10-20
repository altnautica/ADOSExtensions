//! World Engine compute node.
//!
//! The engine behind the extension's `node` service on a workstation or
//! compute node: a SQLite-backed job store, a queue and scheduler with a
//! worker model, the reconstructor and perception-offload traits (with mock
//! backends used in CI), and the master/slave cluster. It is the heavy-compute
//! substrate a drone or GCS pairs with to run reconstruction (gaussian splat /
//! point cloud / mesh / occupancy) and perception offload for NPU-less drones.
//!
//! Real reconstructors and detectors are third-party binaries the workers shell
//! out to, behind the [`Reconstructor`] and [`Detector`] traits. The mock
//! backends keep the whole engine testable with no GPU, no camera, and no
//! network. The job and cluster wire types live in
//! [`world_engine_protocol::compute`]; this crate owns the store, the
//! scheduler, and the backends.

mod api;
mod artifacts;
mod auth;
mod backends;
/// The resumable bulk upload lane for capture sessions.
pub mod bulk;
/// Sensor calibration jobs (Kalibr import, Allan deviation, calibration containers).
pub mod calib;
mod cluster;
mod compute_status;
mod credential_api;
mod engine;
/// Host-GPU reporting for the workstation profile (identity + live utilisation).
/// Public so the node's heartbeat loop can sample it: `world_engine_node::gpu::sample`.
pub mod gpu;
mod heartbeat_sidecar;
mod ingest;
/// Owner routes over a job's deliverables (files, tour bundle, measurement).
pub mod job_files;
/// The job and output shapes the REST API serves.
pub mod job_views;
mod jobs_sync;
mod keyframe_persister;
mod lanes;
mod node_credentials;
mod offload;
mod offload_session_manager;
mod offload_stream;
mod offload_ws;
/// Products built from a finished reconstruction (floorplan, rooms, aisles,
/// change detection, measurement, tour bundle, point-cloud I/O).
pub mod products;
/// The staged reconstruction pipeline over an uploaded capture session.
pub mod recon;
mod reconstructor;
mod rerun_log;
mod rerun_world;
mod scheduler;
#[cfg(unix)]
mod serving_config;
mod session;
mod session_registry;
/// The standalone node (no plugin host): the reconstruction node a Windows
/// workstation runs.
pub mod standalone;
mod store;
/// Persistence of staged-job stages and structured job errors.
pub mod store_stages;
/// The pinned third-party tool set.
pub mod tools;
/// The worker that runs claimed jobs.
pub mod worker;
mod world_descriptors;

pub use api::{build_router, build_router_with_base, ApiState};
pub use artifacts::{
    artifact_router, derive_public_base, resolve_under_root, rewrite_output_to_artifact_url,
};
pub use auth::{admits, job_api_lane, require_job_api, require_lane, ComputeAuth, RateLimiter};
pub use backends::{
    file_uri_to_path, is_apple_silicon, parse_gaussian_count, path_to_file_uri,
    select_reconstructor, CliReconstructor, ReconstructCommand, ReconstructorKind,
    SeededSplatReconstructor, SelectingReconstructor,
};
pub use cluster::Cluster;
pub use compute_status::{compute_status_router, operator_socket_router, LatestComputeStatus};
pub use engine::Engine;
pub use heartbeat_sidecar::{
    status_payload, ComputeHeartbeatSidecar, SlaveEntry, COMPUTE_HEARTBEAT_SIDECAR_VERSION,
};
pub use ingest::{submit_reconstruct_job, AtlasIngest};
pub use jobs_sync::{
    build_atlas_jobs, put_accepted, AtlasJob, AtlasJobEntry, JobsCloudSync, PendingRecord,
    JOBS_COLLECTION,
};
pub use keyframe_persister::{dataset_anchor, dataset_id_for, KeyframePersister};
pub use lanes::{gate, upload_and_artifact_lanes};
#[cfg(unix)]
pub use lanes::{lane_router, AtlasLanes, LaneRoutes};
pub use node_credentials::{
    CredentialError, IssuedCredential, MintedCredential, NodeCredentialStore,
    DEFAULT_NODE_CREDENTIALS_PATH,
};
#[cfg(feature = "onnx")]
pub use offload::OnnxDetector;
pub use offload::{Detection, Detector, FrameRef, MockDetector, NoDetector};
pub use offload_session_manager::{OffloadSessionManager, SessionSpec};
pub use offload_stream::{
    run_offload_session, OffloadFrame, OffloadFrameStream, RtspFrameStream, SessionExit,
    VecFrameStream,
};
pub use offload_ws::{
    offload_ws_path, offload_ws_router, pump_to_broadcaster, DetectionBroadcaster, OFFLOAD_WS_ROUTE,
};
#[cfg(any(test, feature = "mock"))]
pub use reconstructor::MockReconstructor;
pub use reconstructor::{ReconstructOutput, Reconstructor};
pub use rerun_log::{
    log_keyframe, log_mesh, log_occupancy, log_pointcloud, log_splat, RerunArchetype,
    RerunLogEntry, RerunRecording,
};
pub use rerun_world::{build_rerun_output, build_world_recording, RERUN_OUTPUT_FILE};
pub use scheduler::{BackendResult, JobOutcome, Prepared, PreparedInput, Scheduler};
#[cfg(unix)]
pub use serving_config::{load_serving_config, resolve_serving_config, ServingConfig};
pub use session::{LiveReconstructConfig, LiveReconstructDriver};
pub use session_registry::{
    SessionProgress, SessionRegistry, SessionState, SessionStateCounts, SessionView,
    StreamingSession, WorkPriority,
};
pub use store::JobStore;
pub use world_descriptors::{
    derive_descriptors, derive_occupancy, esdf_from_points, EsdfError, EsdfGrid,
    WorldDescriptorSet, DEFAULT_ESDF_RESOLUTION_M, DEFAULT_ESDF_TRUNCATION_M, MAX_ESDF_VOXELS,
};

// Re-export the shared wire contract so callers get one import surface.
pub use world_engine_protocol::compute::{
    CancelResponse, ClusterDescriptor, ComputeGpu, ComputeHeartbeat, ComputeJobKind,
    ComputeJobRequest, ComputeJobState, ComputeJobStatus, ComputeRole, Dataset, JobRecord, Output,
    SlaveDescriptor, SubmitResponse,
};

/// Errors from the compute engine.
#[derive(Debug, thiserror::Error)]
pub enum ComputeError {
    /// The job store failed.
    #[error("store: {0}")]
    Store(#[from] rusqlite::Error),
    /// A params or result value failed to (de)serialize.
    #[error("serde: {0}")]
    Serde(#[from] serde_json::Error),
    /// A reconstructor or detector backend failed.
    #[error("backend {backend}: {message}")]
    Backend { backend: String, message: String },
    /// A job, dataset, or output id was not found.
    #[error("not found: {0}")]
    NotFound(String),
    /// An id already exists (a duplicate submit) — distinct from a store fault.
    #[error("conflict: {0}")]
    Conflict(String),
    /// The job kind does not match the backend it was dispatched to.
    #[error("wrong job kind for {0}")]
    WrongKind(String),
}
