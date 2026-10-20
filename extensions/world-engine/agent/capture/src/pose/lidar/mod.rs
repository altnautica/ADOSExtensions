//! [`LidarPose`]: KISS-ICP LiDAR odometry for a 3D LiDAR (`solid` or
//! `spinning` scan model; `python/kiss_icp_live`, MIT KISS-ICP) as a
//! supervised child process. A planar LiDAR uses [`super::planar`].
//!
//! Input: [`PoseProvider::feed_lidar`] encodes each scan of the configured
//! LiDAR in the [`wire`] format and hands it to a bounded queue with
//! `try_send`, so it never blocks; a full queue drops the scan and counts it
//! in [`LidarStats`].
//!
//! Output: the child answers each scan with `T_w_l`, the LiDAR pose in its
//! odometry frame (the LiDAR frame at its first scan), and the eigenvalues of
//! the translational ICP Hessian. The world frame is the body frame at the
//! first scan, so the provider publishes `T_world_body = T_body_lidar * T_w_l
//! * inv(T_body_lidar)` ([`supervise::world_from_body`]). Each restart starts
//! a new world frame, so the [`PoseTrack`] is reset on every restart.
//!
//! Health: `Lost` while the child is down (or its environment is being
//! provisioned, see [`setup`]); `Degraded` for the first
//! [`supervise::INIT_POSES`] poses of a run and whenever
//! `quality().degeneracy_ratio` (`eig_min / eig_max`) is below
//! [`supervise::DEGENERATE_RATIO`]; `Good` otherwise.

pub mod setup;
pub mod supervise;
pub mod wire;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_setup;

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use ados_capture_contract::{LidarModel, Rig, RigSensorKind};
use serde_json::{json, Value};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use world_engine_protocol::atlas::PoseSource;

use self::supervise::{supervise, ChildSpec};
use self::wire::encode_scan;
use super::{PoseProvider, PoseTrack};
use crate::lidar_scan::LidarScan;

/// Scans that may wait for the child (~0.2 s at 10 Hz).
pub const SCAN_QUEUE_DEPTH: usize = 2;
/// Config and socket file names inside `work_dir`.
pub const CONFIG_FILE: &str = "kiss_icp_live.json";
pub const SOCKET_FILE: &str = "kiss_icp_live.sock";
/// The provisioned environment's directory inside `work_dir`.
pub const ENV_DIR: &str = "env";

/// How the `kiss_icp_live` project's interpreter is found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Launcher {
    /// The `uv` binary: before the first run (and whenever the shipped lock
    /// changes) `uv sync --frozen --no-dev` builds the environment in
    /// `<work_dir>/env` ([`setup`]); the child is that environment's python.
    Uv(PathBuf),
    /// `<python> -m kiss_icp_live` with an interpreter that already has the
    /// project's dependencies.
    Python(PathBuf),
}

#[derive(Debug, Clone)]
pub struct LidarPoseConfig {
    pub launcher: Launcher,
    /// The `kiss_icp_live` uv project directory (holds `pyproject.toml`,
    /// `uv.lock` and the `kiss_icp_live` package); the child runs with it as
    /// its cwd. Only read.
    pub project_dir: PathBuf,
    pub rig: Rig,
    pub lidar_id: String,
    /// Where the config, socket and provisioned environment live; must be
    /// writable and persistent (the plugin data dir).
    pub work_dir: PathBuf,
    /// KISS-ICP config ([`kiss_config`] of the rig's LiDAR model).
    pub kiss: Value,
}

#[derive(Debug, thiserror::Error)]
pub enum LidarPoseError {
    #[error("rig has no lidar sensor `{0}`")]
    NoLidar(String),
    #[error("writing {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
}

#[derive(Debug, Default)]
pub struct LidarCounters {
    pub scans_dropped: AtomicU64,
    pub scans_rejected: AtomicU64,
    pub restarts: AtomicU64,
    pub setup_failures: AtomicU64,
}

/// Input drop and restart counts since spawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LidarStats {
    /// Scans dropped on a full queue.
    pub scans_dropped: u64,
    /// Scans whose per-point vectors disagreed in length.
    pub scans_rejected: u64,
    /// Child exits (each followed by a restart).
    pub restarts: u64,
    /// Failed environment provisioning attempts (each retried after the
    /// restart backoff).
    pub setup_failures: u64,
}

/// KISS-ICP settings for a 3D LiDAR (`solid` or `spinning` scan model):
/// the model's maximum range, a minimum of the model's or 0.5 m, whichever is
/// larger (returns off the rig itself), per-point deskew. `live` tunes the
/// Hessian normals.
pub fn kiss_config(model: &LidarModel) -> Value {
    json!({
        "data": {
            "max_range": model.max_range_m,
            "min_range": model.min_range_m.max(0.5),
            "deskew": true
        },
        "mapping": { "voxel_size": 0.4, "max_points_per_voxel": 20 },
        "live": { "normal_k": 8, "planarity": 0.3, "min_points": 50 }
    })
}

pub struct LidarPose {
    track: Arc<PoseTrack>,
    lidar_id: String,
    tx: mpsc::Sender<Vec<u8>>,
    counters: Arc<LidarCounters>,
    task: JoinHandle<()>,
}

fn io_err(path: &std::path::Path) -> impl FnOnce(std::io::Error) -> LidarPoseError + '_ {
    move |source| LidarPoseError::Io {
        path: path.to_path_buf(),
        source,
    }
}

impl LidarPose {
    /// Write the config into `work_dir` and start the supervised child. Must
    /// be called inside a tokio runtime.
    pub fn spawn(cfg: LidarPoseConfig) -> Result<Self, LidarPoseError> {
        let sensor = cfg
            .rig
            .sensors
            .iter()
            .find(|s| s.id == cfg.lidar_id && s.kind == RigSensorKind::Lidar)
            .ok_or_else(|| LidarPoseError::NoLidar(cfg.lidar_id.clone()))?;
        std::fs::create_dir_all(&cfg.work_dir).map_err(io_err(&cfg.work_dir))?;
        let config = cfg.work_dir.join(CONFIG_FILE);
        let body = serde_json::to_vec_pretty(&cfg.kiss).expect("JSON values serialise");
        std::fs::write(&config, body).map_err(io_err(&config))?;
        let spec = ChildSpec {
            launcher: cfg.launcher.clone(),
            env_dir: cfg.work_dir.join(ENV_DIR),
            cwd: cfg.project_dir.clone(),
            socket: cfg.work_dir.join(SOCKET_FILE),
            config,
            t_body_lidar: sensor.t_body_sensor.0,
        };
        let (tx, rx) = mpsc::channel(SCAN_QUEUE_DEPTH);
        let track = Arc::new(PoseTrack::new());
        let counters = Arc::new(LidarCounters::default());
        let task = tokio::spawn(supervise(spec, rx, track.clone(), counters.clone()));
        Ok(Self {
            track,
            lidar_id: cfg.lidar_id,
            tx,
            counters,
            task,
        })
    }

    pub fn stats(&self) -> LidarStats {
        let c = &self.counters;
        LidarStats {
            scans_dropped: c.scans_dropped.load(Ordering::Relaxed),
            scans_rejected: c.scans_rejected.load(Ordering::Relaxed),
            restarts: c.restarts.load(Ordering::Relaxed),
            setup_failures: c.setup_failures.load(Ordering::Relaxed),
        }
    }
}

impl Drop for LidarPose {
    /// Aborting the supervisor drops the child handle, which kills the child.
    fn drop(&mut self) {
        self.task.abort();
    }
}

impl PoseProvider for LidarPose {
    fn source(&self) -> PoseSource {
        PoseSource::Lidar
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }

    fn feed_lidar(&self, scan: &LidarScan) {
        if scan.sensor_id != self.lidar_id {
            return;
        }
        // Reserve a permit first so a full queue costs no encoding.
        let Ok(permit) = self.tx.try_reserve() else {
            self.counters.scans_dropped.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let mut msg = Vec::new();
        if !encode_scan(scan, &mut msg) {
            self.counters.scans_rejected.fetch_add(1, Ordering::Relaxed);
            return;
        }
        permit.send(msg);
    }
}
