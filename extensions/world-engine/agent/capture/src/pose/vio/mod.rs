//! [`VioPose`]: Basalt stereo-inertial VIO as a supervised `basalt-ados`
//! child process.
//!
//! Inputs: [`PoseProvider::feed_frame`] pairs the left and right frames as Gray8 luma
//! ([`pairing`]) and [`PoseProvider::feed_imu`] rotates the sensor-bus IMU
//! samples from body FLU into the IMU frame (`v_imu = R_imu_body * v_body`;
//! the samples are the IMU's own measurements expressed in body axes, so no
//! lever-arm term applies). Both encode the [`wire`] format and hand it to a
//! bounded queue with `try_send`, so neither ever blocks; a full queue drops
//! the input and counts it in [`VioStats`].
//!
//! Output: the child prints `T_w_i`, the IMU pose in Basalt's world frame.
//! That world is gravity-aligned (z up, Basalt's gravity is `[0, 0, -9.81]`)
//! with its origin where the estimator initialised and an arbitrary yaw; it
//! is used as the session's local world frame (no geo anchor). The provider
//! publishes `T_world_body = T_w_i * inv(T_body_imu)`. Each restart starts a
//! new world frame, so the [`PoseTrack`] is reset on every restart.
//!
//! Health: `Lost` while the child is down or reports `lost`, `Degraded` while
//! it initialises, `Good` while tracking. `quality().tracked_features` is the
//! optical-flow feature count of the left image.

pub mod calib;
pub mod pairing;
pub mod supervise;
pub mod wire;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_run;

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::Rig;
use parking_lot::Mutex;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use world_engine_protocol::atlas::PoseSource;

use self::calib::{basalt_calib, basalt_config, imu_from_body, CalibError};
use self::pairing::{PairOutcome, StereoPairer};
use self::supervise::{supervise, ChildSpec};
use self::wire::{encode_imu, encode_stereo, HEADER_LEN, IMU_LEN};
use super::{se3, PoseProvider, PoseTrack};
use crate::frame_source::CapturedFrame;

/// IMU batches that may wait for the child.
pub const IMU_QUEUE_DEPTH: usize = 256;
/// Stereo pairs that may wait for the child.
pub const FRAME_QUEUE_DEPTH: usize = 3;

/// Calibration, config and socket file names inside `work_dir`.
pub const CALIB_FILE: &str = "basalt_calib.json";
pub const CONFIG_FILE: &str = "basalt_config.json";
pub const SOCKET_FILE: &str = "basalt-ados.sock";

#[derive(Debug, Clone)]
pub struct VioConfig {
    /// The `basalt-ados` executable.
    pub binary: PathBuf,
    pub rig: Rig,
    pub left_id: String,
    pub right_id: String,
    pub imu_id: String,
    /// Where the calibration, config and input socket are written.
    pub work_dir: PathBuf,
}

#[derive(Debug, thiserror::Error)]
pub enum VioError {
    #[error(transparent)]
    Calib(#[from] CalibError),
    #[error("left and right camera are the same sensor `{0}`")]
    SameCamera(String),
    #[error("writing {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
}

#[derive(Debug, Default)]
pub struct VioCounters {
    pub imu_dropped: AtomicU64,
    pub pairs_dropped: AtomicU64,
    pub frames_rejected: AtomicU64,
    pub restarts: AtomicU64,
}

/// Input drop and restart counts since spawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VioStats {
    /// IMU samples dropped on a full queue.
    pub imu_dropped: u64,
    /// Stereo pairs dropped on a full queue.
    pub pairs_dropped: u64,
    /// Frames of the stereo cameras with no luma plane (Bayer) or a malformed buffer.
    pub frames_rejected: u64,
    /// Child exits (each followed by a restart).
    pub restarts: u64,
}

pub struct VioPose {
    track: Arc<PoseTrack>,
    imu_id: String,
    r_imu_body: [f64; 9],
    pairer: Mutex<StereoPairer>,
    imu_tx: mpsc::Sender<Vec<u8>>,
    frame_tx: mpsc::Sender<Vec<u8>>,
    counters: Arc<VioCounters>,
    task: JoinHandle<()>,
}

fn write_json(path: PathBuf, value: &serde_json::Value) -> Result<PathBuf, VioError> {
    let body = serde_json::to_vec_pretty(value).expect("JSON values serialise");
    std::fs::write(&path, body).map_err(|source| VioError::Io {
        path: path.clone(),
        source,
    })?;
    Ok(path)
}

impl VioPose {
    /// Write the Basalt calibration and config into `work_dir` and start the
    /// supervised child. Must be called inside a tokio runtime.
    pub fn spawn(cfg: VioConfig) -> Result<Self, VioError> {
        if cfg.left_id == cfg.right_id {
            return Err(VioError::SameCamera(cfg.left_id));
        }
        let calib = basalt_calib(&cfg.rig, &cfg.left_id, &cfg.right_id, &cfg.imu_id)?;
        let (r_imu_body, t_imu_body) = imu_from_body(&cfg.rig, &cfg.imu_id)?;
        std::fs::create_dir_all(&cfg.work_dir).map_err(|source| VioError::Io {
            path: cfg.work_dir.clone(),
            source,
        })?;
        let spec = ChildSpec {
            binary: cfg.binary,
            socket: cfg.work_dir.join(SOCKET_FILE),
            calib: write_json(cfg.work_dir.join(CALIB_FILE), &calib)?,
            config: write_json(cfg.work_dir.join(CONFIG_FILE), &basalt_config())?,
            t_imu_body: se3::to_mat4(&r_imu_body, t_imu_body),
        };
        let (imu_tx, imu_rx) = mpsc::channel(IMU_QUEUE_DEPTH);
        let (frame_tx, frame_rx) = mpsc::channel(FRAME_QUEUE_DEPTH);
        let track = Arc::new(PoseTrack::new());
        let counters = Arc::new(VioCounters::default());
        let task = tokio::spawn(supervise(
            spec,
            imu_rx,
            frame_rx,
            track.clone(),
            counters.clone(),
        ));
        Ok(Self {
            track,
            imu_id: cfg.imu_id,
            r_imu_body,
            pairer: Mutex::new(StereoPairer::new(cfg.left_id, cfg.right_id)),
            imu_tx,
            frame_tx,
            counters,
            task,
        })
    }

    pub fn stats(&self) -> VioStats {
        let c = &self.counters;
        VioStats {
            imu_dropped: c.imu_dropped.load(Ordering::Relaxed),
            pairs_dropped: c.pairs_dropped.load(Ordering::Relaxed),
            frames_rejected: c.frames_rejected.load(Ordering::Relaxed),
            restarts: c.restarts.load(Ordering::Relaxed),
        }
    }
}

impl Drop for VioPose {
    /// Aborting the supervisor drops the child handle, which kills the child.
    fn drop(&mut self) {
        self.task.abort();
    }
}

impl PoseProvider for VioPose {
    fn source(&self) -> PoseSource {
        PoseSource::Vio
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }

    fn feed_frame(&self, frame: &CapturedFrame) {
        let msg = match self.pairer.lock().push(frame) {
            PairOutcome::Paired(p) => {
                let mut msg = Vec::new();
                encode_stereo(&mut msg, p.t_ns, p.width, p.height, &p.left, &p.right);
                msg
            }
            PairOutcome::Rejected => {
                self.counters
                    .frames_rejected
                    .fetch_add(1, Ordering::Relaxed);
                return;
            }
            PairOutcome::Ignored | PairOutcome::Held => return,
        };
        if self.frame_tx.try_send(msg).is_err() {
            self.counters.pairs_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn feed_imu(&self, src: &str, batch: &ImuBatch) {
        if src != self.imu_id {
            return;
        }
        let mut msg = Vec::with_capacity(batch.samples.len() * (HEADER_LEN + IMU_LEN));
        let n = encode_imu_batch(&self.r_imu_body, batch, &mut msg);
        if n > 0 && self.imu_tx.try_send(msg).is_err() {
            self.counters.imu_dropped.fetch_add(n, Ordering::Relaxed);
        }
    }
}

/// Append every well-formed sample of `batch` (body FLU) to `out` as IMU
/// messages in the IMU frame; returns how many were written.
pub fn encode_imu_batch(r_imu_body: &[f64; 9], batch: &ImuBatch, out: &mut Vec<u8>) -> u64 {
    let mut n = 0;
    for s in &batch.samples {
        let (Ok(g), Ok(a)) = (
            <[f64; 3]>::try_from(s.gyro.as_slice()),
            <[f64; 3]>::try_from(s.accel.as_slice()),
        ) else {
            continue;
        };
        encode_imu(
            out,
            s.t_ns,
            se3::mat_vec(r_imu_body, g),
            se3::mat_vec(r_imu_body, a),
        );
        n += 1;
    }
    n
}
