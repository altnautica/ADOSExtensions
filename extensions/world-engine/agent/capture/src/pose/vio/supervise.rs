//! Running `basalt-ados` as a supervised child.
//!
//! One run: bind the input socket, spawn the child, accept its connection and
//! send HELLO, then pump the queued IMU and stereo messages into the socket
//! while a separate task turns the child's stdout into [`PoseTrack`]
//! estimates. Input that arrives before the child connects is discarded.
//! When the child exits the track is reset and marked lost, and the next run
//! starts after a backoff that doubles from [`MIN_BACKOFF`] to [`MAX_BACKOFF`]
//! and returns to the minimum after a run of at least [`HEALTHY_RUN`].

use std::path::PathBuf;
use std::process::{ExitStatus, Stdio};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::io::{AsyncBufReadExt, AsyncRead, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tokio::process::Command;
use tokio::sync::mpsc;
use world_engine_protocol::atlas::{PoseSource, VioHealth};

use super::wire::{encode_hello, parse_line, ChildLine, ChildState, ChildStatus};
use super::VioCounters;
use crate::pose::{diagonal_cov, se3, PoseEstimate, PoseQuality, PoseTrack};

pub const MIN_BACKOFF: Duration = Duration::from_secs(1);
pub const MAX_BACKOFF: Duration = Duration::from_secs(30);
pub const HEALTHY_RUN: Duration = Duration::from_secs(60);

/// How long the stdout reader may keep draining after the child exits.
const STDOUT_DRAIN: Duration = Duration::from_secs(1);

/// Everything needed to start the child.
#[derive(Debug, Clone)]
pub struct ChildSpec {
    pub binary: PathBuf,
    pub socket: PathBuf,
    pub calib: PathBuf,
    pub config: PathBuf,
    /// Row-major `T_imu_body = inv(T_body_imu)`.
    pub t_imu_body: [f64; 16],
}

/// Nominal covariance by health: Basalt reports no marginal covariance.
pub fn nominal_cov(health: VioHealth) -> [f64; 36] {
    match health {
        VioHealth::Good => diagonal_cov(0.05, 0.02),
        VioHealth::Degraded => diagonal_cov(0.5, 0.1),
        VioHealth::Lost => diagonal_cov(5.0, 0.5),
    }
}

/// `T_world_body = T_w_i * T_imu_body` as an estimate.
pub fn to_estimate(st: &ChildState, t_imu_body: &[f64; 16], health: VioHealth) -> PoseEstimate {
    let (r_wi, t_wi) = se3::from_mat4(&st.t_w_i);
    let (r_ib, t_ib) = se3::from_mat4(t_imu_body);
    let rt = se3::mat_vec(&r_wi, t_ib);
    PoseEstimate {
        t_ns: st.t_ns,
        r: se3::mat_mul(&r_wi, &r_ib),
        t: [rt[0] + t_wi[0], rt[1] + t_wi[1], rt[2] + t_wi[2]],
        cov: st.cov.unwrap_or_else(|| nominal_cov(health)),
        source: PoseSource::Vio,
        health,
    }
}

fn health_of(status: ChildStatus) -> VioHealth {
    match status {
        ChildStatus::Initializing => VioHealth::Degraded,
        ChildStatus::Tracking => VioHealth::Good,
        ChildStatus::Lost => VioHealth::Lost,
    }
}

/// Apply one stdout line to the track; returns the health after it.
pub fn apply_line(
    track: &PoseTrack,
    t_imu_body: &[f64; 16],
    health: VioHealth,
    line: &str,
) -> VioHealth {
    match parse_line(line) {
        Some(ChildLine::Status(s)) => {
            let h = health_of(s);
            track.set_health(h);
            h
        }
        Some(ChildLine::State(st)) => {
            track.push(to_estimate(&st, t_imu_body, health));
            if st.features.is_some() {
                track.set_quality(PoseQuality {
                    tracked_features: st.features,
                    ..track.quality()
                });
            }
            health
        }
        None => {
            tracing::debug!(line, "basalt-ados: unparsed stdout line");
            health
        }
    }
}

fn mark_down(track: &PoseTrack) {
    track.reset();
    track.set_quality(PoseQuality::default());
    track.set_health(VioHealth::Lost);
}

fn discard(rx: &mut mpsc::Receiver<Vec<u8>>) {
    while rx.try_recv().is_ok() {}
}

/// Restart the child forever; ends only when the owning task is aborted.
pub async fn supervise(
    spec: ChildSpec,
    mut imu_rx: mpsc::Receiver<Vec<u8>>,
    mut frame_rx: mpsc::Receiver<Vec<u8>>,
    track: Arc<PoseTrack>,
    counters: Arc<VioCounters>,
) {
    let mut backoff = MIN_BACKOFF;
    loop {
        mark_down(&track);
        let started = Instant::now();
        match run_once(&spec, &mut imu_rx, &mut frame_rx, &track).await {
            Ok(status) => tracing::warn!(%status, "basalt-ados exited"),
            Err(e) => tracing::warn!(error = %e, "basalt-ados failed"),
        }
        mark_down(&track);
        counters.restarts.fetch_add(1, Ordering::Relaxed);
        if started.elapsed() >= HEALTHY_RUN {
            backoff = MIN_BACKOFF;
        }
        // Input queued while the child is down would be stale: discard it.
        let wait = tokio::time::sleep(backoff);
        tokio::pin!(wait);
        loop {
            tokio::select! {
                _ = &mut wait => break,
                Some(_) = imu_rx.recv() => {}
                Some(_) = frame_rx.recv() => {}
            }
        }
        backoff = (backoff * 2).min(MAX_BACKOFF);
    }
}

async fn read_stdout(out: impl AsyncRead + Unpin, track: Arc<PoseTrack>, t_imu_body: [f64; 16]) {
    let mut lines = BufReader::new(out).lines();
    let mut health = VioHealth::Lost;
    while let Ok(Some(line)) = lines.next_line().await {
        health = apply_line(&track, &t_imu_body, health, &line);
    }
}

async fn log_stderr(err: impl AsyncRead + Unpin) {
    let mut lines = BufReader::new(err).lines();
    while let Ok(Some(line)) = lines.next_line().await {
        tracing::info!(target: "basalt_ados", "{line}");
    }
}

async fn send(conn: &mut Option<UnixStream>, msg: &[u8]) {
    let Some(stream) = conn.as_mut() else { return };
    if let Err(e) = stream.write_all(msg).await {
        tracing::warn!(error = %e, "basalt-ados input socket closed");
        *conn = None;
    }
}

async fn run_once(
    spec: &ChildSpec,
    imu_rx: &mut mpsc::Receiver<Vec<u8>>,
    frame_rx: &mut mpsc::Receiver<Vec<u8>>,
    track: &Arc<PoseTrack>,
) -> std::io::Result<ExitStatus> {
    let _ = std::fs::remove_file(&spec.socket);
    let listener = UnixListener::bind(&spec.socket)?;
    let mut child = Command::new(&spec.binary)
        .arg(&spec.socket)
        .arg(&spec.calib)
        .arg(&spec.config)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()?;
    let stdout = child.stdout.take().expect("stdout is piped");
    let stderr = child.stderr.take().expect("stderr is piped");
    let mut out_task = tokio::spawn(read_stdout(stdout, track.clone(), spec.t_imu_body));
    let err_task = tokio::spawn(log_stderr(stderr));
    discard(imu_rx);
    discard(frame_rx);

    let mut conn: Option<UnixStream> = None;
    let mut connected = false;
    let status = loop {
        tokio::select! {
            biased;
            st = child.wait() => break st,
            acc = listener.accept(), if !connected => {
                let (mut stream, _) = acc?;
                let mut hello = Vec::new();
                encode_hello(&mut hello);
                stream.write_all(&hello).await?;
                conn = Some(stream);
                connected = true;
            }
            Some(m) = imu_rx.recv() => send(&mut conn, &m).await,
            Some(m) = frame_rx.recv() => send(&mut conn, &m).await,
        }
    };
    drop(conn);
    let _ = std::fs::remove_file(&spec.socket);
    if tokio::time::timeout(STDOUT_DRAIN, &mut out_task)
        .await
        .is_err()
    {
        out_task.abort();
    }
    err_task.abort();
    status
}
