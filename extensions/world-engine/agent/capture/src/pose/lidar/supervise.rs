//! Running `kiss_icp_live` as a supervised child.
//!
//! One run: bind the socket, spawn the child, accept its connection, then
//! write the queued scan frames into it while a task turns the JSON lines it
//! writes back into [`PoseTrack`] estimates. Scans queued before the child
//! connects are discarded. When the child exits (or its socket closes) the
//! track is reset and marked lost, and the next run starts after a backoff
//! that doubles from [`MIN_BACKOFF`] to [`MAX_BACKOFF`] and returns to the
//! minimum after a run of at least [`HEALTHY_RUN`].

use std::path::PathBuf;
use std::process::{ExitStatus, Stdio};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::io::{AsyncBufReadExt, AsyncRead, AsyncWriteExt, BufReader};
use tokio::net::unix::OwnedWriteHalf;
use tokio::net::UnixListener;
use tokio::process::Command;
use tokio::sync::mpsc;
use world_engine_protocol::atlas::{PoseSource, VioHealth};

use super::wire::{parse_line, ChildPose};
use super::LidarCounters;
use crate::pose::{diagonal_cov, se3, PoseEstimate, PoseQuality, PoseTrack};

pub const MIN_BACKOFF: Duration = Duration::from_secs(1);
pub const MAX_BACKOFF: Duration = Duration::from_secs(30);
pub const HEALTHY_RUN: Duration = Duration::from_secs(60);
/// Poses after a (re)start that are reported `Degraded` while the local map
/// fills.
pub const INIT_POSES: u32 = 3;
/// `eig_min / eig_max` below this is a degenerate (corridor-like) scan.
pub const DEGENERATE_RATIO: f64 = 0.05;

/// How long the result reader may keep draining after the child exits.
const DRAIN: Duration = Duration::from_secs(1);

/// The child's command line and the extrinsics its output is mapped with.
#[derive(Debug, Clone)]
pub struct ChildSpec {
    pub program: PathBuf,
    /// Arguments before `--socket <path> --config <path>`.
    pub args: Vec<String>,
    pub cwd: PathBuf,
    pub socket: PathBuf,
    pub config: PathBuf,
    /// Row-major `T_body_lidar`.
    pub t_body_lidar: [f64; 16],
}

/// Nominal covariance by health: KISS-ICP reports none.
pub fn nominal_cov(health: VioHealth) -> [f64; 36] {
    match health {
        VioHealth::Good => diagonal_cov(0.05, 0.01),
        VioHealth::Degraded => diagonal_cov(0.5, 0.05),
        VioHealth::Lost => diagonal_cov(5.0, 0.5),
    }
}

fn mat4_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let (ra, ta) = se3::from_mat4(a);
    let (rb, tb) = se3::from_mat4(b);
    let t = se3::mat_vec(&ra, tb);
    se3::to_mat4(
        &se3::mat_mul(&ra, &rb),
        [t[0] + ta[0], t[1] + ta[1], t[2] + ta[2]],
    )
}

fn mat4_inv(m: &[f64; 16]) -> [f64; 16] {
    let (r, t) = se3::from_mat4(m);
    let rt = se3::transpose(&r);
    let ti = se3::mat_vec(&rt, t);
    se3::to_mat4(&rt, [-ti[0], -ti[1], -ti[2]])
}

/// `T_world_body` from the odometry's `T_w_l`.
///
/// The world frame is the body frame at the child's first scan:
/// `T_world_w = T_body_lidar`, so `T_world_body = T_body_lidar * T_w_l *
/// inv(T_body_lidar)` and the body starts at the origin with identity
/// attitude.
pub fn world_from_body(t_w_l: &[f64; 16], t_body_lidar: &[f64; 16]) -> [f64; 16] {
    mat4_mul(&mat4_mul(t_body_lidar, t_w_l), &mat4_inv(t_body_lidar))
}

/// Health of one result, `seen` results into the run (1-based).
pub fn health_of(p: &ChildPose, seen: u32) -> VioHealth {
    match p.degeneracy_ratio() {
        _ if seen <= INIT_POSES => VioHealth::Degraded,
        Some(r) if r >= DEGENERATE_RATIO => VioHealth::Good,
        _ => VioHealth::Degraded,
    }
}

/// Apply one result line to the track; returns whether it was a pose.
pub fn apply_line(track: &PoseTrack, t_body_lidar: &[f64; 16], seen: u32, line: &str) -> bool {
    let Some(p) = parse_line(line) else {
        tracing::debug!(line, "kiss_icp_live: unparsed line");
        return false;
    };
    let health = health_of(&p, seen);
    let (r, t) = se3::from_mat4(&world_from_body(&p.t_w_l, t_body_lidar));
    track.set_quality(PoseQuality {
        degeneracy_ratio: p.degeneracy_ratio(),
        ..PoseQuality::default()
    });
    track.push(PoseEstimate {
        t_ns: p.t_ns,
        r,
        t,
        cov: nominal_cov(health),
        source: PoseSource::Lidar,
        health,
    });
    true
}

fn mark_down(track: &PoseTrack) {
    track.reset();
    track.set_quality(PoseQuality::default());
    track.set_health(VioHealth::Lost);
}

/// Restart the child forever; ends only when the owning task is aborted.
pub async fn supervise(
    spec: ChildSpec,
    mut rx: mpsc::Receiver<Vec<u8>>,
    track: Arc<PoseTrack>,
    counters: Arc<LidarCounters>,
) {
    let mut backoff = MIN_BACKOFF;
    loop {
        mark_down(&track);
        let started = Instant::now();
        match run_once(&spec, &mut rx, &track).await {
            Ok(status) => tracing::warn!(%status, "kiss_icp_live exited"),
            Err(e) => tracing::warn!(error = %e, "kiss_icp_live failed"),
        }
        mark_down(&track);
        counters.restarts.fetch_add(1, Ordering::Relaxed);
        if started.elapsed() >= HEALTHY_RUN {
            backoff = MIN_BACKOFF;
        }
        // Scans queued while the child is down would be stale: discard them.
        let wait = tokio::time::sleep(backoff);
        tokio::pin!(wait);
        loop {
            tokio::select! {
                _ = &mut wait => break,
                Some(_) = rx.recv() => {}
            }
        }
        backoff = (backoff * 2).min(MAX_BACKOFF);
    }
}

async fn read_results(rd: impl AsyncRead + Unpin, track: Arc<PoseTrack>, t_body_lidar: [f64; 16]) {
    let mut lines = BufReader::new(rd).lines();
    let mut seen = 0u32;
    while let Ok(Some(line)) = lines.next_line().await {
        if apply_line(&track, &t_body_lidar, seen.saturating_add(1), &line) {
            seen = seen.saturating_add(1);
        }
    }
}

async fn log_lines(out: impl AsyncRead + Unpin) {
    let mut lines = BufReader::new(out).lines();
    while let Ok(Some(line)) = lines.next_line().await {
        tracing::info!(target: "kiss_icp_live", "{line}");
    }
}

async fn send(conn: &mut Option<OwnedWriteHalf>, msg: &[u8]) -> bool {
    let Some(w) = conn.as_mut() else { return true };
    if let Err(e) = w.write_all(msg).await {
        tracing::warn!(error = %e, "kiss_icp_live socket closed");
        *conn = None;
        return false;
    }
    true
}

async fn run_once(
    spec: &ChildSpec,
    rx: &mut mpsc::Receiver<Vec<u8>>,
    track: &Arc<PoseTrack>,
) -> std::io::Result<ExitStatus> {
    let _ = std::fs::remove_file(&spec.socket);
    let listener = UnixListener::bind(&spec.socket)?;
    let mut child = Command::new(&spec.program)
        .args(&spec.args)
        .arg("--socket")
        .arg(&spec.socket)
        .arg("--config")
        .arg(&spec.config)
        .current_dir(&spec.cwd)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()?;
    let out_log = tokio::spawn(log_lines(child.stdout.take().expect("stdout is piped")));
    let err_log = tokio::spawn(log_lines(child.stderr.take().expect("stderr is piped")));
    while rx.try_recv().is_ok() {}

    let mut conn: Option<OwnedWriteHalf> = None;
    let mut reader: Option<tokio::task::JoinHandle<()>> = None;
    let mut reader_done = false;
    let status = loop {
        tokio::select! {
            biased;
            st = child.wait() => break st,
            acc = listener.accept(), if reader.is_none() => {
                let (rd, wr) = acc?.0.into_split();
                reader = Some(tokio::spawn(read_results(rd, track.clone(), spec.t_body_lidar)));
                conn = Some(wr);
            }
            _ = async { reader.as_mut().expect("guarded").await }, if reader.is_some() && !reader_done => {
                // The child closed its end: it is exiting or wedged.
                reader_done = true;
                tracing::warn!("kiss_icp_live closed its socket");
                let _ = child.start_kill();
            }
            Some(m) = rx.recv() => {
                if !send(&mut conn, &m).await {
                    let _ = child.start_kill();
                }
            }
        }
    };
    drop(conn);
    let _ = std::fs::remove_file(&spec.socket);
    if let (Some(mut r), false) = (reader, reader_done) {
        if tokio::time::timeout(DRAIN, &mut r).await.is_err() {
            r.abort();
        }
    }
    out_log.abort();
    err_log.abort();
    status
}
