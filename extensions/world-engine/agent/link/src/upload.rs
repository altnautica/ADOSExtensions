//! The session upload manager: a queue of finished sessions, one uploading at
//! a time to the paired node over the LAN ([`crate::lan_node`]), with
//! per-session progress for `GET /atlas/sessions`.
//!
//! `.uploaded` is written into the session directory only after the node
//! acknowledged the commit (it verified every file against `checksums.b3`);
//! that marker is what makes the session evictable and deletable without
//! `force`. A failed upload resumes from what the node holds on the next try.
//!
//! Auto-upload ([`UploadManager::run_auto`]) enqueues every finished,
//! un-uploaded session when all hold: plugin config `atlas.auto_upload` is
//! true, a node resolves on the LAN and this device holds a usable
//! `sessions.upload` credential for it, the capture service is not recording,
//! and the device is idle or charging ([`crate::power`]). When auto-upload is
//! on but no credential can be picked ([`CredentialGap`]), the waiting sessions
//! read `blocked` with the reason in `error`, and the gap is logged once.

use std::collections::{HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use axum::http::StatusCode;
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use serde_json::json;
use tokio::sync::{watch, Notify};
use world_engine_capture::storage::UPLOADED_MARKER;
use world_engine_protocol::atlas::CaptureState;
use world_engine_transport::{BulkClient, BulkConfig, SessionPlan};

use crate::control::{ControlClient, LiveCapture};
use crate::credentials::{CredentialGap, CredentialStore, SESSION_UPLOAD_LANE};
use crate::host::{config_bool, Host};
use crate::lan_node::resolve_lan;
use crate::power::PowerWatch;
use crate::reply::{codes, ApiError};
use crate::sessions::{scan, LiveSession, SessionDir};
use crate::sleep_or_shutdown;

/// Where session directories live: the capture root as the sandbox sees it.
pub type RootFn = Arc<dyn Fn() -> Option<PathBuf> + Send + Sync>;

/// The plugin config key that turns auto-upload on.
pub const AUTO_UPLOAD_KEY: &str = "atlas.auto_upload";
/// How often the auto-upload gate is evaluated. Auto-queued uploads pause
/// within this of the device becoming busy (armed, recording, discharging).
const AUTO_INTERVAL: Duration = Duration::from_secs(10);
/// A failed session is not auto-enqueued again before this.
const AUTO_RETRY_AFTER: Duration = Duration::from_secs(600);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum UploadState {
    Idle,
    Queued,
    Uploading,
    Committed,
    Failed,
    /// Auto-upload is on but cannot run; `error` names why.
    Blocked,
}

/// The `upload` object of a session row.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct UploadView {
    pub state: UploadState,
    pub bytes_sent: u64,
    pub bytes_total: u64,
    pub node: Option<String>,
    pub error: Option<String>,
}

impl UploadView {
    pub fn idle(bytes_total: u64) -> Self {
        Self {
            state: UploadState::Idle,
            bytes_sent: 0,
            bytes_total,
            node: None,
            error: None,
        }
    }
}

/// What `.uploaded` records about the commit (fields it cannot read are
/// `None`).
#[derive(Debug, Default, Deserialize)]
struct UploadedMarker {
    #[serde(default)]
    node: Option<String>,
    #[serde(default)]
    bytes: Option<u64>,
}

impl UploadedMarker {
    fn read(dir: &Path) -> Self {
        std::fs::read(dir.join(UPLOADED_MARKER))
            .ok()
            .and_then(|b| serde_json::from_slice(&b).ok())
            .unwrap_or_default()
    }
}

struct Job {
    view: UploadView,
    failed_at: Option<Instant>,
    /// Queued by auto-upload: runs only while the auto gate is open.
    auto: bool,
}

struct Inner {
    host: Arc<dyn Host>,
    credentials: CredentialStore,
    root: RootFn,
    cfg: BulkConfig,
    jobs: Mutex<HashMap<String, Job>>,
    queue: Mutex<VecDeque<String>>,
    wake: Notify,
    /// The auto-upload gate as last evaluated (idle or charging, not
    /// recording, a paired node). Auto-queued jobs wait while it is closed and
    /// a running one stops (it resumes from the node's offset later).
    gate: watch::Sender<bool>,
    /// Why auto-upload cannot run although it is on, as last evaluated.
    blocked: Mutex<Option<CredentialGap>>,
}

/// The upload queue and its progress; cheap to clone.
#[derive(Clone)]
pub struct UploadManager {
    inner: Arc<Inner>,
}

impl UploadManager {
    pub fn new(
        host: Arc<dyn Host>,
        credentials: CredentialStore,
        root: RootFn,
        cfg: BulkConfig,
    ) -> Self {
        Self {
            inner: Arc::new(Inner {
                host,
                credentials,
                root,
                cfg,
                jobs: Mutex::default(),
                queue: Mutex::default(),
                wake: Notify::new(),
                gate: watch::channel(false).0,
                blocked: Mutex::default(),
            }),
        }
    }

    /// The capture root, if known.
    pub fn root(&self) -> Option<PathBuf> {
        (self.inner.root)()
    }

    /// The progress of `dir`: the live job, else committed (marker present,
    /// node and bytes as the marker records them) or idle. A session auto-upload
    /// would take (waiting in its queue, or finished and not yet queued) reads
    /// `blocked` while auto-upload is blocked.
    pub fn view(&self, dir: &SessionDir, size_bytes: u64) -> UploadView {
        let blocked = *self.inner.blocked.lock();
        let block = |v: &mut UploadView| {
            if let Some(gap) = blocked {
                v.state = UploadState::Blocked;
                v.error = Some(gap.code().to_string());
            }
        };
        if let Some(job) = self.inner.jobs.lock().get(&dir.id) {
            let mut v = job.view.clone();
            if job.auto && v.state == UploadState::Queued {
                block(&mut v);
            }
            return v;
        }
        let mut v = UploadView::idle(size_bytes);
        if dir.uploaded {
            let marker = UploadedMarker::read(&dir.path);
            v.state = UploadState::Committed;
            v.bytes_total = marker.bytes.unwrap_or(size_bytes);
            v.bytes_sent = v.bytes_total;
            v.node = marker.node;
        } else if dir.finished && !dir.recording {
            block(&mut v);
        }
        v
    }

    /// Whether `id` is queued or uploading.
    pub fn busy(&self, id: &str) -> bool {
        self.inner
            .jobs
            .lock()
            .get(id)
            .is_some_and(|j| matches!(j.view.state, UploadState::Queued | UploadState::Uploading))
    }

    /// Drop the progress of a deleted session.
    pub fn forget(&self, id: &str) {
        self.inner.jobs.lock().remove(id);
    }

    /// Queue `id` at the operator's request (a no-op while it is already
    /// queued or uploading, except that an auto job becomes an operator job).
    pub fn enqueue(&self, id: &str, size_bytes: u64) {
        self.enqueue_as(id, size_bytes, false);
    }

    fn enqueue_as(&self, id: &str, size_bytes: u64, auto: bool) {
        {
            let mut jobs = self.inner.jobs.lock();
            let job = jobs.entry(id.to_string()).or_insert_with(|| Job {
                view: UploadView::idle(size_bytes),
                failed_at: None,
                auto,
            });
            if matches!(job.view.state, UploadState::Queued | UploadState::Uploading) {
                job.auto &= auto;
                self.inner.wake.notify_one();
                return;
            }
            job.auto = auto;
            job.view = UploadView {
                state: UploadState::Queued,
                ..UploadView::idle(size_bytes)
            };
            job.failed_at = None;
        }
        self.inner.queue.lock().push_back(id.to_string());
        self.inner.wake.notify_one();
    }

    /// `POST /atlas/sessions/{id}/upload`: queue a finished session. `live`
    /// is the capture service's open session, which tells a crashed session
    /// (`409 E_NOT_FINISHED`) from one still recording (`409 E_RECORDING`).
    pub fn request(&self, id: &str, live: &LiveSession) -> Result<(), ApiError> {
        let root = self.root().ok_or_else(no_storage)?;
        let dir = SessionDir::open(&root, id).ok_or_else(|| unknown_session(id))?;
        if dir.stale(live) {
            return Err(not_finished());
        }
        if dir.recording {
            return Err(ApiError::new(
                StatusCode::CONFLICT,
                codes::RECORDING,
                "the session is still recording",
            ));
        }
        if !dir.finished {
            return Err(not_finished());
        }
        if !dir.uploaded {
            let size = crate::sessions::dir_size(&dir.path);
            self.enqueue(id, size);
        }
        Ok(())
    }

    fn update(&self, id: &str, f: impl FnOnce(&mut Job)) {
        if let Some(job) = self.inner.jobs.lock().get_mut(id) {
            f(job);
        }
    }

    /// Upload one session; the node's label and the commit on success.
    async fn upload(&self, id: &str) -> Result<String, (Option<String>, String)> {
        let fail = |node: Option<String>, e: ApiError| (node, e.tool_message());
        let root = self.root().ok_or_else(|| fail(None, no_storage()))?;
        let dir = SessionDir::open(&root, id).ok_or_else(|| fail(None, unknown_session(id)))?;
        if !dir.finished {
            return Err((None, not_finished().tool_message()));
        }
        if dir.recording {
            return Err((
                None,
                format!("{}: the session is still recording", codes::RECORDING),
            ));
        }
        let path = dir.path.clone();
        let plan = tokio::task::spawn_blocking(move || SessionPlan::read(&path))
            .await
            .map_err(|e| (None, e.to_string()))?
            .map_err(|e| (None, format!("{}: {e}", e.code())))?;
        let i = &self.inner;
        let node = resolve_lan(i.host.as_ref(), &i.credentials, &SESSION_UPLOAD_LANE)
            .await
            .map_err(|e| fail(None, e))?;
        let label = node.label();
        let credential = node.credential(&i.credentials, &SESSION_UPLOAD_LANE).ok();
        self.update(id, |j| {
            j.view.state = UploadState::Uploading;
            j.view.bytes_total = plan.total_bytes;
            j.view.node = Some(label.clone());
            j.view.error = None;
        });
        let client = BulkClient::new(node.base_url(), credential, i.cfg.clone(), Some(node.pin));
        let this = self.clone();
        let sid = id.to_string();
        let progress = move |n: u64| this.update(&sid, |j| j.view.bytes_sent = n);
        let done = client
            .upload_session(id, &plan, &progress)
            .await
            .map_err(|e| (Some(label.clone()), format!("{}: {e}", e.code())))?;
        let marker = json!({
            "node": label,
            "committed_ms": SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64),
            "files": done.files,
            "bytes": done.bytes,
        });
        tokio::fs::write(dir.path.join(UPLOADED_MARKER), marker.to_string())
            .await
            .map_err(|e| {
                (
                    Some(label.clone()),
                    format!("{}: could not write {UPLOADED_MARKER}: {e}", codes::IO),
                )
            })?;
        Ok(label)
    }

    fn is_auto(&self, id: &str) -> bool {
        self.inner.jobs.lock().get(id).is_some_and(|j| j.auto)
    }

    /// The next runnable job: the first operator job, or the first auto job
    /// while the gate is open.
    fn next_runnable(&self) -> Option<String> {
        let open = *self.inner.gate.borrow();
        let mut q = self.inner.queue.lock();
        let at = q.iter().position(|id| open || !self.is_auto(id))?;
        q.remove(at)
    }

    /// Open or close the auto-upload gate (the auto loop, and tests).
    pub fn set_auto_gate(&self, open: bool) {
        self.inner.gate.send_replace(open);
    }

    /// Run queued uploads, one at a time, until shutdown.
    pub async fn run(self, mut shutdown: watch::Receiver<bool>) {
        let mut gate = self.inner.gate.subscribe();
        loop {
            let Some(id) = self.next_runnable() else {
                tokio::select! {
                    _ = self.inner.wake.notified() => {}
                    _ = gate.changed() => {}
                    r = shutdown.changed() => {
                        if r.is_err() || *shutdown.borrow() {
                            return;
                        }
                    }
                }
                continue;
            };
            let auto = self.is_auto(&id);
            tracing::info!(session = %id, auto, "session upload starting");
            let mut stop_gate = self.inner.gate.subscribe();
            let gate_closed = async {
                if auto {
                    let _ = stop_gate.wait_for(|open| !*open).await;
                } else {
                    std::future::pending::<()>().await;
                }
            };
            let result = tokio::select! {
                r = self.upload(&id) => r,
                _ = gate_closed => {
                    // The device became busy: put the session back at the head
                    // of the queue; the next run resumes from the node's offset.
                    tracing::info!(session = %id, "auto upload paused: device busy");
                    self.update(&id, |j| j.view.state = UploadState::Queued);
                    self.inner.queue.lock().push_front(id);
                    continue;
                }
                _ = shutdown.wait_for(|s| *s) => return,
            };
            match result {
                Ok(node) => {
                    tracing::info!(session = %id, %node, "session uploaded and committed");
                    self.update(&id, |j| {
                        j.view.state = UploadState::Committed;
                        j.view.bytes_sent = j.view.bytes_total;
                        j.view.node = Some(node);
                    });
                }
                Err((node, error)) => {
                    tracing::warn!(session = %id, %error, "session upload failed");
                    self.update(&id, |j| {
                        j.view.state = UploadState::Failed;
                        if node.is_some() {
                            j.view.node = node;
                        }
                        j.view.error = Some(error);
                        j.failed_at = Some(Instant::now());
                    });
                }
            }
        }
    }

    /// Record why auto-upload cannot run (`None`: nothing blocks it), logging
    /// each change once.
    fn note_blocked(&self, gap: Option<CredentialGap>) {
        let mut blocked = self.inner.blocked.lock();
        if *blocked == gap {
            return;
        }
        match gap {
            Some(reason) => {
                tracing::warn!(%reason, "auto-upload blocked: no usable sessions.upload credential for the compute node")
            }
            None => tracing::info!("auto-upload no longer blocked"),
        }
        *blocked = gap;
    }

    /// Whether the auto-upload gate is open now.
    pub(crate) async fn auto_gate(&self, control: &ControlClient, power: &PowerWatch) -> bool {
        let i = &self.inner;
        if !config_bool(i.host.as_ref(), AUTO_UPLOAD_KEY, false).await {
            self.note_blocked(None);
            return false;
        }
        if !power.idle_or_charging() {
            return false;
        }
        let recording = match LiveCapture::from(control.status().await) {
            LiveCapture::Running(s) => {
                !matches!(s.state, CaptureState::Idle | CaptureState::Bagged)
            }
            LiveCapture::NotRunning => false,
            LiveCapture::Unknown => true,
        };
        if recording {
            return false;
        }
        match resolve_lan(i.host.as_ref(), &i.credentials, &SESSION_UPLOAD_LANE).await {
            Ok(node) => {
                let gap = node.credential(&i.credentials, &SESSION_UPLOAD_LANE).err();
                self.note_blocked(gap);
                gap.is_none()
            }
            Err(_) => false,
        }
    }

    /// Enqueue every finished, un-uploaded session not failed recently.
    pub fn enqueue_pending(&self) -> usize {
        let Some(root) = self.root() else { return 0 };
        let mut n = 0;
        for dir in scan(&root) {
            if dir.recording || !dir.finished || dir.uploaded || self.busy(&dir.id) {
                continue;
            }
            let recent_failure = self
                .inner
                .jobs
                .lock()
                .get(&dir.id)
                .and_then(|j| j.failed_at)
                .is_some_and(|t| t.elapsed() < AUTO_RETRY_AFTER);
            if !recent_failure {
                self.enqueue_as(&dir.id, crate::sessions::dir_size(&dir.path), true);
                n += 1;
            }
        }
        n
    }

    /// Evaluate the auto-upload gate every [`AUTO_INTERVAL`] until shutdown.
    pub async fn run_auto(
        self,
        control: ControlClient,
        power: PowerWatch,
        mut shutdown: watch::Receiver<bool>,
    ) {
        loop {
            if sleep_or_shutdown(&mut shutdown, AUTO_INTERVAL).await {
                return;
            }
            let open = self.auto_gate(&control, &power).await;
            self.set_auto_gate(open);
            if open {
                let n = self.enqueue_pending();
                if n > 0 {
                    tracing::info!(sessions = n, "auto-upload queued sessions");
                }
            }
        }
    }
}

pub fn no_storage() -> ApiError {
    ApiError::new(
        StatusCode::SERVICE_UNAVAILABLE,
        codes::NO_STORAGE,
        "the capture root is unknown (no capture-storage sidecar)",
    )
}

pub fn unknown_session(id: &str) -> ApiError {
    ApiError::new(
        StatusCode::NOT_FOUND,
        codes::UNKNOWN_SESSION,
        format!("no session `{id}`"),
    )
}

pub fn not_finished() -> ApiError {
    ApiError::new(
        StatusCode::CONFLICT,
        codes::NOT_FINISHED,
        "the session was left unfinished by its recorder (no manifest)",
    )
}

#[cfg(test)]
#[path = "upload_tests.rs"]
pub(crate) mod tests;
