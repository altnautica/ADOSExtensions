//! The upload manager against an in-process stand-in node.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use axum::body::Bytes;
use axum::extract::{Path as UrlPath, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{post, put};
use axum::{Json, Router};
use parking_lot::Mutex;
use serde_json::json;
use world_engine_capture::storage::{RECORDING_MARKER, UPLOADED_MARKER};

use super::*;
use crate::compute_node::PIN_KEY;
use crate::testing::FakeHost;

#[derive(Default)]
pub struct FakeNode {
    pub files: HashMap<String, Vec<u8>>,
    /// Whether the commit is accepted.
    pub commit_ok: bool,
    pub commits: u32,
    /// The session's `.uploaded` marker, checked when each commit arrives.
    pub marker: PathBuf,
    pub marker_at_commit: Vec<bool>,
}

pub type SharedNode = Arc<Mutex<FakeNode>>;

async fn manifest(State(n): State<SharedNode>, body: Bytes) -> Response {
    n.lock().files.insert("manifest.json".into(), body.to_vec());
    (StatusCode::CREATED, Json(json!({"ok": true}))).into_response()
}

async fn head(
    State(n): State<SharedNode>,
    UrlPath((_id, rel)): UrlPath<(String, String)>,
) -> Response {
    match n.lock().files.get(&rel) {
        Some(b) => (StatusCode::OK, [("upload-offset", b.len().to_string())]).into_response(),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

async fn put_file(
    State(n): State<SharedNode>,
    UrlPath((_id, rel)): UrlPath<(String, String)>,
    headers: HeaderMap,
    body: Bytes,
) -> Response {
    let start: usize = headers
        .get(header::CONTENT_RANGE)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("bytes "))
        .and_then(|v| v.split('-').next())
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let mut n = n.lock();
    let held = n.files.entry(rel).or_default();
    if held.len() != start {
        return (
            StatusCode::CONFLICT,
            Json(json!({"error": "E_OFFSET_MISMATCH", "received": held.len()})),
        )
            .into_response();
    }
    held.extend_from_slice(&body);
    Json(json!({"received": held.len()})).into_response()
}

async fn commit(State(n): State<SharedNode>) -> Response {
    let mut n = n.lock();
    n.commits += 1;
    let seen = n.marker.exists();
    n.marker_at_commit.push(seen);
    if n.commit_ok {
        let bytes: usize = n.files.values().map(Vec::len).sum();
        Json(json!({"ok": true, "files": n.files.len(), "bytes": bytes})).into_response()
    } else {
        (
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({"error": "E_CHECKSUM_MISMATCH", "files": ["rig.json"]})),
        )
            .into_response()
    }
}

/// Serve a stand-in node on loopback; its `host:port`.
pub async fn serve_node(node: SharedNode) -> String {
    let app = Router::new()
        .route("/api/compute/sessions/:id", post(manifest))
        .route("/api/compute/sessions/:id/commit", post(commit))
        .route(
            "/api/compute/sessions/:id/files/*rel",
            put(put_file).head(head),
        )
        .with_state(node);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    addr.to_string()
}

/// A finished session `id` under `root`.
pub fn finished_session(root: &Path, id: &str) -> PathBuf {
    let dir = root.join(id);
    std::fs::create_dir_all(dir.join("segments")).unwrap();
    std::fs::write(dir.join("manifest.json"), br#"{"session_id":"x"}"#).unwrap();
    std::fs::write(dir.join("rig.json"), b"{}").unwrap();
    std::fs::write(dir.join("segments/000000.mcap"), vec![3u8; 5000]).unwrap();
    let h = "0".repeat(64);
    std::fs::write(
        dir.join("checksums.b3"),
        format!("{h}  manifest.json\n{h}  rig.json\n{h}  segments/000000.mcap\n"),
    )
    .unwrap();
    dir
}

pub fn fast() -> BulkConfig {
    BulkConfig {
        chunk_bytes: 1024,
        max_retries: 2,
        backoff_base: Duration::from_millis(1),
        backoff_max: Duration::from_millis(2),
        ..BulkConfig::default()
    }
}

fn manager(host: Arc<FakeHost>, root: &Path) -> UploadManager {
    let root = root.to_path_buf();
    UploadManager::new(
        host,
        CredentialStore { path: None },
        Arc::new(move || Some(root.clone())),
        fast(),
    )
}

async fn settle(m: &UploadManager, dir: &SessionDir) -> UploadView {
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let v = m.view(dir, 0);
            if matches!(v.state, UploadState::Committed | UploadState::Failed) {
                return v;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("the upload settled")
}

#[tokio::test]
async fn the_marker_is_written_only_after_the_commit_ack() {
    let root = tempfile::tempdir().unwrap();
    let id = "visor-1-20261019T101500Z-3fa2";
    let dir_path = finished_session(root.path(), id);
    let node: SharedNode = Arc::default();
    node.lock().marker = dir_path.join(UPLOADED_MARKER);
    let addr = serve_node(node.clone()).await;
    let host = FakeHost::new();
    host.set_config(PIN_KEY, json!(addr));
    let m = manager(host, root.path());
    let (_stop, rx) = watch::channel(false);
    tokio::spawn(m.clone().run(rx));

    // A refused commit leaves no marker and a failed row.
    m.request(id, &LiveSession::None).unwrap();
    let dir = SessionDir::open(root.path(), id).unwrap();
    let v = settle(&m, &dir).await;
    assert_eq!(v.state, UploadState::Failed);
    assert!(v
        .error
        .as_deref()
        .unwrap()
        .starts_with("E_CHECKSUM_MISMATCH"));
    assert_eq!(v.node.as_deref(), Some(addr.as_str()));
    assert!(!dir_path.join(UPLOADED_MARKER).exists());

    // Accepted: the marker follows the ACK.
    node.lock().commit_ok = true;
    m.request(id, &LiveSession::None).unwrap();
    let v = settle(&m, &dir).await;
    assert_eq!(v.state, UploadState::Committed, "{v:?}");
    assert_eq!(v.bytes_sent, v.bytes_total);
    assert!(v.bytes_total > 5000);
    assert!(dir_path.join(UPLOADED_MARKER).exists());
    let n = node.lock();
    assert!(n.marker_at_commit.iter().all(|seen| !seen));
    assert_eq!(n.files["segments/000000.mcap"], vec![3u8; 5000]);
    drop(n);

    // After a link restart (a fresh manager, no live job) the committed row
    // keeps the node and bytes the marker recorded.
    let restarted = manager(FakeHost::new(), root.path());
    let dir = SessionDir::open(root.path(), id).unwrap();
    let after = restarted.view(&dir, 7);
    assert_eq!(after.state, UploadState::Committed);
    assert_eq!(after.node.as_deref(), Some(addr.as_str()));
    assert_eq!(after.bytes_total, v.bytes_total);
    assert_eq!(after.bytes_sent, v.bytes_total);
}

#[test]
fn a_marker_without_fields_lists_committed_with_the_directory_size() {
    let root = tempfile::tempdir().unwrap();
    let id = "visor-1-20261019T101500Z-0003";
    let dir_path = finished_session(root.path(), id);
    std::fs::write(dir_path.join(UPLOADED_MARKER), b"").unwrap();
    let m = manager(FakeHost::new(), root.path());
    let v = m.view(&SessionDir::open(root.path(), id).unwrap(), 42);
    assert_eq!(v.state, UploadState::Committed);
    assert_eq!(v.node, None);
    assert_eq!((v.bytes_sent, v.bytes_total), (42, 42));
}

#[tokio::test]
async fn recording_and_unknown_sessions_are_refused() {
    let root = tempfile::tempdir().unwrap();
    let id = "drone-2-20261019T101500Z-0001";
    let dir = finished_session(root.path(), id);
    std::fs::write(dir.join(RECORDING_MARKER), b"").unwrap();
    let m = manager(FakeHost::new(), root.path());
    let none = LiveSession::None;
    assert_eq!(m.request(id, &none).unwrap_err().code, codes::RECORDING);
    assert_eq!(
        m.request("nope", &none).unwrap_err().code,
        codes::UNKNOWN_SESSION
    );
    // Left by a crashed recorder: not finished, never uploaded.
    let crashed = "drone-2-20261019T101500Z-0002";
    let c = root.path().join(crashed);
    std::fs::create_dir_all(&c).unwrap();
    std::fs::write(c.join(RECORDING_MARKER), b"").unwrap();
    crate::sessions::age_marker(&c, 600);
    assert_eq!(
        m.request(crashed, &none).unwrap_err().code,
        codes::NOT_FINISHED
    );
    // The capture service's open session is recording.
    assert_eq!(
        m.request(id, &LiveSession::Open(id.into()))
            .unwrap_err()
            .code,
        codes::RECORDING
    );
    assert!(!m.busy(id));
}

#[tokio::test]
async fn auto_upload_queues_only_finished_unuploaded_sessions() {
    let root = tempfile::tempdir().unwrap();
    finished_session(root.path(), "a-20261019T101500Z-0001");
    let done = finished_session(root.path(), "b-20261019T101500Z-0002");
    std::fs::write(done.join(UPLOADED_MARKER), b"{}").unwrap();
    let live = finished_session(root.path(), "c-20261019T101500Z-0003");
    std::fs::write(live.join(RECORDING_MARKER), b"").unwrap();
    let m = manager(FakeHost::new(), root.path());
    assert_eq!(m.enqueue_pending(), 1);
    assert!(m.busy("a-20261019T101500Z-0001"));
    // Already queued: not queued twice.
    assert_eq!(m.enqueue_pending(), 0);
}

#[tokio::test]
async fn auto_jobs_wait_while_the_gate_is_closed_and_operator_jobs_do_not() {
    let root = tempfile::tempdir().unwrap();
    let auto_id = "a-20261019T101500Z-0001";
    let op_id = "b-20261019T101500Z-0002";
    let auto_dir = finished_session(root.path(), auto_id);
    finished_session(root.path(), op_id);
    let node: SharedNode = Arc::default();
    node.lock().commit_ok = true;
    node.lock().marker = auto_dir.join(UPLOADED_MARKER);
    let addr = serve_node(node.clone()).await;
    let host = FakeHost::new();
    host.set_config(PIN_KEY, json!(addr));
    let m = manager(host, root.path());
    let (_stop, rx) = watch::channel(false);
    tokio::spawn(m.clone().run(rx));

    // The backlog is queued while the gate was open, then the device turns busy.
    m.set_auto_gate(false);
    assert_eq!(m.enqueue_pending(), 2);
    // An operator request runs regardless of the gate.
    m.request(op_id, &LiveSession::None).unwrap();
    let op = SessionDir::open(root.path(), op_id).unwrap();
    assert_eq!(settle(&m, &op).await.state, UploadState::Committed);
    tokio::time::sleep(Duration::from_millis(200)).await;
    let a = SessionDir::open(root.path(), auto_id).unwrap();
    assert_eq!(
        m.view(&a, 0).state,
        UploadState::Queued,
        "auto job waits for the gate"
    );
    assert!(!auto_dir.join(UPLOADED_MARKER).exists());

    m.set_auto_gate(true);
    assert_eq!(settle(&m, &a).await.state, UploadState::Committed);
}

#[tokio::test]
async fn auto_upload_without_a_usable_credential_reads_blocked_with_the_reason() {
    use world_engine_protocol::node_credential::{InstalledCredential, WorkstationCredentials};
    let root = tempfile::tempdir().unwrap();
    let id = "a-20261019T101500Z-0001";
    finished_session(root.path(), id);
    let host = FakeHost::new();
    host.set_config(AUTO_UPLOAD_KEY, json!(true));
    host.set_config(PIN_KEY, json!("127.0.0.1:8092"));
    let creds = root.path().join("creds.json");
    let root_path = root.path().to_path_buf();
    let m = UploadManager::new(
        host.clone(),
        CredentialStore {
            path: Some(creds.clone()),
        },
        Arc::new(move || Some(root_path.clone())),
        fast(),
    );
    let control = ControlClient::new(root.path().join("none.sock"));
    let power = PowerWatch::default();
    let dir = SessionDir::open(root.path(), id).unwrap();

    // Nothing installed: the gate stays shut and the session says why.
    assert!(!m.auto_gate(&control, &power).await);
    let v = m.view(&dir, 0);
    assert_eq!(
        (v.state, v.error.as_deref()),
        (UploadState::Blocked, Some("no_credential"))
    );
    assert_eq!(serde_json::to_value(&v).unwrap()["state"], "blocked");

    // Two equally good credentials: ambiguous, never a guess.
    let mut store = WorkstationCredentials::default();
    for (node, secret) in [("ws-a", "nc1.A.SECRET"), ("ws-b", "nc1.B.SECRET")] {
        store.upsert(InstalledCredential {
            workstation_node_id: node.into(),
            credential: secret.into(),
            lanes: vec![SESSION_UPLOAD_LANE],
            installed_at_ms: 7,
            node_addr: None,
        });
    }
    store.save(&creds).unwrap();
    assert!(!m.auto_gate(&control, &power).await);
    assert_eq!(
        m.view(&dir, 0).error.as_deref(),
        Some("ambiguous_credential")
    );
    // A queued auto job reads blocked too.
    m.enqueue_pending();
    assert_eq!(m.view(&dir, 0).state, UploadState::Blocked);

    // One of them minted at the pinned address: the gate opens.
    store.workstations[1].node_addr = Some("127.0.0.1".into());
    store.save(&creds).unwrap();
    assert!(m.auto_gate(&control, &power).await);
    let v = m.view(&dir, 0);
    assert_eq!((v.state, v.error), (UploadState::Queued, None));

    // Auto-upload off: nothing is blocked, whatever the store holds.
    std::fs::remove_file(&creds).unwrap();
    host.set_config(AUTO_UPLOAD_KEY, json!(false));
    assert!(!m.auto_gate(&control, &power).await);
    assert_eq!(m.view(&dir, 0).state, UploadState::Queued);
}
