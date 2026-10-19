//! The bulk client against an in-process stand-in of the node's upload lane.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::body::Bytes;
use axum::extract::{Path as UrlPath, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{post, put};
use axum::{Json, Router};
use serde_json::json;

use super::*;
use crate::bulk_plan::{BulkConfig, BulkError, SessionPlan, CHECKSUMS_FILE};

#[derive(Default)]
struct Node {
    manifest: Option<Vec<u8>>,
    /// Bytes received per file, partial or complete.
    files: HashMap<String, Vec<u8>>,
    puts: HashMap<String, u32>,
    log: Vec<String>,
    committed: bool,
    /// `(rel, nth PUT)`: keep half that chunk, then fail the request as a
    /// dropped link would.
    drop_at: Option<(String, u32)>,
    /// The complete contents the commit checks.
    expected: HashMap<String, Vec<u8>>,
}

type Shared = Arc<Mutex<Node>>;

fn err(status: StatusCode, body: serde_json::Value) -> Response {
    (status, Json(body)).into_response()
}

async fn post_manifest(
    State(n): State<Shared>,
    UrlPath(_id): UrlPath<String>,
    body: Bytes,
) -> Response {
    let mut n = n.lock().unwrap();
    n.log.push("POST manifest".into());
    if n.committed {
        return err(
            StatusCode::CONFLICT,
            json!({"error": "E_ALREADY_COMMITTED"}),
        );
    }
    n.manifest = Some(body.to_vec());
    (StatusCode::CREATED, Json(json!({"ok": true}))).into_response()
}

async fn head_file(
    State(n): State<Shared>,
    UrlPath((_id, rel)): UrlPath<(String, String)>,
) -> Response {
    let mut n = n.lock().unwrap();
    n.log.push(format!("HEAD {rel}"));
    match n.files.get(&rel) {
        Some(b) => (StatusCode::OK, [("upload-offset", b.len().to_string())]).into_response(),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

fn range_start(headers: &HeaderMap) -> u64 {
    headers
        .get(header::CONTENT_RANGE)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("bytes "))
        .and_then(|v| v.split('-').next())
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

async fn put_file(
    State(n): State<Shared>,
    UrlPath((_id, rel)): UrlPath<(String, String)>,
    headers: HeaderMap,
    body: Bytes,
) -> Response {
    let mut n = n.lock().unwrap();
    let start = range_start(&headers);
    n.log.push(format!("PUT {rel} {start}"));
    let held = n.files.get(&rel).map_or(0, Vec::len) as u64;
    if start != held {
        return err(
            StatusCode::CONFLICT,
            json!({"error": "E_OFFSET_MISMATCH", "received": held}),
        );
    }
    let nth = {
        let c = n.puts.entry(rel.clone()).or_insert(0);
        *c += 1;
        *c
    };
    let drop = n
        .drop_at
        .as_ref()
        .is_some_and(|(r, k)| *r == rel && *k == nth);
    let keep = if drop { body.len() / 2 } else { body.len() };
    n.files
        .entry(rel.clone())
        .or_default()
        .extend_from_slice(&body[..keep]);
    if drop {
        return err(StatusCode::BAD_GATEWAY, json!({"error": "link dropped"}));
    }
    let received = n.files[&rel].len();
    Json(json!({"received": received, "complete": true})).into_response()
}

async fn commit(State(n): State<Shared>, UrlPath(_id): UrlPath<String>) -> Response {
    let mut n = n.lock().unwrap();
    n.log.push("COMMIT".into());
    if !n.committed {
        let missing: Vec<String> = n
            .expected
            .iter()
            .filter(|(rel, want)| n.files.get(*rel) != Some(*want))
            .map(|(rel, _)| rel.clone())
            .collect();
        if !missing.is_empty() {
            return err(
                StatusCode::UNPROCESSABLE_ENTITY,
                json!({"error": "E_INCOMPLETE", "files": missing}),
            );
        }
        n.committed = true;
    }
    let bytes: usize = n.files.values().map(Vec::len).sum();
    Json(json!({"ok": true, "files": n.files.len(), "bytes": bytes})).into_response()
}

async fn serve(node: Shared) -> String {
    let app = Router::new()
        .route("/api/compute/sessions/:id", post(post_manifest))
        .route("/api/compute/sessions/:id/commit", post(commit))
        .route(
            "/api/compute/sessions/:id/files/*rel",
            put(put_file).head(head_file),
        )
        .with_state(node);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    format!("http://{addr}")
}

fn fast() -> BulkConfig {
    BulkConfig {
        chunk_bytes: 4096,
        max_retries: 3,
        backoff_base: Duration::from_millis(1),
        backoff_max: Duration::from_millis(5),
        ..BulkConfig::default()
    }
}

/// A finished session: manifest, rig, a 10 000-byte segment, an empty file
/// and the checksum list. Returns the files the node must end up with.
fn session(dir: &std::path::Path) -> HashMap<String, Vec<u8>> {
    let segment: Vec<u8> = (0..10_000u32).map(|i| (i % 251) as u8).collect();
    let files: Vec<(&str, Vec<u8>)> = vec![
        ("rig.json", b"{\"rig\":1}".to_vec()),
        ("segments/000000.mcap", segment),
        ("keyframes/empty.bin", Vec::new()),
    ];
    std::fs::write(dir.join("manifest.json"), b"{\"session\":1}").unwrap();
    let h = "0".repeat(64);
    let mut listing = format!("{h}  manifest.json\n");
    let mut expected = HashMap::new();
    for (rel, body) in files {
        let path = dir.join(rel);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &body).unwrap();
        listing.push_str(&format!("{h}  {rel}\n"));
        expected.insert(rel.to_string(), body);
    }
    std::fs::write(dir.join(CHECKSUMS_FILE), &listing).unwrap();
    expected.insert(CHECKSUMS_FILE.to_string(), listing.into_bytes());
    expected
}

#[tokio::test]
async fn a_dropped_chunk_resumes_from_what_the_node_holds() {
    let dir = tempfile::tempdir().unwrap();
    let expected = session(dir.path());
    let node: Shared = Arc::default();
    {
        let mut n = node.lock().unwrap();
        n.expected = expected.clone();
        n.drop_at = Some(("segments/000000.mcap".into(), 2));
    }
    let base = serve(node.clone()).await;
    let plan = SessionPlan::read(dir.path()).unwrap();
    let client = BulkClient::new(base, None, fast());
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = seen.clone();
    let done = client
        .upload_session("s-1", &plan, &move |b| sink.lock().unwrap().push(b))
        .await
        .unwrap();
    assert!(done.files > 0);

    let n = node.lock().unwrap();
    assert!(n.committed);
    for (rel, body) in &expected {
        assert_eq!(n.files.get(rel), Some(body), "{rel}");
    }
    let log = &n.log;
    // Second chunk (4096..8192) was cut at 6144; the client re-probed and
    // continued from there, not from a chunk boundary.
    let drop = log
        .iter()
        .position(|l| l == "PUT segments/000000.mcap 4096")
        .unwrap();
    let resume = log
        .iter()
        .position(|l| l == "PUT segments/000000.mcap 6144")
        .unwrap();
    assert!(log[drop..resume]
        .iter()
        .any(|l| l == "HEAD segments/000000.mcap"));
    // The commit came last, after every file.
    assert_eq!(log.last().map(String::as_str), Some("COMMIT"));
    let seen = seen.lock().unwrap();
    assert_eq!(*seen.last().unwrap(), plan.total_bytes);
}

#[tokio::test]
async fn an_already_committed_session_only_commits() {
    let dir = tempfile::tempdir().unwrap();
    session(dir.path());
    let node: Shared = Arc::default();
    node.lock().unwrap().committed = true;
    let base = serve(node.clone()).await;
    let plan = SessionPlan::read(dir.path()).unwrap();
    BulkClient::new(base, None, fast())
        .upload_session("s-1", &plan, &|_| {})
        .await
        .unwrap();
    assert_eq!(node.lock().unwrap().log, ["POST manifest", "COMMIT"]);
}

#[tokio::test]
async fn a_commit_refusal_after_the_retry_pass_is_reported() {
    let dir = tempfile::tempdir().unwrap();
    session(dir.path());
    let node: Shared = Arc::default();
    node.lock()
        .unwrap()
        .expected
        .insert("never-sent.bin".into(), vec![1]);
    let base = serve(node.clone()).await;
    let plan = SessionPlan::read(dir.path()).unwrap();
    let err = BulkClient::new(base, None, fast())
        .upload_session("s-1", &plan, &|_| {})
        .await
        .unwrap_err();
    assert_eq!(err.code(), "E_INCOMPLETE");
    let commits = node
        .lock()
        .unwrap()
        .log
        .iter()
        .filter(|l| *l == "COMMIT")
        .count();
    assert_eq!(commits, 2);
}

#[tokio::test]
async fn an_unreachable_node_gives_up_after_the_retry_budget() {
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    drop(listener);
    let err = BulkClient::new(format!("http://{addr}"), None, fast())
        .post_manifest("s-1", b"{}")
        .await
        .unwrap_err();
    assert!(matches!(err, BulkError::Unreachable(_)), "{err:?}");
    assert_eq!(err.code(), "E_NODE_UNREACHABLE");
}
