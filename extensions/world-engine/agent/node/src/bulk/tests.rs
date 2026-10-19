//! Upload lane tests against the tiny-session fixture.

use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use axum::body::Body;
use axum::http::{header, HeaderMap, Method, Request, StatusCode};
use axum::Router;
use serde_json::{json, Value};
use tower::ServiceExt;

use super::*;

mod flows;
mod guards;
mod refusals;

const ID: &str = "sim-1-20261019T000000Z-0001";
const MCAP: &str = "segments/000000.mcap";
const CHUNK: usize = 4096;
/// Fixture files sent with PUT (the manifest goes in the POST body).
const PUT_FILES: [&str; 3] = ["rig.json", MCAP, "checksums.b3"];

fn fixture(rel: &str) -> Vec<u8> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../contract/fixtures/tiny-session")
        .join(ID);
    std::fs::read(dir.join(rel)).unwrap()
}

struct Reply {
    status: StatusCode,
    headers: HeaderMap,
    json: Value,
}

struct Harness {
    app: Router,
    store: Arc<SessionStore>,
    calls: Arc<AtomicUsize>,
    _tmp: tempfile::TempDir,
}

/// A store under a fresh temp dir. The volume free-space probe is stubbed out
/// so a nearly full dev disk does not fail unrelated tests.
fn store_with(
    limits: impl FnOnce(SessionStore) -> SessionStore,
) -> (SessionStore, tempfile::TempDir) {
    let tmp = tempfile::tempdir().unwrap();
    let store = SessionStore::new(tmp.path().join("sessions"))
        .with_free_space(Arc::new(|_: &std::path::Path| None));
    (limits(store), tmp)
}

fn harness_with(result: Result<(), String>) -> Harness {
    let (store, tmp) = store_with(|s| s);
    harness_on(store, tmp, result)
}

fn harness_on(store: SessionStore, tmp: tempfile::TempDir, result: Result<(), String>) -> Harness {
    let store = Arc::new(store);
    let calls = Arc::new(AtomicUsize::new(0));
    let seen = calls.clone();
    let hook: CommitHook = Arc::new(move |s: &CommittedSession| {
        assert_eq!(s.session_id, ID);
        assert_eq!(s.manifest.session_id, ID);
        assert!(s.dir.join(MCAP).is_file());
        seen.fetch_add(1, Ordering::SeqCst);
        result.clone()
    });
    Harness {
        app: bulk_router(store.clone(), hook),
        store,
        calls,
        _tmp: tmp,
    }
}

fn harness() -> Harness {
    harness_with(Ok(()))
}

fn file_uri(rel: &str) -> String {
    format!("/api/compute/sessions/{ID}/files/{rel}")
}

impl Harness {
    async fn call(&self, req: Request<Body>) -> Reply {
        let resp = self.app.clone().oneshot(req).await.unwrap();
        let status = resp.status();
        let headers = resp.headers().clone();
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        let json = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
        Reply {
            status,
            headers,
            json,
        }
    }

    async fn request(&self, method: Method, uri: &str, body: Vec<u8>) -> Reply {
        let req = Request::builder()
            .method(method)
            .uri(uri)
            .body(Body::from(body))
            .unwrap();
        self.call(req).await
    }

    async fn post_manifest_to(&self, id: &str, body: Vec<u8>) -> Reply {
        self.request(Method::POST, &format!("/api/compute/sessions/{id}"), body)
            .await
    }

    async fn post_manifest(&self) -> Reply {
        self.post_manifest_to(ID, fixture(MANIFEST_FILE)).await
    }

    async fn put(&self, rel: &str, start: usize, data: &[u8], total: usize) -> Reply {
        let range = format!("bytes {}-{}/{}", start, start + data.len() - 1, total);
        let req = Request::builder()
            .method(Method::PUT)
            .uri(file_uri(rel))
            .header(header::CONTENT_RANGE, range)
            .body(Body::from(data.to_vec()))
            .unwrap();
        self.call(req).await
    }

    async fn put_whole(&self, rel: &str, data: &[u8]) -> Reply {
        self.request(Method::PUT, &file_uri(rel), data.to_vec())
            .await
    }

    async fn head(&self, rel: &str) -> Reply {
        self.request(Method::HEAD, &file_uri(rel), Vec::new()).await
    }

    async fn get(&self) -> Reply {
        self.request(
            Method::GET,
            &format!("/api/compute/sessions/{ID}"),
            Vec::new(),
        )
        .await
    }

    async fn commit(&self) -> Reply {
        self.request(
            Method::POST,
            &format!("/api/compute/sessions/{ID}/commit"),
            Vec::new(),
        )
        .await
    }

    /// Send `data[from..]` in [`CHUNK`]-byte ranges.
    async fn upload_from(&self, rel: &str, data: &[u8], from: usize) {
        let mut start = from;
        while start < data.len() {
            let end = (start + CHUNK).min(data.len());
            let reply = self.put(rel, start, &data[start..end], data.len()).await;
            assert_eq!(
                reply.status,
                StatusCode::OK,
                "{rel} @ {start}: {}",
                reply.json
            );
            assert_eq!(reply.json["received"], end as u64);
            assert_eq!(reply.json["complete"], end == data.len());
            start = end;
        }
    }

    async fn upload(&self, rel: &str) {
        self.upload_from(rel, &fixture(rel), 0).await;
    }

    async fn upload_session(&self) {
        assert_eq!(self.post_manifest().await.status, StatusCode::CREATED);
        for rel in PUT_FILES {
            self.upload(rel).await;
        }
    }
}

fn fixture_bytes() -> u64 {
    PUT_FILES
        .iter()
        .chain([&MANIFEST_FILE])
        .map(|rel| fixture(rel).len() as u64)
        .sum()
}
