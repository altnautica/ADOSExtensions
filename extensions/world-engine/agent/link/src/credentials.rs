//! The credentials workstations issued this node, and `POST /node-credential`.
//!
//! A node's own pairing key means nothing to a workstation, so the ground
//! station (which holds the owner key of both) asks the workstation to issue
//! this node a credential scoped to the lanes it uses and installs it here. The
//! lanes (the Atlas forwarder, the offload session, a ground station's Atlas
//! relay) present it to the workstation that issued it, picked by that
//! workstation's advertised node id.
//!
//! Installs land in the extension's own store under the plugin data dir, and
//! every lookup reads that store. The agent's node-wide credential file is
//! hidden from plugin units, so it is never consulted.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::post;
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::json;
use world_engine_protocol::node_credential::{
    InstalledCredential, NodeLane, WorkstationCredentials,
};

use crate::api::detail;

/// The extension store's file name under the plugin data dir.
pub const STORE_FILE: &str = "workstation-credentials.json";

/// The lane the Atlas forwarder and the ground-station relay present.
pub const ATLAS_INGEST_LANE: NodeLane = NodeLane::from_static("atlas.ingest");
/// The lane the perception-offload session presents.
pub const OFFLOAD_STREAM_LANE: NodeLane = NodeLane::from_static("offload.stream");

/// A node id, not free text.
const MAX_NODE_ID_LEN: usize = 128;
/// Far above an issued token's length; bounds what is written to disk.
const MAX_CREDENTIAL_LEN: usize = 256;

/// Where credentials are read from and installed to: the extension's own store.
#[derive(Debug, Clone)]
pub struct CredentialStore {
    /// The store file; `None` when the host gave no data dir, in which case
    /// nothing can be installed and every lookup finds nothing.
    pub path: Option<PathBuf>,
}

impl CredentialStore {
    /// The store under `data_dir`.
    pub fn new(data_dir: Option<&Path>) -> Self {
        Self {
            path: data_dir.map(|d| d.join(STORE_FILE)),
        }
    }

    /// The installed credentials issued for `lane`.
    fn for_lane(&self, lane: &NodeLane) -> WorkstationCredentials {
        let store = self
            .path
            .as_deref()
            .map(WorkstationCredentials::load_or_empty)
            .unwrap_or_default();
        WorkstationCredentials {
            workstations: store
                .workstations
                .into_iter()
                .filter(|c| c.lanes.contains(lane))
                .collect(),
        }
    }

    /// The credential to present on `lane` to `workstation_node_id`. A known
    /// node id picks only its own credential; `None` (a pinned address) picks
    /// the sole credential for the lane.
    pub fn lookup(&self, workstation_node_id: Option<&str>, lane: &NodeLane) -> Option<String> {
        self.for_lane(lane)
            .for_node(workstation_node_id)
            .map(|c| c.credential.clone())
    }

    /// Every workstation that issued this node a credential for `lane`, so
    /// discovery can prefer one.
    pub fn issuers(&self, lane: &NodeLane) -> Vec<String> {
        let mut ids: Vec<String> = self
            .for_lane(lane)
            .workstations
            .into_iter()
            .map(|c| c.workstation_node_id)
            .collect();
        ids.sort();
        ids.dedup();
        ids
    }
}

#[derive(Debug, Deserialize)]
pub struct InstallRequest {
    workstation_node_id: String,
    credential: String,
    lanes: Vec<NodeLane>,
}

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// `POST /node-credential`, served on every profile the link binds HTTP on.
pub fn router(stores: CredentialStore) -> Router {
    Router::new()
        .route("/node-credential", post(install))
        .with_state(stores)
}

async fn install(
    State(stores): State<CredentialStore>,
    Json(req): Json<InstallRequest>,
) -> Response {
    let Some(path) = stores.path else {
        return detail(
            StatusCode::INTERNAL_SERVER_ERROR,
            "could not store the credential: no plugin data directory",
        );
    };
    match tokio::task::spawn_blocking(move || install_at(&path, req, now_ms())).await {
        Ok(resp) => resp,
        Err(e) => detail(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()),
    }
}

fn install_at(path: &Path, req: InstallRequest, now_ms: i64) -> Response {
    let node_id = req.workstation_node_id.trim();
    if node_id.is_empty() || node_id.len() > MAX_NODE_ID_LEN {
        return detail(
            StatusCode::BAD_REQUEST,
            "workstation_node_id must be 1-128 characters",
        );
    }
    let credential = req.credential.trim();
    let header_safe = credential.bytes().all(|b| b.is_ascii_graphic());
    if credential.is_empty() || credential.len() > MAX_CREDENTIAL_LEN || !header_safe {
        return detail(StatusCode::BAD_REQUEST, "credential is not a valid token");
    }
    if req.lanes.is_empty() {
        return detail(StatusCode::BAD_REQUEST, "at least one lane is required");
    }
    // An unreadable store is replaced rather than refusing the install: every
    // credential it held is equally unusable, and the owner is re-provisioning.
    let mut store = WorkstationCredentials::load_or_empty(path);
    store.upsert(InstalledCredential {
        workstation_node_id: node_id.to_string(),
        credential: credential.to_string(),
        lanes: req.lanes.clone(),
        installed_at_ms: now_ms,
    });
    if let Err(e) = store.save(path) {
        tracing::error!(error = %e, "workstation credential install failed");
        return detail(
            StatusCode::INTERNAL_SERVER_ERROR,
            format!("could not store the credential: {e}"),
        );
    }
    tracing::info!(workstation = %node_id, "workstation credential installed");
    Json(json!({
        "installed": true,
        "workstation_node_id": node_id,
        "lanes": req.lanes,
        "installed_at_ms": now_ms,
    }))
    .into_response()
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;

    fn req(node: &str, cred: &str) -> InstallRequest {
        InstallRequest {
            workstation_node_id: node.into(),
            credential: cred.into(),
            lanes: vec![ATLAS_INGEST_LANE, OFFLOAD_STREAM_LANE],
        }
    }

    fn write_store(path: &Path, entries: &[(&str, &str, &[NodeLane])]) {
        let mut store = WorkstationCredentials::default();
        for (node, secret, lanes) in entries {
            store.upsert(InstalledCredential {
                workstation_node_id: (*node).into(),
                credential: (*secret).into(),
                lanes: lanes.to_vec(),
                installed_at_ms: 1,
            });
        }
        store.save(path).unwrap();
    }

    #[tokio::test]
    async fn the_route_installs_into_the_extension_store() {
        let dir = tempfile::tempdir().unwrap();
        let stores = CredentialStore {
            path: Some(dir.path().join(STORE_FILE)),
        };
        let resp = router(stores.clone())
            .oneshot(
                Request::post("/node-credential")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        r#"{"workstation_node_id":"ws-a","credential":"nc1.ID.SECRET","lanes":["atlas.ingest"]}"#,
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::OK);
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        let body: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(body["installed"], true);
        assert_eq!(
            stores.lookup(Some("ws-a"), &ATLAS_INGEST_LANE).as_deref(),
            Some("nc1.ID.SECRET")
        );
    }

    #[tokio::test]
    async fn a_reinstall_from_the_same_workstation_replaces_it() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(STORE_FILE);
        install_at(&path, req("ws-a", "old"), 1);
        install_at(&path, req("ws-a", "new"), 2);
        let store = WorkstationCredentials::load(&path).unwrap();
        assert_eq!(store.workstations.len(), 1);
        assert_eq!(store.workstations[0].credential, "new");
    }

    #[tokio::test]
    async fn malformed_installs_are_refused() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(STORE_FILE);
        for bad in [
            req("", "nc1.a.b"),
            req("ws", ""),
            req("ws", "has space"),
            req("ws", "line\nbreak"),
            InstallRequest {
                lanes: vec![],
                ..req("ws", "nc1.a.b")
            },
        ] {
            assert_eq!(install_at(&path, bad, 1).status(), StatusCode::BAD_REQUEST);
        }
        assert!(!path.exists());
    }

    #[test]
    fn lookup_picks_the_named_node_and_a_pinned_address_takes_the_sole_credential() {
        let dir = tempfile::tempdir().unwrap();
        let ext = dir.path().join(STORE_FILE);
        write_store(&ext, &[("ws-a", "ext-a", &[ATLAS_INGEST_LANE])]);
        let stores = CredentialStore { path: Some(ext) };

        assert_eq!(
            stores.lookup(Some("ws-a"), &ATLAS_INGEST_LANE).as_deref(),
            Some("ext-a")
        );
        // A node the store does not hold gets nothing, never another node's.
        assert_eq!(stores.lookup(Some("ws-c"), &ATLAS_INGEST_LANE), None);
        assert_eq!(
            stores.lookup(None, &ATLAS_INGEST_LANE).as_deref(),
            Some("ext-a")
        );
        assert_eq!(stores.issuers(&ATLAS_INGEST_LANE), ["ws-a"]);
        // No data dir: nothing is found.
        assert_eq!(
            CredentialStore { path: None }.lookup(None, &ATLAS_INGEST_LANE),
            None
        );
    }

    #[test]
    fn lookup_never_offers_a_credential_for_a_lane_it_was_not_issued_for() {
        let dir = tempfile::tempdir().unwrap();
        let ext = dir.path().join(STORE_FILE);
        write_store(&ext, &[("ws-a", "ext-a", &[OFFLOAD_STREAM_LANE])]);
        let stores = CredentialStore { path: Some(ext) };
        assert_eq!(stores.lookup(Some("ws-a"), &ATLAS_INGEST_LANE), None);
        assert_eq!(
            stores.lookup(Some("ws-a"), &OFFLOAD_STREAM_LANE).as_deref(),
            Some("ext-a")
        );
        assert!(stores.issuers(&ATLAS_INGEST_LANE).is_empty());
    }
}
