//! The credentials workstations issued this node, and `POST /node-credential`.
//!
//! A node's own pairing key means nothing to a workstation, so the ground
//! station (which holds the owner key of both) asks the workstation to issue
//! this node a credential scoped to the lanes it uses and installs it here. The
//! lanes (the Atlas forwarder, the offload session, a ground station's Atlas
//! relay) present it to the workstation that issued it, picked by that
//! workstation's advertised node id, or for a pinned address by
//! [`CredentialStore::select`].
//!
//! Installs land in the extension's own store under the plugin data dir, and
//! every lookup reads that store. The agent's node-wide credential file is
//! hidden from plugin units, so it is never consulted.

use std::fmt;
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
    is_issued_token, InstalledCredential, NodeLane, WorkstationCredentials,
};

use crate::api::detail;
use crate::compute_node::{split_host_port, DEFAULT_JOB_API_PORT};

/// The extension store's file name under the plugin data dir.
pub const STORE_FILE: &str = "workstation-credentials.json";

/// The lane the Atlas forwarder and the ground-station relay present.
pub const ATLAS_INGEST_LANE: NodeLane = NodeLane::from_static("atlas.ingest");
/// The lane the perception-offload session presents.
pub const OFFLOAD_STREAM_LANE: NodeLane = NodeLane::from_static("offload.stream");
/// The lane the session uploader presents (`/api/compute/sessions/<id>/...`).
pub const SESSION_UPLOAD_LANE: NodeLane = NodeLane::from_static("sessions.upload");
/// The lane a reconstruction submit presents (`POST /api/compute/jobs`).
pub const JOB_SUBMIT_LANE: NodeLane = NodeLane::from_static("jobs.submit");

/// A node id, not free text.
const MAX_NODE_ID_LEN: usize = 128;
/// Far above an issued token's length; bounds what is written to disk.
const MAX_CREDENTIAL_LEN: usize = 256;
/// A `host[:port]`, not free text.
const MAX_NODE_ADDR_LEN: usize = 255;

/// Why no credential can be presented to a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CredentialGap {
    /// None is installed for the lane, or none from the node that named itself.
    NoCredential,
    /// Several fit a pinned address equally well and nothing tells them apart.
    Ambiguous,
}

impl CredentialGap {
    /// The reason code reported in an upload state.
    pub fn code(self) -> &'static str {
        match self {
            Self::NoCredential => "no_credential",
            Self::Ambiguous => "ambiguous_credential",
        }
    }
}

impl fmt::Display for CredentialGap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.code())
    }
}

/// `addr` as a comparable `(host, port)`, the port defaulted.
fn addr_key(addr: &str) -> Option<(String, u16)> {
    split_host_port(addr, DEFAULT_JOB_API_PORT).map(|(h, p)| (h.to_ascii_lowercase(), p))
}

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
    fn for_lane(&self, lane: &NodeLane) -> Vec<InstalledCredential> {
        let store = self
            .path
            .as_deref()
            .map(WorkstationCredentials::load_or_empty)
            .unwrap_or_default();
        store
            .workstations
            .into_iter()
            .filter(|c| c.lanes.contains(lane))
            .collect()
    }

    /// The credential to present on `lane` to a node.
    ///
    /// A node that names itself (`node_id`, its mDNS advert) gets the
    /// credential it issued and nothing else, so a credential is never offered
    /// to a node that did not issue it. A node known only by its address
    /// (`addr`, a pinned `host[:port]`) gets the best of the lane's
    /// credentials: an issued-form token over any other (the workstation admits
    /// nothing else), then one installed for that address, then the most
    /// recently installed. Two equal on all three are ambiguous.
    pub fn select(
        &self,
        node_id: Option<&str>,
        addr: Option<&str>,
        lane: &NodeLane,
    ) -> Result<String, CredentialGap> {
        let creds = self.for_lane(lane);
        if let Some(id) = node_id.filter(|id| !id.is_empty()) {
            return creds
                .into_iter()
                .find(|c| c.workstation_node_id == id)
                .map(|c| c.credential)
                .ok_or(CredentialGap::NoCredential);
        }
        let pinned = addr.and_then(addr_key);
        let rank = |c: &InstalledCredential| {
            let at_pin = pinned.is_some() && c.node_addr.as_deref().and_then(addr_key) == pinned;
            (is_issued_token(&c.credential), at_pin, c.installed_at_ms)
        };
        let mut ranked: Vec<_> = creds.into_iter().map(|c| (rank(&c), c)).collect();
        ranked.sort_by_key(|(r, _)| std::cmp::Reverse(*r));
        match ranked.as_slice() {
            [] => Err(CredentialGap::NoCredential),
            [(best, _), (next, _), ..] if best == next => Err(CredentialGap::Ambiguous),
            [(_, c), ..] => Ok(c.credential.clone()),
        }
    }

    /// Every workstation that issued this node a credential for `lane`, so
    /// discovery can prefer one.
    pub fn issuers(&self, lane: &NodeLane) -> Vec<String> {
        let mut ids: Vec<String> = self
            .for_lane(lane)
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
    /// The workstation's `host[:port]` the credential was minted at.
    #[serde(default)]
    node_addr: Option<String>,
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
    let node_addr = req
        .node_addr
        .as_deref()
        .map(str::trim)
        .filter(|a| !a.is_empty());
    if let Some(addr) = node_addr {
        let well_formed = addr.len() <= MAX_NODE_ADDR_LEN
            && addr
                .bytes()
                .all(|b| b.is_ascii_graphic() && !b"/@?#".contains(&b))
            && addr_key(addr).is_some();
        if !well_formed {
            return detail(StatusCode::BAD_REQUEST, "node_addr must be a host[:port]");
        }
    }
    // An unreadable store is replaced rather than refusing the install: every
    // credential it held is equally unusable, and the owner is re-provisioning.
    let mut store = WorkstationCredentials::load_or_empty(path);
    store.upsert(InstalledCredential {
        workstation_node_id: node_id.to_string(),
        credential: credential.to_string(),
        lanes: req.lanes.clone(),
        installed_at_ms: now_ms,
        node_addr: node_addr.map(str::to_string),
    });
    if let Err(e) = store.save(path) {
        tracing::error!(error = %e, "workstation credential install failed");
        return detail(
            StatusCode::INTERNAL_SERVER_ERROR,
            format!("could not store the credential: {e}"),
        );
    }
    tracing::info!(workstation = %node_id, node_addr = ?node_addr, "workstation credential installed");
    Json(json!({
        "installed": true,
        "workstation_node_id": node_id,
        "node_addr": node_addr,
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
            node_addr: None,
        }
    }

    fn install(
        path: &Path,
        node: &str,
        cred: &str,
        addr: Option<&str>,
        lanes: &[NodeLane],
        at: i64,
    ) {
        let r = InstallRequest {
            lanes: lanes.to_vec(),
            node_addr: addr.map(str::to_string),
            ..req(node, cred)
        };
        assert_eq!(install_at(path, r, at).status(), StatusCode::OK);
    }

    fn write_store(path: &Path, entries: &[(&str, &str, &[NodeLane])]) {
        let mut store = WorkstationCredentials::default();
        for (node, secret, lanes) in entries {
            store.upsert(InstalledCredential {
                workstation_node_id: (*node).into(),
                credential: (*secret).into(),
                lanes: lanes.to_vec(),
                installed_at_ms: 1,
                node_addr: None,
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
                        r#"{"workstation_node_id":"ws-a","credential":"nc1.ID.SECRET","lanes":["atlas.ingest"],"node_addr":"192.0.2.5:9000"}"#,
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
        assert_eq!(body["node_addr"], "192.0.2.5:9000");
        assert_eq!(
            stores
                .select(Some("ws-a"), None, &ATLAS_INGEST_LANE)
                .as_deref(),
            Ok("nc1.ID.SECRET")
        );
        let store = WorkstationCredentials::load(stores.path.as_deref().unwrap()).unwrap();
        assert_eq!(
            store.workstations[0].node_addr.as_deref(),
            Some("192.0.2.5:9000")
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
            InstallRequest {
                node_addr: Some("http://192.0.2.5:8092".into()),
                ..req("ws", "nc1.a.b")
            },
            InstallRequest {
                node_addr: Some("host name".into()),
                ..req("ws", "nc1.a.b")
            },
        ] {
            assert_eq!(install_at(&path, bad, 1).status(), StatusCode::BAD_REQUEST);
        }
        assert!(!path.exists());
    }

    #[test]
    fn a_named_node_gets_only_its_own_credential() {
        let dir = tempfile::tempdir().unwrap();
        let ext = dir.path().join(STORE_FILE);
        write_store(
            &ext,
            &[
                ("ws-a", "ext-a", &[ATLAS_INGEST_LANE]),
                ("ws-b", "ext-b", &[ATLAS_INGEST_LANE]),
            ],
        );
        let stores = CredentialStore { path: Some(ext) };
        assert_eq!(
            stores
                .select(Some("ws-b"), Some("192.0.2.5"), &ATLAS_INGEST_LANE)
                .as_deref(),
            Ok("ext-b")
        );
        // A node the store does not hold gets nothing, never another node's.
        assert_eq!(
            stores.select(Some("ws-c"), None, &ATLAS_INGEST_LANE),
            Err(CredentialGap::NoCredential)
        );
        assert_eq!(stores.issuers(&ATLAS_INGEST_LANE), ["ws-a", "ws-b"]);
        // An empty id is no id: the pinned-address rule applies.
        assert_eq!(
            stores.select(Some(""), None, &ATLAS_INGEST_LANE),
            Err(CredentialGap::Ambiguous)
        );
    }

    #[test]
    fn a_pinned_address_takes_the_credential_installed_for_it() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(STORE_FILE);
        let lanes = [SESSION_UPLOAD_LANE, JOB_SUBMIT_LANE];
        install(&path, "ws-a", "nc1.A.SECRET", Some("10.99.0.2"), &lanes, 1);
        install(
            &path,
            "ws-b",
            "nc1.B.SECRET",
            Some("10.99.0.7:8092"),
            &lanes,
            2,
        );
        let stores = CredentialStore { path: Some(path) };
        // The older credential, minted at the pinned address, wins over the newer.
        assert_eq!(
            stores
                .select(None, Some("10.99.0.2:8092"), &SESSION_UPLOAD_LANE)
                .as_deref(),
            Ok("nc1.A.SECRET")
        );
        assert_eq!(
            stores
                .select(None, Some("10.99.0.7:8092"), &SESSION_UPLOAD_LANE)
                .as_deref(),
            Ok("nc1.B.SECRET")
        );
        // An address neither was minted at: the most recently installed.
        assert_eq!(
            stores
                .select(None, Some("10.99.0.9:8092"), &SESSION_UPLOAD_LANE)
                .as_deref(),
            Ok("nc1.B.SECRET")
        );
    }

    #[test]
    fn a_pinned_address_presents_the_node_credential_over_an_owner_token() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(STORE_FILE);
        let lanes = [SESSION_UPLOAD_LANE, JOB_SUBMIT_LANE];
        // The node's credential, then its owner bearer token filed under
        // another id and installed later: the workstation admits only the first.
        install(&path, "standalone-1", "nc1.ID.SECRET", None, &lanes, 1);
        install(&path, "host-node", "owner-bearer-token", None, &lanes, 2);
        let stores = CredentialStore { path: Some(path) };
        assert_eq!(
            stores
                .select(None, Some("10.99.0.2:8092"), &SESSION_UPLOAD_LANE)
                .as_deref(),
            Ok("nc1.ID.SECRET")
        );
    }

    #[test]
    fn an_unusable_store_is_a_named_gap() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(STORE_FILE);
        let stores = CredentialStore {
            path: Some(path.clone()),
        };
        assert_eq!(
            stores.select(None, Some("10.99.0.2"), &SESSION_UPLOAD_LANE),
            Err(CredentialGap::NoCredential)
        );
        assert_eq!(
            CredentialStore { path: None }.select(None, None, &SESSION_UPLOAD_LANE),
            Err(CredentialGap::NoCredential)
        );
        // Two alike in every respect: nothing tells them apart.
        install(
            &path,
            "ws-a",
            "nc1.A.SECRET",
            None,
            &[SESSION_UPLOAD_LANE],
            5,
        );
        install(
            &path,
            "ws-b",
            "nc1.B.SECRET",
            None,
            &[SESSION_UPLOAD_LANE],
            5,
        );
        assert_eq!(
            stores.select(None, Some("10.99.0.2"), &SESSION_UPLOAD_LANE),
            Err(CredentialGap::Ambiguous)
        );
        assert_eq!(CredentialGap::NoCredential.code(), "no_credential");
        assert_eq!(CredentialGap::Ambiguous.code(), "ambiguous_credential");
    }

    #[test]
    fn a_credential_is_never_offered_for_a_lane_it_was_not_issued_for() {
        let dir = tempfile::tempdir().unwrap();
        let ext = dir.path().join(STORE_FILE);
        write_store(&ext, &[("ws-a", "ext-a", &[OFFLOAD_STREAM_LANE])]);
        let stores = CredentialStore { path: Some(ext) };
        assert_eq!(
            stores.select(Some("ws-a"), None, &ATLAS_INGEST_LANE),
            Err(CredentialGap::NoCredential)
        );
        assert_eq!(
            stores.select(None, None, &ATLAS_INGEST_LANE),
            Err(CredentialGap::NoCredential)
        );
        assert_eq!(
            stores
                .select(Some("ws-a"), None, &OFFLOAD_STREAM_LANE)
                .as_deref(),
            Ok("ext-a")
        );
        assert!(stores.issuers(&ATLAS_INGEST_LANE).is_empty());
    }
}
