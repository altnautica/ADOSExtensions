//! The lanes other nodes use on this node's listener, each behind its gate.
//!
//! Beside the owner's job API the node serves the lanes a drone or a handheld
//! capture device reaches: the reconstruction artifacts, the capture-session
//! upload, the offloaded-detection return stream, the Atlas capture-event
//! ingest and the world-model descriptor stream. Each is mounted here and
//! nowhere else, wrapped in [`crate::auth::require_lane`] for its own
//! [`NodeLane`], so a lane cannot be mounted without its gate: it serves only
//! the owner (on `http.sock`, or the standalone bearer) or, over TCP, a node
//! holding a credential issued for that lane.

use std::sync::Arc;

use axum::Router;
use world_engine_protocol::node_credential::NodeLane;

use crate::auth::{require_lane, ComputeAuth};

/// The world-model capture ingest (`POST /api/atlas/event` and its probe).
pub(crate) const ATLAS_INGEST: NodeLane = NodeLane::from_static("atlas.ingest");
/// The world-model descriptor stream (`GET /ws/atlas/<device_id>`).
pub(crate) const ATLAS_WORLD: NodeLane = NodeLane::from_static("atlas.world");
/// The offloaded-detection return stream (`GET /ws/offload/<session_id>`).
pub(crate) const OFFLOAD_STREAM: NodeLane = NodeLane::from_static("offload.stream");
/// Reconstruction artifacts (`GET /artifacts/*`).
pub(crate) const ARTIFACTS: NodeLane = NodeLane::from_static("artifacts.read");
/// Submitting a job (an offload session, a reconstruction of an uploaded
/// session) and reading session health.
pub(crate) const JOB_SUBMIT: NodeLane = NodeLane::from_static("jobs.submit");
/// Uploading a capture session (`/api/compute/sessions/<id>/...`).
pub(crate) const SESSION_UPLOAD: NodeLane = NodeLane::from_static("sessions.upload");
/// Every lane this node serves, in wire order: the default grant for a drone.
#[cfg_attr(not(unix), allow(dead_code))]
pub(crate) const ALL: [NodeLane; 6] = [
    ATLAS_INGEST,
    ATLAS_WORLD,
    OFFLOAD_STREAM,
    ARTIFACTS,
    JOB_SUBMIT,
    SESSION_UPLOAD,
];

/// Wrap `router` in the gate for `lane`.
pub fn gate(auth: &Arc<ComputeAuth>, router: Router, lane: NodeLane) -> Router {
    router.route_layer(axum::middleware::from_fn_with_state(
        (auth.clone(), lane),
        require_lane,
    ))
}

/// The capture-session upload lane and the artifacts lane, the two lanes a
/// standalone node serves.
pub fn upload_and_artifact_lanes(
    auth: &Arc<ComputeAuth>,
    uploads: Router,
    work_root: std::path::PathBuf,
) -> Router {
    gate(auth, uploads, SESSION_UPLOAD).merge(gate(
        auth,
        crate::artifacts::artifact_router(work_root),
        ARTIFACTS,
    ))
}

#[cfg(unix)]
pub use host::{lane_router, AtlasLanes, LaneRoutes};

#[cfg(unix)]
mod host {
    use std::path::PathBuf;
    use std::sync::Arc;

    use axum::Router;
    use tokio::sync::mpsc::Sender;
    use world_engine_transport::{
        atlas_event_router, world_ws_router, AtlasEvent, WorldBroadcaster,
    };

    use super::{gate, ATLAS_INGEST, ATLAS_WORLD, OFFLOAD_STREAM};
    use crate::auth::ComputeAuth;
    use crate::offload_ws::{offload_ws_router, DetectionBroadcaster};

    /// The Atlas lanes.
    pub struct AtlasLanes {
        /// Where decoded capture events go (the receiver loop drains it).
        pub events: Sender<AtlasEvent>,
        /// The world-model descriptor fan-out.
        pub world: Arc<WorldBroadcaster>,
    }

    /// What the lane routers serve.
    pub struct LaneRoutes {
        /// The artifact work root (path-jailed by the artifact router).
        pub work_root: PathBuf,
        /// The capture-session upload router ([`crate::bulk::bulk_router`]).
        pub uploads: Router,
        /// The offload detection fan-out.
        pub offload: Arc<DetectionBroadcaster>,
        /// The Atlas ingest and world-model lanes.
        pub atlas: AtlasLanes,
    }

    /// Every lane router, each behind the gate for its lane.
    pub fn lane_router(auth: Arc<ComputeAuth>, routes: LaneRoutes) -> Router {
        super::upload_and_artifact_lanes(&auth, routes.uploads, routes.work_root)
            .merge(gate(
                &auth,
                offload_ws_router(routes.offload),
                OFFLOAD_STREAM,
            ))
            .merge(gate(
                &auth,
                atlas_event_router(routes.atlas.events),
                ATLAS_INGEST,
            ))
            .merge(gate(
                &auth,
                world_ws_router(routes.atlas.world),
                ATLAS_WORLD,
            ))
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::node_credentials::NodeCredentialStore;
    use crate::offload_ws::DetectionBroadcaster;
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tokio::sync::mpsc::{channel, Receiver};
    use tower::ServiceExt;
    use world_engine_transport::{AtlasEvent, WorldBroadcaster};

    struct Fixture {
        router: Router,
        auth: Arc<ComputeAuth>,
        events: Receiver<AtlasEvent>,
        _dir: tempfile::TempDir,
    }

    fn fixture() -> Fixture {
        let dir = tempfile::tempdir().unwrap();
        let work = dir.path().join("work");
        std::fs::create_dir_all(work.join("job-1")).unwrap();
        std::fs::write(work.join("job-1/cloud.ply"), b"ply").unwrap();
        let auth = Arc::new(ComputeAuth::new(NodeCredentialStore::open(
            dir.path().join("creds.json"),
            "ws-node",
        )));
        let (tx, rx) = channel(8);
        let router = lane_router(
            auth.clone(),
            LaneRoutes {
                work_root: work,
                uploads: Router::new(),
                offload: Arc::new(DetectionBroadcaster::new(8)),
                atlas: AtlasLanes {
                    events: tx,
                    world: Arc::new(WorldBroadcaster::new(8)),
                },
            },
        );
        Fixture {
            router,
            auth,
            events: rx,
            _dir: dir,
        }
    }

    /// One TCP request with the given headers.
    async fn status(
        router: &Router,
        method: &str,
        path: &str,
        headers: &[(&str, &str)],
        body: Vec<u8>,
    ) -> StatusCode {
        let mut builder = Request::builder().method(method).uri(path);
        for (k, v) in headers {
            builder = builder.header(*k, *v);
        }
        let req = builder.body(Body::from(body)).unwrap();
        router.clone().oneshot(req).await.unwrap().status()
    }

    fn event_body() -> Vec<u8> {
        AtlasEvent::new("atlas.keyframe", Some("drone-1".into()), vec![1, 2, 3])
            .encode()
            .unwrap()
    }

    #[tokio::test]
    async fn every_lane_refuses_a_tcp_caller_without_a_credential() {
        let f = fixture();
        assert_eq!(
            status(&f.router, "GET", "/artifacts/job-1/cloud.ply", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "GET", "/ws/offload/s1", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "POST", "/api/atlas/event", &[], event_body()).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "GET", "/api/atlas/health", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "GET", "/ws/atlas/drone-1", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
    }

    #[tokio::test]
    async fn a_drone_credential_reaches_the_lanes_it_was_issued_for() {
        let mut f = fixture();
        let m = f
            .auth
            .credentials
            .mint("drone-1", &[ATLAS_INGEST], 1)
            .unwrap();
        let cred = [("x-ados-node-credential", m.credential.as_str())];
        assert_eq!(
            status(&f.router, "POST", "/api/atlas/event", &cred, event_body()).await,
            StatusCode::ACCEPTED
        );
        assert_eq!(f.events.recv().await.unwrap().topic, "atlas.keyframe");
        // Not issued for artifacts.
        assert_eq!(
            status(
                &f.router,
                "GET",
                "/artifacts/job-1/cloud.ply",
                &cred,
                vec![]
            )
            .await,
            StatusCode::UNAUTHORIZED
        );
        // The operator (on http.sock) reaches the artifact.
        let mut req = Request::builder()
            .uri("/artifacts/job-1/cloud.ply")
            .body(Body::empty())
            .unwrap();
        req.extensions_mut()
            .insert(world_engine_transport::UnixPeer);
        assert_eq!(
            f.router.clone().oneshot(req).await.unwrap().status(),
            StatusCode::OK
        );
    }
}
