//! The lanes other nodes use on this node's listener, each behind its gate.
//!
//! Beside the owner's job API the node serves the lanes a drone or a handheld
//! capture device reaches: the reconstruction artifacts, the capture-session
//! upload, the offloaded-detection return stream, the capture-event
//! ingest and the reconstruction descriptor stream. Each is mounted here and
//! nowhere else, wrapped in [`crate::auth::require_lane`] for its own
//! [`NodeLane`], so a lane cannot be mounted without its gate: it serves only
//! the owner (on `http.sock`, or the standalone bearer) or, over TCP, a node
//! holding a credential issued for that lane.

use std::sync::Arc;

use ados_mapping_protocol::node_credential::NodeLane;
use axum::Router;

use crate::auth::{require_lane, ComputeAuth};

/// The reconstruction capture ingest (`POST /api/capture/event` and its probe).
pub(crate) const CAPTURE_INGEST: NodeLane = NodeLane::from_static("capture.ingest");
/// The reconstruction descriptor stream (`GET /ws/live/<device_id>`).
pub(crate) const LIVE_WORLD: NodeLane = NodeLane::from_static("reconstruct.live");
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
pub(crate) const ALL: [NodeLane; 6] = [
    CAPTURE_INGEST,
    LIVE_WORLD,
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
pub use host::{lane_router, CaptureLanes, LaneRoutes};

#[cfg(unix)]
mod host {
    use std::path::PathBuf;
    use std::sync::Arc;

    use ados_mapping_transport::{event_router, world_ws_router, NodeEvent, WorldBroadcaster};
    use axum::Router;
    use tokio::sync::mpsc::Sender;

    use super::{gate, CAPTURE_INGEST, LIVE_WORLD, OFFLOAD_STREAM};
    use crate::auth::ComputeAuth;
    use crate::offload_ws::{offload_ws_router, DetectionBroadcaster};

    /// The capture lanes.
    pub struct CaptureLanes {
        /// Where decoded capture events go (the receiver loop drains it).
        pub events: Sender<NodeEvent>,
        /// The reconstruction descriptor fan-out.
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
        /// The capture ingest and live stream lanes.
        pub capture: CaptureLanes,
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
                event_router(routes.capture.events),
                CAPTURE_INGEST,
            ))
            .merge(gate(
                &auth,
                world_ws_router(routes.capture.world),
                LIVE_WORLD,
            ))
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::node_credentials::NodeCredentialStore;
    use crate::offload_ws::DetectionBroadcaster;
    use ados_mapping_transport::{NodeEvent, WorldBroadcaster};
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tokio::sync::mpsc::{channel, Receiver};
    use tower::ServiceExt;

    struct Fixture {
        router: Router,
        auth: Arc<ComputeAuth>,
        events: Receiver<NodeEvent>,
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
                uploads: crate::bulk::bulk_router(
                    Arc::new(crate::bulk::SessionStore::new(dir.path().join("sessions"))),
                    Arc::new(|_| Ok(())),
                ),
                offload: Arc::new(DetectionBroadcaster::new(8)),
                capture: CaptureLanes {
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
        NodeEvent::new("capture.keyframe", Some("drone-1".into()), vec![1, 2, 3])
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
            status(&f.router, "POST", "/api/capture/event", &[], event_body()).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "GET", "/api/capture/health", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
        assert_eq!(
            status(&f.router, "GET", "/ws/live/drone-1", &[], vec![]).await,
            StatusCode::UNAUTHORIZED
        );
    }

    #[tokio::test]
    async fn a_drone_credential_reaches_the_lanes_it_was_issued_for() {
        let mut f = fixture();
        let m = f
            .auth
            .credentials
            .mint("drone-1", &[CAPTURE_INGEST], 1)
            .unwrap();
        let cred = [("x-ados-node-credential", m.credential.as_str())];
        assert_eq!(
            status(&f.router, "POST", "/api/capture/event", &cred, event_body()).await,
            StatusCode::ACCEPTED
        );
        assert_eq!(f.events.recv().await.unwrap().topic, "capture.keyframe");
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
            .insert(ados_mapping_transport::UnixPeer);
        assert_eq!(
            f.router.clone().oneshot(req).await.unwrap().status(),
            StatusCode::OK
        );
    }
}
