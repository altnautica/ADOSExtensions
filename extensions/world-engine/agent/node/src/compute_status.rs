//! `GET /compute/status` on the plugin's operator socket: this node's latest
//! compute status, so a paired GCS renders the compute-cluster card local-first.
//!
//! The body is the heartbeat loop's latest [`ComputeHeartbeatSidecar`] (the
//! camelCase `compute*` fields plus the snake_case host `gpu` block, which is
//! always present: all-`null` off macOS), held in memory. Before the first
//! heartbeat tick the route is a `404` with a `detail` body, never a `500`.
//!
//! Mounted only on `http.sock` (see [`operator_socket_router`]), where
//! ados-control has already authenticated the operator.

use std::sync::{Arc, RwLock};

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::get;
use axum::{Json, Router};

use crate::heartbeat_sidecar::ComputeHeartbeatSidecar;

/// The latest compute status the heartbeat loop sampled, shared with the route.
#[derive(Debug, Default)]
pub struct LatestComputeStatus(RwLock<Option<ComputeHeartbeatSidecar>>);

impl LatestComputeStatus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Replace the latest status (called on every heartbeat tick).
    pub fn set(&self, status: ComputeHeartbeatSidecar) {
        *self.0.write().unwrap_or_else(|p| p.into_inner()) = Some(status);
    }

    /// The latest status, or `None` before the first heartbeat.
    pub fn get(&self) -> Option<ComputeHeartbeatSidecar> {
        self.0.read().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

/// The router serving `GET /compute/status` from `latest`.
pub fn compute_status_router(latest: Arc<LatestComputeStatus>) -> Router {
    Router::new()
        .route("/compute/status", get(get_compute_status))
        .with_state(latest)
}

/// The router served on the operator socket: the shared node router (job API
/// and lanes, which admit an operator-socket caller as the owner) plus the
/// socket-only `GET /compute/status`.
pub fn operator_socket_router(shared: Router, latest: Arc<LatestComputeStatus>) -> Router {
    shared.merge(compute_status_router(latest))
}

async fn get_compute_status(State(latest): State<Arc<LatestComputeStatus>>) -> Response {
    match latest.get() {
        Some(status) => (StatusCode::OK, Json(status)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "detail": "no compute status (not a compute node, or the compute daemon is not running)"
            })),
        )
            .into_response(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ClusterDescriptor, ComputeGpu, ComputeRole};
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;
    use world_engine_protocol::compute::ComputeHeartbeat;

    async fn get(router: &Router) -> (StatusCode, serde_json::Value) {
        let resp = router
            .clone()
            .oneshot(Request::get("/compute/status").body(Body::empty()).unwrap())
            .await
            .unwrap();
        let status = resp.status();
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        (status, serde_json::from_slice(&bytes).unwrap())
    }

    #[tokio::test]
    async fn is_a_404_with_detail_before_the_first_heartbeat_then_serves_the_latest() {
        let latest = Arc::new(LatestComputeStatus::new());
        let router = compute_status_router(latest.clone());
        let (st, body) = get(&router).await;
        assert_eq!(st, StatusCode::NOT_FOUND);
        assert!(body["detail"]
            .as_str()
            .unwrap()
            .contains("no compute status"));

        let hb = ComputeHeartbeat {
            role: ComputeRole::Master,
            cluster: ClusterDescriptor {
                master_id: "node-a".into(),
                slaves: vec![],
                aggregate_workers_idle: 3,
            },
            queue_depth: 2,
            active_jobs: 1,
            workers_idle: 3,
            active_sessions: 0,
        };
        // A non-macOS sample: the gpu block is still present, all-null.
        let sidecar = ComputeHeartbeatSidecar::from_heartbeat(&hb, ComputeGpu::default(), 1234);
        latest.set(sidecar.clone());
        let (st, body) = get(&router).await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body, serde_json::to_value(&sidecar).unwrap());
        assert_eq!(body["computeClusterMasterId"], "node-a");
        assert!(body["gpu"].is_object());
        for key in [
            "name",
            "cores",
            "unified_memory_mb",
            "metal",
            "utilization_pct",
        ] {
            assert!(body["gpu"][key].is_null(), "{key} is null");
        }
    }
}
