//! Auth for the compute node's two listeners.
//!
//! - **The operator socket (`http.sock`).** A request marked with [`UnixPeer`]
//!   came through ados-control, which already authenticated the operator: it
//!   is the owner and reaches every route, with no key or credential,
//!   and without spending the TCP rate budget.
//! - **The TCP listener (`:8092`).** Only other nodes use it. A node presents
//!   the credential this node issued it ([`crate::node_credentials`]) in the
//!   node-credential header and reaches only the lanes it was issued for.
//!   Owner-only routes are never reachable over TCP, from any peer.
//!
//! [`require_job_api`] guards the job API: owner-only except the two calls a
//! drone's offload session makes ([`job_api_lane`]). [`require_lane`] guards
//! each lane router. TCP callers share one rate budget on the job API.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::{Request, State};
use axum::http::{Method, StatusCode};
use axum::middleware::Next;
use axum::response::{IntoResponse, Response};
use axum::Json;

use world_engine_protocol::node_credential::{NodeLane, NODE_CREDENTIAL_HEADER};
use world_engine_transport::UnixPeer;

use crate::node_credentials::NodeCredentialStore;

/// A fixed-window token bucket that guards the TCP edge of the job API so a
/// runaway caller cannot pin the node. Operator-socket requests are never
/// limited.
pub struct RateLimiter {
    capacity: u32,
    window: Duration,
    state: Mutex<RateState>,
}

struct RateState {
    tokens: u32,
    window_start: Instant,
}

impl RateLimiter {
    /// A limiter granting `capacity` requests per `window`.
    pub fn new(capacity: u32, window: Duration) -> Self {
        Self {
            capacity,
            window,
            state: Mutex::new(RateState {
                tokens: capacity,
                window_start: Instant::now(),
            }),
        }
    }

    /// The default TCP budget: a generous per-second rate that session health
    /// polling and offload submits both fit under.
    pub fn default_control() -> Self {
        Self::new(120, Duration::from_secs(1))
    }

    /// Try to admit one request; `false` when the window's budget is exhausted.
    pub fn check(&self) -> bool {
        let mut s = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.window_start.elapsed() >= self.window {
            s.window_start = Instant::now();
            s.tokens = self.capacity;
        }
        if s.tokens > 0 {
            s.tokens -= 1;
            true
        } else {
            false
        }
    }
}

/// The auth state the middleware carries: the TCP rate limiter for the job API
/// and the credentials this node issued.
pub struct ComputeAuth {
    pub limiter: RateLimiter,
    pub credentials: NodeCredentialStore,
}

impl ComputeAuth {
    /// Build the auth state over the issued credential store, with the default
    /// TCP rate budget.
    pub fn new(credentials: NodeCredentialStore) -> Self {
        Self {
            limiter: RateLimiter::default_control(),
            credentials,
        }
    }
}

/// The node lane a job-API request belongs to, when another node may make it:
/// submitting an offload session and reading session health. Everything else
/// on the job API (listing, cancelling, datasets, outputs, issuing credentials)
/// is the owner's alone.
pub fn job_api_lane(method: &Method, path: &str) -> Option<NodeLane> {
    match (method, path) {
        (&Method::POST, "/api/compute/jobs") | (&Method::GET, "/api/compute/sessions") => {
            Some(crate::lanes::JOB_SUBMIT)
        }
        _ => None,
    }
}

/// Decide one TCP request: admitted only when it names a node lane and
/// presents a credential issued for that lane. An owner-only route (`lane:
/// None`) is never admitted over TCP.
pub fn admits(
    credential: Option<&str>,
    lane: Option<&NodeLane>,
    credentials: &NodeCredentialStore,
) -> bool {
    match (credential, lane) {
        (Some(c), Some(lane)) => credentials.admits(c, lane),
        _ => false,
    }
}

/// True when the request arrived on the plugin's operator socket: ados-control
/// authenticated the operator before proxying it, so it is the owner.
fn is_operator_socket(req: &Request) -> bool {
    req.extensions().get::<UnixPeer>().is_some()
}

fn presented_credential(req: &Request) -> Option<&str> {
    req.headers()
        .get(NODE_CREDENTIAL_HEADER)
        .and_then(|v| v.to_str().ok())
}

/// A terse, state-independent 401: the status itself is the only signal.
fn unauthorized() -> Response {
    (
        StatusCode::UNAUTHORIZED,
        Json(serde_json::json!({ "error": "unauthorized" })),
    )
        .into_response()
}

/// Axum middleware for the job API: the operator reaches every route; another
/// node reaches only the routes [`job_api_lane`] names, with a credential
/// issued for that lane. TCP callers share one rate budget (429 when spent).
pub async fn require_job_api(
    State(auth): State<Arc<ComputeAuth>>,
    req: Request,
    next: Next,
) -> Response {
    if is_operator_socket(&req) {
        return next.run(req).await;
    }
    if !auth.limiter.check() {
        return (
            StatusCode::TOO_MANY_REQUESTS,
            Json(serde_json::json!({ "error": "rate limited" })),
        )
            .into_response();
    }
    let lane = job_api_lane(req.method(), req.uri().path());
    if admits(presented_credential(&req), lane.as_ref(), &auth.credentials) {
        next.run(req).await
    } else {
        unauthorized()
    }
}

/// Axum middleware for one lane router: the operator, or a node credential
/// issued for `lane`.
pub async fn require_lane(
    State((auth, lane)): State<(Arc<ComputeAuth>, NodeLane)>,
    req: Request,
    next: Next,
) -> Response {
    if is_operator_socket(&req)
        || admits(presented_credential(&req), Some(&lane), &auth.credentials)
    {
        next.run(req).await
    } else {
        unauthorized()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(dir: &std::path::Path) -> NodeCredentialStore {
        NodeCredentialStore::open(dir.join("creds.json"), "ws-node")
    }

    #[test]
    fn a_drone_credential_reaches_only_its_lanes_and_never_owner_routes() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        let m = s
            .mint(
                "drone-1",
                &[crate::lanes::ATLAS_INGEST, crate::lanes::JOB_SUBMIT],
                1,
            )
            .unwrap();
        let cred = Some(m.credential.as_str());
        assert!(admits(cred, Some(&crate::lanes::ATLAS_INGEST), &s));
        assert!(admits(cred, Some(&crate::lanes::JOB_SUBMIT), &s));
        // A lane it was not issued for.
        assert!(!admits(cred, Some(&crate::lanes::ARTIFACTS), &s));
        // An owner-only route (issuing credentials, listing jobs, ...).
        assert!(!admits(cred, None, &s));
    }

    #[test]
    fn only_offload_submit_and_session_health_are_node_reachable_on_the_job_api() {
        assert_eq!(
            job_api_lane(&Method::POST, "/api/compute/jobs"),
            Some(crate::lanes::JOB_SUBMIT)
        );
        assert_eq!(
            job_api_lane(&Method::GET, "/api/compute/sessions"),
            Some(crate::lanes::JOB_SUBMIT)
        );
        for (m, p) in [
            (Method::GET, "/api/compute/jobs"),
            (Method::GET, "/api/compute/status"),
            (Method::POST, "/api/compute/jobs/j/cancel"),
            (Method::POST, "/api/compute/datasets"),
            (Method::POST, "/api/compute/node-credentials"),
        ] {
            assert_eq!(job_api_lane(&m, p), None, "{m} {p}");
        }
    }
}
