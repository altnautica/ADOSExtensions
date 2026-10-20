//! Auth for the compute node's listeners.
//!
//! - **The operator socket (`http.sock`, plugin-hosted node).** A request
//!   marked with [`UnixPeer`] came through ados-control, which already
//!   authenticated the operator: it is the owner and reaches every route, with
//!   no key or credential, and without spending the TCP rate budget.
//! - **The owner bearer (standalone node).** With no operator socket, the
//!   owner presents the node's token as `Authorization: Bearer <token>` over
//!   TCP and reaches every route.
//! - **Other nodes, over TCP (`:8092`).** A node presents the credential this
//!   node issued it ([`crate::node_credentials`]) in the node-credential
//!   header and reaches only the lanes it was issued for. Owner-only routes
//!   are never reachable with a node credential.
//!
//! [`require_job_api`] guards the job API: owner-only except the calls another
//! node makes ([`job_api_lane`]) and the node's identity, which any caller may
//! read ([`public_job_api_route`]). [`require_lane`] guards each lane router.
//! TCP callers share one rate budget on the job API.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::{Request, State};
use axum::http::{Method, StatusCode};
use axum::middleware::Next;
use axum::response::{IntoResponse, Response};
use axum::Json;

use world_engine_protocol::node_credential::{NodeLane, NODE_CREDENTIAL_HEADER};
#[cfg(unix)]
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

/// The auth state the middleware carries: the TCP rate limiter for the job API,
/// the credentials this node issued, and (standalone node only) the owner's
/// bearer token.
pub struct ComputeAuth {
    pub limiter: RateLimiter,
    pub credentials: NodeCredentialStore,
    /// A standalone node (no plugin host, so no operator socket) admits the
    /// owner over TCP by this bearer token (`Authorization: Bearer <token>`).
    owner_token: Option<String>,
}

impl ComputeAuth {
    /// Build the auth state over the issued credential store, with the default
    /// TCP rate budget.
    pub fn new(credentials: NodeCredentialStore) -> Self {
        Self {
            limiter: RateLimiter::default_control(),
            credentials,
            owner_token: None,
        }
    }

    /// Admit the owner over TCP by `token` as well (the standalone node).
    pub fn with_owner_token(mut self, token: String) -> Self {
        self.owner_token = Some(token);
        self
    }

    /// Whether the request carries the owner's bearer token.
    fn bearer_is_owner(&self, req: &Request) -> bool {
        let Some(token) = self.owner_token.as_deref() else {
            return false;
        };
        req.headers()
            .get(axum::http::header::AUTHORIZATION)
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.strip_prefix("Bearer "))
            .is_some_and(|given| constant_time_eq(given.trim().as_bytes(), token.as_bytes()))
    }
}

/// Compare two secrets without an early exit on the first differing byte.
pub(crate) fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let diff = a.iter().zip(b).fold(0u8, |d, (x, y)| d | (x ^ y));
    std::hint::black_box(diff) == 0
}

/// The node lane a job-API request belongs to, when another node may make it:
/// submitting a job, reading session health, and reading the status and
/// outputs of a job it submitted itself (scoped by [`NodeCaller`] in the
/// handlers). Everything else on the job API (listing, cancelling, datasets,
/// issuing credentials) is the owner's alone.
pub fn job_api_lane(method: &Method, path: &str) -> Option<NodeLane> {
    match (method, path) {
        (&Method::POST, "/api/compute/jobs") | (&Method::GET, "/api/compute/sessions") => {
            Some(crate::lanes::JOB_SUBMIT)
        }
        (&Method::GET, p) if own_job_read(p) => Some(crate::lanes::JOB_SUBMIT),
        _ => None,
    }
}

/// `/api/compute/jobs/<id>` or `/api/compute/jobs/<id>/outputs`.
fn own_job_read(path: &str) -> bool {
    let Some(rest) = path.strip_prefix("/api/compute/jobs/") else {
        return false;
    };
    let mut parts = rest.split('/');
    let id_ok = parts.next().is_some_and(|id| !id.is_empty());
    id_ok
        && matches!(
            (parts.next(), parts.next()),
            (None, _) | (Some("outputs"), None)
        )
}

/// Whether a job-API request is open to any caller: only reading the node's
/// identity, which it also advertises over mDNS.
pub fn public_job_api_route(method: &Method, path: &str) -> bool {
    method == Method::GET && path == world_engine_protocol::compute::NODE_IDENTITY_PATH
}

/// The issued credential a node request was admitted with (its id), set on
/// the request by [`require_job_api`]; handlers scope a node's job reads to
/// the jobs it submitted. Absent for the owner.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeCaller(pub String);

/// The id part of an `nc1.<id>.<secret>` credential.
fn credential_id(req: &Request) -> Option<String> {
    let token = req.headers().get(NODE_CREDENTIAL_HEADER)?.to_str().ok()?;
    let mut parts = token.trim().splitn(3, '.');
    parts.next()?;
    parts.next().map(str::to_string)
}

/// Decide one node request: admitted only when it names a node lane and
/// presents a credential issued for that lane. An owner-only route (`lane:
/// None`) is never admitted with a node credential.
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

/// True when the request comes from the owner: through the plugin's operator
/// socket (ados-control authenticated the operator before proxying it), or,
/// on a standalone node, with the owner's bearer token.
fn is_owner(auth: &ComputeAuth, req: &Request) -> bool {
    #[cfg(unix)]
    if req.extensions().get::<UnixPeer>().is_some() {
        return true;
    }
    auth.bearer_is_owner(req)
}

/// Whether a node credential issued for `lane` rides the request.
fn node_admitted(auth: &ComputeAuth, req: &Request, lane: Option<&NodeLane>) -> bool {
    let credential = req
        .headers()
        .get(NODE_CREDENTIAL_HEADER)
        .and_then(|v| v.to_str().ok());
    admits(credential, lane, &auth.credentials)
}

/// A terse, state-independent 401: the status itself is the only signal.
fn unauthorized() -> Response {
    (
        StatusCode::UNAUTHORIZED,
        Json(serde_json::json!({ "error": "unauthorized" })),
    )
        .into_response()
}

/// Axum middleware for the job API: the owner reaches every route; another
/// node reaches only the routes [`job_api_lane`] names, with a credential
/// issued for that lane. TCP callers share one rate budget (429 when spent).
pub async fn require_job_api(
    State(auth): State<Arc<ComputeAuth>>,
    req: Request,
    next: Next,
) -> Response {
    // The operator socket is never rate limited.
    #[cfg(unix)]
    if req.extensions().get::<UnixPeer>().is_some() {
        return next.run(req).await;
    }
    if !auth.limiter.check() {
        return (
            StatusCode::TOO_MANY_REQUESTS,
            Json(serde_json::json!({ "error": "rate limited" })),
        )
            .into_response();
    }
    if public_job_api_route(req.method(), req.uri().path()) {
        return next.run(req).await;
    }
    let lane = job_api_lane(req.method(), req.uri().path());
    if is_owner(&auth, &req) {
        return next.run(req).await;
    }
    if node_admitted(&auth, &req, lane.as_ref()) {
        let mut req = req;
        if let Some(id) = credential_id(&req) {
            req.extensions_mut().insert(NodeCaller(id));
        }
        next.run(req).await
    } else {
        unauthorized()
    }
}

/// Axum middleware for one lane router: the owner, or a node credential
/// issued for `lane`.
pub async fn require_lane(
    State((auth, lane)): State<(Arc<ComputeAuth>, NodeLane)>,
    req: Request,
    next: Next,
) -> Response {
    if is_owner(&auth, &req) || node_admitted(&auth, &req, Some(&lane)) {
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
    fn a_node_may_read_single_jobs_and_their_outputs_but_nothing_wider() {
        for p in ["/api/compute/jobs/j-1", "/api/compute/jobs/j-1/outputs"] {
            assert_eq!(
                job_api_lane(&Method::GET, p),
                Some(crate::lanes::JOB_SUBMIT),
                "{p}"
            );
        }
        for (m, p) in [
            (Method::GET, "/api/compute/jobs/"),
            (Method::GET, "/api/compute/jobs/j-1/files/x.json"),
            (Method::GET, "/api/compute/jobs/j-1/tour.zip"),
            (Method::GET, "/api/compute/jobs/j-1/outputs/x"),
            (Method::DELETE, "/api/compute/jobs/j-1"),
            (Method::POST, "/api/compute/jobs/j-1/cancel"),
        ] {
            assert_eq!(job_api_lane(&m, p), None, "{m} {p}");
        }
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

    #[test]
    fn only_reading_the_identity_is_open_to_any_caller() {
        assert!(public_job_api_route(&Method::GET, "/api/compute/identity"));
        for (m, p) in [
            (Method::POST, "/api/compute/identity"),
            (Method::GET, "/api/compute/identity/x"),
            (Method::GET, "/api/compute/status"),
            (Method::GET, "/api/compute/node-credentials"),
        ] {
            assert!(!public_job_api_route(&m, p), "{m} {p}");
        }
    }

    fn auth_with_token(token: &str) -> ComputeAuth {
        let dir = tempfile::tempdir().unwrap();
        ComputeAuth::new(store(dir.path())).with_owner_token(token.into())
    }

    fn req_with(header: Option<&str>) -> Request {
        let mut b = axum::http::Request::builder().uri("/api/compute/jobs");
        if let Some(h) = header {
            b = b.header(axum::http::header::AUTHORIZATION, h);
        }
        b.body(axum::body::Body::empty()).unwrap()
    }

    #[test]
    fn the_owner_bearer_admits_only_the_exact_token() {
        let auth = auth_with_token("s3cret-token");
        assert!(is_owner(&auth, &req_with(Some("Bearer s3cret-token"))));
        assert!(!is_owner(&auth, &req_with(Some("Bearer s3cret-tokeX"))));
        assert!(!is_owner(&auth, &req_with(Some("s3cret-token"))));
        assert!(!is_owner(&auth, &req_with(None)));
        assert!(constant_time_eq(b"abc", b"abc"));
        assert!(!constant_time_eq(b"abc", b"abcd"));
    }
}
