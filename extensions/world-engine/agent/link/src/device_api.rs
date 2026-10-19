//! The two routes the agent itself serves, relayed through host methods (the
//! sandbox cannot reach the agent's own listeners):
//!
//! - `POST /atlas/capture/whep?camera=<id>` (body: the SDP offer,
//!   `application/sdp`) → `201` with the SDP answer and a `Location` of
//!   `whep?location=<opaque>` (relative to this route), through
//!   `video.whep.offer`. `<id>` is the camera's own id (`[A-Za-z0-9_-]+`); the
//!   host resolves it to the media leg that serves it (the primary camera to
//!   `main`) and refuses one that is not a configured, enabled camera (`404
//!   E_UNKNOWN_CAMERA`).
//! - `DELETE /atlas/capture/whep?location=<opaque>` → `{"ok":true}`, through
//!   `video.whep.close`.
//! - `PUT /atlas/rig` (a contract rig document): checked locally (`400
//!   E_RIG_INVALID`), then relayed through `sensors.rig.write` to core `PUT
//!   /api/v1/sensors/rig`; the core status and body are returned verbatim.
//!
//! Host refusals: a missing capability is `403 E_CAPABILITY_DENIED`; the agent
//! endpoint being down (`*_unavailable`) or a host without the method is
//! `503 E_WHEP_UNREACHABLE` / `E_RIG_UNREACHABLE`.

use ados_sdk::ClientError;
use axum::body::Bytes;
use axum::extract::{Query, State};
use axum::http::{header, HeaderValue, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{post, put};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::api::DroneApi;
use crate::reply::{codes, ApiError};

/// Largest SDP offer relayed.
const MAX_SDP: usize = 64 * 1024;

pub fn routes() -> Router<DroneApi> {
    Router::new()
        .route("/atlas/capture/whep", post(whep_offer).delete(whep_close))
        .route("/atlas/rig", put(put_rig))
}

/// A host-method failure as a route refusal.
pub fn host_error(e: ClientError, unreachable: &str) -> ApiError {
    match e {
        ClientError::CapabilityDenied(cap) => ApiError::new(
            StatusCode::FORBIDDEN,
            codes::CAPABILITY_DENIED,
            format!("the extension is not granted `{cap}`"),
        ),
        ClientError::Rpc(m) => {
            let head = m.split(':').next().unwrap_or("").trim();
            if head == "unknown_camera" {
                return ApiError::new(StatusCode::NOT_FOUND, codes::UNKNOWN_CAMERA, m);
            }
            let down = head.ends_with("_unavailable")
                || matches!(head, "not_implemented" | "not_available" | "unknown_method");
            if down {
                ApiError::new(StatusCode::SERVICE_UNAVAILABLE, unreachable, m)
            } else {
                ApiError::new(StatusCode::BAD_GATEWAY, codes::HOST, m)
            }
        }
        other => ApiError::new(
            StatusCode::SERVICE_UNAVAILABLE,
            unreachable,
            other.to_string(),
        ),
    }
}

/// Percent-encode a query value.
fn pct(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => {
                out.push(b as char)
            }
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

/// A camera id the host relays: `[A-Za-z0-9_-]+`, the host's path-segment
/// rule (no length bound there either).
fn camera_id_ok(id: &str) -> bool {
    !id.is_empty()
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-'))
}

#[derive(Debug, Default, Deserialize)]
struct WhepQuery {
    camera: Option<String>,
    location: Option<String>,
}

async fn whep_offer(
    State(api): State<DroneApi>,
    Query(q): Query<WhepQuery>,
    body: Bytes,
) -> Response {
    let Some(camera) = q.camera.filter(|c| camera_id_ok(c)) else {
        return ApiError::bad_request("`camera` must be a camera id ([A-Za-z0-9_-]+)")
            .into_response();
    };
    if body.is_empty() || body.len() > MAX_SDP {
        return ApiError::bad_request("the body must be an SDP offer").into_response();
    }
    let Ok(offer) = std::str::from_utf8(&body) else {
        return ApiError::bad_request("the SDP offer is not UTF-8").into_response();
    };
    match api.host.whep_offer(&camera, offer).await {
        Ok(answer) => {
            let mut resp = (StatusCode::CREATED, answer.sdp).into_response();
            let h = resp.headers_mut();
            h.insert(
                header::CONTENT_TYPE,
                HeaderValue::from_static("application/sdp"),
            );
            if let Some(loc) = answer.location {
                if let Ok(v) = HeaderValue::from_str(&format!("whep?location={}", pct(&loc))) {
                    h.insert(header::LOCATION, v);
                }
            }
            resp
        }
        Err(e) => host_error(e, codes::WHEP_UNREACHABLE).into_response(),
    }
}

async fn whep_close(State(api): State<DroneApi>, Query(q): Query<WhepQuery>) -> Response {
    let Some(location) = q.location.filter(|l| !l.is_empty()) else {
        return ApiError::bad_request("`location` is the value the offer returned").into_response();
    };
    match api.host.whep_close(&location).await {
        Ok(()) => Json(json!({ "ok": true })).into_response(),
        Err(e) => host_error(e, codes::WHEP_UNREACHABLE).into_response(),
    }
}

async fn put_rig(State(api): State<DroneApi>, body: Bytes) -> Response {
    if let Err(e) = serde_json::from_slice::<ados_capture_contract::Rig>(&body) {
        return ApiError::new(StatusCode::BAD_REQUEST, codes::RIG_INVALID, e.to_string())
            .into_response();
    }
    let rig: Value = match serde_json::from_slice(&body) {
        Ok(v) => v,
        Err(e) => {
            return ApiError::new(StatusCode::BAD_REQUEST, codes::RIG_INVALID, e.to_string())
                .into_response()
        }
    };
    match api.host.put_rig(&rig).await {
        Ok((status, body)) => match StatusCode::from_u16(status) {
            Ok(s) => (s, Json(body)).into_response(),
            Err(_) => ApiError::new(
                StatusCode::BAD_GATEWAY,
                codes::HOST,
                format!("the agent answered status {status}"),
            )
            .into_response(),
        },
        Err(e) => host_error(e, codes::RIG_UNREACHABLE).into_response(),
    }
}

#[cfg(test)]
#[path = "device_api_tests.rs"]
mod tests;
