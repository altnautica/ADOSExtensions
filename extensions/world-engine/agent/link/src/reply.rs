//! Error replies of the capture-device routes and tools:
//! `{"error": CODE, "detail": ".."}` with a status from the code.

use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde_json::{json, Value};
use world_engine_protocol::capture_ctl;
use world_engine_transport::ClientError as NodeClientError;

use crate::control::ControlError;

/// Codes the link itself answers with (the capture service's own refusal
/// codes pass through from [`capture_ctl::codes`]).
pub mod codes {
    pub use world_engine_protocol::capture_ctl::codes::BAD_REQUEST;
    /// The capture service is not running.
    pub const CAPTURE_UNAVAILABLE: &str = "E_CAPTURE_UNAVAILABLE";
    /// The capture service did not answer in time; the command may still apply.
    pub const CAPTURE_TIMEOUT: &str = "E_CAPTURE_TIMEOUT";
    /// The capture service answered something that is not a control reply.
    pub const CAPTURE_REPLY: &str = "E_CAPTURE_REPLY";
    pub const UNKNOWN_SESSION: &str = "E_UNKNOWN_SESSION";
    /// The session is still recording.
    pub const RECORDING: &str = "E_RECORDING";
    /// The session is queued for upload or uploading.
    pub const UPLOADING: &str = "E_UPLOADING";
    /// The session has not been committed on a node.
    pub const NOT_UPLOADED: &str = "E_NOT_UPLOADED";
    /// The capture root is unknown (no capture-storage sidecar).
    pub const NO_STORAGE: &str = "E_NO_STORAGE";
    /// No compute node is pinned or discovered.
    pub const NO_NODE: &str = "E_NO_NODE";
    /// The node's address is not a LAN address.
    pub const NOT_LAN: &str = "E_NOT_LAN";
    pub const NODE_UNREACHABLE: &str = "E_NODE_UNREACHABLE";
    /// The node refused this device's credential.
    pub const NODE_UNAUTHORIZED: &str = "E_NODE_UNAUTHORIZED";
    pub const NODE: &str = "E_NODE";
    /// The job produced no tour bundle.
    pub const NO_TOUR: &str = "E_NO_TOUR";
    pub const NO_CALIB: &str = "E_NO_CALIB";
    pub const IO: &str = "E_IO";
    pub const WHEP_UNREACHABLE: &str = "E_WHEP_UNREACHABLE";
    pub const RIG_INVALID: &str = "E_RIG_INVALID";
    pub const RIG_UNREACHABLE: &str = "E_RIG_UNREACHABLE";
}

/// A refused route or tool call.
#[derive(Debug, Clone, PartialEq)]
pub struct ApiError {
    pub status: StatusCode,
    pub code: String,
    pub detail: String,
}

impl ApiError {
    pub fn new(status: StatusCode, code: &str, detail: impl Into<String>) -> Self {
        Self {
            status,
            code: code.to_string(),
            detail: detail.into(),
        }
    }

    pub fn bad_request(detail: impl Into<String>) -> Self {
        Self::new(StatusCode::BAD_REQUEST, codes::BAD_REQUEST, detail)
    }

    pub fn io(e: impl std::fmt::Display) -> Self {
        Self::new(StatusCode::INTERNAL_SERVER_ERROR, codes::IO, e.to_string())
    }

    /// The message a tool call fails with.
    pub fn tool_message(&self) -> String {
        format!("{}: {}", self.code, self.detail)
    }
}

impl From<ControlError> for ApiError {
    fn from(e: ControlError) -> Self {
        if e.not_running() {
            return Self::new(
                StatusCode::SERVICE_UNAVAILABLE,
                codes::CAPTURE_UNAVAILABLE,
                "the capture service is not running",
            );
        }
        match e {
            ControlError::Refused { code, detail } => {
                let status = StatusCode::from_u16(capture_ctl::http_status(&code))
                    .unwrap_or(StatusCode::INTERNAL_SERVER_ERROR);
                Self {
                    status,
                    code,
                    detail,
                }
            }
            ControlError::Timeout => Self::new(
                StatusCode::GATEWAY_TIMEOUT,
                codes::CAPTURE_TIMEOUT,
                "the capture service did not confirm in time; the command may still apply",
            ),
            other => Self::new(
                StatusCode::BAD_GATEWAY,
                codes::CAPTURE_REPLY,
                other.to_string(),
            ),
        }
    }
}

impl From<NodeClientError> for ApiError {
    fn from(e: NodeClientError) -> Self {
        match e {
            NodeClientError::Http(401, _) => Self::new(
                StatusCode::BAD_GATEWAY,
                codes::NODE_UNAUTHORIZED,
                "the node did not accept this device's credential for the route",
            ),
            NodeClientError::Http(status, body) => {
                let parsed: Value = serde_json::from_str(&body).unwrap_or(Value::Null);
                let code = parsed
                    .get("error")
                    .and_then(Value::as_str)
                    .filter(|c| c.starts_with("E_"))
                    .unwrap_or(codes::NODE)
                    .to_string();
                let detail = parsed
                    .get("detail")
                    .or_else(|| parsed.get("message"))
                    .and_then(Value::as_str)
                    .map(str::to_string)
                    .unwrap_or(body);
                let status = StatusCode::from_u16(status)
                    .ok()
                    .filter(|s| s.is_client_error())
                    .unwrap_or(StatusCode::BAD_GATEWAY);
                Self {
                    status,
                    code,
                    detail,
                }
            }
            NodeClientError::Request(m) => {
                Self::new(StatusCode::BAD_GATEWAY, codes::NODE_UNREACHABLE, m)
            }
            NodeClientError::Decode(m) => Self::new(StatusCode::BAD_GATEWAY, codes::NODE, m),
        }
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        (
            self.status,
            Json(json!({ "error": self.code, "detail": self.detail })),
        )
            .into_response()
    }
}

/// A route reply: the value, or the refusal.
pub fn respond(r: Result<Value, ApiError>) -> Response {
    match r {
        Ok(v) => Json(v).into_response(),
        Err(e) => e.into_response(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_refusals_keep_their_code_and_status() {
        let e = ApiError::from(ControlError::Refused {
            code: "E_STORAGE_FULL".into(),
            detail: "full".into(),
        });
        assert_eq!(e.status, StatusCode::CONFLICT);
        assert_eq!(e.code, "E_STORAGE_FULL");
        let gone = std::io::Error::from(std::io::ErrorKind::NotFound);
        assert_eq!(
            ApiError::from(ControlError::Io(gone)).status,
            StatusCode::SERVICE_UNAVAILABLE
        );
        assert_eq!(
            ApiError::from(ControlError::Timeout).status,
            StatusCode::GATEWAY_TIMEOUT
        );
    }

    #[test]
    fn node_refusals_pass_their_code_through() {
        let e = ApiError::from(NodeClientError::Http(
            409,
            r#"{"error":"E_NOT_UPLOADED","detail":"session s-1 is not committed"}"#.into(),
        ));
        assert_eq!(e.status, StatusCode::CONFLICT);
        assert_eq!(e.code, "E_NOT_UPLOADED");
        assert_eq!(e.detail, "session s-1 is not committed");
        let e = ApiError::from(NodeClientError::Http(500, "boom".into()));
        assert_eq!(
            (e.status, e.code.as_str()),
            (StatusCode::BAD_GATEWAY, "E_NODE")
        );
    }
}
