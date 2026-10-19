//! The capture service's control-socket client (protocol v2,
//! [`world_engine_protocol::capture_ctl`]).
//!
//! The capture service binds `atlas-control.sock` in the extension's private
//! socket directory. One connection per exchange: one newline-delimited JSON
//! [`CtlRequest`] in, one newline-delimited JSON [`CtlReply`] out. A refusal
//! (`ok: false`) surfaces as [`ControlError::Refused`] with its code.

use std::path::PathBuf;
use std::time::Duration;

use serde_json::Value;
use thiserror::Error;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use world_engine_protocol::atlas::CaptureStatus;
use world_engine_protocol::capture_ctl::{CtlReply, CtlRequest};

/// A bounded per-request timeout so a wedged capture service cannot hang the
/// route's connection; a local reply is near-instant.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(3);
/// `stop` closes the last segment and writes the manifest and checksums.
const STOP_TIMEOUT: Duration = Duration::from_secs(10);

/// A capture-control error.
#[derive(Debug, Error)]
pub enum ControlError {
    /// The control socket could not be reached or the I/O failed.
    #[error("atlas control socket io failed: {0}")]
    Io(#[from] std::io::Error),
    /// The reply line was not a control reply (or its result did not decode).
    #[error("atlas control reply parse failed: {0}")]
    Parse(String),
    /// The request did not complete within the bounded timeout.
    #[error("atlas control request timed out")]
    Timeout,
    /// The service refused the request.
    #[error("{code}: {detail}")]
    Refused { code: String, detail: String },
}

impl ControlError {
    /// Nothing is listening: the service is not running.
    pub fn not_running(&self) -> bool {
        matches!(
            self,
            Self::Io(e) if matches!(
                e.kind(),
                std::io::ErrorKind::NotFound | std::io::ErrorKind::ConnectionRefused
            )
        )
    }
}

/// What the control socket said about the live session.
#[derive(Debug)]
pub enum LiveCapture {
    /// The service answered with its status.
    Running(CaptureStatus),
    /// Nothing is listening: the service is not running (Atlas disabled, or no
    /// cameras), so there is no session.
    NotRunning,
    /// The service is there but did not answer usefully (a timeout, a broken
    /// exchange, a refusal). Nothing about the session is known.
    Unknown,
}

impl From<Result<CaptureStatus, ControlError>> for LiveCapture {
    fn from(r: Result<CaptureStatus, ControlError>) -> Self {
        match r {
            Ok(s) => Self::Running(s),
            Err(e) if e.not_running() => Self::NotRunning,
            Err(_) => Self::Unknown,
        }
    }
}

/// Runs single request/response exchanges on the control socket.
#[derive(Debug, Clone)]
pub struct ControlClient {
    socket_path: PathBuf,
}

impl ControlClient {
    pub fn new(socket_path: PathBuf) -> Self {
        Self { socket_path }
    }

    /// Send `req`; the reply's `result` on success.
    pub async fn request(&self, req: &CtlRequest) -> Result<Value, ControlError> {
        let limit = match req {
            CtlRequest::Stop => STOP_TIMEOUT,
            _ => REQUEST_TIMEOUT,
        };
        let reply = match tokio::time::timeout(limit, self.exchange(req)).await {
            Ok(r) => r?,
            Err(_) => return Err(ControlError::Timeout),
        };
        if reply.ok {
            return Ok(reply.result);
        }
        Err(ControlError::Refused {
            code: reply.error.unwrap_or_else(|| "E_UNKNOWN".to_string()),
            detail: reply.detail.unwrap_or_default(),
        })
    }

    /// Send `req` and decode its result as `T`.
    pub async fn call<T: serde::de::DeserializeOwned>(
        &self,
        req: &CtlRequest,
    ) -> Result<T, ControlError> {
        let v = self.request(req).await?;
        serde_json::from_value(v).map_err(|e| ControlError::Parse(e.to_string()))
    }

    /// The live session status (`status`).
    pub async fn status(&self) -> Result<CaptureStatus, ControlError> {
        self.call(&CtlRequest::Status).await
    }

    async fn exchange(&self, req: &CtlRequest) -> Result<CtlReply, ControlError> {
        let stream = UnixStream::connect(&self.socket_path).await?;
        let (read_half, mut write_half) = stream.into_split();
        let mut line = serde_json::to_vec(req).map_err(|e| ControlError::Parse(e.to_string()))?;
        line.push(b'\n');
        write_half.write_all(&line).await?;
        write_half.flush().await?;
        let mut reader = BufReader::new(read_half);
        let mut reply = String::new();
        if reader.read_line(&mut reply).await? == 0 {
            return Err(ControlError::Parse("empty reply".to_string()));
        }
        serde_json::from_str(reply.trim()).map_err(|e| ControlError::Parse(e.to_string()))
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use parking_lot::Mutex;
    use std::path::Path;
    use std::sync::Arc;
    use tokio::net::UnixListener;
    use world_engine_protocol::atlas::{CaptureState, PoseSource, VioHealth};

    pub fn capturing_status() -> CaptureStatus {
        CaptureStatus {
            session_id: "sess-1".to_string(),
            state: CaptureState::Capturing,
            keyframes: 4,
            vio_health: VioHealth::Good,
            camera_count: 1,
            ingest_rate_hz: 9.5,
            capped: false,
            anchored: true,
            pose_tier: PoseSource::FcEkf,
            dropped_keyframes: 0,
        }
    }

    /// A stand-in control server: answers every request with `answer` and
    /// records the request lines.
    pub fn fake_control_socket(
        path: &Path,
        answer: impl Fn(CtlRequest) -> CtlReply + Send + Sync + 'static,
    ) -> Arc<Mutex<Vec<String>>> {
        let listener = UnixListener::bind(path).unwrap();
        let recorded = Arc::new(Mutex::new(Vec::new()));
        let rec = recorded.clone();
        tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let (read_half, mut write_half) = stream.into_split();
                let mut reader = BufReader::new(read_half);
                let mut line = String::new();
                let _ = reader.read_line(&mut line).await;
                rec.lock().push(line.trim().to_string());
                let reply = match serde_json::from_str::<CtlRequest>(line.trim()) {
                    Ok(req) => answer(req),
                    Err(e) => CtlReply::err("E_BAD_REQUEST", e.to_string()),
                };
                let mut body = serde_json::to_vec(&reply).unwrap();
                body.push(b'\n');
                let _ = write_half.write_all(&body).await;
            }
        });
        recorded
    }

    /// A control server that answers `status` with `status` and every other
    /// request with `ok` and a null result.
    pub fn fake_status_socket(path: &Path, status: CaptureStatus) -> Arc<Mutex<Vec<String>>> {
        fake_control_socket(path, move |req| match req {
            CtlRequest::Status => CtlReply::ok(&status),
            _ => CtlReply::ok(Value::Null),
        })
    }

    #[tokio::test]
    async fn a_request_carries_its_cmd_and_reads_the_result() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("atlas-control.sock");
        let recorded = fake_status_socket(&socket, capturing_status());
        let client = ControlClient::new(socket);
        let status = client.status().await.unwrap();
        assert_eq!(status.session_id, "sess-1");
        assert_eq!(recorded.lock()[0], r#"{"cmd":"status"}"#);
    }

    #[tokio::test]
    async fn a_refusal_carries_its_code() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("atlas-control.sock");
        fake_control_socket(&socket, |_| CtlReply::err("E_STORAGE_FULL", "full"));
        let err = ControlClient::new(socket)
            .request(&CtlRequest::Stop)
            .await
            .unwrap_err();
        assert!(
            matches!(&err, ControlError::Refused { code, detail } if code == "E_STORAGE_FULL" && detail == "full"),
            "{err:?}"
        );
    }

    #[tokio::test]
    async fn an_absent_socket_is_a_service_that_is_not_running() {
        let dir = tempfile::tempdir().unwrap();
        let client = ControlClient::new(dir.path().join("nope.sock"));
        assert!(matches!(
            LiveCapture::from(client.status().await),
            LiveCapture::NotRunning
        ));
        // A service that did not answer is unknown, not "not running".
        assert!(matches!(
            LiveCapture::from(Err(ControlError::Timeout)),
            LiveCapture::Unknown
        ));
    }
}
