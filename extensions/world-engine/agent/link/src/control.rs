//! The capture service's control-socket client.
//!
//! The capture service owns the drone's capture session and binds a control
//! socket (`atlas-control.sock` in the extension's private socket directory)
//! that starts, stops, pauses, resumes or reads the session without a restart.
//! The wire is one newline-delimited JSON request `{"cmd": "..."}` and one
//! newline-delimited JSON [`CaptureStatus`] reply (the status AFTER the command
//! applied). One fresh connection per call: control actions are infrequent.

use std::path::PathBuf;
use std::time::Duration;

use thiserror::Error;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use world_engine_protocol::atlas::CaptureStatus;

/// A bounded per-request timeout so a wedged capture service cannot hang the
/// route's connection; the reply for a local socket is near-instant.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(3);

/// A capture-control error.
#[derive(Debug, Error)]
pub enum ControlError {
    /// The control socket could not be reached or the I/O failed.
    #[error("atlas control socket io failed: {0}")]
    Io(#[from] std::io::Error),
    /// The reply line could not be parsed as a capture status.
    #[error("atlas control reply parse failed: {0}")]
    Parse(String),
    /// The request did not complete within the bounded timeout.
    #[error("atlas control request timed out")]
    Timeout,
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
    /// exchange, an unparseable reply). A busy capturing service can look like
    /// this, so nothing about the session is known.
    Unknown,
}

impl From<Result<CaptureStatus, ControlError>> for LiveCapture {
    fn from(r: Result<CaptureStatus, ControlError>) -> Self {
        match r {
            Ok(s) => Self::Running(s),
            Err(ControlError::Io(e))
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::NotFound | std::io::ErrorKind::ConnectionRefused
                ) =>
            {
                Self::NotRunning
            }
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

    /// Send `cmd` (`status` / `start` / `stop` / `pause` / `resume`) with the
    /// bounded timeout.
    pub async fn command(&self, cmd: &str) -> Result<CaptureStatus, ControlError> {
        match tokio::time::timeout(REQUEST_TIMEOUT, self.exchange(cmd)).await {
            Ok(result) => result,
            Err(_) => Err(ControlError::Timeout),
        }
    }

    async fn exchange(&self, cmd: &str) -> Result<CaptureStatus, ControlError> {
        let stream = UnixStream::connect(&self.socket_path).await?;
        let (read_half, mut write_half) = stream.into_split();
        let request = format!("{{\"cmd\":\"{cmd}\"}}\n");
        write_half.write_all(request.as_bytes()).await?;
        write_half.flush().await?;
        let mut reader = BufReader::new(read_half);
        let mut line = String::new();
        if reader.read_line(&mut line).await? == 0 {
            return Err(ControlError::Parse("empty reply".to_string()));
        }
        serde_json::from_str(line.trim()).map_err(|e| ControlError::Parse(e.to_string()))
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
            pose_tier: PoseSource::LocalVio,
            dropped_keyframes: 0,
        }
    }

    /// A stand-in control server: answers every connection's request line with
    /// `reply` and records the requests.
    pub fn fake_control_socket(path: &Path, reply: CaptureStatus) -> Arc<Mutex<Vec<String>>> {
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
                let mut body = serde_json::to_vec(&reply).unwrap();
                body.push(b'\n');
                let _ = write_half.write_all(&body).await;
            }
        });
        recorded
    }

    #[tokio::test]
    async fn a_command_sends_its_verb_and_reads_the_status() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("atlas-control.sock");
        let recorded = fake_control_socket(&socket, capturing_status());
        let client = ControlClient::new(socket);
        let status = client.command("stop").await.unwrap();
        assert_eq!(status.session_id, "sess-1");
        assert_eq!(recorded.lock()[0], r#"{"cmd":"stop"}"#);
    }

    #[tokio::test]
    async fn an_absent_socket_is_a_service_that_is_not_running() {
        let dir = tempfile::tempdir().unwrap();
        let client = ControlClient::new(dir.path().join("nope.sock"));
        assert!(matches!(
            LiveCapture::from(client.command("status").await),
            LiveCapture::NotRunning
        ));
        // A service that did not answer is unknown, not "not running".
        assert!(matches!(
            LiveCapture::from(Err(ControlError::Timeout)),
            LiveCapture::Unknown
        ));
    }
}
