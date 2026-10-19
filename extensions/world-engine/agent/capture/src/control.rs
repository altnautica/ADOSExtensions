//! The capture control socket (`atlas-control.sock`, protocol v2, see
//! [`world_engine_protocol::capture_ctl`]).
//!
//! Each connection carries one newline-delimited JSON [`CtlRequest`]; it is
//! handed to the capture loop (the single owner of the session) with a
//! one-shot for the [`CtlReply`], which is written back as one line. A line
//! that does not parse is answered `E_BAD_REQUEST` without reaching the loop.

use std::io;

use ados_protocol::ipc::bind_command_socket;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;
use world_engine_protocol::capture_ctl::{codes, CtlReply, CtlRequest};

/// A request for the capture loop and where its reply goes.
pub struct ControlMsg {
    pub req: CtlRequest,
    pub reply: oneshot::Sender<CtlReply>,
}

/// Longest request line accepted.
const MAX_LINE: usize = 16 * 1024;

/// Bind the control socket and spawn its accept loop.
pub async fn serve_control(
    socket_path: &str,
    tx: mpsc::Sender<ControlMsg>,
) -> io::Result<JoinHandle<()>> {
    let listener = bind_command_socket(socket_path, 0o660)?;
    tracing::info!(path = %socket_path, "atlas_control_listening");
    Ok(tokio::spawn(async move {
        loop {
            let stream = match listener.accept().await {
                Ok((s, _)) => s,
                Err(e) => {
                    tracing::warn!(error = %e, "atlas_control_accept_failed");
                    break;
                }
            };
            let tx = tx.clone();
            tokio::spawn(async move {
                if let Err(e) = handle_conn(stream, tx).await {
                    tracing::debug!(error = %e, "atlas_control_conn_error");
                }
            });
        }
    }))
}

/// Parse one request line.
pub fn parse_request(line: &str) -> Result<CtlRequest, CtlReply> {
    if line.len() > MAX_LINE {
        return Err(CtlReply::err(codes::BAD_REQUEST, "request too long"));
    }
    serde_json::from_str(line.trim()).map_err(|e| CtlReply::err(codes::BAD_REQUEST, e.to_string()))
}

async fn handle_conn(stream: UnixStream, tx: mpsc::Sender<ControlMsg>) -> io::Result<()> {
    let (read_half, mut write_half) = stream.into_split();
    let mut reader = BufReader::new(read_half).take(MAX_LINE as u64 + 1);
    let mut line = String::new();
    if reader.read_line(&mut line).await? == 0 {
        return Ok(());
    }
    let reply = match parse_request(&line) {
        Err(reply) => reply,
        Ok(req) => {
            let (reply_tx, reply_rx) = oneshot::channel();
            if tx
                .send(ControlMsg {
                    req,
                    reply: reply_tx,
                })
                .await
                .is_err()
            {
                // The loop is gone (the service is stopping).
                return Ok(());
            }
            match reply_rx.await {
                Ok(r) => r,
                Err(_) => return Ok(()),
            }
        }
    };
    let mut body = serde_json::to_vec(&reply).map_err(io::Error::other)?;
    body.push(b'\n');
    write_half.write_all(&body).await?;
    write_half.flush().await
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_protocol::capture_ctl::MarkArgs;

    async fn exchange(path: &std::path::Path, line: &str) -> CtlReply {
        let mut s = UnixStream::connect(path).await.unwrap();
        s.write_all(line.as_bytes()).await.unwrap();
        let mut out = String::new();
        s.read_to_string(&mut out).await.unwrap();
        serde_json::from_str(out.trim()).unwrap()
    }

    #[tokio::test]
    async fn requests_reach_the_loop_and_replies_come_back() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("ctl.sock");
        let (tx, mut rx) = mpsc::channel(4);
        let server = serve_control(path.to_str().unwrap(), tx).await.unwrap();
        let lp = tokio::spawn(async move {
            let m = rx.recv().await.unwrap();
            assert_eq!(
                m.req,
                CtlRequest::Mark(MarkArgs {
                    room: "Hall".into(),
                    floor: 2
                })
            );
            m.reply
                .send(CtlReply::ok(serde_json::json!({"segment": 4})))
                .unwrap();
        });
        let r = exchange(&path, "{\"cmd\":\"mark\",\"room\":\"Hall\",\"floor\":2}\n").await;
        assert!(r.ok);
        assert_eq!(r.result["segment"], 4);
        lp.await.unwrap();
        let bad = exchange(&path, "{\"cmd\":\"teleport\"}\n").await;
        assert_eq!(bad.error.as_deref(), Some(codes::BAD_REQUEST));
        server.abort();
    }
}
