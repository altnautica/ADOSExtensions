//! The extension's MCP tools, answered through `ctx.tools.serve`. Each tool
//! runs the same [`DroneApi`] operation its HTTP route does; a refusal is the
//! error string `"<CODE>: <detail>"`.
//!
//! The host waits 5 s for a reply, so every call answers within
//! [`TOOL_DEADLINE`]. The operation runs on its own task, so it still
//! completes after the reply: `capture_stop` then answers
//! `{ok, accepted: true, state: "closing"}` (the session is finalising), and
//! `reconstruct` submits with an id chosen here and answers
//! `{ok, accepted: true, node: null, job_id}` (poll it with `job_status`).
//! Any other tool answers `E_TIMEOUT`.
//!
//! | tool | arguments | result |
//! |---|---|---|
//! | `capture_start` | `{site?, floor?, room?}` | `{ok, session_id}` |
//! | `capture_stop` | - | the capture state with `ok` |
//! | `capture_mark` | `{room, floor}` | `{ok, segment}` |
//! | `sessions_list` | - | `{sessions:[...]}` |
//! | `reconstruct` | `{session_id, preset}` | `{ok, node, job_id}` |
//! | `job_status` | `{job_id}` | the node's job view plus `node` |
//! | `tour_bundle` | `{job_id}` | `{job_id, node, uri, meta}` |

use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde::de::DeserializeOwned;
use serde::Deserialize;
use serde_json::{json, Value};
use world_engine_protocol::capture_ctl::{CtlRequest, MarkArgs, StartArgs};

use crate::api::DroneApi;
use crate::host::to_json;
use crate::reply::{codes, ApiError};

/// Every tool this handler answers, in manifest order.
pub const TOOLS: [&str; 7] = [
    "capture_start",
    "capture_stop",
    "capture_mark",
    "sessions_list",
    "reconstruct",
    "job_status",
    "tour_bundle",
];

/// The longest a tool call waits before answering; under the host's 5 s.
pub const TOOL_DEADLINE: Duration = Duration::from_secs(4);

#[derive(Debug, Default, Deserialize)]
struct StartToolArgs {
    #[serde(default)]
    site: Option<String>,
    #[serde(default)]
    floor: Option<i64>,
    #[serde(default)]
    room: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ReconstructArgs {
    session_id: String,
    preset: String,
}

#[derive(Debug, Deserialize)]
struct JobArgs {
    job_id: String,
}

/// One parsed tool call.
#[derive(Debug, Clone)]
enum Call {
    Start(StartArgs),
    Stop,
    Mark(MarkArgs),
    List,
    Reconstruct {
        session_id: String,
        preset: String,
        job_id: String,
    },
    JobStatus(String),
    Tour(String),
}

fn args<T: DeserializeOwned>(v: Value) -> Result<T, String> {
    // A tool called with no arguments arrives as an empty map or nil.
    let v = if v.is_null() {
        Value::Object(Default::default())
    } else {
        v
    };
    serde_json::from_value(v).map_err(|e| format!("{}: {e}", codes::BAD_REQUEST))
}

/// A job id for a tool-submitted reconstruction.
fn new_job_id() -> String {
    let ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos());
    format!("rc-{ns}")
}

fn parse(tool: &str, arguments: Value) -> Result<Call, String> {
    Ok(match tool {
        "capture_start" => {
            let a: StartToolArgs = args(arguments)?;
            Call::Start(StartArgs {
                site: a.site,
                floor: a.floor,
                room: a.room,
                operator_anchor: None,
            })
        }
        "capture_stop" => Call::Stop,
        "capture_mark" => Call::Mark(args(arguments)?),
        "sessions_list" => Call::List,
        "reconstruct" => {
            let a: ReconstructArgs = args(arguments)?;
            Call::Reconstruct {
                session_id: a.session_id,
                preset: a.preset,
                job_id: new_job_id(),
            }
        }
        "job_status" => Call::JobStatus(args::<JobArgs>(arguments)?.job_id),
        "tour_bundle" => Call::Tour(args::<JobArgs>(arguments)?.job_id),
        other => return Err(format!("E_UNKNOWN_TOOL: {other}")),
    })
}

async fn run(api: &DroneApi, call: Call) -> Result<Value, ApiError> {
    match call {
        Call::Start(a) => api.capture_start(a).await,
        Call::Stop => api.capture_command(CtlRequest::Stop).await,
        Call::Mark(a) => api.capture_mark(a).await,
        Call::List => api.sessions_list().await,
        Call::Reconstruct {
            session_id,
            preset,
            job_id,
        } => api.reconstruct(&session_id, &preset, Some(job_id)).await,
        Call::JobStatus(id) => api.job_status(&id).await,
        Call::Tour(id) => api.tour_bundle(&id).await,
    }
}

/// The answer when `call` is still running at the deadline.
fn still_running(call: &Call) -> Result<Value, String> {
    match call {
        Call::Stop => Ok(json!({ "ok": true, "accepted": true, "state": "closing" })),
        Call::Reconstruct { job_id, .. } => {
            Ok(json!({ "ok": true, "accepted": true, "node": null, "job_id": job_id }))
        }
        _ => Err(format!(
            "{}: no answer within {} ms",
            codes::TIMEOUT,
            TOOL_DEADLINE.as_millis()
        )),
    }
}

/// Run `tool` with JSON `arguments`, answering within [`TOOL_DEADLINE`].
pub async fn invoke(api: &DroneApi, tool: &str, arguments: Value) -> Result<Value, String> {
    invoke_within(api, tool, arguments, TOOL_DEADLINE).await
}

async fn invoke_within(
    api: &DroneApi,
    tool: &str,
    arguments: Value,
    deadline: Duration,
) -> Result<Value, String> {
    let call = parse(tool, arguments)?;
    let (api, task_call) = (api.clone(), call.clone());
    let task = tokio::spawn(async move { run(&api, task_call).await });
    match tokio::time::timeout(deadline, task).await {
        Ok(Ok(out)) => out.map_err(|e| e.tool_message()),
        Ok(Err(e)) => Err(format!("{}: {e}", codes::HOST)),
        Err(_) => still_running(&call),
    }
}

/// One tool call's future, as the SDK's handler type spells it.
type ToolFuture = Pin<Box<dyn Future<Output = Result<rmpv::Value, String>> + Send>>;

/// The handler `ctx.tools.serve` registers.
pub fn handler(api: DroneApi) -> ados_sdk::ToolHandler {
    Arc::new(move |tool: String, arguments: rmpv::Value| -> ToolFuture {
        let api = api.clone();
        Box::pin(async move {
            let result = invoke(&api, &tool, to_json(arguments)).await?;
            rmpv::ext::to_value(&result).map_err(|e| format!("E_ENCODE: {e}"))
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::tests::test_api;
    use crate::control::tests::fake_control_socket;
    use crate::testing::FakeHost;
    use serde_json::json;
    use world_engine_protocol::capture_ctl::CtlReply;

    #[tokio::test]
    async fn tools_run_the_route_operations() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("atlas-control.sock");
        let recorded = fake_control_socket(&sock, |req| match req {
            CtlRequest::Start(_) => CtlReply::ok(json!({"session_id": "s-20261019T101500Z-0001"})),
            CtlRequest::Mark(_) => CtlReply::ok(json!({"segment": 2})),
            _ => CtlReply::err("E_NOT_RECORDING", "idle"),
        });
        let api = test_api(FakeHost::new(), &sock, dir.path());
        let h = handler(api.clone());

        let started = h(
            "capture_start".into(),
            rmpv::ext::to_value(json!({"site": "Plant", "floor": 1})).unwrap(),
        )
        .await
        .unwrap();
        assert_eq!(
            to_json(started),
            json!({"ok": true, "session_id": "s-20261019T101500Z-0001"})
        );
        let sent: Value = serde_json::from_str(&recorded.lock()[0]).unwrap();
        assert_eq!(
            (sent["site"].clone(), sent["floor"].clone()),
            (json!("Plant"), json!(1))
        );

        let marked = invoke(&api, "capture_mark", json!({"room": "Kitchen", "floor": 0}))
            .await
            .unwrap();
        assert_eq!(marked["segment"], 2);
        assert_eq!(
            invoke(&api, "capture_stop", Value::Null).await.unwrap_err(),
            "E_NOT_RECORDING: idle"
        );
        assert_eq!(
            invoke(&api, "sessions_list", json!({})).await.unwrap(),
            json!({"sessions": []})
        );
        assert!(invoke(&api, "capture_mark", json!({}))
            .await
            .unwrap_err()
            .starts_with("E_BAD_REQUEST"));
        assert!(invoke(
            &api,
            "reconstruct",
            json!({"session_id": "s-20261019T101500Z-0001", "preset": "x"})
        )
        .await
        .unwrap_err()
        .starts_with("E_BAD_REQUEST"));
        assert_eq!(
            invoke(&api, "nope", Value::Null).await.unwrap_err(),
            "E_UNKNOWN_TOOL: nope"
        );
    }

    #[tokio::test]
    async fn slow_operations_answer_inside_the_deadline() {
        let dir = tempfile::tempdir().unwrap();
        // A capture service that takes requests and never answers.
        let sock = dir.path().join("atlas-control.sock");
        let listener = tokio::net::UnixListener::bind(&sock).unwrap();
        tokio::spawn(async move {
            let mut held = Vec::new();
            while let Ok((s, _)) = listener.accept().await {
                held.push(s);
            }
        });
        // A node that takes connections and never answers.
        let node = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = node.local_addr().unwrap();
        tokio::spawn(async move {
            let mut held = Vec::new();
            while let Ok((s, _)) = node.accept().await {
                held.push(s);
            }
        });
        let host = FakeHost::new();
        host.set_config(crate::compute_node::PIN_KEY, json!(addr.to_string()));
        let api = test_api(host, &sock, dir.path());
        let short = Duration::from_millis(200);

        let started = std::time::Instant::now();
        let stop = invoke_within(&api, "capture_stop", Value::Null, short)
            .await
            .unwrap();
        assert_eq!(
            stop,
            json!({"ok": true, "accepted": true, "state": "closing"})
        );
        let rc = invoke_within(
            &api,
            "reconstruct",
            json!({"session_id": "s-20261019T101500Z-0001", "preset": "tour"}),
            short,
        )
        .await
        .unwrap();
        assert_eq!(rc["accepted"], true);
        assert!(rc["job_id"].as_str().unwrap().starts_with("rc-"));
        let err = invoke_within(&api, "capture_mark", json!({"room": "Hall"}), short)
            .await
            .unwrap_err();
        assert!(err.starts_with("E_TIMEOUT"), "{err}");
        assert!(started.elapsed() < Duration::from_secs(3));
    }
}
