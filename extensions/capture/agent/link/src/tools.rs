//! The extension's MCP tools, answered through `ctx.tools.serve`. Each tool
//! runs the same [`DroneApi`] operation its HTTP route does; a refusal is the
//! error string `"<CODE>: <detail>"`.
//!
//! The host waits 5 s for a reply, so every call answers within
//! [`TOOL_DEADLINE`]. The operation runs on its own task, so it still
//! completes after the reply: `capture_stop` then answers
//! `{ok, accepted: true, state: "closing"}` (the session is finalising). Any
//! other tool answers `E_TIMEOUT`.
//!
//! | tool | arguments | result |
//! |---|---|---|
//! | `capture_start` | `{site?, floor?, room?}` | `{ok, session_id}` |
//! | `capture_stop` | - | the capture state with `ok` |
//! | `capture_mark` | `{room, floor}` | `{ok, segment}` |
//! | `sessions_list` | - | `{sessions:[...]}` |

use std::time::Duration;

use ados_mapping_protocol::capture_ctl::{CtlRequest, MarkArgs, StartArgs};
use ados_mapping_transport::tools::{answer_within, args, timed_out, TOOL_DEADLINE};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::api::DroneApi;

/// Every tool this handler answers, in manifest order.
pub const TOOLS: [&str; 4] = [
    "capture_start",
    "capture_stop",
    "capture_mark",
    "sessions_list",
];

#[derive(Debug, Default, Deserialize)]
struct StartToolArgs {
    #[serde(default)]
    site: Option<String>,
    #[serde(default)]
    floor: Option<i64>,
    #[serde(default)]
    room: Option<String>,
}

/// One parsed tool call.
#[derive(Debug, Clone)]
enum Call {
    Start(StartArgs),
    Stop,
    Mark(MarkArgs),
    List,
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
        other => return Err(format!("E_UNKNOWN_TOOL: {other}")),
    })
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
    let stopping = matches!(call, Call::Stop);
    let api = api.clone();
    let op = async move {
        match call {
            Call::Start(a) => api.capture_start(a).await,
            Call::Stop => api.capture_command(CtlRequest::Stop).await,
            Call::Mark(a) => api.capture_mark(a).await,
            Call::List => api.sessions_list().await,
        }
    };
    answer_within(deadline, op, || {
        if stopping {
            Ok(json!({ "ok": true, "accepted": true, "state": "closing" }))
        } else {
            timed_out()
        }
    })
    .await
}

/// The handler `ctx.tools.serve` registers.
pub fn handler(api: DroneApi) -> ados_sdk::ToolHandler {
    ados_mapping_transport::tools::handler(move |tool, arguments| {
        let api = api.clone();
        async move { invoke(&api, &tool, arguments).await }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::tests::test_api;
    use crate::control::tests::fake_control_socket;
    use ados_mapping_protocol::capture_ctl::CtlReply;
    use ados_mapping_transport::host::to_json;
    use ados_mapping_transport::testing::FakeHost;
    use serde_json::json;

    #[tokio::test]
    async fn tools_run_the_route_operations() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("capture-control.sock");
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
        assert_eq!(
            invoke(&api, "nope", Value::Null).await.unwrap_err(),
            "E_UNKNOWN_TOOL: nope"
        );
    }

    #[tokio::test]
    async fn slow_operations_answer_inside_the_deadline() {
        let dir = tempfile::tempdir().unwrap();
        // A capture service that takes requests and never answers.
        let sock = dir.path().join("capture-control.sock");
        let listener = tokio::net::UnixListener::bind(&sock).unwrap();
        tokio::spawn(async move {
            let mut held = Vec::new();
            while let Ok((s, _)) = listener.accept().await {
                held.push(s);
            }
        });
        let api = test_api(FakeHost::new(), &sock, dir.path());
        let short = Duration::from_millis(200);

        let started = std::time::Instant::now();
        let stop = invoke_within(&api, "capture_stop", Value::Null, short)
            .await
            .unwrap();
        assert_eq!(
            stop,
            json!({"ok": true, "accepted": true, "state": "closing"})
        );
        let err = invoke_within(&api, "capture_mark", json!({"room": "Hall"}), short)
            .await
            .unwrap_err();
        assert!(err.starts_with("E_TIMEOUT"), "{err}");
        assert!(started.elapsed() < Duration::from_secs(3));
    }
}
