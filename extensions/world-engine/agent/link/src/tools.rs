//! The extension's MCP tools, answered through `ctx.tools.serve`. Each tool
//! runs the same [`DroneApi`] operation its HTTP route does; a refusal is the
//! error string `"<CODE>: <detail>"`. The host waits 5 s for a reply, so the
//! long operations return handles (`reconstruct` a job id, `tour_bundle` a
//! URL) rather than waiting for the work.
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

use serde::de::DeserializeOwned;
use serde::Deserialize;
use serde_json::Value;
use world_engine_protocol::capture_ctl::{CtlRequest, MarkArgs, StartArgs};

use crate::api::DroneApi;
use crate::host::to_json;
use crate::reply::ApiError;

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

fn args<T: DeserializeOwned>(v: Value) -> Result<T, String> {
    // A tool called with no arguments arrives as an empty map or nil.
    let v = if v.is_null() {
        Value::Object(Default::default())
    } else {
        v
    };
    serde_json::from_value(v).map_err(|e| format!("E_BAD_REQUEST: {e}"))
}

/// Run `tool` with JSON `arguments`.
pub async fn invoke(api: &DroneApi, tool: &str, arguments: Value) -> Result<Value, String> {
    let out: Result<Value, ApiError> = match tool {
        "capture_start" => {
            let a: StartToolArgs = args(arguments)?;
            api.capture_start(StartArgs {
                site: a.site,
                floor: a.floor,
                room: a.room,
                operator_anchor: None,
            })
            .await
        }
        "capture_stop" => api.capture_command(CtlRequest::Stop).await,
        "capture_mark" => api.capture_mark(args::<MarkArgs>(arguments)?).await,
        "sessions_list" => api.sessions_list().await,
        "reconstruct" => {
            let a: ReconstructArgs = args(arguments)?;
            api.reconstruct(&a.session_id, &a.preset).await
        }
        "job_status" => api.job_status(&args::<JobArgs>(arguments)?.job_id).await,
        "tour_bundle" => api.tour_bundle(&args::<JobArgs>(arguments)?.job_id).await,
        other => return Err(format!("E_UNKNOWN_TOOL: {other}")),
    };
    out.map_err(|e| e.tool_message())
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
}
