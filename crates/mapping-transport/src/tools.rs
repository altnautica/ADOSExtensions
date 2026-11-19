//! The MCP tool plumbing the capture and reconstruct processes share: argument
//! parsing, the answer deadline under the host's 5 s invoke window, and the
//! handler `ctx.tools.serve` registers. A refusal is the error string
//! `"<CODE>: <detail>"`.

use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use serde::de::DeserializeOwned;
use serde_json::Value;

use crate::host::to_json;
use crate::reply::{codes, ApiError};

/// The longest a tool call waits before answering; under the host's 5 s.
pub const TOOL_DEADLINE: Duration = Duration::from_secs(4);

/// Tool arguments as `T`; a call with no arguments arrives as an empty map or
/// nil.
pub fn args<T: DeserializeOwned>(v: Value) -> Result<T, String> {
    let v = if v.is_null() {
        Value::Object(Default::default())
    } else {
        v
    };
    serde_json::from_value(v).map_err(|e| format!("{}: {e}", codes::BAD_REQUEST))
}

/// Run `op` on its own task and answer within `deadline`. The operation
/// still completes after the reply; `still_running` is the answer when it
/// has not finished by then.
pub async fn answer_within<F>(
    deadline: Duration,
    op: F,
    still_running: impl FnOnce() -> Result<Value, String>,
) -> Result<Value, String>
where
    F: Future<Output = Result<Value, ApiError>> + Send + 'static,
{
    match tokio::time::timeout(deadline, tokio::spawn(op)).await {
        Ok(Ok(out)) => out.map_err(|e| e.tool_message()),
        Ok(Err(e)) => Err(format!("{}: {e}", codes::HOST)),
        Err(_) => still_running(),
    }
}

/// The plain timeout answer for a tool with nothing to report early.
pub fn timed_out() -> Result<Value, String> {
    Err(format!(
        "{}: no answer within {} ms",
        codes::TIMEOUT,
        TOOL_DEADLINE.as_millis()
    ))
}

/// One tool call's future, as the SDK's handler type spells it.
type ToolFuture = Pin<Box<dyn Future<Output = Result<rmpv::Value, String>> + Send>>;

/// The handler `ctx.tools.serve` registers: `invoke(tool, json arguments)`.
pub fn handler<F, Fut>(invoke: F) -> ados_sdk::ToolHandler
where
    F: Fn(String, Value) -> Fut + Send + Sync + 'static,
    Fut: Future<Output = Result<Value, String>> + Send + 'static,
{
    let invoke = Arc::new(invoke);
    Arc::new(move |tool: String, arguments: rmpv::Value| -> ToolFuture {
        let invoke = invoke.clone();
        Box::pin(async move {
            let result = invoke(tool, to_json(arguments)).await?;
            rmpv::ext::to_value(&result).map_err(|e| format!("E_ENCODE: {e}"))
        })
    })
}
