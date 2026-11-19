//! Reconstruct's MCP tools on a drone, answered through `ctx.tools.serve`,
//! each against the paired node's job API ([`NodeJobs`]). A refusal is the
//! error string `"<CODE>: <detail>"`. Every call answers within
//! [`TOOL_DEADLINE`]; `reconstruct` submits with an id chosen here, so a
//! submit still running then answers `{ok, accepted: true, node: null,
//! job_id}` (poll it with `job_status`).
//!
//! | tool | arguments | result |
//! |---|---|---|
//! | `reconstruct` | `{session_id, preset}` | `{ok, node, job_id}` |
//! | `job_status` | `{job_id}` | the node's job view plus `node` |
//! | `tour_bundle` | `{job_id}` | `{job_id, node, uri, meta}` |

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ados_mapping_transport::jobs::NodeJobs;
use ados_mapping_transport::tools::{answer_within, args, timed_out, TOOL_DEADLINE};
use serde::Deserialize;
use serde_json::{json, Value};

/// Every tool this handler answers, in manifest order.
pub const TOOLS: [&str; 3] = ["reconstruct", "job_status", "tour_bundle"];

#[derive(Debug, Deserialize)]
struct ReconstructArgs {
    session_id: String,
    preset: String,
}

#[derive(Debug, Deserialize)]
struct JobArgs {
    job_id: String,
}

/// A job id for a tool-submitted reconstruction.
fn new_job_id() -> String {
    let ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos());
    format!("rc-{ns}")
}

/// Run `tool` with JSON `arguments`, answering within [`TOOL_DEADLINE`].
pub async fn invoke(jobs: &NodeJobs, tool: &str, arguments: Value) -> Result<Value, String> {
    invoke_within(jobs, tool, arguments, TOOL_DEADLINE).await
}

async fn invoke_within(
    jobs: &NodeJobs,
    tool: &str,
    arguments: Value,
    deadline: Duration,
) -> Result<Value, String> {
    let jobs = jobs.clone();
    match tool {
        "reconstruct" => {
            let a: ReconstructArgs = args(arguments)?;
            let job_id = new_job_id();
            let id = job_id.clone();
            answer_within(
                deadline,
                async move { jobs.reconstruct(&a.session_id, &a.preset, Some(id)).await },
                move || Ok(json!({ "ok": true, "accepted": true, "node": null, "job_id": job_id })),
            )
            .await
        }
        "job_status" => {
            let id = args::<JobArgs>(arguments)?.job_id;
            answer_within(
                deadline,
                async move { jobs.job_status(&id).await },
                timed_out,
            )
            .await
        }
        "tour_bundle" => {
            let id = args::<JobArgs>(arguments)?.job_id;
            answer_within(
                deadline,
                async move { jobs.tour_bundle(&id).await },
                timed_out,
            )
            .await
        }
        other => Err(format!("E_UNKNOWN_TOOL: {other}")),
    }
}

/// The handler `ctx.tools.serve` registers.
pub fn handler(jobs: NodeJobs) -> ados_sdk::ToolHandler {
    ados_mapping_transport::tools::handler(move |tool, arguments| {
        let jobs = jobs.clone();
        async move { invoke(&jobs, &tool, arguments).await }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_mapping_transport::credentials::CredentialStore;
    use ados_mapping_transport::testing::FakeHost;

    #[tokio::test]
    async fn a_slow_submit_answers_with_the_id_it_will_have_and_bad_calls_say_why() {
        // A node that takes connections and never answers.
        let node = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = node.local_addr().unwrap();
        tokio::spawn(async move {
            let mut held = Vec::new();
            while let Ok((s, _)) = node.accept().await {
                held.push(s);
            }
        });
        let dir = tempfile::tempdir().unwrap();
        let host = FakeHost::new();
        host.set_config(crate::PIN_KEY, json!(addr.to_string()));
        let jobs = NodeJobs {
            host,
            credentials: CredentialStore::new(Some(dir.path()), crate::PIN_KEY),
        };
        let short = Duration::from_millis(200);
        let rc = invoke_within(
            &jobs,
            "reconstruct",
            json!({"session_id": "s-20261019T101500Z-0001", "preset": "tour"}),
            short,
        )
        .await
        .unwrap();
        assert_eq!(rc["accepted"], true);
        assert!(rc["job_id"].as_str().unwrap().starts_with("rc-"));
        let err = invoke_within(&jobs, "job_status", json!({"job_id": "j-1"}), short)
            .await
            .unwrap_err();
        assert!(err.starts_with("E_TIMEOUT"), "{err}");
        assert!(invoke(&jobs, "reconstruct", json!({"session_id": "s"}))
            .await
            .unwrap_err()
            .starts_with("E_BAD_REQUEST"));
        assert_eq!(
            invoke(&jobs, "capture_start", Value::Null)
                .await
                .unwrap_err(),
            "E_UNKNOWN_TOOL: capture_start"
        );
    }
}
