use std::sync::Arc;

use axum::extract::Path as UrlPath;
use axum::routing::get;
use axum::Json;
use serde_json::{json, Value};

use super::*;
use crate::compute_node::tests::PIN_KEY;
use crate::testing::FakeHost;

/// A stand-in job API serving one finished tour job (`j-1`), one running
/// (`j-2`) and one failed with a structured error (`j-3`).
async fn serve_jobs() -> String {
    let app = axum::Router::new()
        .route(
            "/api/compute/jobs/:id",
            get(|UrlPath(id): UrlPath<String>| async move {
                let (state, stage, error) = match id.as_str() {
                    "j-1" => ("completed", "done", Value::Null),
                    "j-3" => (
                        "failed",
                        "failed",
                        json!({"code": "E_TOOL_MISSING", "message": "colmap is not installed", "tool": "colmap"}),
                    ),
                    _ => ("running", "train", Value::Null),
                };
                Json(json!({"id": id, "kind": "reconstruct", "state": state, "progress": 0.5,
                    "params": {"preset": "tour"}, "created_ms": 1, "updated_ms": 2,
                    "stage": stage, "stage_progress": 0.42, "stages": [], "gpu": null,
                    "eta_s": null, "tools": {"colmap": "4.2.1"}, "error": error}))
            }),
        )
        .route(
            "/api/compute/jobs/:id/outputs",
            get(|UrlPath(id): UrlPath<String>| async move {
                Json(json!([
                    {"id": "o1", "job_id": id, "kind": "splat", "uri": "http://192.168.1.50:8092/artifacts/j-1/scene.spz", "meta": null, "created_ms": 1},
                    {"id": "o2", "job_id": id, "kind": "tour", "uri": "http://192.168.1.50:8092/artifacts/j-1/tour.zip", "meta": {"rooms": 3}, "created_ms": 2, "file": "tour.zip"}
                ]))
            }),
        );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    addr.to_string()
}

fn jobs(host: Arc<FakeHost>, dir: &std::path::Path) -> NodeJobs {
    NodeJobs {
        host,
        credentials: CredentialStore::new(Some(dir), PIN_KEY),
    }
}

#[tokio::test]
async fn a_job_view_and_its_tour_come_from_the_paired_node() {
    let dir = tempfile::tempdir().unwrap();
    let addr = serve_jobs().await;
    let host = FakeHost::new();
    host.set_config(PIN_KEY, json!(addr));
    let jobs = jobs(host, dir.path());

    let status = jobs.job_status("j-1").await.unwrap();
    assert_eq!(status["stage"], "done");
    assert_eq!(status["tools"]["colmap"], "4.2.1");
    assert_eq!(status["node"], json!(addr));
    let tour = jobs.tour_bundle("j-1").await.unwrap();
    assert_eq!(
        tour["uri"],
        "http://192.168.1.50:8092/artifacts/j-1/tour.zip"
    );
    assert_eq!(tour["meta"]["rooms"], 3);

    // Not finished, or failed with a structured error.
    let failed = jobs.job_status("j-3").await.unwrap();
    assert_eq!(failed["error"]["code"], "E_TOOL_MISSING");
    assert_eq!(failed["error"]["tool"], "colmap");
    assert_eq!(jobs.tour_bundle("j-2").await.unwrap_err().code, "E_NO_TOUR");
    assert_eq!(
        jobs.tour_bundle("j-3").await.unwrap_err().code,
        "E_TOOL_MISSING"
    );
}

#[tokio::test]
async fn an_unknown_preset_is_refused_before_the_node_is_asked() {
    let dir = tempfile::tempdir().unwrap();
    let err = jobs(FakeHost::new(), dir.path())
        .reconstruct("v-1", "ultra", None)
        .await
        .unwrap_err();
    assert_eq!(err.code, "E_BAD_REQUEST");
    let err = jobs(FakeHost::new(), dir.path())
        .reconstruct("v-1", "preview", None)
        .await
        .unwrap_err();
    assert_eq!(err.code, "E_NO_NODE");
}
