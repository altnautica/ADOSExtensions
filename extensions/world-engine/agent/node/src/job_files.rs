//! Owner routes over a staged job's deliverables: any file under its `out/`
//! (Range requests honoured), the tour bundle, and point-to-point / ray
//! measurement on its reconstructed mesh.

use std::path::PathBuf;
use std::sync::{Arc, Mutex as StdMutex};

use axum::extract::{Path, Request, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use tower::ServiceExt;
use tower_http::services::ServeFile;

use crate::auth::{require_job_api, ComputeAuth};
use crate::products::cloud::read_ply_mesh;
use crate::products::measure::{measure, MeasureRequest, MeshIndex};
use crate::resolve_under_root;

#[derive(Clone)]
struct FilesState {
    work_root: Arc<PathBuf>,
    /// The last job's mesh index, so repeated measurements reuse it.
    mesh: Arc<StdMutex<Option<(String, Arc<MeshIndex>)>>>,
}

/// The job-file and measurement routes, behind the job-API gate (owner only).
pub fn job_files_router(auth: Arc<ComputeAuth>, work_root: PathBuf) -> Router {
    Router::new()
        .route("/api/compute/jobs/:id/files/*rel", get(job_file))
        .route("/api/compute/jobs/:id/tour.zip", get(tour_zip))
        .route("/api/compute/measure", post(measure_route))
        .layer(axum::middleware::from_fn_with_state(auth, require_job_api))
        .with_state(FilesState {
            work_root: Arc::new(work_root),
            mesh: Arc::new(StdMutex::new(None)),
        })
}

fn error(status: StatusCode, code: &str, detail: impl Into<String>) -> Response {
    (
        status,
        Json(serde_json::json!({ "error": code, "detail": detail.into() })),
    )
        .into_response()
}

/// A job id is one plain path component.
fn valid_job_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.'))
        && !id.starts_with('.')
}

async fn serve(state: &FilesState, id: &str, rel: &str, req: Request) -> Response {
    if !valid_job_id(id) {
        return error(StatusCode::NOT_FOUND, "E_NOT_FOUND", "no such file");
    }
    let rel = format!("jobs/{id}/out/{rel}");
    let Some(path) = resolve_under_root(&state.work_root, &rel) else {
        return error(StatusCode::NOT_FOUND, "E_NOT_FOUND", "no such file");
    };
    if !path.is_file() {
        return error(StatusCode::NOT_FOUND, "E_NOT_FOUND", "no such file");
    }
    match ServeFile::new(path).oneshot(req).await {
        Ok(r) => r.into_response(),
        Err(e) => error(StatusCode::INTERNAL_SERVER_ERROR, "E_IO", e.to_string()),
    }
}

async fn job_file(
    State(state): State<FilesState>,
    Path((id, rel)): Path<(String, String)>,
    req: Request,
) -> Response {
    serve(&state, &id, &rel, req).await
}

async fn tour_zip(
    State(state): State<FilesState>,
    Path(id): Path<String>,
    req: Request,
) -> Response {
    serve(&state, &id, "tour.zip", req).await
}

async fn measure_route(
    State(state): State<FilesState>,
    Json(req): Json<MeasureRequest>,
) -> Response {
    if !valid_job_id(&req.job_id) {
        return error(StatusCode::NOT_FOUND, "E_NO_MESH", "unknown job");
    }
    let cached = {
        let guard = state.mesh.lock().unwrap_or_else(|p| p.into_inner());
        guard
            .as_ref()
            .filter(|(id, _)| *id == req.job_id)
            .map(|(_, m)| m.clone())
    };
    let index = match cached {
        Some(m) => m,
        None => {
            let path = state
                .work_root
                .join("jobs")
                .join(&req.job_id)
                .join("mesh")
                .join("mesh.ply");
            let loaded = tokio::task::spawn_blocking(move || {
                read_ply_mesh(&path).map(|m| Arc::new(MeshIndex::new(&m)))
            })
            .await;
            match loaded {
                Ok(Ok(m)) => {
                    *state.mesh.lock().unwrap_or_else(|p| p.into_inner()) =
                        Some((req.job_id.clone(), m.clone()));
                    m
                }
                _ => {
                    return error(
                        StatusCode::NOT_FOUND,
                        "E_NO_MESH",
                        "the job has no reconstructed mesh",
                    )
                }
            }
        }
    };
    match measure(&index, &req) {
        Some(r) => Json(r).into_response(),
        None => error(
            StatusCode::UNPROCESSABLE_ENTITY,
            "E_NO_HIT",
            "the ray does not hit the mesh",
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request as HttpRequest;

    #[test]
    fn job_ids_are_single_plain_components() {
        assert!(valid_job_id("job-17_3.a"));
        assert!(!valid_job_id("../x"));
        assert!(!valid_job_id("a/b"));
        assert!(!valid_job_id(".hidden"));
        assert!(!valid_job_id(""));
    }

    #[cfg(not(unix))]
    fn router(work: PathBuf) -> Router {
        job_files_router(Arc::new(ComputeAuth::standalone("t".into())), work)
    }

    #[cfg(unix)]
    fn router(work: PathBuf) -> Router {
        let creds = crate::NodeCredentialStore::open(work.join("creds.json"), "n");
        job_files_router(
            Arc::new(ComputeAuth::new(creds).with_owner_token("t".into())),
            work,
        )
    }

    #[tokio::test]
    async fn files_are_served_with_ranges_and_jailed() {
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("jobs/j1/out");
        std::fs::create_dir_all(&out).unwrap();
        std::fs::write(out.join("floorplan.json"), b"0123456789").unwrap();
        let r = router(dir.path().to_path_buf());
        let req = HttpRequest::builder()
            .uri("/api/compute/jobs/j1/files/floorplan.json")
            .header("authorization", "Bearer t")
            .header("range", "bytes=2-5")
            .body(Body::empty())
            .unwrap();
        let resp = r.clone().oneshot(req).await.unwrap();
        assert_eq!(resp.status(), StatusCode::PARTIAL_CONTENT);
        let body = axum::body::to_bytes(resp.into_body(), 64).await.unwrap();
        assert_eq!(&body[..], b"2345");
        for uri in [
            "/api/compute/jobs/j1/files/../../../creds.json",
            "/api/compute/jobs/j1/files/missing.json",
            "/api/compute/jobs/j1/tour.zip",
        ] {
            let req = HttpRequest::builder()
                .uri(uri)
                .header("authorization", "Bearer t")
                .body(Body::empty())
                .unwrap();
            assert_eq!(
                r.clone().oneshot(req).await.unwrap().status(),
                StatusCode::NOT_FOUND,
                "{uri}"
            );
        }
        let anon = HttpRequest::builder()
            .uri("/api/compute/jobs/j1/files/floorplan.json")
            .body(Body::empty())
            .unwrap();
        assert_eq!(
            r.oneshot(anon).await.unwrap().status(),
            StatusCode::UNAUTHORIZED
        );
    }
}
