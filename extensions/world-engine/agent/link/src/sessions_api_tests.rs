use super::*;
use crate::api::drone_router;
use crate::api::tests::{call, test_api};
use crate::testing::FakeHost;
use crate::upload::tests::finished_session;
use world_engine_capture::storage::{RECORDING_MARKER, UPLOADED_MARKER};

#[tokio::test]
async fn sessions_list_with_their_upload_state() {
    let dir = tempfile::tempdir().unwrap();
    let app = drone_router(test_api(
        FakeHost::new(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    // No sessions directory yet: an empty list, not an error.
    let (st, body) = call(app.clone(), "GET", "/atlas/sessions", None).await;
    assert_eq!((st, body), (StatusCode::OK, json!({"sessions": []})));

    let root = dir.path().join("sessions");
    let old = finished_session(&root, "visor-1-20261018T090000Z-0001");
    std::fs::write(old.join(UPLOADED_MARKER), b"{}").unwrap();
    finished_session(&root, "visor-1-20261019T101500Z-3fa2");
    let (st, body) = call(app, "GET", "/atlas/sessions", None).await;
    assert_eq!(st, StatusCode::OK);
    let rows = body["sessions"].as_array().unwrap();
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[0]["session_id"], "visor-1-20261019T101500Z-3fa2");
    assert_eq!(rows[0]["started_at_utc"], "2026-10-19T10:15:00Z");
    assert_eq!(rows[0]["uploaded"], false);
    assert_eq!(rows[0]["upload"]["state"], "idle");
    assert!(rows[0]["frames"].is_null());
    assert!(rows[0]["size_bytes"].as_u64().unwrap() > 5000);
    assert_eq!(rows[0]["upload"]["bytes_total"], rows[0]["size_bytes"]);
    for key in ["duration_s", "rooms", "floors", "timebase", "anchor_source"] {
        assert!(rows[0].get(key).is_some(), "{key}");
    }
    assert_eq!(rows[1]["uploaded"], true);
    assert_eq!(rows[1]["upload"]["state"], "committed");
}

#[tokio::test]
async fn delete_needs_a_commit_or_force_and_never_touches_a_recording() {
    let dir = tempfile::tempdir().unwrap();
    let api = test_api(FakeHost::new(), &dir.path().join("none.sock"), dir.path());
    let app = drone_router(api.clone());
    let root = dir.path().join("sessions");
    let a = finished_session(&root, "a-20261019T101500Z-0001");
    let (st, body) = call(
        app.clone(),
        "DELETE",
        "/atlas/sessions/a-20261019T101500Z-0001",
        None,
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_NOT_UPLOADED");
    assert!(a.exists());
    let (st, body) = call(
        app.clone(),
        "DELETE",
        "/atlas/sessions/a-20261019T101500Z-0001?force=true",
        None,
    )
    .await;
    assert_eq!((st, body), (StatusCode::OK, json!({"ok": true})));
    assert!(!a.exists());

    let b = finished_session(&root, "b-20261019T101500Z-0002");
    std::fs::write(b.join(UPLOADED_MARKER), b"{}").unwrap();
    let (st, _) = call(
        app.clone(),
        "DELETE",
        "/atlas/sessions/b-20261019T101500Z-0002",
        None,
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let c = finished_session(&root, "c-20261019T101500Z-0003");
    std::fs::write(c.join(RECORDING_MARKER), b"").unwrap();
    let (st, body) = call(
        app.clone(),
        "DELETE",
        "/atlas/sessions/c-20261019T101500Z-0003?force=true",
        None,
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_RECORDING");
    let (st, body) = call(
        app.clone(),
        "POST",
        "/atlas/sessions/c-20261019T101500Z-0003/upload",
        None,
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_RECORDING");

    // A queued session is not deleted from under the uploader.
    finished_session(&root, "d-20261019T101500Z-0004");
    let (st, body) = call(
        app.clone(),
        "POST",
        "/atlas/sessions/d-20261019T101500Z-0004/upload",
        None,
    )
    .await;
    assert_eq!((st, body), (StatusCode::OK, json!({"ok": true})));
    let (st, body) = call(
        app.clone(),
        "DELETE",
        "/atlas/sessions/d-20261019T101500Z-0004?force=true",
        None,
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_UPLOADING");

    let (st, body) = call(app, "DELETE", "/atlas/sessions/nope", None).await;
    assert_eq!(st, StatusCode::NOT_FOUND);
    assert_eq!(body["error"], "E_UNKNOWN_SESSION");
}
