//! Requests the lane refuses, and per-session serialization.

use super::*;

#[tokio::test]
async fn bad_paths_are_rejected() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    for rel in [
        "..%2Fx",
        "a/../b",
        "%2e%2e/x",
        "..",
        "a%5Cb",
        "C:/x",
        "C%3A%5Cx",
        ".hidden",
        "segments/.x",
        "a//b",
        "/abs",
        "a/b/c/d/e/f/g",
        "a%20b",
        "nul",
    ] {
        let reply = h.put_whole(rel, b"x").await;
        assert_eq!(reply.status, StatusCode::BAD_REQUEST, "{rel}");
        assert_eq!(reply.json, json!({ "error": "E_BAD_PATH" }), "{rel}");
    }
    assert_eq!(h.head("a/../b").await.status, StatusCode::BAD_REQUEST);
    assert_eq!(
        h.put_whole("a/b/c/d/e/f", b"x").await.status,
        StatusCode::OK
    );
    assert_eq!(h.put_whole("p/q", b"x").await.status, StatusCode::OK);
    let reply = h.put_whole("p/q/r", b"x").await;
    assert_eq!(reply.json, json!({ "error": "E_BAD_PATH" }));
    // A file cannot become a directory, nor a directory a file.
    for rel in ["a/b/c/d/e/f/x", "a/b/c/d/e/f/g", "a/b/c/d/e", "a"] {
        let reply = h.put_whole(rel, b"x").await;
        assert_eq!(reply.status, StatusCode::BAD_REQUEST, "{rel}");
        assert_eq!(reply.json, json!({ "error": "E_BAD_PATH" }), "{rel}");
    }
}

#[tokio::test]
async fn manifest_validation() {
    let h = harness();
    let manifest = fixture(MANIFEST_FILE);
    let reply = h.post_manifest_to("other-session", manifest.clone()).await;
    assert_eq!(reply.status, StatusCode::BAD_REQUEST);
    assert_eq!(reply.json, json!({ "error": "E_SESSION_MISMATCH" }));
    let reply = h.post_manifest_to(ID, b"{}".to_vec()).await;
    assert_eq!(reply.status, StatusCode::BAD_REQUEST);
    assert_eq!(reply.json["error"], "E_BAD_MANIFEST");
    let mut doc: Value = serde_json::from_slice(&manifest).unwrap();
    doc["contract"] = json!("ados.capture/2");
    let reply = h
        .post_manifest_to(ID, serde_json::to_vec(&doc).unwrap())
        .await;
    assert_eq!(reply.json["error"], "E_BAD_MANIFEST");
    let long = "a".repeat(129);
    for id in ["..", ".x", long.as_str()] {
        let reply = h.post_manifest_to(id, manifest.clone()).await;
        assert_eq!(reply.json, json!({ "error": "E_BAD_ID" }), "{id}");
    }

    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    assert_eq!(h.post_manifest().await.status, StatusCode::OK);
    // Different bytes before commit replace the stored manifest verbatim.
    doc["contract"] = json!("ados.capture/1");
    let compact = serde_json::to_vec(&doc).unwrap();
    assert_ne!(compact, manifest);
    assert_eq!(
        h.post_manifest_to(ID, compact.clone()).await.status,
        StatusCode::OK
    );
    let stored = std::fs::read(h.store.session_dir(ID).join(MANIFEST_FILE)).unwrap();
    assert_eq!(stored, compact);
}

#[tokio::test]
async fn unknown_session_is_404() {
    let h = harness();
    let reply = h.put_whole("rig.json", &fixture("rig.json")).await;
    assert_eq!(reply.status, StatusCode::NOT_FOUND);
    assert_eq!(reply.json, json!({ "error": "E_UNKNOWN_SESSION" }));
    assert_eq!(h.head("rig.json").await.status, StatusCode::NOT_FOUND);
    assert_eq!(h.get().await.status, StatusCode::NOT_FOUND);
    assert_eq!(h.commit().await.status, StatusCode::NOT_FOUND);
    assert!(h.store.committed().is_empty());
}

#[tokio::test]
async fn committed_session_refuses_changes() {
    let h = harness();
    h.upload_session().await;
    assert_eq!(h.commit().await.status, StatusCode::OK);
    let reply = h.put_whole("late.bin", b"late").await;
    assert_eq!(reply.status, StatusCode::CONFLICT);
    assert_eq!(reply.json, json!({ "error": "E_ALREADY_COMMITTED" }));
    let reply = h.post_manifest().await;
    assert_eq!(reply.status, StatusCode::CONFLICT);
    assert_eq!(reply.json, json!({ "error": "E_ALREADY_COMMITTED" }));
}

#[tokio::test]
async fn failing_hook_rolls_back_commit() {
    let h = harness_with(Err("queue full".to_string()));
    h.upload_session().await;
    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::INTERNAL_SERVER_ERROR);
    assert_eq!(
        reply.json,
        json!({ "error": "E_COMMIT_HOOK", "message": "queue full" })
    );
    assert!(!h.store.is_committed(ID));
    assert_eq!(h.calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn bad_chunks_leave_no_data() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    let mcap = fixture(MCAP);
    // Declared 100 bytes, sent 50.
    let req = Request::builder()
        .method(Method::PUT)
        .uri(file_uri(MCAP))
        .header(header::CONTENT_RANGE, format!("bytes 0-99/{}", mcap.len()))
        .body(Body::from(mcap[..50].to_vec()))
        .unwrap();
    let reply = h.call(req).await;
    assert_eq!(reply.status, StatusCode::BAD_REQUEST);
    assert_eq!(reply.json, json!({ "error": "E_LENGTH_MISMATCH" }));
    assert_eq!(h.head(MCAP).await.status, StatusCode::NOT_FOUND);

    let req = Request::builder()
        .method(Method::PUT)
        .uri(file_uri(MCAP))
        .header(header::CONTENT_RANGE, "bytes 9-0/10")
        .body(Body::from(vec![0u8; 10]))
        .unwrap();
    assert_eq!(h.call(req).await.json, json!({ "error": "E_BAD_RANGE" }));

    // A later chunk declaring a different total is refused.
    h.put(MCAP, 0, &mcap[..CHUNK], mcap.len()).await;
    let reply = h
        .put(MCAP, CHUNK, &mcap[CHUNK..2 * CHUNK], mcap.len() + 1)
        .await;
    assert_eq!(reply.json, json!({ "error": "E_BAD_RANGE" }));
    assert_eq!(
        h.head(MCAP).await.headers["upload-offset"],
        CHUNK.to_string().as_str()
    );
}

#[tokio::test]
async fn concurrent_puts_to_one_file_serialize() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    let mcap = fixture(MCAP);
    let (a, b) = tokio::join!(
        h.put(MCAP, 0, &mcap[..CHUNK], mcap.len()),
        h.put(MCAP, 0, &mcap[..CHUNK], mcap.len()),
    );
    let mut statuses = [a.status, b.status];
    statuses.sort();
    assert_eq!(statuses, [StatusCode::OK, StatusCode::CONFLICT]);
    assert_eq!(
        h.head(MCAP).await.headers["upload-offset"],
        CHUNK.to_string().as_str()
    );
}
