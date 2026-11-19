//! Upload, resume, repair and commit flows.

use super::*;

#[tokio::test]
async fn full_upload_commits_once() {
    let h = harness();
    h.upload_session().await;

    let status = h.get().await;
    assert_eq!(status.status, StatusCode::OK);
    assert_eq!(status.json["committed"], false);
    for rel in PUT_FILES.iter().chain([&MANIFEST_FILE]) {
        assert_eq!(status.json["files"][rel]["complete"], true, "{rel}");
    }

    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::OK, "{}", reply.json);
    let expected = json!({ "ok": true, "session_id": ID, "files": 4, "bytes": fixture_bytes() });
    assert_eq!(reply.json, expected);
    assert_eq!(h.calls.load(Ordering::SeqCst), 1);
    assert!(h.store.is_committed(ID));
    assert_eq!(h.store.committed(), vec![ID.to_string()]);
    let dir = h.store.session_dir(ID);
    assert_eq!(std::fs::read(dir.join(MCAP)).unwrap(), fixture(MCAP));
    assert!(!dir.join(UPLOAD_DIR).exists());
    ados_capture_contract::verify_checksums(&dir).unwrap();

    // Re-commit is idempotent and does not rerun the hook.
    let again = h.commit().await;
    assert_eq!(again.status, StatusCode::OK);
    assert_eq!(again.json, expected);
    assert_eq!(h.calls.load(Ordering::SeqCst), 1);
    assert_eq!(h.get().await.json["committed"], true);
}

#[tokio::test]
async fn resume_after_partial_upload() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    h.upload("rig.json").await;
    h.upload("checksums.b3").await;

    let mcap = fixture(MCAP);
    let half = mcap.len() / 2;
    let reply = h.put(MCAP, 0, &mcap[..half], mcap.len()).await;
    assert_eq!(reply.status, StatusCode::OK);
    assert_eq!(
        reply.json,
        json!({ "received": half, "total": mcap.len(), "complete": false })
    );

    let probe = h.head(MCAP).await;
    assert_eq!(probe.status, StatusCode::OK);
    assert_eq!(probe.headers["upload-offset"], half.to_string().as_str());
    assert_eq!(
        probe.headers["upload-length"],
        mcap.len().to_string().as_str()
    );
    assert_eq!(
        probe.headers[header::CONTENT_LENGTH],
        half.to_string().as_str()
    );

    // Re-posting the same manifest resumes and reports the partial file.
    let resumed = h.post_manifest().await;
    assert_eq!(resumed.status, StatusCode::OK);
    assert_eq!(resumed.json["files"][MCAP]["received"], half as u64);
    assert_eq!(resumed.json["files"][MCAP]["complete"], false);

    // A pending part blocks the commit.
    let early = h.commit().await;
    assert_eq!(early.status, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(
        early.json,
        json!({ "error": "E_INCOMPLETE", "files": [MCAP] })
    );

    for wrong in [0, half - 1, half + 1] {
        let reply = h
            .put(MCAP, wrong, &mcap[wrong..wrong + 10], mcap.len())
            .await;
        assert_eq!(reply.status, StatusCode::CONFLICT);
        assert_eq!(
            reply.json,
            json!({ "error": "E_OFFSET_MISMATCH", "received": half })
        );
    }
    // A whole-file PUT starts at 0, which no longer matches either.
    assert_eq!(h.put_whole(MCAP, &mcap).await.status, StatusCode::CONFLICT);

    h.upload_from(MCAP, &mcap, half).await;
    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::OK, "{}", reply.json);
    assert_eq!(h.calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn corrupt_file_is_removed_and_reuploaded() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    h.upload("rig.json").await;
    h.upload("checksums.b3").await;
    let mut bad = fixture(MCAP);
    bad[5000] ^= 0x01;
    h.upload_from(MCAP, &bad, 0).await;

    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(
        reply.json,
        json!({ "error": "E_CHECKSUM_MISMATCH", "files": [MCAP] })
    );
    assert_eq!(h.head(MCAP).await.status, StatusCode::NOT_FOUND);
    assert_eq!(h.head("rig.json").await.status, StatusCode::OK);
    assert!(h.get().await.json["files"].get(MCAP).is_none());
    assert!(!h.store.is_committed(ID));
    assert_eq!(h.calls.load(Ordering::SeqCst), 0);

    h.upload(MCAP).await;
    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::OK, "{}", reply.json);
    assert_eq!(h.calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn missing_and_unlisted_files_refuse_commit() {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    h.upload("rig.json").await;
    h.upload(MCAP).await;

    let reply = h.commit().await;
    assert_eq!(
        reply.json,
        json!({ "error": "E_INCOMPLETE", "files": ["checksums.b3"] })
    );

    h.upload("checksums.b3").await;
    std::fs::remove_file(h.store.session_dir(ID).join("rig.json")).unwrap();
    let reply = h.commit().await;
    assert_eq!(reply.status, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(
        reply.json,
        json!({ "error": "E_INCOMPLETE", "files": ["rig.json"] })
    );

    h.upload("rig.json").await;
    assert_eq!(
        h.put_whole("extra/notes.txt", b"x").await.status,
        StatusCode::OK
    );
    let reply = h.commit().await;
    assert_eq!(
        reply.json,
        json!({ "error": "E_UNLISTED", "files": ["extra/notes.txt"] })
    );
    assert_eq!(h.calls.load(Ordering::SeqCst), 0);
}

#[tokio::test]
async fn escaping_checksum_paths_refuse_commit() {
    let hash = "0".repeat(64);
    for rel in [
        "../rig.json",
        "C:/rig.json",
        "segments\\..\\rig.json",
        "/rig.json",
    ] {
        let h = harness();
        assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
        h.upload("rig.json").await;
        let dir = h.store.session_dir(ID);
        std::fs::write(dir.join("checksums.b3"), format!("{hash}  {rel}\n")).unwrap();
        let reply = h.commit().await;
        assert_eq!(reply.status, StatusCode::UNPROCESSABLE_ENTITY, "{rel}");
        assert_eq!(reply.json["error"], "E_BAD_CHECKSUMS", "{rel}");
        assert_eq!(h.calls.load(Ordering::SeqCst), 0);
    }
}
