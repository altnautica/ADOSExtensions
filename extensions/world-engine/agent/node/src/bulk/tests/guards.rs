//! Commit-time document validation, case-insensitive session ids and the
//! storage limits.

use super::*;

/// A `checksums.b3` body listing `files`.
fn listing(files: &[(&str, &[u8])]) -> Vec<u8> {
    files
        .iter()
        .map(|(rel, body)| format!("{}  {rel}\n", blake3::hash(body).to_hex()))
        .collect::<String>()
        .into_bytes()
}

/// Upload a session whose stored manifest and rig are `manifest` and `rig`
/// (the manifest written straight to disk, as a stored session could hold it)
/// with a matching checksum list, then commit.
async fn commit_docs(manifest: &[u8], rig: &[u8]) -> Harness {
    let h = harness();
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    std::fs::write(h.store.session_dir(ID).join(MANIFEST_FILE), manifest).unwrap();
    let mcap = fixture(MCAP);
    for (rel, body) in [("rig.json", rig), (MCAP, &mcap)] {
        assert_eq!(h.put_whole(rel, body).await.status, StatusCode::OK, "{rel}");
    }
    let sums = listing(&[(MANIFEST_FILE, manifest), ("rig.json", rig), (MCAP, &mcap)]);
    assert_eq!(
        h.put_whole("checksums.b3", &sums).await.status,
        StatusCode::OK
    );
    h
}

async fn assert_invalid(manifest: &[u8], rig: &[u8], what: &str) {
    let h = commit_docs(manifest, rig).await;
    let reply = h.commit().await;
    assert_eq!(
        reply.status,
        StatusCode::UNPROCESSABLE_ENTITY,
        "{what}: {}",
        reply.json
    );
    assert_eq!(reply.json["error"], "E_INVALID_SESSION", "{what}");
    assert!(reply.json["detail"].is_string(), "{what}");
    assert!(!h.store.is_committed(ID), "{what}");
    assert_eq!(h.calls.load(Ordering::SeqCst), 0, "{what}");
}

fn doc(rel: &str) -> Value {
    serde_json::from_slice(&fixture(rel)).unwrap()
}

fn bytes(v: &Value) -> Vec<u8> {
    serde_json::to_vec(v).unwrap()
}

#[tokio::test]
async fn fixture_docs_pass_and_commit() {
    validate_session_docs(&fixture(MANIFEST_FILE), &fixture("rig.json")).unwrap();
    let h = commit_docs(&fixture(MANIFEST_FILE), &fixture("rig.json")).await;
    assert_eq!(h.commit().await.status, StatusCode::OK);
    assert!(h.store.is_committed(ID));
}

#[tokio::test]
async fn unsafe_rig_ids_fail_commit() {
    let manifest = fixture(MANIFEST_FILE);
    for bad in ["../x", "a/b", "cam\nzero", "Cam0", ""] {
        let mut rig = doc("rig.json");
        rig["sensors"][0]["id"] = json!(bad);
        assert_invalid(&manifest, &bytes(&rig), &format!("sensor {bad:?}")).await;
    }
    // Ids the schema leaves unconstrained are still held to the id rule.
    for field in ["rig_id", "calib_version"] {
        for bad in ["../x", "a/b", "TINY", ""] {
            let mut rig = doc("rig.json");
            rig[field] = json!(bad);
            assert_invalid(&manifest, &bytes(&rig), &format!("{field} {bad:?}")).await;
        }
    }
}

#[tokio::test]
async fn schema_invalid_manifest_fails_commit() {
    let rig = fixture("rig.json");
    let mut manifest = doc(MANIFEST_FILE);
    manifest["timebase"] = json!("wall_clock");
    assert_invalid(&bytes(&manifest), &rig, "timebase").await;
    let mut manifest = doc(MANIFEST_FILE);
    manifest["device_id"] = json!("../sim");
    assert_invalid(&bytes(&manifest), &rig, "device_id").await;
    let mut manifest = doc(MANIFEST_FILE);
    manifest["segments"][0]["file"] = json!("segments/../x.mcap");
    assert_invalid(&bytes(&manifest), &rig, "segment file").await;
}

#[test]
fn validate_session_docs_reports_the_field() {
    let mut rig = doc("rig.json");
    rig["rig_id"] = json!("a/b");
    let err = validate_session_docs(&fixture(MANIFEST_FILE), &bytes(&rig)).unwrap_err();
    assert!(err.contains("rig_id"), "{err}");
    assert!(validate_session_docs(b"{", &fixture("rig.json")).is_err());
}

#[tokio::test]
async fn case_variant_ids_share_a_lock_and_conflict() {
    let h = harness();
    let variant = ID.to_ascii_uppercase();
    assert!(Arc::ptr_eq(&h.store.lock(ID), &h.store.lock(&variant)));
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    let mut manifest = doc(MANIFEST_FILE);
    manifest["session_id"] = json!(variant);
    let reply = h.post_manifest_to(&variant, bytes(&manifest)).await;
    assert_eq!(reply.status, StatusCode::CONFLICT);
    assert_eq!(reply.json, json!({ "error": "E_SESSION_EXISTS" }));
    // The original id still resumes.
    assert_eq!(h.post_manifest().await.status, StatusCode::OK);
}

fn limited(max_total: Option<u64>, min_free: u64, free: Option<u64>) -> Harness {
    let (store, tmp) = store_with(|s| {
        s.with_limits(max_total, min_free)
            .with_free_space(Arc::new(move |_: &std::path::Path| free))
    });
    harness_on(store, tmp, Ok(()))
}

fn assert_full(reply: &Reply, free: u64) {
    assert_eq!(
        reply.status,
        StatusCode::INSUFFICIENT_STORAGE,
        "{}",
        reply.json
    );
    assert_eq!(
        reply.json,
        json!({ "error": "E_STORAGE_FULL", "free_bytes": free })
    );
}

#[tokio::test]
async fn put_beyond_total_limit_is_refused() {
    let manifest_len = fixture(MANIFEST_FILE).len() as u64;
    let h = limited(Some(manifest_len + 100), 0, None);
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    // Declared by Content-Range: refused before any byte lands.
    assert_full(&h.put("big.bin", 0, &[0u8; 101], 101).await, 100);
    assert_eq!(h.head("big.bin").await.status, StatusCode::NOT_FOUND);
    // Undeclared whole-file body: cut off at the headroom.
    assert_full(&h.put_whole("big.bin", &[0u8; 101]).await, 100);
    assert_eq!(h.head("big.bin").await.status, StatusCode::NOT_FOUND);
    assert_eq!(
        h.put_whole("a.bin", &[0u8; 60]).await.status,
        StatusCode::OK
    );
    assert_full(&h.put_whole("b.bin", &[0u8; 41]).await, 40);
    assert_eq!(
        h.put_whole("b.bin", &[0u8; 40]).await.status,
        StatusCode::OK
    );
}

#[tokio::test]
async fn put_below_free_floor_is_refused() {
    let h = limited(None, 1000, Some(1010));
    assert_eq!(h.post_manifest().await.status, StatusCode::CREATED);
    assert_full(&h.put("x.bin", 0, &[0u8; 20], 20).await, 10);
    assert_full(&h.put_whole("x.bin", &[0u8; 11]).await, 10);
    assert_eq!(
        h.put_whole("x.bin", &[0u8; 10]).await.status,
        StatusCode::OK
    );
}
