//! `POST /api/compute/sessions/:id/commit`: verify every session file against
//! `checksums.b3`, mark the session committed and run the commit hook.

use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use ados_capture_contract::{Manifest, CHECKSUMS_FILE};
use axum::extract::{Path as UrlPath, State};
use axum::http::StatusCode;
use axum::Json;
use serde_json::{json, Value};

use super::disk::{self, Marker};
use super::paths;
use super::routes::{blocking, check_id, require_session, ApiError, ApiResult, Ctx};
use super::{CommittedSession, MANIFEST_FILE, UPLOAD_DIR};

/// Outcome of checking a session directory against its checksum list.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Verdict {
    Verified {
        files: u64,
        bytes: u64,
    },
    /// Listed files not received, still partial, or no `checksums.b3`.
    Incomplete(Vec<String>),
    /// Received files the checksum list does not cover.
    Unlisted(Vec<String>),
    /// Files whose hash differs; they have been deleted for re-upload.
    Mismatch(Vec<String>),
    /// `checksums.b3` is malformed.
    BadList(String),
}

/// Parse `checksums.b3` into `(rel, lowercase hex hash)` pairs.
fn parse_listing(body: &str) -> Result<Vec<(String, String)>, String> {
    body.lines()
        .filter(|l| !l.is_empty())
        .map(|line| {
            let (hash, rel) = line
                .split_once("  ")
                .ok_or_else(|| format!("bad checksum line `{line}`"))?;
            if hash.len() != 64 || !hash.bytes().all(|b| b.is_ascii_hexdigit()) {
                return Err(format!("bad hash for `{rel}`"));
            }
            if !paths::valid_rel(rel) || rel == CHECKSUMS_FILE {
                return Err(format!("checksum path `{rel}` is not a session file"));
            }
            Ok((rel.to_string(), hash.to_ascii_lowercase()))
        })
        .collect()
}

fn hash_file(path: &Path) -> io::Result<String> {
    let mut hasher = blake3::Hasher::new();
    hasher.update_reader(File::open(path)?)?;
    Ok(hasher.finalize().to_hex().to_string())
}

/// Check every file of `dir` against its checksum list, reporting every
/// failure rather than the first. Mismatching files are deleted so the
/// device re-sends them.
pub(crate) fn verify_session(dir: &Path) -> io::Result<Verdict> {
    let pending = disk::pending_parts(dir)?;
    if !pending.is_empty() {
        return Ok(Verdict::Incomplete(pending));
    }
    let body = match fs::read_to_string(dir.join(CHECKSUMS_FILE)) {
        Ok(body) => body,
        Err(e) if e.kind() == io::ErrorKind::NotFound => {
            return Ok(Verdict::Incomplete(vec![CHECKSUMS_FILE.to_string()]))
        }
        Err(e) if e.kind() == io::ErrorKind::InvalidData => {
            return Ok(Verdict::BadList("checksums.b3 is not UTF-8".to_string()))
        }
        Err(e) => return Err(e),
    };
    let listing = match parse_listing(&body) {
        Ok(listing) => listing,
        Err(message) => return Ok(Verdict::BadList(message)),
    };
    let listed: BTreeSet<&str> = listing.iter().map(|(rel, _)| rel.as_str()).collect();
    let received = disk::complete_files(dir)?;
    let have: BTreeSet<&str> = received.iter().map(|(rel, _)| rel.as_str()).collect();

    let missing: Vec<String> = listed
        .iter()
        .filter(|rel| !have.contains(*rel))
        .map(|rel| rel.to_string())
        .collect();
    if !missing.is_empty() {
        return Ok(Verdict::Incomplete(missing));
    }
    let unlisted: Vec<String> = have
        .iter()
        .filter(|rel| **rel != CHECKSUMS_FILE && !listed.contains(*rel))
        .map(|rel| rel.to_string())
        .collect();
    if !unlisted.is_empty() {
        return Ok(Verdict::Unlisted(unlisted));
    }

    let mut mismatched = BTreeSet::new();
    for (rel, hash) in &listing {
        if hash_file(&dir.join(rel))? != *hash {
            mismatched.insert(rel.clone());
        }
    }
    if !mismatched.is_empty() {
        for rel in &mismatched {
            fs::remove_file(dir.join(rel))?;
        }
        return Ok(Verdict::Mismatch(mismatched.into_iter().collect()));
    }
    Ok(Verdict::Verified {
        files: received.len() as u64,
        bytes: received.iter().map(|(_, len)| len).sum(),
    })
}

fn unprocessable(code: &str, files: Vec<String>) -> ApiError {
    ApiError::with(
        StatusCode::UNPROCESSABLE_ENTITY,
        json!({ "error": code, "files": files }),
    )
}

fn success(id: &str, files: u64, bytes: u64) -> Json<Value> {
    Json(json!({ "ok": true, "session_id": id, "files": files, "bytes": bytes }))
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64)
}

pub(crate) async fn commit(
    State(ctx): State<Ctx>,
    UrlPath(id): UrlPath<String>,
) -> ApiResult<Json<Value>> {
    check_id(&id)?;
    let lock = ctx.store.lock(&id);
    let _guard = lock.lock().await;
    let dir = require_session(&ctx.store, &id)?;
    if let Some(marker) = disk::read_marker(&dir)? {
        return Ok(success(&id, marker.files, marker.bytes));
    }

    let verdict = {
        let dir = dir.clone();
        blocking(move || verify_session(&dir)).await?
    };
    let (files, bytes) = match verdict {
        Verdict::Verified { files, bytes } => (files, bytes),
        Verdict::Incomplete(list) => return Err(unprocessable("E_INCOMPLETE", list)),
        Verdict::Unlisted(list) => return Err(unprocessable("E_UNLISTED", list)),
        Verdict::Mismatch(list) => {
            tracing::warn!(session = %id, files = ?list, "checksum mismatch, files removed for re-upload");
            return Err(unprocessable("E_CHECKSUM_MISMATCH", list));
        }
        Verdict::BadList(message) => {
            return Err(ApiError::with(
                StatusCode::UNPROCESSABLE_ENTITY,
                json!({ "error": "E_BAD_CHECKSUMS", "message": message }),
            ))
        }
    };
    let manifest_bytes = tokio::fs::read(dir.join(MANIFEST_FILE)).await?;
    let manifest: Manifest = serde_json::from_slice(&manifest_bytes).map_err(|e| {
        ApiError::with(
            StatusCode::UNPROCESSABLE_ENTITY,
            json!({ "error": "E_BAD_MANIFEST", "message": e.to_string() }),
        )
    })?;

    let marker = Marker {
        committed_ms: now_ms(),
        files,
        bytes,
    };
    disk::write_marker(&dir, &marker).await?;
    let session = CommittedSession {
        session_id: id.clone(),
        dir: dir.clone(),
        files,
        bytes,
        manifest,
    };
    let hook = ctx.hook.clone();
    let hooked = tokio::task::spawn_blocking(move || hook(&session))
        .await
        .unwrap_or_else(|e| Err(format!("commit hook panicked: {e}")));
    if let Err(message) = hooked {
        disk::remove_if_exists(&dir.join(super::COMMITTED_MARKER)).await?;
        tracing::warn!(session = %id, error = %message, "commit hook failed, commit rolled back");
        return Err(ApiError::with(
            StatusCode::INTERNAL_SERVER_ERROR,
            json!({ "error": "E_COMMIT_HOOK", "message": message }),
        ));
    }
    if let Err(e) = tokio::fs::remove_dir_all(dir.join(UPLOAD_DIR)).await {
        if e.kind() != io::ErrorKind::NotFound {
            tracing::warn!(session = %id, error = %e, "could not clear upload scratch");
        }
    }
    tracing::info!(session = %id, files, bytes, "session committed");
    Ok(success(&id, files, bytes))
}
