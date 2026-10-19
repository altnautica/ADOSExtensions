//! The resumable bulk upload client: one capture session directory to a
//! compute node's session-upload lane (`/api/compute/sessions/{id}`).
//!
//! 1. `POST /api/compute/sessions/{id}` with the session's `manifest.json`.
//!    `409 E_ALREADY_COMMITTED` skips straight to the commit.
//! 2. Every file `checksums.b3` lists (then `checksums.b3` itself): `HEAD
//!    .../files/{rel}` for the length the node holds (`Upload-Offset`, 404 =
//!    nothing), then `PUT .../files/{rel}` chunks of at most
//!    [`MAX_CHUNK_BYTES`] with `Content-Range: bytes <a>-<b>/<total>`.
//! 3. `POST /api/compute/sessions/{id}/commit`: the node verifies every file
//!    against `checksums.b3`. When it reports a checksum mismatch (it removed
//!    the bad files) or a missing file, the files are sent once more.
//!
//! A transport failure, a 408/429 or a 5xx is retried with exponential
//! backoff, and the file is probed with `HEAD` again before the next chunk, so
//! a link that drops mid-chunk resumes from what the node actually stored.
//! Consecutive failures without progress are bounded by
//! [`BulkConfig::max_retries`].
//!
//! The client speaks plain HTTP to the node's LAN address only; the caller is
//! responsible for choosing a LAN node (the radio and cloud lanes never carry
//! bulk session data).

use std::future::Future;
use std::io::SeekFrom;
use std::path::Path;

use reqwest::header::{HeaderMap, CONTENT_RANGE};
use reqwest::{Client, RequestBuilder, StatusCode};
use serde_json::Value;
use tokio::io::{AsyncReadExt, AsyncSeekExt};
use world_engine_protocol::node_credential::NODE_CREDENTIAL_HEADER;

use crate::bulk_plan::{BulkConfig, BulkError, Committed, PlanFile, SessionPlan, MAX_CHUNK_BYTES};
use crate::client::{node_http, PinnedHost};

/// The resume-offset header a `HEAD` answers with.
const UPLOAD_OFFSET: &str = "upload-offset";
/// Upload passes before a failing commit is final.
const COMMIT_PASSES: u32 = 2;

/// The outcome of one request that did not succeed outright.
enum Step {
    /// Worth retrying after a backoff (transport failure, 408, 429, 5xx).
    Transient(String),
    Fatal(BulkError),
}

/// Consecutive-failure budget with exponential backoff.
struct Retry<'a> {
    cfg: &'a BulkConfig,
    failures: u32,
}

impl<'a> Retry<'a> {
    fn new(cfg: &'a BulkConfig) -> Self {
        Self { cfg, failures: 0 }
    }

    /// Count a failure and back off; `Err` once the budget is spent.
    async fn failed(&mut self, why: String) -> Result<(), BulkError> {
        self.failures += 1;
        if self.failures > self.cfg.max_retries {
            return Err(BulkError::Unreachable(why));
        }
        let exp = self.failures.saturating_sub(1).min(16);
        let delay = self
            .cfg
            .backoff_base
            .saturating_mul(1u32 << exp)
            .min(self.cfg.backoff_max);
        tracing::debug!(failures = self.failures, ?delay, %why, "bulk upload retry");
        tokio::time::sleep(delay).await;
        Ok(())
    }

    fn progressed(&mut self) {
        self.failures = 0;
    }
}

/// A complete response: status, headers and body.
struct Reply {
    status: StatusCode,
    headers: HeaderMap,
    body: Value,
}

impl Reply {
    fn code(&self) -> Option<&str> {
        self.body.get("error").and_then(Value::as_str)
    }

    /// Transient for 408/429/5xx, else the node's refusal.
    fn failure(self) -> Step {
        let s = self.status;
        if s.is_server_error()
            || s == StatusCode::REQUEST_TIMEOUT
            || s == StatusCode::TOO_MANY_REQUESTS
        {
            return Step::Transient(format!("http {s}"));
        }
        Step::Fatal(BulkError::Refused {
            status: s.as_u16(),
            code: self.code().unwrap_or("E_HTTP").to_string(),
            body: self.body.to_string(),
        })
    }
}

/// Percent-encode one path segment (session ids and file names are already
/// plain; this guards anything else).
fn encode_segment(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => {
                out.push(b as char)
            }
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

/// Read `len` bytes of `path` at `offset`.
async fn read_at(path: &Path, offset: u64, len: u64) -> std::io::Result<Vec<u8>> {
    let mut file = tokio::fs::File::open(path).await?;
    file.seek(SeekFrom::Start(offset)).await?;
    let mut buf = vec![0u8; len as usize];
    file.read_exact(&mut buf).await?;
    Ok(buf)
}

/// A client of one node's session-upload lane.
pub struct BulkClient {
    http: Client,
    base_url: String,
    credential: Option<String>,
    cfg: BulkConfig,
}

impl BulkClient {
    /// `base_url` is the node's LAN job-API base (`http://<lan-addr>:8092`);
    /// `credential` the one it issued for the `sessions.upload` lane; `pin`
    /// the vetted address every request connects to. Redirects are not
    /// followed.
    pub fn new(
        base_url: impl Into<String>,
        credential: Option<String>,
        cfg: BulkConfig,
        pin: Option<PinnedHost>,
    ) -> Self {
        Self {
            http: node_http(cfg.connect_timeout, cfg.request_timeout, pin.as_ref()),
            base_url: base_url.into().trim_end_matches('/').to_string(),
            credential,
            cfg,
        }
    }

    fn session_url(&self, id: &str, tail: &str) -> String {
        format!(
            "{}/api/compute/sessions/{}{tail}",
            self.base_url,
            encode_segment(id)
        )
    }

    fn file_url(&self, id: &str, rel: &str) -> String {
        let rel: Vec<String> = rel.split('/').map(encode_segment).collect();
        self.session_url(id, &format!("/files/{}", rel.join("/")))
    }

    /// Send `rb` and read the whole reply; `Err` is a transport failure.
    async fn exchange(&self, rb: RequestBuilder) -> Result<Reply, Step> {
        let rb = match &self.credential {
            Some(c) => rb.header(NODE_CREDENTIAL_HEADER, c),
            None => rb,
        };
        let resp = rb
            .send()
            .await
            .map_err(|e| Step::Transient(e.to_string()))?;
        let status = resp.status();
        let headers = resp.headers().clone();
        let bytes = resp
            .bytes()
            .await
            .map_err(|e| Step::Transient(e.to_string()))?;
        let body = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
        Ok(Reply {
            status,
            headers,
            body,
        })
    }

    /// Run `op` until it succeeds, retrying transient failures.
    async fn with_retry<T, F, Fut>(&self, mut op: F) -> Result<T, BulkError>
    where
        F: FnMut() -> Fut,
        Fut: Future<Output = Result<T, Step>>,
    {
        let mut retry = Retry::new(&self.cfg);
        loop {
            match op().await {
                Ok(v) => return Ok(v),
                Err(Step::Transient(why)) => retry.failed(why).await?,
                Err(Step::Fatal(e)) => return Err(e),
            }
        }
    }

    /// Post the manifest. `Ok(true)` when the node already committed the
    /// session.
    pub async fn post_manifest(&self, id: &str, manifest: &[u8]) -> Result<bool, BulkError> {
        self.with_retry(move || async move {
            let rb = self
                .http
                .post(self.session_url(id, ""))
                .header(reqwest::header::CONTENT_TYPE, "application/json")
                .body(manifest.to_vec());
            let reply = self.exchange(rb).await?;
            if reply.status.is_success() {
                Ok(false)
            } else if reply.status == StatusCode::CONFLICT
                && reply.code() == Some("E_ALREADY_COMMITTED")
            {
                Ok(true)
            } else {
                Err(reply.failure())
            }
        })
        .await
    }

    /// `HEAD` a file: the bytes the node holds, `None` when it holds none.
    async fn probe(&self, id: &str, rel: &str) -> Result<Option<u64>, Step> {
        let reply = self
            .exchange(self.http.head(self.file_url(id, rel)))
            .await?;
        if reply.status == StatusCode::NOT_FOUND {
            return Ok(None);
        }
        if !reply.status.is_success() {
            return Err(reply.failure());
        }
        reply
            .headers
            .get(UPLOAD_OFFSET)
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.trim().parse().ok())
            .map(Some)
            .ok_or_else(|| Step::Transient("HEAD without Upload-Offset".to_string()))
    }

    /// The bytes of `rel` the node holds (`0` when none), with retries.
    pub async fn received(&self, id: &str, rel: &str) -> Result<u64, BulkError> {
        self.with_retry(move || async move { Ok(self.probe(id, rel).await?.unwrap_or(0)) })
            .await
    }

    /// `PUT` one chunk at `offset` of a `total`-byte file; the node's received
    /// length after it. An empty file is one `PUT` with no range.
    async fn put_chunk(
        &self,
        id: &str,
        rel: &str,
        offset: u64,
        data: Vec<u8>,
        total: u64,
    ) -> Result<u64, Step> {
        let mut rb = self.http.put(self.file_url(id, rel));
        if !data.is_empty() {
            let end = offset + data.len() as u64 - 1;
            rb = rb.header(CONTENT_RANGE, format!("bytes {offset}-{end}/{total}"));
        }
        let reply = self.exchange(rb.body(data)).await?;
        if reply.status.is_success() {
            return reply
                .body
                .get("received")
                .and_then(Value::as_u64)
                .ok_or_else(|| Step::Transient("PUT reply without `received`".to_string()));
        }
        if reply.status == StatusCode::CONFLICT && reply.code() == Some("E_OFFSET_MISMATCH") {
            if let Some(r) = reply.body.get("received").and_then(Value::as_u64) {
                return Ok(r);
            }
        }
        Err(reply.failure())
    }

    /// Bring `file` on the node up to its full length. `report` gets the
    /// file's confirmed byte count.
    async fn upload_file(
        &self,
        id: &str,
        dir: &Path,
        file: &PlanFile,
        report: &(dyn Fn(u64) + Send + Sync),
    ) -> Result<(), BulkError> {
        let path = dir.join(&file.rel);
        let chunk = self.cfg.chunk_bytes.clamp(1, MAX_CHUNK_BYTES);
        let mut retry = Retry::new(&self.cfg);
        'probe: loop {
            let held = match self.probe(id, &file.rel).await {
                Ok(h) => h,
                Err(Step::Transient(why)) => {
                    retry.failed(why).await?;
                    continue;
                }
                Err(Step::Fatal(e)) => return Err(e),
            };
            let mut offset = held.unwrap_or(0);
            if offset > file.len {
                return Err(BulkError::Invalid(format!(
                    "the node holds {offset} bytes of {}, the session {}",
                    file.rel, file.len
                )));
            }
            report(offset);
            if held.is_some() && offset == file.len {
                return Ok(());
            }
            loop {
                let n = chunk.min(file.len - offset);
                let data = read_at(&path, offset, n).await?;
                match self.put_chunk(id, &file.rel, offset, data, file.len).await {
                    Ok(received) if received > file.len => {
                        return Err(BulkError::Invalid(format!(
                            "the node reports {received} bytes of {}",
                            file.rel
                        )))
                    }
                    Ok(received) => {
                        let moved = received > offset;
                        offset = received;
                        report(offset);
                        if offset == file.len {
                            return Ok(());
                        }
                        if moved {
                            retry.progressed();
                        } else {
                            // An offset resync that moved nothing.
                            retry.failed(format!("no progress on {}", file.rel)).await?;
                        }
                    }
                    Err(Step::Transient(why)) => {
                        retry.failed(why).await?;
                        continue 'probe;
                    }
                    Err(Step::Fatal(e)) => return Err(e),
                }
            }
        }
    }

    /// Commit the session.
    pub async fn commit(&self, id: &str) -> Result<Committed, BulkError> {
        self.with_retry(move || async move {
            let reply = self
                .exchange(self.http.post(self.session_url(id, "/commit")))
                .await?;
            if !reply.status.is_success() {
                return Err(reply.failure());
            }
            let n = |k: &str| reply.body.get(k).and_then(Value::as_u64).unwrap_or(0);
            Ok(Committed {
                files: n("files"),
                bytes: n("bytes"),
            })
        })
        .await
    }

    /// Upload and commit a whole session. `progress` gets the confirmed bytes
    /// so far (of [`SessionPlan::total_bytes`]).
    pub async fn upload_session(
        &self,
        id: &str,
        plan: &SessionPlan,
        progress: &(dyn Fn(u64) + Send + Sync),
    ) -> Result<Committed, BulkError> {
        let manifest_len = plan.manifest.len() as u64;
        let mut pass = 0;
        loop {
            pass += 1;
            let already = self.post_manifest(id, &plan.manifest).await?;
            if !already {
                let mut done = manifest_len;
                progress(done);
                for file in &plan.files {
                    let base = done;
                    self.upload_file(id, &plan.dir, file, &|n| progress(base + n))
                        .await?;
                    done += file.len;
                }
            }
            match self.commit(id).await {
                Ok(c) => {
                    progress(plan.total_bytes);
                    return Ok(c);
                }
                Err(BulkError::Refused { ref code, .. })
                    if pass < COMMIT_PASSES
                        && matches!(code.as_str(), "E_CHECKSUM_MISMATCH" | "E_INCOMPLETE") =>
                {
                    tracing::warn!(session = id, %code, "commit refused, sending the files again");
                }
                Err(e) => return Err(e),
            }
        }
    }
}

#[cfg(test)]
#[path = "bulk_tests.rs"]
mod tests;
