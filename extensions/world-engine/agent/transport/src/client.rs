//! The compute-offload client: the agent-side caller of a compute node's job
//! API.
//!
//! A drone's offload submits a perception-offload session through this client
//! and reads its health. Local-first: `base_url` is the node's LAN address, and
//! the drone presents the credential that node issued it (installed by the
//! owner) in the node-credential header. It never sends its own pairing key:
//! that key is full authority over the drone and means nothing to the node.

use std::time::Duration;

use reqwest::Client;
use serde::Serialize;

use ados_protocol::node_credential::NODE_CREDENTIAL_HEADER;
use world_engine_protocol::compute::{
    CancelResponse, ComputeHeartbeat, ComputeJobKind, Dataset, JobRecord, Output, SubmitResponse,
};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(15);

/// A failure calling the compute node's job API.
#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    /// The request itself failed (connect / timeout / transport).
    #[error("request: {0}")]
    Request(String),
    /// The node returned a non-success status; the body is the error JSON.
    #[error("http {0}: {1}")]
    Http(u16, String),
    /// The response body did not decode to the expected type.
    #[error("decode: {0}")]
    Decode(String),
}

#[derive(Serialize)]
struct CreateDatasetBody<'a> {
    kind: &'a str,
    meta: serde_json::Value,
}

#[derive(Serialize)]
struct SubmitBody {
    /// An optional caller-chosen id. Reusing it on a retry makes the submit
    /// idempotent: a duplicate yields a `409 Conflict` instead of a second job.
    #[serde(skip_serializing_if = "Option::is_none")]
    job_id: Option<String>,
    kind: ComputeJobKind,
    dataset_id: Option<String>,
    params: serde_json::Value,
}

/// Percent-encode a value going into a URL path segment, so a job id with a
/// `/`, `?`, `#`, or whitespace cannot misroute the request. Server-generated
/// ids are already URL-safe, so this is identity for them; it guards an
/// arbitrary caller-supplied id.
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

/// A client to one compute node's job API.
pub struct ComputeClient {
    http: Client,
    base_url: String,
    credential: Option<String>,
}

impl ComputeClient {
    /// A client targeting `base_url` (e.g. `http://compute.local:8092`).
    /// `credential` is the one this node issued the caller, sent in the
    /// node-credential header; `None` when none is installed (an unpaired node
    /// or an on-box caller is served without one). The client carries connect +
    /// request timeouts so a hung node fails the call rather than parking the
    /// caller.
    pub fn new(base_url: impl Into<String>, credential: Option<String>) -> Self {
        // The workspace's reqwest is unified onto the no-provider rustls path, so
        // building a Client tries to construct a default TLS config that needs a
        // process-default crypto provider; install it first or build() panics
        // ("No provider set"), non-deterministically under concurrent first
        // builds.
        ados_protocol::crypto::ensure_crypto_provider();
        // build() only fails on a TLS-stack init error; `expect` keeps the
        // timeout guarantee (a silent fall back to a timeout-less default would
        // let a hung node park the caller — the exact thing the timeouts prevent).
        let http = Client::builder()
            .connect_timeout(CONNECT_TIMEOUT)
            .timeout(REQUEST_TIMEOUT)
            .build()
            .expect("build compute http client");
        Self {
            http,
            base_url: base_url.into().trim_end_matches('/').to_string(),
            credential,
        }
    }

    fn url(&self, path: &str) -> String {
        format!("{}{path}", self.base_url)
    }

    fn auth(&self, rb: reqwest::RequestBuilder) -> reqwest::RequestBuilder {
        match &self.credential {
            Some(c) => rb.header(NODE_CREDENTIAL_HEADER, c),
            None => rb,
        }
    }

    async fn send_json<T: serde::de::DeserializeOwned>(
        &self,
        rb: reqwest::RequestBuilder,
    ) -> Result<T, ClientError> {
        let resp = self
            .auth(rb)
            .send()
            .await
            .map_err(|e| ClientError::Request(e.to_string()))?;
        let status = resp.status();
        if !status.is_success() {
            let body = resp.text().await.unwrap_or_default();
            return Err(ClientError::Http(status.as_u16(), body));
        }
        resp.json::<T>()
            .await
            .map_err(|e| ClientError::Decode(e.to_string()))
    }

    /// Read the node + cluster heartbeat.
    pub async fn status(&self) -> Result<ComputeHeartbeat, ClientError> {
        self.send_json(self.http.get(self.url("/api/compute/status")))
            .await
    }

    /// Register a dataset. The heavy input rides a
    /// separate bulk/stream lane; this records the dataset the job consumes.
    pub async fn write_dataset(
        &self,
        kind: &str,
        meta: serde_json::Value,
    ) -> Result<Dataset, ClientError> {
        self.send_json(
            self.http
                .post(self.url("/api/compute/datasets"))
                .json(&CreateDatasetBody { kind, meta }),
        )
        .await
    }

    /// Submit a job: a reconstruction over a dataset, or a
    /// perception / SLAM offload whose frame rides `params`. Pass `job_id` to make
    /// the submit idempotent — reusing the same id on a retry (after a timeout
    /// over a lossy link) yields a `409 Conflict` rather than a duplicate job.
    pub async fn submit_job(
        &self,
        kind: ComputeJobKind,
        dataset_id: Option<String>,
        params: serde_json::Value,
        job_id: Option<String>,
    ) -> Result<SubmitResponse, ClientError> {
        self.send_json(
            self.http
                .post(self.url("/api/compute/jobs"))
                .json(&SubmitBody {
                    job_id,
                    kind,
                    dataset_id,
                    params,
                }),
        )
        .await
    }

    /// Read a job's status + progress.
    pub async fn job_status(&self, id: &str) -> Result<JobRecord, ClientError> {
        let seg = encode_segment(id);
        self.send_json(self.http.get(self.url(&format!("/api/compute/jobs/{seg}"))))
            .await
    }

    /// Cancel a job. Returns whether it was a non-terminal job that was cancelled.
    pub async fn cancel_job(&self, id: &str) -> Result<bool, ClientError> {
        let seg = encode_segment(id);
        let resp: CancelResponse = self
            .send_json(
                self.http
                    .post(self.url(&format!("/api/compute/jobs/{seg}/cancel"))),
            )
            .await?;
        Ok(resp.cancelled)
    }

    /// Read a finished job's outputs.
    pub async fn job_outputs(&self, id: &str) -> Result<Vec<Output>, ClientError> {
        let seg = encode_segment(id);
        self.send_json(
            self.http
                .get(self.url(&format!("/api/compute/jobs/{seg}/outputs"))),
        )
        .await
    }
}
