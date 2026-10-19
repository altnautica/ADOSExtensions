//! What a bulk session upload sends and how it is tuned: the [`SessionPlan`]
//! read from a finished session directory, the [`BulkConfig`] and the
//! [`BulkError`] an upload stops with.

use std::path::{Path, PathBuf};
use std::time::Duration;

/// Largest chunk one `PUT` carries.
pub const MAX_CHUNK_BYTES: u64 = 8 << 20;
/// The session manifest, posted rather than uploaded as a file.
pub const MANIFEST_FILE: &str = "manifest.json";
/// The session checksum list (BLAKE3, `<hex>  <rel>` per line).
pub const CHECKSUMS_FILE: &str = "checksums.b3";

/// Tuning of the upload client.
#[derive(Debug, Clone)]
pub struct BulkConfig {
    /// Bytes per `PUT`, clamped to `1..=MAX_CHUNK_BYTES`.
    pub chunk_bytes: u64,
    /// Consecutive failed requests (without progress) before giving up.
    pub max_retries: u32,
    /// First backoff; doubles per consecutive failure.
    pub backoff_base: Duration,
    /// Backoff ceiling.
    pub backoff_max: Duration,
    pub connect_timeout: Duration,
    /// Per-request timeout; sized for one full chunk on a slow LAN.
    pub request_timeout: Duration,
}

impl Default for BulkConfig {
    fn default() -> Self {
        Self {
            chunk_bytes: MAX_CHUNK_BYTES,
            max_retries: 8,
            backoff_base: Duration::from_secs(1),
            backoff_max: Duration::from_secs(30),
            connect_timeout: Duration::from_secs(5),
            request_timeout: Duration::from_secs(120),
        }
    }
}

/// Why an upload stopped.
#[derive(Debug, thiserror::Error)]
pub enum BulkError {
    /// The node stayed unreachable (or kept failing) through every retry.
    #[error("node unreachable: {0}")]
    Unreachable(String),
    /// The node refused a request; `code` is its `error` field.
    #[error("node refused with {status} {code}")]
    Refused {
        status: u16,
        code: String,
        body: String,
    },
    /// A session file could not be read.
    #[error("session file: {0}")]
    Io(#[from] std::io::Error),
    /// The session directory is not uploadable (no manifest, no checksum list,
    /// a listed path that is not a plain relative path).
    #[error("invalid session: {0}")]
    Invalid(String),
}

impl BulkError {
    /// A stable code for status surfaces.
    pub fn code(&self) -> String {
        match self {
            Self::Unreachable(_) => "E_NODE_UNREACHABLE".to_string(),
            Self::Refused { code, .. } => code.clone(),
            Self::Io(_) => "E_IO".to_string(),
            Self::Invalid(_) => "E_INVALID_SESSION".to_string(),
        }
    }
}

/// One file to send.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanFile {
    /// `/`-separated path inside the session directory.
    pub rel: String,
    pub len: u64,
}

/// What uploading a session sends: the manifest body, then the listed files.
#[derive(Debug, Clone)]
pub struct SessionPlan {
    pub dir: PathBuf,
    pub manifest: Vec<u8>,
    /// The files `checksums.b3` lists (manifest excluded), then `checksums.b3`.
    pub files: Vec<PlanFile>,
    /// Manifest plus every file.
    pub total_bytes: u64,
}

/// A listed path the node will accept: relative, `/`-separated, no empty,
/// `.`/`..` or hidden segment, no backslash.
fn plain_rel(rel: &str) -> bool {
    !rel.is_empty()
        && !rel.contains('\\')
        && rel
            .split('/')
            .all(|s| !s.is_empty() && !s.starts_with('.') && s != "..")
}

impl SessionPlan {
    /// Read the plan from a finished session directory (blocking file I/O).
    pub fn read(dir: &Path) -> Result<Self, BulkError> {
        let read = |name: &str| match std::fs::read(dir.join(name)) {
            Ok(body) => Ok(body),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                Err(BulkError::Invalid(format!("{name} missing")))
            }
            Err(e) => Err(BulkError::Io(e)),
        };
        let manifest = read(MANIFEST_FILE)?;
        let listing = String::from_utf8(read(CHECKSUMS_FILE)?)
            .map_err(|_| BulkError::Invalid(format!("{CHECKSUMS_FILE} is not UTF-8")))?;
        let mut files = Vec::new();
        for line in listing.lines().filter(|l| !l.is_empty()) {
            let (_, rel) = line
                .split_once("  ")
                .ok_or_else(|| BulkError::Invalid(format!("bad checksum line `{line}`")))?;
            if !plain_rel(rel) {
                return Err(BulkError::Invalid(format!("listed path `{rel}`")));
            }
            if rel == MANIFEST_FILE || rel == CHECKSUMS_FILE {
                continue;
            }
            let len = std::fs::metadata(dir.join(rel))?.len();
            files.push(PlanFile {
                rel: rel.to_string(),
                len,
            });
        }
        files.push(PlanFile {
            rel: CHECKSUMS_FILE.to_string(),
            len: listing.len() as u64,
        });
        let total_bytes = manifest.len() as u64 + files.iter().map(|f| f.len).sum::<u64>();
        Ok(Self {
            dir: dir.to_path_buf(),
            manifest,
            files,
            total_bytes,
        })
    }
}

/// The node's commit acknowledgement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Committed {
    pub files: u64,
    pub bytes: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plan_lists_every_checksummed_file_then_the_list() {
        let dir = tempfile::tempdir().unwrap();
        let d = dir.path();
        std::fs::create_dir_all(d.join("segments")).unwrap();
        std::fs::write(d.join("manifest.json"), b"{}").unwrap();
        std::fs::write(d.join("rig.json"), b"rig").unwrap();
        std::fs::write(d.join("segments/000000.mcap"), vec![7u8; 10]).unwrap();
        let h = "0".repeat(64);
        let listing = format!("{h}  manifest.json\n{h}  rig.json\n{h}  segments/000000.mcap\n");
        std::fs::write(d.join(CHECKSUMS_FILE), &listing).unwrap();
        let plan = SessionPlan::read(d).unwrap();
        let rels: Vec<&str> = plan.files.iter().map(|f| f.rel.as_str()).collect();
        assert_eq!(rels, ["rig.json", "segments/000000.mcap", CHECKSUMS_FILE]);
        assert_eq!(plan.total_bytes, 2 + 3 + 10 + listing.len() as u64);
    }

    #[test]
    fn a_session_without_its_list_or_with_an_escaping_path_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let d = dir.path();
        std::fs::write(d.join("manifest.json"), b"{}").unwrap();
        assert!(matches!(SessionPlan::read(d), Err(BulkError::Invalid(_))));
        let h = "0".repeat(64);
        std::fs::write(d.join(CHECKSUMS_FILE), format!("{h}  ../etc/passwd\n")).unwrap();
        assert!(matches!(SessionPlan::read(d), Err(BulkError::Invalid(_))));
        assert!(!plain_rel(".recording"));
        assert!(!plain_rel("a//b"));
        assert!(plain_rel("keyframes/cam0/000001.jpg"));
    }
}
