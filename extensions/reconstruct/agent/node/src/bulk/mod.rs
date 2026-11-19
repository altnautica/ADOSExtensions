//! The resumable bulk upload lane for capture sessions.
//!
//! A device posts the session's `manifest.json`, uploads every session file in
//! ranged chunks that survive a dropped link (`Content-Range`, `HEAD` for the
//! resume offset), then commits. The commit verifies every file against the
//! session's `checksums.b3` and hands the finished session to a hook.
//!
//! On-disk layout under the store root:
//! - `<id>/<rel>`: a fully received file (`manifest.json` is the posted body).
//! - `<id>/.upload/<rel>.part` and `<rel>.total`: a partial file and its
//!   declared total length.
//! - `<id>/.committed`: the commit marker (`{committed_ms, files, bytes}`).
//!
//! A commit also validates `manifest.json` and `rig.json` against the
//! contract schemas and the id rules of [`is_safe_id`].

mod commit;
mod disk;
mod paths;
mod routes;
mod storage;
#[cfg(test)]
mod tests;
mod validate;

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

pub use routes::bulk_router;
pub use storage::{FreeSpaceFn, DEFAULT_MIN_FREE_BYTES};
pub use validate::{is_safe_id, validate_session_docs};

/// File name of the session manifest inside a session directory.
pub(crate) const MANIFEST_FILE: &str = "manifest.json";
/// Directory holding partial uploads inside a session directory.
pub(crate) const UPLOAD_DIR: &str = ".upload";
/// Commit marker file inside a session directory.
pub(crate) const COMMITTED_MARKER: &str = ".committed";

/// Called once per successful commit; an `Err` rolls the commit back.
pub type CommitHook = Arc<dyn Fn(&CommittedSession) -> Result<(), String> + Send + Sync>;

/// A session whose every file passed checksum verification.
#[derive(Debug, Clone)]
pub struct CommittedSession {
    pub session_id: String,
    pub dir: PathBuf,
    /// Number of session files, `manifest.json` and `checksums.b3` included.
    pub files: u64,
    /// Total size of those files.
    pub bytes: u64,
    pub manifest: ados_capture_contract::Manifest,
}

/// The directory tree uploaded sessions land in, per-session locks and the
/// storage limits uploads are held to.
pub struct SessionStore {
    root: PathBuf,
    locks: Mutex<HashMap<String, Arc<tokio::sync::Mutex<()>>>>,
    limits: storage::Limits,
    ledger: storage::Ledger,
}

impl SessionStore {
    /// A store rooted at `root`; the directory is created on first upload.
    /// No total cap, and [`DEFAULT_MIN_FREE_BYTES`] kept free on the volume.
    pub fn new(root: PathBuf) -> Self {
        Self {
            root,
            locks: Mutex::new(HashMap::new()),
            limits: storage::Limits::default(),
            ledger: storage::Ledger::default(),
        }
    }

    /// Cap the bytes held across every session (`None`: no cap) and the free
    /// space an upload must leave on the store's volume.
    pub fn with_limits(mut self, max_total_bytes: Option<u64>, min_free_bytes: u64) -> Self {
        self.limits.max_total_bytes = max_total_bytes;
        self.limits.min_free_bytes = min_free_bytes;
        self
    }

    /// Replace the volume free-space lookup (tests, or a quota-aware probe).
    pub fn with_free_space(mut self, free_space: FreeSpaceFn) -> Self {
        self.limits.free_space = free_space;
        self
    }

    /// The store root directory.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Directory of session `id` (whether or not it exists yet).
    pub fn session_dir(&self, id: &str) -> PathBuf {
        self.root.join(id)
    }

    /// Whether session `id` has been committed.
    pub fn is_committed(&self, id: &str) -> bool {
        paths::valid_session_id(id) && disk::is_committed(&self.session_dir(id))
    }

    /// Ids of every committed session, sorted.
    pub fn committed(&self) -> Vec<String> {
        let mut ids: Vec<String> = self
            .session_names()
            .into_iter()
            .filter(|id| self.is_committed(id))
            .collect();
        ids.sort();
        ids
    }

    /// Names of every entry directly under the root.
    fn session_names(&self) -> Vec<String> {
        let Ok(entries) = fs::read_dir(&self.root) else {
            return Vec::new();
        };
        entries
            .filter_map(Result::ok)
            .filter_map(|e| e.file_name().into_string().ok())
            .collect()
    }

    /// An existing session whose id equals `id` up to ASCII case but not
    /// exactly. Such ids share one folder on a case-insensitive filesystem.
    pub(crate) fn case_variant_of(&self, id: &str) -> Option<String> {
        self.session_names()
            .into_iter()
            .find(|name| name != id && name.eq_ignore_ascii_case(id))
    }

    /// Reserve storage for an upload into `part` (currently `start` bytes
    /// long): `want` bytes, or when undeclared as many as the limits allow up
    /// to `cap`. `Ok(Err(headroom))` when `want` does not fit. The grant is
    /// released when the returned reservation drops.
    pub(crate) fn reserve(
        &self,
        part: PathBuf,
        start: u64,
        want: Option<u64>,
        cap: u64,
    ) -> std::io::Result<Result<storage::Reservation, u64>> {
        self.limits
            .reserve(&self.ledger, &self.root, part, start, want, cap)
    }

    /// The async mutex serializing every mutation of session `id`, keyed by
    /// the lowercased id so case variants share one lock.
    pub(crate) fn lock(&self, id: &str) -> Arc<tokio::sync::Mutex<()>> {
        let mut locks = self.locks.lock().unwrap_or_else(|e| e.into_inner());
        locks.entry(id.to_ascii_lowercase()).or_default().clone()
    }
}
