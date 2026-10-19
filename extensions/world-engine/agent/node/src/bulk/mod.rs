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

mod commit;
mod disk;
mod paths;
mod routes;
#[cfg(test)]
mod tests;

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

pub use routes::bulk_router;

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

/// The directory tree uploaded sessions land in, plus per-session locks.
pub struct SessionStore {
    root: PathBuf,
    locks: Mutex<HashMap<String, Arc<tokio::sync::Mutex<()>>>>,
}

impl SessionStore {
    /// A store rooted at `root`; the directory is created on first upload.
    pub fn new(root: PathBuf) -> Self {
        Self {
            root,
            locks: Mutex::new(HashMap::new()),
        }
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
        let Ok(entries) = fs::read_dir(&self.root) else {
            return Vec::new();
        };
        let mut ids: Vec<String> = entries
            .filter_map(Result::ok)
            .filter_map(|e| e.file_name().into_string().ok())
            .filter(|id| self.is_committed(id))
            .collect();
        ids.sort();
        ids
    }

    /// The async mutex serializing every mutation of session `id`.
    pub(crate) fn lock(&self, id: &str) -> Arc<tokio::sync::Mutex<()>> {
        let mut locks = self.locks.lock().unwrap_or_else(|e| e.into_inner());
        locks.entry(id.to_string()).or_default().clone()
    }
}
