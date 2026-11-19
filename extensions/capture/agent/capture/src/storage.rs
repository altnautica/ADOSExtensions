//! Where capture sessions live, read from the agent's capture-storage sidecar
//! (`/run/ados/capture-storage/capture-storage.json`, written by the supervisor's janitor).
//!
//! The janitor owns the capture root, its budget and eviction; this module
//! only asks it. A session directory is the janitor's to count once it holds
//! [`RECORDING_MARKER`] (while recording) or `manifest.json` (once finished);
//! [`UPLOADED_MARKER`] makes it evictable.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

use ados_config::capture_storage::{self as cs, CaptureStorageSidecar, Refused};

pub use cs::{E_STORAGE_FULL, MANIFEST_NAME, RECORDING_MARKER, UPLOADED_MARKER};

/// Env override of the sidecar path (tests and development hosts).
pub const SIDECAR_ENV: &str = "ADOS_CAPTURE_STORAGE_SIDECAR";

/// Env override of the capture-active marker directory.
pub const ACTIVE_DIR_ENV: &str = "ADOS_CAPTURE_ACTIVE_DIR";

/// The capture-storage sidecar path.
pub fn sidecar_path() -> PathBuf {
    std::env::var_os(SIDECAR_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| ados_protocol::paths::run_path("capture-storage"))
}

/// The marker the MAVLink router watches to raise the FC sensor rates while a
/// session records: `/run/ados/capture/active`.
pub fn active_marker() -> PathBuf {
    std::env::var_os(ACTIVE_DIR_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| ados_protocol::paths::run_path("capture"))
        .join("active")
}

/// Admission for a new session: the fresh sidecar (its `capture_root` is where
/// the session goes), or why not. Missing or stale counts as full.
pub fn admit(path: &Path) -> Result<CaptureStorageSidecar, Refused> {
    cs::admit_from_sidecar(path, SystemTime::now())
}

/// The capture root the sidecar names, fresh or not (listing finished
/// sessions does not need a live measurement).
pub fn current_root(path: &Path) -> Option<PathBuf> {
    let text = std::fs::read(path).ok()?;
    let s: CaptureStorageSidecar = serde_json::from_slice(&text).ok()?;
    Some(PathBuf::from(s.capture_root))
}

/// Set by the plugin unit (`capture.storage.write`): where the host capture
/// root is mounted inside the sandbox, and which host root that is.
pub const ROOT_MOUNT_ENV: &str = "ADOS_CAPTURE_ROOT_MOUNT";
pub const ROOT_HOST_ENV: &str = "ADOS_CAPTURE_ROOT_HOST";

/// Where this process reads and writes sessions for a host capture root.
///
/// Inside the plugin sandbox the host root is bound at a fixed mount; the
/// mapping holds only while the janitor's root is the one bound at unit
/// start, so a moved root is `None` (the unit must be re-rendered). Outside a
/// sandbox (no env) the host path is used as is.
pub fn local_root_for(host_root: &Path) -> Option<PathBuf> {
    local_root_with(
        host_root,
        std::env::var_os(ROOT_MOUNT_ENV).map(PathBuf::from),
        std::env::var_os(ROOT_HOST_ENV).map(PathBuf::from),
    )
}

fn local_root_with(
    host_root: &Path,
    mount: Option<PathBuf>,
    bound_host: Option<PathBuf>,
) -> Option<PathBuf> {
    match (mount, bound_host) {
        (Some(mount), Some(bound)) => (bound == host_root).then_some(mount),
        _ => Some(host_root.to_path_buf()),
    }
}

/// The local directory holding finished and recording sessions, fresh
/// sidecar or not.
pub fn session_root() -> Option<PathBuf> {
    local_root_for(&current_root(&sidecar_path())?)
}

/// The last sidecar reading, fresh or not, for the HUD storage chip.
pub fn read_sidecar(path: &Path) -> Option<CaptureStorageSidecar> {
    serde_json::from_slice(&std::fs::read(path).ok()?).ok()
}

/// Whether `id` is safe to join under the capture root (one path segment).
pub fn safe_session_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && !id.starts_with('.')
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn session_ids_are_one_safe_segment() {
        assert!(safe_session_id("visor-1-20261019T101500Z-3fa2"));
        assert!(!safe_session_id("../etc"));
        assert!(!safe_session_id(".hidden"));
        assert!(!safe_session_id("a/b"));
        assert!(!safe_session_id(""));
    }

    #[test]
    fn the_sandbox_mount_stands_in_for_the_bound_root_only() {
        let host = Path::new("/mnt/nvme0/ados/capture");
        let mount = Some(PathBuf::from("/run/ados/capture-root"));
        assert_eq!(
            local_root_with(host, mount.clone(), Some(host.to_path_buf())),
            Some(PathBuf::from("/run/ados/capture-root"))
        );
        assert_eq!(
            local_root_with(host, mount, Some(PathBuf::from("/var/ados/capture"))),
            None
        );
        assert_eq!(local_root_with(host, None, None), Some(host.to_path_buf()));
    }

    #[test]
    fn a_missing_sidecar_refuses_admission() {
        let dir = tempfile::tempdir().unwrap();
        let err = admit(&dir.path().join("none.json")).unwrap_err();
        assert_eq!(err.code(), E_STORAGE_FULL);
        assert!(current_root(&dir.path().join("none.json")).is_none());
    }
}
