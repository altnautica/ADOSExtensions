//! The device calibration store: every calibration this device was given, per
//! rig, plus which one is active.
//!
//! Layout under the store root:
//! - `<rig_id>/<calib_version>.json`: one [`CalibRecord`]
//!   (`{calib_version, created_utc, source, report, rig}`), written once.
//! - `<rig_id>/active`: the active `calib_version`, one line.
//!
//! The root is `$ADOS_PLUGIN_DATA_DIR/calib` when the plugin host exported a
//! data directory, else `$HOME/.ados/world-engine/calib` (a development host);
//! see [`default_root`].

use std::io;
use std::path::{Path, PathBuf};

use ados_capture_contract::Rig;
use serde::{Deserialize, Serialize};
use serde_json::Value;

pub use crate::calib_kalibr::{import_kalibr, KalibrError};

/// The env var naming the plugin's data directory.
pub const DATA_DIR_ENV: &str = "ADOS_PLUGIN_DATA_DIR";
/// The store directory under the data directory.
pub const CALIB_DIR: &str = "calib";
/// The active-version pointer inside a rig directory.
pub const ACTIVE_FILE: &str = "active";

/// The store root: `$ADOS_PLUGIN_DATA_DIR/calib`, else
/// `$HOME/.ados/world-engine/calib`; `None` when neither variable is set.
pub fn default_root() -> Option<PathBuf> {
    if let Some(dir) = std::env::var_os(DATA_DIR_ENV).filter(|d| !d.is_empty()) {
        return Some(PathBuf::from(dir).join(CALIB_DIR));
    }
    std::env::var_os("HOME")
        .filter(|h| !h.is_empty())
        .map(|h| PathBuf::from(h).join(".ados/world-engine").join(CALIB_DIR))
}

/// One stored calibration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibRecord {
    pub calib_version: String,
    /// RFC 3339 UTC (`2026-10-19T10:15:00Z`).
    pub created_utc: String,
    /// Where it came from (`kalibr`, `node`, `gcs`, `factory`, ...).
    pub source: String,
    /// The solver report (`reprojection_rms_px`, residuals, tool versions),
    /// stored verbatim; `null` when there is none.
    #[serde(default)]
    pub report: Value,
    pub rig: Rig,
}

/// A record without its rig: one `GET /atlas/calib` version row.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibSummary {
    pub calib_version: String,
    pub created_utc: String,
    pub source: String,
    pub report: Value,
}

impl From<&CalibRecord> for CalibSummary {
    fn from(r: &CalibRecord) -> Self {
        Self {
            calib_version: r.calib_version.clone(),
            created_utc: r.created_utc.clone(),
            source: r.source.clone(),
            report: r.report.clone(),
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum CalibError {
    #[error("calib store io: {0}")]
    Io(#[from] io::Error),
    #[error("calib record: {0}")]
    Json(#[from] serde_json::Error),
    /// A rig id or version that is not one plain path segment.
    #[error("invalid id `{0}`")]
    InvalidId(String),
    /// Versions are immutable; this one is already stored.
    #[error("calibration {rig_id}/{version} already exists")]
    Exists { rig_id: String, version: String },
    #[error("no calibration {rig_id}/{version}")]
    Unknown { rig_id: String, version: String },
    /// The record's `calib_version` disagrees with its rig's.
    #[error("record version `{record}` differs from its rig's `{rig}`")]
    VersionMismatch { record: String, rig: String },
}

/// One plain path segment: `[A-Za-z0-9._-]`, 1..=128 bytes, not hidden.
pub fn safe_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && !id.starts_with('.')
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'))
}

fn check(id: &str) -> Result<(), CalibError> {
    if safe_id(id) {
        Ok(())
    } else {
        Err(CalibError::InvalidId(id.to_string()))
    }
}

/// Write `body` to `path` through a temp file and a rename.
fn write_atomic(path: &Path, body: &[u8]) -> io::Result<()> {
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, body)?;
    std::fs::rename(&tmp, path)
}

/// The store (blocking file I/O; small files).
#[derive(Debug, Clone)]
pub struct CalibStore {
    root: PathBuf,
}

impl CalibStore {
    /// Open (and create) the store at `root`.
    pub fn open(root: impl Into<PathBuf>) -> io::Result<Self> {
        let root = root.into();
        std::fs::create_dir_all(&root)?;
        Ok(Self { root })
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    fn rig_dir(&self, rig_id: &str) -> PathBuf {
        self.root.join(rig_id)
    }

    /// Every rig with a directory in the store, sorted.
    pub fn rigs(&self) -> Vec<String> {
        let Ok(entries) = std::fs::read_dir(&self.root) else {
            return Vec::new();
        };
        let mut rigs: Vec<String> = entries
            .filter_map(Result::ok)
            .filter(|e| e.path().is_dir())
            .filter_map(|e| e.file_name().into_string().ok())
            .filter(|n| safe_id(n))
            .collect();
        rigs.sort();
        rigs
    }

    /// Every stored version of `rig_id`, oldest first. A file that does not
    /// parse is skipped (and logged), never fatal for the listing.
    pub fn list(&self, rig_id: &str) -> Result<Vec<CalibSummary>, CalibError> {
        check(rig_id)?;
        let entries = match std::fs::read_dir(self.rig_dir(rig_id)) {
            Ok(e) => e,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => return Err(e.into()),
        };
        let mut out = Vec::new();
        for entry in entries.filter_map(Result::ok) {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("json") {
                continue;
            }
            match std::fs::read(&path)
                .map_err(CalibError::from)
                .and_then(|b| serde_json::from_slice::<CalibRecord>(&b).map_err(CalibError::from))
            {
                Ok(r) => out.push(CalibSummary::from(&r)),
                Err(e) => {
                    tracing::warn!(path = %path.display(), error = %e, "calib record skipped")
                }
            }
        }
        out.sort_by(|a, b| {
            (a.created_utc.as_str(), a.calib_version.as_str())
                .cmp(&(b.created_utc.as_str(), b.calib_version.as_str()))
        });
        Ok(out)
    }

    /// One stored version, `None` when absent.
    pub fn get(&self, rig_id: &str, version: &str) -> Result<Option<CalibRecord>, CalibError> {
        check(rig_id)?;
        check(version)?;
        match std::fs::read(self.rig_dir(rig_id).join(format!("{version}.json"))) {
            Ok(b) => Ok(Some(serde_json::from_slice(&b)?)),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    /// Store a new version under its rig's id. Versions are immutable.
    pub fn put(&self, record: &CalibRecord) -> Result<(), CalibError> {
        let rig_id = record.rig.rig_id.as_str();
        check(rig_id)?;
        check(&record.calib_version)?;
        if record.rig.calib_version != record.calib_version {
            return Err(CalibError::VersionMismatch {
                record: record.calib_version.clone(),
                rig: record.rig.calib_version.clone(),
            });
        }
        let dir = self.rig_dir(rig_id);
        std::fs::create_dir_all(&dir)?;
        let path = dir.join(format!("{}.json", record.calib_version));
        if path.exists() {
            return Err(CalibError::Exists {
                rig_id: rig_id.to_string(),
                version: record.calib_version.clone(),
            });
        }
        write_atomic(&path, &serde_json::to_vec_pretty(record)?)?;
        Ok(())
    }

    /// The active version of `rig_id`, if one is set.
    pub fn active_version(&self, rig_id: &str) -> Option<String> {
        if !safe_id(rig_id) {
            return None;
        }
        let text = std::fs::read_to_string(self.rig_dir(rig_id).join(ACTIVE_FILE)).ok()?;
        let v = text.trim();
        safe_id(v).then(|| v.to_string())
    }

    /// The active rig of `rig_id`, if one is set and stored.
    pub fn active(&self, rig_id: &str) -> Option<Rig> {
        let version = self.active_version(rig_id)?;
        match self.get(rig_id, &version) {
            Ok(Some(r)) => Some(r.rig),
            Ok(None) => None,
            Err(e) => {
                tracing::warn!(rig_id, %version, error = %e, "active calibration unreadable");
                None
            }
        }
    }

    /// Point `rig_id` at a stored `version`.
    pub fn set_active(&self, rig_id: &str, version: &str) -> Result<(), CalibError> {
        if self.get(rig_id, version)?.is_none() {
            return Err(CalibError::Unknown {
                rig_id: rig_id.to_string(),
                version: version.to_string(),
            });
        }
        write_atomic(
            &self.rig_dir(rig_id).join(ACTIVE_FILE),
            format!("{version}\n").as_bytes(),
        )?;
        Ok(())
    }
}

/// RFC 3339 UTC (`2026-10-19T10:15:00Z`) of `unix_s` seconds since the epoch.
pub fn format_utc(unix_s: i64) -> String {
    let days = unix_s.div_euclid(86_400);
    let secs = unix_s.rem_euclid(86_400);
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        secs / 3600,
        (secs % 3600) / 60,
        secs % 60
    )
}

#[cfg(test)]
#[path = "calib_tests.rs"]
mod tests;
