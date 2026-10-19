//! The device's capture sessions on disk, one `GET /atlas/sessions` row each.
//!
//! A session directory under the capture root counts once it holds
//! `manifest.json` (finished) or `.recording` (recording now); `.uploaded`
//! marks it committed on a node. A row reads the manifest leniently (a field
//! it cannot read is null, the row is never dropped):
//!
//! - `started_at_utc`: the manifest's `utc_at_t0_ns`, else the
//!   `<YYYYMMDD>T<HHMMSS>Z` stamp in the session id, else null.
//! - `duration_s`: last segment end minus first segment start.
//! - `size_bytes`: every file under the session directory.
//! - `frames`: null (the manifest carries no image count).
//! - `rooms` / `floors`: the operator room marks (`operator.rooms[]`) plus
//!   `operator.floor`.
//! - `timebase`, `anchor_source`: verbatim from the manifest.
//!
//! A crashed recorder leaves `.recording` behind. The marker is **stale** when
//! the capture service does not report the session as its open one and the
//! marker is older than [`STALE_AFTER`] (the recorder touches it every 30 s).
//! A stale session is listed with `upload.state` `failed` and error
//! `not_finished`, may be deleted with `force`, and is never uploaded.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use serde::Serialize;
use serde_json::Value;
use world_engine_capture::calib::format_utc;
use world_engine_capture::storage::{
    safe_session_id, MANIFEST_NAME, RECORDING_MARKER, UPLOADED_MARKER,
};

use crate::upload::{UploadState, UploadView};

/// A `.recording` marker untouched this long may belong to a crashed recorder.
pub const STALE_AFTER: Duration = Duration::from_secs(120);

/// The session the capture service has open, as far as the link can tell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LiveSession {
    /// The service is not running, or runs with no session open.
    None,
    /// This session is recording, paused or closing.
    Open(String),
    /// The service did not answer; nothing is known.
    Unknown,
}

/// One session directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SessionDir {
    pub id: String,
    pub path: PathBuf,
    pub recording: bool,
    pub uploaded: bool,
    pub finished: bool,
    /// Time since `.recording` was last touched (when present).
    pub marker_age: Option<Duration>,
}

impl SessionDir {
    /// The session `id` under `root`, when it is a session directory.
    pub fn open(root: &Path, id: &str) -> Option<Self> {
        if !safe_session_id(id) {
            return None;
        }
        let path = root.join(id);
        let marker = std::fs::metadata(path.join(RECORDING_MARKER)).ok();
        let recording = marker.is_some();
        let marker_age = marker
            .and_then(|m| m.modified().ok())
            .map(|t| SystemTime::now().duration_since(t).unwrap_or_default());
        let finished = path.join(MANIFEST_NAME).is_file();
        (path.is_dir() && (recording || finished)).then(|| Self {
            id: id.to_string(),
            uploaded: path.join(UPLOADED_MARKER).exists(),
            path,
            recording,
            finished,
            marker_age,
        })
    }

    /// Whether `.recording` was left by a recorder that is gone.
    pub fn stale(&self, live: &LiveSession) -> bool {
        let old = self.marker_age.is_some_and(|a| a > STALE_AFTER);
        let not_open = match live {
            LiveSession::None => true,
            LiveSession::Open(id) => *id != self.id,
            LiveSession::Unknown => false,
        };
        self.recording && old && not_open
    }
}

/// The upload view of a crashed session.
pub fn not_finished_view(size_bytes: u64) -> UploadView {
    UploadView {
        state: UploadState::Failed,
        error: Some("not_finished".to_string()),
        ..UploadView::idle(size_bytes)
    }
}

/// Every session directory under `root`.
pub fn scan(root: &Path) -> Vec<SessionDir> {
    let Ok(entries) = std::fs::read_dir(root) else {
        return Vec::new();
    };
    entries
        .filter_map(Result::ok)
        .filter_map(|e| e.file_name().into_string().ok())
        .filter_map(|name| SessionDir::open(root, &name))
        .collect()
}

/// One `GET /atlas/sessions` row.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SessionRow {
    pub session_id: String,
    pub started_at_utc: Option<String>,
    pub duration_s: Option<f64>,
    pub size_bytes: u64,
    pub frames: Option<u64>,
    pub rooms: Vec<String>,
    pub floors: Vec<i64>,
    pub timebase: Option<String>,
    pub anchor_source: Option<String>,
    pub uploaded: bool,
    pub upload: UploadView,
}

/// Bytes of every file under `dir`.
pub fn dir_size(dir: &Path) -> u64 {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return 0;
    };
    entries
        .filter_map(Result::ok)
        .map(|e| match e.file_type() {
            Ok(t) if t.is_dir() => dir_size(&e.path()),
            Ok(t) if t.is_file() => e.metadata().map_or(0, |m| m.len()),
            _ => 0,
        })
        .sum()
}

/// `2026-10-19T10:15:00Z` from the `YYYYMMDDTHHMMSSZ` stamp in a session id.
fn utc_from_id(id: &str) -> Option<String> {
    id.split('-').find_map(|part| {
        let b = part.as_bytes();
        let shaped = b.len() == 16
            && b[8] == b'T'
            && b[15] == b'Z'
            && b[..8].iter().chain(&b[9..15]).all(u8::is_ascii_digit);
        shaped.then(|| {
            format!(
                "{}-{}-{}T{}:{}:{}Z",
                &part[0..4],
                &part[4..6],
                &part[6..8],
                &part[9..11],
                &part[11..13],
                &part[13..15]
            )
        })
    })
}

fn manifest_of(dir: &Path) -> Value {
    std::fs::read(dir.join(MANIFEST_NAME))
        .ok()
        .and_then(|b| serde_json::from_slice(&b).ok())
        .unwrap_or(Value::Null)
}

/// Last segment end minus the session start `t0_ns` (both on the session
/// timebase); the earliest segment start when the manifest has no `t0_ns`.
/// A segment may start before `t0_ns` (a sample stamped before the start), so
/// the segment start is only the fallback.
fn duration_s(manifest: &Value) -> Option<f64> {
    let segments = manifest.get("segments")?.as_array()?;
    let end = segments
        .iter()
        .filter_map(|s| s.get("t_end_ns")?.as_u64())
        .max()?;
    let start = match manifest.get("t0_ns").and_then(Value::as_u64) {
        Some(t0) => t0,
        None => segments
            .iter()
            .filter_map(|s| s.get("t_start_ns")?.as_u64())
            .min()?,
    };
    Some(end.saturating_sub(start) as f64 / 1e9)
}

fn rooms_and_floors(manifest: &Value) -> (Vec<String>, Vec<i64>) {
    let operator = manifest.get("operator");
    let mut rooms: Vec<String> = Vec::new();
    let mut floors: Vec<i64> = operator
        .and_then(|o| o.get("floor"))
        .and_then(Value::as_i64)
        .into_iter()
        .collect();
    let marks = operator
        .and_then(|o| o.get("rooms"))
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    for mark in &marks {
        if let Some(name) = mark.get("name").and_then(Value::as_str) {
            if !rooms.iter().any(|r| r == name) {
                rooms.push(name.to_string());
            }
        }
        if let Some(floor) = mark.get("floor").and_then(Value::as_i64) {
            floors.push(floor);
        }
    }
    floors.sort_unstable();
    floors.dedup();
    (rooms, floors)
}

/// The row for `dir` (blocking file I/O).
pub fn row(dir: &SessionDir, upload: UploadView) -> SessionRow {
    let manifest = manifest_of(&dir.path);
    let started_at_utc = manifest
        .get("utc_at_t0_ns")
        .and_then(Value::as_i64)
        .map(|ns| format_utc(ns.div_euclid(1_000_000_000)))
        .or_else(|| utc_from_id(&dir.id));
    let (rooms, floors) = rooms_and_floors(&manifest);
    let text = |v: Option<&Value>| v.and_then(Value::as_str).map(str::to_string);
    SessionRow {
        session_id: dir.id.clone(),
        started_at_utc,
        duration_s: duration_s(&manifest),
        size_bytes: dir_size(&dir.path),
        frames: None,
        rooms,
        floors,
        timebase: text(manifest.get("timebase")),
        anchor_source: text(manifest.get("anchor").and_then(|a| a.get("source"))),
        uploaded: dir.uploaded,
        upload,
    }
}

/// Backdate `dir`'s `.recording` by `secs` (tests).
#[cfg(test)]
pub(crate) fn age_marker(dir: &Path, secs: u64) {
    let f = std::fs::File::options()
        .write(true)
        .open(dir.join(RECORDING_MARKER))
        .unwrap();
    f.set_modified(SystemTime::now() - Duration::from_secs(secs))
        .unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn idle(total: u64) -> UploadView {
        UploadView::idle(total)
    }

    #[test]
    fn a_marker_is_stale_only_when_old_and_not_the_open_session() {
        let root = tempfile::tempdir().unwrap();
        let id = "visor-1-20261019T101500Z-3fa2";
        let d = root.path().join(id);
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join(RECORDING_MARKER), b"").unwrap();
        let fresh = SessionDir::open(root.path(), id).unwrap();
        assert!(!fresh.stale(&LiveSession::None));
        age_marker(&d, 300);
        let old = SessionDir::open(root.path(), id).unwrap();
        assert!(old.stale(&LiveSession::None));
        assert!(old.stale(&LiveSession::Open("other".into())));
        assert!(!old.stale(&LiveSession::Open(id.into())));
        assert!(!old.stale(&LiveSession::Unknown));
        let v = serde_json::to_value(not_finished_view(9)).unwrap();
        assert_eq!(v["state"], "failed");
        assert_eq!(v["error"], "not_finished");
    }

    #[test]
    fn a_finished_session_reads_its_manifest() {
        let root = tempfile::tempdir().unwrap();
        let id = "visor-1-20261019T101500Z-3fa2";
        let dir = root.path().join(id);
        std::fs::create_dir_all(dir.join("segments")).unwrap();
        std::fs::write(dir.join("segments/000000.mcap"), vec![0u8; 1000]).unwrap();
        let manifest = json!({
            "session_id": id, "timebase": "hub_freerun", "utc_at_t0_ns": 1_760_868_900_000_000_000i64,
            "t0_ns": 5_000_000_000u64,
            "anchor": {"source": "operator"},
            "operator": {"floor": 0, "rooms": [
                {"name": "Dock", "floor": 0, "t_start_ns": 1, "t_end_ns": 2},
                {"name": "Aisle 1", "floor": 1, "t_start_ns": 2, "t_end_ns": 3},
                {"name": "Dock", "floor": 0, "t_start_ns": 3, "t_end_ns": 4}]},
            "segments": [
                {"file": "segments/000000.mcap", "t_start_ns": 4_990_000_000u64, "t_end_ns": 9_000_000_000u64, "bytes": 1000},
                {"file": "segments/000001.mcap", "t_start_ns": 9_000_000_000u64, "t_end_ns": 317_400_000_000u64, "bytes": 0}]
        });
        let body = serde_json::to_vec(&manifest).unwrap();
        std::fs::write(dir.join(MANIFEST_NAME), &body).unwrap();
        std::fs::write(dir.join(UPLOADED_MARKER), b"{}").unwrap();

        let dirs = scan(root.path());
        assert_eq!(dirs.len(), 1);
        let r = row(&dirs[0], idle(0));
        assert_eq!(r.started_at_utc.as_deref(), Some("2025-10-19T10:15:00Z"));
        // From t0, not from the IMU sample stamped 10 ms before it.
        assert_eq!(r.duration_s, Some(312.4));
        let mut no_t0 = manifest.clone();
        no_t0.as_object_mut().unwrap().remove("t0_ns");
        assert_eq!(duration_s(&no_t0), Some(312.41));
        assert_eq!(r.size_bytes, 1000 + body.len() as u64 + 2);
        assert_eq!(r.frames, None);
        assert_eq!(r.rooms, ["Dock", "Aisle 1"]);
        assert_eq!(r.floors, [0, 1]);
        assert_eq!(r.timebase.as_deref(), Some("hub_freerun"));
        assert_eq!(r.anchor_source.as_deref(), Some("operator"));
        assert!(r.uploaded);
        let v = serde_json::to_value(&r).unwrap();
        assert_eq!(v["upload"]["state"], "idle");
    }

    #[test]
    fn a_recording_session_is_listed_and_strays_are_not() {
        let root = tempfile::tempdir().unwrap();
        let id = "drone-7-20261019T080000Z-00ab";
        std::fs::create_dir_all(root.path().join(id)).unwrap();
        std::fs::write(root.path().join(id).join(RECORDING_MARKER), b"").unwrap();
        std::fs::create_dir_all(root.path().join("lost+found")).unwrap();
        std::fs::create_dir_all(root.path().join("empty-dir")).unwrap();
        let dirs = scan(root.path());
        assert_eq!(dirs.len(), 1);
        assert!(dirs[0].recording && !dirs[0].finished);
        let r = row(&dirs[0], idle(0));
        assert_eq!(r.started_at_utc.as_deref(), Some("2026-10-19T08:00:00Z"));
        assert_eq!(r.duration_s, None);
        assert!(r.rooms.is_empty() && r.timebase.is_none());
        assert_eq!(r.upload.state, UploadState::Idle);
        assert!(SessionDir::open(root.path(), "../x").is_none());
    }
}
