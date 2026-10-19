//! On-disk upload state: file listings, partial files, the commit marker, and
//! streaming a request body into a partial file.

use std::collections::BTreeMap;
use std::fs;
use std::io::{self, SeekFrom};
use std::path::{Path, PathBuf};

use axum::body::Body;
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncSeekExt, AsyncWriteExt};

use super::{COMMITTED_MARKER, UPLOAD_DIR};

/// Upload progress of one session file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub(crate) struct FileState {
    pub received: u64,
    /// Declared total; `None` while every chunk so far used `/*`.
    pub total: Option<u64>,
    pub complete: bool,
}

impl FileState {
    fn complete(len: u64) -> Self {
        Self {
            received: len,
            total: Some(len),
            complete: true,
        }
    }
}

/// Contents of the `.committed` marker.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub(crate) struct Marker {
    pub committed_ms: u64,
    pub files: u64,
    pub bytes: u64,
}

pub(crate) fn part_path(dir: &Path, rel: &str) -> PathBuf {
    dir.join(UPLOAD_DIR).join(format!("{rel}.part"))
}

pub(crate) fn total_path(dir: &Path, rel: &str) -> PathBuf {
    dir.join(UPLOAD_DIR).join(format!("{rel}.total"))
}

/// The declared total recorded for a partial file.
pub(crate) fn read_total(dir: &Path, rel: &str) -> io::Result<Option<u64>> {
    match fs::read_to_string(total_path(dir, rel)) {
        Ok(s) => s
            .trim()
            .parse()
            .map(Some)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "bad upload total")),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e),
    }
}

/// Upload state of `rel`, `None` when nothing has been received.
pub(crate) fn file_state(dir: &Path, rel: &str) -> io::Result<Option<FileState>> {
    if let Ok(meta) = fs::metadata(dir.join(rel)) {
        if meta.is_file() {
            return Ok(Some(FileState::complete(meta.len())));
        }
    }
    match fs::metadata(part_path(dir, rel)) {
        Ok(meta) if meta.is_file() => Ok(Some(FileState {
            received: meta.len(),
            total: read_total(dir, rel)?,
            complete: false,
        })),
        _ => Ok(None),
    }
}

/// Whether `rel` cannot become a file: it names an existing directory, or one
/// of its parents is an existing file.
pub(crate) fn blocked_by_existing(dir: &Path, rel: &str) -> bool {
    if dir.join(rel).is_dir() {
        return true;
    }
    let mut prefix = PathBuf::from(dir);
    let mut segs = rel.split('/').peekable();
    while let Some(seg) = segs.next() {
        if segs.peek().is_none() {
            break;
        }
        prefix.push(seg);
        if prefix.is_file() {
            return true;
        }
    }
    false
}

/// Every fully received file (`/`-separated rel path, length), sorted.
pub(crate) fn complete_files(dir: &Path) -> io::Result<Vec<(String, u64)>> {
    let mut out = Vec::new();
    walk(dir, "", &mut out)?;
    out.sort();
    Ok(out)
}

/// Rel paths of every partial file, sorted.
pub(crate) fn pending_parts(dir: &Path) -> io::Result<Vec<String>> {
    let mut out = Vec::new();
    walk(&dir.join(UPLOAD_DIR), "", &mut out)?;
    let mut rels: Vec<String> = out
        .into_iter()
        .filter_map(|(rel, _)| rel.strip_suffix(".part").map(str::to_string))
        .collect();
    rels.sort();
    Ok(rels)
}

/// Every received or partially received file of a session.
pub(crate) fn list_files(dir: &Path) -> io::Result<BTreeMap<String, FileState>> {
    let mut out: BTreeMap<String, FileState> = complete_files(dir)?
        .into_iter()
        .map(|(rel, len)| (rel, FileState::complete(len)))
        .collect();
    for rel in pending_parts(dir)? {
        if out.contains_key(&rel) {
            continue;
        }
        let received = fs::metadata(part_path(dir, &rel))?.len();
        let total = read_total(dir, &rel)?;
        out.insert(
            rel,
            FileState {
                received,
                total,
                complete: false,
            },
        );
    }
    Ok(out)
}

/// Collect regular files under `dir`, skipping dot-names (bookkeeping).
fn walk(dir: &Path, prefix: &str, out: &mut Vec<(String, u64)>) -> io::Result<()> {
    let entries = match fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(e),
    };
    for entry in entries {
        let entry = entry?;
        let Ok(name) = entry.file_name().into_string() else {
            continue;
        };
        if name.starts_with('.') {
            continue;
        }
        let rel = if prefix.is_empty() {
            name
        } else {
            format!("{prefix}/{name}")
        };
        let kind = entry.file_type()?;
        if kind.is_dir() {
            walk(&entry.path(), &rel, out)?;
        } else if kind.is_file() {
            out.push((rel, entry.metadata()?.len()));
        }
    }
    Ok(())
}

pub(crate) fn is_committed(dir: &Path) -> bool {
    dir.join(COMMITTED_MARKER).is_file()
}

pub(crate) fn read_marker(dir: &Path) -> io::Result<Option<Marker>> {
    match fs::read(dir.join(COMMITTED_MARKER)) {
        Ok(body) => serde_json::from_slice(&body)
            .map(Some)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e)),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e),
    }
}

/// Write `<dir>/<name>` through a temporary sibling and a rename.
pub(crate) async fn write_atomic(dir: &Path, name: &str, body: &[u8]) -> io::Result<()> {
    let tmp = dir.join(format!(".{name}.tmp"));
    tokio::fs::write(&tmp, body).await?;
    tokio::fs::rename(&tmp, dir.join(name)).await
}

pub(crate) async fn write_marker(dir: &Path, marker: &Marker) -> io::Result<()> {
    let body = serde_json::to_vec(marker).map_err(io::Error::other)?;
    write_atomic(dir, COMMITTED_MARKER, &body).await
}

pub(crate) async fn remove_if_exists(path: &Path) -> io::Result<()> {
    match tokio::fs::remove_file(path).await {
        Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
}

/// Why a chunk was not stored.
#[derive(Debug)]
pub(crate) enum StreamFail {
    /// The body length differs from the declared range.
    Length,
    /// The body exceeds the per-request cap.
    TooLarge,
    /// The client's body stream failed.
    Body(String),
    Io(io::Error),
}

/// Append a request body to `part` at `start` (its current length). A chunk
/// lands whole or not at all: on any failure the file is cut back to `start`
/// (removed when `start` is 0).
pub(crate) async fn stream_to_part(
    part: &Path,
    start: u64,
    body: Body,
    expected: Option<u64>,
    cap: u64,
) -> Result<u64, StreamFail> {
    let mut file = tokio::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(false)
        .open(part)
        .await
        .map_err(StreamFail::Io)?;
    let result = match file.seek(SeekFrom::Start(start)).await {
        Ok(_) => copy_body(&mut file, body, expected, cap).await,
        Err(e) => Err(StreamFail::Io(e)),
    };
    if result.is_err() {
        let _ = file.flush().await;
        let _ = file.set_len(start).await;
        drop(file);
        if start == 0 {
            let _ = tokio::fs::remove_file(part).await;
        }
    }
    result
}

async fn copy_body(
    file: &mut tokio::fs::File,
    body: Body,
    expected: Option<u64>,
    cap: u64,
) -> Result<u64, StreamFail> {
    let mut stream = body.into_data_stream();
    let mut n: u64 = 0;
    while let Some(chunk) = stream.next().await {
        let chunk = chunk.map_err(|e| StreamFail::Body(e.to_string()))?;
        n += chunk.len() as u64;
        if n > cap {
            return Err(StreamFail::TooLarge);
        }
        if expected.is_some_and(|e| n > e) {
            return Err(StreamFail::Length);
        }
        file.write_all(&chunk).await.map_err(StreamFail::Io)?;
    }
    if expected.is_some_and(|e| n != e) {
        return Err(StreamFail::Length);
    }
    file.flush().await.map_err(StreamFail::Io)?;
    Ok(n)
}
