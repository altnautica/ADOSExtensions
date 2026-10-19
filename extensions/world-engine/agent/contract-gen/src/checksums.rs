//! `checksums.b3`: one `<blake3 hex>  <relative path>` line per session file,
//! sorted by path, `/`-separated. The checksum file itself is not listed.

use std::fs::{self, File};
use std::path::{Path, PathBuf};

use crate::{Error, Result};

/// File name of the checksum list in a session directory.
pub const CHECKSUMS_FILE: &str = "checksums.b3";

fn hash_file(path: &Path) -> Result<String> {
    let mut hasher = blake3::Hasher::new();
    hasher.update_reader(File::open(path)?)?;
    Ok(hasher.finalize().to_hex().to_string())
}

fn collect(dir: &Path, root: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            collect(&path, root, out)?;
        } else if path != root.join(CHECKSUMS_FILE) {
            out.push(path.strip_prefix(root).expect("under root").to_path_buf());
        }
    }
    Ok(())
}

fn rel_string(rel: &Path) -> String {
    rel.components()
        .map(|c| c.as_os_str().to_string_lossy())
        .collect::<Vec<_>>()
        .join("/")
}

/// Hash every file under `session_dir` and write `checksums.b3`.
pub fn write_checksums(session_dir: &Path) -> Result<()> {
    let mut files = Vec::new();
    collect(session_dir, session_dir, &mut files)?;
    let mut lines: Vec<(String, String)> = files
        .iter()
        .map(|rel| Ok((rel_string(rel), hash_file(&session_dir.join(rel))?)))
        .collect::<Result<_>>()?;
    lines.sort();
    let body: String = lines.iter().map(|(p, h)| format!("{h}  {p}\n")).collect();
    fs::write(session_dir.join(CHECKSUMS_FILE), body)?;
    Ok(())
}

/// Recompute every hash listed in `checksums.b3`; fails on the first file that
/// is missing, differs, or escapes the session directory.
pub fn verify_checksums(session_dir: &Path) -> Result<()> {
    let body = fs::read_to_string(session_dir.join(CHECKSUMS_FILE))?;
    for line in body.lines().filter(|l| !l.is_empty()) {
        let (hash, rel) = line
            .split_once("  ")
            .ok_or_else(|| Error::Invalid(format!("bad checksum line `{line}`")))?;
        if rel.split('/').any(|c| c == ".." || c.is_empty()) {
            return Err(Error::Invalid(format!(
                "checksum path `{rel}` escapes the session"
            )));
        }
        if hash_file(&session_dir.join(rel))? != hash {
            return Err(Error::Checksum(rel.to_string()));
        }
    }
    Ok(())
}
