//! `checksums.b3`: one `<blake3 hex>  <relative path>` line per session file,
//! sorted by path, `/`-separated. The checksum file itself is not listed, nor
//! any dot-file (the device's state markers such as `.recording` and
//! `.uploaded`, which come and go after the session is closed).

use std::collections::BTreeSet;
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
        if path
            .file_name()
            .is_some_and(|n| n.to_string_lossy().starts_with('.'))
        {
            continue;
        }
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

/// A listed path stays inside the session: `/`-separated, relative, no empty,
/// `.`/`..` or dot-file segment, no backslash and no drive prefix (`C:`).
fn safe_rel(rel: &str) -> bool {
    rel != CHECKSUMS_FILE
        && rel.split('/').all(|seg| {
            !seg.is_empty() && !seg.starts_with('.') && !seg.contains('\\') && !seg.contains(':')
        })
}

/// Parse a `checksums.b3` body into `(rel path, lowercase hex hash)` pairs,
/// rejecting malformed lines, bad hashes, duplicates and unsafe paths.
pub fn parse_checksums(body: &str) -> Result<Vec<(String, String)>> {
    let mut seen = BTreeSet::new();
    body.lines()
        .filter(|l| !l.is_empty())
        .map(|line| {
            let (hash, rel) = line
                .split_once("  ")
                .ok_or_else(|| Error::Invalid(format!("bad checksum line `{line}`")))?;
            if hash.len() != 64 || !hash.bytes().all(|b| b.is_ascii_hexdigit()) {
                return Err(Error::Invalid(format!("bad hash for `{rel}`")));
            }
            if !safe_rel(rel) {
                return Err(Error::Invalid(format!(
                    "checksum path `{rel}` escapes the session"
                )));
            }
            if !seen.insert(rel) {
                return Err(Error::Invalid(format!(
                    "checksum path `{rel}` listed twice"
                )));
            }
            Ok((rel.to_string(), hash.to_ascii_lowercase()))
        })
        .collect()
}

/// Result of comparing a session directory with its `checksums.b3`.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct ChecksumReport {
    /// Listed but absent on disk.
    pub missing: Vec<String>,
    /// On disk (dot-files excluded) but not listed.
    pub unlisted: Vec<String>,
    /// Listed and present with a different hash. Only computed when nothing
    /// is missing or unlisted.
    pub mismatched: Vec<String>,
    /// Files on disk covered by the comparison, and their total size.
    pub files: u64,
    pub bytes: u64,
}

impl ChecksumReport {
    /// The listed set equals the files on disk and every hash matches.
    pub fn is_ok(&self) -> bool {
        self.missing.is_empty() && self.unlisted.is_empty() && self.mismatched.is_empty()
    }
}

/// Compare `session_dir` against its `checksums.b3`, reporting every failure.
/// Errors when the list is absent (`Io`), malformed or unsafe (`Invalid`).
pub fn check_checksums(session_dir: &Path) -> Result<ChecksumReport> {
    let body = fs::read_to_string(session_dir.join(CHECKSUMS_FILE))?;
    let listing = parse_checksums(&body)?;
    let mut files = Vec::new();
    collect(session_dir, session_dir, &mut files)?;
    let on_disk: BTreeSet<String> = files.iter().map(|rel| rel_string(rel)).collect();
    let listed: BTreeSet<&str> = listing.iter().map(|(rel, _)| rel.as_str()).collect();

    let mut report = ChecksumReport {
        missing: listed
            .iter()
            .filter(|rel| !on_disk.contains(**rel))
            .map(|rel| rel.to_string())
            .collect(),
        unlisted: on_disk
            .iter()
            .filter(|rel| !listed.contains(rel.as_str()))
            .cloned()
            .collect(),
        ..ChecksumReport::default()
    };
    for rel in &files {
        report.files += 1;
        report.bytes += fs::metadata(session_dir.join(rel))?.len();
    }
    if report.missing.is_empty() && report.unlisted.is_empty() {
        for (rel, hash) in &listing {
            if hash_file(&session_dir.join(rel))? != *hash {
                report.mismatched.push(rel.clone());
            }
        }
    }
    Ok(report)
}

/// Require the files listed in `checksums.b3` to be exactly the session files
/// on disk (dot-files excluded) with matching hashes.
pub fn verify_checksums(session_dir: &Path) -> Result<()> {
    let report = check_checksums(session_dir)?;
    if let Some(rel) = report.missing.first() {
        return Err(Error::Checksum(format!("{rel} missing")));
    }
    if let Some(rel) = report.unlisted.first() {
        return Err(Error::Checksum(format!("{rel} not listed")));
    }
    if let Some(rel) = report.mismatched.first() {
        return Err(Error::Checksum(rel.clone()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const H: &str = "af1349b9f5f9a1a6a0404dea36dcc9499bcb25c9adc112b7cc9a93cae41f3262";

    fn session() -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir(dir.path().join("cam")).unwrap();
        fs::write(dir.path().join("cam/a.bin"), b"a").unwrap();
        fs::write(dir.path().join("manifest.json"), b"{}").unwrap();
        fs::write(dir.path().join(".recording"), b"").unwrap();
        write_checksums(dir.path()).unwrap();
        dir
    }

    #[test]
    fn round_trip_verifies() {
        let dir = session();
        verify_checksums(dir.path()).unwrap();
        let report = check_checksums(dir.path()).unwrap();
        assert_eq!((report.files, report.bytes), (2, 3));
    }

    #[test]
    fn unlisted_file_fails() {
        let dir = session();
        fs::write(dir.path().join("extra.bin"), b"x").unwrap();
        let report = check_checksums(dir.path()).unwrap();
        assert_eq!(report.unlisted, vec!["extra.bin".to_string()]);
        assert!(verify_checksums(dir.path()).is_err());
    }

    #[test]
    fn missing_and_mismatch_fail() {
        let dir = session();
        fs::write(dir.path().join("cam/a.bin"), b"b").unwrap();
        let report = check_checksums(dir.path()).unwrap();
        assert_eq!(report.mismatched, vec!["cam/a.bin".to_string()]);
        fs::remove_file(dir.path().join("cam/a.bin")).unwrap();
        let report = check_checksums(dir.path()).unwrap();
        assert_eq!(report.missing, vec!["cam/a.bin".to_string()]);
        assert!(verify_checksums(dir.path()).is_err());
    }

    #[test]
    fn unsafe_paths_rejected() {
        for rel in [
            "../x",
            "a/../x",
            "/etc/x",
            "a\\..\\x",
            "C:x",
            "C:/x",
            "a//b",
            ".hidden",
            CHECKSUMS_FILE,
        ] {
            let body = format!("{H}  {rel}\n");
            assert!(parse_checksums(&body).is_err(), "{rel} accepted");
        }
        assert!(parse_checksums(&format!("{H}  a\n{H}  a\n")).is_err());
        assert!(parse_checksums("zz  a\n").is_err());
        assert_eq!(
            parse_checksums(&format!("{H}  cam/a.bin\n")).unwrap().len(),
            1
        );
    }
}
