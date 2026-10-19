//! Storage limits of the upload lane: a cap on the bytes held across every
//! session and a floor of free space left on the store's volume.

use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};
use std::sync::Arc;

/// Free space left on the volume by default, 5 GiB.
pub const DEFAULT_MIN_FREE_BYTES: u64 = 5 << 30;

/// Free bytes on the volume holding a path; `None` when unknown.
pub type FreeSpaceFn = Arc<dyn Fn(&Path) -> Option<u64> + Send + Sync>;

/// Configured limits plus the free-space lookup.
#[derive(Clone)]
pub(crate) struct Limits {
    pub max_total_bytes: Option<u64>,
    pub min_free_bytes: u64,
    pub free_space: FreeSpaceFn,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_total_bytes: None,
            min_free_bytes: DEFAULT_MIN_FREE_BYTES,
            free_space: Arc::new(volume_free_bytes),
        }
    }
}

impl Limits {
    /// Bytes the store may still accept under `root`.
    pub(crate) fn headroom(&self, root: &Path) -> io::Result<u64> {
        let mut room = u64::MAX;
        if let Some(max) = self.max_total_bytes {
            room = room.min(max.saturating_sub(tree_bytes(root)?));
        }
        if let Some(free) = (self.free_space)(root) {
            room = room.min(free.saturating_sub(self.min_free_bytes));
        }
        Ok(room)
    }
}

/// Total size of every regular file under `dir`, bookkeeping included.
fn tree_bytes(dir: &Path) -> io::Result<u64> {
    let entries = match fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(0),
        Err(e) => return Err(e),
    };
    let mut total = 0u64;
    for entry in entries {
        let entry = entry?;
        let kind = entry.file_type()?;
        if kind.is_dir() {
            total += tree_bytes(&entry.path())?;
        } else if kind.is_file() {
            total += entry.metadata()?.len();
        }
    }
    Ok(total)
}

/// The nearest existing ancestor of `path`, made absolute without the
/// Windows verbatim prefix so it compares against disk mount points.
fn existing_absolute(path: &Path) -> Option<PathBuf> {
    let mut probe = path;
    loop {
        if let Ok(abs) = fs::canonicalize(probe) {
            return Some(strip_verbatim(abs));
        }
        probe = probe.parent()?;
    }
}

fn strip_verbatim(path: PathBuf) -> PathBuf {
    let text = path.to_string_lossy();
    match text.strip_prefix(r"\\?\") {
        Some(rest) if !rest.starts_with("UNC") => PathBuf::from(rest),
        _ => path,
    }
}

/// Available space on the disk whose mount point is the longest prefix of
/// `path`.
pub(crate) fn volume_free_bytes(path: &Path) -> Option<u64> {
    let abs = existing_absolute(path)?;
    let disks = sysinfo::Disks::new_with_refreshed_list();
    disks
        .list()
        .iter()
        .filter(|d| abs.starts_with(d.mount_point()))
        .max_by_key(|d| d.mount_point().components().filter(not_root).count())
        .map(sysinfo::Disk::available_space)
}

fn not_root(c: &Component) -> bool {
    !matches!(c, Component::RootDir | Component::Prefix(_))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real_volume_reports_free_space() {
        let dir = tempfile::tempdir().unwrap();
        assert!(volume_free_bytes(&dir.path().join("not-yet")).is_some_and(|n| n > 0));
    }

    #[test]
    fn headroom_takes_the_tighter_limit() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("s/.upload")).unwrap();
        fs::write(dir.path().join("s/a"), [0u8; 100]).unwrap();
        fs::write(dir.path().join("s/.upload/b.part"), [0u8; 50]).unwrap();
        let limits = |max, free: u64| Limits {
            max_total_bytes: max,
            min_free_bytes: 1000,
            free_space: Arc::new(move |_: &Path| Some(free)),
        };
        assert_eq!(limits(Some(400), 10_000).headroom(dir.path()).unwrap(), 250);
        assert_eq!(limits(Some(400), 1100).headroom(dir.path()).unwrap(), 100);
        assert_eq!(limits(None, 500).headroom(dir.path()).unwrap(), 0);
    }
}
