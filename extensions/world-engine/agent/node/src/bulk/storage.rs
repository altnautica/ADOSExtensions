//! Storage limits of the upload lane: a cap on the bytes held across every
//! session and a floor of free space left on the store's volume. Uploads
//! reserve their bytes through one store-wide ledger, so concurrent uploads
//! to different sessions cannot jointly overrun a limit.

use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

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
    /// Bytes the store may still accept under `root`, less `outstanding`
    /// bytes already granted to uploads but not yet on disk.
    fn headroom(&self, root: &Path, outstanding: u64) -> io::Result<u64> {
        let mut room = u64::MAX;
        if let Some(max) = self.max_total_bytes {
            room = room.min(max.saturating_sub(tree_bytes(root)?));
        }
        if let Some(free) = (self.free_space)(root) {
            room = room.min(free.saturating_sub(self.min_free_bytes));
        }
        Ok(room.saturating_sub(outstanding))
    }

    /// Grant an upload into `part` (currently `start` bytes long) `want`
    /// bytes, or when undeclared as many as the limits allow up to `cap`.
    /// `Ok(Err(headroom))` when `want` does not fit. Grants are made one at a
    /// time under the ledger lock and count every in-flight upload's
    /// not-yet-written bytes.
    pub(crate) fn reserve(
        &self,
        ledger: &Ledger,
        root: &Path,
        part: PathBuf,
        start: u64,
        want: Option<u64>,
        cap: u64,
    ) -> io::Result<Result<Reservation, u64>> {
        let mut state = lock(&ledger.state);
        // In-flight bytes first, disk usage second: a write landing in between
        // is then counted twice (conservative), never missed.
        let outstanding: u64 = state.pending.iter().map(Pending::outstanding).sum();
        let headroom = self.headroom(root, outstanding)?;
        let amount = match want {
            Some(n) if n > headroom => return Ok(Err(headroom)),
            Some(n) => n,
            None => headroom.min(cap),
        };
        state.next += 1;
        let key = state.next;
        state.pending.push(Pending {
            key,
            part,
            start,
            amount,
        });
        Ok(Ok(Reservation {
            state: ledger.state.clone(),
            key,
            amount,
        }))
    }
}

/// Uploads in flight and the bytes each was granted.
#[derive(Default)]
pub(crate) struct Ledger {
    state: Arc<Mutex<LedgerState>>,
}

#[derive(Default)]
struct LedgerState {
    next: u64,
    pending: Vec<Pending>,
}

struct Pending {
    key: u64,
    part: PathBuf,
    start: u64,
    amount: u64,
}

impl Pending {
    /// Granted bytes not yet written to the partial file.
    fn outstanding(&self) -> u64 {
        let len = fs::metadata(&self.part).map_or(0, |m| m.len());
        self.amount.saturating_sub(len.saturating_sub(self.start))
    }
}

fn lock(state: &Mutex<LedgerState>) -> MutexGuard<'_, LedgerState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Bytes granted to one upload; released when dropped.
pub(crate) struct Reservation {
    state: Arc<Mutex<LedgerState>>,
    key: u64,
    amount: u64,
}

impl Reservation {
    /// The granted byte count.
    pub(crate) fn amount(&self) -> u64 {
        self.amount
    }
}

impl Drop for Reservation {
    fn drop(&mut self) {
        lock(&self.state).pending.retain(|p| p.key != self.key);
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
        let headroom = |l: Limits, out| l.headroom(dir.path(), out).unwrap();
        assert_eq!(headroom(limits(Some(400), 10_000), 0), 250);
        assert_eq!(headroom(limits(Some(400), 1100), 0), 100);
        assert_eq!(headroom(limits(Some(400), 1100), 30), 70);
        assert_eq!(headroom(limits(None, 500), 0), 0);
    }

    #[test]
    fn reservations_hold_bytes_until_dropped() {
        let dir = tempfile::tempdir().unwrap();
        let limits = Limits {
            max_total_bytes: Some(100),
            min_free_bytes: 0,
            free_space: Arc::new(|_: &Path| None),
        };
        let ledger = Ledger::default();
        let part = dir.path().join("a.part");
        let reserve = |rel: &str, want| {
            limits
                .reserve(&ledger, dir.path(), dir.path().join(rel), 0, want, 1000)
                .unwrap()
        };
        let first = reserve("a.part", Some(60)).unwrap();
        assert_eq!(reserve("b.part", Some(41)).err(), Some(40));
        // Bytes already written count once: on disk, not as outstanding.
        fs::write(&part, [0u8; 30]).unwrap();
        assert_eq!(reserve("b.part", Some(41)).err(), Some(40));
        assert_eq!(reserve("b.part", None).unwrap().amount(), 40);
        drop(first);
        assert_eq!(reserve("b.part", None).unwrap().amount(), 70);
    }
}
