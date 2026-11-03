//! `atlas.memory_budget_mb`: the RAM the live map stores may hold, and how it
//! splits. Unset, it is 80 % of the capture unit's memory cap (the manifest's
//! per-profile cap the host applies): `ADOS_PLUGIN_MEMORY_MAX_BYTES`, which
//! the host exports into the unit, else the cgroup's `memory.max` (a cgroup
//! namespace can hide it), so a 4 GB drone and a workstation get budgets that
//! fit them without a per-board setting.

use std::path::Path;

const MIB: u64 = 1024 * 1024;
/// The budget when neither the config nor the cgroup gives one (no cgroup v2,
/// or `memory.max` is `max`).
pub const FALLBACK_BUDGET_MB: u64 = 512;
/// The configured floor.
pub const MIN_BUDGET_MB: u64 = 16;

/// The resolved budget and its shares, bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Budget {
    pub total: u64,
}

impl Budget {
    pub fn from_mb(mb: u64) -> Self {
        Self {
            total: mb.max(MIN_BUDGET_MB) * MIB,
        }
    }

    /// The keyframe queue (JPEG held in RAM until written): 40 %.
    pub fn keyframes(&self) -> usize {
        (self.total * 40 / 100) as usize
    }

    /// The scan ring (polar records waiting for the writer): 10 %.
    pub fn scans(&self) -> usize {
        (self.total * 10 / 100) as usize
    }

    /// The live sweep-cloud voxel hash: 30 %.
    pub fn sweep(&self) -> usize {
        (self.total * 30 / 100) as usize
    }

    /// The live occupancy tiles: 20 %.
    pub fn occupancy(&self) -> usize {
        (self.total * 20 / 100) as usize
    }

    pub fn total_mb(&self) -> f32 {
        self.total as f32 / MIB as f32
    }
}

/// The unit's `memory.max` in bytes from a cgroup v2 `memory.max` body
/// (`"max"` reads as unlimited).
pub fn parse_memory_max(body: &str) -> Option<u64> {
    body.trim().parse().ok()
}

/// This process's cgroup v2 directory from `/proc/self/cgroup` (`0::/path`).
pub fn cgroup_dir(proc_cgroup: &str) -> Option<String> {
    proc_cgroup
        .lines()
        .find_map(|l| l.strip_prefix("0::"))
        .map(|p| format!("/sys/fs/cgroup{}", p.trim_end()))
}

/// The env var the plugin host exports the unit's `MemoryMax` in, bytes.
pub const MEMORY_MAX_ENV: &str = "ADOS_PLUGIN_MEMORY_MAX_BYTES";

/// The budget: the configured MiB, else 80 % of the unit cap (`env_cap`, the
/// host's export, else the cgroup's read under `root`, `/` on a device), else
/// [`FALLBACK_BUDGET_MB`].
pub fn resolve(configured_mb: Option<u64>, env_cap: Option<&str>, root: &Path) -> Budget {
    if let Some(mb) = configured_mb {
        return Budget::from_mb(mb);
    }
    let from_env = env_cap.and_then(parse_memory_max);
    let cap = from_env.or_else(|| cgroup_cap(root));
    match cap {
        Some(bytes) => Budget::from_mb(bytes * 8 / 10 / MIB),
        None => Budget::from_mb(FALLBACK_BUDGET_MB),
    }
}

fn cgroup_cap(root: &Path) -> Option<u64> {
    std::fs::read_to_string(root.join("proc/self/cgroup"))
        .ok()
        .and_then(|c| cgroup_dir(&c))
        .and_then(|d| {
            std::fs::read_to_string(root.join(d.trim_start_matches('/')).join("memory.max")).ok()
        })
        .and_then(|b| parse_memory_max(&b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_shares_add_to_the_whole() {
        let b = Budget::from_mb(100);
        let sum = b.keyframes() + b.scans() + b.sweep() + b.occupancy();
        assert_eq!(sum as u64, b.total);
        assert_eq!(b.keyframes() as u64, 40 * MIB);
    }

    #[test]
    fn unset_it_is_80_percent_of_the_unit_cap() {
        let root = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(root.path().join("proc/self")).unwrap();
        std::fs::write(
            root.path().join("proc/self/cgroup"),
            "0::/ados.slice/ados-plugin-x.service\n",
        )
        .unwrap();
        let unit = root.path().join("sys/fs/cgroup/ados.slice/ados-plugin-x.service");
        std::fs::create_dir_all(&unit).unwrap();
        std::fs::write(unit.join("memory.max"), "805306368\n").unwrap();
        // 768 MiB * 0.8.
        assert_eq!(resolve(None, None, root.path()).total, 614 * MIB);
        assert_eq!(resolve(Some(64), None, root.path()).total, 64 * MIB);
        // The host's export wins over the cgroup read.
        assert_eq!(resolve(None, Some("1073741824"), root.path()).total, 819 * MIB);

        std::fs::write(unit.join("memory.max"), "max\n").unwrap();
        assert_eq!(resolve(None, None, root.path()).total, FALLBACK_BUDGET_MB * MIB);
    }
}
