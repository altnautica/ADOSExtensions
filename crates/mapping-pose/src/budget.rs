//! A process's memory budget: the RAM its in-memory stores may hold (the
//! capture service's keyframe and scan queues, navigation's live sweep cloud
//! and occupancy tiles). Unset, it is 80 % of the unit's memory cap (the
//! manifest's per-profile cap the host applies): `ADOS_PLUGIN_MEMORY_MAX_BYTES`,
//! which the host exports into the unit, else the cgroup's `memory.max` (a
//! cgroup namespace can hide it), so a 4 GB drone and a workstation get
//! budgets that fit them without a per-board setting. Each process splits its
//! budget into its own shares ([`Budget::share`]).

use std::path::Path;

const MIB: u64 = 1024 * 1024;
/// The budget when neither the config nor the cgroup gives one (no cgroup v2,
/// or `memory.max` is `max`).
pub const FALLBACK_BUDGET_MB: u64 = 512;
/// The configured floor.
pub const MIN_BUDGET_MB: u64 = 16;

/// The resolved budget, bytes.
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

    /// `percent` of the budget, bytes.
    pub fn share(&self, percent: u64) -> usize {
        (self.total * percent / 100) as usize
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
    fn a_share_is_its_percent_of_the_whole() {
        let b = Budget::from_mb(100);
        assert_eq!(b.share(60) + b.share(40), b.total as usize);
        assert_eq!(b.share(40) as u64, 40 * MIB);
        assert_eq!(Budget::from_mb(1).total, MIN_BUDGET_MB * MIB);
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
        let unit = root
            .path()
            .join("sys/fs/cgroup/ados.slice/ados-plugin-x.service");
        std::fs::create_dir_all(&unit).unwrap();
        std::fs::write(unit.join("memory.max"), "805306368\n").unwrap();
        // 768 MiB * 0.8.
        assert_eq!(resolve(None, None, root.path()).total, 614 * MIB);
        assert_eq!(resolve(Some(64), None, root.path()).total, 64 * MIB);
        // The host's export wins over the cgroup read.
        assert_eq!(
            resolve(None, Some("1073741824"), root.path()).total,
            819 * MIB
        );

        std::fs::write(unit.join("memory.max"), "max\n").unwrap();
        assert_eq!(
            resolve(None, None, root.path()).total,
            FALLBACK_BUDGET_MB * MIB
        );
    }
}
