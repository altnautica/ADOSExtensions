//! Host-GPU reporting for the workstation profile.
//!
//! The compute status carries a `gpu` block: whether the node has a hardware
//! GPU, its identity (name, core count, Metal support, graphics memory) and a
//! live utilisation sample. The identity is static (resolved once, cached); the
//! utilisation is sampled with a short cache so a rapid poll never re-spawns
//! `powermetrics` or `nvidia-smi`.
//!
//! Best-effort + honest: every field is `Option`, and any failure (a
//! missing tool, no passwordless sudo for `powermetrics`, a parse miss) leaves
//! that field `null` rather than fabricating a value. A platform with no probe
//! (Windows) reports the whole block `null`, `present` included.
//!
//! Sources (macOS): `system_profiler SPDisplaysDataType -json` for name / cores /
//! Metal, `sysinfo` for the unified-memory size, and
//! `sudo -n powermetrics --samplers gpu_power -i 200 -n 1` for the live
//! utilisation (the "GPU HW active residency" line).
//!
//! Sources (Linux): the DRM render nodes under `/sys/class/drm` and their
//! kernel drivers decide `present` (a virtual or software device does not
//! count); `nvidia-smi` names an NVIDIA GPU, its memory and its utilisation.

#[cfg(target_os = "macos")]
use std::sync::Mutex;
use std::sync::OnceLock;
#[cfg(target_os = "macos")]
use std::time::{Duration, Instant};

use ados_mapping_protocol::compute::ComputeGpu;

/// Sample the host GPU: the cached static identity plus a freshly-sampled (short-
/// cached) utilisation.
pub fn sample() -> ComputeGpu {
    let mut gpu = identity();
    gpu.utilization_pct = utilization();
    gpu
}

/// The static GPU identity (name, cores, unified memory, Metal), resolved once
/// and cached for the process lifetime. Utilisation is intentionally NOT cached
/// here (it is live).
fn identity() -> ComputeGpu {
    static IDENTITY: OnceLock<ComputeGpu> = OnceLock::new();
    IDENTITY.get_or_init(probe_identity).clone()
}

#[cfg(target_os = "macos")]
fn probe_identity() -> ComputeGpu {
    let mut gpu = ComputeGpu {
        present: Some(true),
        unified_memory_mb: total_memory_mb(),
        ..ComputeGpu::default()
    };
    if let Some(display) = first_display_json() {
        gpu.name = display
            .get("sppci_model")
            .or_else(|| display.get("_name"))
            .and_then(|v| v.as_str())
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty());
        gpu.cores = display
            .get("sppci_cores")
            .and_then(|v| v.as_str())
            .and_then(|s| s.trim().parse::<u32>().ok());
        gpu.metal = display
            .get("spdisplays_metalfamily")
            .or_else(|| display.get("spdisplays_mtlgpufamilysupport"))
            .and_then(|v| v.as_str())
            .map(prettify_metal);
    }
    gpu
}

#[cfg(target_os = "linux")]
fn probe_identity() -> ComputeGpu {
    let nvidia = crate::worker::nvidia_gpu();
    let hardware = render_node_drivers(std::path::Path::new("/sys/class/drm"))
        .iter()
        .any(|driver| !VIRTUAL_DRM_DRIVERS.contains(&driver.as_str()));
    ComputeGpu {
        present: Some(nvidia.is_some() || hardware),
        name: nvidia.as_ref().map(|g| g.name.clone()),
        unified_memory_mb: nvidia.and_then(|g| g.vram_total_mb),
        ..ComputeGpu::default()
    }
}

#[cfg(not(any(target_os = "macos", target_os = "linux")))]
fn probe_identity() -> ComputeGpu {
    // No GPU identity source on this platform: the block stays all-null.
    ComputeGpu::default()
}

/// Kernel drivers of render nodes that are not a GPU a reconstruction can use:
/// virtual and software devices.
#[cfg(any(target_os = "linux", all(test, unix)))]
const VIRTUAL_DRM_DRIVERS: &[&str] = &[
    "vgem",
    "vkms",
    "virtio_gpu",
    "virtio-pci",
    "qxl",
    "bochs-drm",
    "simpledrm",
];

/// The kernel driver of every DRM render node (`renderD*`) under `drm_dir`,
/// read from its `device/driver` link. Empty when the directory is missing.
#[cfg(any(target_os = "linux", all(test, unix)))]
fn render_node_drivers(drm_dir: &std::path::Path) -> Vec<String> {
    let Ok(entries) = std::fs::read_dir(drm_dir) else {
        return Vec::new();
    };
    entries
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("renderD"))
        .filter_map(|e| std::fs::read_link(e.path().join("device/driver")).ok())
        .filter_map(|link| link.file_name().map(|n| n.to_string_lossy().into_owned()))
        .collect()
}

/// Total physical memory in MB — the unified memory the Apple-Silicon GPU shares
/// with the system. `None` when indeterminate.
#[cfg(target_os = "macos")]
fn total_memory_mb() -> Option<u64> {
    use sysinfo::System;
    let mut sys = System::new();
    sys.refresh_memory();
    let bytes = sys.total_memory();
    if bytes == 0 {
        None
    } else {
        Some(((bytes as f64) / (1024.0 * 1024.0)).round() as u64)
    }
}

/// Parse the first `SPDisplaysDataType` entry from `system_profiler … -json` into
/// a JSON object, or `None` on any spawn / parse / shape failure.
#[cfg(target_os = "macos")]
fn first_display_json() -> Option<serde_json::Map<String, serde_json::Value>> {
    let out = std::process::Command::new("system_profiler")
        .arg("SPDisplaysDataType")
        .arg("-json")
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).ok()?;
    v.get("SPDisplaysDataType")?
        .as_array()?
        .first()?
        .as_object()
        .cloned()
}

/// Turn a `system_profiler` Metal token into a friendly label: `spdisplays_metal3`
/// → `Metal 3`, `spdisplays_metal` → `Metal`. An unrecognised shape passes through
/// trimmed.
#[cfg(target_os = "macos")]
fn prettify_metal(raw: &str) -> String {
    let s = raw.trim().trim_start_matches("spdisplays_");
    if let Some(rest) = s.strip_prefix("metal") {
        let n = rest.trim();
        if n.is_empty() {
            "Metal".to_string()
        } else {
            format!("Metal {n}")
        }
    } else {
        raw.trim().to_string()
    }
}

/// A short-cached utilisation sample: the instant it was taken and the value
/// (which is itself optional — a sample can fail).
#[cfg(target_os = "macos")]
type UtilSample = Option<(Instant, Option<f32>)>;

/// A short-cached live GPU utilisation percentage. The cache (a ~1 s TTL) bounds
/// how often `powermetrics` is spawned under a rapid poll. `None` when not
/// sampleable (off macOS, or any sample failure).
#[cfg(target_os = "macos")]
fn utilization() -> Option<f32> {
    const TTL: Duration = Duration::from_millis(1000);
    static CACHE: OnceLock<Mutex<UtilSample>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(None));

    if let Ok(guard) = cache.lock() {
        if let Some((at, val)) = guard.as_ref() {
            if at.elapsed() < TTL {
                return *val;
            }
        }
    }
    let sampled = sample_gpu_residency();
    if let Ok(mut guard) = cache.lock() {
        *guard = Some((Instant::now(), sampled));
    }
    sampled
}

/// An NVIDIA GPU's utilisation from `nvidia-smi` (cached 5 s); `None` without one.
#[cfg(target_os = "linux")]
fn utilization() -> Option<f32> {
    crate::worker::nvidia_gpu().and_then(|g| g.util_pct)
}

#[cfg(not(any(target_os = "macos", target_os = "linux")))]
fn utilization() -> Option<f32> {
    None
}

/// Run `powermetrics` once and parse the GPU active-residency percentage.
/// `powermetrics` requires root; on the dev Mac passwordless sudo is configured,
/// so this uses `sudo -n` (non-interactive) and returns `None` if sudo is denied,
/// the tool is missing, or the residency line is absent — never a fabricated
/// value, never a blocking prompt.
#[cfg(target_os = "macos")]
fn sample_gpu_residency() -> Option<f32> {
    let out = std::process::Command::new("sudo")
        .args([
            "-n",
            "powermetrics",
            "--samplers",
            "gpu_power",
            "-i",
            "200",
            "-n",
            "1",
        ])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&out.stdout);
    parse_gpu_residency(&text)
}

/// Parse the "GPU HW active residency" percentage out of a `powermetrics
/// --samplers gpu_power` block. Returns the residency percentage (the first
/// number before `%` on that line), or `None` if the line is absent /
/// unparseable. Pure (platform-agnostic) so it is unit-tested on any host.
/// Only `sample_gpu_residency` (macOS) calls it in a non-test build, so off
/// macOS it is exercised solely by the tests — allow dead_code there.
#[cfg_attr(not(target_os = "macos"), allow(dead_code))]
fn parse_gpu_residency(text: &str) -> Option<f32> {
    for line in text.lines() {
        let lower = line.to_ascii_lowercase();
        if lower.contains("gpu") && lower.contains("active residency") {
            let after = line.split(':').nth(1)?;
            return parse_first_percent(after);
        }
    }
    None
}

/// Parse the first percentage in a string like `  12.50% (396 MHz: 12% …)`: the
/// number up to the first `%`. `None` when it does not parse. Reached only via
/// `parse_gpu_residency` (macOS) or the tests, so allow dead_code off macOS.
#[cfg_attr(not(target_os = "macos"), allow(dead_code))]
fn parse_first_percent(s: &str) -> Option<f32> {
    let pct = s.split('%').next()?.trim();
    pct.parse::<f32>().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_residency_line() {
        let block = "\
**** GPU usage ****
GPU HW active frequency: 396 MHz
GPU HW active residency:  12.50% (396 MHz: 12% 528 MHz:  0%)
GPU idle residency:  87.50%
GPU Power: 45 mW
";
        assert_eq!(parse_gpu_residency(block), Some(12.5));
    }

    #[test]
    fn residency_line_with_no_decimal() {
        let block = "GPU HW active residency: 100% (...)";
        assert_eq!(parse_gpu_residency(block), Some(100.0));
    }

    #[test]
    fn missing_residency_line_is_none() {
        let block = "GPU Power: 45 mW\nGPU idle residency: 87.50%";
        assert_eq!(parse_gpu_residency(block), None);
    }

    #[test]
    fn first_percent_parses_and_rejects_garbage() {
        assert_eq!(parse_first_percent(" 12.50% (extra)"), Some(12.5));
        assert_eq!(parse_first_percent("  0%"), Some(0.0));
        assert_eq!(parse_first_percent("n/a"), None);
    }

    #[test]
    fn sample_never_panics_and_is_self_consistent() {
        // Best-effort on every platform (util is null when powermetrics needs
        // sudo it lacks, or with no NVIDIA GPU). Either way it must not panic
        // and a present utilisation is a sane percentage.
        let gpu = sample();
        if let Some(u) = gpu.utilization_pct {
            assert!((0.0..=100.0).contains(&u), "utilisation in range: {u}");
        }
    }

    #[cfg(unix)]
    #[test]
    fn only_hardware_render_nodes_count_as_a_gpu() {
        use std::os::unix::fs::symlink;
        let dir = tempfile::tempdir().unwrap();
        let node = |name: &str, driver: &str| {
            let dev = dir.path().join(format!("dev-{name}"));
            std::fs::create_dir_all(&dev).unwrap();
            symlink(
                format!("../../bus/pci/drivers/{driver}"),
                dev.join("driver"),
            )
            .unwrap();
            std::fs::create_dir_all(dir.path().join(name)).unwrap();
            symlink(&dev, dir.path().join(name).join("device")).unwrap();
        };
        node("card0", "amdgpu");
        node("renderD128", "virtio_gpu");
        let drivers = render_node_drivers(dir.path());
        assert_eq!(drivers, ["virtio_gpu"], "only render nodes are read");
        assert!(drivers
            .iter()
            .all(|d| VIRTUAL_DRM_DRIVERS.contains(&d.as_str())));
        node("renderD129", "amdgpu");
        let mut drivers = render_node_drivers(dir.path());
        drivers.sort();
        assert_eq!(drivers, ["amdgpu", "virtio_gpu"]);
        assert!(drivers
            .iter()
            .any(|d| !VIRTUAL_DRM_DRIVERS.contains(&d.as_str())));
        assert!(render_node_drivers(&dir.path().join("missing")).is_empty());
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn prettifies_metal_tokens() {
        assert_eq!(prettify_metal("spdisplays_metal3"), "Metal 3");
        assert_eq!(prettify_metal("spdisplays_metal"), "Metal");
        assert_eq!(prettify_metal("Metal 3"), "Metal 3");
    }
}
