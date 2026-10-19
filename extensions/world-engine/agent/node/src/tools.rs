//! The pinned third-party tool set and where a node finds it.
//!
//! `tools/tool-pins.json` (embedded at build time) names every program the
//! pipeline drives, its exact version and, per platform, the archive the fetch
//! scripts install. A node resolves a tool only from that pinned install
//! (`<tools_root>/<name>/<version>/<bin>`) or an explicit `ADOS_TOOL_<NAME>`
//! override: never from whatever happens to be on `PATH`, so a job always runs
//! the version the pipeline was written against.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::LazyLock;

use serde::Deserialize;

const PINS_JSON: &str = include_str!("../tools/tool-pins.json");

/// How a tool is installed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ToolKind {
    /// A release archive extracted under the tools root.
    Archive,
    /// An npm package installed with `--prefix` under the tools root.
    Npm,
    /// A uv project shipped with the node.
    Uv,
    /// A container image run through docker.
    Docker,
    /// A Rust binary built with `cargo install --root` under the tools root.
    Cargo,
}

/// One platform's download.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct PlatformPin {
    #[serde(default)]
    pub url: Option<String>,
    #[serde(default)]
    pub sha256: Option<String>,
    #[serde(default)]
    pub archive: Option<String>,
    #[serde(default)]
    pub bin: Option<String>,
}

/// A tool's variant inside the WSL distribution, used on Windows hosts.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct WslPin {
    /// Linux interpreter of a prebuilt environment for a uv project.
    #[serde(default)]
    pub python: Option<String>,
}

/// One pinned tool.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ToolPin {
    #[serde(skip)]
    pub name: String,
    pub version: String,
    #[serde(default)]
    pub license: Option<String>,
    pub kind: ToolKind,
    #[serde(default)]
    pub probe: Vec<String>,
    #[serde(default)]
    pub platforms: BTreeMap<String, PlatformPin>,
    #[serde(default)]
    pub package: Option<String>,
    #[serde(default)]
    pub bin: Option<String>,
    #[serde(default)]
    pub project: Option<String>,
    #[serde(default)]
    pub image: Option<String>,
    #[serde(default)]
    pub wsl: Option<WslPin>,
}

impl ToolPin {
    /// The executable path relative to the install dir on this platform.
    pub fn bin_for(&self, platform: &str) -> Option<&str> {
        self.platforms
            .get(platform)
            .and_then(|p| p.bin.as_deref())
            .or(self.bin.as_deref())
    }
}

/// The whole pinned set.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ToolPins {
    pub schema: u32,
    pub tools: BTreeMap<String, ToolPin>,
}

impl ToolPins {
    /// Parse a pins document.
    pub fn parse(json: &str) -> Result<Self, serde_json::Error> {
        let mut pins: ToolPins = serde_json::from_str(json)?;
        for (name, pin) in pins.tools.iter_mut() {
            pin.name = name.clone();
        }
        Ok(pins)
    }

    pub fn get(&self, name: &str) -> Option<&ToolPin> {
        self.tools.get(name)
    }

    /// `{name: version}` for every pinned tool, as surfaced on a job's `tools`.
    pub fn versions(&self) -> BTreeMap<String, String> {
        self.tools
            .iter()
            .map(|(k, v)| (k.clone(), v.version.clone()))
            .collect()
    }
}

static PINS: LazyLock<ToolPins> =
    LazyLock::new(|| ToolPins::parse(PINS_JSON).expect("tools/tool-pins.json is valid"));

/// The pins embedded in this build.
pub fn pins() -> &'static ToolPins {
    &PINS
}

/// This build's platform key in the pins (`x86_64-windows`, `aarch64-linux`, ...).
pub fn platform_key() -> String {
    let os = match std::env::consts::OS {
        "macos" => "macos",
        "windows" => "windows",
        _ => "linux",
    };
    format!("{}-{os}", std::env::consts::ARCH)
}

/// Where the node's own assets (the `tools/` dir, the python projects, the
/// tour viewer) live: `ADOS_WE_NODE_ASSETS` when set; else the packed layout
/// beside the binary (`<exe>/../../agent/node`); else this source checkout.
pub fn assets_root() -> PathBuf {
    if let Some(p) = env_path("ADOS_WE_NODE_ASSETS") {
        return p;
    }
    if let Some(dir) = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(Path::to_path_buf))
    {
        let packed = dir.join("../../agent/node");
        if packed.join("tools").is_dir() {
            return packed;
        }
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Root of installed tools: `ADOS_WE_TOOLS_DIR` when set, else
/// `<assets>/tools/cache` (what the fetch scripts populate).
pub fn tools_root() -> PathBuf {
    env_path("ADOS_WE_TOOLS_DIR").unwrap_or_else(|| assets_root().join("tools").join("cache"))
}

/// The built static tour viewer the tour bundle copies (`gcs/tour-viewer/`,
/// shipped in the extension archive): `ADOS_WE_TOUR_VIEWER` when set, else
/// the packed `<exe>/../../gcs/tour-viewer`, else this checkout's GCS build.
pub fn tour_viewer_dir() -> PathBuf {
    if let Some(p) = env_path("ADOS_WE_TOUR_VIEWER") {
        return p;
    }
    if let Some(dir) = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(Path::to_path_buf))
    {
        let packed = dir.join("../../gcs/tour-viewer");
        if packed.join("index.html").is_file() {
            return packed;
        }
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../gcs/tour-viewer")
}

fn env_path(key: &str) -> Option<PathBuf> {
    std::env::var_os(key)
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
}

/// The env var that overrides a tool's executable (`ADOS_TOOL_SPLAT_TRANSFORM`).
pub fn override_var(tool: &str) -> String {
    let up: String = tool
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_uppercase()
            } else {
                '_'
            }
        })
        .collect();
    format!("ADOS_TOOL_{up}")
}

/// Map a host path to the path a container sees it at through `-v`. On
/// Windows the containers run in WSL, where `D:\a\b` is `/mnt/d/a/b`.
pub fn container_path(host: &Path) -> String {
    if cfg!(windows) {
        wsl_path(&host.to_string_lossy())
    } else {
        host.to_string_lossy().into_owned()
    }
}

/// `path` made absolute against the current directory (no symlink
/// resolution, so the directory need not exist), with the `\\?\` verbatim
/// prefix removed: `\\?\D:\a` becomes `D:\a`, `\\?\UNC\srv\s` becomes
/// `\\srv\s`.
pub fn absolute_path(path: &Path) -> std::io::Result<PathBuf> {
    let abs = std::path::absolute(path)?;
    let s = abs.to_string_lossy();
    if let Some(rest) = s.strip_prefix(r"\\?\UNC\") {
        return Ok(PathBuf::from(format!(r"\\{rest}")));
    }
    match s.strip_prefix(r"\\?\") {
        Some(rest) if drive_prefix_len(rest).is_some() => Ok(PathBuf::from(rest)),
        _ => Ok(abs),
    }
}

/// Whether `s` starts with a Windows network-share or device path
/// (`\\server\share`, `//server/share`, `\\?\UNC\...`, `\\.\pipe\...`),
/// which WSL programs cannot open. The verbatim and device forms of a drive
/// path (`\\?\D:\a`, `\\.\D:\a`) are local and do not count.
pub fn is_network_path(s: &str) -> bool {
    let sep = |c: u8| c == b'\\' || c == b'/';
    let b = s.as_bytes();
    if b.len() < 3 || !sep(b[0]) || !sep(b[1]) || sep(b[2]) {
        return false;
    }
    if matches!(b[2], b'?' | b'.') && b.get(3).copied().is_some_and(sep) {
        let rest = s[4..].replace('/', "\\");
        return drive_prefix_len(&rest).is_none();
    }
    true
}

/// `D:\a\b` (or `D:/a/b`, or the `\\?\D:\a` verbatim form) → `/mnt/d/a/b`.
/// A network-share path is returned unchanged (see [`is_network_path`]).
pub fn wsl_path(p: &str) -> String {
    if is_network_path(p) {
        return p.to_string();
    }
    let p = p.strip_prefix(r"\\?\").unwrap_or(p).replace('\\', "/");
    let b = p.as_bytes();
    if b.len() >= 2 && b[1] == b':' && b[0].is_ascii_alphabetic() {
        let drive = (b[0] as char).to_ascii_lowercase();
        let rest = p[2..].trim_start_matches('/');
        if rest.is_empty() {
            return format!("/mnt/{drive}");
        }
        return format!("/mnt/{drive}/{rest}");
    }
    p
}

/// Length of the drive prefix (`D:` or `\\?\D:`) when `s` starts with an
/// absolute Windows drive path.
fn drive_prefix_len(s: &str) -> Option<usize> {
    let (skip, rest) = match s.strip_prefix(r"\\?\") {
        Some(r) => (4, r),
        None => (0, s),
    };
    let b = rest.as_bytes();
    (b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'\\' | b'/'))
        .then_some(skip + 2)
}

/// One argument of a program run inside WSL. An absolute Windows path is
/// mapped with [`wsl_path`] when it is the whole argument, follows the first
/// `=` (`--out=D:\x`, `K=D:\x`), or precedes a `:` suffix (`D:\x:/data`);
/// anything else passes unchanged.
pub fn wsl_arg(a: &str) -> String {
    if let Some(n) = drive_prefix_len(a) {
        return match a[n..].find(':') {
            Some(i) => format!("{}{}", wsl_path(&a[..n + i]), &a[n + i..]),
            None => wsl_path(a),
        };
    }
    match a.split_once('=') {
        Some((k, v)) if drive_prefix_len(v).is_some() => format!("{k}={}", wsl_arg(v)),
        _ => a.to_string(),
    }
}

/// The WSL distribution that runs the Linux-only tools (docker, the gsplat
/// environment) on a Windows node.
pub const WSL_DISTRO: &str = "Ubuntu-24.04";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_pins_parse() {
        let p = pins();
        assert_eq!(p.schema, 1);
        for (name, pin) in &p.tools {
            assert_eq!(&pin.name, name);
            assert!(!pin.version.is_empty(), "{name} has a version");
        }
    }

    #[test]
    fn parses_every_kind_and_platform_bin() {
        let json = r#"{"schema":1,"tools":{
          "colmap":{"version":"4.2.0","kind":"archive","probe":["-h"],
            "platforms":{"x86_64-windows":{"url":"u","sha256":"s","archive":"zip","bin":"c/bin/colmap.exe"}}},
          "splat-transform":{"version":"1.0.0","kind":"npm","package":"p","bin":"node_modules/.bin/splat-transform"},
          "gsplat_train":{"version":"0.1.0","kind":"uv","project":"python/gsplat_train",
            "wsl":{"python":"/opt/v/bin/python"}},
          "kalibr":{"version":"1","kind":"docker","image":"img@sha256:ab"}}}"#;
        let p = ToolPins::parse(json).unwrap();
        assert_eq!(
            p.get("colmap").unwrap().bin_for("x86_64-windows"),
            Some("c/bin/colmap.exe")
        );
        assert_eq!(p.get("colmap").unwrap().bin_for("x86_64-linux"), None);
        assert_eq!(
            p.get("splat-transform").unwrap().bin_for("x86_64-linux"),
            Some("node_modules/.bin/splat-transform")
        );
        assert_eq!(p.get("kalibr").unwrap().kind, ToolKind::Docker);
        assert_eq!(p.versions()["gsplat_train"], "0.1.0");
        let wsl = p.get("gsplat_train").unwrap().wsl.as_ref().unwrap();
        assert_eq!(wsl.python.as_deref(), Some("/opt/v/bin/python"));
        assert_eq!(p.get("kalibr").unwrap().wsl, None);
    }

    #[test]
    fn wsl_paths_map_drives() {
        assert_eq!(wsl_path(r"D:\Github\x y\z"), "/mnt/d/Github/x y/z");
        assert_eq!(wsl_path("C:/a"), "/mnt/c/a");
        assert_eq!(wsl_path(r"\\?\E:\w"), "/mnt/e/w");
        assert_eq!(wsl_path("/already/unix"), "/already/unix");
    }

    #[test]
    fn network_paths_are_recognised_and_left_unmapped() {
        for p in [
            r"\\srv\share\job",
            "//srv/share/job",
            r"\\?\UNC\srv\share",
            r"\\.\pipe\x",
            r"\\srv\share\job:/data",
        ] {
            assert!(is_network_path(p), "{p}");
            assert_eq!(wsl_path(p), p);
        }
        for p in [
            r"\\?\D:\a",
            r"\\.\D:\a",
            r"D:\a",
            "/mnt/d/a",
            "///x",
            "",
            r"\\",
        ] {
            assert!(!is_network_path(p), "{p}");
        }
    }

    #[test]
    fn absolute_paths_drop_the_verbatim_prefix() {
        let rel = absolute_path(Path::new("data")).unwrap();
        assert!(rel.is_absolute());
        assert_eq!(rel, std::env::current_dir().unwrap().join("data"));
        #[cfg(windows)]
        {
            assert_eq!(
                absolute_path(Path::new(r"\\?\D:\x\y")).unwrap(),
                Path::new(r"D:\x\y")
            );
            assert_eq!(
                absolute_path(Path::new(r"\\?\UNC\srv\s\x")).unwrap(),
                Path::new(r"\\srv\s\x")
            );
            assert_eq!(
                absolute_path(Path::new("D:/x")).unwrap(),
                Path::new(r"D:\x")
            );
        }
    }

    #[test]
    fn wsl_args_map_embedded_windows_paths() {
        // Whole-argument paths: spaces, unicode, drive letters, verbatim form.
        assert_eq!(wsl_arg(r"D:\x y\z.png"), "/mnt/d/x y/z.png");
        assert_eq!(wsl_arg(r"D:\Daten\Übung été\x"), "/mnt/d/Daten/Übung été/x");
        assert_eq!(wsl_arg(r"c:\A\B"), "/mnt/c/A/B");
        assert_eq!(wsl_arg("Z:/"), "/mnt/z");
        assert_eq!(wsl_arg(r"\\?\D:\long\p"), "/mnt/d/long/p");
        // After `=`, and before a `:` suffix (a volume spec).
        assert_eq!(wsl_arg(r"--x=D:\y"), "--x=/mnt/d/y");
        assert_eq!(wsl_arg(r"config_path:=D:\c fg"), "config_path:=/mnt/d/c fg");
        assert_eq!(wsl_arg(r"D:\x:/data"), "/mnt/d/x:/data");
        assert_eq!(wsl_arg(r"D:\x:/data:ro"), "/mnt/d/x:/data:ro");
        assert_eq!(wsl_arg(r"\\?\D:\x:/data"), "/mnt/d/x:/data");
        assert_eq!(
            wsl_arg(r"--volume=D:\w é:/data"),
            "--volume=/mnt/d/w é:/data"
        );
        // Linux paths, other arguments and shell metacharacters pass unchanged.
        for a in [
            "/mnt/d/x:/data",
            "/opt/ados-recon/gsplat-venv/bin/python",
            "local/kalibr:1f60227442d2",
            "img@sha256:ab",
            "D:",
            "D:rel",
            r"\\?\UNC\srv\share",
            r"\\srv\share",
            "$HOME/*.png",
            "{{.Server.Version}}",
            "source /a && rosrun k x --folder /data/k",
            "a=b",
            "",
        ] {
            assert_eq!(wsl_arg(a), a);
        }
    }

    #[test]
    fn override_var_is_shell_safe() {
        assert_eq!(override_var("splat-transform"), "ADOS_TOOL_SPLAT_TRANSFORM");
        assert_eq!(override_var("gsplat_train"), "ADOS_TOOL_GSPLAT_TRAIN");
    }

    #[test]
    fn platform_key_has_arch_and_os() {
        let k = platform_key();
        assert!(k.contains('-'));
    }
}
