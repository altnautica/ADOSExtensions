//! The real [`ToolRunner`]: resolves pinned tools and spawns them.

use std::collections::VecDeque;
use std::ffi::OsString;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::sync::Mutex;

use super::{ResolvedTool, StageError, ToolInvocation, ToolRunner};
use crate::tools::{self, ToolKind, ToolPin};

/// Output lines kept for a failure message.
const TAIL_LINES: usize = 40;

/// Spawns the pinned tools.
pub struct ProcessRunner {
    tools_root: PathBuf,
    assets: PathBuf,
    docker: Mutex<Option<Result<ResolvedTool, StageError>>>,
}

impl Default for ProcessRunner {
    fn default() -> Self {
        Self::new(tools::tools_root(), tools::assets_root())
    }
}

impl ProcessRunner {
    pub fn new(tools_root: PathBuf, assets: PathBuf) -> Self {
        Self {
            tools_root,
            assets,
            docker: Mutex::new(None),
        }
    }

    fn resolve_pin(&self, pin: &ToolPin) -> Result<ResolvedTool, StageError> {
        let dir = self.tools_root.join(&pin.name).join(&pin.version);
        match pin.kind {
            ToolKind::Archive | ToolKind::Npm | ToolKind::Cargo => {
                let platform = tools::platform_key();
                let bin = pin.bin_for(&platform).ok_or_else(|| {
                    StageError::tool_missing(
                        &pin.name,
                        format!("{} {} has no build for {platform}", pin.name, pin.version),
                    )
                })?;
                let program = executable(&dir.join(bin)).ok_or_else(|| {
                    StageError::tool_missing(
                        &pin.name,
                        format!(
                            "{} {} is not installed under {} (run node/tools/fetch-tools)",
                            pin.name,
                            pin.version,
                            dir.display()
                        ),
                    )
                })?;
                Ok(ResolvedTool {
                    name: pin.name.clone(),
                    version: pin.version.clone(),
                    program,
                    prefix_args: Vec::new(),
                    home: dir,
                })
            }
            ToolKind::Uv => {
                let project = self.assets.join(pin.project.as_deref().unwrap_or("python"));
                if !project.join(".venv").is_dir() {
                    return Err(StageError::tool_missing(
                        &pin.name,
                        format!(
                            "the {} environment is not synced at {} (run node/tools/fetch-tools)",
                            pin.name,
                            project.display()
                        ),
                    ));
                }
                let uv = find_uv().ok_or_else(|| StageError::tool_missing("uv", "uv not found"))?;
                Ok(ResolvedTool {
                    name: pin.name.clone(),
                    version: pin.version.clone(),
                    program: uv,
                    prefix_args: ["run", "--frozen", "--project"]
                        .into_iter()
                        .map(OsString::from)
                        .chain([project.clone().into_os_string(), "python".into()])
                        .collect(),
                    home: project,
                })
            }
            ToolKind::Docker => {
                let docker = self.resolve_docker()?;
                Ok(ResolvedTool {
                    name: pin.name.clone(),
                    version: pin.version.clone(),
                    home: PathBuf::new(),
                    ..docker
                })
            }
        }
    }

    /// docker on this host (through WSL on Windows), probed once.
    fn resolve_docker(&self) -> Result<ResolvedTool, StageError> {
        let mut cached = self.docker.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(r) = cached.as_ref() {
            return r.clone();
        }
        let (program, prefix): (PathBuf, Vec<OsString>) = if cfg!(windows) {
            (
                PathBuf::from("wsl.exe"),
                vec![
                    "-d".into(),
                    tools::WSL_DISTRO.into(),
                    "--".into(),
                    "docker".into(),
                ],
            )
        } else {
            (PathBuf::from("docker"), Vec::new())
        };
        let ok = Command::new(&program)
            .args(&prefix)
            .args(["version", "--format", "{{.Server.Version}}"])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .output()
            .ok()
            .filter(|o| o.status.success());
        let r = match ok {
            Some(o) => Ok(ResolvedTool {
                name: "docker".into(),
                version: String::from_utf8_lossy(&o.stdout).trim().to_string(),
                program,
                prefix_args: prefix,
                home: PathBuf::new(),
            }),
            None => Err(StageError::tool_missing(
                "docker",
                if cfg!(windows) {
                    format!("docker is not available in WSL {}", tools::WSL_DISTRO)
                } else {
                    "docker is not available".to_string()
                },
            )),
        };
        *cached = Some(r.clone());
        r
    }
}

/// `path`, or `path.exe` / `path.cmd` / `path.bat` on Windows, when it is a file.
fn executable(path: &Path) -> Option<PathBuf> {
    if path.is_file() {
        return Some(path.to_path_buf());
    }
    if cfg!(windows) {
        for ext in ["exe", "cmd", "bat"] {
            let p = path.with_extension(ext);
            if p.is_file() {
                return Some(p);
            }
        }
    }
    None
}

/// uv: `ADOS_UV`, else `~/.local/bin/uv`, else `uv` on `PATH`.
fn find_uv() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("ADOS_UV").filter(|v| !v.is_empty()) {
        return Some(PathBuf::from(p));
    }
    let home = std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" })?;
    let local = executable(&Path::new(&home).join(".local").join("bin").join("uv"));
    local.or_else(|| {
        let paths = std::env::var_os("PATH")?;
        std::env::split_paths(&paths).find_map(|d| executable(&d.join("uv")))
    })
}

impl ToolRunner for ProcessRunner {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        if let Some(p) = std::env::var_os(tools::override_var(tool)).filter(|v| !v.is_empty()) {
            return Ok(ResolvedTool {
                name: tool.to_string(),
                version: "override".into(),
                home: Path::new(&p)
                    .parent()
                    .map(Path::to_path_buf)
                    .unwrap_or_default(),
                program: PathBuf::from(p),
                prefix_args: Vec::new(),
            });
        }
        if tool == "docker" {
            return self.resolve_docker();
        }
        match tools::pins().get(tool) {
            Some(pin) => self.resolve_pin(pin),
            None => Err(StageError::tool_missing(
                tool,
                format!("{tool} is not a pinned tool"),
            )),
        }
    }

    fn run(&self, inv: &ToolInvocation, on_line: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        let tool = self.resolve(&inv.tool)?;
        let mut cmd = Command::new(&tool.program);
        cmd.args(&tool.prefix_args)
            .args(&inv.args)
            .current_dir(&inv.cwd)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        for (k, v) in &inv.env {
            cmd.env(k, v);
        }
        tracing::info!(tool = %inv.tool, args = ?inv.args_lossy(), "tool start");
        let mut child = cmd.spawn().map_err(|e| {
            StageError::tool_missing(&inv.tool, format!("spawn {}: {e}", tool.program.display()))
        })?;
        // Both streams feed one channel so lines arrive in roughly the order
        // the tool wrote them, without either pipe filling and blocking it.
        let (tx, rx) = mpsc::channel::<String>();
        let mut readers = Vec::new();
        if let Some(out) = child.stdout.take() {
            let tx = tx.clone();
            readers.push(std::thread::spawn(move || {
                for line in BufReader::new(out).lines().map_while(Result::ok) {
                    let _ = tx.send(line);
                }
            }));
        }
        if let Some(err) = child.stderr.take() {
            let tx = tx.clone();
            readers.push(std::thread::spawn(move || {
                for line in BufReader::new(err).lines().map_while(Result::ok) {
                    let _ = tx.send(line);
                }
            }));
        }
        drop(tx);
        let mut tail: VecDeque<String> = VecDeque::with_capacity(TAIL_LINES);
        for line in rx {
            on_line(&line);
            if tail.len() == TAIL_LINES {
                tail.pop_front();
            }
            tail.push_back(line);
        }
        for r in readers {
            let _ = r.join();
        }
        let status = child.wait().map_err(StageError::io)?;
        if status.success() {
            Ok(())
        } else {
            let tail: Vec<String> = tail.into_iter().collect();
            Err(StageError::tool_failed(
                &inv.tool,
                format!("{} exited {status}: {}", inv.tool, tail.join("\n")),
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unpinned_or_uninstalled_tool_is_tool_missing() {
        let dir = tempfile::tempdir().unwrap();
        let r = ProcessRunner::new(dir.path().to_path_buf(), dir.path().to_path_buf());
        let e = r.resolve("not-a-tool").unwrap_err();
        assert_eq!(e.code, "E_TOOL_MISSING");
        assert_eq!(e.tool.as_deref(), Some("not-a-tool"));
        for (name, pin) in &tools::pins().tools {
            if matches!(
                pin.kind,
                ToolKind::Archive | ToolKind::Npm | ToolKind::Cargo | ToolKind::Uv
            ) {
                let e = r.resolve(name).unwrap_err();
                assert_eq!(e.code, "E_TOOL_MISSING", "{name}");
            }
        }
    }

    #[test]
    fn an_overridden_tool_runs_and_a_failure_carries_its_output() {
        let dir = tempfile::tempdir().unwrap();
        // A tool resolved through its override env var runs end to end.
        #[cfg(windows)]
        let script = {
            let p = dir.path().join("echo-tool.cmd");
            std::fs::write(
                &p,
                "@echo line-one\r\n@echo line-two 1>&2\r\n@exit /b 3\r\n",
            )
            .unwrap();
            p
        };
        #[cfg(not(windows))]
        let script = {
            use std::os::unix::fs::PermissionsExt;
            let p = dir.path().join("echo-tool");
            std::fs::write(&p, "#!/bin/sh\necho line-one\necho line-two >&2\nexit 3\n").unwrap();
            std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
            p
        };
        std::env::set_var(tools::override_var("echo-test-tool"), &script);
        let r = ProcessRunner::new(dir.path().to_path_buf(), dir.path().to_path_buf());
        let mut lines = Vec::new();
        let e = r
            .run(
                &ToolInvocation::new("echo-test-tool", dir.path()),
                &mut |l| lines.push(l.to_string()),
            )
            .unwrap_err();
        assert_eq!(e.code, "E_TOOL_FAILED");
        assert!(lines.iter().any(|l| l.contains("line-one")));
        assert!(lines.iter().any(|l| l.contains("line-two")));
        assert!(e.message.contains("line-two"));
    }
}
