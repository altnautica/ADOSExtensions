//! Finding the interpreter that runs `kiss_icp_live`.
//!
//! A release node runs the prebuilt KISS-ICP runtime the install unpacked
//! into the plugin's tree (`navigation.lidar.runtime`, by default
//! `lib/<arch>-linux/kiss-icp-runtime`): a relocatable CPython whose own
//! site-packages hold the wheels the project's `uv.lock` pins
//! (`kiss-runtime/build-runtime.sh`). It needs no sync and no network and is
//! never written to; the child runs `-s -E -B`, so neither the user site,
//! nor `PYTHON*` variables, nor a bytecode write reach it. The plugin sandbox
//! executes only from the install tree and the system library dirs, which is
//! why the runtime lives there and not in the data dir.
//!
//! Before it is used the tree is checked ([`check_tree`]): the runtime dir,
//! everything under it and every directory above it must be owned by root or
//! by this process's user and writable by neither group nor others (a sticky
//! directory such as `/tmp` excepted above the runtime), and nothing under it
//! may be a symlink or a special file. A tree that fails is refused, not run.
//!
//! Without a runtime (a developer checkout) the provider falls back to
//! `navigation.lidar.uv`: `uv sync` builds the environment in the data dir
//! ([`super::setup`]). That fallback needs the package index and a data dir
//! the process may execute from, which a sandboxed node has neither of; the
//! status names it ([`Resolved::note`]).

use std::os::unix::fs::{MetadataExt, PermissionsExt};
use std::path::{Path, PathBuf};

use super::Launcher;

/// The interpreter inside a runtime dir.
pub const RUNTIME_PYTHON: &str = "bin/python3";

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RuntimeError {
    #[error("KISS-ICP runtime refused: {path}: {why}")]
    Unsafe { path: PathBuf, why: String },
    #[error("no KISS-ICP runtime at {runtime} and no uv `{uv}` for the developer fallback")]
    Missing { runtime: PathBuf, uv: String },
}

/// The launcher a provider starts, and why it is not the prebuilt runtime.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolved {
    pub launcher: Launcher,
    /// Set when the launcher is the developer fallback: why, for the status.
    pub note: Option<String>,
}

/// Where the child's code comes from.
#[derive(Debug, Clone, Copy)]
pub struct Sources<'a> {
    /// The prebuilt runtime dir.
    pub runtime: &'a Path,
    /// The `kiss_icp_live` project the child runs (its cwd).
    pub project: &'a Path,
    /// The plugin's install dir, which both must lie inside.
    pub install: &'a Path,
}

/// `path` itself, refused unless it resolves inside `install` without
/// passing through a symlink (its canonical form equals its absolute form).
fn contained(path: &Path, install: &Path) -> Result<PathBuf, RuntimeError> {
    let refuse = |why: String| RuntimeError::Unsafe {
        path: path.to_path_buf(),
        why,
    };
    let abs = std::path::absolute(path).map_err(|e| refuse(e.to_string()))?;
    let canon = abs.canonicalize().map_err(|e| refuse(e.to_string()))?;
    if canon != abs {
        return Err(refuse(format!(
            "reached through a symlink (resolves to {})",
            canon.display()
        )));
    }
    let install = install.canonicalize().map_err(|e| refuse(e.to_string()))?;
    if !canon.starts_with(&install) {
        return Err(refuse(format!(
            "outside the install dir {}",
            install.display()
        )));
    }
    Ok(canon)
}

/// The prebuilt runtime when it is there and safe, its project with it; the
/// uv fallback (`uv`: a path, or a bare name looked up on `PATH`) when there
/// is no runtime at all.
pub fn resolve(src: Sources<'_>, uv: &Path) -> Result<Resolved, RuntimeError> {
    let runtime = src.runtime;
    if runtime.symlink_metadata().is_ok() {
        let unsafe_at = |(path, why): (PathBuf, String)| RuntimeError::Unsafe { path, why };
        let runtime = contained(runtime, src.install)?;
        check_tree(&runtime, euid()).map_err(unsafe_at)?;
        // The project's code runs in the same child, so it is held to the
        // same rules.
        let project = contained(src.project, src.install)?;
        check_tree(&project, euid()).map_err(unsafe_at)?;
        let python = runtime.join(RUNTIME_PYTHON);
        let meta = python
            .symlink_metadata()
            .map_err(|e| RuntimeError::Unsafe {
                path: python.clone(),
                why: format!("no interpreter: {e}"),
            })?;
        if !meta.is_file() || meta.permissions().mode() & 0o111 == 0 {
            return Err(RuntimeError::Unsafe {
                path: python,
                why: "the interpreter is not an executable file".into(),
            });
        }
        return Ok(Resolved {
            launcher: Launcher::Python(python),
            note: None,
        });
    }
    let missing = || RuntimeError::Missing {
        runtime: runtime.to_path_buf(),
        uv: uv.display().to_string(),
    };
    let uv = find_program(uv).ok_or_else(missing)?;
    Ok(Resolved {
        note: Some(format!(
            "no prebuilt KISS-ICP runtime at {}; developer fallback: uv sync into the data dir \
             (needs the package index and an executable data dir)",
            runtime.display()
        )),
        launcher: Launcher::Uv(uv),
    })
}

/// This process's effective uid, read off `/proc/self` (owned by it); `None`
/// when unreadable, which leaves root the only trusted owner.
fn euid() -> Option<u32> {
    std::fs::metadata("/proc/self").ok().map(|m| m.uid())
}

/// Why `path` (described by `meta`) may not hold runnable code: owned by
/// someone other than root or `euid`, or writable by group or others (unless
/// `sticky_ok` and the sticky bit is set).
fn unsafe_entry(meta: &std::fs::Metadata, euid: Option<u32>, sticky_ok: bool) -> Option<String> {
    let uid = meta.uid();
    if uid != 0 && Some(uid) != euid {
        return Some(format!("owned by uid {uid}"));
    }
    let mode = meta.permissions().mode();
    if mode & 0o022 != 0 && !(sticky_ok && mode & 0o1000 != 0) {
        return Some(format!(
            "writable by group or others (mode {:o})",
            mode & 0o7777
        ));
    }
    None
}

/// Check the runtime tree and every directory above it; the first offending
/// path and why.
pub fn check_tree(root: &Path, euid: Option<u32>) -> Result<(), (PathBuf, String)> {
    let root = std::path::absolute(root).map_err(|e| (root.to_path_buf(), e.to_string()))?;
    for dir in root.ancestors().skip(1) {
        let meta = std::fs::metadata(dir).map_err(|e| (dir.to_path_buf(), e.to_string()))?;
        if let Some(why) = unsafe_entry(&meta, euid, true) {
            return Err((dir.to_path_buf(), why));
        }
    }
    let mut stack = vec![root];
    while let Some(path) = stack.pop() {
        let meta = path
            .symlink_metadata()
            .map_err(|e| (path.clone(), e.to_string()))?;
        let kind = meta.file_type();
        if kind.is_symlink() {
            return Err((path, "is a symlink".into()));
        }
        if !kind.is_dir() && !kind.is_file() {
            return Err((path, "is not a regular file or directory".into()));
        }
        if let Some(why) = unsafe_entry(&meta, euid, false) {
            return Err((path, why));
        }
        if kind.is_dir() {
            let entries = std::fs::read_dir(&path).map_err(|e| (path.clone(), e.to_string()))?;
            for entry in entries {
                stack.push(entry.map_err(|e| (path.clone(), e.to_string()))?.path());
            }
        }
    }
    Ok(())
}

/// `program` itself when it names a path, else the first executable file of
/// that name on `PATH`.
fn find_program(program: &Path) -> Option<PathBuf> {
    let runnable = |p: &Path| {
        p.metadata()
            .is_ok_and(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
    };
    if program.components().count() > 1 || program.is_absolute() {
        return runnable(program).then(|| program.to_path_buf());
    }
    std::env::split_paths(&std::env::var_os("PATH")?)
        .map(|dir| dir.join(program))
        .find(|p| runnable(p))
}
