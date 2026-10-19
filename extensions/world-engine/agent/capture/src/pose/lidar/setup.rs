//! Provisioning the `kiss_icp_live` environment on the device.
//!
//! The plugin host has no install hook and the plugin tree is read-only at
//! run time, so the capture service builds the environment itself, in its
//! writable data dir, the first time the LiDAR provider starts:
//!
//! ```text
//! uv sync --project <project_dir> --frozen --no-dev
//!   UV_PROJECT_ENVIRONMENT=<env_dir>/venv   the environment
//!   UV_CACHE_DIR=<env_dir>/cache            wheel cache
//!   UV_PYTHON_INSTALL_DIR=<env_dir>/python  a managed interpreter, if needed
//!   UV_NO_CONFIG=1                          no user or system uv config
//! ```
//!
//! `--frozen` installs exactly the versions in the shipped `uv.lock` and never
//! writes to the project. After a successful sync `<env_dir>/sync.stamp`
//! records a fingerprint of `pyproject.toml` and `uv.lock`; while it matches
//! and the interpreter exists, later starts skip the sync. A plugin update
//! that changes the lock re-syncs. The stamp is removed before a sync starts,
//! so an interrupted sync is redone. The first sync needs network access to
//! the package index.

use std::path::{Path, PathBuf};
use std::process::{ExitStatus, Stdio};

use tokio::process::Command;

pub const VENV_DIR: &str = "venv";
pub const CACHE_DIR: &str = "cache";
pub const PYTHON_DIR: &str = "python";
pub const STAMP_FILE: &str = "sync.stamp";
/// Files of the project whose change requires a re-sync.
const FINGERPRINTED: [&str; 2] = ["pyproject.toml", "uv.lock"];
/// Output lines of a failed sync kept for the error.
const TAIL_LINES: usize = 12;

#[derive(Debug, thiserror::Error)]
pub enum SetupError {
    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("cannot run uv `{path}`: {source}")]
    Spawn {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("uv sync failed ({status}): {tail}")]
    Sync { status: ExitStatus, tail: String },
    #[error("uv sync left no interpreter at {0}")]
    NoPython(PathBuf),
}

fn io(path: &Path) -> impl FnOnce(std::io::Error) -> SetupError + '_ {
    move |source| SetupError::Io {
        path: path.to_path_buf(),
        source,
    }
}

/// The environment's interpreter.
pub fn env_python(env_dir: &Path) -> PathBuf {
    env_dir.join(VENV_DIR).join("bin").join("python")
}

/// FNV-1a 64 over the fingerprinted files, each preceded by its name and
/// length (change detection only; the lock itself pins hashes).
pub async fn fingerprint(project_dir: &Path) -> Result<String, SetupError> {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for name in FINGERPRINTED {
        let path = project_dir.join(name);
        let body = tokio::fs::read(&path).await.map_err(io(&path))?;
        feed(name.as_bytes());
        feed(&(body.len() as u64).to_le_bytes());
        feed(&body);
    }
    Ok(format!("fnv1a64:{h:016x}\n"))
}

/// The interpreter of an environment synced from `project_dir`, syncing it
/// first when it is missing or stale.
pub async fn ensure_env(
    uv: &Path,
    project_dir: &Path,
    env_dir: &Path,
) -> Result<PathBuf, SetupError> {
    let want = fingerprint(project_dir).await?;
    let stamp = env_dir.join(STAMP_FILE);
    let python = env_python(env_dir);
    let have = tokio::fs::read_to_string(&stamp).await.ok();
    if have.as_deref() == Some(want.as_str()) && python.exists() {
        return Ok(python);
    }
    tokio::fs::create_dir_all(env_dir)
        .await
        .map_err(io(env_dir))?;
    match tokio::fs::remove_file(&stamp).await {
        Err(e) if e.kind() != std::io::ErrorKind::NotFound => return Err(io(&stamp)(e)),
        _ => {}
    }
    tracing::info!(project = %project_dir.display(), env = %env_dir.display(), "kiss_icp_live: uv sync");
    let out = Command::new(uv)
        .arg("sync")
        .arg("--project")
        .arg(project_dir)
        .args(["--frozen", "--no-dev"])
        .env("UV_PROJECT_ENVIRONMENT", env_dir.join(VENV_DIR))
        .env("UV_CACHE_DIR", env_dir.join(CACHE_DIR))
        .env("UV_PYTHON_INSTALL_DIR", env_dir.join(PYTHON_DIR))
        .env("UV_NO_CONFIG", "1")
        .current_dir(env_dir)
        .stdin(Stdio::null())
        .kill_on_drop(true)
        .output()
        .await
        .map_err(|source| SetupError::Spawn {
            path: uv.to_path_buf(),
            source,
        })?;
    if !out.status.success() {
        let text = String::from_utf8_lossy(&out.stderr);
        let lines: Vec<&str> = text.lines().collect();
        let tail = lines[lines.len().saturating_sub(TAIL_LINES)..].join(" | ");
        return Err(SetupError::Sync {
            status: out.status,
            tail,
        });
    }
    if !python.exists() {
        return Err(SetupError::NoPython(python));
    }
    let tmp = env_dir.join(format!("{STAMP_FILE}.tmp"));
    tokio::fs::write(&tmp, want.as_bytes())
        .await
        .map_err(io(&tmp))?;
    tokio::fs::rename(&tmp, &stamp).await.map_err(io(&stamp))?;
    Ok(python)
}
