//! Environment provisioning with a stand-in `uv`.

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use world_engine_protocol::atlas::VioHealth;

use super::setup::{ensure_env, env_python, SetupError, STAMP_FILE};
use super::tests::{config, scan, FAKE_MAIN};
use super::*;
use crate::pose::PoseProvider;

/// A stand-in `uv`: checks the exact command line and environment the
/// capture service uses, fails when the project holds a `FAIL` file, and
/// otherwise makes `$UV_PROJECT_ENVIRONMENT/bin/python` (a wrapper around the
/// system python3) and counts the sync in `<env_dir>/syncs`.
const FAKE_UV: &str = r#"#!/bin/sh
[ "$1" = sync ] && [ "$2" = --project ] && [ "$4" = --frozen ] && [ "$5" = --no-dev ] || exit 9
[ -f "$3/uv.lock" ] && [ "$UV_NO_CONFIG" = 1 ] || exit 9
[ -n "$UV_CACHE_DIR" ] && [ -n "$UV_PYTHON_INSTALL_DIR" ] && [ -n "$UV_PROJECT_ENVIRONMENT" ] || exit 9
if [ -f "$3/FAIL" ]; then echo "error: failed to fetch kiss-icp: network unreachable" >&2; exit 1; fi
mkdir -p "$UV_PROJECT_ENVIRONMENT/bin" || exit 9
printf '#!/bin/sh\nexec python3 "$@"\n' > "$UV_PROJECT_ENVIRONMENT/bin/python"
chmod +x "$UV_PROJECT_ENVIRONMENT/bin/python"
echo sync >> "$UV_PROJECT_ENVIRONMENT/../syncs"
"#;

fn executable(path: &Path, body: &str) {
    std::fs::write(path, body).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

/// A project dir with a lock and the fake child package, and a fake uv.
fn project(dir: &Path) -> (PathBuf, PathBuf) {
    let proj = dir.join("kiss_icp_live");
    let pkg = proj.join("kiss_icp_live");
    std::fs::create_dir_all(&pkg).unwrap();
    std::fs::write(proj.join("pyproject.toml"), "[project]\nname = \"x\"\n").unwrap();
    std::fs::write(proj.join("uv.lock"), "version = 1\n").unwrap();
    std::fs::write(pkg.join("__init__.py"), "").unwrap();
    std::fs::write(pkg.join("__main__.py"), FAKE_MAIN).unwrap();
    let uv = dir.join("fake-uv");
    executable(&uv, FAKE_UV);
    (proj, uv)
}

fn syncs(env: &Path) -> usize {
    std::fs::read_to_string(env.join("syncs"))
        .map(|s| s.lines().count())
        .unwrap_or(0)
}

#[tokio::test]
async fn syncs_once_and_again_only_when_needed() {
    let dir = tempfile::tempdir().unwrap();
    let (proj, uv) = project(dir.path());
    let env = dir.path().join("work").join(ENV_DIR);

    let py = ensure_env(&uv, &proj, &env).await.unwrap();
    assert_eq!(py, env_python(&env));
    assert!(py.is_file());
    assert_eq!(syncs(&env), 1);
    assert!(env.join(STAMP_FILE).is_file());

    // Current: no sync.
    ensure_env(&uv, &proj, &env).await.unwrap();
    assert_eq!(syncs(&env), 1);

    // A plugin update that changes the lock re-syncs.
    std::fs::write(proj.join("uv.lock"), "version = 1\n# new pin\n").unwrap();
    ensure_env(&uv, &proj, &env).await.unwrap();
    assert_eq!(syncs(&env), 2);

    // A deleted environment re-syncs.
    std::fs::remove_file(&py).unwrap();
    ensure_env(&uv, &proj, &env).await.unwrap();
    assert_eq!(syncs(&env), 3);

    // A failed sync leaves no stamp, so the next start tries again.
    std::fs::write(proj.join("pyproject.toml"), "[project]\nname = \"y\"\n").unwrap();
    std::fs::write(proj.join("FAIL"), "").unwrap();
    match ensure_env(&uv, &proj, &env).await {
        Err(SetupError::Sync { tail, .. }) => {
            assert!(tail.contains("network unreachable"), "{tail}")
        }
        other => panic!("{other:?}"),
    }
    assert!(!env.join(STAMP_FILE).exists());
    std::fs::remove_file(proj.join("FAIL")).unwrap();
    ensure_env(&uv, &proj, &env).await.unwrap();
    assert_eq!(syncs(&env), 4);
}

#[tokio::test]
async fn missing_uv_or_project_is_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let (proj, _) = project(dir.path());
    let env = dir.path().join("env");
    let err = ensure_env(&dir.path().join("no-uv"), &proj, &env)
        .await
        .unwrap_err();
    assert!(matches!(err, SetupError::Spawn { .. }), "{err}");
    let err = ensure_env(&dir.path().join("no-uv"), &dir.path().join("nope"), &env)
        .await
        .unwrap_err();
    assert!(matches!(err, SetupError::Io { .. }), "{err}");
}

fn python3_available() -> bool {
    std::process::Command::new("python3")
        .arg("-V")
        .output()
        .is_ok()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_uv_launcher_provisions_once_then_runs_the_environment() {
    if !python3_available() {
        eprintln!("python3 not found; skipped");
        return;
    }
    let dir = tempfile::tempdir().unwrap();
    let (proj, uv) = project(dir.path());
    let cfg = LidarPoseConfig {
        launcher: Launcher::Uv(uv),
        project_dir: proj.clone(),
        ..config(dir.path(), "mid360")
    };
    let env = cfg.work_dir.join(ENV_DIR);
    let pose = LidarPose::spawn(cfg).unwrap();
    let runs = || {
        std::fs::read_to_string(proj.join("runs.txt"))
            .map(|s| s.lines().count())
            .unwrap_or(0)
    };
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut t = 1_000_000_000u64;
    let mut posed = false;
    while Instant::now() < deadline && !(posed && runs() >= 2) {
        pose.feed_lidar(&scan(t, 4));
        t += 100_000_000;
        tokio::time::sleep(Duration::from_millis(30)).await;
        posed |= pose.latest().is_some();
    }
    assert!(posed, "the provisioned interpreter ran the child");
    assert!(runs() >= 2, "the child was restarted");
    assert_eq!(syncs(&env), 1, "a restart does not re-sync");
    assert_eq!(pose.stats().setup_failures, 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_failing_sync_keeps_the_provider_lost_and_retries() {
    let dir = tempfile::tempdir().unwrap();
    let (proj, uv) = project(dir.path());
    std::fs::write(proj.join("FAIL"), "").unwrap();
    let pose = LidarPose::spawn(LidarPoseConfig {
        launcher: Launcher::Uv(uv),
        project_dir: proj,
        ..config(dir.path(), "mid360")
    })
    .unwrap();
    // First attempt at once, the second after the 1 s backoff.
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && pose.stats().setup_failures < 2 {
        pose.feed_lidar(&scan(1, 4));
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let st = pose.stats();
    assert!(st.setup_failures >= 2, "{st:?}");
    assert_eq!(st.restarts, 0, "no child ever started");
    assert_eq!(pose.health(), VioHealth::Lost);
    assert!(pose.latest().is_none());
}
