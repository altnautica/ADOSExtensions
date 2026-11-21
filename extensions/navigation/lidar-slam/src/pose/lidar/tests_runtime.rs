use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use ados_mapping_pose::pose::PoseProvider;

use super::runtime::{check_tree, resolve, RuntimeError, RUNTIME_PYTHON};
use super::tests::{config, scan, FAKE_MAIN};
use super::*;

fn chmod(path: &Path, mode: u32) {
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
}

/// A runtime tree as an install lays it out: `bin/python3` (a wrapper around
/// the system python3, so the fake child runs) and a library file, every
/// dir `0755` and file `0755`/`0644`, inside a private `0700` parent.
fn runtime_in(dir: &Path) -> PathBuf {
    chmod(dir, 0o700);
    let rt = dir.join("lib/kiss-icp-runtime");
    std::fs::create_dir_all(rt.join("bin")).unwrap();
    std::fs::create_dir_all(rt.join("lib/site")).unwrap();
    let python = rt.join(RUNTIME_PYTHON);
    std::fs::write(&python, "#!/bin/sh\nexec python3 \"$@\"\n").unwrap();
    chmod(&python, 0o755);
    std::fs::write(rt.join("lib/site/mod.py"), "VALUE = 1\n").unwrap();
    chmod(&rt.join("lib/site/mod.py"), 0o644);
    for d in [
        dir.join("lib"),
        rt.clone(),
        rt.join("bin"),
        rt.join("lib"),
        rt.join("lib/site"),
    ] {
        chmod(&d, 0o755);
    }
    rt
}

fn no_uv(dir: &Path) -> PathBuf {
    dir.join("no-such-uv")
}

#[test]
fn the_install_tree_runtime_is_picked_with_no_note() {
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    let got = resolve(&rt, &no_uv(dir.path())).unwrap();
    assert_eq!(got.launcher, Launcher::Python(rt.join(RUNTIME_PYTHON)));
    assert_eq!(got.note, None);
}

#[test]
fn a_runtime_writable_by_others_is_refused() {
    let unsafe_path = |rt: &Path| match resolve(rt, Path::new("sh")) {
        Err(RuntimeError::Unsafe { path, why }) => (path, why),
        other => panic!("{other:?}"),
    };
    // A group- or world-writable dir or file anywhere in the tree.
    for (rel, mode) in [
        ("lib/site", 0o775),
        ("lib/site/mod.py", 0o666),
        ("bin", 0o777),
    ] {
        let dir = tempfile::tempdir().unwrap();
        let rt = runtime_in(dir.path());
        chmod(&rt.join(rel), mode);
        let (path, why) = unsafe_path(&rt);
        assert_eq!(path, rt.join(rel), "{why}");
        assert!(why.contains("writable by group or others"), "{why}");
    }
    // A writable dir above the runtime that is not sticky.
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    chmod(&dir.path().join("lib"), 0o777);
    assert_eq!(unsafe_path(&rt).0, dir.path().join("lib"));
    // A symlink inside the tree.
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    std::os::unix::fs::symlink("/usr/bin/python3", rt.join("bin/python3.12")).unwrap();
    let (path, why) = unsafe_path(&rt);
    assert_eq!(
        (path, why.as_str()),
        (rt.join("bin/python3.12"), "is a symlink")
    );
    // No interpreter, or one that does not execute.
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    chmod(&rt.join(RUNTIME_PYTHON), 0o644);
    assert!(unsafe_path(&rt).1.contains("not an executable file"));
    std::fs::remove_file(rt.join(RUNTIME_PYTHON)).unwrap();
    assert!(unsafe_path(&rt).1.contains("no interpreter"));
}

#[test]
fn only_root_or_this_user_may_own_the_tree() {
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    let me = std::fs::metadata("/proc/self").ok().map(|m| {
        use std::os::unix::fs::MetadataExt;
        m.uid()
    });
    assert_eq!(check_tree(&rt, me), Ok(()));
    if me.is_some_and(|uid| uid != 0) {
        // Seen as another user, this user's files are someone else's.
        let (_, why) = check_tree(&rt, Some(u32::MAX)).unwrap_err();
        assert!(why.starts_with("owned by uid"), "{why}");
    }
}

#[test]
fn without_a_runtime_the_uv_fallback_is_reported() {
    let dir = tempfile::tempdir().unwrap();
    let missing = dir.path().join("lib/kiss-icp-runtime");
    let got = resolve(&missing, Path::new("sh")).unwrap();
    assert!(matches!(&got.launcher, Launcher::Uv(p) if p.is_absolute() && p.ends_with("sh")));
    let note = got.note.unwrap();
    assert!(note.contains("no prebuilt KISS-ICP runtime"), "{note}");
    assert!(note.contains("developer fallback"), "{note}");
    assert_eq!(
        resolve(&missing, &no_uv(dir.path())),
        Err(RuntimeError::Missing {
            runtime: missing.clone(),
            uv: no_uv(dir.path()).display().to_string(),
        })
    );
}

/// The resolved runtime runs the child: poses come back through the
/// install-tree interpreter, and the child writes nothing into the tree.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_resolved_runtime_runs_the_child() {
    if std::process::Command::new("python3")
        .arg("-V")
        .output()
        .is_err()
    {
        eprintln!("python3 not found; skipped");
        return;
    }
    let dir = tempfile::tempdir().unwrap();
    let rt = runtime_in(dir.path());
    let pkg = dir.path().join("kiss_icp_live");
    std::fs::create_dir_all(&pkg).unwrap();
    std::fs::write(pkg.join("__init__.py"), "").unwrap();
    std::fs::write(pkg.join("__main__.py"), FAKE_MAIN).unwrap();
    let before = listing(&rt);
    let resolved = resolve(&rt, &no_uv(dir.path())).unwrap();
    let pose = LidarPose::spawn(LidarPoseConfig {
        launcher: resolved.launcher,
        ..config(dir.path(), "mid360")
    })
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut t = 1_000_000_000u64;
    while Instant::now() < deadline && pose.latest().is_none() {
        pose.feed_lidar(&scan(t, 4));
        t += 100_000_000;
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    assert!(
        pose.latest().is_some(),
        "the runtime's interpreter ran the child"
    );
    assert_eq!(pose.stats().setup_failures, 0);
    assert_eq!(listing(&rt), before, "nothing was written into the runtime");
}

fn listing(root: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(p) = stack.pop() {
        if p.is_dir() {
            stack.extend(std::fs::read_dir(&p).unwrap().map(|e| e.unwrap().path()));
        }
        out.push(p);
    }
    out.sort();
    out
}
