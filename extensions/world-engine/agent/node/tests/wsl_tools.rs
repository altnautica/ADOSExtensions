//! Tools run inside WSL on a Windows node.
//!
//! `process_command` checks the command the runner builds (any host). The
//! `real` tests run docker, the Kalibr container runs the calibration stage
//! builds, and the gsplat probe through WSL; they are ignored by default and
//! need the WSL distribution with docker, the pinned Kalibr image and the
//! gsplat environment (`node/tools/fetch-tools.ps1`). Their work directories
//! carry spaces and non-ASCII characters on purpose.
//!
//! ```text
//! cargo test -p world-engine-node --test wsl_tools -- --ignored --nocapture
//! ```

mod process_command {
    use std::ffi::OsStr;
    use std::path::{Path, PathBuf};
    use std::process::Command;

    use world_engine_node::calib::container;
    use world_engine_node::recon::process::command;
    use world_engine_node::recon::{ResolvedTool, ToolInvocation};
    use world_engine_node::tools::WSL_DISTRO;

    fn tool(program: &str, wsl: bool) -> ResolvedTool {
        ResolvedTool {
            name: "t".into(),
            version: "1".into(),
            program: program.into(),
            prefix_args: Vec::new(),
            home: PathBuf::new(),
            wsl,
        }
    }

    fn argv(cmd: &Command) -> Vec<String> {
        cmd.get_args()
            .map(|a| a.to_string_lossy().into_owned())
            .collect()
    }

    #[test]
    fn a_wsl_tool_runs_through_exec_with_mapped_paths_and_env() {
        let mut inv = ToolInvocation::new("t", r"D:\Daten\Übung été\job")
            .arg(r"D:\Daten\Übung été\x\train.py")
            .args(["--out", r"\\?\D:\w x\o", "--glob", "$HOME/*.png"])
            .args([r"--colmap=D:\s", "/data/already", "C:"]);
        inv.env = vec![("K".into(), r"D:\e f".into()), ("N".into(), "$1".into())];
        let cmd = command(&tool("/opt/v/bin/python", true), &inv).unwrap();
        assert_eq!(cmd.get_program(), "wsl.exe");
        assert_eq!(
            argv(&cmd),
            [
                "-d",
                WSL_DISTRO,
                "--cd",
                "/mnt/d/Daten/Übung été/job",
                "--exec",
                "/usr/bin/env",
                "K=/mnt/d/e f",
                "N=$1",
                "/opt/v/bin/python",
                "/mnt/d/Daten/Übung été/x/train.py",
                "--out",
                "/mnt/d/w x/o",
                "--glob",
                "$HOME/*.png",
                "--colmap=/mnt/d/s",
                "/data/already",
                "C:",
            ]
        );
        // Nothing is set on wsl.exe itself.
        assert_eq!(cmd.get_envs().count(), 0);
        assert_eq!(cmd.get_current_dir(), None);
    }

    #[test]
    fn a_wsl_container_run_keeps_the_shell_command_one_argument() {
        let shell = "source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_bagcreater --help";
        let inv = ToolInvocation::new("docker", r"D:\w o\job").args([
            "run",
            "--rm",
            "-v",
            r"D:\w o\job:/data",
            "--entrypoint",
            "/bin/bash",
            "local/kalibr:1f60227442d2",
            "-c",
            shell,
        ]);
        let cmd = command(&tool("docker", true), &inv).unwrap();
        let a = argv(&cmd);
        assert_eq!(
            a[..5],
            ["-d", WSL_DISTRO, "--cd", "/mnt/d/w o/job", "--exec"]
        );
        assert_eq!(a[5..9], ["docker", "run", "--rm", "-v"]);
        assert_eq!(a[9], "/mnt/d/w o/job:/data");
        assert_eq!(a[12], "local/kalibr:1f60227442d2");
        assert_eq!(a[13..], ["-c", shell]);
    }

    #[test]
    fn a_native_tool_runs_as_resolved() {
        let mut t = tool(r"D:\t\colmap.exe", false);
        t.prefix_args = vec!["run".into()];
        let mut inv = ToolInvocation::new("t", r"D:\job").arg(r"D:\in");
        inv.env = vec![("K".into(), "v".into())];
        let cmd = command(&t, &inv).unwrap();
        assert_eq!(cmd.get_program(), r"D:\t\colmap.exe");
        assert_eq!(argv(&cmd), ["run", r"D:\in"]);
        assert_eq!(cmd.get_current_dir(), Some(Path::new(r"D:\job")));
        let envs: Vec<_> = cmd.get_envs().collect();
        assert_eq!(envs, [(OsStr::new("K"), Some(OsStr::new("v")))]);
    }

    #[test]
    fn a_network_share_path_is_refused_before_spawning() {
        let share = r"\\srv\share\job";
        let local = || ToolInvocation::new("docker", r"D:\job");
        let mut with_env = local();
        with_env.env = vec![("K".into(), share.into())];
        let cases = [
            ToolInvocation::new("docker", share),
            ToolInvocation::new("docker", "//srv/share/job"),
            local().arg(share),
            local().arg(r"\\?\UNC\srv\share\x"),
            local().arg(r"--out=\\srv\share\x"),
            local().args(["-v", r"\\srv\share\job:/data"]),
            with_env,
            // The calibration container builder over a job on a share.
            container::kalibr_bagcreater(Path::new(share), "img"),
        ];
        for inv in cases {
            let e = command(&tool("docker", true), &inv).unwrap_err();
            assert_eq!(e.code, "E_PATH_UNSUPPORTED", "{inv:?}");
            assert_eq!(e.tool.as_deref(), Some("docker"));
            // The message names the offending path.
            assert!(
                e.message.contains("srv") && e.message.contains("share"),
                "{}",
                e.message
            );
        }
        // A native tool is spawned by Windows, which opens shares itself.
        assert!(command(&tool("colmap.exe", false), &local().arg(share)).is_ok());
    }
}

#[cfg(windows)]
mod real {
    use std::path::Path;

    use world_engine_node::calib::container;
    use world_engine_node::recon::{train, ProcessRunner, ToolInvocation, ToolRunner};

    fn work_dir(prefix: &str) -> tempfile::TempDir {
        tempfile::Builder::new().prefix(prefix).tempdir().unwrap()
    }

    fn run(r: &ProcessRunner, inv: &ToolInvocation) -> Vec<String> {
        let mut out = Vec::new();
        let res = r.run(inv, &mut |l| {
            println!("{l}");
            out.push(l.to_string());
        });
        res.unwrap();
        out
    }

    #[test]
    #[ignore = "needs docker in WSL"]
    fn docker_resolves_through_wsl() {
        let d = ProcessRunner::default().resolve("docker").unwrap();
        println!("docker {}", d.version);
        assert!(d.wsl);
        assert!(!d.version.is_empty());
        let k = ProcessRunner::default().resolve("kalibr").unwrap();
        assert!(k.wsl && k.program == Path::new("docker"));
    }

    #[test]
    #[ignore = "needs the Kalibr image in WSL docker"]
    fn kalibr_bagcreater_help_runs_in_its_container() {
        let work = work_dir("kalibr help é x ");
        let image = container::pinned_image("kalibr").unwrap();
        let mut inv = container::kalibr_bagcreater(work.path(), &image);
        // The same container run, asking for usage instead of building a bag.
        let shell = inv.args.last_mut().unwrap();
        let s = shell.to_string_lossy().into_owned();
        let (head, _) = s.split_once(" --folder").unwrap();
        *shell = format!("{head} --help").into();
        let out = run(&ProcessRunner::default(), &inv);
        assert!(out.iter().any(|l| l.contains("usage")));
    }

    #[test]
    #[ignore = "needs the Kalibr image in WSL docker"]
    fn kalibr_bagcreater_writes_a_bag_through_the_mapped_volume() {
        let work = work_dir("kalibr bag é x ");
        let cam = work
            .path()
            .join(container::KALIBR_DIR)
            .join(container::KALIBR_FOLDER)
            .join("cam0");
        std::fs::create_dir_all(&cam).unwrap();
        for t in [1_000_000_000u64, 1_100_000_000] {
            image::GrayImage::from_fn(64, 48, |x, y| image::Luma([((x + y) * 2) as u8]))
                .save(cam.join(format!("{t}.png")))
                .unwrap();
        }
        let image = container::pinned_image("kalibr").unwrap();
        let inv = container::kalibr_bagcreater(work.path(), &image);
        run(&ProcessRunner::default(), &inv);
        let bag = work
            .path()
            .join(container::KALIBR_DIR)
            .join(container::KALIBR_BAG);
        assert!(bag.metadata().unwrap().len() > 0, "{}", bag.display());
    }

    #[test]
    #[ignore = "needs the gsplat environment in WSL"]
    fn gsplat_probe_runs_in_wsl() {
        let r = ProcessRunner::default();
        let t = r.resolve("gsplat_train").unwrap();
        println!("{} via {}", t.name, t.program.display());
        assert!(t.wsl);
        let cwd = work_dir("gsplat probe é x ");
        let inv = ToolInvocation::new("gsplat_train", cwd.path())
            .arg(t.home.join("common.py"))
            .arg("--probe");
        let out = run(&r, &inv);
        assert!(out.iter().any(|l| l.contains("gsplat probe OK")));
        train::gsplat_available(&r, cwd.path()).unwrap();
    }
}
