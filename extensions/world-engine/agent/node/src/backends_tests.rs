//! Tests of `backends`.

use super::*;

fn dataset(id: &str, input: Option<&str>) -> Dataset {
    let meta = match input {
        Some(p) => serde_json::json!({ "input_path": p }),
        None => serde_json::json!({}),
    };
    Dataset {
        id: id.into(),
        kind: "bag".into(),
        created_ms: 0,
        meta,
    }
}

fn arg_val(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1).cloned())
}

#[test]
fn kind_from_hint_maps_known_tools() {
    assert_eq!(
        ReconstructorKind::from_hint("BRUSH"),
        Some(ReconstructorKind::Brush)
    );
    assert_eq!(
        ReconstructorKind::from_hint("msplat"),
        Some(ReconstructorKind::Msplat)
    );
    assert_eq!(ReconstructorKind::from_hint("splatfacto"), None);
}

#[test]
fn brush_command_bounds_densification_and_honors_overrides() {
    let r = CliReconstructor::new(ReconstructorKind::Brush, "/work");
    let cmd = r.command(
        &dataset("ds", Some("/d")),
        &serde_json::json!({ "steps": 7000 }),
    );
    assert_eq!(cmd.program, "brush");
    assert_eq!(cmd.output_path, Path::new("/work/ds/output.ply"));
    assert_eq!(cmd.args.first().map(String::as_str), Some("/d"));
    assert_eq!(
        arg_val(&cmd.args, "--growth-stop-iter").as_deref(),
        Some("3500")
    );
    assert_eq!(
        arg_val(&cmd.args, "--max-splats").as_deref(),
        Some("1500000")
    );
    assert_eq!(
        arg_val(&cmd.args, "--export-name").as_deref(),
        Some("output.ply")
    );
    let cmd = r.command(
        &dataset("ds", Some("/d")),
        &serde_json::json!({ "steps": 15000, "max_splats": 600000, "sh_degree": 2, "growth_stop_iter": 5000 }),
    );
    assert_eq!(
        arg_val(&cmd.args, "--max-splats").as_deref(),
        Some("600000")
    );
    assert_eq!(arg_val(&cmd.args, "--sh-degree").as_deref(), Some("2"));
    assert_eq!(
        arg_val(&cmd.args, "--growth-stop-iter").as_deref(),
        Some("5000")
    );
}

#[test]
fn msplat_command_maps_dataset_steps_and_sh() {
    let r = CliReconstructor::new(ReconstructorKind::Msplat, "/work");
    let cmd = r.command(
        &dataset("ds", Some("/colmap")),
        &serde_json::json!({ "steps": 7000, "sh_degree": 2 }),
    );
    assert_eq!(cmd.program, "msplat");
    assert_eq!(arg_val(&cmd.args, "-n").as_deref(), Some("7000"));
    assert_eq!(
        arg_val(&cmd.args, "-o").map(PathBuf::from),
        Some(PathBuf::from("/work/ds").join("output.ply"))
    );
}

#[test]
fn gaussian_count_parses_the_first_run_only() {
    assert_eq!(
        parse_gaussian_count("training...\ngaussians: 248123\n"),
        248123
    );
    assert_eq!(
        parse_gaussian_count("Num splats: 1000\nNum splats: 2500"),
        2500
    );
    assert_eq!(parse_gaussian_count("no count here"), 0);
    assert_eq!(parse_gaussian_count("gaussians: 248123 of 1000000"), 248123);
}

#[test]
fn selection_never_falls_back_to_a_placeholder() {
    let r = select_reconstructor(&serde_json::json!({ "backend": "msplat" }), "/work");
    assert_eq!(r.name(), "seeded-splat");
    let r = select_reconstructor(&serde_json::json!({ "backend": "no-such-tool" }), "/work");
    assert_ne!(r.name(), "mock");
}

#[test]
fn an_uninstalled_trainer_fails_with_tool_missing() {
    let dir = tempfile::tempdir().unwrap();
    std::env::set_var("ADOS_WE_TOOLS_DIR", dir.path());
    let err = CliReconstructor::new(ReconstructorKind::Brush, dir.path())
        .reconstruct(&dataset("ds-x", None), &serde_json::json!({}))
        .unwrap_err();
    match err {
        ComputeError::Backend { backend, message } => {
            assert_eq!(backend, "brush");
            assert!(message.contains("E_TOOL_MISSING"), "{message}");
        }
        other => panic!("expected a backend error, got {other:?}"),
    }
}

#[test]
fn file_uri_round_trips_a_spaced_path() {
    let uri = path_to_file_uri("/My Work/ds 1/output.ply");
    assert_eq!(uri, "file:///My%20Work/ds%201/output.ply");
    assert_eq!(
        file_uri_to_path(&uri),
        Path::new("/My Work/ds 1/output.ply")
    );
    assert_eq!(file_uri_to_path("/plain/path"), Path::new("/plain/path"));
}
