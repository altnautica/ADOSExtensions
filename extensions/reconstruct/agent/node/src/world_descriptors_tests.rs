use super::esdf::transform_1d;
use super::*;

fn output(kind: &str, uri: &str, meta: serde_json::Value) -> Output {
    Output {
        id: format!("out-{kind}"),
        job_id: "job-1".into(),
        kind: kind.into(),
        uri: uri.into(),
        meta,
        created_ms: 0,
    }
}

fn write_ply(dir: &Path, name: &str, points: &[[f32; 3]]) -> PathBuf {
    let path = dir.join(name);
    let mut body = format!(
        "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n",
        points.len()
    );
    for p in points {
        body.push_str(&format!("{} {} {}\n", p[0], p[1], p[2]));
    }
    std::fs::write(&path, body).unwrap();
    path
}

#[test]
fn the_one_dimensional_transform_is_exact() {
    // A single seed at index 3 on a length-8 line: the exact squared distance
    // is (q-3)^2 everywhere. An approximate (chamfer) transform would not
    // reproduce these values.
    const INF: f32 = 1e20;
    let mut f = vec![INF; 8];
    f[3] = 0.0;
    let mut out = vec![0.0f32; 8];
    transform_1d(&f, &mut out);
    for (q, got) in out.iter().enumerate() {
        let d = q as f32 - 3.0;
        assert!(
            (got - d * d).abs() < 1e-3,
            "index {q}: expected {}, got {got}",
            d * d
        );
    }

    // Two seeds: every cell takes the nearer one.
    let mut f = vec![INF; 9];
    f[1] = 0.0;
    f[7] = 0.0;
    let mut out = vec![0.0f32; 9];
    transform_1d(&f, &mut out);
    let expect: Vec<f32> = (0..9)
        .map(|q| {
            let a = (q as f32 - 1.0).abs();
            let b = (q as f32 - 7.0).abs();
            let m = a.min(b);
            m * m
        })
        .collect();
    for (q, (got, want)) in out.iter().zip(expect.iter()).enumerate() {
        assert!(
            (got - want).abs() < 1e-3,
            "index {q}: want {want}, got {got}"
        );
    }
}

#[test]
fn the_esdf_measures_real_metric_clearance_from_a_surface() {
    // One point at the origin, 1 m voxels, 4 m truncation. The field must
    // report 0 at the surface and the true Euclidean distance around it —
    // this is the number an aircraft would fly a clearance against, so an
    // approximation is not good enough.
    let grid = esdf_from_points(&[[0.0, 0.0, 0.0]], 1.0, 4.0).unwrap();
    assert_eq!(grid.resolution_m, 1.0);
    assert_eq!(grid.truncation_m, 4.0);
    assert_eq!(grid.voxel_count(), grid.distances.len());
    // The seed voxel sits where the point does.
    let seed = (
        ((0.0 - grid.origin[0]) / 1.0).round() as u32,
        ((0.0 - grid.origin[1]) / 1.0).round() as u32,
        ((0.0 - grid.origin[2]) / 1.0).round() as u32,
    );
    assert_eq!(grid.at(seed.0, seed.1, seed.2), Some(0.0));
    // One voxel along x is exactly 1 m.
    let one = grid.at(seed.0 + 1, seed.1, seed.2).unwrap();
    assert!((one - 1.0).abs() < 1e-3, "1 m along x, got {one}");
    // The diagonal is sqrt(2) m, NOT 2 m — the thing a chamfer approximation
    // gets wrong and a planner would fly against.
    let diag = grid.at(seed.0 + 1, seed.1 + 1, seed.2).unwrap();
    assert!(
        (diag - 2.0f32.sqrt()).abs() < 1e-3,
        "the diagonal must be sqrt(2) m, got {diag}"
    );
    // And the 3-D diagonal is sqrt(3) m.
    let diag3 = grid.at(seed.0 + 1, seed.1 + 1, seed.2 + 1).unwrap();
    assert!(
        (diag3 - 3.0f32.sqrt()).abs() < 1e-3,
        "the 3-D diagonal must be sqrt(3) m, got {diag3}"
    );
    // Nothing exceeds the truncation radius.
    assert!(grid.distances.iter().all(|d| *d <= 4.0 + 1e-6));
    // The buffer is f32 little-endian, as the descriptor promises.
    assert_eq!(grid.to_le_bytes().len(), grid.distances.len() * 4);
}

#[test]
fn the_esdf_refuses_input_it_cannot_measure_rather_than_inventing_a_field() {
    // Each of these would otherwise produce a grid a planner reads as real
    // free space, which is the most dangerous possible output.
    assert_eq!(esdf_from_points(&[], 0.2, 4.0), Err(EsdfError::NoPoints));
    assert_eq!(
        esdf_from_points(&[[f32::NAN, 0.0, 0.0]], 0.2, 4.0),
        Err(EsdfError::NonFinite)
    );
    assert_eq!(
        esdf_from_points(&[[0.0, 0.0, 0.0]], 0.0, 4.0),
        Err(EsdfError::BadResolution(0.0))
    );
    // A capture whose extent implies more than the voxel ceiling is refused,
    // not silently truncated to a grid that omits most of the world.
    let huge = esdf_from_points(&[[0.0, 0.0, 0.0], [5_000.0, 5_000.0, 5_000.0]], 0.2, 4.0);
    assert!(matches!(huge, Err(EsdfError::TooLarge { .. })), "{huge:?}");
}

#[test]
fn descriptors_carry_the_session_the_generation_and_real_measured_counts() {
    let dir = tempfile::tempdir().unwrap();
    let ply = write_ply(
        dir.path(),
        "cloud.ply",
        &[[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [-1.0, 0.5, 0.0]],
    );
    let outputs = vec![
        output(
            "pointcloud",
            &format!("file://{}", ply.display()),
            serde_json::Value::Null,
        ),
        output(
            "splat",
            "http://node.example/artifacts/job-1/scene.spz",
            serde_json::json!({"gaussian_count": 1_250_000, "step": 30_000, "lod_levels": 4,
                               "manifest_url": "http://node.example/artifacts/job-1/manifest.json"}),
        ),
        output(
            "mesh",
            "http://node.example/artifacts/job-1/mesh.glb",
            serde_json::json!({"vertex_count": 8_000, "face_count": 16_000}),
        ),
    ];
    let placement = Placement::identity(None);
    let set = derive_descriptors("atlas-drone-1-1000", 7, &placement, &outputs, dir.path());
    assert!(!set.is_empty());

    let cloud = set.pointcloud.expect("a point cloud descriptor");
    assert_eq!(cloud.session_id, "atlas-drone-1-1000");
    assert_eq!(cloud.generation, 7);
    assert_eq!(
        cloud.point_count, 3,
        "the count is parsed from the real .ply, not guessed"
    );
    assert_eq!(
        cloud.bounds,
        [-1.0, 0.0, 0.0, 1.0, 2.0, 3.0],
        "bounds are the real axis-aligned extent"
    );

    let splat = set.splat.expect("a splat descriptor");
    assert_eq!(splat.generation, 7);
    assert_eq!(splat.gaussian_count, 1_250_000);
    assert_eq!(splat.lod_levels, 4);
    assert!(
        splat.manifest_url.is_some(),
        "the LOD manifest rides through"
    );

    let mesh = set.mesh.expect("a mesh descriptor");
    assert_eq!(mesh.vertex_count, 8_000);
    assert_eq!(mesh.face_count, 16_000);
    // Every artifact of the generation states the same placement.
    assert_eq!(mesh.placement, placement);
    assert_eq!(splat.placement, placement);
    assert_eq!(cloud.placement, placement);
}

#[test]
fn a_generation_with_no_readable_geometry_publishes_nothing_rather_than_an_empty_world() {
    let dir = tempfile::tempdir().unwrap();
    // A mock backend's placeholder output: no file, nothing measurable.
    let outputs = vec![output(
        "splat",
        "mock://splat/ds-9",
        serde_json::Value::Null,
    )];
    let occ = derive_occupancy(
        "sess-x",
        1,
        &Placement::identity(None),
        &outputs,
        dir.path(),
        "job-1",
        "http://node.example",
    )
    .unwrap();
    assert!(
        occ.is_none(),
        "no geometry means no planning input, never an empty free-space grid"
    );
    // And a generation with no outputs at all yields an empty set.
    assert!(
        derive_descriptors("sess-x", 1, &Placement::identity(None), &[], dir.path()).is_empty()
    );
}

#[test]
fn occupancy_is_derived_written_and_addressable() {
    let dir = tempfile::tempdir().unwrap();
    let ply = write_ply(
        dir.path(),
        "cloud.ply",
        &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    );
    let outputs = vec![output(
        "pointcloud",
        &format!("file://{}", ply.display()),
        serde_json::Value::Null,
    )];
    let (desc, grid) = derive_occupancy(
        "sess-esdf",
        3,
        &Placement::identity(None),
        &outputs,
        dir.path(),
        "job-7",
        "http://node.example/",
    )
    .unwrap()
    .expect("geometry yields an ESDF");

    assert_eq!(desc.session_id, "sess-esdf");
    assert_eq!(desc.generation, 3);
    assert_eq!(
        desc.field,
        OccupancyField::Esdf,
        "a planner needs a distance field, not a voxel dump"
    );
    assert_eq!(desc.resolution_m, DEFAULT_ESDF_RESOLUTION_M);
    assert_eq!(desc.truncation_m, DEFAULT_ESDF_TRUNCATION_M);
    assert_eq!(desc.dims, grid.dims);
    assert_eq!(desc.origin, grid.origin);

    // The buffer is on disk at the advertised path and is exactly the
    // f32 grid the descriptor's dims describe.
    let url = desc.url.expect("a fetchable url");
    assert!(url.starts_with("http://node.example/artifacts/"), "{url}");
    let rel = url.split_once("/artifacts/").unwrap().1;
    let bytes = std::fs::read(dir.path().join(rel)).expect("the esdf buffer was written");
    assert_eq!(
        bytes.len(),
        grid.dims[0] as usize * grid.dims[1] as usize * grid.dims[2] as usize * 4
    );
    assert_eq!(bytes, grid.to_le_bytes());
}
