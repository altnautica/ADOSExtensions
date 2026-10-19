use super::super::measure::{measure, MeasureMode, MeasureRequest, MeshIndex};
use super::super::TriMesh;

fn unit_cube() -> TriMesh {
    let vertices = (0..8)
        .map(|i| [(i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64])
        .collect();
    let quads = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let triangles = quads
        .iter()
        .flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
        .collect();
    TriMesh {
        vertices,
        triangles,
    }
}

fn req(a: [f64; 3], b: [f64; 3], mode: MeasureMode) -> MeasureRequest {
    MeasureRequest {
        job_id: "j".into(),
        a,
        b,
        mode,
    }
}

#[test]
fn point_mode_snaps_to_surface() {
    let idx = MeshIndex::new(&unit_cube());
    let r = measure(
        &idx,
        &req([0.3, 0.6, 1.1], [0.3, 0.6, -0.15], MeasureMode::Point),
    )
    .unwrap();
    assert!((r.distance_m - 1.0).abs() < 1e-3, "{r:?}");
    let a = r.a_hit.unwrap();
    assert!((a[2] - 1.0).abs() < 1e-9 && (a[0] - 0.3).abs() < 1e-9);
    assert!((r.b_hit.unwrap()[2]).abs() < 1e-9);

    // Out of snap reach: the raw point is used and no hit reported.
    let r = measure(
        &idx,
        &req([0.5, 0.5, 1.5], [0.5, 0.5, 0.95], MeasureMode::Point),
    )
    .unwrap();
    assert!(r.a_hit.is_none());
    let b = r.b_hit.unwrap();
    assert!((b[2] - 1.0).abs() < 1e-9);
    assert!((r.distance_m - 0.5).abs() < 1e-3);

    assert_eq!(idx.nearest([3.0, 3.0, 3.0], 0.25), None);
    let corner = idx.nearest([1.1, 1.1, 1.1], 0.25).unwrap();
    assert!(corner.iter().all(|v| (v - 1.0).abs() < 1e-9));
}

#[test]
fn ray_mode_hits_first_face() {
    let idx = MeshIndex::new(&unit_cube());
    let r = measure(
        &idx,
        &req([0.25, 0.75, 3.0], [0.0, 0.0, -2.0], MeasureMode::Ray),
    )
    .unwrap();
    assert!((r.distance_m - 2.0).abs() < 1e-3, "{r:?}");
    assert_eq!(r.a_hit, Some([0.25, 0.75, 3.0]));
    let hit = r.b_hit.unwrap();
    assert!((hit[2] - 1.0).abs() < 1e-9 && (hit[0] - 0.25).abs() < 1e-9);

    // From inside, the ray exits through the far face.
    let (t, p) = idx.raycast([0.5, 0.5, 0.5], [1.0, 0.0, 0.0]).unwrap();
    assert!((t - 0.5).abs() < 1e-9 && (p[0] - 1.0).abs() < 1e-9);

    assert!(measure(
        &idx,
        &req([0.5, 0.5, 3.0], [0.0, 0.0, 1.0], MeasureMode::Ray)
    )
    .is_none());
    assert!(measure(
        &idx,
        &req([0.5, 0.5, 3.0], [0.0, 0.0, 0.0], MeasureMode::Ray)
    )
    .is_none());
}

#[test]
fn request_mode_defaults_to_point() {
    let r: MeasureRequest =
        serde_json::from_str(r#"{"job_id":"x","a":[0,0,0],"b":[1,0,0]}"#).unwrap();
    assert_eq!(r.mode, MeasureMode::Point);
    let r: MeasureRequest =
        serde_json::from_str(r#"{"job_id":"x","a":[0,0,0],"b":[1,0,0],"mode":"ray"}"#).unwrap();
    assert_eq!(r.mode, MeasureMode::Ray);
}
