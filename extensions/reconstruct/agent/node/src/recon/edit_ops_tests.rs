use super::*;

const SQUARE: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];

fn never() -> bool {
    false
}

/// A `n`×`n` grid of points 1 m apart from (0.5, 0.5) at heights 0 and 2.
fn grid(n: usize) -> Layered {
    let mut points = Vec::new();
    for i in 0..n {
        for j in 0..n {
            for z in [0.0, 2.0] {
                points.push([i as f64 + 0.5, j as f64 + 0.5, z]);
            }
        }
    }
    let len = points.len();
    Layered {
        cloud: PointCloud {
            points,
            colors: Some(vec![[1000, 2000, 3000]; len]),
        },
        layers: vec![0; len],
    }
}

#[test]
fn crop_keeps_exactly_the_points_inside_or_outside() {
    let mut inside = grid(20);
    let removed = crop(&mut inside, Keep::Inside, &SQUARE, None, &never).unwrap();
    assert_eq!(inside.len(), 200, "10 × 10 cells, two heights");
    assert_eq!(removed, 800 - 200);
    assert!(inside
        .cloud
        .points
        .iter()
        .all(|p| p[0] < 10.0 && p[1] < 10.0));
    assert_eq!(inside.cloud.colors.as_ref().unwrap().len(), 200);

    let mut outside = grid(20);
    crop(&mut outside, Keep::Outside, &SQUARE, None, &never).unwrap();
    assert_eq!(outside.len(), 600);

    let mut banded = grid(20);
    crop(&mut banded, Keep::Inside, &SQUARE, Some([1.0, 3.0]), &never).unwrap();
    assert_eq!(banded.len(), 100, "only the upper height inside the band");
}

#[test]
fn delete_removes_only_the_selection() {
    let mut c = grid(20);
    let removed = delete(&mut c, &SQUARE, Some([-1.0, 1.0]), &never).unwrap();
    assert_eq!(removed, 100);
    assert_eq!(c.len(), 700);
    assert!(!c
        .cloud
        .points
        .iter()
        .any(|p| p[0] < 10.0 && p[1] < 10.0 && p[2] == 0.0));
}

#[test]
fn a_cancelled_filter_changes_nothing() {
    let mut c = grid(800); // 1.28 M points: one cancellation check fires
    let before = c.len();
    assert_eq!(delete(&mut c, &SQUARE, None, &|| true), Err(Cancelled));
    assert_eq!(c.len(), before);
}

#[test]
fn a_patch_is_its_own_layer_placed_by_rotation_then_translation() {
    let mut c = grid(20);
    let source = PatchSource {
        bbox: [[0.0, 0.0], [2.0, 2.0]],
    };
    let t = Transform2 {
        translate: [10.0, 0.0],
        rotate_deg: 90.0,
    };
    let p = patch(&mut c, 1, &source, &t);
    assert_eq!(p.added, 8, "2 × 2 cells, two heights");
    assert!(!p.self_overlap);
    assert_eq!(p.source_layers, vec![0]);
    assert_eq!(c.counts(), vec![(0, 800), (1, 8)]);
    // (0.5, 0.5) about the centre (1, 1) by 90°: (1.5, 0.5), then +10 east.
    let placed: Vec<[f64; 3]> = c.cloud.points[800..].to_vec();
    assert!(placed
        .iter()
        .any(|q| (q[0] - 11.5).abs() < 1e-9 && (q[1] - 0.5).abs() < 1e-9));
    assert_eq!(c.cloud.colors.as_ref().unwrap().len(), 808);

    let mut again = grid(20);
    let near = Transform2 {
        translate: [1.0, 0.0],
        rotate_deg: 0.0,
    };
    assert!(patch(&mut again, 1, &source, &near).self_overlap);
}

#[test]
fn deleting_a_layer_removes_every_one_of_its_points() {
    let mut c = grid(20);
    let source = PatchSource {
        bbox: [[0.0, 0.0], [3.0, 3.0]],
    };
    patch(
        &mut c,
        1,
        &source,
        &Transform2 {
            translate: [30.0, 0.0],
            rotate_deg: 0.0,
        },
    );
    let removed = delete_layer(&mut c, 1, &never).unwrap();
    assert_eq!(removed, 18);
    assert_eq!(c.counts(), vec![(0, 800)]);
}

#[test]
fn decimation_lands_within_two_percent_and_keeps_layers_apart() {
    let mut c = grid(300); // 180,000 points
    patch(
        &mut c,
        1,
        &PatchSource {
            bbox: [[0.0, 0.0], [50.0, 50.0]],
        },
        &Transform2 {
            translate: [400.0, 0.0],
            rotate_deg: 0.0,
        },
    );
    let target = DecimateTarget {
        target_points: Some(40_000),
        target_percent: None,
    };
    let (out, edge) = decimate(&c, &target, &never).unwrap();
    let err = (out.len() as f64 - 40_000.0).abs() / 40_000.0;
    assert!(err <= 0.02, "{} points, edge {edge}", out.len());
    let runs = out.runs();
    assert_eq!(
        runs.iter().map(|r| r.0).collect::<Vec<_>>(),
        vec![0, 1],
        "layers stay contiguous and ordered"
    );
    let dev = deviation(&c.cloud, &out.cloud).unwrap();
    assert!(
        dev.mean > 0.0 && dev.p95 >= dev.mean && dev.p95 < edge * 2.0,
        "{dev:?}"
    );
}
