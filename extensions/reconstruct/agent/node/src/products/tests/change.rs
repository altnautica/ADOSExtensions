use super::super::change::compare;
use super::super::PointCloud;

/// Surface samples of an axis-aligned box `[min, min + size]`, 2 cm spacing.
fn box_surface(min: [f64; 3], size: f64) -> Vec<[f64; 3]> {
    let n = (size / 0.02).round() as usize;
    let mut pts = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            for k in 0..=n {
                let on_face = [i, j, k].iter().any(|&v| v == 0 || v == n);
                if on_face {
                    let f = |v: usize| v as f64 * size / n as f64;
                    pts.push([min[0] + f(i), min[1] + f(j), min[2] + f(k)]);
                }
            }
        }
    }
    pts
}

/// A 4 × 4 m floor (2 cm grid) and a 0.4 m box on a shelf at 1 m.
fn scene(box_x: f64) -> (PointCloud, usize) {
    let mut pts: Vec<[f64; 3]> = (0..200)
        .flat_map(|i| (0..200).map(move |j| [i as f64 * 0.02, j as f64 * 0.02, 0.0]))
        .collect();
    let floor = pts.len();
    pts.extend(box_surface([box_x, 1.5, 1.0], 0.4));
    (
        PointCloud {
            points: pts,
            colors: None,
        },
        floor,
    )
}

#[test]
fn shifted_box_is_the_change() {
    let (a, _) = scene(1.0);
    let (b, floor) = scene(1.1);
    let res = compare(&a, &b, 0.05);
    let s = &res.summary;
    assert_eq!(s.points_compared, b.points.len());
    assert_eq!(res.distances.len(), b.points.len());
    assert!(s.points_changed > 0);
    // Only box points change; the leading face (x = 1.5) moved fully clear.
    for (i, &d) in res.distances.iter().enumerate() {
        if i < floor {
            assert!(d < 0.05, "floor point {i} flagged at {d}");
        }
    }
    let lead = b.points[floor..]
        .iter()
        .filter(|p| (p[0] - 1.5).abs() < 1e-9)
        .count();
    assert!(
        s.points_changed >= lead,
        "{} < leading face {lead}",
        s.points_changed
    );
    let bb = s.bbox_changed.expect("changed bbox");
    assert!(
        bb[0][0] >= 1.1 - 1e-9 && (bb[1][0] - 1.5).abs() < 1e-6,
        "{bb:?}"
    );
    assert!(
        (bb[0][1] - 1.5).abs() < 1e-6 && (bb[1][1] - 1.9).abs() < 1e-6,
        "{bb:?}"
    );
    assert!(
        (bb[0][2] - 1.0).abs() < 1e-6 && (bb[1][2] - 1.4).abs() < 1e-6,
        "{bb:?}"
    );
    assert!((s.max_m - 0.1).abs() < 0.005, "max {}", s.max_m);
    assert!(s.mean_changed_m >= 0.05 && s.mean_changed_m <= 0.1);
    assert!((s.changed_pct - 100.0 * s.points_changed as f64 / b.points.len() as f64).abs() < 1e-9);

    let colors = res.cloud.colors.as_ref().unwrap();
    assert_eq!(colors[0], [128 * 257; 3]);
    let changed = res.distances.iter().position(|&d| d >= 0.05).unwrap();
    assert_eq!(colors[changed][0], 65535);
    assert_eq!(colors[changed][2], 0);
}

#[test]
fn identical_clouds_have_no_change() {
    let (a, _) = scene(1.0);
    let res = compare(&a, &a, 0.05);
    assert_eq!(res.summary.points_changed, 0);
    assert_eq!(res.summary.changed_pct, 0.0);
    assert!(res.summary.bbox_changed.is_none());
    let json = serde_json::to_value(&res.summary).unwrap();
    assert!(json["bbox_changed"].is_null());
    assert_eq!(json["points_changed"], 0);
}
