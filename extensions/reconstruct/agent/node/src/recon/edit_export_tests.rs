use super::*;

/// A wavy 600 × 600 surface, 10 cm apart, coloured: 360,000 points.
fn surface() -> PointCloud {
    let mut points = Vec::new();
    let mut colors = Vec::new();
    for i in 0..600 {
        for j in 0..600 {
            let (x, y) = (i as f64 * 0.1, j as f64 * 0.1);
            points.push([x, y, (x * 0.7).sin() + (y * 0.3).cos()]);
            colors.push([(i * 97 % 65536) as u16, (j * 31 % 65536) as u16, 20000]);
        }
    }
    PointCloud {
        points,
        colors: Some(colors),
    }
}

#[test]
fn the_predicted_laz_size_lands_within_ten_percent() {
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("out");
    fs::create_dir_all(&out).unwrap();
    let c = surface();
    cloud::write_laz(&out.join("cloud.laz"), &c).unwrap();

    let (points, predicted) =
        estimate_size(dir.path(), "out/cloud.laz", ExportFormat::Laz, None).unwrap();
    let actual = fs::metadata(out.join("cloud.laz")).unwrap().len();
    assert_eq!(points, 360_000);
    let err = (predicted as f64 - actual as f64).abs() / actual as f64;
    assert!(err <= 0.10, "predicted {predicted}, wrote {actual}");

    // Decimated to a quarter: predicted against what the editor writes.
    let target = DecimateTarget {
        target_points: None,
        target_percent: Some(25.0),
    };
    let (points, predicted) = estimate_size(
        dir.path(),
        "out/cloud.laz",
        ExportFormat::Laz,
        Some(&target),
    )
    .unwrap();
    assert_eq!(points, 90_000);
    let layered = Layered {
        layers: vec![0; c.points.len()],
        cloud: c,
    };
    let (dec, _) = edit_ops::decimate(&layered, &target, &|| false).unwrap();
    cloud::write_laz_layers(&out.join("dec.laz"), &dec.cloud, Some(&dec.layers)).unwrap();
    let actual = fs::metadata(out.join("dec.laz")).unwrap().len();
    let err = (predicted as f64 - actual as f64).abs() / actual as f64;
    assert!(err <= 0.10, "predicted {predicted}, wrote {actual}");
}

#[test]
fn the_volume_above_a_plane_sums_the_highest_point_per_cell() {
    // A 4 m × 4 m block 2 m high on a floor at 0, sampled every 10 cm, plus
    // points under the block's top that must not count twice.
    let mut points = Vec::new();
    for i in 0..40 {
        for j in 0..40 {
            let (x, y) = (i as f64 * 0.1 + 0.05, j as f64 * 0.1 + 0.05);
            points.push([x, y, 2.0]);
            points.push([x, y, 1.0]);
        }
    }
    let c = PointCloud {
        points,
        colors: None,
    };
    let square = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
    let (v, cells) = volume_above(&c, &square, 0.0, 0.1);
    assert_eq!(cells, 1600);
    assert!((v - 32.0).abs() < 1e-6, "{v}");
    assert_eq!(
        volume_above(&c, &square, 0.0, 0.1).0,
        v,
        "the same measurement twice is the same number"
    );
}

#[test]
fn ecef_export_needs_a_georeference_and_names_what_it_writes() {
    let anchor = GeoAnchor {
        lat_deg: 12.97,
        lon_deg: 77.59,
        h_ellipsoid_m: 900.0,
        geoid_undulation_m: 0.0,
        datum: ados_capture_contract::GeoAnchorDatum::Wgs84,
        source: ados_capture_contract::GeoAnchorSource::GnssMean,
    };
    assert_eq!(
        resolve_crs(Some(&anchor), Some("epsg:4978"), None).unwrap(),
        ECEF
    );
    assert!(resolve_crs(None, Some(ECEF), None)
        .unwrap_err()
        .message
        .contains("no georeference"));
    assert_eq!(resolve_crs(None, None, None).unwrap(), "site-local frame");
    assert_eq!(
        resolve_crs(Some(&anchor), None, Some("local ENU at …")).unwrap(),
        "inherited: local ENU at …"
    );
    // The origin of the frame is the anchor itself; 1 km up stays 1 km from it.
    let o = enu_to_ecef(&anchor, [0.0, 0.0, 0.0]);
    let up = enu_to_ecef(&anchor, [0.0, 0.0, 1000.0]);
    let d = ((up[0] - o[0]).powi(2) + (up[1] - o[1]).powi(2) + (up[2] - o[2]).powi(2)).sqrt();
    assert!((d - 1000.0).abs() < 1e-6);
    let r = (o[0].powi(2) + o[1].powi(2) + o[2].powi(2)).sqrt();
    assert!((6_370_000.0..6_380_000.0).contains(&r), "{r}");
}
