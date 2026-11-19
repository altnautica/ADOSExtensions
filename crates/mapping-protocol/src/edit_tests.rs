use super::*;

const SQUARE: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];

fn crop(polygon: &[[f64; 2]]) -> EditOp {
    EditOp::Crop {
        keep: Keep::Inside,
        polygon: polygon.to_vec(),
        z: None,
    }
}

fn params(ops: Vec<EditOp>) -> EditParams {
    EditParams {
        job: None,
        source: "out/cloud.laz".into(),
        ops,
    }
}

#[test]
fn the_documented_submit_body_parses() {
    let body = serde_json::json!({
        "source": "out/cloud.laz",
        "ops": [{"op": "crop", "keep": "inside",
                 "polygon": [[527104.2, 2093118.4], [527146.9, 2093118.4],
                             [527146.9, 2093161.1], [527104.2, 2093161.1]]}]
    });
    let p: EditParams = serde_json::from_value(body).unwrap();
    assert!(
        matches!(&p.ops[0], EditOp::Crop { keep: Keep::Inside, polygon, z: None } if polygon.len() == 4)
    );
    p.validate().unwrap();
}

#[test]
fn a_provenance_record_keeps_the_documented_shape() {
    let rec = EditRecord {
        op: EditOp::Patch {
            layer: "patch-1".into(),
            source: PatchSource {
                bbox: [[527118.0, 2093126.0], [527124.0, 2093132.0]],
            },
            target: Transform2 {
                translate: [3.2, -1.1],
                rotate_deg: 0.0,
            },
        },
        removed: None,
        added: Some(5410),
        self_overlap: Some(false),
        source_layers: vec!["base".into()],
        deviation_m: None,
        result: "out/cloud.v2.laz".into(),
        ts: "2026-11-18T09:44:03Z".into(),
    };
    let v = serde_json::to_value(&rec).unwrap();
    assert_eq!(v["op"], "patch");
    assert_eq!(v["layer"], "patch-1");
    assert_eq!(v["source"]["box"][1][0], 527124.0);
    assert_eq!(v["target"]["translate"][1], -1.1);
    assert_eq!(v["added"], 5410);
    assert!(v.get("removed").is_none());
    let back: EditRecord = serde_json::from_value(v).unwrap();
    assert_eq!(back, rec);

    let dec: EditRecord = serde_json::from_value(serde_json::json!({
        "op": "decimate", "target_points": 20000000,
        "deviation_m": {"mean": 0.011, "p95": 0.034},
        "result": "out/cloud.v3.laz", "ts": "2026-11-18T09:47:55Z"
    }))
    .unwrap();
    assert_eq!(
        dec.op,
        EditOp::Decimate(DecimateTarget {
            target_points: Some(20_000_000),
            target_percent: None
        })
    );
    assert_eq!(dec.deviation_m.unwrap().p95, 0.034);
}

#[test]
fn a_crossing_polygon_is_refused_and_a_simple_one_accepted() {
    assert_eq!(valid_polygon(&SQUARE), Ok(()));
    let bowtie = [[0.0, 0.0], [10.0, 10.0], [10.0, 0.0], [0.0, 10.0]];
    assert_eq!(valid_polygon(&bowtie), Err(EditError::SelfIntersecting));
    let concave = [
        [0.0, 0.0],
        [10.0, 0.0],
        [5.0, 3.0],
        [10.0, 10.0],
        [0.0, 10.0],
    ];
    assert_eq!(valid_polygon(&concave), Ok(()));
    assert_eq!(valid_polygon(&SQUARE[..2]), Err(EditError::TooFewVertices));
    let many: Vec<[f64; 2]> = (0..300)
        .map(|i| {
            let a = i as f64 / 300.0 * std::f64::consts::TAU;
            [a.cos(), a.sin()]
        })
        .collect();
    assert_eq!(valid_polygon(&many), Err(EditError::TooManyVertices));
    assert_eq!(
        valid_polygon(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 1.0]]),
        Err(EditError::NotFinite)
    );
}

#[test]
fn points_in_a_polygon_are_found_by_ray_casting() {
    assert!(point_in_polygon(&SQUARE, [5.0, 5.0]));
    assert!(!point_in_polygon(&SQUARE, [15.0, 5.0]));
    let concave = [
        [0.0, 0.0],
        [10.0, 0.0],
        [5.0, 5.0],
        [10.0, 10.0],
        [0.0, 10.0],
    ];
    assert!(point_in_polygon(&concave, [2.0, 5.0]));
    assert!(!point_in_polygon(&concave, [8.0, 5.0]));
}

#[test]
fn the_source_must_stay_under_out() {
    for bad in [
        "cloud.laz",
        "out/",
        "out/../creds.json",
        "/out/cloud.laz",
        "out//x.laz",
        "out/a\\b",
    ] {
        let mut p = params(vec![crop(&SQUARE)]);
        p.source = bad.into();
        assert_eq!(p.validate(), Err(EditError::BadSource), "{bad}");
    }
    let mut p = params(vec![crop(&SQUARE)]);
    p.source = "out/cloud.v2.laz".into();
    assert_eq!(p.validate(), Ok(()));
}

#[test]
fn decimate_takes_exactly_one_target() {
    let t = |points, percent| DecimateTarget {
        target_points: points,
        target_percent: percent,
    };
    assert_eq!(t(Some(10), None).validate(), Ok(()));
    assert_eq!(t(None, Some(25.0)).validate(), Ok(()));
    assert_eq!(
        t(Some(10), Some(25.0)).validate(),
        Err(EditError::DecimateTarget)
    );
    assert_eq!(t(None, None).validate(), Err(EditError::DecimateTarget));
    assert_eq!(
        t(None, Some(100.0)).validate(),
        Err(EditError::DecimatePercent)
    );
    assert_eq!(t(Some(0), None).validate(), Err(EditError::DecimatePoints));
    assert_eq!(t(None, Some(25.0)).points_for(1000), 250);
    assert_eq!(t(Some(5000), None).points_for(1000), 1000);
}

#[test]
fn layers_and_bands_are_checked() {
    let patch = |layer: &str| EditOp::Patch {
        layer: layer.into(),
        source: PatchSource {
            bbox: [[0.0, 0.0], [1.0, 1.0]],
        },
        target: Transform2 {
            translate: [2.0, 0.0],
            rotate_deg: 90.0,
        },
    };
    assert_eq!(patch("patch-1").validate(), Ok(()));
    assert_eq!(patch("base").validate(), Err(EditError::BadLayer));
    assert_eq!(patch("a b").validate(), Err(EditError::BadLayer));
    let flat = EditOp::Patch {
        layer: "patch-1".into(),
        source: PatchSource {
            bbox: [[0.0, 0.0], [1.0, 0.0]],
        },
        target: Transform2 {
            translate: [0.0, 0.0],
            rotate_deg: 0.0,
        },
    };
    assert_eq!(flat.validate(), Err(EditError::EmptyBox));
    let banded = EditOp::Delete {
        polygon: SQUARE.to_vec(),
        z: Some([2.0, 1.0]),
    };
    assert_eq!(banded.validate(), Err(EditError::BadBand));
    assert_eq!(params(vec![]).validate(), Err(EditError::NoOps));
}
