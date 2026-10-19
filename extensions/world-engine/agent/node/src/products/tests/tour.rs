use std::fs;

use super::super::tour::{build_bundle, TourManifest, TourRoom};
use super::super::ProductError;

fn manifest() -> TourManifest {
    TourManifest {
        contract: "ados.capture/1".into(),
        title: "Flat".into(),
        splat: "scene.spz".into(),
        lod: None,
        mesh: Some("collision.glb".into()),
        floorplan: Some("floorplan.json".into()),
        anchor: None,
        placement: None,
        rooms: vec![TourRoom {
            name: "Living".into(),
            floor: 0,
            hotspot: [2.5, 2.0, 1.6],
            still: Some("stills/0.jpg".into()),
        }],
    }
}

#[test]
fn bundle_zips_viewer_files_and_manifest() {
    let tmp = tempfile::tempdir().unwrap();
    let viewer = tmp.path().join("viewer");
    fs::create_dir_all(viewer.join("assets")).unwrap();
    fs::write(viewer.join("index.html"), "<!doctype html>").unwrap();
    fs::write(viewer.join("assets/app.js"), "console.log(1)").unwrap();
    let out = tmp.path().join("out");
    fs::create_dir_all(&out).unwrap();
    let spz = out.join("scene.spz");
    fs::write(&spz, [1u8, 2, 3]).unwrap();
    let still = tmp.path().join("s.jpg");
    fs::write(&still, [9u8]).unwrap();

    let files = vec![
        (spz, "scene.spz".to_string()),
        (still, "stills/0.jpg".to_string()),
    ];
    let zip_path = build_bundle(&out, &viewer, &manifest(), &files).unwrap();
    assert_eq!(zip_path, out.join("tour.zip"));
    assert!(out.join("tour/stills/0.jpg").is_file());

    let mut zip = zip::ZipArchive::new(fs::File::open(&zip_path).unwrap()).unwrap();
    let names: Vec<String> = zip.file_names().map(|n| n.unwrap().to_string()).collect();
    for want in [
        "tour.json",
        "index.html",
        "assets/app.js",
        "scene.spz",
        "stills/0.jpg",
    ] {
        assert!(
            names.iter().any(|n| n == want),
            "{want} missing from {names:?}"
        );
    }
    let mut json = String::new();
    std::io::Read::read_to_string(&mut zip.by_name("tour.json").unwrap(), &mut json).unwrap();
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(v["splat"], "scene.spz");
    assert!(v["lod"].is_null() && v["anchor"].is_null() && v["placement"].is_null());
    assert_eq!(v["rooms"][0]["still"], "stills/0.jpg");
    let back: TourManifest = serde_json::from_value(v).unwrap();
    assert_eq!(back, manifest());

    // Rebuilding replaces the staged tree.
    build_bundle(&out, &viewer, &manifest(), &[]).unwrap();
    assert!(!out.join("tour/stills/0.jpg").exists());
}

#[test]
fn bundle_errors() {
    let tmp = tempfile::tempdir().unwrap();
    let viewer = tmp.path().join("viewer");
    fs::create_dir_all(&viewer).unwrap();
    let err = build_bundle(tmp.path(), &viewer, &manifest(), &[]).unwrap_err();
    assert!(matches!(err, ProductError::ViewerMissing(p) if p == viewer));

    fs::write(viewer.join("index.html"), "x").unwrap();
    let src = tmp.path().join("f");
    fs::write(&src, "x").unwrap();
    let err = build_bundle(
        tmp.path(),
        &viewer,
        &manifest(),
        &[(src, "../escape".into())],
    )
    .unwrap_err();
    assert!(matches!(err, ProductError::Format(_)));
}
