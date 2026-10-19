use super::*;
use serde_json::json;
use world_engine_protocol::atlas::CameraRole;
use world_engine_protocol::atlas::PoseSource;

fn values(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.clone()))
        .collect()
}

#[test]
fn no_keys_is_the_disabled_default() {
    let c = AtlasRuntimeConfig::from_values("drone-42", &BTreeMap::new()).unwrap();
    assert!(!c.enabled);
    assert_eq!(c.device_id, "drone-42");
    assert_eq!(c.pose_provider, PoseProviderConfig::Auto);
    assert!(!c.record_on_start);
    assert_eq!(c.mains_hz, 50);
    assert_eq!(c.capture.profile, CaptureProfile::Freeform);
    assert!(c.capture.cameras.is_empty());
    assert!((c.hfov_deg - 70.0).abs() < 1e-12);
    assert_eq!(c.capture.selection, SelectionParams::default());
    assert_eq!(c.pose_publish_interval_ms, 100);
    // A null value (the host's "unset") is the same as an absent key.
    let nulls = values(&[(KEY_ENABLED, Value::Null), (KEY_CAMERAS, Value::Null)]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("drone-42", &nulls).unwrap(),
        c
    );
}

#[test]
fn configured_exposure_is_quantised_and_capped() {
    let v = values(&[(
        KEY_EXPOSURE,
        json!({"locked": true, "exposure_us": 9000, "mode": "slow"}),
    )]);
    let c = AtlasRuntimeConfig::from_values("d", &v).unwrap();
    assert_eq!(c.exposure.exposure_us, Some(10_000));
    assert!(c.exposure.locked && !c.exposure.applied);
    let v = values(&[(
        KEY_EXPOSURE,
        json!({"locked": false, "exposure_us": 9000, "mode": "walking"}),
    )]);
    let c = AtlasRuntimeConfig::from_values("d", &v).unwrap();
    assert!(c.exposure.exposure_us.unwrap() <= 4_000);
}

#[test]
fn full_key_set_loads() {
    let v = values(&[
        (KEY_ENABLED, json!(true)),
        (KEY_CAPTURE_PROFILE, json!("orbit")),
        (KEY_POSE_PROVIDER, json!("vio")),
        (KEY_RECORD_ON_START, json!(true)),
        (KEY_IMAGE_ENCODING, json!("jpeg")),
        (KEY_MAINS_HZ, json!(60)),
        (KEY_VIO, json!({"left": "a", "right": "b"})),
        (KEY_SEGMENT, json!({"max_duration_s": 60})),
        (KEY_HFOV_DEG, json!(90)),
        (
            KEY_CAMERAS,
            json!([
                {"id": "front", "role": "primary", "enabled": true, "reconstruct": true},
                {"id": "down", "role": "down", "enabled": false, "reconstruct": false},
            ]),
        ),
        (KEY_MIN_TRANSLATION_M, json!(1.0)),
        (KEY_MIN_ROTATION_RAD, json!(0.3)),
        // An integral float for an integer key is accepted.
        (KEY_MAX_INTERVAL_MS, json!(1500.0)),
        (KEY_MAX_KEYFRAMES, json!(400)),
        (
            KEY_INTRINSICS,
            json!({"front": {"fx": 900.0, "fy": 900.0, "cx": 640.0, "cy": 360.0}}),
        ),
        (KEY_POSE_PRIOR, json!({"position_sigma_m": 0.05})),
    ]);
    let c = AtlasRuntimeConfig::from_values("", &v).unwrap();
    assert!(c.enabled);
    assert_eq!(c.pose_provider, PoseProviderConfig::Vio);
    assert!(c.record_on_start);
    assert_eq!(c.image_encoding, ImageEncoding::Jpeg);
    assert_eq!(c.mains_hz, 60);
    assert_eq!((c.vio.left.as_str(), c.vio.imu.as_str()), ("a", "imu0"));
    assert_eq!(c.segment.limits().max_duration_ns, 60_000_000_000);
    assert_eq!(c.capture.profile, CaptureProfile::Orbit);
    assert_eq!(c.capture.enabled_camera_count(), 1);
    assert!((c.hfov_deg - 90.0).abs() < 1e-9);
    assert!((c.capture.selection.min_translation_m - 1.0).abs() < 1e-9);
    assert!((c.capture.selection.min_rotation_rad - 0.3).abs() < 1e-9);
    assert_eq!(c.capture.selection.max_interval_ms, 1500);
    assert_eq!(c.capture.selection.max_keyframes, 400);
    // A partial prior keeps the other figures at their defaults.
    assert_eq!(c.pose_prior.position_sigma_m, Some(0.05));
    assert!((c.pose_prior.gnss_uere_m - 2.5).abs() < 1e-12);
    // The configured intrinsics override is used for `front`.
    let k = c.intrinsics_for("front", 1280, 720);
    assert!((k.fx - 900.0).abs() < 1e-9);
    assert!((k.cx - 640.0).abs() < 1e-9);
    assert_eq!((k.width, k.height), (1280, 720));
    assert!(k.calibrated);
    // An uncalibrated camera falls back to the derived pinhole.
    assert!(!c.intrinsics_for("down", 1280, 720).calibrated);
}

#[test]
fn a_minimal_camera_entry_fills_its_defaults() {
    let v = values(&[(KEY_CAMERAS, json!([{"id": "front"}]))]);
    let c = AtlasRuntimeConfig::from_values("", &v).unwrap();
    assert_eq!(c.capture.cameras[0].role, CameraRole::Primary);
    assert!(c.capture.cameras[0].enabled);
    assert!(c.capture.cameras[0].reconstruct);
}

#[test]
fn an_unparseable_key_is_an_error_naming_it() {
    // An unknown enum is loud, never a silent default-to-disabled.
    let bad_role = values(&[
        (KEY_ENABLED, json!(true)),
        (KEY_CAMERAS, json!([{"id": "front", "role": "sideways"}])),
    ]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("", &bad_role)
            .unwrap_err()
            .key,
        KEY_CAMERAS
    );
    let bad_tier = values(&[(KEY_POSE_PROVIDER, json!("telepathic"))]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("", &bad_tier)
            .unwrap_err()
            .key,
        KEY_POSE_PROVIDER
    );
    let bad_mains = values(&[(KEY_MAINS_HZ, json!(55))]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("", &bad_mains)
            .unwrap_err()
            .key,
        KEY_MAINS_HZ
    );
    // A fractional or negative count is not silently truncated.
    let frac = values(&[(KEY_MAX_INTERVAL_MS, json!(1500.5))]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("", &frac).unwrap_err().key,
        KEY_MAX_INTERVAL_MS
    );
    let neg = values(&[(KEY_MAX_KEYFRAMES, json!(-1))]);
    assert_eq!(
        AtlasRuntimeConfig::from_values("", &neg).unwrap_err().key,
        KEY_MAX_KEYFRAMES
    );
}

#[test]
fn auto_picks_vio_on_a_visor_stereo_rig_and_the_ekf_elsewhere() {
    let r = |c, p, s| resolve_pose_provider(c, p, s);
    assert_eq!(r(PoseProviderConfig::Auto, "visor", true), PoseSource::Vio);
    assert_eq!(
        r(PoseProviderConfig::Auto, "visor", false),
        PoseSource::FcEkf
    );
    assert_eq!(
        r(PoseProviderConfig::Auto, "drone", true),
        PoseSource::FcEkf
    );
    assert_eq!(
        r(PoseProviderConfig::Lidar, "drone", false),
        PoseSource::Lidar
    );
    assert_eq!(
        r(PoseProviderConfig::FcEkf, "visor", true),
        PoseSource::FcEkf
    );
}

#[test]
fn the_fc_covariance_uses_the_stated_sigma_or_dop_times_uere() {
    let p = PosePrior::default();
    assert!((p.fc_covariance()[0] - 25.0).abs() < 1e-9);
    let rtk = PosePrior {
        position_sigma_m: Some(0.02),
        ..p
    };
    assert!((rtk.fc_covariance()[0] - 0.0004).abs() < 1e-12);
}

#[test]
fn derived_intrinsics_centre_the_principal_point() {
    let k = default_intrinsics(1280, 720, 70.0);
    assert!((k.cx - 640.0).abs() < 1e-9, "cx at centre");
    assert!((k.cy - 360.0).abs() < 1e-9, "cy at centre");
    assert!(k.fx > 0.0, "positive focal length");
    // fx == fy (square pixels), a pinhole with no distortion, frame size kept.
    assert!((k.fx - k.fy).abs() < 1e-9);
    assert_eq!(k.model, CameraModelModel::Pinhole);
    assert!(k.dist.is_empty());
    assert_eq!((k.width, k.height), (1280, 720));
}

#[test]
fn an_override_with_the_wrong_coefficient_count_is_refused() {
    let v = values(&[(
        KEY_INTRINSICS,
        json!({"front": {"model": "opencv", "fx": 900.0, "fy": 900.0, "cx": 640.0,
                         "cy": 360.0, "dist": [0.1, 0.0, 0.0, 0.0]}}),
    )]);
    let err = AtlasRuntimeConfig::from_values("", &v).unwrap_err();
    assert_eq!(err.key, KEY_INTRINSICS);
    let ok = values(&[(
        KEY_INTRINSICS,
        json!({"front": {"model": "kb4", "fx": 700.0, "fy": 700.0, "cx": 640.0,
                         "cy": 360.0, "dist": [0.01, 0.0, 0.0, 0.0]}}),
    )]);
    let c = AtlasRuntimeConfig::from_values("", &ok).unwrap();
    let m = c.intrinsics_for("front", 1280, 720);
    assert_eq!(m.model, CameraModelModel::Kb4);
    assert_eq!(m.dist.len(), 4);
}

#[test]
fn the_pre_contract_override_shape_fails_loudly() {
    let old = values(&[(
        KEY_INTRINSICS,
        json!({"front": {"fx": 900.0, "fy": 900.0, "cx": 640.0, "cy": 360.0,
                         "distortion_model": "radtan",
                         "distortion_params": [0.1, -0.2, 0.001, 0.002]}}),
    )]);
    let err = AtlasRuntimeConfig::from_values("", &old).unwrap_err();
    assert_eq!(err.key, KEY_INTRINSICS);
    assert!(err.reason.contains("distortion_model"), "{}", err.reason);
    assert!(
        err.reason.contains("dist"),
        "the error names the new keys: {}",
        err.reason
    );
}
