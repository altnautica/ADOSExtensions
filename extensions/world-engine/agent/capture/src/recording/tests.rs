use super::*;
use ados_capture_contract::{SegmentLimits, SessionReader};
use ados_config::capture_storage::{CaptureState, CaptureStorageSidecar, CaptureUsage};

/// A fresh janitor sidecar naming `root`.
pub fn write_sidecar(path: &Path, root: &Path, state: CaptureState) {
    let s = CaptureStorageSidecar {
        version: 1,
        updated_ns: 0,
        capture_root: root.to_string_lossy().into_owned(),
        volume: None,
        capture: CaptureUsage {
            used_bytes: 0,
            budget_bytes: 1 << 40,
            budget_pct: 85,
            floor_free_bytes: 5 << 30,
            sessions: 0,
            uploaded: 0,
            evict_unuploaded: false,
            state,
        },
        write_probe: None,
    };
    std::fs::write(path, serde_json::to_vec(&s).unwrap()).unwrap();
}

fn env<'a>(sidecar: &'a Path) -> StartEnv<'a> {
    StartEnv {
        device_id: "visor 1",
        profile: "visor",
        rig: crate::recorder::tests::rig(),
        timebase: Timebase::HubFreerun,
        clock: crate::clock::ClockMap::new(),
        sigma_ns: 420_000,
        t0_ns: 5_000_000_000,
        utc_at_t0_ns: Some(1_760_000_000_000_000_000),
        gnss_anchor: None,
        sidecar,
        limits: SegmentLimits::default(),
        encoding: ImageEncoding::Raw,
        active_marker: None,
    }
}

fn health() -> SessionHealth {
    SessionHealth {
        blur: true,
        texture: false,
        flicker: false,
        corridor: false,
        frames_dropped_pct: 0.5,
    }
}

#[test]
fn a_missing_or_full_sidecar_refuses_with_storage_full() {
    let dir = tempfile::tempdir().unwrap();
    let sidecar = dir.path().join("capture-storage.json");
    let e = ActiveSession::start(StartArgs::default(), env(&sidecar), 0)
        .err()
        .unwrap();
    assert_eq!(e.error.as_deref(), Some(codes::STORAGE_FULL));
    write_sidecar(&sidecar, dir.path(), CaptureState::Full);
    let e = ActiveSession::start(StartArgs::default(), env(&sidecar), 0)
        .err()
        .unwrap();
    assert_eq!(e.error.as_deref(), Some(codes::STORAGE_FULL));
}

#[test]
fn rooms_from_mark_and_the_operator_anchor_reach_the_manifest() {
    let dir = tempfile::tempdir().unwrap();
    let sidecar = dir.path().join("capture-storage.json");
    let root = dir.path().join("capture");
    std::fs::create_dir_all(&root).unwrap();
    write_sidecar(&sidecar, &root, CaptureState::Ok);
    let args = StartArgs {
        site: Some("Warehouse 3".into()),
        floor: Some(0),
        room: Some("Dock".into()),
        operator_anchor: Some(OperatorAnchor {
            lat_deg: 12.97,
            lon_deg: 77.59,
            h_ellipsoid_m: 840.0,
            heading_deg: Some(90.0),
        }),
    };
    let mut s = ActiveSession::start(args, env(&sidecar), 0).unwrap();
    assert!(s.session_id.starts_with("visor-1-"), "{}", s.session_id);
    assert_eq!((s.room(), s.segment()), (Some("Dock"), 1));
    assert_eq!(s.mark("Aisle 1".into(), 0, 6_000_000_000), 2);
    assert_eq!(s.mark("Mezzanine".into(), 1, 7_000_000_000), 3);
    let dir_path = s.recorder.dir().to_path_buf();
    assert!(dir_path.join(".recording").exists());
    let m = s.finish(8_000_000_000, health()).unwrap();
    assert_eq!(m.timebase, Timebase::HubFreerun);
    assert_eq!(m.anchor.unwrap().source, GeoAnchorSource::Operator);
    let rooms: Vec<(&str, i32, u64, u64)> = m
        .operator
        .rooms
        .iter()
        .map(|r| (r.name.as_str(), r.floor, r.t_start_ns, r.t_end_ns))
        .collect();
    assert_eq!(
        rooms,
        [
            ("Dock", 0, 5_000_000_000, 6_000_000_000),
            ("Aisle 1", 0, 6_000_000_000, 7_000_000_000),
            ("Mezzanine", 1, 7_000_000_000, 8_000_000_000),
        ]
    );
    assert!(m.health.blur);
    assert!(!dir_path.join(".recording").exists());
    SessionReader::open(&dir_path).unwrap().verify().unwrap();
}

#[test]
fn a_bad_operator_anchor_is_refused() {
    let a = OperatorAnchor {
        lat_deg: 120.0,
        lon_deg: 0.0,
        h_ellipsoid_m: 0.0,
        heading_deg: None,
    };
    assert_eq!(
        operator_anchor(&a).unwrap_err().error.as_deref(),
        Some(codes::BAD_REQUEST)
    );
    assert_eq!(device_segment("dr one/7"), "dr-one-7");
    assert_eq!(device_segment(""), "device");
}
