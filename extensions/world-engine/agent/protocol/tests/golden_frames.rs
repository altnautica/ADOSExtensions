//! The GCS half decodes atlas frames with its own msgpack reader, so its golden
//! fixtures (`gcs/tests/lib/atlas/golden-atlas-frames.ts`) must be bytes this
//! producer actually emits. This test renders them from the real types and
//! checks the committed block; `ADOS_ATLAS_REGEN_GOLDEN=1 cargo test -p
//! world-engine-protocol --test golden_frames` rewrites it.

use std::path::PathBuf;

use ados_capture_contract::{GeoAnchor, GeoAnchorDatum, GeoAnchorSource, Placement};
use world_engine_protocol::atlas::{
    AtlasEvent, CaptureState, CaptureStatus, MeshDescriptor, OccupancyDescriptor, OccupancyField,
    PointCloudDescriptor, PoseSource, SplatDescriptor, VioHealth, PLUGIN_ATLAS_MESH_TOPIC,
    PLUGIN_ATLAS_SPLAT_TOPIC,
};

const BEGIN: &str = "// <generated:golden-frames>";
const END: &str = "// </generated:golden-frames>";
const SESSION: &str = "atlas-drone-1-1000";
const BASE: &str = "http://198.51.100.5:8092/artifacts/job-1";

fn placement() -> Placement {
    Placement {
        anchor: Some(GeoAnchor {
            lat_deg: 12.9716,
            lon_deg: 77.5946,
            h_ellipsoid_m: 834.5,
            geoid_undulation_m: -86.25,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::GnssMean,
        }),
        r_enu_artifact: [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        t_enu_artifact: [1.5, -2.0, 0.25],
        scale: 1.0,
    }
}

fn splat() -> SplatDescriptor {
    SplatDescriptor {
        session_id: SESSION.into(),
        generation: 7,
        placement: placement(),
        gaussian_count: 1_250_000,
        step: 30_000,
        url: Some(format!("{BASE}/splat.ply")),
        handle: None,
        manifest_url: Some(format!("{BASE}/manifest.json")),
        lod_levels: 4,
    }
}

fn occupancy(field: OccupancyField) -> OccupancyDescriptor {
    let esdf = field == OccupancyField::Esdf;
    OccupancyDescriptor {
        session_id: SESSION.into(),
        generation: 7,
        placement: placement(),
        origin: [-16.5, -12.0, -4.0],
        resolution_m: 0.2,
        dims: [240, 180, 60],
        field,
        truncation_m: if esdf { 4.0 } else { 0.0 },
        shm_name: None,
        slot: None,
        seq: None,
        url: esdf.then(|| format!("{BASE}/esdf-g7.f32")),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn constant(out: &mut String, doc: &str, name: &str, bytes: &[u8]) {
    if !doc.is_empty() {
        out.push_str(&format!("/** {doc} */\n"));
    }
    out.push_str(&format!("export const {name} =\n  \"{}\";\n\n", hex(bytes)));
}

fn render() -> String {
    let cloud = PointCloudDescriptor {
        session_id: SESSION.into(),
        generation: 7,
        placement: placement(),
        point_count: 480_000,
        bounds: [-12.5, -8.0, 0.0, 31.25, 19.5, 42.75],
        shm_name: None,
        slot: None,
        seq: None,
        url: Some(format!("{BASE}/cloud.ply")),
    };
    let mesh = MeshDescriptor {
        session_id: SESSION.into(),
        generation: 7,
        placement: placement(),
        vertex_count: 90_000,
        face_count: 178_000,
        url: Some(format!("{BASE}/mesh.glb")),
        handle: None,
    };
    let status = CaptureStatus {
        session_id: SESSION.into(),
        state: CaptureState::Capturing,
        keyframes: 146,
        vio_health: VioHealth::Degraded,
        camera_count: 3,
        ingest_rate_hz: 9.75,
        capped: true,
        anchored: true,
        pose_tier: PoseSource::OffloadedSlam,
        dropped_keyframes: 4,
    };
    let splat_bytes = splat().to_msgpack().unwrap();
    let event = AtlasEvent::new(
        PLUGIN_ATLAS_SPLAT_TOPIC,
        Some("drone-1".into()),
        splat_bytes.clone(),
    );
    let bare = AtlasEvent::new(PLUGIN_ATLAS_MESH_TOPIC, None, vec![0xc0]);

    let mut out = format!(
        "{BEGIN} (ADOS_ATLAS_REGEN_GOLDEN=1 cargo test -p world-engine-protocol --test golden_frames)\n\n"
    );
    constant(&mut out, "", "GOLDEN_SPLAT_HEX", &splat_bytes);
    constant(
        &mut out,
        "",
        "GOLDEN_CLOUD_HEX",
        &cloud.to_msgpack().unwrap(),
    );
    constant(&mut out, "", "GOLDEN_MESH_HEX", &mesh.to_msgpack().unwrap());
    constant(
        &mut out,
        "",
        "GOLDEN_OCCUPANCY_ESDF_HEX",
        &occupancy(OccupancyField::Esdf).to_msgpack().unwrap(),
    );
    constant(
        &mut out,
        "The same grid published as a plain occupancy-probability buffer: `field` is\n * \"occupancy\", `truncation_m` is the meaningless zero, and `url` is nil.",
        "GOLDEN_OCCUPANCY_PLAIN_HEX",
        &occupancy(OccupancyField::Occupancy).to_msgpack().unwrap(),
    );
    constant(
        &mut out,
        "A framed envelope: the current version, splat topic, device \"drone-1\", the\n * splat descriptor as its payload.",
        "GOLDEN_EVENT_HEX",
        &event.encode().unwrap(),
    );
    constant(
        &mut out,
        "A local-bus envelope: `device_id` is skipped entirely (the only atlas field\n * carrying `skip_serializing_if`), so a decoder must read its absence as null\n * rather than failing the frame.",
        "GOLDEN_EVENT_NO_DEVICE_HEX",
        &bare.encode().unwrap(),
    );
    constant(
        &mut out,
        "A capture-status descriptor carrying the honesty fields (`capped`,\n * `anchored`, `pose_tier`, `dropped_keyframes`).",
        "GOLDEN_CAPTURE_STATUS_HEX",
        &status.to_msgpack().unwrap(),
    );
    out.push_str(END);
    out
}

#[test]
fn gcs_golden_frames_match_the_producer() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../gcs/tests/lib/atlas/golden-atlas-frames.ts");
    let file = std::fs::read_to_string(&path).unwrap();
    let start = file.find(BEGIN).expect("begin marker");
    let end = file.find(END).expect("end marker") + END.len();
    let rendered = render();
    if std::env::var_os("ADOS_ATLAS_REGEN_GOLDEN").is_some() {
        let updated = format!("{}{}{}", &file[..start], rendered, &file[end..]);
        std::fs::write(&path, updated).unwrap();
        return;
    }
    assert_eq!(
        &file[start..end],
        rendered,
        "golden-atlas-frames.ts is stale; regenerate it"
    );
}
