use std::collections::BTreeMap;

use serde_json::{json, Value};

use super::config::{resolve, DepthSourceKind};
use super::wire::{self, Header, Reassembler, WireError, CHUNK, HEADER_LEN};
use super::*;

const KEY: &str = "navigation.depth";

fn head(seq: u32, w: u16, h: u16) -> Header {
    Header {
        seq,
        sim_ns: 1_000_000 * u64::from(seq),
        w,
        h,
        fx: 84.3024,
        fy: 108.2429,
        cx: 80.0,
        cy: 60.0,
        frag: 0,
        frags: 0,
        offset: 0,
    }
}

/// The datagrams of one synthetic frame.
fn datagrams(seq: u32, w: u16, h: u16) -> (Vec<u16>, Vec<Vec<u8>>) {
    let depth: Vec<u16> = (0..usize::from(w) * usize::from(h))
        .map(|i| (i as u32 * 7 + seq) as u16)
        .collect();
    let mut out = Vec::new();
    let mut buf = Vec::new();
    wire::for_each_datagram(&head(seq, w, h), &depth, &mut buf, |d| out.push(d.to_vec()));
    (depth, out)
}

#[test]
fn header_parses_the_world_engine_layout() {
    let (_, dgs) = datagrams(7, 160, 120);
    assert_eq!(dgs.len(), (160 * 120 * 2usize).div_ceil(CHUNK));
    assert!(dgs.iter().all(|d| d.len() <= wire::MAX_DATAGRAM));
    let (h, chunk) = wire::parse(&dgs[1]).unwrap();
    assert_eq!((h.seq, h.sim_ns, h.w, h.h), (7, 7_000_000, 160, 120));
    assert_eq!((h.fx, h.fy, h.cx, h.cy), (84.3024, 108.2429, 80.0, 60.0));
    assert_eq!(
        (h.frag, h.frags, h.offset),
        (1, dgs.len() as u16, CHUNK as u32)
    );
    assert_eq!(chunk.len(), CHUNK);
    // Field offsets as the sender writes them.
    assert_eq!(&dgs[1][..10], b"ADOSDEPTH1");
    assert_eq!(
        u32::from_le_bytes(dgs[1][46..50].try_into().unwrap()),
        CHUNK as u32
    );

    assert_eq!(
        wire::parse(&dgs[0][..HEADER_LEN - 1]),
        Err(WireError::Short(49))
    );
    let mut bad = dgs[0].clone();
    bad[0] = b'X';
    assert_eq!(wire::parse(&bad), Err(WireError::Magic));
    let mut bad = dgs[0].clone();
    bad[22..24].copy_from_slice(&0u16.to_le_bytes());
    assert_eq!(wire::parse(&bad), Err(WireError::Geometry));
    let mut bad = dgs[0].clone();
    bad[26..30].copy_from_slice(&f32::NAN.to_le_bytes());
    assert_eq!(wire::parse(&bad), Err(WireError::Geometry));
    let mut bad = dgs[0].clone();
    bad[42..44].copy_from_slice(&999u16.to_le_bytes());
    assert_eq!(wire::parse(&bad), Err(WireError::Fragment));
    let last = dgs.last().unwrap();
    let mut bad = last.clone();
    bad.extend_from_slice(&[0; CHUNK]);
    assert_eq!(
        wire::parse(&bad),
        Err(WireError::Fragment),
        "past the payload end"
    );
}

#[test]
fn reassembles_in_order_and_out_of_order() {
    let mut r = Reassembler::default();
    let (depth, dgs) = datagrams(1, 160, 120);
    let mut got = None;
    for d in &dgs {
        got = got.or(r.push(d));
    }
    let a = got.unwrap();
    assert_eq!((a.head.seq, a.depth_mm.as_slice()), (1, depth.as_slice()));

    // Reversed, with a duplicate.
    let (depth2, mut dgs2) = datagrams(2, 160, 120);
    dgs2.reverse();
    dgs2.insert(3, dgs2[0].clone());
    let done: Vec<_> = dgs2.iter().filter_map(|d| r.push(d)).collect();
    assert_eq!(done.len(), 1);
    assert_eq!(done[0].depth_mm, depth2);
    assert_eq!(r.stats.stale, 1, "the duplicate");
    // A straggler of a finished frame changes nothing.
    assert!(r.push(&dgs[0]).is_none());
    assert_eq!(
        (r.stats.frames, r.stats.incomplete, r.stats.rejected),
        (2, 0, 0)
    );
}

#[test]
fn a_fragment_overtaken_by_the_next_frame_still_completes() {
    let mut r = Reassembler::default();
    let (d3, a) = datagrams(3, 160, 120);
    let (d4, b) = datagrams(4, 160, 120);
    // Frame 3 minus its last datagram, frame 4's first, then 3's last.
    let mut done = Vec::new();
    for d in a[..a.len() - 1]
        .iter()
        .chain(&b[..1])
        .chain(&a[a.len() - 1..])
    {
        done.extend(r.push(d));
    }
    for d in &b[1..] {
        done.extend(r.push(d));
    }
    let seqs: Vec<u32> = done.iter().map(|f| f.head.seq).collect();
    assert_eq!(seqs, [3, 4]);
    assert_eq!(
        (done[0].depth_mm.clone(), done[1].depth_mm.clone()),
        (d3, d4)
    );
}

#[test]
fn loss_drops_the_frame_and_the_next_ones_still_arrive() {
    let mut r = Reassembler::default();
    let (_, a) = datagrams(10, 160, 120);
    let (d11, b) = datagrams(11, 160, 120);
    for (i, d) in a.iter().enumerate() {
        if i != 5 {
            assert!(r.push(d).is_none());
        }
    }
    let done: Vec<_> = b.iter().filter_map(|d| r.push(d)).collect();
    assert_eq!(done.len(), 1);
    assert_eq!(done[0].depth_mm, d11);
    assert_eq!(r.stats.incomplete, 1, "frame 10 dropped once 11 completed");
    // Frame 10's missing datagram, late: stale, not a resurrection.
    assert!(r.push(&a[5]).is_none());

    // Three frames in flight: the oldest incomplete one is evicted.
    let (_, c) = datagrams(12, 160, 120);
    let (_, d) = datagrams(13, 160, 120);
    let (d14, e) = datagrams(14, 160, 120);
    r.push(&c[0]);
    r.push(&d[0]);
    let done: Vec<_> = e.iter().filter_map(|x| r.push(x)).collect();
    assert_eq!(done[0].depth_mm, d14);
    assert_eq!(r.stats.incomplete, 3);
}

#[test]
fn a_restarted_stream_is_picked_up() {
    let mut r = Reassembler::default();
    for d in datagrams(500, 16, 12).1 {
        r.push(&d);
    }
    let done: Vec<_> = datagrams(0, 16, 12)
        .1
        .iter()
        .filter_map(|d| r.push(d))
        .collect();
    assert_eq!(
        done.len(),
        1,
        "seq 0 after 500 is a new run, not a stale frame"
    );
}

#[test]
fn inconsistent_fragments_are_rejected() {
    let mut r = Reassembler::default();
    let (_, a) = datagrams(1, 160, 120);
    let (_, b) = datagrams(1, 80, 60);
    r.push(&a[0]);
    assert!(r.push(&b[1]).is_none());
    assert_eq!(r.stats.rejected, 1, "same seq, other geometry");
}

/// A camera 0.2 m ahead of and 0.1 m above the body origin, pitched 30°
/// down, looking at level ground 2 m below the body origin: every point lands
/// on that ground plane.
#[test]
fn points_of_a_tilted_camera_land_on_the_plane() {
    let (w, h) = (64u32, 48u32);
    let (fx, fy, cx, cy) = (40.0f32, 40.0f32, 31.5f32, 23.5f32);
    let t = [0.2f32, 0.0, -0.1];
    let ext = Extrinsic::from_rpy_deg(t, [0.0, -30.0, 0.0]);
    let plane_d = 2.0f32; // body z of the ground (down positive)
    let [ox, oy, oz] = ext.optical_to_body();
    // Depth along the optical axis of each pixel's ray to the plane.
    let mut depth_mm = vec![0u16; (w * h) as usize];
    for v in 0..h {
        for u in 0..w {
            let (xn, yn) = ((u as f32 - cx) / fx, (v as f32 - cy) / fy);
            let dz = ox[2] * xn + oy[2] * yn + oz[2];
            if dz <= 0.05 {
                continue; // at or above the horizon: no return
            }
            let z = (plane_d - t[2]) / dz;
            depth_mm[(v * w + u) as usize] = (z * 1000.0).round() as u16;
        }
    }
    let frame = DepthFrame {
        stamp_ns: 1,
        source_ns: 1,
        seq: 1,
        w,
        h,
        fx,
        fy,
        cx,
        cy,
        depth_mm,
        extrinsic: ext,
    };
    let mut pts = Vec::with_capacity(4096);
    let cap = pts.capacity();
    frame.to_points(1, 100.0, &mut pts);
    assert!(pts.len() > 1000);
    assert_eq!(
        pts.capacity(),
        cap,
        "no reallocation into a big enough buffer"
    );
    for p in &pts {
        assert!((p[2] - plane_d).abs() < 0.01, "{p:?} off the plane");
        assert!(p[0] > t[0], "{p:?} behind a forward camera");
    }
    // The optical centre's ray: straight ahead, 30° down.
    let mut centre = frame.clone();
    centre.depth_mm.fill(0);
    let z = (plane_d - t[2]) / oz[2];
    centre.depth_mm[(24 * w + 32) as usize] = (z * 1000.0).round() as u16;
    centre.cx = 32.0;
    centre.cy = 24.0;
    centre.to_points(1, 100.0, &mut pts);
    assert_eq!(pts.len(), 1);
    let along = (plane_d - t[2]) / 30f32.to_radians().tan();
    assert!((pts[0][0] - (t[0] + along)).abs() < 0.01, "{:?}", pts[0]);
    assert!(pts[0][1].abs() < 1e-3);

    // Stride and range limit.
    frame.to_points(4, 100.0, &mut pts);
    let strided = pts.len();
    frame.to_points(1, 100.0, &mut pts);
    let full = pts.len();
    assert!(strided * 12 < full && strided * 20 > full);
    frame.to_points(1, 3.0, &mut pts);
    let near = pts.clone();
    assert!(!near.is_empty() && near.len() < full);
    for p in near {
        let r = ((p[0] - t[0]).powi(2) + (p[1] - t[1]).powi(2) + (p[2] - t[2]).powi(2)).sqrt();
        assert!(r <= 3.0 + 1e-3);
    }
}

#[test]
fn identity_mount_maps_optical_axes_to_frd() {
    let [ox, oy, oz] = Extrinsic::identity().optical_to_body();
    assert_eq!(
        (ox, oy, oz),
        ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    );
    // Pitch −90: the optical axis points down.
    let [_, _, oz] = Extrinsic::from_rpy_deg([0.0; 3], [0.0, -90.0, 0.0]).optical_to_body();
    assert!((oz[2] - 1.0).abs() < 1e-6 && oz[0].abs() < 1e-6);
    // The same mount as a quaternion.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let q = Extrinsic::from_quat_wxyz([0.0; 3], [h, 0.0, -h, 0.0]).optical_to_body();
    assert!((q[2][2] - 1.0).abs() < 1e-6);
}

fn depth(v: Value) -> Result<DepthSettings, ados_mapping_pose::config::ConfigError> {
    resolve(&BTreeMap::from([(KEY.to_string(), v)]), KEY)
}

#[test]
fn config_defaults_off_and_validates() {
    let d = resolve(&BTreeMap::new(), KEY).unwrap();
    assert_eq!(d.source, DepthSourceKind::Off);
    assert_eq!(d.extrinsic(), Extrinsic::identity());
    assert!(open(&d).unwrap().is_none());

    let we = depth(json!({
        "source": "we", "we_host": "192.0.2.1", "we_port": 7811,
        "extrinsic": {"translation_m": [0.13, 0.0, 0.0], "rpy_deg": [0.0, -20.0, 0.0]}
    }))
    .unwrap();
    assert_eq!((we.source, we.we_port), (DepthSourceKind::We, 7811));
    assert_eq!(we.extrinsic().t, [0.13, 0.0, 0.0]);

    let rs = depth(json!({"source": "realsense", "realsense": {"width": 640, "fps": 15}})).unwrap();
    assert_eq!((rs.realsense.width, rs.realsense.height), (640, 480));

    for (v, field) in [
        (json!({"source": "kinect"}), "unknown variant"),
        (json!({"sorce": "we"}), "unknown field"),
        (json!({"source": "we", "we_port": 0}), "we_port"),
        (json!({"source": "we", "we_host": " "}), "we_host"),
        (
            json!({"source": "realsense", "realsense": {"width": 1280, "height": 720}}),
            "width",
        ),
        (
            json!({"source": "realsense", "realsense": {"fps": 60}}),
            "fps",
        ),
        (
            json!({"extrinsic": {"rpy_deg": [0, 0, 0], "quat_wxyz": [1, 0, 0, 0]}}),
            "not both",
        ),
        (
            json!({"extrinsic": {"quat_wxyz": [0, 0, 0, 0]}}),
            "unit quaternion",
        ),
        (json!({"extrinsic": {"translation_m": [50, 0, 0]}}), "10 m"),
    ] {
        let e = depth(v.clone()).unwrap_err();
        assert_eq!(e.key, KEY);
        assert!(e.reason.contains(field), "{v}: {}", e.reason);
    }
}

#[test]
fn a_missing_realsense_library_is_unavailable() {
    let s = DepthSettings {
        source: DepthSourceKind::Realsense,
        realsense: config::RealsenseSettings {
            library: "/nonexistent/librealsense2.so".into(),
            ..Default::default()
        },
        ..Default::default()
    };
    let Err(e) = open(&s) else {
        panic!("opened without a library");
    };
    assert!(matches!(e, DepthSourceError::Unavailable(_)), "{e}");
    let st = DepthStatus::unavailable(DepthSourceKind::Realsense, &e);
    assert_eq!(st.state, DepthState::Unavailable);
    assert!(st.reason.unwrap().contains("librealsense2"));
}

/// With `ADOS_TEST_LIBREALSENSE2` naming a built library and no camera on
/// the bus, the driver loads every symbol and reports no camera.
#[cfg(unix)]
#[test]
fn a_real_library_without_a_camera_is_unavailable() {
    let Some(lib) = std::env::var_os("ADOS_TEST_LIBREALSENSE2") else {
        return;
    };
    let cfg = realsense::RealsenseConfig {
        library: lib.into(),
        width: 848,
        height: 480,
        fps: 30,
        serial: String::new(),
        extrinsic: Extrinsic::identity(),
    };
    match realsense::RealsenseSource::open(&cfg) {
        Err(DepthSourceError::Unavailable(m)) => assert!(m.contains("no RealSense camera"), "{m}"),
        Err(e) => panic!("{e}"),
        Ok(_) => panic!("a camera is attached; this test expects none"),
    }
}
