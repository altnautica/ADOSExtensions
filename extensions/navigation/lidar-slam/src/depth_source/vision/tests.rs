use std::collections::BTreeMap;

use ados_protocol::vision_depth_frame::{DepthFit, FRAME_COLS, FRAME_ROWS};
use serde_json::json;

use super::*;
use crate::depth_source::config::{resolve, DepthSettings};

const W: usize = FRAME_COLS as usize;
const H: usize = FRAME_ROWS as usize;

/// A synthetic engine frame: a wall 4 m ahead, σ 0.2 m, except the top-left
/// pixel, whose σ (2 m) is over half its depth.
fn bus_frame(camera: &str, inliers: u32, held_ms: u32) -> BusFrame {
    let mut sigma_mm = vec![200; W * H];
    sigma_mm[0] = 2000;
    BusFrame {
        camera_id: camera.into(),
        stamp_ns: 42,
        w: FRAME_COLS,
        h: FRAME_ROWS,
        fx: 110.0,
        fy: 83.0,
        cx: 64.0,
        cy: 48.0,
        pitch_down_deg: 20.0,
        depth_mm: vec![4000; W * H],
        sigma_mm,
        fit: DepthFit {
            scale: 0.2,
            shift: 0.05,
            inliers,
            residual_m: 0.3,
            held_ms,
        },
    }
}

/// The frame as the bus delivers it: the msgpack map, decoded.
fn payload(f: &BusFrame) -> rmpv::Value {
    rmpv::decode::read_value(&mut &f.to_msgpack()[..]).unwrap()
}

fn delivery(f: &BusFrame) -> rmpv::Value {
    rmpv::Value::Map(vec![
        ("topic".into(), "vision.depth.v2".into()),
        ("payload".into(), payload(f)),
    ])
}

fn cfg() -> VisionConfig {
    VisionConfig {
        camera: String::new(),
        max_fit_age_ms: 2000,
        extrinsic: Extrinsic {
            t: [0.1, 0.0, 0.0],
            ..Extrinsic::identity()
        },
        frame_mount: true,
    }
}

fn subscribed() -> Arc<Mailbox> {
    let m = Arc::new(Mailbox::default());
    m.set_subscribed(true);
    m
}

#[test]
fn a_bus_frame_decodes_into_a_mapped_frame() {
    let mailbox = subscribed();
    let mut src = VisionSource::open(mailbox.clone(), cfg()).unwrap();
    (mailbox.callback())(delivery(&bus_frame("main", 9, 0)));
    let f = src
        .next_frame(Duration::from_millis(100))
        .unwrap()
        .expect("a frame");
    assert_eq!((f.w, f.h, f.source_ns, f.seq), (128, 96, 42, 0));
    // Pixel centres move from i + 0.5 to i.
    assert_eq!((f.fx, f.fy, f.cx, f.cy), (110.0, 83.0, 63.5, 47.5));
    // The over-uncertain pixel is blanked, the rest kept.
    assert_eq!(f.depth_mm[0], 0);
    assert!(f.depth_mm[1..].iter().all(|&z| z == 4000));
    // The mount is the frame's pitch: the centre ray points 20° down.
    let mut pts = Vec::new();
    f.to_points(1, 20.0, &mut pts);
    let centre = pts[(H / 2 - 1) * W + W / 2 - 1];
    let down_deg = centre[2].atan2(centre[0] - 0.1).to_degrees();
    assert!((down_deg - 20.0).abs() < 0.5, "{centre:?}");
    assert_eq!(src.health().frames, 1);
}

#[test]
fn frames_without_a_metric_scale_are_counted_not_mapped() {
    let mailbox = subscribed();
    let mut src = VisionSource::open(mailbox.clone(), cfg()).unwrap();
    let wait = Duration::from_millis(20);
    for f in [bus_frame("main", 0, 0), bus_frame("main", 9, 5000)] {
        mailbox.put(payload(&f));
        assert!(src.next_frame(wait).unwrap().is_none());
    }
    let mut nan = bus_frame("main", 9, 0);
    nan.fit.scale = f32::NAN;
    mailbox.put(payload(&nan));
    assert!(src.next_frame(wait).unwrap().is_none());
    assert_eq!(src.health().unscaled, 3);
    assert_eq!(src.health().frames, 0);

    // Another camera is skipped; a malformed delivery is rejected.
    let only_main = VisionConfig {
        camera: "main".into(),
        ..cfg()
    };
    let mut src = VisionSource::open(mailbox.clone(), only_main).unwrap();
    mailbox.put(payload(&bus_frame("aux", 9, 0)));
    assert!(src.next_frame(wait).unwrap().is_none());
    mailbox.put(rmpv::Value::from("not a frame"));
    assert!(src.next_frame(wait).unwrap().is_none());
    assert_eq!((src.health().unscaled, src.health().rejected), (0, 1));
}

#[test]
fn the_newest_frame_wins_and_older_ones_count_as_drops() {
    let mailbox = subscribed();
    let mut src = VisionSource::open(mailbox.clone(), cfg()).unwrap();
    for stamp in [1, 2, 3] {
        let mut f = bus_frame("main", 9, 0);
        f.stamp_ns = stamp;
        mailbox.put(payload(&f));
    }
    let f = src.next_frame(Duration::ZERO).unwrap().expect("a frame");
    assert_eq!(f.source_ns, 3);
    assert_eq!(src.health().drops, 2);
}

#[test]
fn the_source_is_unavailable_until_subscribed_and_configures() {
    let Err(e) = VisionSource::open(Arc::new(Mailbox::default()), cfg()) else {
        panic!("opened without a subscription");
    };
    assert!(e.to_string().contains("vision.depth.read"), "{e}");

    let values = BTreeMap::from([(
        "navigation.depth".to_string(),
        json!({"source": "vision", "vision_camera": "main"}),
    )]);
    let s: DepthSettings = resolve(&values, "navigation.depth").unwrap();
    assert_eq!(s.source, DepthSourceKind::Vision);
    assert_eq!(
        (s.vision_camera.as_str(), s.vision_max_fit_age_ms),
        ("main", 2000)
    );
    let over = BTreeMap::from([(
        "navigation.depth".to_string(),
        json!({"source": "vision", "vision_max_fit_age_ms": 70000}),
    )]);
    assert!(resolve(&over, "navigation.depth")
        .unwrap_err()
        .reason
        .contains("vision_max_fit_age_ms"));
}
