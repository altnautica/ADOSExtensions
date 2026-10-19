use super::*;

fn level(h: &[HealthView], id: &str) -> String {
    h.iter().find(|c| c.id == id).unwrap().level.clone()
}

#[test]
fn chips_come_in_display_order_with_the_six_ids() {
    let mut m = QualityMonitor::new();
    let ids: Vec<String> = m.health().into_iter().map(|c| c.id).collect();
    assert_eq!(
        ids,
        ["blur", "texture", "flicker", "corridor", "storage", "clock"]
    );
}

#[test]
fn blur_only_raises_after_it_holds_for_a_second() {
    let mut m = QualityMonitor::new();
    let blurred = FrameQuality {
        blur: 30.0,
        corners: 200,
        mean: 100.0,
    };
    m.on_frame("cam_fl", 0, blurred);
    m.on_frame("cam_fl", 500_000_000, blurred);
    assert_eq!(level(&m.health(), "blur"), "ok");
    m.on_frame("cam_fl", 1_000_000_000, blurred);
    assert_eq!(level(&m.health(), "blur"), "warn");
    // A sharp frame clears it at once; the session flag stays raised.
    m.on_frame(
        "cam_fl",
        1_100_000_000,
        FrameQuality {
            blur: 300.0,
            corners: 200,
            mean: 100.0,
        },
    );
    assert_eq!(level(&m.health(), "blur"), "ok");
    assert!(m.raised.blur);
}

#[test]
fn texture_reports_the_worst_camera() {
    let mut m = QualityMonitor::new();
    for k in 0..12u64 {
        let t = k * 100_000_000;
        m.on_frame(
            "cam_fl",
            t,
            FrameQuality {
                blur: 300.0,
                corners: 10,
                mean: 100.0,
            },
        );
        m.on_frame(
            "cam_fr",
            t,
            FrameQuality {
                blur: 300.0,
                corners: 300,
                mean: 100.0,
            },
        );
    }
    let h = m.health();
    assert_eq!(level(&h, "texture"), "bad");
    assert_eq!(h[1].value, Some(10.0));
}

#[test]
fn corridor_follows_the_degeneracy_ratio() {
    let mut m = QualityMonitor::new();
    m.on_pose_quality(Some(0.5), Some(140));
    assert_eq!(level(&m.health(), "corridor"), "ok");
    m.on_pose_quality(Some(0.1), None);
    assert_eq!(level(&m.health(), "corridor"), "warn");
    m.on_pose_quality(Some(0.02), None);
    assert_eq!(level(&m.health(), "corridor"), "bad");
    assert!(m.raised.corridor);
}

#[test]
fn clock_and_storage_chips() {
    let mut m = QualityMonitor::new();
    assert_eq!(level(&m.health(), "clock"), "bad", "no clock service");
    m.clock = ClockReading {
        connected: true,
        locked: true,
        sigma_ns: 420_000,
    };
    let h = m.health();
    assert_eq!(level(&h, "clock"), "ok");
    assert_eq!(h[5].value, Some(0.42));
    m.clock.sigma_ns = 5_000_000;
    assert_eq!(level(&m.health(), "clock"), "warn");
    m.storage = StorageReading {
        free_pct: Some(3.0),
        level: Level::Bad,
    };
    assert_eq!(level(&m.health(), "storage"), "bad");
}

#[test]
fn flicker_chip_from_the_frame_means() {
    let mut m = QualityMonitor::new();
    for i in 0..60u64 {
        let t = i as f64 / 30.0;
        let mean = 100.0 * (1.0 + 0.2 * (std::f64::consts::TAU * 10.0 * t).sin());
        m.on_frame(
            "cam_fl",
            (t * 1e9) as u64,
            FrameQuality {
                blur: 300.0,
                corners: 300,
                mean,
            },
        );
    }
    assert_eq!(level(&m.health(), "flicker"), "bad");
    assert!(m.raised.flicker);
}
