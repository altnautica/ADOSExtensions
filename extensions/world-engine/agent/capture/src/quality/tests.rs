use super::*;

/// Host time the tests read the chips at (every test frame is fresh).
const NOW: u64 = 2_000_000_000;

fn level(h: &[HealthView], id: &str) -> String {
    h.iter().find(|c| c.id == id).unwrap().level.clone()
}

fn recording() -> QualityMonitor {
    QualityMonitor {
        recording: true,
        ..QualityMonitor::new()
    }
}

#[test]
fn chips_come_in_display_order_with_the_six_ids() {
    let mut m = QualityMonitor::new();
    let ids: Vec<String> = m.health(NOW).into_iter().map(|c| c.id).collect();
    assert_eq!(
        ids,
        ["blur", "texture", "flicker", "corridor", "storage", "clock"]
    );
}

#[test]
fn blur_only_raises_after_it_holds_for_a_second() {
    let mut m = recording();
    let blurred = FrameQuality {
        blur: 30.0,
        corners: 200,
        mean: 100.0,
    };
    m.on_frame("cam_fl", 0, blurred);
    m.on_frame("cam_fl", 500_000_000, blurred);
    assert_eq!(level(&m.health(NOW), "blur"), "ok");
    m.on_frame("cam_fl", 1_000_000_000, blurred);
    assert_eq!(level(&m.health(NOW), "blur"), "warn");
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
    assert_eq!(level(&m.health(NOW), "blur"), "ok");
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
    let h = m.health(NOW);
    assert_eq!(level(&h, "texture"), "bad");
    assert_eq!(h[1].value, Some(10.0));
}

#[test]
fn corridor_follows_the_degeneracy_ratio() {
    let mut m = recording();
    m.on_pose_quality(Some(0.5), Some(140));
    assert_eq!(level(&m.health(NOW), "corridor"), "ok");
    m.on_pose_quality(Some(0.1), None);
    assert_eq!(level(&m.health(NOW), "corridor"), "warn");
    m.on_pose_quality(Some(0.02), None);
    assert_eq!(level(&m.health(NOW), "corridor"), "bad");
    assert!(m.raised.corridor);
}

#[test]
fn clock_and_storage_chips() {
    let mut m = QualityMonitor::new();
    assert_eq!(level(&m.health(NOW), "clock"), "bad", "no clock service");
    m.clock = ClockReading {
        connected: true,
        locked: true,
        sigma_ns: 420_000,
    };
    let h = m.health(NOW);
    assert_eq!(level(&h, "clock"), "ok");
    assert_eq!(h[5].value, Some(0.42));
    m.clock.sigma_ns = 5_000_000;
    assert_eq!(level(&m.health(NOW), "clock"), "warn");
    m.storage = StorageReading {
        free_pct: Some(3.0),
        level: Level::Bad,
    };
    assert_eq!(level(&m.health(NOW), "storage"), "bad");
}

#[test]
fn flicker_chip_from_the_frame_means() {
    let mut m = recording();
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
    assert_eq!(level(&m.health(NOW), "flicker"), "bad");
    assert!(m.raised.flicker);
}

#[test]
fn issues_while_not_recording_raise_no_session_flag() {
    let mut m = QualityMonitor::new();
    let blurred = FrameQuality {
        blur: 5.0,
        corners: 5,
        mean: 100.0,
    };
    for k in 0..15u64 {
        m.on_frame("cam_fl", k * 100_000_000, blurred);
    }
    m.on_pose_quality(Some(0.01), None);
    let h = m.health(1_400_000_000);
    assert_eq!(level(&h, "blur"), "bad", "the chip still shows it");
    assert_eq!(m.raised, SessionFlags::default());
    m.recording = true;
    m.on_frame("cam_fl", 1_500_000_000, blurred);
    assert!(m.raised.blur && m.raised.texture);
}

#[test]
fn a_silent_camera_leaves_the_chips() {
    let mut m = QualityMonitor::new();
    for k in 0..12u64 {
        let t = k * 100_000_000;
        m.on_frame(
            "cam_fl",
            t,
            FrameQuality {
                blur: 300.0,
                corners: 5,
                mean: 100.0,
            },
        );
    }
    for k in 0..50u64 {
        m.on_frame(
            "cam_fr",
            k * 100_000_000,
            FrameQuality {
                blur: 300.0,
                corners: 300,
                mean: 100.0,
            },
        );
    }
    // cam_fl's last frame is at 1.1 s.
    assert_eq!(level(&m.health(3_000_000_000), "texture"), "bad");
    let h = m.health(3_200_000_000);
    assert_eq!(level(&h, "texture"), "ok");
    assert_eq!(h[1].value, Some(300.0));
}

#[test]
fn no_live_camera_warns_instead_of_reading_ok() {
    let mut m = QualityMonitor::new();
    let h = m.health(NOW);
    for id in ["blur", "texture"] {
        let chip = h.iter().find(|c| c.id == id).unwrap();
        assert_eq!(chip.level, "warn", "{id} with nothing measuring");
        assert_eq!(chip.value, None);
    }
    m.on_frame(
        "cam_fl",
        0,
        FrameQuality {
            blur: 300.0,
            corners: 300,
            mean: 100.0,
        },
    );
    // The only camera went silent: back to no measurement.
    assert_eq!(level(&m.health(CAMERA_STALE_NS + 1), "blur"), "warn");
}
