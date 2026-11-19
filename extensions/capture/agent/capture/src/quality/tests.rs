use super::*;

/// Host time the tests read the chips at (every test frame is fresh).
const NOW: u64 = 2_000_000_000;
const MS: u64 = 1_000_000;

fn level(h: &[HealthView], id: &str) -> String {
    h.iter().find(|c| c.id == id).unwrap().level.clone()
}

fn recording() -> QualityMonitor {
    QualityMonitor {
        recording: true,
        ..QualityMonitor::new()
    }
}

fn fq(blur: f64, corners: usize) -> FrameQuality {
    FrameQuality {
        blur,
        corners,
        mean: 100.0,
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
    // 120 after a camera averaging 300: 0.4 of its usual sharpness.
    m.on_frame("cam_fl", 0, fq(300.0, 200), None);
    m.on_frame("cam_fl", 100 * MS, fq(120.0, 200), None);
    m.on_frame("cam_fl", 600 * MS, fq(120.0, 200), None);
    assert_eq!(level(&m.health(NOW), "blur"), "ok");
    m.on_frame("cam_fl", 1_100 * MS, fq(120.0, 200), None);
    let h = m.health(NOW);
    assert_eq!(level(&h, "blur"), "warn");
    assert!((h[0].value.unwrap() - 0.4).abs() < 0.02, "{:?}", h[0].value);
    // A sharp frame clears it at once; the session flag stays raised.
    m.on_frame("cam_fl", 1_200 * MS, fq(300.0, 200), None);
    assert_eq!(level(&m.health(NOW), "blur"), "ok");
    assert!(m.raised.blur);
}

#[test]
fn motion_blur_chip_thresholds() {
    // 500 px focal length, 4 ms exposure: 1 rad/s smears 2 px.
    let optics = Optics {
        fx_px: 500.0,
        exposure_s: 0.004,
    };
    assert!((predicted_blur_px(1.0, optics) - 2.0).abs() < 1e-12);
    let run = |rate: f64| {
        let mut m = QualityMonitor::new();
        for k in 0..12u64 {
            let t = k * 100 * MS;
            m.on_gyro(t, &[0.0, 0.0, rate]);
            m.on_frame("cam_fl", t, fq(300.0, 200), Some(optics));
        }
        level(&m.health(1_100 * MS), "blur")
    };
    assert_eq!(run(0.3), "ok", "0.6 px");
    assert_eq!(run(0.75), "warn", "1.5 px");
    assert_eq!(run(1.5), "bad", "3.0 px");

    // A stale gyro sample predicts nothing; without optics nothing either.
    let mut m = QualityMonitor::new();
    m.on_gyro(0, &[0.0, 0.0, 5.0]);
    assert_eq!(m.motion_blur_px(GYRO_STALE_NS + 1, optics), None);
    for k in 6..20u64 {
        m.on_frame("cam_fl", k * 100 * MS, fq(300.0, 200), Some(optics));
    }
    assert_eq!(level(&m.health(1_900 * MS), "blur"), "ok");
    m.on_gyro(0, &[0.0, 0.0, 5.0]);
    for k in 0..12u64 {
        m.on_frame("cam_fr", k * 100 * MS, fq(300.0, 200), None);
    }
    assert_eq!(m.cameras["cam_fr"].levels.0, Level::Ok);

    // The Laplacian score counts relative to the camera's own average: a
    // bare wall from the start is not blur, a sudden drop to a quarter is.
    let mut m = QualityMonitor::new();
    for k in 0..12u64 {
        m.on_frame("cam_fl", k * 100 * MS, fq(15.0, 200), None);
    }
    assert_eq!(level(&m.health(1_100 * MS), "blur"), "ok");
    for k in 12..24u64 {
        m.on_frame("cam_fl", k * 100 * MS, fq(3.75, 200), None);
    }
    assert_eq!(level(&m.health(2_300 * MS), "blur"), "bad");
}

#[test]
fn texture_reports_the_worst_camera() {
    let mut m = QualityMonitor::new();
    for k in 0..12u64 {
        let t = k * 100 * MS;
        m.on_frame("cam_fl", t, fq(300.0, 10), None);
        m.on_frame("cam_fr", t, fq(300.0, 300), None);
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
            None,
        );
    }
    assert_eq!(level(&m.health(NOW), "flicker"), "bad");
    assert!(m.raised.flicker);
}

#[test]
fn issues_while_not_recording_raise_no_session_flag() {
    let mut m = QualityMonitor::new();
    m.on_frame("cam_fl", 0, fq(300.0, 300), None);
    let blurred = fq(5.0, 5);
    for k in 1..15u64 {
        m.on_frame("cam_fl", k * 100 * MS, blurred, None);
    }
    m.on_pose_quality(Some(0.01), None);
    let h = m.health(1_400 * MS);
    assert_eq!(level(&h, "blur"), "bad", "the chip still shows it");
    assert_eq!(m.raised, SessionFlags::default());
    m.recording = true;
    m.on_frame("cam_fl", 1_500 * MS, blurred, None);
    assert!(m.raised.blur && m.raised.texture);
}

#[test]
fn a_silent_camera_leaves_the_chips() {
    let mut m = QualityMonitor::new();
    for k in 0..12u64 {
        m.on_frame("cam_fl", k * 100 * MS, fq(300.0, 5), None);
    }
    for k in 0..50u64 {
        m.on_frame("cam_fr", k * 100 * MS, fq(300.0, 300), None);
    }
    // cam_fl's last frame is at 1.1 s.
    assert_eq!(level(&m.health(3_000 * MS), "texture"), "bad");
    let h = m.health(3_200 * MS);
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
    m.on_frame("cam_fl", 0, fq(300.0, 300), None);
    // The only camera went silent: back to no measurement.
    assert_eq!(level(&m.health(CAMERA_STALE_NS + 1), "blur"), "warn");
}
