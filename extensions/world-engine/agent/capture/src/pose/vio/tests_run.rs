use std::os::unix::fs::PermissionsExt;
use std::time::{Duration, Instant};

use ados_capture_contract::CameraModelModel;
use ados_protocol::framebus::FrameFormat;
use world_engine_protocol::atlas::VioHealth;

use super::pairing::{PairOutcome, StereoPairer};
use super::tests::{assert_close, mat4_inv, mat4_mul, rig, POSE, R_Z90};
use super::*;
use crate::pose::{se3, PoseProvider};

fn frame(
    cam: &str,
    ts_ns: u64,
    trigger_seq: Option<u64>,
    format: FrameFormat,
    fill: u8,
) -> CapturedFrame {
    CapturedFrame {
        camera_id: cam.into(),
        ts_ms: 0,
        ts_ns,
        ts_sensor_ns: None,
        timebase: 0,
        trigger_seq,
        exposure_ns: None,
        seq: 0,
        width: 4,
        height: 2,
        format,
        pixels: vec![fill; 8],
    }
}

fn paired(out: PairOutcome<'_>) -> (u64, Vec<u8>, Vec<u8>) {
    match out {
        PairOutcome::Paired(p) => (p.t_ns, p.left.into_owned(), p.right.into_owned()),
        other => panic!("expected a pair, got {other:?}"),
    }
}

#[test]
fn pairs_by_trigger_seq_before_time() {
    let mut p = StereoPairer::new("cam_l", "cam_r");
    let l = frame("cam_l", 1_000_000_000, Some(7), FrameFormat::Gray8, 1);
    let r = frame("cam_r", 1_005_000_000, Some(7), FrameFormat::Gray8, 2);
    assert!(matches!(p.push(&l), PairOutcome::Held));
    let (t, left, right) = paired(p.push(&r));
    assert_eq!((t, left[0], right[0]), (1_000_000_000, 1, 2));
    // Same time, different trigger: not a pair.
    let l = frame("cam_l", 2_000_000_000, Some(8), FrameFormat::Gray8, 1);
    let r = frame("cam_r", 2_000_000_000, Some(9), FrameFormat::Gray8, 2);
    assert!(matches!(p.push(&l), PairOutcome::Held));
    assert!(matches!(p.push(&r), PairOutcome::Held));
}

#[test]
fn pairs_untriggered_within_one_ms() {
    let mut p = StereoPairer::new("cam_l", "cam_r");
    // Right first: the pair is still (left, right) at the left time.
    assert!(matches!(
        p.push(&frame("cam_r", 5_000_900_000, None, FrameFormat::Gray8, 2)),
        PairOutcome::Held
    ));
    let (t, left, right) =
        paired(p.push(&frame("cam_l", 5_000_000_000, None, FrameFormat::Gray8, 1)));
    assert_eq!((t, left[0], right[0]), (5_000_000_000, 1, 2));
    assert!(matches!(
        p.push(&frame("cam_l", 6_000_000_000, None, FrameFormat::Gray8, 1)),
        PairOutcome::Held
    ));
    assert!(matches!(
        p.push(&frame("cam_r", 6_001_000_001, None, FrameFormat::Gray8, 2)),
        PairOutcome::Held
    ));
    // One side triggered, the other not: time decides.
    paired(p.push(&frame(
        "cam_l",
        6_001_500_000,
        Some(3),
        FrameFormat::Gray8,
        1,
    )));
}

#[test]
fn pairing_rejects_non_gray_and_ignores_other_cameras() {
    let mut p = StereoPairer::new("cam_l", "cam_r");
    assert!(matches!(
        p.push(&frame("cam_l", 1, None, FrameFormat::Nv12, 1)),
        PairOutcome::Rejected
    ));
    assert!(matches!(
        p.push(&frame("down", 1, None, FrameFormat::Gray8, 1)),
        PairOutcome::Ignored
    ));
    let mut short = frame("cam_r", 1, None, FrameFormat::Gray8, 1);
    short.pixels.pop();
    assert!(matches!(p.push(&short), PairOutcome::Rejected));
}

async fn wait_for(what: &str, mut cond: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !cond() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn supervises_and_restarts_a_fake_child() {
    let dir = tempfile::tempdir().unwrap();
    let runs = dir.path().join("runs");
    let script = dir.path().join("fake-basalt.sh");
    std::fs::write(
        &script,
        format!(
            "#!/bin/sh\n[ -f \"$2\" ] && [ -f \"$3\" ] || exit 2\necho run >> '{}'\n\
             echo '{{\"status\":\"initializing\"}}'\nsleep 0.2\necho '{{\"status\":\"tracking\"}}'\n\
             echo '{}'\necho 'basalt-ados: log line' >&2\nsleep 0.5\necho '{{\"status\":\"lost\"}}'\n\
             sleep 0.3\nexit 3\n",
            runs.display(),
            POSE
        ),
    )
    .unwrap();
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    let vio = VioPose::spawn(VioConfig {
        binary: script,
        rig: rig(CameraModelModel::Kb4, vec![0.0; 4], None),
        left_id: "cam_l".into(),
        right_id: "cam_r".into(),
        imu_id: "imu0".into(),
        work_dir: dir.path().join("work"),
    })
    .unwrap();
    assert!(dir.path().join("work").join(CALIB_FILE).is_file());
    // Inputs never block, child connected or not.
    for i in 0..10 {
        vio.feed_frame(&frame("cam_l", i, Some(i), FrameFormat::Gray8, 1));
        vio.feed_frame(&frame("cam_r", i, Some(i), FrameFormat::Gray8, 2));
    }
    wait_for("first pose", || vio.latest().is_some()).await;
    assert_eq!(vio.health(), VioHealth::Good);
    assert_eq!(vio.quality().tracked_features, Some(42));
    // Body pose = T_w_i * inv(T_body_imu).
    let t_imu_body = mat4_inv(&se3::to_mat4(&R_Z90, [0.1, 0.0, 0.02]));
    let want = mat4_mul(&se3::to_mat4(&se3::IDENTITY, [1.0, 2.0, 3.0]), &t_imu_body);
    let got = vio.latest().unwrap();
    assert_close(&se3::to_mat4(&got.r, got.t), &want, 1e-12);
    wait_for("lost", || vio.health() == VioHealth::Lost).await;
    wait_for("exit and reset", || vio.stats().restarts >= 1).await;
    assert!(
        vio.latest().is_none(),
        "track is reset when the child exits"
    );
    // Restarted after the 1 s backoff, with a fresh track.
    wait_for("second run pose", || {
        std::fs::read_to_string(&runs)
            .map(|s| s.lines().count())
            .unwrap_or(0)
            >= 2
            && vio.latest().is_some()
    })
    .await;
    assert_eq!(vio.latest().unwrap().t_ns, 1000);
}

#[test]
fn spawn_refuses_same_camera_twice() {
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();
    let _g = rt.enter();
    let err = VioPose::spawn(VioConfig {
        binary: "/nonexistent".into(),
        rig: rig(CameraModelModel::Kb4, vec![0.0; 4], None),
        left_id: "cam_l".into(),
        right_id: "cam_l".into(),
        imu_id: "imu0".into(),
        work_dir: std::env::temp_dir(),
    });
    assert!(matches!(err, Err(VioError::SameCamera(_))));
}
