//! Engine → capture frame seam: a loud regression net for the hop where a frame
//! the vision engine delivers through the plugin SDK must reach the capture
//! loop's frame source byte for byte, filtered to the enabled cameras.
//!
//! `FakeVisionEngine` writes each frame into a real framebus ring and resolves
//! it through the same seqlock read the SDK client uses, then invokes the exact
//! callback the capture service hands `ctx.vision.subscribe_frames`. Every read
//! is bounded by a timeout so a broken hop fails loudly instead of hanging.

use std::collections::HashSet;
use std::time::Duration;

use ados_protocol::framebus::FrameFormat;
use ados_sdk::testing::FakeVisionEngine;
use world_engine_capture::{CapturedFrame, EngineFrameSource, FrameRouter};

/// An 8x8 RGB24 frame with a non-uniform pattern, so a wrong slot or a
/// truncated read is caught instead of being masked by all-equal bytes.
fn rgb24_pattern(seed: u8) -> Vec<u8> {
    let n = FrameFormat::Rgb24.frame_bytes(8, 8);
    (0..n)
        .map(|i| ((i * 7 + seed as usize) % 251) as u8)
        .collect()
}

fn engine(camera: &str, router: &std::sync::Arc<FrameRouter>) -> FakeVisionEngine {
    let mut e = FakeVisionEngine::new(camera, 8, 8, FrameFormat::Rgb24);
    e.on_frame_arc(router.callback());
    e
}

async fn next(src: &mut EngineFrameSource) -> CapturedFrame {
    tokio::time::timeout(Duration::from_secs(2), src.next())
        .await
        .expect("frame source timed out")
        .expect("frame source ended")
}

/// Nothing further is queued on `src` (it neither yields nor ends).
async fn assert_drained(src: &mut EngineFrameSource) {
    assert!(
        tokio::time::timeout(Duration::from_millis(100), src.next())
            .await
            .is_err(),
        "no further frame is queued"
    );
}

#[tokio::test]
async fn delivered_pixels_reach_the_capture_source_byte_for_byte() {
    let router = FrameRouter::new();
    let mut src = router.attach(HashSet::from(["front".to_string()]));
    let mut front = engine("front", &router);

    let pixels = rgb24_pattern(37);
    front.push_frame(pixels.clone());
    assert_eq!(front.deliver_all(), 1);

    let frame = next(&mut src).await;
    assert_eq!(frame.camera_id, "front");
    assert_eq!(frame.format, FrameFormat::Rgb24);
    assert_eq!(
        frame.pixels, pixels,
        "capture must read the exact pixels the engine delivered"
    );
}

#[tokio::test]
async fn a_disabled_camera_is_filtered_out() {
    let router = FrameRouter::new();
    let mut src = router.attach(HashSet::from(["front".to_string()]));
    let mut front = engine("front", &router);
    let mut down = engine("down", &router);

    // A disabled-camera frame first, then the wanted one: only the wanted one
    // surfaces, with ITS pixels.
    let disabled_px = vec![200u8; FrameFormat::Rgb24.frame_bytes(8, 8)];
    let wanted_px = rgb24_pattern(83);
    down.push_frame(disabled_px.clone());
    down.deliver_all();
    front.push_frame(wanted_px.clone());
    front.deliver_all();

    let got = next(&mut src).await;
    assert_eq!(got.camera_id, "front");
    assert_eq!(got.pixels, wanted_px);
    assert_drained(&mut src).await;
}

#[tokio::test]
async fn a_capture_loop_that_is_behind_never_blocks_the_callback() {
    // The callback runs on the SDK's IPC reader task. With nobody draining the
    // source, delivering far more frames than the queue holds must return, and
    // the source then yields a bounded backlog, not every frame.
    let router = FrameRouter::new();
    let mut src = router.attach(HashSet::new());
    let mut front = engine("front", &router);
    for i in 0..32 {
        front.push_solid(i);
    }
    assert_eq!(front.deliver_all(), 32);

    let mut queued = 0;
    while tokio::time::timeout(Duration::from_millis(100), src.next())
        .await
        .is_ok_and(|f| f.is_some())
    {
        queued += 1;
    }
    assert!(
        (1..32).contains(&queued),
        "a bounded backlog is kept, the rest dropped (got {queued})"
    );
}

#[tokio::test]
async fn reattaching_moves_frames_to_the_new_run() {
    // A config change restarts capture: the old run's source ends and the new
    // run receives frames under its own camera filter.
    let router = FrameRouter::new();
    let mut old = router.attach(HashSet::from(["front".to_string()]));
    let mut new = router.attach(HashSet::from(["down".to_string()]));
    assert!(tokio::time::timeout(Duration::from_secs(1), old.next())
        .await
        .expect("the replaced source ends")
        .is_none());

    let mut front = engine("front", &router);
    let mut down = engine("down", &router);
    front.push_solid(1);
    front.deliver_all();
    down.push_solid(2);
    down.deliver_all();
    assert_eq!(next(&mut new).await.camera_id, "down");
    assert_drained(&mut new).await;

    // Detached: frames are dropped at the callback and the source ends.
    router.detach();
    down.push_solid(3);
    down.deliver_all();
    assert!(new.next().await.is_none());
}
