//! Where the capture service gets camera frames.
//!
//! Two sources behind one enum (the [`crate::pose_source`] sibling pattern):
//!
//! - [`EngineFrameSource`] is the real path. The plugin host delivers the
//!   vision engine's frames through `ctx.vision.subscribe_frames`, whose
//!   callback runs on the SDK's IPC reader task and must never block. A
//!   [`FrameRouter`] owns that callback for the life of the process (a frame
//!   subscription cannot be withdrawn) and hands each frame of an enabled
//!   camera to the currently attached source over a bounded channel, dropping
//!   the frame when the capture loop is behind.
//! - [`SyntheticFrameSource`] emits deterministic frames with no hardware, for
//!   the SITL harness and demo runs.

use std::collections::{HashMap, HashSet};

use ados_protocol::framebus::FrameFormat;
use ados_sdk::vision::{Frame, FrameCallback};
use parking_lot::Mutex;
use std::sync::Arc;
use tokio::sync::mpsc;

/// One frame pulled from a source: its pixels plus what they are.
#[derive(Debug, Clone)]
pub struct CapturedFrame {
    pub camera_id: String,
    pub ts_ms: i64,
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub pixels: Vec<u8>,
}

impl From<Frame> for CapturedFrame {
    fn from(frame: Frame) -> Self {
        let d = frame.descriptor;
        Self {
            camera_id: d.camera_id,
            ts_ms: d.ts_ms,
            width: d.width,
            height: d.height,
            format: d.format,
            pixels: frame.pixels,
        }
    }
}

/// The frame source the capture loop runs with.
pub enum AtlasFrameSource {
    Engine(EngineFrameSource),
    Synthetic(SyntheticFrameSource),
}

impl AtlasFrameSource {
    /// Pull the next frame. `None` means the source needs a moment (a detached
    /// engine source, or an exhausted synthetic sequence); the caller backs off
    /// and calls again.
    pub async fn next(&mut self) -> Option<CapturedFrame> {
        match self {
            AtlasFrameSource::Engine(e) => e.next().await,
            AtlasFrameSource::Synthetic(s) => s.next().await,
        }
    }
}

/// How many frames may wait for the capture loop. The loop takes one frame at a
/// time; a small queue absorbs a keyframe encode without holding frames long
/// enough to go stale against their pose.
const FRAME_QUEUE_DEPTH: usize = 4;

/// The attached receiver's side of the router: where frames go and which
/// cameras are wanted.
struct Sink {
    tx: mpsc::Sender<CapturedFrame>,
    /// Enabled camera ids; empty means accept every camera.
    enabled: HashSet<String>,
    /// Camera ids already warned about, so a persistent engine↔capture id
    /// mismatch is logged once per id, not on every frame.
    warned_unmatched: HashSet<String>,
    /// The last ring sequence routed per camera. A subscription retried after a
    /// failed request leaves its callback registered twice, so the same frame
    /// can arrive twice; it is routed once.
    last_seq: HashMap<String, u64>,
}

/// Routes engine frames from the one process-lifetime SDK subscription to the
/// capture run currently attached, applying that run's enabled-camera filter.
#[derive(Default)]
pub struct FrameRouter {
    sink: Mutex<Option<Sink>>,
}

impl FrameRouter {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// The callback to hand `ctx.vision.subscribe_frames`. Never blocks: a
    /// frame for a disabled camera, with nothing attached, or with the capture
    /// loop behind is dropped.
    pub fn callback(self: &Arc<Self>) -> FrameCallback {
        let router = self.clone();
        Arc::new(move |frame: Frame| router.deliver(frame))
    }

    /// Attach a new capture run, filtering to `enabled` camera ids (empty =
    /// every camera). Replaces any prior attachment, whose source then ends.
    pub fn attach(&self, enabled: HashSet<String>) -> EngineFrameSource {
        let (tx, rx) = mpsc::channel(FRAME_QUEUE_DEPTH);
        *self.sink.lock() = Some(Sink {
            tx,
            enabled,
            warned_unmatched: HashSet::new(),
            last_seq: HashMap::new(),
        });
        EngineFrameSource { rx }
    }

    /// Detach the current run; later frames are dropped at the callback.
    pub fn detach(&self) {
        *self.sink.lock() = None;
    }

    fn deliver(&self, frame: Frame) {
        let mut guard = self.sink.lock();
        let Some(sink) = guard.as_mut() else {
            return;
        };
        let camera_id = &frame.descriptor.camera_id;
        if !sink.enabled.is_empty() && !sink.enabled.contains(camera_id) {
            // A dropped frame must be visible, never silent: a mismatched
            // camera id between the vision engine and the capture config is the
            // classic cause of `ingest_rate_hz: 0` with no error. Warn once per
            // unexpected id so a real misconfig is one log read away.
            if sink.warned_unmatched.insert(camera_id.clone()) {
                let enabled: Vec<&str> = sink.enabled.iter().map(String::as_str).collect();
                tracing::warn!(
                    camera_id = %camera_id,
                    enabled = ?enabled,
                    "atlas dropping frames: camera id not in the enabled set (engine/capture camera id mismatch)"
                );
            }
            return;
        }
        let seq = frame.descriptor.seq;
        if sink.last_seq.insert(camera_id.clone(), seq) == Some(seq) {
            return;
        }
        match sink.tx.try_send(CapturedFrame::from(frame)) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(f)) => {
                tracing::trace!(camera = %f.camera_id, "atlas_frame_dropped_loop_behind");
            }
            // The run's source was dropped; stop routing to it.
            Err(mpsc::error::TrySendError::Closed(_)) => *guard = None,
        }
    }
}

/// The real source: the frames a [`FrameRouter`] routes to this capture run.
pub struct EngineFrameSource {
    rx: mpsc::Receiver<CapturedFrame>,
}

impl EngineFrameSource {
    /// The next routed frame, or `None` once the router detached this source.
    pub async fn next(&mut self) -> Option<CapturedFrame> {
        self.rx.recv().await
    }
}

/// A deterministic source for the SITL harness and demo runs: a fixed list of
/// frames replayed in order, then exhausted (`next` returns `None`). No
/// hardware, no sockets, no shared memory.
pub struct SyntheticFrameSource {
    frames: std::collections::VecDeque<CapturedFrame>,
}

impl SyntheticFrameSource {
    /// Build from an explicit frame list (the caller controls camera id, size,
    /// format, bytes, and timestamps).
    pub fn new(frames: Vec<CapturedFrame>) -> Self {
        Self {
            frames: frames.into(),
        }
    }

    /// Build a simple one-camera RGB sequence: `count` solid-grey frames spaced
    /// `interval_ms` apart starting at `start_ts_ms`.
    pub fn solid(
        camera_id: &str,
        width: u32,
        height: u32,
        count: usize,
        start_ts_ms: i64,
        interval_ms: i64,
    ) -> Self {
        let frames = (0..count)
            .map(|i| CapturedFrame {
                camera_id: camera_id.to_string(),
                ts_ms: start_ts_ms + i as i64 * interval_ms,
                width,
                height,
                format: FrameFormat::Rgb24,
                pixels: vec![(i % 256) as u8; (width * height * 3) as usize],
            })
            .collect();
        Self::new(frames)
    }

    async fn next(&mut self) -> Option<CapturedFrame> {
        self.frames.pop_front()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn synthetic_source_replays_then_exhausts() {
        let mut s =
            AtlasFrameSource::Synthetic(SyntheticFrameSource::solid("front", 8, 8, 3, 0, 100));
        let f0 = s.next().await.unwrap();
        assert_eq!(f0.camera_id, "front");
        assert_eq!(f0.ts_ms, 0);
        assert_eq!(f0.format, FrameFormat::Rgb24);
        assert_eq!(s.next().await.unwrap().ts_ms, 100);
        assert_eq!(s.next().await.unwrap().ts_ms, 200);
        assert!(s.next().await.is_none(), "exhausted");
    }
}
