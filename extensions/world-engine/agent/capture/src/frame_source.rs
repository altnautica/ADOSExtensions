//! Where the capture service gets camera frames.
//!
//! Two sources behind one enum:
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

/// One frame pulled from a source: its pixels plus what they are and when
/// they were exposed.
#[derive(Debug, Clone)]
pub struct CapturedFrame {
    pub camera_id: String,
    /// Capture time in unix milliseconds (the keyframe selector's clock).
    pub ts_ms: i64,
    /// Host monotonic nanoseconds; mid-exposure when the source knows it.
    pub ts_ns: u64,
    /// The source clock's own stamp, interpreted per `timebase`.
    pub ts_sensor_ns: Option<u64>,
    /// `ados_protocol::framebus::TIMEBASE_*` of `ts_sensor_ns`.
    pub timebase: u8,
    /// The hardware trigger that exposed this frame, when the source joined one.
    pub trigger_seq: Option<u64>,
    pub exposure_ns: Option<u32>,
    /// The producer's per-camera frame sequence.
    pub seq: u64,
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
            ts_ms: unix_ms_of_mono(d.ts_ns),
            ts_ns: d.ts_ns,
            ts_sensor_ns: d.ts_sensor_ns,
            timebase: d.timebase,
            trigger_seq: d.trigger_seq,
            exposure_ns: d.exposure_ns,
            seq: d.seq,
            width: d.width,
            height: d.height,
            format: d.format,
            pixels: frame.pixels,
        }
    }
}

/// Unix milliseconds of a host-monotonic instant. The frame descriptor stamps
/// the host monotonic clock; the capture record carries unix time, so the
/// frame's age on the monotonic clock is subtracted from the wall clock now.
fn unix_ms_of_mono(ts_ns: u64) -> i64 {
    let age_ms = (crate::clock::mono_ns().saturating_sub(ts_ns) / 1_000_000) as i64;
    let now_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0);
    now_ms - age_ms
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

/// How many frames may wait for the capture loop. Every frame is recorded, so
/// the queue absorbs a segment roll or a keyframe encode (about a second of a
/// two-camera 20 Hz rig); a frame that finds it full is dropped and counted.
const FRAME_QUEUE_DEPTH: usize = 48;

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
    /// Frames dropped at the callback because the loop was behind, per camera,
    /// since the last [`FrameRouter::take_drops`].
    drops: Mutex<HashMap<String, u64>>,
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

    /// Frames dropped because the capture loop was behind, per camera, since
    /// the previous call.
    pub fn take_drops(&self) -> HashMap<String, u64> {
        std::mem::take(&mut *self.drops.lock())
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
                *self.drops.lock().entry(f.camera_id).or_default() += 1;
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
    /// A source over an explicit channel (replay and tests).
    pub fn from_receiver(rx: mpsc::Receiver<CapturedFrame>) -> Self {
        Self { rx }
    }

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
                ts_ns: (start_ts_ms + i as i64 * interval_ms).max(0) as u64 * 1_000_000,
                ts_sensor_ns: None,
                timebase: ados_protocol::framebus::TIMEBASE_HOST,
                trigger_seq: None,
                exposure_ns: None,
                seq: i as u64,
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

    #[test]
    fn monotonic_stamp_maps_to_unix_now_minus_its_age() {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as i64;
        let ts_ns = crate::clock::mono_ns() - 2_000_000_000;
        let unix = unix_ms_of_mono(ts_ns);
        assert!((unix - (now - 2_000)).abs() < 500, "{unix} vs {now}");
    }
}
