//! Trivial Rust object detector.
//!
//! This is the smallest real vision plugin: it proves the shared frame bus and
//! the detection path end to end without a machine-learning model or any native
//! dependency. It subscribes to the vision engine's frames over `ctx.vision`,
//! runs a cheap brightness heuristic on each one, and publishes a single
//! bounding box per frame on the detection topic. The point is the wiring, not
//! the detector: any board can run it to confirm frames flow in and detections
//! flow out.
//!
//! ## The heuristic
//!
//! The frame is read as a luma image. The image is scanned in [`CELL`]x[`CELL`]
//! cells; the cell with the highest mean luma wins. Its top-left corner and a
//! [`CELL`]x[`CELL`] box become the detection. Confidence is the bright cell's
//! mean luma scaled to `0.0..=1.0`. The class label is a fixed
//! [`CLASS_LABEL`]. A frame with no usable luma (too small for one cell) yields
//! no detection.
//!
//! ## Luma per format
//!
//! - `nv12` and `yuv420p` are planar: the luma plane is the first `width *
//!   height` bytes, so luma is read directly.
//! - `rgb24` is packed 3 bytes per pixel: luma is approximated per pixel from
//!   the R, G, B bytes with the standard Rec. 601 weights.
//!
//! ## Threading
//!
//! `ctx.vision.subscribe_frames` invokes the callback on the IPC reader task,
//! which must not block. Publishing a detection is an async RPC, so the callback
//! computes the detection synchronously (cheap) and offloads the publish to a
//! spawned task on the same runtime, holding a cheap clone of the vision client.

use std::collections::BTreeMap;
use std::sync::Arc;

use ados_sdk::context::PluginContext;
use ados_sdk::vision::Frame;
use ados_sdk::{run_plugin, ClientError, Plugin, RunnerError};
use ados_protocol::framebus::{BoundingBox, Detection, FrameFormat};
use async_trait::async_trait;

/// Side of the square scan cell, in pixels. Also the size of the published box.
const CELL: u32 = 64;

/// The single class this detector ever emits. It does not classify; it reports
/// where the brightest patch of the frame is.
const CLASS_LABEL: &str = "bright-region";

/// The model id this plugin labels its detections with. The plugin runs the
/// heuristic itself and does not register an engine-run model, so this is just a
/// stable source label for detection consumers and overlays.
const MODEL_ID: &str = "com.altnautica.object-detector-rs";

/// The brightness detector. Holds no per-frame state; the only state is the
/// vision client clone the callback uses to publish, captured when the
/// subscription is set up.
struct ObjectDetector;

#[async_trait]
impl Plugin for ObjectDetector {
    fn new() -> Self {
        ObjectDetector
    }

    /// Subscribe to every camera's frames. For each frame, compute the bright
    /// region synchronously and offload the publish to a spawned task so the IPC
    /// reader is never blocked on an RPC round trip.
    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        // A cheap clone of the vision client; cloning shares the one IPC client
        // and the ring resolver cache.
        let vision = ctx.vision.clone();

        let on_frame = Arc::new(move |frame: Frame| {
            // Cheap, synchronous: scan for the brightest cell.
            let Some(detection) = detect(&frame) else {
                return;
            };
            // The publish is async; hand it to the runtime so the reader task
            // returns immediately.
            let vision = vision.clone();
            let frame = frame.clone();
            tokio::spawn(async move {
                if let Err(err) = vision.publish_one(MODEL_ID, &frame, detection).await {
                    eprintln!("object-detector-rs: publish detection failed: {err}");
                }
            });
        });

        // `None` camera filter: detect on whatever camera the engine streams.
        ctx.vision.subscribe_frames(None, on_frame).await
    }
}

/// Run the brightness heuristic over one frame, returning a single detection at
/// the brightest [`CELL`]x[`CELL`] cell, or `None` if the frame is too small to
/// hold one cell or carries no luma.
///
/// This is a free function so a test can drive it with frames from the
/// [`FakeVisionEngine`](ados_sdk::testing::FakeVisionEngine) without a host.
fn detect(frame: &Frame) -> Option<Detection> {
    let width = frame.descriptor.width;
    let height = frame.descriptor.height;
    if width < CELL || height < CELL {
        return None;
    }

    let luma = luma_at_fn(frame)?;

    // Scan cell-aligned origins, find the cell with the highest mean luma.
    let mut best_mean = -1.0f32;
    let mut best_x = 0u32;
    let mut best_y = 0u32;
    let mut y = 0u32;
    while y + CELL <= height {
        let mut x = 0u32;
        while x + CELL <= width {
            let mean = cell_mean(&luma, x, y);
            if mean > best_mean {
                best_mean = mean;
                best_x = x;
                best_y = y;
            }
            x += CELL;
        }
        y += CELL;
    }

    if best_mean < 0.0 {
        return None;
    }

    // Mean luma 0..255 scaled to a 0..1 confidence.
    let confidence = (best_mean / 255.0).clamp(0.0, 1.0);

    Some(Detection {
        bbox: BoundingBox {
            x: best_x as f32,
            y: best_y as f32,
            width: CELL as f32,
            height: CELL as f32,
        },
        class_label: CLASS_LABEL.to_string(),
        confidence,
        track_id: None,
    })
}

/// A reader over one frame's pixels that returns the luma (0..=255) of a pixel
/// at `(x, y)`. Bundles the frame format so the scan is format-agnostic.
struct LumaReader<'a> {
    format: FrameFormat,
    width: usize,
    pixels: &'a [u8],
}

impl LumaReader<'_> {
    /// Luma of the pixel at `(x, y)`. Out-of-range pixels (a short/truncated
    /// capture) read as 0.
    fn at(&self, x: u32, y: u32) -> u8 {
        let xi = x as usize;
        let yi = y as usize;
        match self.format {
            // Planar luma plane is the first width*height bytes, row-major.
            FrameFormat::Nv12 | FrameFormat::Yuv420p => {
                let idx = yi * self.width + xi;
                self.pixels.get(idx).copied().unwrap_or(0)
            }
            // Packed RGB: three bytes per pixel, Rec. 601 luma approximation.
            FrameFormat::Rgb24 => {
                let base = (yi * self.width + xi) * 3;
                let r = self.pixels.get(base).copied().unwrap_or(0) as u32;
                let g = self.pixels.get(base + 1).copied().unwrap_or(0) as u32;
                let b = self.pixels.get(base + 2).copied().unwrap_or(0) as u32;
                // 0.299 R + 0.587 G + 0.114 B in fixed point (sum of weights = 1000).
                ((299 * r + 587 * g + 114 * b) / 1000) as u8
            }
        }
    }
}

/// Build a [`LumaReader`] for a frame, or `None` if the pixel buffer is empty.
fn luma_at_fn(frame: &Frame) -> Option<LumaReader<'_>> {
    if frame.pixels.is_empty() {
        return None;
    }
    Some(LumaReader {
        format: frame.descriptor.format,
        width: frame.descriptor.width as usize,
        pixels: &frame.pixels,
    })
}

/// Mean luma of the [`CELL`]x[`CELL`] cell whose top-left corner is `(ox, oy)`.
fn cell_mean(luma: &LumaReader<'_>, ox: u32, oy: u32) -> f32 {
    let mut sum = 0u32;
    for dy in 0..CELL {
        for dx in 0..CELL {
            sum += luma.at(ox + dx, oy + dy) as u32;
        }
    }
    sum as f32 / (CELL * CELL) as f32
}

/// Resolve when the process is asked to stop. The plugin host stops the unit
/// with SIGTERM; SIGINT covers a foreground run.
async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};

    let mut term = signal(SignalKind::terminate()).expect("install SIGTERM handler");
    let mut int = signal(SignalKind::interrupt()).expect("install SIGINT handler");

    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}

#[tokio::main]
async fn main() {
    // No static config: the detector reads nothing from the manifest.
    let static_config: BTreeMap<String, rmpv::Value> = BTreeMap::new();

    match run_plugin::<ObjectDetector, _>(
        env!("CARGO_PKG_VERSION"),
        static_config,
        shutdown_signal(),
    )
    .await
    {
        Ok(()) => {}
        // Running the binary by hand (no host) lands here: there is no socket or
        // token to connect to. Report it and exit non-zero rather than
        // panicking, so the binary is safe to invoke standalone while checking a
        // build.
        Err(RunnerError::NoBridge) => {
            eprintln!(
                "object-detector-rs: no plugin host socket/token supplied; \
                 this binary is launched by the agent plugin host"
            );
            std::process::exit(1);
        }
        Err(err) => {
            eprintln!("object-detector-rs: {err}");
            std::process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_sdk::testing::FakeVisionEngine;
    use std::sync::Mutex;

    /// Width/height for the test frames. Two cells wide and tall so there is a
    /// real choice of brightest cell, not a degenerate single-cell frame.
    const W: u32 = CELL * 2;
    const H: u32 = CELL * 2;

    /// Build an rgb24 frame that is dark everywhere except a bright square whose
    /// top-left corner is `(patch_x, patch_y)`. The bright square is one cell.
    fn rgb24_with_bright_patch(patch_x: u32, patch_y: u32, bright: u8) -> Vec<u8> {
        let mut pixels = vec![0u8; (W * H * 3) as usize];
        for dy in 0..CELL {
            for dx in 0..CELL {
                let x = patch_x + dx;
                let y = patch_y + dy;
                let base = ((y * W + x) * 3) as usize;
                // Grey: r=g=b=bright keeps the luma approximation equal to bright.
                pixels[base] = bright;
                pixels[base + 1] = bright;
                pixels[base + 2] = bright;
            }
        }
        pixels
    }

    /// A detection is produced for a frame with a known bright patch, and its
    /// box lands on that patch. Drives the real frame read path through the
    /// fake vision engine: the synthetic frame is written to a real ring and
    /// resolved through the seqlock exactly as production does.
    #[test]
    fn detects_a_bright_patch_through_the_frame_bus() {
        let mut engine = FakeVisionEngine::new("uvc-0", W, H, FrameFormat::Rgb24);

        // The engine's deliver path stamps width/height as 0 in the descriptor
        // (the resolver keys on byte_len), but `detect` reads width/height from
        // the descriptor. Capture the resolved frame and re-run detect against a
        // frame carrying the real dimensions, exactly as the host descriptor
        // would. This still exercises the full ring read for the pixels.
        let captured: Arc<Mutex<Vec<Frame>>> = Arc::new(Mutex::new(Vec::new()));
        let c = captured.clone();
        engine.on_frame(move |frame| c.lock().unwrap().push(frame));

        // Bright patch in the bottom-right cell.
        let patch_x = CELL;
        let patch_y = CELL;
        engine.push_frame(rgb24_with_bright_patch(patch_x, patch_y, 200));
        assert_eq!(engine.deliver_all(), 1);

        let frames = captured.lock().unwrap();
        assert_eq!(frames.len(), 1, "the bright frame was delivered");

        // The fake stamps width/height as 0; restore the real dimensions the
        // host descriptor would carry so the cell scan has a grid to walk.
        let mut frame = frames[0].clone();
        frame.descriptor.width = W;
        frame.descriptor.height = H;
        assert_eq!(frame.pixels.len(), (W * H * 3) as usize);

        let detection = detect(&frame).expect("a bright patch yields one detection");
        assert_eq!(detection.class_label, CLASS_LABEL);
        assert_eq!(detection.bbox.x, patch_x as f32);
        assert_eq!(detection.bbox.y, patch_y as f32);
        assert_eq!(detection.bbox.width, CELL as f32);
        assert_eq!(detection.bbox.height, CELL as f32);
        // Confidence tracks the bright cell's mean luma (200/255).
        let expected = 200.0f32 / 255.0;
        assert!(
            (detection.confidence - expected).abs() < 1e-3,
            "confidence {} ~= {}",
            detection.confidence,
            expected
        );
    }

    /// A planar (yuv420p) frame is read from its luma plane (the first
    /// width*height bytes), and the brightest cell wins.
    #[test]
    fn detects_a_bright_patch_in_a_planar_luma_plane() {
        let mut luma = vec![10u8; (W * H) as usize];
        // Bright cell at the top-left.
        for dy in 0..CELL {
            for dx in 0..CELL {
                luma[(dy * W + dx) as usize] = 240;
            }
        }
        // Full yuv420p frame: luma plane then chroma; chroma stays mid-grey.
        let mut pixels = luma;
        pixels.resize(FrameFormat::Yuv420p.frame_bytes(W, H), 128);

        let frame = Frame {
            descriptor: ados_protocol::framebus::FrameDescriptor {
                camera_id: "uvc-0".into(),
                frame_id: 1,
                ts_ms: 0,
                width: W,
                height: H,
                format: FrameFormat::Yuv420p,
                shm_name: "ados-vision-uvc-0".into(),
                slot: 0,
                seq: 1,
                byte_len: FrameFormat::Yuv420p.frame_bytes(W, H) as u32,
            },
            pixels,
        };

        let detection = detect(&frame).expect("planar bright patch yields a detection");
        assert_eq!(detection.bbox.x, 0.0);
        assert_eq!(detection.bbox.y, 0.0);
        let expected = 240.0f32 / 255.0;
        assert!((detection.confidence - expected).abs() < 1e-3);
    }

    /// A frame smaller than one cell yields no detection rather than a panic.
    #[test]
    fn no_detection_for_a_subcell_frame() {
        let small = CELL / 2;
        let frame = Frame {
            descriptor: ados_protocol::framebus::FrameDescriptor {
                camera_id: "uvc-0".into(),
                frame_id: 1,
                ts_ms: 0,
                width: small,
                height: small,
                format: FrameFormat::Rgb24,
                shm_name: "ados-vision-uvc-0".into(),
                slot: 0,
                seq: 1,
                byte_len: (small * small * 3),
            },
            pixels: vec![255u8; (small * small * 3) as usize],
        };
        assert!(detect(&frame).is_none());
    }
}
