# ADOS Object Detector Demo

A minimal Python plugin that proves the shared vision frame bus end to end. It
subscribes to camera frames the agent's vision engine publishes, runs a cheap
heuristic over each frame's luma plane, and publishes one bounding-box
detection. There is no neural network, model file, or accelerator dependency,
so it runs on any host and is a reference for how a Python vision plugin reads
frames and publishes detections.

## What it demonstrates

- Subscribing to normalized frames over `ctx.vision.subscribe_frames`, which
  resolves each frame from the engine's shared-memory ring read-only.
- Reading the luma plane for the three frame formats the engine emits: `nv12`
  and `yuv420p` (luma is the first `width * height` bytes) and `rgb24` (luma
  derived per pixel with Rec. 601 weights).
- Publishing a `DetectionBatch` over `ctx.vision.publish_one`, labelled by the
  source camera and frame so an overlay can align the box to its frame.

## The heuristic

Each frame's luma plane is split into a grid of 64x64 cells. The cell with the
highest mean luma becomes one detection: a bounding box at that cell, a class
label of `bright-region`, and a confidence equal to the cell's mean luma
normalized to 0..1. Brighter scenes yield higher confidence. The result is a
predictable function of the pixels, which is what makes it a good end-to-end
proof rather than a real detector.

## Capabilities

| Capability | Why |
|---|---|
| `vision.frame.read` | Subscribe to engine frames. |
| `vision.detection.publish` | Publish the heuristic detection. |
| `event.subscribe` | Receive frame-descriptor delivery events. |
| `event.publish` | Emit a status event when the first frame arrives. |

## Layout

```
object-detector-demo/
  manifest.yaml                      plugin manifest (agent half only)
  agent/
    pyproject.toml                   package + entry point
    src/altnautica_object_detector_demo/
      detector.py                    luma extraction + brightest-cell scan
      plugin.py                      lifecycle: subscribe, detect, publish
    tests/                           heuristic unit tests + end-to-end test
```

## Running the tests

From the repository root, with the agent virtual environment that provides the
`ados` SDK:

```
/path/to/ADOSDroneAgent/.venv/bin/python -m pytest \
  extensions/object-detector-demo/agent/tests -q
```

The end-to-end test uses `ados.sdk.testing.FakeVisionEngine`, which builds a
real frame ring and drives synthetic frames through the production read path.
A frame with a known bright patch is fed in, and the test asserts a detection
batch comes back with a box on the patch and a confidence that tracks its mean
luma. No real host, camera, or hardware is required.
