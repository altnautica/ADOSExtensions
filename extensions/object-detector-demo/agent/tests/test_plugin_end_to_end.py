"""End-to-end test: a synthetic frame flows through the real shared-memory
ring and the real VisionClient into the plugin's on_start path, and a
DetectionBatch comes back out on the detection-publish RPC.

No real plugin host, vision engine, OS shared memory, or socket is involved.
The fake engine builds a real frame ring through the shared frame-transport
contract and the client maps it read-only and resolves frames through the
per-slot seqlock, so the frame-read path is exercised exactly as in production.
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from ados.plugins.ipc_client import PluginContext
from ados.sdk.testing import FakeVisionEngine
from ados.sdk.testing.stubs import FakeIpcClient
from ados.sdk.vision import (
    DetectionBatch,
    FrameFormat,
    VisionClient,
    VISION_FRAME_TOPIC,
)

from altnautica_object_detector_demo.detector import CLASS_LABEL
from altnautica_object_detector_demo.plugin import (
    MODEL_ID,
    STATUS_TOPIC,
    ObjectDetectorDemoPlugin,
)

CAMERA = "cam0"
# A square frame so the harness (which publishes descriptors with width/height
# zeroed) and the plugin's buffer-length geometry inference agree on dimensions.
# For yuv420p, 192x192 luma + chroma is 55296 bytes, which infers back to
# exactly 192x192.
WIDTH = 192
HEIGHT = 192
CELL = 64
DARK = 16
BRIGHT = 240
PATCH_X = 128
PATCH_Y = 64

GRANTED = {
    "event.subscribe",
    "event.publish",
    "vision.frame.read",
    "vision.detection.publish",
}


def _yuv420p_with_patch() -> bytes:
    luma = bytearray([DARK] * (WIDTH * HEIGHT))
    for y in range(PATCH_Y, PATCH_Y + CELL):
        row = y * WIDTH
        for x in range(PATCH_X, PATCH_X + CELL):
            luma[row + x] = BRIGHT
    chroma = bytes([128] * (WIDTH * HEIGHT // 2))
    return bytes(luma) + chroma


def _yuv420p_solid(value: int) -> bytes:
    return bytes([value] * (WIDTH * HEIGHT)) + bytes([128] * (WIDTH * HEIGHT // 2))


def _published_batches(ipc: FakeIpcClient) -> list[DetectionBatch]:
    """Decode every detection batch the plugin published over the RPC wire."""
    batches: list[DetectionBatch] = []
    for method, args in ipc.requests:
        if method.endswith("publish_detection"):
            batches.append(DetectionBatch.from_msgpack(args["batch"]))
    return batches


async def _run_scenario(frames: list[bytes]) -> tuple[
    ObjectDetectorDemoPlugin, FakeIpcClient, FakeVisionEngine
]:
    """Build a real ring + client + context, start the plugin, deliver frames."""
    engine = FakeVisionEngine.with_shm_dir(CAMERA, WIDTH, HEIGHT, FrameFormat.YUV420P)
    for f in frames:
        engine.push_frame(f)

    ipc = FakeIpcClient(plugin_id="com.altnautica.object-detector-demo",
                        granted_capabilities=set(GRANTED))
    ctx: Any = PluginContext(
        plugin_id="com.altnautica.object-detector-demo",
        plugin_version="0.1.0",
        config={},
        ipc=ipc,
    )
    # Point the context's vision client at the engine's file-backed ring so the
    # production resolver maps the real /dev/shm-style region the fake created.
    ctx.vision = VisionClient(ipc, shm_dir=Path(engine._shm_dir))  # noqa: SLF001

    plugin = ObjectDetectorDemoPlugin(cell=CELL)
    await plugin.on_start(ctx)

    # Drive each queued frame through the real ring + delivery event.
    while engine._pending:  # noqa: SLF001
        descriptor = engine._write_next()  # noqa: SLF001
        await ipc.deliver(
            VISION_FRAME_TOPIC, {"descriptor": descriptor.to_msgpack()}
        )

    await plugin.on_stop(ctx)
    return plugin, ipc, engine


def test_bright_patch_frame_publishes_box_on_the_patch() -> None:
    plugin, ipc, _ = asyncio.run(_run_scenario([_yuv420p_with_patch()]))

    assert plugin.frames_seen == 1
    assert plugin.detections_published == 1

    batches = _published_batches(ipc)
    assert len(batches) == 1
    batch = batches[0]
    assert batch.model_id == MODEL_ID
    assert batch.camera_id == CAMERA
    assert len(batch.detections) == 1

    det = batch.detections[0]
    assert det.class_label == CLASS_LABEL
    assert (det.bbox.x, det.bbox.y) == (float(PATCH_X), float(PATCH_Y))
    assert abs(det.confidence - BRIGHT / 255.0) < 1e-6


def test_first_frame_emits_status_event() -> None:
    _, ipc, _ = asyncio.run(_run_scenario([_yuv420p_with_patch()]))
    topics = [t for t, _ in ipc.published]
    assert STATUS_TOPIC in topics


def test_two_frames_publish_two_batches_with_higher_confidence_for_brighter() -> None:
    plugin, ipc, _ = asyncio.run(
        _run_scenario([_yuv420p_solid(30), _yuv420p_solid(220)])
    )
    assert plugin.frames_seen == 2
    batches = _published_batches(ipc)
    assert len(batches) == 2
    c_dark = batches[0].detections[0].confidence
    c_bright = batches[1].detections[0].confidence
    assert c_bright > c_dark
