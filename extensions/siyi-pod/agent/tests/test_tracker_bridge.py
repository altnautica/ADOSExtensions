"""Tracker-republish tests: the pod's box lands on the shared detection bus."""

from __future__ import annotations

from altnautica_siyi_pod.tracker_bridge import SiyiTrackerBridge


class _FakeVision:
    def __init__(self) -> None:
        self.published: list = []

    async def publish_detection(self, batch) -> dict:
        self.published.append(batch)
        return {"ok": True}


class _FakeCtx:
    def __init__(self) -> None:
        self.vision = _FakeVision()


async def test_publish_box_builds_locked_detection():
    ctx = _FakeCtx()
    bridge = SiyiTrackerBridge(ctx, camera_id="siyi-pod")
    await bridge.publish_box(x=10, y=20, width=30, height=40, track_id=7)
    assert len(ctx.vision.published) == 1
    batch = ctx.vision.published[0]
    assert batch.camera_id == "siyi-pod"
    assert len(batch.detections) == 1
    det = batch.detections[0]
    assert det.track_id == 7
    assert det.lock_state == "locked"
    assert (det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height) == (10, 20, 30, 40)


async def test_publish_lost_sends_empty_batch():
    ctx = _FakeCtx()
    bridge = SiyiTrackerBridge(ctx)
    await bridge.publish_box(x=1, y=2, width=3, height=4, track_id=1)
    await bridge.publish_lost()
    assert len(ctx.vision.published) == 2
    assert ctx.vision.published[-1].detections == []
    # Frame ids advance monotonically so consumers can order batches.
    assert ctx.vision.published[1].frame_id > ctx.vision.published[0].frame_id
