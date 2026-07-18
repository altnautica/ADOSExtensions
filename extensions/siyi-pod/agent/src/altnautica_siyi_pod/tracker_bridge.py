"""Republish the pod's own AI tracker onto the shared detection bus.

The pod owns the tracking loop: it detects a subject on its camera and self-slews
its gimbal to hold it, with no companion NPU. This bridge reads the pod's tracker
box and republishes it as a normal detection batch on ``vision.detection`` (the
same bus the cockpit overlay and Follow-Me consume), so the existing
click-to-track UI, the locked-target safety gate, and geolocation all work over
the pod's tracker. The box is stamped with the pod's tracking camera id so the
overlay only draws it while that stream is shown.

The reverse direction (an operator designating a box in the GCS, forwarded down
to the pod's tracker) is driven by the plugin's control loop through the pod
facade's AI-track methods.
"""

from __future__ import annotations

import logging
import time

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

log = logging.getLogger(__name__)


class SiyiTrackerBridge:
    """Publishes the pod's tracker box onto ``vision.detection``."""

    def __init__(
        self,
        ctx,
        *,
        camera_id: str = "siyi-pod",
        model_id: str = "siyi-pod-tracker",
        class_label: str = "subject",
    ) -> None:
        self._ctx = ctx
        self._camera_id = camera_id
        self._model_id = model_id
        self._class_label = class_label
        self._frame_id = 0

    async def publish_box(
        self,
        *,
        x: float,
        y: float,
        width: float,
        height: float,
        track_id: int,
        locked: bool = True,
        confidence: float = 1.0,
    ) -> None:
        """Republish the pod's current tracker box as a locked detection."""
        self._frame_id += 1
        detection = Detection(
            bbox=BoundingBox(x=x, y=y, width=width, height=height),
            class_label=self._class_label,
            confidence=confidence,
            track_id=track_id,
            lock_state="locked" if locked else "uncertain",
        )
        await self._publish([detection])

    async def publish_lost(self) -> None:
        """Publish an empty batch so consumers see the track drop (never a
        silent stall)."""
        self._frame_id += 1
        await self._publish([])

    async def _publish(self, detections: list[Detection]) -> None:
        batch = DetectionBatch(
            model_id=self._model_id,
            camera_id=self._camera_id,
            frame_id=self._frame_id,
            ts_ms=int(time.monotonic() * 1000) & 0x7FFFFFFF,
            detections=detections,
        )
        try:
            await self._ctx.vision.publish_detection(batch)
        except Exception:  # noqa: BLE001
            log.debug("tracker republish failed", exc_info=True)
