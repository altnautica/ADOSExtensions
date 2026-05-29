"""Plugin entry point for the ADOS Object Detector Demo.

The host instantiates :class:`ObjectDetectorDemoPlugin`, issues capability
tokens, then calls :meth:`on_start`. At start the plugin subscribes to the
shared vision frame bus via ``ctx.vision.subscribe_frames``. For each frame the
host delivers, the plugin runs a cheap brightest-region heuristic on the luma
plane (see :mod:`altnautica_object_detector_demo.detector`) and publishes one
bounding-box detection through ``ctx.vision.publish_one``.

There is no model file, accelerator, or native dependency. The detection is a
predictable function of the frame's pixels, so the plugin proves the
frame-read and detection-publish contract end to end on any host: a frame goes
in over the shared-memory ring, a :class:`~ados.sdk.vision.DetectionBatch`
comes out on the detection topic.

Capabilities declared in the manifest and exercised here:

* ``vision.frame.read`` — subscribe to engine frames.
* ``vision.detection.publish`` — publish the heuristic detection.
* ``event.subscribe`` — receive frame-descriptor delivery events.
* ``event.publish`` — emit a small status event when the first frame lands.
"""

from __future__ import annotations

from typing import Any

from ados.sdk.vision import Frame

from altnautica_object_detector_demo.detector import DEFAULT_CELL, detection_for

# Model id the published detections are labelled with. The plugin runs the
# heuristic in-process and never registers an engine-run model, so this is a
# stable name for the heuristic's output rather than a loaded model file.
MODEL_ID = "com.altnautica.object-detector-demo.bright-region"

# Status topic the plugin emits one event on when it sees its first frame, so
# an operator can confirm the frame bus is live without watching detections.
STATUS_TOPIC = "object-detector-demo.status"


class ObjectDetectorDemoPlugin:
    """Heuristic detector that proves the vision frame bus end to end."""

    plugin_id = "com.altnautica.object-detector-demo"
    version = "0.1.0"

    def __init__(self, *, cell: int = DEFAULT_CELL) -> None:
        self._cell = int(cell)
        self._ctx: Any | None = None
        self._frames_seen = 0
        self._detections_published = 0

    async def on_start(self, ctx: Any) -> None:
        """Subscribe to every camera's frames and start emitting detections."""
        self._ctx = ctx
        await ctx.vision.subscribe_frames(self._on_frame)
        try:
            ctx.log.info(
                "object-detector-demo subscribed to vision frames",
                extra={"cell": self._cell},
            )
        except Exception:
            # Logging is best-effort. Production gives a structured logger;
            # tests may pass a stub without one.
            pass

    async def on_stop(self, ctx: Any) -> None:
        """Release the vision client's cached ring mappings."""
        try:
            ctx.vision.close()
        except Exception:
            pass
        self._ctx = None

    async def _on_frame(self, frame: Frame) -> None:
        """Run the heuristic on one frame and publish its detection."""
        if self._ctx is None:
            return
        self._frames_seen += 1

        detection = detection_for(frame, cell=self._cell)
        if detection is None:
            return

        await self._ctx.vision.publish_one(MODEL_ID, frame, detection)
        self._detections_published += 1

        if self._frames_seen == 1:
            try:
                await self._ctx.events.publish(
                    STATUS_TOPIC,
                    {"first_frame": True, "camera_id": frame.descriptor.camera_id},
                )
            except Exception:
                # The status event is informational; a denied or missing
                # publish surface must not stop detections.
                pass

    @property
    def frames_seen(self) -> int:
        """Test helper: number of frames the plugin has processed."""
        return self._frames_seen

    @property
    def detections_published(self) -> int:
        """Test helper: number of detections the plugin has published."""
        return self._detections_published


__all__ = ["ObjectDetectorDemoPlugin", "MODEL_ID", "STATUS_TOPIC"]
