"""CSI camera capture via the libcamera Python binding (picamera2).

The Lucas-Kanade processor only needs a single-channel luma image, so
the stream is configured as YUV420 and the Y plane is sliced out at
the source. This keeps the per-frame copy small on the slowest target
boards (Pi Zero 2 W is the floor for this code path).

Boards without picamera2 installed raise at construction time. The
pipeline orchestrator catches the error and falls back to the V4L2
backend.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, AsyncIterator, Optional, Tuple

try:  # pragma: no cover - import guard tested via fallback path.
    from picamera2 import Picamera2  # type: ignore[import-untyped]
    _PICAMERA2_AVAILABLE = True
except ImportError:  # pragma: no cover
    Picamera2 = None  # type: ignore[assignment]
    _PICAMERA2_AVAILABLE = False

try:  # pragma: no cover
    import numpy as np  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    np = None  # type: ignore[assignment]

log = logging.getLogger(__name__)

PREWARM_FRAMES = 5


class LibcameraSource:
    """Capture luma frames from a CSI camera via libcamera + picamera2.

    Parameters
    ----------
    width, height:
        Requested frame dimensions. The driver may settle on a nearby
        supported size and the actual configured value is read back
        onto ``self.width`` / ``self.height`` after start.
    fps:
        Target frame rate. Translated to a frame-duration window.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: float = 30.0) -> None:
        if not _PICAMERA2_AVAILABLE:
            raise RuntimeError("libcamera not available on this board")
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.requested_fps = float(fps)
        self.width = self.requested_width
        self.height = self.requested_height
        self.fps = self.requested_fps
        self._cam: Optional[Any] = None
        self._running = False

    def start(self) -> None:
        """Configure libcamera in YUV420 mode and warm the sensor."""

        if self._cam is not None:
            return
        cam = Picamera2()  # type: ignore[misc]
        frame_duration_us = int(round(1_000_000 / max(self.requested_fps, 0.1)))
        cfg = cam.create_video_configuration(
            main={
                "size": (self.requested_width, self.requested_height),
                "format": "YUV420",
            },
            controls={
                "FrameDurationLimits": (frame_duration_us, frame_duration_us),
            },
        )
        cam.configure(cfg)
        cam.start()
        # Read back what libcamera actually settled on.
        try:
            self.width, self.height = cam.camera_configuration()["main"]["size"]
        except Exception:  # noqa: BLE001 - any layout drift, fall back to request.
            pass
        for _ in range(PREWARM_FRAMES):
            cam.capture_array("main")
        self._cam = cam
        self._running = True
        log.info(
            "libcamera source opened: size=%dx%d fps=%.2f",
            self.width,
            self.height,
            self.fps,
        )

    def stop(self) -> None:
        """Stop the camera and release resources. Idempotent."""

        self._running = False
        if self._cam is not None:
            try:
                self._cam.stop()
                self._cam.close()
            except Exception:  # noqa: BLE001 - best-effort teardown.
                log.warning("libcamera teardown raised; ignoring", exc_info=True)
            self._cam = None

    async def frames(self) -> AsyncIterator[Tuple[int, "np.ndarray"]]:  # type: ignore[name-defined]
        """Yield ``(monotonic_ns, frame_luma)`` tuples until ``stop()``.

        ``frame_luma`` is a single-channel ``uint8`` array of shape
        ``(height, width)`` extracted from the YUV420 buffer.
        """

        if self._cam is None or not self._running:
            raise RuntimeError("libcamera source not started")

        while self._running:
            buf = await asyncio.to_thread(self._cam.capture_array, "main")
            stamp_ns = time.monotonic_ns()
            luma = buf[: self.height, : self.width]
            yield stamp_ns, luma
