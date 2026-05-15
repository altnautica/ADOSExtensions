"""USB UVC camera capture via OpenCV's V4L2 backend.

Yields BGR frames stamped at v4l2 receipt. The optical-flow processor
downstream converts to grayscale; the source intentionally stays
format-agnostic so the same module can feed other consumers (preview
encoding, future drivers) without reshaping.

A short pre-warm loop is performed in ``start()`` because UVC modules
typically take a few frames to settle exposure and white balance, and
the first ``read()`` after open can return a stale buffer.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import AsyncIterator, Optional, Tuple

try:  # pragma: no cover - only fails when opencv-python is absent.
    import cv2  # type: ignore[import-untyped]
    import numpy as np  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]

log = logging.getLogger(__name__)

PREWARM_FRAMES = 5
RETRY_DELAY_S = 1.0


class V4l2Source:
    """Capture frames from a /dev/videoN node via OpenCV's V4L2 backend.

    Parameters
    ----------
    device_path:
        The character-device path, e.g. ``"/dev/video0"``.
    width, height:
        Requested frame dimensions in pixels. The driver may settle on
        the nearest supported size; the value reported by OpenCV after
        ``open`` is treated as authoritative and stored on the instance.
    fps:
        Requested frame rate. Same caveat as the dimensions.
    """

    def __init__(
        self,
        device_path: str,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
    ) -> None:
        if cv2 is None:  # pragma: no cover - import guard.
            raise RuntimeError(
                "opencv-python (or opencv-python-headless) is required for "
                "V4l2Source"
            )
        self.device_path = device_path
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.requested_fps = float(fps)
        self.width = self.requested_width
        self.height = self.requested_height
        self.fps = self.requested_fps
        self._cap: Optional["cv2.VideoCapture"] = None  # type: ignore[name-defined]
        self._running = False

    def start(self) -> None:
        """Open the device, configure it, and discard a few warm-up frames."""

        if self._cap is not None:
            return
        cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"failed to open v4l2 device {self.device_path!r}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
        cap.set(cv2.CAP_PROP_FPS, self.requested_fps)
        # Read back the values the driver actually settled on.
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.requested_width
        self.height = (
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.requested_height
        )
        reported_fps = cap.get(cv2.CAP_PROP_FPS)
        if reported_fps > 0:
            self.fps = float(reported_fps)
        for _ in range(PREWARM_FRAMES):
            cap.read()
        self._cap = cap
        self._running = True
        log.info(
            "v4l2 source opened: device=%s size=%dx%d fps=%.2f",
            self.device_path,
            self.width,
            self.height,
            self.fps,
        )

    def stop(self) -> None:
        """Release the capture handle if held. Idempotent."""

        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    async def frames(self) -> AsyncIterator[Tuple[int, "np.ndarray"]]:  # type: ignore[name-defined]
        """Yield ``(monotonic_ns, frame_bgr)`` tuples until ``stop()``.

        On a read failure the loop waits one second and tries once more
        before raising. The retry covers a brief USB stall after suspend
        or hot-plug; persistent failure surfaces to the orchestrator so
        it can swap sources or alert.
        """

        if self._cap is None or not self._running:
            raise RuntimeError("v4l2 source not started")

        retry_used = False
        while self._running:
            ok, frame = await asyncio.to_thread(self._cap.read)
            stamp_ns = time.monotonic_ns()
            if not ok or frame is None:
                if retry_used:
                    raise RuntimeError(
                        f"v4l2 source {self.device_path!r} failed to deliver "
                        "frames after retry"
                    )
                log.warning(
                    "v4l2 source %s read failed; retrying after %.1fs",
                    self.device_path,
                    RETRY_DELAY_S,
                )
                await asyncio.sleep(RETRY_DELAY_S)
                retry_used = True
                continue
            retry_used = False
            yield stamp_ns, frame
