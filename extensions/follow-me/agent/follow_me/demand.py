"""The detection lease on the follow camera.

The vision engine runs person detection on a camera only while a consumer
holds a lease on it. While the behaviour is active this plugin holds one on
the camera it follows: a request every ``RENEW_S`` for ``HOLD_S`` (so one
missed renewal never lets detection lapse), moved when the follow camera
changes, and released with a zero hold when the behaviour goes inactive or the
plugin stops.

An agent whose SDK predates ``request_detection`` has no lease to hold: the
plugin logs that once and the engine decides on its own where detection runs.
"""

from __future__ import annotations

import math
from typing import Any

# Renewal period and requested hold of the follow-camera lease.
RENEW_S = 2.0
HOLD_S = 6.0


class DetectionLease:
    """One plugin-owned detection lease, moved to whichever camera is wanted."""

    def __init__(self, renew_s: float = RENEW_S, hold_s: float = HOLD_S) -> None:
        self._renew_s = renew_s
        self._hold_s = hold_s
        # The camera the engine last granted a lease on, or None.
        self._held: str | None = None
        # The camera and time of the last request, so a refused request is
        # retried on the renewal period rather than on every loop tick.
        self._asked: str | None = None
        self._asked_at: float = -math.inf
        self._unsupported = False

    @property
    def camera(self) -> str | None:
        """The camera the lease is currently held on."""
        return self._held

    def _request(self, ctx: Any) -> Any:
        request = getattr(ctx.vision, "request_detection", None)
        if request is None and not self._unsupported:
            self._unsupported = True
            ctx.log.info("follow_me_detection_lease_unsupported")
        return request

    async def update(self, ctx: Any, camera: str | None, now: float) -> None:
        """Hold the lease on ``camera`` (``None``: hold none) as of ``now``."""
        if camera is None:
            await self.release(ctx)
            return
        if camera == self._asked and (now - self._asked_at) < self._renew_s:
            return
        request = self._request(ctx)
        if request is None:
            return
        if self._held is not None and self._held != camera:
            await self.release(ctx)
        self._asked = camera
        self._asked_at = now
        try:
            await request(camera, self._hold_s)
        except Exception as exc:  # noqa: BLE001
            self._held = None
            ctx.log.warning(
                "follow_me_detection_lease_failed", camera=camera, error=str(exc)
            )
            return
        self._held = camera

    async def release(self, ctx: Any) -> None:
        """Release the held lease, if any."""
        held = self._held
        self._held = None
        self._asked = None
        self._asked_at = -math.inf
        if held is None:
            return
        request = self._request(ctx)
        if request is None:
            return
        try:
            await request(held, 0.0)
        except Exception as exc:  # noqa: BLE001
            ctx.log.warning(
                "follow_me_detection_release_failed", camera=held, error=str(exc)
            )


__all__ = ["HOLD_S", "RENEW_S", "DetectionLease"]
