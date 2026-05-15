"""MAVLink gyro tap for derotation of optical flow.

Subscribes to the firmware's ``RAW_IMU`` (#27) stream and keeps the
most recent reading in a thread-safe attribute. The optical-flow
processor reads ``latest()`` once per frame and subtracts the body
angular rate from the visual rate to recover the pure translational
component.

RAW_IMU's gyro fields are in milliradians per second per MAVLink's
canonical units; this module converts to radians per second on
ingestion so downstream consumers can treat the value as SI.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

log = logging.getLogger(__name__)

MILLIRAD_PER_S_TO_RAD_PER_S = 0.001


@dataclass(frozen=True)
class GyroReading:
    """One RAW_IMU sample reduced to gyro axes in rad/s.

    ``ts`` is captured at receive time on the agent's monotonic clock
    so it can be compared with the camera frame timestamps without
    cross-clock drift.
    """

    ts: int
    xgyro: float
    ygyro: float
    zgyro: float


class GyroTap:
    """Subscribe to RAW_IMU and expose the latest gyro reading.

    The ``ctx`` argument is the plugin SDK's ``PluginContext``. It must
    expose ``ctx.mavlink.subscribe(msg_name, handler)`` returning a
    callable that, when invoked, removes the subscription.
    """

    def __init__(self, ctx: Any) -> None:
        self._ctx = ctx
        self._lock = threading.Lock()
        self._latest: Optional[GyroReading] = None
        self._unsubscribe: Optional[Callable[[], None]] = None

    def start(self) -> None:
        """Register the RAW_IMU subscription."""

        if self._unsubscribe is not None:
            return
        self._unsubscribe = self._ctx.mavlink.subscribe(
            "RAW_IMU", self._on_raw_imu
        )

    def stop(self) -> None:
        """Cancel the RAW_IMU subscription. Idempotent."""

        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception:  # noqa: BLE001 - tap teardown must not raise.
                log.warning("gyro tap unsubscribe raised; ignoring", exc_info=True)
            self._unsubscribe = None

    def latest(self) -> Optional[GyroReading]:
        """Return the most recent reading or ``None`` if no IMU data yet."""

        with self._lock:
            return self._latest

    def _on_raw_imu(self, data: Any) -> None:
        """Handler invoked by the SDK with one RAW_IMU payload.

        ``data`` may be a dict (test fixtures) or an object with
        attributes (live SDK). Both shapes are supported.
        """

        try:
            xgyro_raw = _field(data, "xgyro")
            ygyro_raw = _field(data, "ygyro")
            zgyro_raw = _field(data, "zgyro")
        except KeyError as exc:
            log.warning("RAW_IMU payload missing field %s", exc)
            return

        reading = GyroReading(
            ts=time.monotonic_ns(),
            xgyro=float(xgyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
            ygyro=float(ygyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
            zgyro=float(zgyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
        )
        with self._lock:
            self._latest = reading


def _field(data: Any, name: str) -> Any:
    """Read ``name`` from either a mapping or an attribute holder."""

    if isinstance(data, dict):
        if name not in data:
            raise KeyError(name)
        return data[name]
    if hasattr(data, name):
        return getattr(data, name)
    raise KeyError(name)
