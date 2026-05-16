"""SCALED_IMU2 (msg #116) subscriber.

When the FC publishes SCALED_IMU2 we prefer it: ArduPilot streams the
second IMU at a configurable rate (commonly 100 Hz) and the units are
explicit on the wire (mG, mrad/s). On boards that do not publish it
the plugin falls back to RAW_IMU automatically.

This source uses the same SI translation as RAW_IMU but reads from a
different message and tags itself with ``source_id =
"mavlink-scaled-imu2"`` so the GCS sensors card can surface which
path the estimator is consuming.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

from altnautica_vision_nav.imu.base import BaseImuSource, ImuSample

log = logging.getLogger(__name__)

MILLIRAD_PER_S_TO_RAD_PER_S = 0.001
MG_TO_M_PER_S2 = 9.80665 * 0.001


class MavlinkScaledImu2(BaseImuSource):
    """Subscribe to SCALED_IMU2 and translate each sample to SI units."""

    source_id = "mavlink-scaled-imu2"

    def __init__(self, ctx: Any, *, buffer_capacity: int = 400) -> None:
        super().__init__(buffer_capacity=buffer_capacity)
        self._ctx = ctx
        self._lock = threading.Lock()
        self._unsubscribe: Optional[Callable[[], None]] = None

    def start(self) -> None:
        if self._unsubscribe is not None:
            return
        self._unsubscribe = self._ctx.mavlink.subscribe(
            "SCALED_IMU2", self._on_scaled_imu2
        )

    def stop(self) -> None:
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception:  # noqa: BLE001 - teardown must not raise
                log.warning(
                    "scaled imu2 unsubscribe raised; ignoring", exc_info=True
                )
            self._unsubscribe = None

    def _on_scaled_imu2(self, data: Any) -> None:
        try:
            xgyro_raw = _field(data, "xgyro")
            ygyro_raw = _field(data, "ygyro")
            zgyro_raw = _field(data, "zgyro")
            xacc_raw = _field(data, "xacc")
            yacc_raw = _field(data, "yacc")
            zacc_raw = _field(data, "zacc")
        except KeyError as exc:
            log.warning("SCALED_IMU2 payload missing field %s", exc)
            return

        sample = ImuSample(
            ts_ns=time.monotonic_ns(),
            xgyro=float(xgyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
            ygyro=float(ygyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
            zgyro=float(zgyro_raw) * MILLIRAD_PER_S_TO_RAD_PER_S,
            xacc=float(xacc_raw) * MG_TO_M_PER_S2,
            yacc=float(yacc_raw) * MG_TO_M_PER_S2,
            zacc=float(zacc_raw) * MG_TO_M_PER_S2,
        )
        with self._lock:
            self._record(sample)


def _field(data: Any, name: str) -> Any:
    if isinstance(data, dict):
        if name not in data:
            raise KeyError(name)
        return data[name]
    if hasattr(data, name):
        return getattr(data, name)
    raise KeyError(name)
