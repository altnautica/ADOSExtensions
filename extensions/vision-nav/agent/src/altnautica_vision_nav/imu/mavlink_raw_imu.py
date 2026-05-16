"""RAW_IMU (msg #27) subscriber.

Translates the firmware's wire format (mg accel, mrad/s gyro) into SI
:class:`ImuSample` records. This is the universal IMU path: every FC
publishes RAW_IMU at the body-rate it can manage (typically 50–200 Hz
on ArduPilot, configurable via SR0_RAW_SENS).

The 100 Hz preferred path is :class:`MavlinkScaledImu2`, which the
plugin selects first when the FC publishes it. RAW_IMU is the
fallback that always works.
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


class MavlinkRawImu(BaseImuSource):
    """Subscribe to RAW_IMU and translate each sample to SI units."""

    source_id = "mavlink-raw-imu"

    def __init__(self, ctx: Any, *, buffer_capacity: int = 400) -> None:
        super().__init__(buffer_capacity=buffer_capacity)
        self._ctx = ctx
        self._lock = threading.Lock()
        self._unsubscribe: Optional[Callable[[], None]] = None

    def start(self) -> None:
        if self._unsubscribe is not None:
            return
        self._unsubscribe = self._ctx.mavlink.subscribe(
            "RAW_IMU", self._on_raw_imu
        )

    def stop(self) -> None:
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception:  # noqa: BLE001 - teardown must not raise
                log.warning(
                    "raw imu unsubscribe raised; ignoring", exc_info=True
                )
            self._unsubscribe = None

    def _on_raw_imu(self, data: Any) -> None:
        try:
            xgyro_raw = _field(data, "xgyro")
            ygyro_raw = _field(data, "ygyro")
            zgyro_raw = _field(data, "zgyro")
            xacc_raw = _field(data, "xacc")
            yacc_raw = _field(data, "yacc")
            zacc_raw = _field(data, "zacc")
        except KeyError as exc:
            log.warning("RAW_IMU payload missing field %s", exc)
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
    """Read ``name`` from either a mapping or an attribute holder."""

    if isinstance(data, dict):
        if name not in data:
            raise KeyError(name)
        return data[name]
    if hasattr(data, name):
        return getattr(data, name)
    raise KeyError(name)
