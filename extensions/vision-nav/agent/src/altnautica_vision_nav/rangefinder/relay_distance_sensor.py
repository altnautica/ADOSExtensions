"""Flight-controller-owned rangefinder relay.

The flight controller is already publishing MAVLink DISTANCE_SENSOR (#132)
on the same link the plugin is bridged to. This driver subscribes to that
stream and republishes the latest reading through the common RangefinderDriver
contract, so the rest of the pipeline can stay sensor-agnostic.
"""
from __future__ import annotations

import time
from typing import Any, Callable, Mapping, Optional

from .base import RangefinderDriver, RangeReading

DISTANCE_SENSOR_MESSAGE = "DISTANCE_SENSOR"
STALE_AFTER_NS = 200_000_000  # 200 ms
FALLBACK_MIN_M = 0.0
FALLBACK_MAX_M = 40.0
ACCEPT_ANY_SENSOR_ID = 0


class RelayDistanceSensor(RangefinderDriver):
    """Republish DISTANCE_SENSOR frames already flowing on the MAVLink bus."""

    def __init__(
        self,
        ctx: Any,
        sensor_id: int = ACCEPT_ANY_SENSOR_ID,
    ) -> None:
        self._ctx = ctx
        self._sensor_id = sensor_id
        self._latest: Optional[RangeReading] = None
        self._last_min_m: Optional[float] = None
        self._last_max_m: Optional[float] = None
        self._callback: Optional[Callable[[Mapping[str, Any]], None]] = None
        self._subscribed = False

    @property
    def name(self) -> str:
        return "fc_relay"

    @property
    def min_range_m(self) -> float:
        if self._last_min_m is not None:
            return self._last_min_m
        return FALLBACK_MIN_M

    @property
    def max_range_m(self) -> float:
        if self._last_max_m is not None:
            return self._last_max_m
        return FALLBACK_MAX_M

    async def open(self) -> None:
        if self._subscribed:
            return
        self._callback = self._on_distance
        self._ctx.mavlink.subscribe(DISTANCE_SENSOR_MESSAGE, self._callback)
        self._subscribed = True

    async def close(self) -> None:
        if not self._subscribed:
            return
        unsubscribe = getattr(self._ctx.mavlink, "unsubscribe", None)
        if unsubscribe is not None and self._callback is not None:
            try:
                unsubscribe(DISTANCE_SENSOR_MESSAGE, self._callback)
            except Exception:
                pass
        self._subscribed = False
        self._callback = None

    async def read(self) -> Optional[RangeReading]:
        latest = self._latest
        if latest is None:
            return None
        age_ns = time.monotonic_ns() - latest.timestamp_monotonic_ns
        if age_ns > STALE_AFTER_NS:
            return None
        return latest

    def _on_distance(self, data: Mapping[str, Any]) -> None:
        sensor_id = int(data.get("id", 0))
        if self._sensor_id != ACCEPT_ANY_SENSOR_ID and sensor_id != self._sensor_id:
            return
        current_cm = data.get("current_distance")
        if current_cm is None:
            return
        try:
            distance_m = float(current_cm) / 100.0
        except (TypeError, ValueError):
            return

        min_cm = data.get("min_distance")
        max_cm = data.get("max_distance")
        if min_cm is not None:
            try:
                self._last_min_m = float(min_cm) / 100.0
            except (TypeError, ValueError):
                pass
        if max_cm is not None:
            try:
                self._last_max_m = float(max_cm) / 100.0
            except (TypeError, ValueError):
                pass

        covariance = data.get("covariance", 0)
        try:
            covariance_int = int(covariance)
        except (TypeError, ValueError):
            covariance_int = 0
        quality = max(0, min(100, 100 - covariance_int))

        self._latest = RangeReading(
            distance_m=distance_m,
            quality=quality,
            timestamp_monotonic_ns=time.monotonic_ns(),
            raw_status=sensor_id,
        )
