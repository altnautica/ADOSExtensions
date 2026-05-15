"""ST VL53L1X Time-of-Flight rangefinder driver over I2C.

Wraps the upstream `VL53L1X` Python library. The library is loaded lazily
from `open()` so the module imports cleanly on hosts that do not have it
installed; this matches how the pipeline orchestrator decides at runtime
which driver to use based on per-drone config.
"""
from __future__ import annotations

import asyncio
import enum
import time
from typing import Optional

from .base import RangefinderDriver, RangeReading

DEFAULT_I2C_ADDRESS = 0x29


class RangeMode(enum.IntEnum):
    """ToF range mode. Values match the upstream library."""

    SHORT = 1
    MEDIUM = 2
    LONG = 3


_MAX_RANGE_BY_MODE: dict[RangeMode, float] = {
    RangeMode.SHORT: 1.3,
    RangeMode.MEDIUM: 3.0,
    RangeMode.LONG: 4.0,
}


class Vl53l1xI2c(RangefinderDriver):
    """ST VL53L1X driver over an I2C bus."""

    def __init__(
        self,
        bus_number: int,
        address: int = DEFAULT_I2C_ADDRESS,
        range_mode: RangeMode = RangeMode.LONG,
    ) -> None:
        self._bus_number = bus_number
        self._address = address
        self._range_mode = range_mode
        self._sensor: Optional[object] = None

    @property
    def name(self) -> str:
        return "vl53l1x_i2c"

    @property
    def min_range_m(self) -> float:
        return 0.04

    @property
    def max_range_m(self) -> float:
        return _MAX_RANGE_BY_MODE.get(self._range_mode, 4.0)

    async def open(self) -> None:
        def _open() -> object:
            try:
                import VL53L1X  # type: ignore[import-not-found]
            except ImportError as exc:
                raise ImportError(
                    "VL53L1X Python library is required for the vl53l1x_i2c "
                    "driver. Install it with `pip install VL53L1X` or add the "
                    "[vl53l1x] extra to the plugin install."
                ) from exc
            sensor = VL53L1X.VL53L1X(
                i2c_bus=self._bus_number,
                i2c_address=self._address,
            )
            sensor.open()
            sensor.start_ranging(int(self._range_mode))
            return sensor

        self._sensor = await asyncio.to_thread(_open)

    async def close(self) -> None:
        sensor = self._sensor
        self._sensor = None
        if sensor is None:
            return

        def _close() -> None:
            try:
                sensor.stop_ranging()  # type: ignore[attr-defined]
            except Exception:
                pass
            try:
                sensor.close()  # type: ignore[attr-defined]
            except Exception:
                pass

        await asyncio.to_thread(_close)

    async def read(self) -> Optional[RangeReading]:
        sensor = self._sensor
        if sensor is None:
            return None

        def _read() -> Optional[int]:
            try:
                distance_mm = sensor.get_distance()  # type: ignore[attr-defined]
            except Exception:
                return None
            return distance_mm

        distance_mm = await asyncio.to_thread(_read)
        if distance_mm is None or distance_mm <= 0:
            return None
        distance_m = distance_mm / 1000.0
        in_range = self.min_range_m <= distance_m <= self.max_range_m
        quality = 100 if in_range else 0
        return RangeReading(
            distance_m=distance_m,
            quality=quality,
            timestamp_monotonic_ns=time.monotonic_ns(),
            raw_status=int(distance_mm),
        )
