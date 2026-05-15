"""Garmin LIDAR-Lite v3HP rangefinder driver over I2C.

Register map (subset):
    0x00  ACQ_COMMAND              -- write 0x04 to start a measurement
    0x01  STATUS                   -- bit 0 == 1 while busy
    0x0E  SIGNAL_STRENGTH          -- 0..255, higher is better
    0x8F  DISTANCE_HIGH_AND_LOW    -- 2-byte auto-increment read (big-endian cm)
"""
from __future__ import annotations

import asyncio
import time
from typing import Optional

from smbus2 import SMBus

from .base import RangefinderDriver, RangeReading

REG_ACQ_COMMAND = 0x00
REG_STATUS = 0x01
REG_SIGNAL_STRENGTH = 0x0E
REG_DISTANCE = 0x8F
CMD_TAKE_MEASUREMENT = 0x04
STATUS_BUSY_BIT = 0x01
BUSY_POLL_INTERVAL_S = 0.001
BUSY_POLL_TIMEOUT_S = 0.05
GOOD_SIGNAL_THRESHOLD = 200


class GarminLidarLiteI2c(RangefinderDriver):
    """Garmin LIDAR-Lite v3HP driver over an I2C bus."""

    def __init__(self, bus_number: int, address: int = 0x62) -> None:
        self._bus_number = bus_number
        self._address = address
        self._bus: Optional[SMBus] = None

    @property
    def name(self) -> str:
        return "garmin_lidarlite_i2c"

    @property
    def min_range_m(self) -> float:
        return 0.05

    @property
    def max_range_m(self) -> float:
        return 40.0

    async def open(self) -> None:
        self._bus = await asyncio.to_thread(SMBus, self._bus_number)

    async def close(self) -> None:
        bus = self._bus
        self._bus = None
        if bus is not None:
            await asyncio.to_thread(bus.close)

    async def read(self) -> Optional[RangeReading]:
        bus = self._bus
        if bus is None:
            return None

        def _measure() -> Optional[tuple[float, int]]:
            bus.write_byte_data(self._address, REG_ACQ_COMMAND, CMD_TAKE_MEASUREMENT)
            deadline = time.monotonic() + BUSY_POLL_TIMEOUT_S
            while True:
                status = bus.read_byte_data(self._address, REG_STATUS)
                if not (status & STATUS_BUSY_BIT):
                    break
                if time.monotonic() >= deadline:
                    return None
                time.sleep(BUSY_POLL_INTERVAL_S)
            distance_bytes = bus.read_i2c_block_data(self._address, REG_DISTANCE, 2)
            distance_cm = (distance_bytes[0] << 8) | distance_bytes[1]
            signal_strength = bus.read_byte_data(self._address, REG_SIGNAL_STRENGTH)
            return distance_cm / 100.0, signal_strength

        result = await asyncio.to_thread(_measure)
        if result is None:
            return None
        distance_m, signal_strength = result
        if signal_strength > GOOD_SIGNAL_THRESHOLD:
            quality = 100
        else:
            quality = int(signal_strength * 100 / GOOD_SIGNAL_THRESHOLD)
        quality = max(0, min(100, quality))
        return RangeReading(
            distance_m=distance_m,
            quality=quality,
            timestamp_monotonic_ns=time.monotonic_ns(),
            raw_status=signal_strength,
        )
