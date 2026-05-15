"""Tests for the rangefinder driver implementations.

Synthetic-input tests cover the TF-Luna packet parser and the FC-relay
DISTANCE_SENSOR republisher. Hardware-bound tests for Garmin LIDAR-Lite and
ST VL53L1X are skipped here; they require a real I2C bus and physical sensor.
"""
from __future__ import annotations

import asyncio
from typing import Any, Callable, Mapping

import pytest

from altnautica_vision_nav.rangefinder.relay_distance_sensor import RelayDistanceSensor
from altnautica_vision_nav.rangefinder.tfluna_uart import find_frame, parse_frame


def _build_tfluna_packet(distance_cm: int, signal_strength: int) -> bytes:
    """Build a valid 9-byte TF-Luna packet for tests."""
    payload = bytes(
        [
            0x59,
            0x59,
            distance_cm & 0xFF,
            (distance_cm >> 8) & 0xFF,
            signal_strength & 0xFF,
            (signal_strength >> 8) & 0xFF,
            0x00,
            0x00,
        ]
    )
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum])


def test_tfluna_uart_parse_valid_packet() -> None:
    """A 1.50 m / strength 10000 packet parses to distance_m=1.5, quality=50."""
    packet = _build_tfluna_packet(distance_cm=150, signal_strength=10000)
    parsed = parse_frame(packet)
    assert parsed is not None
    distance_m, quality, raw_status = parsed
    assert distance_m == pytest.approx(1.5)
    assert quality == 50
    assert raw_status == 10000


def test_tfluna_uart_invalid_checksum() -> None:
    """Corrupting the checksum byte rejects the frame."""
    packet = bytearray(_build_tfluna_packet(distance_cm=150, signal_strength=10000))
    packet[8] ^= 0xFF
    assert parse_frame(bytes(packet)) is None


def test_tfluna_find_frame_skips_garbage_before_header() -> None:
    """The frame scanner re-syncs on the 0x59 0x59 header after garbage."""
    packet = _build_tfluna_packet(distance_cm=100, signal_strength=2000)
    buffer = b"\x00\x12\x34" + packet + b"\xaa"
    frame, remaining = find_frame(buffer)
    assert frame is not None
    parsed = parse_frame(frame)
    assert parsed is not None
    distance_m, _quality, _raw = parsed
    assert distance_m == pytest.approx(1.0)
    # The trailing 0xaa byte is preserved for the next scan.
    assert remaining.endswith(b"\xaa")


class _FakeMavlink:
    """Minimal stand-in for ctx.mavlink that captures subscriptions."""

    def __init__(self) -> None:
        self.subscriptions: dict[str, Callable[[Mapping[str, Any]], None]] = {}

    def subscribe(
        self,
        message: str,
        callback: Callable[[Mapping[str, Any]], None],
    ) -> None:
        self.subscriptions[message] = callback

    def unsubscribe(
        self,
        message: str,
        callback: Callable[[Mapping[str, Any]], None],
    ) -> None:
        if self.subscriptions.get(message) is callback:
            self.subscriptions.pop(message, None)


class _FakeCtx:
    """Minimal stand-in for a PluginContext object."""

    def __init__(self) -> None:
        self.mavlink = _FakeMavlink()


def test_relay_distance_sensor_subscribes_and_reads() -> None:
    """A DISTANCE_SENSOR frame fired into the captured callback round-trips."""
    ctx = _FakeCtx()
    driver = RelayDistanceSensor(ctx)

    asyncio.run(driver.open())

    callback = ctx.mavlink.subscriptions.get("DISTANCE_SENSOR")
    assert callback is not None

    callback(
        {
            "time_boot_ms": 12345,
            "min_distance": 20,
            "max_distance": 800,
            "current_distance": 250,
            "type": 0,
            "id": 0,
            "orientation": 25,
            "covariance": 5,
        }
    )

    reading = asyncio.run(driver.read())
    assert reading is not None
    assert reading.distance_m == pytest.approx(2.5)
    assert reading.quality == 95
    assert driver.min_range_m == pytest.approx(0.2)
    assert driver.max_range_m == pytest.approx(8.0)

    asyncio.run(driver.close())
    assert "DISTANCE_SENSOR" not in ctx.mavlink.subscriptions


def test_relay_distance_sensor_filters_by_sensor_id() -> None:
    """When sensor_id is non-zero, frames with a different id are dropped."""
    ctx = _FakeCtx()
    driver = RelayDistanceSensor(ctx, sensor_id=2)
    asyncio.run(driver.open())
    callback = ctx.mavlink.subscriptions["DISTANCE_SENSOR"]
    callback(
        {
            "current_distance": 300,
            "min_distance": 10,
            "max_distance": 4000,
            "id": 1,
            "covariance": 0,
        }
    )
    assert asyncio.run(driver.read()) is None
    callback(
        {
            "current_distance": 300,
            "min_distance": 10,
            "max_distance": 4000,
            "id": 2,
            "covariance": 0,
        }
    )
    reading = asyncio.run(driver.read())
    assert reading is not None
    assert reading.distance_m == pytest.approx(3.0)
    asyncio.run(driver.close())


def test_relay_distance_sensor_fallback_ranges_before_first_message() -> None:
    """Before any DISTANCE_SENSOR is seen, min/max fall back to safe defaults."""
    ctx = _FakeCtx()
    driver = RelayDistanceSensor(ctx)
    assert driver.min_range_m == pytest.approx(0.0)
    assert driver.max_range_m == pytest.approx(40.0)


@pytest.mark.skip(reason="requires I2C hardware")
def test_garmin_lidarlite_hardware() -> None:
    """Smoke test for the Garmin driver on a real I2C bus."""


@pytest.mark.skip(reason="requires I2C hardware")
def test_vl53l1x_hardware() -> None:
    """Smoke test for the VL53L1X driver on a real I2C bus."""
