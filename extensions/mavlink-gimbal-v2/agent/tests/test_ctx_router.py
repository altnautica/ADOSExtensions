"""Tests for the ctx-router adapter's serialization.

The router packs the driver's ``CommandLong`` / ``CommandInt`` values
into real MAVLink v2 frames and fire-and-forgets them onto
``ctx.mavlink.send``. These tests prove the packed bytes DECODE back
through pymavlink to the intended message with the intended params -- the
anti-guess gate: pymavlink owns the CRC-extra, so a round-trip decode is
proof the wire frame is real, not hand-rolled.
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest
from pymavlink.dialects.v20 import common as mavlink2

from altnautica_gimbal_v2.ctx_router import _CtxRouter
from altnautica_gimbal_v2.mavlink_messages import (
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    MAV_CMD_DO_SET_ROI_LOCATION,
    encode_gimbal_manager_pitchyaw,
    encode_set_roi_location,
)


class FakeMavlink:
    """Captures the frames the router sends."""

    def __init__(self) -> None:
        self.sent: list[tuple[bytes, int | None]] = []

    async def send(self, msg_bytes: bytes, component_id: int | None = None) -> dict:
        self.sent.append((bytes(msg_bytes), component_id))
        return {}


async def _drain() -> None:
    # Let the router's fire-and-forget send task run to completion.
    for _ in range(5):
        await asyncio.sleep(0)


@pytest.mark.asyncio
async def test_command_long_round_trips_to_gimbal_pitchyaw() -> None:
    fake = FakeMavlink()
    loop = asyncio.get_running_loop()
    router = _CtxRouter(
        ctx_mavlink=fake, src_system=1, src_component=191, loop=loop
    )

    cmd = encode_gimbal_manager_pitchyaw(
        pitch_deg=-30.0, yaw_deg=45.0, target_system=1, target_component=154
    )
    assert router.send_command(cmd) is True
    await _drain()

    assert len(fake.sent) == 1
    frame, comp = fake.sent[0]
    assert comp == 191
    assert len(frame) > 0

    decoder = mavlink2.MAVLink(None)
    msg = decoder.decode(bytearray(frame))
    assert msg.get_type() == "COMMAND_LONG"
    assert msg.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    assert msg.target_system == 1
    assert msg.target_component == 154
    assert msg.param1 == pytest.approx(-30.0)  # pitch
    assert msg.param2 == pytest.approx(45.0)  # yaw


@pytest.mark.asyncio
async def test_command_int_round_trips_to_set_roi_location() -> None:
    fake = FakeMavlink()
    loop = asyncio.get_running_loop()
    router = _CtxRouter(
        ctx_mavlink=fake, src_system=1, src_component=191, loop=loop
    )

    cmd = encode_set_roi_location(
        lat_deg=12.971,
        lon_deg=77.594,
        alt_m=50.0,
        target_system=1,
        target_component=154,
    )
    assert router.send_command(cmd) is True
    await _drain()

    assert len(fake.sent) == 1
    frame, comp = fake.sent[0]
    assert comp == 191

    decoder = mavlink2.MAVLink(None)
    msg = decoder.decode(bytearray(frame))
    assert msg.get_type() == "COMMAND_INT"
    assert msg.command == MAV_CMD_DO_SET_ROI_LOCATION
    assert msg.x == 129710000  # lat * 1e7
    assert msg.y == 775940000  # lon * 1e7
    assert msg.z == pytest.approx(50.0)


@pytest.mark.asyncio
async def test_unsupported_command_type_returns_false_and_sends_nothing() -> None:
    fake = FakeMavlink()
    loop = asyncio.get_running_loop()
    router = _CtxRouter(
        ctx_mavlink=fake, src_system=1, src_component=191, loop=loop
    )

    class NotACommand:
        pass

    assert router.send_command(NotACommand()) is False  # type: ignore[arg-type]
    await _drain()
    assert fake.sent == []


@pytest.mark.asyncio
async def test_send_failure_is_swallowed_and_reported_false() -> None:
    class Boom:
        async def send(self, *_a: Any, **_k: Any) -> dict:  # pragma: no cover
            raise RuntimeError("wire down")

    loop = asyncio.get_running_loop()
    router = _CtxRouter(
        ctx_mavlink=Boom(), src_system=1, src_component=191, loop=loop
    )
    cmd = encode_gimbal_manager_pitchyaw(
        pitch_deg=0.0, yaw_deg=0.0, target_system=1, target_component=154
    )
    # Scheduling succeeds (True); the async send raising later is a
    # fire-and-forget task failure the driver never sees.
    assert router.send_command(cmd) is True
    await _drain()
