"""MAVLink interop-bridge tests: real frames that decode back correctly."""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import common as mavlink2

from altnautica_siyi_pod.mavlink_bridge import (
    COMP_GIMBAL,
    SiyiMavlinkBridge,
    euler_to_quaternion,
)


class _FakeMavlink:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, int | None]] = []

    async def send(self, frame: bytes, component_id: int | None = None) -> None:
        self.sent.append((frame, component_id))


class _FakeCtx:
    def __init__(self) -> None:
        self.mavlink = _FakeMavlink()


def _decode(frame: bytes):
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    msgs = parser.parse_buffer(frame)
    assert msgs, "frame did not decode to a MAVLink message"
    return msgs[0]


def test_euler_quaternion_identity_and_norm():
    q = euler_to_quaternion(0.0, 0.0, 0.0)
    assert math.isclose(q[0], 1.0)
    assert all(math.isclose(v, 0.0, abs_tol=1e-9) for v in q[1:])
    q2 = euler_to_quaternion(10.0, -20.0, 30.0)
    norm = math.sqrt(sum(v * v for v in q2))
    assert math.isclose(norm, 1.0, rel_tol=1e-6)


async def test_attitude_frame_decodes():
    ctx = _FakeCtx()
    bridge = SiyiMavlinkBridge(ctx, system_id=1)
    await bridge.send_attitude(30.0, -20.0, 0.0)
    frame, comp = ctx.mavlink.sent[0]
    assert comp == COMP_GIMBAL
    msg = _decode(frame)
    assert msg.get_type() == "GIMBAL_DEVICE_ATTITUDE_STATUS"
    assert len(msg.q) == 4
    assert math.isclose(sum(v * v for v in msg.q), 1.0, rel_tol=1e-5)


async def test_distance_frame_decodes_with_range():
    ctx = _FakeCtx()
    bridge = SiyiMavlinkBridge(ctx, system_id=1)
    await bridge.send_distance(42.0)
    frame, comp = ctx.mavlink.sent[0]
    assert comp == COMP_GIMBAL
    msg = _decode(frame)
    assert msg.get_type() == "DISTANCE_SENSOR"
    assert msg.current_distance == 4200  # centimetres
    assert msg.max_distance == 0xFFFF  # uint16 cm field ceiling (655.35 m)


async def test_distance_frame_clamps_beyond_field_ceiling():
    ctx = _FakeCtx()
    bridge = SiyiMavlinkBridge(ctx, system_id=1)
    await bridge.send_distance(900.0)  # beyond the uint16-cm ceiling
    frame, _comp = ctx.mavlink.sent[0]
    msg = _decode(frame)
    assert msg.current_distance == 0xFFFF  # saturates; true range is in telemetry
