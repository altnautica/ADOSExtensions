"""Session arbitration tests: seq correlation, concurrency, push, timeout."""

from __future__ import annotations

import asyncio

import pytest

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.capability_profile import HW_ZT30
from altnautica_siyi_pod.framing import Frame
from altnautica_siyi_pod.session import SiyiSession, SiyiTimeout
from helpers import make_session


async def test_request_correlates_reply():
    session, _t = await make_session(model=HW_ZT30)
    reply = await session.request(C.request_hardware_id())
    assert reply.cmd_id == C.CMD_HARDWARE_ID
    assert reply.data[0] == HW_ZT30
    await session.stop()


async def test_concurrent_requests_each_get_their_reply():
    session, _t = await make_session()
    hw, att, rng = await asyncio.gather(
        session.request(C.request_hardware_id()),
        session.request(C.request_gimbal_attitude()),
        session.request(C.request_laser_range()),
    )
    assert hw.cmd_id == C.CMD_HARDWARE_ID
    assert att.cmd_id == C.CMD_GIMBAL_ATTITUDE
    assert rng.cmd_id == C.CMD_LASER_RANGE
    await session.stop()


async def test_push_frames_fan_out_by_cmd_id():
    session, transport = await make_session()
    received: list[Frame] = []
    session.subscribe(received.append, cmd_id=C.CMD_GIMBAL_ATTITUDE)
    transport.push_attitude()
    assert len(received) == 1
    assert received[0].cmd_id == C.CMD_GIMBAL_ATTITUDE
    await session.stop()


async def test_liveness_counter_advances():
    session, _t = await make_session()
    before = session.frames_received
    await session.request(C.request_hardware_id())
    assert session.frames_received == before + 1
    await session.stop()


class _SilentTransport:
    """A transport that accepts sends but never replies (dead pod)."""

    def set_on_bytes(self, _cb):
        pass

    async def open(self):
        pass

    async def close(self):
        pass

    async def send(self, _frame):
        pass


async def test_timeout_and_retry_budget():
    session = SiyiSession(_SilentTransport(), timeout_s=0.01, retries=1)
    await session.start()
    with pytest.raises(SiyiTimeout):
        await session.request(C.request_hardware_id())
    await session.stop()
