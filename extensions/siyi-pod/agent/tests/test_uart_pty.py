"""UART transport over a pseudo-terminal pair.

The slave end stands in for the pod's serial adapter: the transport opens it
through pyserial, the test plays the pod on the master end, and a request goes
out and its framed ack comes back through the event loop's reader callback.
"""

from __future__ import annotations

import asyncio
import os

import pytest

pytest.importorskip("pty")
pytest.importorskip("serial")
if not hasattr(os, "openpty"):
    pytest.skip("os.openpty is unavailable on this platform", allow_module_level=True)

from altnautica_siyi_pod import commands as C  # noqa: E402
from altnautica_siyi_pod.capability_profile import HW_ZT30  # noqa: E402
from altnautica_siyi_pod.framing import build_frame, iter_frames  # noqa: E402
from altnautica_siyi_pod.session import SiyiSession  # noqa: E402
from altnautica_siyi_pod.transport import UartTransport  # noqa: E402


async def _read_frame(fd: int, timeout_s: float = 2.0):
    """Read from the pty master until one whole frame has arrived."""
    buf = b""
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        try:
            buf += os.read(fd, 4096)
        except BlockingIOError:
            await asyncio.sleep(0.01)
            continue
        frames, buf = iter_frames(buf)
        if frames:
            return frames[0]
    raise AssertionError("no frame arrived on the pty master")


async def test_request_and_framed_ack_round_trip_over_a_pty():
    master, slave = os.openpty()
    os.set_blocking(master, False)
    transport = UartTransport(os.ttyname(slave))
    session = SiyiSession(transport, timeout_s=2.0, retries=0)
    try:
        await session.start()
        assert session.link_up

        pending = asyncio.create_task(session.request(C.request_hardware_id()))
        request = await _read_frame(master)
        assert request.cmd_id == C.CMD_HARDWARE_ID

        # The pod answers with the same sequence number.
        os.write(
            master,
            build_frame(C.CMD_HARDWARE_ID, bytes([HW_ZT30, 0x00]), seq=request.seq),
        )
        reply = await asyncio.wait_for(pending, timeout=2.0)
        assert reply.cmd_id == C.CMD_HARDWARE_ID
        assert reply.data[0] == HW_ZT30
        assert session.frames_received == 1
    finally:
        await session.stop()
        os.close(master)
        os.close(slave)
