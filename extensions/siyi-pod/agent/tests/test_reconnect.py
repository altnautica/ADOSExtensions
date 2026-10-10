"""Transport supervision: a failed or lost link is reopened on a fixed interval."""

from __future__ import annotations

import asyncio
import types

import pytest

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod import session as session_mod
from altnautica_siyi_pod.capability_profile import HW_ZT30
from altnautica_siyi_pod.plugin import SiyiPodPlugin
from altnautica_siyi_pod.session import SiyiSession, SiyiTimeout
from altnautica_siyi_pod.transport import MockTransport


class _Sleeps:
    """Stands in for the session's ``asyncio.sleep``: records each wait and,
    while ``gate`` is set, holds it until the test releases the gate."""

    def __init__(self) -> None:
        self.delays: list[float] = []
        self.gate: asyncio.Event | None = None

    async def sleep(self, delay, result=None):
        self.delays.append(delay)
        if self.gate is not None:
            await self.gate.wait()
        await asyncio.sleep(0)
        return result


@pytest.fixture
def sleeps(monkeypatch):
    fake = _Sleeps()
    proxy = types.SimpleNamespace(**vars(asyncio))
    proxy.sleep = fake.sleep
    monkeypatch.setattr(session_mod, "asyncio", proxy)
    return fake


async def _settle() -> None:
    for _ in range(20):
        await asyncio.sleep(0)


async def test_open_failure_is_retried_until_the_link_comes_up(sleeps):
    transport = MockTransport(model=HW_ZT30, fail_opens=2)
    session = SiyiSession(transport)
    sleeps.gate = asyncio.Event()

    await session.start()  # must not raise on an absent pod
    assert session.link_up is False
    with pytest.raises(SiyiTimeout):
        await session.request(C.request_hardware_id())

    sleeps.gate.set()
    await _settle()
    assert session.link_up is True
    assert transport.opens == 3
    # A fixed interval between attempts.
    assert sleeps.delays == [2.0, 2.0]
    reply = await session.request(C.request_hardware_id())
    assert reply.data[0] == HW_ZT30
    await session.stop()


async def test_lost_link_is_reopened_and_listeners_are_told(sleeps):
    transport = MockTransport(model=HW_ZT30)
    session = SiyiSession(transport)
    reconnects: list[int] = []
    session.add_reconnect_listener(lambda: reconnects.append(1))
    await session.start()
    assert session.link_up is True
    assert reconnects == []  # the first open is not a reconnect

    sleeps.gate = asyncio.Event()
    transport.drop()
    await _settle()
    assert session.link_up is False
    assert transport.opened is False
    with pytest.raises(SiyiTimeout):
        await session.request(C.request_hardware_id())

    sleeps.gate.set()
    await _settle()
    assert session.link_up is True
    assert reconnects == [1]
    reply = await session.request(C.request_hardware_id())
    assert reply.cmd_id == C.CMD_HARDWARE_ID
    await session.stop()


async def test_a_pending_request_fails_when_the_link_drops(sleeps):
    transport = MockTransport(model=HW_ZT30, answer_identity=False)
    session = SiyiSession(transport, timeout_s=5.0, retries=3)
    sleeps.gate = asyncio.Event()
    await session.start()
    pending = asyncio.create_task(session.request(C.request_hardware_id()))
    await _settle()
    transport.drop()
    with pytest.raises(SiyiTimeout):
        await asyncio.wait_for(pending, timeout=1.0)
    await session.stop()


class _ConfigKV:
    def __init__(self) -> None:
        self._d: dict = {}

    def static(self, key, default):
        return default

    async def get(self, key, default=None):
        return self._d.get(key, default)

    async def set(self, key, value) -> None:
        self._d[key] = value


class _Host:
    """Accepts every optional host call the plugin makes."""

    async def register_component(self, *_a):
        return None

    async def subscribe(self, *_a):
        return None

    async def send(self, *_a, **_k):
        return None

    async def extend(self, *_a):
        return None

    async def publish(self, *_a):
        return None


class _Ctx:
    def __init__(self) -> None:
        self.config_kv = _ConfigKV()
        self.mavlink = _Host()
        self.telemetry = _Host()
        self.events = _Host()


async def test_plugin_survives_an_absent_pod_and_renegotiates_on_reconnect(sleeps):
    ctx = _Ctx()
    transport = MockTransport(model=HW_ZT30, fail_opens=1)
    plugin = SiyiPodPlugin(transport_factory=lambda _cfg: transport)
    sleeps.gate = asyncio.Event()

    await plugin.on_start(ctx)  # the pod is absent: must not raise
    assert plugin.pod.negotiated is False
    assert plugin.state.connected is False
    await ctx.config_kv.set("zoom", 4.0)

    sleeps.gate.set()
    await _settle()
    await plugin.apply_config_once()
    assert plugin.pod.negotiated is True
    assert transport.zoom == 4.0

    # The pod is power-cycled: the link drops and comes back on its defaults.
    sleeps.gate.clear()
    transport.drop()
    await _settle()
    transport.zoom = 1.0
    sleeps.gate.set()
    await _settle()
    assert plugin.pod.negotiated is False
    assert plugin.state.connected is False

    # The control loop re-identifies the pod and re-applies the settings.
    await plugin.apply_config_once()
    assert plugin.pod.negotiated is True
    assert transport.zoom == 4.0
    await plugin.on_stop(ctx)
