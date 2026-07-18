"""Shared async builders for the SIYI pod tests (all mock-backed)."""

from __future__ import annotations

from altnautica_siyi_pod.capability_profile import HW_ZT30
from altnautica_siyi_pod.pod import SiyiPod
from altnautica_siyi_pod.session import SiyiSession
from altnautica_siyi_pod.transport import MockTransport


async def make_session(model: int = HW_ZT30, **kwargs):
    transport = MockTransport(model=model, **kwargs)
    session = SiyiSession(transport)
    await session.start()
    return session, transport


async def make_pod(model: int = HW_ZT30, **kwargs):
    session, transport = await make_session(model, **kwargs)
    pod = SiyiPod(session)
    await pod.negotiate()
    return pod, session, transport
