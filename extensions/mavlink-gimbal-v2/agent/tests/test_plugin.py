"""Tests for the plugin lifecycle + the aim safety gate.

A fake ``PluginContext`` mocks the four surfaces the plugin touches
(``mavlink``, ``vision``, ``config_kv``, ``events``). The tests assert
that ``on_start`` wires the detection subscription and registers the
gimbal component, that a vision-locked target produces a gimbal command,
and -- the safety gate -- that an uncertain / lost / untracked target, or
an inactive aim flag, produces NO command.
"""

from __future__ import annotations

import asyncio

import pytest
from pymavlink.dialects.v20 import common as mavlink2

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch
from altnautica_gimbal_v2.mavlink_messages import MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
from altnautica_gimbal_v2.plugin import GimbalV2Plugin


class FakeMavlink:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, int | None]] = []
        self.registered: list[tuple[int, str]] = []

    async def send(self, msg_bytes: bytes, component_id: int | None = None) -> dict:
        self.sent.append((bytes(msg_bytes), component_id))
        return {}

    async def register_component(self, comp_id: int, kind: str) -> dict:
        self.registered.append((comp_id, kind))
        return {}


class FakeVision:
    def __init__(self) -> None:
        self.callback = None
        self.camera_id: object = "UNSET"

    async def subscribe_detections(self, callback, *, camera_id=None) -> None:
        self.callback = callback
        self.camera_id = camera_id


class FakeConfigKv:
    def __init__(self, live=None, static=None) -> None:
        self._live = dict(live or {})
        self._static = dict(static or {})

    def static(self, key, default=None):
        return self._static.get(key, default)

    async def get(self, key, default=None):
        return self._live.get(key, default)


class FakeEvents:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    async def publish(self, topic, payload=None) -> int:
        self.published.append((topic, payload))
        return 0


class FakeCtx:
    def __init__(self, live=None, static=None) -> None:
        self.mavlink = FakeMavlink()
        self.vision = FakeVision()
        self.config_kv = FakeConfigKv(live=live, static=static)
        self.events = FakeEvents()


def _locked_batch(lock_state: str | None, track_id: int | None) -> DetectionBatch:
    """A single off-centre detection (right of frame centre) so, when the
    gate passes, the controller yields a non-zero command."""
    det = Detection(
        bbox=BoundingBox(x=900.0, y=350.0, width=40.0, height=40.0),
        class_label="person",
        confidence=0.9,
        track_id=track_id,
        lock_state=lock_state,
    )
    return DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=1,
        detections=[det],
    )


async def _settle() -> None:
    for _ in range(5):
        await asyncio.sleep(0)


@pytest.mark.asyncio
async def test_on_start_subscribes_and_registers_component() -> None:
    ctx = FakeCtx(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        assert ctx.vision.callback is not None
        # designate_camera defaults to "" -> None (all cameras).
        assert ctx.vision.camera_id is None
        assert (154, "gimbal") in ctx.mavlink.registered
        assert any(t == "sensor.gimbal.health" for t, _ in ctx.events.published)
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_locked_target_produces_a_gimbal_command() -> None:
    ctx = FakeCtx(live={"aim": True})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()  # drain the open-time configure send
        ctx.mavlink.sent.clear()

        await ctx.vision.callback(_locked_batch("locked", track_id=7))
        await _settle()

        assert len(ctx.mavlink.sent) == 1
        frame, comp = ctx.mavlink.sent[0]
        assert comp == 191
        # It is a real gimbal-manager pitch/yaw command.
        msg = mavlink2.MAVLink(None).decode(bytearray(frame))
        assert msg.get_type() == "COMMAND_LONG"
        assert msg.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "lock_state, track_id",
    [
        ("uncertain", 7),
        ("lost", 7),
        (None, 7),
        ("locked", None),  # locked but not tracked -> not eligible
    ],
)
async def test_unlocked_target_produces_no_command(
    lock_state: str | None, track_id: int | None
) -> None:
    ctx = FakeCtx(live={"aim": True})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        ctx.mavlink.sent.clear()

        await ctx.vision.callback(_locked_batch(lock_state, track_id))
        await _settle()

        assert ctx.mavlink.sent == []  # the safety gate blocks the command
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_aim_inactive_produces_no_command() -> None:
    ctx = FakeCtx(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        ctx.mavlink.sent.clear()

        # Even a perfectly locked target is ignored while aim is off.
        await ctx.vision.callback(_locked_batch("locked", track_id=7))
        await _settle()

        assert ctx.mavlink.sent == []
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_non_mavlink_transport_skips_aim_wiring() -> None:
    ctx = FakeCtx(live={"aim": True, "transport": "sbgc-uart"})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        # The stub serial driver reports no candidates, so on_start returns
        # early: no component registration, no detection subscription.
        assert ctx.vision.callback is None
        assert ctx.mavlink.registered == []
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_on_stop_closes_cleanly_and_is_idempotent() -> None:
    ctx = FakeCtx(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    assert plugin.session is not None
    assert plugin.driver is not None

    await plugin.on_stop(ctx)
    assert plugin.session is None
    assert plugin.driver is None

    # A second on_stop must not raise.
    await plugin.on_stop(ctx)
