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

    async def set(self, key, value, scope="drone"):
        self._live[key] = value
        return {"ok": True}


class FakeTools:
    def __init__(self) -> None:
        self.handlers: dict = {}

    def register(self, name, handler) -> None:
        self.handlers[name] = handler


class FakeEvents:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    async def publish(self, topic, payload=None) -> int:
        self.published.append((topic, payload))
        return 0


class FakeCtx:
    def __init__(self, live=None, static=None, with_tools=False) -> None:
        self.mavlink = FakeMavlink()
        self.vision = FakeVision()
        self.config_kv = FakeConfigKv(live=live, static=static)
        self.events = FakeEvents()
        if with_tools:
            self.tools = FakeTools()


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


def _decode(frame: bytes):
    return mavlink2.MAVLink(None).decode(bytearray(frame))


@pytest.mark.asyncio
async def test_recenter_action_centres_and_resets_the_key() -> None:
    ctx = FakeCtx(live={"aim": False, "recenter": True})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        ctx.mavlink.sent.clear()

        await plugin.poll_control_once()
        await _settle()

        # One gimbal-manager command to (0, 0).
        assert len(ctx.mavlink.sent) == 1
        msg = _decode(ctx.mavlink.sent[0][0])
        assert msg.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        assert msg.param1 == 0.0 and msg.param2 == 0.0
        # The one-shot key is cleared so a re-press fires again.
        assert await ctx.config_kv.get("recenter", False) is False
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_nadir_action_points_straight_down() -> None:
    ctx = FakeCtx(live={"aim": False, "nadir": True})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        ctx.mavlink.sent.clear()

        await plugin.poll_control_once()
        await _settle()

        assert len(ctx.mavlink.sent) == 1
        msg = _decode(ctx.mavlink.sent[0][0])
        # Nadir points to the driver's lower pitch limit (straight down).
        assert msg.param1 == plugin._nadir_pitch
        assert msg.param1 <= -90.0 or msg.param1 < 0.0
        assert await ctx.config_kv.get("nadir", False) is False
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_rate_mode_sends_a_rate_command() -> None:
    # aim on + rate_mode on: a locked off-centre target drives a RATE command
    # (non-zero pitch/yaw rate slots), not an absolute-angle command.
    ctx = FakeCtx(live={"aim": True, "rate_mode": True})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        # Pick up rate_mode from config (also set at start here).
        await plugin.poll_control_once()
        ctx.mavlink.sent.clear()

        await ctx.vision.callback(_locked_batch("locked", track_id=7))
        await _settle()

        assert len(ctx.mavlink.sent) == 1
        msg = _decode(ctx.mavlink.sent[0][0])
        assert msg.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        # The target is right of centre, so the yaw rate slot (param4) is set.
        assert msg.param4 != 0.0
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_pinned_designate_camera_filters_the_subscription() -> None:
    ctx = FakeCtx(live={"aim": False, "designate_camera": "uvc-1"})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        # A pinned id filters the detection subscription to that camera; auto /
        # empty subscribes to all (None), the default other tests cover.
        assert ctx.vision.camera_id == "uvc-1"
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_mcp_tools_registered_and_callable() -> None:
    ctx = FakeCtx(live={"aim": False}, with_tools=True)
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        assert set(ctx.tools.handlers) == {"status", "point_at", "recenter"}

        status = await ctx.tools.handlers["status"]({})
        assert status["connected"] is True
        assert status["rate_mode"] is False

        await _settle()
        ctx.mavlink.sent.clear()
        result = await ctx.tools.handlers["point_at"](
            {"pitch_deg": -20.0, "yaw_deg": 15.0}
        )
        assert result["ok"] is True
        await _settle()
        assert len(ctx.mavlink.sent) == 1
        msg = _decode(ctx.mavlink.sent[0][0])
        assert msg.param1 == -20.0 and msg.param2 == 15.0
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_tools_absent_when_ctx_has_no_tools() -> None:
    # No tools surface (mcp.expose not granted) -> registration is skipped, no
    # crash.
    ctx = FakeCtx(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        assert not hasattr(ctx, "tools")
    finally:
        await plugin.on_stop(ctx)
