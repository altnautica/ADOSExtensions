"""The pilot motion lease, revocation, orbit goals and stick nudges."""

from __future__ import annotations

import math
from typing import Any

import pytest
from ados.sdk.vision import BoundingBox, Detection, DetectionBatch
from follow_fakes import (
    _arm_guided,
    _bbox_center_frame,
    _Ctx,
    _install_clock,
    _level_pose,
    _make_plugin,
    _state_events,
    _telemetry_config,
)

from follow_me import FollowMePlugin
from follow_me.motion import (
    LEASE_REVOKED_TOPIC,
    STICKS_TOPIC,
    Nudges,
    nudge_orbit_rate,
)
from follow_me.state import (
    FOLLOW_STOPPED_TOPIC,
    HOLD_LEASE_BUSY,
    LOCK_LOCKED,
    FollowConfig,
)


def _cfg(**over: Any) -> dict[str, Any]:
    cfg = _telemetry_config()
    cfg.update(over)
    return cfg


def _feed(plugin: FollowMePlugin, clock: Any, bbox: BoundingBox | None = None) -> None:
    plugin._on_batch(
        DetectionBatch(
            model_id="coco-person",
            camera_id="uvc-0",
            frame_id=1,
            ts_ms=int(clock.now * 1000),
            frame_width=640,
            frame_height=480,
            detections=[
                Detection(
                    bbox=bbox or _bbox_center_frame(),
                    class_label="person",
                    confidence=0.9,
                    track_id=7,
                    lock_state=LOCK_LOCKED,
                )
            ],
        )
    )


async def _tick(plugin: FollowMePlugin, clock: Any, dt: float = 0.05) -> None:
    clock.advance(dt)
    _level_pose(plugin)
    _arm_guided(plugin)
    _feed(plugin, clock)
    await plugin._tick()


@pytest.mark.asyncio
async def test_lease_acquire_goals_at_20_hz_then_revoke_switches_off(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await plugin.on_start(ctx)
    plugin._loop_task.cancel()
    assert LEASE_REVOKED_TOPIC in ctx.events.subscriptions
    assert STICKS_TOPIC in ctx.events.subscriptions
    assert ctx.flight.sticks_subscribed

    for _ in range(20):
        await _tick(plugin, clock)
    # One acquire, then a goal every tick under that lease.
    assert len(ctx.flight.acquires) == 1
    assert len(ctx.flight.setpoints) == 20
    assert set(ctx.flight.goal_leases) == {"L1"}
    assert plugin._setpoint_rate_hz(clock.now) == 10.0  # 20 goals / 2 s window

    ctx.events.subscriptions[LEASE_REVOKED_TOPIC]({"lease_id": "L1", "reason": "takeover"})
    sent = len(ctx.flight.setpoints)
    await _tick(plugin, clock)
    assert ctx.config_kv._values["active"] is False
    stopped = [p for (t, p) in ctx.events.published if t == FOLLOW_STOPPED_TOPIC]
    assert stopped == [{"reason": "takeover"}]
    assert len(ctx.flight.setpoints) == sent, "nothing is sent after a revoke"
    assert _state_events(ctx)[-1]["active"] is False
    assert _state_events(ctx)[-1]["lease_id"] is None


@pytest.mark.asyncio
async def test_a_revoke_for_another_lease_is_ignored(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _tick(plugin, clock)
    plugin._on_lease_revoked({"lease_id": "other", "reason": "x"})
    await _tick(plugin, clock)
    assert plugin._lease_id == "L1"
    assert not [p for (t, p) in ctx.events.published if t == FOLLOW_STOPPED_TOPIC]


@pytest.mark.asyncio
async def test_a_busy_lease_holds_and_retries_once_a_second(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    ctx.flight.acquire_reply = {"ok": False, "err": "busy", "holder": {"plugin_id": "x"}}
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    for _ in range(10):
        await _tick(plugin, clock)
    assert _state_events(ctx)[-1]["hold_reason"] == HOLD_LEASE_BUSY
    assert not ctx.flight.setpoints
    assert len(ctx.flight.acquires) == 1
    ctx.flight.acquire_reply = {"ok": True, "lease_id": "L2"}
    for _ in range(20):
        await _tick(plugin, clock)
    assert len(ctx.flight.acquires) == 2
    assert ctx.flight.goal_leases[-1] == "L2"


@pytest.mark.asyncio
async def test_orbit_mode_sends_orbit_goals_on_the_ring(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(follow_mode="orbit", orbit_radius_m=8.0))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _tick(plugin, clock)
    g = ctx.flight.setpoints[-1]
    assert g["kind"] == "orbit"
    c = g["candidates"]
    assert c["radius_m"] == 8.0 and c["angle_span_deg"] == 60.0
    sn, se = g["subject_ne"]
    assert math.hypot(g["n"] - sn, g["e"] - se) == pytest.approx(8.0)
    st = _state_events(ctx)[-1]
    assert st["follow_mode"] == "orbit" and st["orbit_phase_deg"] is not None


def _nudge(cfg: FollowConfig, axes: list[float], seconds: float) -> tuple[Nudges, FollowConfig]:
    n = Nudges()
    t = 0.0
    n.on_sticks(cfg, axes, t)
    while t < seconds - 1e-9:
        t += 0.05
        n.on_sticks(n.apply_to(cfg), axes, t)
    return n, n.apply_to(cfg)


def test_follow_nudges_change_the_right_parameter_at_the_right_rate() -> None:
    base = FollowConfig(follow_distance_m=10.0, follow_height_m=4.0, follow_angle_deg=180.0)
    _, c = _nudge(base, [1.0, 0.0, 0.0, 0.0], 1.0)
    assert c.follow_angle_deg == pytest.approx(240.0)
    assert c.follow_distance_m == 10.0
    _, c = _nudge(base, [0.0, 1.0, 0.0, 0.0], 1.0)
    assert c.follow_distance_m == pytest.approx(8.0)
    _, c = _nudge(base, [0.0, 0.0, 1.0, 0.0], 1.0)
    assert c.follow_height_m == pytest.approx(5.0)
    # Yaw never nudges anything.
    n, c = _nudge(base, [0.0, 0.0, 0.0, 1.0], 1.0)
    assert c == base and not n.nudged


def test_orbit_nudges_and_clamps() -> None:
    base = FollowConfig(follow_mode="orbit", orbit_radius_m=8.0, orbit_rate_dps=10.0)
    _, c = _nudge(base, [1.0, 0.0, 0.0, 0.0], 1.0)
    assert c.orbit_rate_dps == pytest.approx(20.0) and c.orbit_direction == "cw"
    _, c = _nudge(base, [0.0, -1.0, 0.0, 0.0], 1.0)
    assert c.orbit_radius_m == pytest.approx(10.0)
    _, c = _nudge(base, [1.0, 0.0, 0.0, 0.0], 10.0)
    assert c.orbit_rate_dps == 30.0
    _, c = _nudge(base, [0.0, 1.0, 0.0, 0.0], 10.0)
    assert c.orbit_radius_m == 3.0
    _, c = _nudge(base, [0.0, 0.0, -1.0, 0.0], 30.0)
    assert c.follow_height_m == 0.0


def test_orbit_rate_through_the_minimum_flips_direction() -> None:
    assert nudge_orbit_rate(2.2, "cw", -0.5) == (2.0, "ccw")
    assert nudge_orbit_rate(2.0, "ccw", 0.5) == (2.0, "cw")
    assert nudge_orbit_rate(10.0, "ccw", -0.5) == (10.5, "ccw")
    base = FollowConfig(follow_mode="orbit", orbit_rate_dps=10.0)
    _, c = _nudge(base, [-1.0, 0.0, 0.0, 0.0], 0.75)
    assert c.orbit_direction == "cw" and c.orbit_rate_dps == pytest.approx(2.5)
    _, c = _nudge(base, [-1.0, 0.0, 0.0, 0.0], 1.0)
    assert c.orbit_direction == "ccw"


@pytest.mark.asyncio
async def test_sticks_nudge_live_and_write_back_at_most_once_a_second(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg(follow_mode="orbit", orbit_rate_dps=10.0))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    writes: list[tuple[str, Any]] = []
    real_set = ctx.config_kv.set

    async def spy(key: str, value: Any, scope: str = "drone") -> dict:
        writes.append((key, value))
        return await real_set(key, value, scope)

    monkeypatch.setattr(ctx.config_kv, "set", spy)
    await _tick(plugin, clock)
    for _ in range(40):
        plugin._on_sticks({"lease_id": "L1", "axes": [1.0, 0.0, 0.0, 0.0]})
        await _tick(plugin, clock)
    rate_writes = [v for (k, v) in writes if k == "orbit_rate_dps"]
    # 2 s of sticks: at most 3 write-backs (≤ 1 Hz, first one immediate).
    assert 1 <= len(rate_writes) <= 3
    assert plugin._cfg.orbit_rate_dps > 15.0
    assert _state_events(ctx)[-1]["nudged"] is True
    # Sticks for a lease we do not hold are ignored.
    before = plugin._cfg.orbit_rate_dps
    plugin._on_sticks({"lease_id": "nope", "axes": [1.0, 0.0, 0.0, 0.0]})
    clock.advance(0.05)
    plugin._on_sticks({"lease_id": "nope", "axes": [1.0, 0.0, 0.0, 0.0]})
    assert plugin._nudges.apply_to(plugin._cfg).orbit_rate_dps == before


def test_resolve_clamps_and_normalises_the_new_keys() -> None:
    c = FollowConfig.resolve(
        {
            "follow_mode": "ORBIT",
            "follow_angle_deg": 370,
            "orbit_radius_m": 99,
            "orbit_rate_dps": 0.1,
            "orbit_direction": "sideways",
            "follow_distance_m": 1,
            "follow_height_m": 50,
            "obstacle_margin_m": 0.1,
        },
        {},
    )
    assert c.follow_mode == "orbit"
    assert c.follow_angle_deg == pytest.approx(10.0)
    assert (c.orbit_radius_m, c.orbit_rate_dps, c.orbit_direction) == (30.0, 2.0, "cw")
    assert (c.follow_distance_m, c.follow_height_m, c.obstacle_margin_m) == (3.0, 20.0, 0.8)
    assert FollowConfig.resolve({"follow_mode": "x"}, {}).follow_mode == "follow"
