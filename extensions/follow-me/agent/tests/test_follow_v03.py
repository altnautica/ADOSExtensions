"""Follow behaviour tests: local-frame setpoints, terrain-following height,
holds, setpoint rate and latency, and strict / smart lock loss with
synthetic batches.
"""

from __future__ import annotations

from typing import Any

import pytest

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

import follow_me
from follow_me.state import (
    FOLLOW_LOST_TOPIC,
    FOLLOW_REACQUIRED_TOPIC,
    HOLD_LOCK_LOST,
    LOCK_LOCKED,
    LOCK_LOST,
    PHASE_COASTING,
    PHASE_FOLLOWING,
    PHASE_LOST,
    PHASE_SEARCHING,
    FollowConfig,
)
from follow_fakes import (
    _Ctx,
    _distance_sensor,
    _level_pose,
    _arm_guided,
    _bbox_center_frame,
    _make_plugin,
    _Clock,
    _install_clock,
    _state_events,
    _telemetry_config,
)


# ---------------------------------------------------------------------------
# Local-frame setpoints, holds, rangefinder, rates and latency.
# ---------------------------------------------------------------------------

_EMB = [1.0, 0.0, 0.0, 0.0]
_OTHER_EMB = [0.0, 1.0, 0.0, 0.0]


def _follow_config(**over: Any) -> dict[str, Any]:
    cfg = _telemetry_config()
    cfg.update(over)
    return cfg


def _person(
    *,
    track_id: int | None,
    lock_state: str | None,
    bbox: BoundingBox | None = None,
    embedding: list[float] | None = None,
) -> Detection:
    attrs = (
        {"embedding": list(embedding), "embedding_model": "osnet_x0_25"}
        if embedding is not None
        else None
    )
    return Detection(
        bbox=bbox if bbox is not None else _bbox_center_frame(),
        class_label="person",
        confidence=0.9,
        track_id=track_id,
        lock_state=lock_state,
        attributes=attrs,
    )


def _feed(plugin: follow_me.FollowMePlugin, *dets: Detection, ts_ms: int = 0) -> None:
    plugin._on_batch(
        DetectionBatch(
            model_id="coco-person",
            camera_id="uvc-0",
            frame_id=1,
            ts_ms=ts_ms,
            frame_width=640,
            frame_height=480,
            detections=list(dets),
        )
    )


def _fresh(plugin: follow_me.FollowMePlugin) -> None:
    _level_pose(plugin)
    _arm_guided(plugin)


async def _follow_one(
    plugin: follow_me.FollowMePlugin, clock: _Clock, *, track_id: int = 7
) -> None:
    """One live sighting and one commanding tick."""
    _fresh(plugin)
    _feed(
        plugin,
        _person(track_id=track_id, lock_state=LOCK_LOCKED, embedding=_EMB),
        ts_ms=int(clock.now * 1000) - 80,
    )
    await plugin._tick()


def _events(ctx: _Ctx, topic: str) -> list[dict[str, Any]]:
    return [p for (t, p) in ctx.events.published if t == topic]


@pytest.mark.asyncio
async def test_follow_sends_a_goal_under_the_lease_and_publishes_state(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_follow_config())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)

    sp = ctx.flight.setpoints[-1]
    assert sp["kind"] == "follow"
    assert ctx.flight.acquires == [{"kind": "motion", "skill_id": "follow-me"}]
    assert ctx.flight.goal_leases[-1] == "L1"
    assert sp["subject_ne"] is not None and sp["candidates"]["angle_span_deg"] == 90.0
    assert sp["margin_m"] == 1.5
    state = _state_events(ctx)[-1]
    assert state["phase"] == PHASE_FOLLOWING
    assert state["commanding"] is True
    assert state["range_m"] is not None and state["range_sigma_m"] is not None
    assert {c["kind"] for c in state["range_cues"]} >= {"ground"}
    assert state["agl_source"] == "baro"
    assert state["target_ned"] is not None
    assert 80 <= state["pipeline_latency_ms"] <= 81
    assert state["setpoint_rate_hz"] == 0.5
    assert state["lease_id"] == "L1"


def test_loop_runs_at_20_hz() -> None:
    assert follow_me.loop._LOOP_HZ == 20.0


@pytest.mark.asyncio
async def test_setpoint_rate_counts_the_last_two_seconds(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    for _ in range(9):
        clock.advance(0.05)
        _fresh(plugin)
        await plugin._tick()
    assert len(ctx.flight.setpoints) == 10
    assert plugin._setpoint_rate_hz(clock.now) == 5.0
    clock.advance(3.0)
    assert plugin._setpoint_rate_hz(clock.now) == 0.0


@pytest.mark.asyncio
async def test_rangefinder_height_is_used_and_only_downward(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    # A forward-facing rangefinder never becomes the height source.
    plugin._on_distance_sensor(_distance_sensor(3.0, orientation=0))
    assert not plugin._rangefinder.report.has_reported
    plugin._on_distance_sensor(_distance_sensor(6.0))
    await _follow_one(plugin, clock)
    state = _state_events(ctx)[-1]
    assert state["agl_source"] == "rng"
    assert state["agl_m"] == pytest.approx(6.0)
    # Terrain-follow: 6 m above ground, want 4 m: the setpoint is 2 m lower.
    assert ctx.flight.setpoints[-1]["d"] == pytest.approx(-20.0 + 2.0)


@pytest.mark.asyncio
async def test_switching_off_releases_the_lease_and_sends_nothing_more(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_follow_config())
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    sent = len(ctx.flight.setpoints)

    await ctx.config_kv.set("active", False)
    for _ in range(8):
        clock.advance(0.05)
        _fresh(plugin)
        await plugin._tick()
    # The pilot brakes to its own hold once the lease is released.
    assert ctx.flight.setpoints[sent:] == []
    assert ctx.flight.releases == ["L1"]
    assert _state_events(ctx)[-1]["phase"] == "idle"


@pytest.mark.asyncio
async def test_strict_loss_gives_up_announces_and_holds(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config(reacquire_mode="strict"))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    sent = len(ctx.flight.setpoints)

    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()

    assert plugin._coast is None
    assert plugin._tracker.has_lock is False
    assert _events(ctx, FOLLOW_LOST_TOPIC) == [{"track_id": 7, "reason": "lock_lost"}]
    state = _state_events(ctx)[-1]
    assert state["phase"] == PHASE_LOST
    assert state["hold_reason"] == HOLD_LOCK_LOST
    hold = ctx.flight.setpoints[sent]
    assert hold["kind"] == "position"
    assert (hold["vn"], hold["ve"]) == (0.0, 0.0)
    assert (hold["n"], hold["e"], hold["d"]) == (0.0, 0.0, -20.0)

    # The engine keeps reporting the lost track: no repeat announcement.
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()
    assert len(_events(ctx, FOLLOW_LOST_TOPIC)) == 1


async def _lose_in_smart(
    plugin: follow_me.FollowMePlugin, clock: _Clock
) -> None:
    await _follow_one(plugin, clock)
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()


@pytest.mark.asyncio
async def test_smart_loss_coasts_at_reduced_feed_forward(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config(reacquire_mode="smart"))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _lose_in_smart(plugin, clock)

    assert plugin._coast is not None
    state = _state_events(ctx)[-1]
    assert state["phase"] in (PHASE_COASTING, PHASE_SEARCHING)
    assert state["commanding"] is True
    assert 0.0 < state["coast_remaining_s"] <= 4.0
    assert not _events(ctx, FOLLOW_LOST_TOPIC)
    # The lost track does not re-latch the gate during the coast.
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    assert plugin._tracker.has_lock is False


@pytest.mark.asyncio
async def test_smart_reacquires_a_matching_person_with_an_announced_designate(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config=_follow_config(reacquire_mode="smart"),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _lose_in_smart(plugin, clock)

    # The subject reappears, untracked, where the prediction says it is.
    clock.advance(0.5)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()

    assert len(ctx.vision.designate_calls) == 1
    assert ctx.vision.designate_calls[0]["class_label"] == "person"
    (event,) = _events(ctx, FOLLOW_REACQUIRED_TOPIC)
    assert event["track_id"] == 9
    assert event["similarity"] == pytest.approx(1.0)
    assert event["gap_s"] == pytest.approx(0.5)

    # The engine locks the designate; the follow resumes on track 9 with the
    # same subject filter.
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=9, lock_state=LOCK_LOCKED, embedding=_EMB))
    await plugin._tick()
    assert plugin._coast is None
    assert plugin._filter_track_id == 9
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING


@pytest.mark.asyncio
async def test_smart_ignores_a_person_who_looks_different(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config(reacquire_mode="smart"))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _lose_in_smart(plugin, clock)

    clock.advance(0.2)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_OTHER_EMB))
    await plugin._tick()
    assert ctx.vision.designate_calls == []


@pytest.mark.asyncio
async def test_smart_ignores_a_lookalike_far_from_the_prediction(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_follow_config(reacquire_mode="smart"))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _lose_in_smart(plugin, clock)

    clock.advance(0.2)
    _fresh(plugin)
    # Same appearance, but at the far left edge of the frame.
    far = BoundingBox(x=5.0, y=100.0, width=10.0, height=10.0)
    _feed(plugin, _person(track_id=None, lock_state=None, bbox=far, embedding=_EMB))
    await plugin._tick()
    assert ctx.vision.designate_calls == []


@pytest.mark.asyncio
async def test_smart_window_expiry_gives_up(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_follow_config(reacquire_mode="smart"))
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    await _lose_in_smart(plugin, clock)

    clock.advance(4.1)
    _fresh(plugin)
    await plugin._tick()
    assert plugin._coast is None
    assert _events(ctx, FOLLOW_LOST_TOPIC) == [
        {"track_id": 7, "reason": "reacquire_timeout"}
    ]
    assert _state_events(ctx)[-1]["phase"] == PHASE_LOST


def test_reacquire_settings_resolve_and_clamp() -> None:
    cfg = FollowConfig.resolve(
        {"reacquire_mode": "SMART", "reacquire_window_s": 99, "max_speed_mps": "3"},
        {},
    )
    assert cfg.reacquire_mode == "smart"
    assert cfg.reacquire_window_s == 15.0
    assert cfg.max_speed_mps == 3.0
    # An unknown mode resolves to the conservative strict.
    assert FollowConfig.resolve({"reacquire_mode": "bogus"}, {}).reacquire_mode == "strict"
    assert FollowConfig.resolve({}, {}).reacquire_mode == "smart"


def test_scan_payload_parses_only_body_frame_scans() -> None:
    ok = follow_me.parse_scan(
        {
            "t_ns": 0,
            "frame": "body_frd",
            "angle_start_rad": 0.0,
            "angle_increment_rad": 0.017453,
            "ranges_m": [1.0] * 360,
        }
    )
    assert ok is not None and len(ok.ranges_m) == 360
    assert follow_me.parse_scan({"frame": "map_ned", "ranges_m": [1.0]}) is None
    assert follow_me.parse_scan({"frame": "body_frd"}) is None


@pytest.mark.asyncio
async def test_on_start_wires_telemetry_embeddings_and_the_scan(monkeypatch: Any) -> None:
    ctx = _Ctx(config={"active": False})
    plugin = follow_me.FollowMePlugin()
    await plugin.on_start(ctx)
    try:
        for name in ("ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR", "HEARTBEAT"):
            assert name in ctx.mavlink.subscriptions
        # The router owns the stream rates; the plugin sends nothing at start.
        assert ctx.mavlink.sent == []
        assert ctx.vision.embeddings is True
        assert follow_me.SCAN_TOPIC in ctx.events.subscriptions
        plugin._on_scan(
            {
                "frame": "body_frd",
                "angle_start_rad": 0.0,
                "angle_increment_rad": 0.017453,
                "ranges_m": [2.0] * 360,
            }
        )
        assert plugin._fresh_scan(follow_me.fc_state.monotonic()) is not None
    finally:
        await plugin.on_stop(ctx)
