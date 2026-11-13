"""Recovery: predicting, repositioning, searching, then holding for the
operator; re-lock on the same person only; the uncertain-lock setting, the
predictive-search toggle, and the operator's resume and re-designate."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest
from follow_fakes import (
    _Clock,
    _Ctx,
    _events,
    _heartbeat,
    _install_clock,
    _make_plugin,
    _state_events,
    _telemetry_config,
)
from follow_me import FollowMePlugin
from follow_me.state import (
    FOLLOW_LOST_TOPIC,
    FOLLOW_REACQUIRED_TOPIC,
    LOCK_LOCKED,
    LOCK_LOST,
    LOCK_UNCERTAIN,
    PHASE_AWAIT_OPERATOR,
    PHASE_FOLLOWING,
    PHASE_PREDICTING,
    PHASE_REPOSITIONING,
    PHASE_SEARCHING,
)
from test_follow_v03 import _EMB, _OTHER_EMB, _feed, _follow_one, _fresh, _person


def _cfg(**over: Any) -> dict[str, Any]:
    c = _telemetry_config()
    c.update({"reacquire_mode": "smart", "predictive_search": True, "search_budget_s": 10.0})
    c.update(over)
    return c


def _plugin(ctx: _Ctx) -> FollowMePlugin:
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    return plugin


async def _lose(plugin: FollowMePlugin, clock: _Clock) -> None:
    await _follow_one(plugin, clock)
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()


async def _step(plugin: FollowMePlugin, clock: _Clock, dt: float) -> None:
    clock.advance(dt)
    _fresh(plugin)
    await plugin._tick()


def _status(state: str, **over: Any) -> dict[str, Any]:
    p: dict[str, Any] = {
        "lease_id": 7,
        "plugin_id": "com.altnautica.follow-me",
        "state": state,
        "vantage_ne": None,
        "coverage": 0.0,
        "occluder_ne": None,
        "occluder_width_m": None,
    }
    p.update(over)
    return p


@pytest.mark.asyncio
async def test_loss_predicts_then_asks_the_pilot_for_a_vantage(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_PREDICTING and st["commanding"] is True
    assert st["predicted_path_ne"] and st["search_area"] is not None
    assert 9.0 < st["reacquire_remaining_s"] <= 10.0 and st["search_budget_s"] == 10.0
    assert ctx.flight.setpoints[-1]["kind"] == "follow"
    await _step(plugin, clock, 2.1)
    sp = ctx.flight.setpoints[-1]
    assert sp["kind"] == "vantage"
    assert sp["vantage"]["max_radius_m"] == 12.0 and sp["max_speed_mps"] <= 2.0
    assert sp["vantage"]["sigma_k"] == 1.0 and sp["vantage"]["standoff_m"] == 6.0
    assert sp["vantage"]["anchor_ne"] == [0.0, 0.0]
    assert len(sp["vantage"]["cov_ne"]) == 3
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_REPOSITIONING and st["predicted_path_ne"] and st["search_area"]


@pytest.mark.asyncio
async def test_arrival_starts_the_sweep_and_widens(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    # The pilot's first report on the new request.
    await _step(plugin, clock, 0.35)
    plugin._on_goal_status(
        _status("arrived", vantage_ne=[1.0, 2.0], coverage=0.8, occluder_ne=[3.0, 0.0], occluder_width_m=0.6)
    )
    await _step(plugin, clock, 0.0)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_SEARCHING and st["search_step"] == 0
    assert st["occluder"] == {"ne": [3.0, 0.0], "kind": "narrow"}
    assert st["vantage_ne"] == [1.0, 2.0] and st["vantage_state"] == "arrived"
    yaws = []
    for _ in range(20):
        await _step(plugin, clock, 0.1)
        yaws.append(ctx.flight.setpoints[-1]["yaw"])
    assert max(yaws) - min(yaws) > 0.5  # it sweeps
    # The step's dwell runs out: a wider vantage request, back to repositioning.
    for _ in range(25):
        await _step(plugin, clock, 0.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_REPOSITIONING
    assert ctx.flight.setpoints[-1]["vantage"]["sigma_k"] == 2.0


@pytest.mark.asyncio
async def test_a_wide_occluder_is_reported_as_wide(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    await _step(plugin, clock, 0.35)
    plugin._on_goal_status(_status("enroute", occluder_ne=[3.0, 0.0], occluder_width_m=4.0))
    # Another lease's status is not ours.
    plugin._on_goal_status(_status("arrived", lease_id=99))
    await _step(plugin, clock, 0.05)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_REPOSITIONING and st["occluder"]["kind"] == "wide"


@pytest.mark.asyncio
async def test_budget_runs_out_into_a_latched_hover_and_the_prompt(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "search-exhausted"
    (ev,) = _events(ctx, FOLLOW_LOST_TOPIC)
    assert ev["reason"] == "search_exhausted" and ev["awaiting_operator"] is True
    assert ev["track_id"] == 7 and ev["hold_ne"] == st["hold_ne"]
    hold = ctx.flight.setpoints[-1]
    assert hold["kind"] == "position" and [hold["n"], hold["e"]] == st["hold_ne"]
    # The pose estimate drifts; the hold point does not.
    plugin._pose.north_m += 0.7
    clock.advance(0.05)
    _fresh(plugin)
    plugin._pose.north_m += 0.7
    await plugin._tick()
    assert [ctx.flight.setpoints[-1]["n"], ctx.flight.setpoints[-1]["e"]] == st["hold_ne"]
    assert len(_events(ctx, FOLLOW_LOST_TOPIC)) == 1


@pytest.mark.asyncio
async def test_only_the_same_person_is_relocked_and_it_is_announced(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(), designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"})
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_OTHER_EMB))
    await plugin._tick()
    assert ctx.vision.designate_calls == []
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    assert len(ctx.vision.designate_calls) == 1
    assert _events(ctx, FOLLOW_REACQUIRED_TOPIC) == []
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=9, lock_state=LOCK_LOCKED, embedding=_EMB))
    await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    assert plugin._recovery is None and plugin._filter_track_id == 9
    (ev,) = _events(ctx, FOLLOW_REACQUIRED_TOPIC)
    assert ev["track_id"] == 9 and ev["phase"] == PHASE_PREDICTING and "mahalanobis" in ev


@pytest.mark.asyncio
async def test_strict_goes_straight_to_awaiting_the_operator(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(reacquire_mode="strict"))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "awaiting-operator"
    assert _events(ctx, FOLLOW_LOST_TOPIC)[0]["reason"] == "lock_lost"


@pytest.mark.asyncio
async def test_predictive_search_off_predicts_briefly_then_waits(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(predictive_search=False))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    await _step(plugin, clock, 2.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "predictive-search-off"
    assert _events(ctx, FOLLOW_LOST_TOPIC)[0]["reason"] == "predictive_search_off"
    assert all(sp["kind"] != "vantage" for sp in ctx.flight.setpoints)


@pytest.mark.asyncio
async def test_no_vantage_searches_from_where_it_is(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    await _step(plugin, clock, 0.35)
    plugin._on_goal_status(_status("no-vantage"))
    await _step(plugin, clock, 0.0)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_SEARCHING and st["vantage_state"] == "no-vantage"


@pytest.mark.asyncio
async def test_uncertain_keeps_following_at_reduced_feed_forward_then_recovers(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg(uncertain_mode="follow"))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_UNCERTAIN, embedding=_EMB))
    await plugin._tick()
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_FOLLOWING and st["lock_state"] == "uncertain" and st["commanding"] is True
    assert ctx.flight.setpoints[-1]["kind"] == "follow"
    for _ in range(31):
        clock.advance(0.1)
        _fresh(plugin)
        _feed(plugin, _person(track_id=7, lock_state=LOCK_UNCERTAIN, embedding=_EMB))
        await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING


@pytest.mark.asyncio
async def test_uncertain_hold_never_lasts_past_the_limit(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(uncertain_mode="hold"))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_UNCERTAIN, embedding=_EMB))
    await plugin._tick()
    st = _state_events(ctx)[-1]
    assert st["hold_reason"] == "lock-uncertain" and st["commanding"] is False
    for _ in range(31):
        clock.advance(0.1)
        _fresh(plugin)
        _feed(plugin, _person(track_id=7, lock_state=LOCK_UNCERTAIN, embedding=_EMB))
        await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING


@pytest.mark.asyncio
async def test_arming_turns_predictive_search_back_on(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(predictive_search=False))
    plugin = _make_plugin(ctx)
    _install_clock(monkeypatch)
    plugin._on_heartbeat(_heartbeat(base_mode=0x01, custom_mode=4, autopilot=3))
    await asyncio.sleep(0)
    assert await ctx.config_kv.get("predictive_search") is False
    plugin._on_heartbeat(_heartbeat(base_mode=0x81, custom_mode=4, autopilot=3))
    await asyncio.sleep(0)
    assert await ctx.config_kv.get("predictive_search") is True


@pytest.mark.asyncio
async def test_resume_restarts_the_search_with_a_fresh_budget(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    await ctx.config_kv.set("operator_resume_seq", 1)
    await _step(plugin, clock, 1.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_PREDICTING and st["reacquire_remaining_s"] > 4.0
    assert st["operator_resume_seq"] == 1


@pytest.mark.asyncio
async def test_a_new_designate_while_waiting_follows_the_new_subject(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=21, lock_state=LOCK_LOCKED, embedding=_OTHER_EMB))
    await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    assert plugin._filter_track_id == 21 and plugin._recovery is None


@pytest.mark.asyncio
async def test_waiting_never_relocks_on_its_own(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    clock.advance(0.3)
    _fresh(plugin)
    # The engine still reports the lost track, and an untracked look-alike.
    _feed(
        plugin,
        _person(track_id=7, lock_state=LOCK_LOST),
        _person(track_id=None, lock_state=None, embedding=_EMB),
    )
    await plugin._tick()
    assert ctx.vision.designate_calls == []
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
