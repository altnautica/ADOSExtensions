"""Recovery edge cases: no silent re-lock while holding for the operator,
strict on an unsure lock, the speed of the prediction leg, stale or missing
pilot status, the latched hover height, a designate that never locks, and
the arm edge."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest
from follow_fakes import (
    _Ctx,
    _events,
    _heartbeat,
    _install_clock,
    _make_plugin,
    _state_events,
)
from follow_me.state import (
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
from test_follow_v03 import _EMB, _feed, _follow_one, _fresh, _person
from test_recovery import _cfg, _lose, _plugin, _status, _step


async def _unsure_for(plugin: Any, clock: Any, seconds: float) -> None:
    for _ in range(round(seconds / 0.1)):
        clock.advance(0.1)
        _fresh(plugin)
        _feed(plugin, _person(track_id=7, lock_state=LOCK_UNCERTAIN, embedding=_EMB))
        await plugin._tick()


async def _engine_reports(plugin: Any, clock: Any, track_id: int, lock_state: str) -> None:
    clock.advance(0.1)
    _fresh(plugin)
    _feed(plugin, _person(track_id=track_id, lock_state=lock_state, embedding=_EMB))
    await plugin._tick()


@pytest.mark.asyncio
async def test_strict_waiting_ignores_the_engine_reconfirming_the_old_track(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(reacquire_mode="strict", uncertain_mode="hold"))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _unsure_for(plugin, clock, 3.2)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    # The engine still has its own designate and calls it locked again: no
    # operator action, so the follow keeps waiting.
    await _engine_reports(plugin, clock, 7, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    assert plugin._tracker.has_lock is False
    # A real operator designate (a new engine track) is followed.
    await _engine_reports(plugin, clock, 21, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    assert plugin._filter_track_id == 21


@pytest.mark.asyncio
async def test_smart_waiting_ignores_the_engine_reconfirming_the_old_track(monkeypatch: Any) -> None:
    # The engine gives every designate a fresh track id.
    ctx = _Ctx(
        config=_cfg(search_budget_s=5.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _unsure_for(plugin, clock, 3.2)
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    await _step(plugin, clock, 5.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    announced = len(_events(ctx, FOLLOW_REACQUIRED_TOPIC))
    # The engine re-confirms the lost track, and the abandoned re-acquire
    # designate: neither is followed, and nothing is announced.
    await _engine_reports(plugin, clock, 7, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    await _engine_reports(plugin, clock, 9, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    assert len(_events(ctx, FOLLOW_REACQUIRED_TOPIC)) == announced


@pytest.mark.asyncio
async def test_a_designate_that_locks_after_the_budget_ran_out_is_not_followed(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=5.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    assert len(ctx.vision.designate_calls) == 1
    # The budget runs out before the engine confirms the designate.
    clock.advance(4.8)
    _fresh(plugin)
    await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    await _engine_reports(plugin, clock, 9, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR


@pytest.mark.asyncio
async def test_strict_holds_on_an_unsure_lock_whatever_the_uncertain_mode(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(reacquire_mode="strict"))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _engine_reports(plugin, clock, 7, LOCK_UNCERTAIN)
    st = _state_events(ctx)[-1]
    assert st["commanding"] is False and st["hold_reason"] == "lock-uncertain"
    assert ctx.flight.setpoints[-1]["kind"] == "position"


@pytest.mark.asyncio
async def test_the_prediction_leg_flies_at_the_search_speed(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=12.0, search_speed_mps=2.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    sp = ctx.flight.setpoints[-1]
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    assert sp["kind"] == "follow" and sp["max_speed_mps"] == 2.0
    assert abs(sp["vn"]) <= 2.0 and abs(sp["ve"]) <= 2.0


async def _at_second_vantage(plugin: Any, clock: Any) -> None:
    """Lose, reach the first vantage, sweep it out: the second request is out."""
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    await _step(plugin, clock, 0.35)
    plugin._on_goal_status(_status("arrived"))
    await _step(plugin, clock, 0.0)
    for _ in range(41):
        await _step(plugin, clock, 0.1)


@pytest.mark.asyncio
async def test_a_status_for_the_previous_vantage_does_not_end_the_reposition(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _at_second_vantage(plugin, clock)
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    assert ctx.flight.setpoints[-1]["vantage"]["sigma_k"] == 2.0
    # Still in flight for the first vantage: ignored.
    plugin._on_goal_status(_status("arrived"))
    await _step(plugin, clock, 0.05)
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    # The pilot's report on the new request is taken.
    await _step(plugin, clock, 0.3)
    plugin._on_goal_status(_status("arrived"))
    await _step(plugin, clock, 0.05)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_SEARCHING and st["search_step"] == 1


@pytest.mark.asyncio
async def test_without_pilot_status_the_search_starts_from_where_it_is(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    for _ in range(10):
        await _step(plugin, clock, 0.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    for _ in range(11):
        await _step(plugin, clock, 0.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_SEARCHING and st["vantage_state"] is None


@pytest.mark.asyncio
async def test_the_hover_height_is_latched_too(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    latched = ctx.flight.setpoints[-1]["d"]
    clock.advance(0.05)
    _fresh(plugin)
    plugin._pose.down_m -= 1.5
    await plugin._tick()
    assert ctx.flight.setpoints[-1]["d"] == latched


@pytest.mark.asyncio
async def test_waiting_publishes_no_stale_vantage(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    await _step(plugin, clock, 0.35)
    plugin._on_goal_status(
        _status("enroute", vantage_ne=[1.0, 2.0], occluder_ne=[3.0, 0.0], occluder_width_m=0.6)
    )
    await _step(plugin, clock, 0.05)
    assert _state_events(ctx)[-1]["vantage_ne"] == [1.0, 2.0]
    await _step(plugin, clock, 3.0)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR
    assert (st["vantage_ne"], st["vantage_state"], st["occluder"]) == (None, None, None)


@pytest.mark.asyncio
async def test_a_designate_the_engine_loses_lets_matching_continue(monkeypatch: Any) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=25.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    assert len(ctx.vision.designate_calls) == 1
    # The engine loses the designate: it is abandoned and the same batch is
    # matched again at once.
    await _engine_reports(plugin, clock, 9, LOCK_LOST)
    assert len(ctx.vision.designate_calls) == 2
    assert 9 in plugin._recovery.excluded_tracks
    # Nothing has locked, so nothing is announced.
    assert _events(ctx, FOLLOW_REACQUIRED_TOPIC) == []


@pytest.mark.asyncio
async def test_a_designate_that_never_locks_lets_matching_continue(monkeypatch: Any) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=25.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    await _step(plugin, clock, 2.5)
    assert plugin._recovery.designated is True
    await _step(plugin, clock, 0.6)
    assert plugin._recovery.designated is False


@pytest.mark.asyncio
async def test_an_unsure_designate_is_given_time_and_not_announced_without_a_lock(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=25.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    # The engine is unsure of its designate for a while: not abandoned.
    for _ in range(25):
        await _engine_reports(plugin, clock, 9, LOCK_UNCERTAIN)
    assert plugin._recovery.designated is True
    assert len(ctx.vision.designate_calls) == 1
    # Still unsure past the limit: abandoned and matched again. No lock, so
    # no announcement.
    for _ in range(8):
        await _engine_reports(plugin, clock, 9, LOCK_UNCERTAIN)
    assert len(ctx.vision.designate_calls) == 2
    assert _events(ctx, FOLLOW_REACQUIRED_TOPIC) == []


@pytest.mark.asyncio
async def test_a_designate_whose_request_failed_still_relocks_announced(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)

    async def _timeout(*_a: Any, **_k: Any) -> Any:
        raise TimeoutError("request timed out")

    ctx.vision.designate_track = _timeout  # type: ignore[method-assign]
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    # The engine applied it anyway (fresh id 9) and locks it: that is this
    # recovery's re-lock, announced, on the same subject filter.
    filt = plugin._filter
    await _engine_reports(plugin, clock, 9, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    (ev,) = _events(ctx, FOLLOW_REACQUIRED_TOPIC)
    assert ev["track_id"] == 9
    assert plugin._filter is filt and plugin._filter.initialized


@pytest.mark.asyncio
async def test_the_person_actually_relocked_is_announced(monkeypatch: Any) -> None:
    ids = iter([9, 11])
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)

    async def _designate(*_a: Any, **_k: Any) -> Any:
        return {"designated": True, "track_id": next(ids), "camera_id": "uvc-0"}

    ctx.vision.designate_track = _designate  # type: ignore[method-assign]
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    # The engine loses designate 9 at once (and the same batch designates 11).
    await _engine_reports(plugin, clock, 9, LOCK_LOST)
    await _engine_reports(plugin, clock, 11, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    (ev,) = _events(ctx, FOLLOW_REACQUIRED_TOPIC)
    assert ev["track_id"] == 11


@pytest.mark.asyncio
async def test_a_resume_still_never_adopts_an_abandoned_designate(monkeypatch: Any) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=5.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    assert len(ctx.vision.designate_calls) == 1
    # Later designates are refused, so only that first one can be in play.
    ctx.vision._designate_result = {"designated": False}
    await _step(plugin, clock, 4.8)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    await ctx.config_kv.set("operator_resume_seq", 1)
    await _step(plugin, clock, 1.1)
    await _step(plugin, clock, 5.2)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    announced = len(_events(ctx, FOLLOW_REACQUIRED_TOPIC))
    # The engine's abandoned designate locks after the second wait began.
    await _engine_reports(plugin, clock, 9, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    assert len(_events(ctx, FOLLOW_REACQUIRED_TOPIC)) == announced
    # The lost subject's own track stays excluded too.
    await _engine_reports(plugin, clock, 7, LOCK_LOCKED)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR


@pytest.mark.asyncio
async def test_a_resume_does_not_match_a_batch_seen_before_it(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0), designate_result={"designated": False})
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(0.3)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    assert len(ctx.vision.designate_calls) == 1
    await _step(plugin, clock, 4.8)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    await ctx.config_kv.set("operator_resume_seq", 1)
    await _step(plugin, clock, 1.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    # The sighting from before the wait is seconds old: never matched again.
    assert len(ctx.vision.designate_calls) == 1


@pytest.mark.asyncio
async def test_gap_counts_from_the_loss_after_a_resume(monkeypatch: Any) -> None:
    ctx = _Ctx(
        config=_cfg(search_budget_s=5.0),
        designate_result={"designated": True, "track_id": 9, "camera_id": "uvc-0"},
    )
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    await ctx.config_kv.set("operator_resume_seq", 1)
    await _step(plugin, clock, 1.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    clock.advance(0.1)
    _fresh(plugin)
    _feed(plugin, _person(track_id=None, lock_state=None, embedding=_EMB))
    await plugin._tick()
    await _engine_reports(plugin, clock, 9, LOCK_LOCKED)
    (ev,) = _events(ctx, FOLLOW_REACQUIRED_TOPIC)
    assert ev["gap_s"] > 6.0


@pytest.mark.asyncio
async def test_an_arm_seen_first_after_a_restart_does_not_reset_the_search(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg(predictive_search=False))
    plugin = _make_plugin(ctx)
    _install_clock(monkeypatch)
    # The plugin starts while the vehicle is already flying.
    plugin._on_heartbeat(_heartbeat(base_mode=0x81, custom_mode=4, autopilot=3))
    await asyncio.sleep(0)
    assert await ctx.config_kv.get("predictive_search") is False
    plugin._on_heartbeat(_heartbeat(base_mode=0x01, custom_mode=4, autopilot=3))
    plugin._on_heartbeat(_heartbeat(base_mode=0x81, custom_mode=4, autopilot=3))
    await asyncio.sleep(0)
    assert await ctx.config_kv.get("predictive_search") is True
