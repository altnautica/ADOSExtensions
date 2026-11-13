"""Recovery edge cases: a gate bounded by how far the subject can have
gone, the whole budget searched, Resume after the prediction expired, the
stop and turn-back hypotheses, a vantage that is never reached, operator
control mid-search, an EKF position reset, and a subject faster than the
limit."""

from __future__ import annotations

import math
from typing import Any

import pytest
from follow_fakes import (
    _arm_guided,
    _attitude,
    _Ctx,
    _events,
    _install_clock,
    _local_position,
    _state_events,
)
from follow_me.predict import predict_target, search_ellipse
from follow_me.search import SearchBudget, search_hypotheses, search_steps
from follow_me.state import (
    FOLLOW_REACQUIRED_TOPIC,
    FOLLOW_STOPPED_TOPIC,
    LOCK_LOCKED,
    LOCK_LOST,
    PHASE_AWAIT_OPERATOR,
    PHASE_FOLLOWING,
    PHASE_PREDICTING,
    PHASE_REPOSITIONING,
    PHASE_SEARCHING,
    RECOVERY_PHASES,
)
from follow_me.target_filter import TargetFilter, position_sigma
from test_follow_v03 import _EMB, _OTHER_EMB, _feed, _follow_one, _fresh, _person
from test_recovery import _cfg, _lose, _plugin, _status, _step

_R = ((0.25, 0.0), (0.0, 0.25))


def _walking(speed: float = 1.4) -> TargetFilter:
    f = TargetFilter()
    f.max_speed_mps = 3.0
    for k in range(10):
        f.update(k * 0.33, speed * k * 0.33, 0.0, _R)
    return f


# -- a gate bounded by the reachable distance --------------------------------


def test_the_position_gate_never_admits_a_point_beyond_the_reachable_distance() -> None:
    f = _walking()
    last = f.last_estimate()
    assert last is not None
    sigma_last = f.last_sigma_m
    t = f.last_update_s + 10.0
    reach = 3.0 * 10.0 + 3.0 * sigma_last
    # Due east of the last estimate, a little past anywhere the subject could
    # have got to in 10 s.
    d = f.mahalanobis(t, last[0], last[1] + reach + 1.0, _R)
    assert d is not None and d > 3.0
    # A point it could have reached is still admitted.
    near = f.mahalanobis(t, last[0] + 1.4 * 10.0, last[1], _R)
    assert near is not None and near <= 3.0


def test_the_search_ellipse_is_bounded_by_the_reachable_distance() -> None:
    f = _walking()
    t = f.last_update_s + 10.0
    p = predict_target(f, t)
    assert p is not None
    e = search_ellipse(p, k=3.0)
    assert e.semi_major_m <= 3.0 * 10.0 + 3.0 * f.last_sigma_m + 1e-6
    # Shortly after the last sighting the filter's own spread is smaller and
    # is left alone.
    early = predict_target(f, f.last_update_s + 0.2)
    assert early is not None
    pred = f.predicted(f.last_update_s + 0.2)
    assert pred is not None
    assert early.cov_ne[0] == pytest.approx(pred[1][0][0])


def test_a_prediction_past_its_horizon_is_the_last_estimate_with_a_bounded_spread() -> None:
    f = _walking()
    t = f.last_update_s + 120.0
    assert f.predicted(t) is None
    held = f.search_predicted(t)
    assert held is not None
    x, p = held
    last = f.last_estimate()
    assert last is not None
    assert (x[0], x[1], x[2], x[3]) == (last[0], last[1], 0.0, 0.0)
    assert 3.0 * position_sigma(p) == pytest.approx(3.0 * 120.0 + 3.0 * f.last_sigma_m)


@pytest.mark.asyncio
async def test_a_60_s_budget_is_searched_in_full(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=60.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    for _ in range(118):
        await _step(plugin, clock, 0.5)
    assert _state_events(ctx)[-1]["phase"] in RECOVERY_PHASES
    await _step(plugin, clock, 1.2)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "search-exhausted"


@pytest.mark.asyncio
async def test_resume_after_the_prediction_expired_searches_around_the_last_estimate(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 25.1)
    await _step(plugin, clock, 60.0)
    assert _state_events(ctx)[-1]["phase"] == PHASE_AWAIT_OPERATOR
    assert plugin._filter.predicted(clock.now) is None
    await ctx.config_kv.set("operator_resume_seq", 1)
    await _step(plugin, clock, 1.1)
    st = _state_events(ctx)[-1]
    # Nothing left to extrapolate, so it asks for a vantage at once.
    assert st["phase"] == PHASE_REPOSITIONING
    assert st["reacquire_remaining_s"] > 24.0
    last = plugin._filter.last_estimate()
    assert last is not None
    assert st["search_area"]["center_ne"] == [round(last[0], 2), round(last[1], 2)]
    assert ctx.flight.setpoints[-1]["kind"] == "vantage"


# -- the stop and turn-back hypotheses, the all-round sweep -------------------


def test_the_hypotheses_are_the_last_spot_and_three_metres_back() -> None:
    stop, back = search_hypotheses((10.0, 2.0, 1.4, 0.0))
    assert stop == (10.0, 2.0)
    assert back == pytest.approx((7.0, 2.0))
    # A subject that was standing still has no direction to turn back on.
    assert search_hypotheses((10.0, 2.0, 0.05, 0.0)) == [(10.0, 2.0)]


@pytest.mark.asyncio
async def test_vantage_requests_carry_the_stop_and_reverse_hypotheses(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    sp = ctx.flight.setpoints[-1]
    last = plugin._filter.last_estimate()
    assert last is not None
    assert sp["vantage"]["hypotheses_ne"][0] == [last[0], last[1]]


def test_the_last_step_sweeps_all_round() -> None:
    steps = search_steps(SearchBudget(25.0, 12.0, 2.0))
    assert steps[-1].yaw_offsets_deg[0] == -180.0 and steps[-1].yaw_offsets_deg[-1] == 180.0
    assert all(s.yaw_offsets_deg[-1] < 180.0 for s in steps[:-1])


@pytest.mark.asyncio
async def test_a_vantage_never_reached_still_gets_swept(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    # The pilot keeps saying it is on its way to a point 5 m off; the vehicle
    # never gets there (gusts, a fence).
    for _ in range(24):
        await _step(plugin, clock, 0.2)
        plugin._on_goal_status(_status("enroute", vantage_ne=[5.0, 0.0]))
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    for _ in range(6):
        await _step(plugin, clock, 0.2)
        plugin._on_goal_status(_status("enroute", vantage_ne=[5.0, 0.0]))
    assert _state_events(ctx)[-1]["phase"] == PHASE_SEARCHING


@pytest.mark.asyncio
async def test_hovering_near_the_vantage_starts_the_sweep(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=25.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    for _ in range(8):
        await _step(plugin, clock, 0.2)
        plugin._on_goal_status(_status("enroute", vantage_ne=[1.0, 0.0]))
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    for _ in range(6):
        await _step(plugin, clock, 0.2)
        plugin._on_goal_status(_status("enroute", vantage_ne=[1.0, 0.0]))
    assert _state_events(ctx)[-1]["phase"] == PHASE_SEARCHING


# -- operator control and transitions mid-search ------------------------------


@pytest.mark.asyncio
async def test_an_operator_designate_mid_search_follows_the_new_subject(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    clock.advance(0.1)
    _fresh(plugin)
    _feed(plugin, _person(track_id=21, lock_state=LOCK_LOCKED, embedding=_OTHER_EMB))
    await plugin._tick()
    assert _state_events(ctx)[-1]["phase"] == PHASE_FOLLOWING
    assert plugin._filter_track_id == 21 and plugin._recovery is None
    assert _events(ctx, FOLLOW_REACQUIRED_TOPIC) == []


@pytest.mark.asyncio
async def test_a_revoke_mid_search_ends_the_recovery_and_stops(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    plugin._on_lease_revoked({"lease_id": 7, "reason": "takeover"})
    await _step(plugin, clock, 0.05)
    assert plugin._recovery is None
    assert _events(ctx, FOLLOW_STOPPED_TOPIC) == [{"reason": "takeover"}]
    assert await ctx.config_kv.get("active") is False
    assert _state_events(ctx)[-1]["phase"] == "idle"


@pytest.mark.asyncio
async def test_the_vantage_goal_keeps_the_follow_height(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 2.1)
    sp = ctx.flight.setpoints[-1]
    # 20 m above the take-off point by the barometer, follow height 4 m.
    assert sp["kind"] == "vantage" and sp["d"] == pytest.approx(-20.0 + (20.0 - 4.0))


@pytest.mark.asyncio
async def test_a_loss_before_any_range_waits_for_the_operator(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    _install_clock(monkeypatch)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOCKED, embedding=_EMB))
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()
    assert not plugin._filter.initialized
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "awaiting-operator"


@pytest.mark.asyncio
async def test_a_stale_pose_mid_search_holds_and_the_budget_still_runs(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    clock.advance(1.0)
    _arm_guided(plugin)
    await plugin._tick()
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_PREDICTING and st["commanding"] is False
    assert st["hold_reason"] == "pose-stale"
    clock.advance(4.2)
    _arm_guided(plugin)
    await plugin._tick()
    assert _state_events(ctx)[-1]["hold_reason"] == "search-exhausted"


@pytest.mark.asyncio
async def test_switching_to_strict_mid_search_waits_for_the_operator(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await ctx.config_kv.set("reacquire_mode", "strict")
    await _step(plugin, clock, 1.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_reason"] == "awaiting-operator"


# -- an EKF position reset ------------------------------------------------------


def _pose_at(plugin: Any, n: float, e: float, d: float, t_ms: int, vn: float = 0.0) -> None:
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    plugin._on_local_position(_local_position(n, e, d, vn, t_ms=t_ms))
    _arm_guided(plugin)


@pytest.mark.asyncio
async def test_an_ekf_position_reset_shifts_the_subject_and_the_hold_point(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(search_budget_s=5.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _lose(plugin, clock)
    await _step(plugin, clock, 5.1)
    st = _state_events(ctx)[-1]
    assert st["phase"] == PHASE_AWAIT_OPERATOR and st["hold_ne"] == [0.0, 0.0]
    before = plugin._filter.last_estimate()
    anchor = plugin._recovery.anchor_ne
    assert before is not None
    _pose_at(plugin, 0.0, 0.0, -20.0, 10_000)
    # Flown, not a reset: 1 m in 0.1 s at 10 m/s.
    _pose_at(plugin, 1.0, 0.0, -20.0, 10_100, vn=10.0)
    _pose_at(plugin, 1.0, 0.0, -20.0, 10_200)
    assert plugin._recovery.hold_ne == (0.0, 0.0)
    # The EKF re-bases its position: 5 m north, 3 m east in one sample.
    plugin.__class__._on_local_position(plugin, _local_position(6.0, 3.0, -21.0, t_ms=10_300))
    assert plugin._recovery.hold_ne == pytest.approx((5.0, 3.0))
    assert plugin._recovery.hold_d == pytest.approx(-21.0)
    assert plugin._recovery.anchor_ne == pytest.approx((anchor[0] + 5.0, anchor[1] + 3.0))
    after = plugin._filter.last_estimate()
    assert after is not None
    assert (after[0], after[1]) == pytest.approx((before[0] + 5.0, before[1] + 3.0))
    clock.advance(0.05)
    _arm_guided(plugin)
    await plugin._tick()
    sp = ctx.flight.setpoints[-1]
    assert (sp["n"], sp["e"], sp["d"]) == pytest.approx((5.0, 3.0, -21.0))


# -- a subject faster than the limit ------------------------------------------


@pytest.mark.asyncio
async def test_a_subject_faster_than_the_cap_is_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=1.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    st = _state_events(ctx)[-1]
    assert st["outpaced"] is False and st["speed_limit_mps"] == 1.0
    plugin._filter._x[2] = 2.0  # the estimate says 2 m/s
    cfg = plugin._cfg
    await plugin._publish_state(cfg, now=clock.now, active=True, phase=PHASE_FOLLOWING, commanding=True)
    assert _state_events(ctx)[-1]["outpaced"] is False
    clock.advance(2.1)
    await plugin._publish_state(cfg, now=clock.now, active=True, phase=PHASE_FOLLOWING, commanding=True)
    assert _state_events(ctx)[-1]["outpaced"] is True
    plugin._filter._x[2] = 0.5
    plugin._filter._x[3] = 0.0
    clock.advance(0.2)
    await plugin._publish_state(cfg, now=clock.now, active=True, phase=PHASE_FOLLOWING, commanding=True)
    assert _state_events(ctx)[-1]["outpaced"] is False
    assert math.isfinite(_state_events(ctx)[-1]["speed_limit_mps"])
