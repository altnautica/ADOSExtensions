"""A search for a subject lost while it stood still keeps clear of where it
stood.

Range noise on a standing subject leaves the estimate a slow walk's velocity.
A search that runs that velocity on carries its prediction metres off the
subject, and the vantage points a standoff from the prediction come round
onto the subject. So the search centres on where a still subject stood, with
no velocity (:mod:`follow_me.stillness`), and on the decayed coast for one
that moved; its guard keeps clear of the nearest place the subject may be,
the vantage it asks for is a standoff from each of them, and its goals name
one subject, the search centre, with the search's spread, to the pilot's
subject brake.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
import safety_cases as sc

from follow_me.recovery import vantage_standoff
from follow_me.state import MIN_FOLLOW_HEIGHT_M, PHASE_SEARCHING, SUBJECT_HEIGHT_M
from follow_me.stillness import STILL_SPEED_MPS, SpeedHistory
from follow_me.target_filter import TargetFilter

_R = ((0.75**2, 0.0), (0.0, 0.2**2))
# Horizontal distance never closed on the subject unless this far above
# its head.
_CLEAR_M = 3.0
# From the loss on (the follow's goal hands over to the search's once,
# 3 s of unsure lock after the boxes stop).
_LOST_S = sc.STILL_S + 3.0


def _assert_clear(f: Any, seed: int) -> float:
    least = math.inf
    for i in f.after(sc.STILL_S):
        above_head = f.height[i] - SUBJECT_HEIGHT_M
        assert f.range(i) >= _CLEAR_M or above_head >= MIN_FOLLOW_HEIGHT_M, (seed, f.t[i])
        least = min(least, f.range(i))
    return least


def _assert_settled(f: Any, seed: int, backing_max: float = 0.1) -> None:
    # One subject named through the search (the hand-over from the follow's
    # goal to the search's is the one step allowed), so the pilot's subject
    # brake stays armed; and the search asks for vantages, not backing away
    # from them.
    assert sc.subject_jumps(f, _LOST_S) <= 1, seed
    assert sc.backing_away_share(f, _LOST_S) < backing_max, seed


@pytest.mark.asyncio
@pytest.mark.parametrize("hz", [3.0, 10.0])
@pytest.mark.parametrize("brake", [False, True])
async def test_a_subject_lost_standing_is_searched_round_where_it_stood(
    monkeypatch: Any, hz: float, brake: bool
) -> None:
    least = math.inf
    for seed in range(1, 11):
        f = await sc.still_then_lost(monkeypatch, seed, brake, hz=hz)
        filt = f.plugin._filter
        est = filt.last_estimate()
        # The scenario: the estimate was walking off when the lock went.
        assert math.hypot(est[2], est[3]) >= 0.3
        assert PHASE_SEARCHING in f.phases
        centre = filt.still_centre(f.plugin._recovery.anchor_ne)
        assert centre is not None
        least = min(least, _assert_clear(f, seed))
        _assert_settled(f, seed)
        vantage_goals = [g for g in f.goals if g.get("kind") == "vantage"]
        assert vantage_goals
        for g in vantage_goals:
            assert g["vantage"]["predicted_ne"] == [centre[0], centre[1]]
            assert g["subject_ne"] == [centre[0], centre[1]]
            assert g["subject_sigma_m"] > 0.0
        for v in f.vantage:
            if v is not None:
                assert math.hypot(v[0] - centre[0], v[1] - centre[1]) >= sc.STANDOFF_M - 1e-6
    print(f"{hz} Hz, brake {brake}: least separation {least:.2f} m over 10 seeds")


@pytest.mark.asyncio
@pytest.mark.parametrize("hz", [3.0, 10.0])
async def test_a_standing_subject_whose_estimate_drifts_toward_the_vehicle(
    monkeypatch: Any, hz: float
) -> None:
    # The same, the drift toward the vehicle: no longer taken for still (it
    # may be walking up), so the search keeps clear of the walk run on too,
    # without fighting its own vantage requests.
    least = math.inf
    for seed in range(1, 6):
        f = await sc.still_then_lost(monkeypatch, seed, True, hz=hz, toward=True)
        assert PHASE_SEARCHING in f.phases
        least = min(least, _assert_clear(f, seed))
        _assert_settled(f, seed)
    print(f"{hz} Hz: least separation {least:.2f} m over 5 seeds")


@pytest.mark.asyncio
@pytest.mark.parametrize("hz", [3.0, 10.0])
async def test_a_subject_that_sets_off_toward_the_vehicle_before_the_loss(
    monkeypatch: Any, hz: float
) -> None:
    # Standing, then walking up to the vehicle at 0.5 m/s from 2 s before
    # the boxes stop: it reads much like a drifting standing subject. The
    # vehicle backs away while the walk may still be coming (up to
    # GUARD_RUN_ON_S), never for most of the search.
    least = math.inf
    for seed in range(1, 6):
        f = await sc.stand_then_walk_toward(monkeypatch, seed, True, hz=hz)
        least = min(least, _assert_clear(f, seed))
        _assert_settled(f, seed, backing_max=0.5)
    print(f"{hz} Hz: least separation {least:.2f} m over 5 seeds")


def test_the_vantage_is_asked_a_standoff_from_every_place_the_subject_may_be() -> None:
    x = [0.0, 0.0, 0.0, 0.0]
    assert vantage_standoff(6.0, x, [((0.0, 0.0), (0.0, 0.0))]) == 6.0
    # A point 1.5 m off the centre: the circle widens past it by the standoff.
    near = [((0.0, 0.0), (0.0, 0.0)), ((-1.5, 0.0), (0.0, 0.0))]
    r = vantage_standoff(6.0, x, near)
    assert r == pytest.approx(7.5)
    for a in range(0, 360, 15):
        v = (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
        assert math.hypot(v[0] + 1.5, v[1]) >= 6.0 - 1e-9
    # One far outside the circle by the standoff leaves it as it is.
    far = [((0.0, 0.0), (0.0, 0.0)), ((30.0, 0.0), (0.0, 0.0))]
    assert vantage_standoff(6.0, x, far) == 6.0


def test_a_walk_lost_coasts_rather_than_running_on() -> None:
    f = TargetFilter()
    f.max_speed_mps = 5.0
    t = 0.0
    for i in range(60):
        t = i / 10.0
        f.update(t, 1.4 * t, 0.0, _R)
    assert f.still_centre() is None
    late = t + 30.0
    search = f.search_predicted(late)
    assert search is not None
    assert search == f.coasted(late)
    # The coast stops within a few metres of the last estimate; the
    # constant-velocity prediction would be 42 m on.
    assert search[0][0] - f.last_estimate()[0] < 5.0  # type: ignore[index]
    assert math.hypot(search[0][2], search[0][3]) < 0.01


def test_a_still_subject_is_searched_at_its_centre_with_no_velocity() -> None:
    f = TargetFilter()
    f.max_speed_mps = 5.0
    t = 0.0
    for i in range(60):
        t = i / 10.0
        # Standing at (3, 2), the last 1.5 s of boxes drifting 0.45 m/s off.
        drift = 0.45 * max(0.0, t - 4.5)
        f.update(t, 3.0 + drift, 2.0, _R)
    centre = f.still_centre()
    assert centre is not None
    assert math.hypot(centre[0] - 3.0, centre[1] - 2.0) < 0.3
    search = f.search_predicted(t + 40.0)
    assert search is not None
    assert search[0] == [centre[0], centre[1], 0.0, 0.0]


def _history(points: list[tuple[float, float, float, float, float]]) -> SpeedHistory:
    h = SpeedHistory()
    for p in points:
        h.add(*p)
    return h


def test_the_still_rule() -> None:
    # (t, n, e, vn, ve); the vehicle 6 m south.
    vehicle = (-6.0, 0.0)
    standing = [(i / 3.0, 0.1 * (-1) ** i, 0.0, 0.2, 0.0) for i in range(20)]
    assert _history(standing).still_centre(vehicle) is not None
    # Too little seen to say.
    assert _history(standing[:12]).still_centre(vehicle) is None
    # A slow walk: low speed, but it got somewhere.
    walk = [(i / 3.0, 0.35 * i / 3.0, 0.0, 0.25, 0.0) for i in range(20)]
    assert _history(walk).still_centre(vehicle) is None
    # Speeds a walk keeps up.
    brisk = [(t, n, e, STILL_SPEED_MPS + 0.2, 0.0) for t, n, e, _vn, _ve in standing]
    assert _history(brisk).still_centre(vehicle) is None
    # Set off just before the loss.
    off = standing[:-1] + [(standing[-1][0], 0.0, 0.0, 0.9, 0.0)]
    assert _history(off).still_centre(vehicle) is None
    # The last 2 s drifting 0.45 m/s: away from the vehicle it still stood
    # (a walk away is searched for behind it); toward it, it may be walking
    # up, and is not.
    def drifting(sign: float) -> list[tuple[float, float, float, float, float]]:
        out = []
        for t, n, e, _vn, _ve in standing:
            d = max(0.0, t - (standing[-1][0] - 2.0))
            out.append((t, n + sign * 0.45 * d, e, sign * min(0.45, d), 0.0))
        return out

    assert _history(drifting(1.0)).still_centre(vehicle) is not None
    assert _history(drifting(-1.0)).still_centre(vehicle) is None
    assert _history(drifting(-1.0)).still_centre() is not None
    # A frame jump moves the centre with it.
    h = _history(standing)
    before = h.still_centre(vehicle)
    h.shift(5.0, -2.0)
    after = h.still_centre((vehicle[0] + 5.0, vehicle[1] - 2.0))
    assert before is not None and after is not None
    assert after == pytest.approx((before[0] + 5.0, before[1] - 2.0))
