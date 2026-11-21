"""A search for a subject lost while it stood still keeps clear of where it
stood.

Range noise on a standing subject leaves the estimate a slow walk's velocity.
A search that runs that velocity on carries its prediction metres off the
subject, and the vantage points a standoff from the prediction come round
onto the subject. So the search centres on where a still subject stood, with
no velocity (:mod:`follow_me.stillness`), and on the decayed coast for one
that moved; its guard keeps clear of the nearest place the subject may be,
and its goals name the search's spread to the pilot's subject brake.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
import safety_cases as sc

from follow_me.state import MIN_FOLLOW_HEIGHT_M, PHASE_SEARCHING, SUBJECT_HEIGHT_M
from follow_me.stillness import STILL_SPEED_MPS, SpeedHistory
from follow_me.target_filter import TargetFilter

_R = ((0.75**2, 0.0), (0.0, 0.2**2))
# Horizontal distance never closed on the subject unless this far above
# its head.
_CLEAR_M = 3.0


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
        centre = filt.still_centre()
        assert centre is not None
        for i in f.after(sc.STILL_S):
            above_head = f.height[i] - SUBJECT_HEIGHT_M
            assert f.range(i) >= _CLEAR_M or above_head >= MIN_FOLLOW_HEIGHT_M, (seed, f.t[i])
            least = min(least, f.range(i))
        vantage_goals = [g for g in f.goals if g.get("kind") == "vantage"]
        assert vantage_goals
        for g in vantage_goals:
            assert g["vantage"]["predicted_ne"] == [centre[0], centre[1]]
            assert g["subject_sigma_m"] > 0.0
        for v in f.vantage:
            if v is not None:
                assert math.hypot(v[0] - centre[0], v[1] - centre[1]) >= sc.STANDOFF_M - 1e-6
    print(f"{hz} Hz, brake {brake}: least separation {least:.2f} m over 10 seeds")


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


def _history(points: list[tuple[float, float, float, float]]) -> SpeedHistory:
    h = SpeedHistory()
    for p in points:
        h.add(*p)
    return h


def test_the_still_rule() -> None:
    standing = [(i / 3.0, 0.1 * (-1) ** i, 0.0, 0.2) for i in range(20)]
    assert _history(standing).still_centre() is not None
    # Too little seen to say.
    assert _history(standing[:12]).still_centre() is None
    # A slow walk: low speed, but it got somewhere.
    walk = [(i / 3.0, 0.35 * i / 3.0, 0.0, 0.25) for i in range(20)]
    assert _history(walk).still_centre() is None
    # Speeds a walk keeps up.
    brisk = [(t, n, e, STILL_SPEED_MPS + 0.2) for t, n, e, _s in standing]
    assert _history(brisk).still_centre() is None
    # Set off just before the loss.
    off = standing[:-1] + [(standing[-1][0], 0.0, 0.0, 0.9)]
    assert _history(off).still_centre() is None
    # A frame jump moves the centre with it.
    h = _history(standing)
    before = h.still_centre()
    h.shift(5.0, -2.0)
    after = h.still_centre()
    assert before is not None and after is not None
    assert after == pytest.approx((before[0] + 5.0, before[1] - 2.0))
