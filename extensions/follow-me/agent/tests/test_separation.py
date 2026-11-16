"""The follow never flies into a subject that turns back on it.

A walker or runner followed from behind who turns back toward the vehicle
closes the 6 m standoff at 3-6 m/s. Monocular range (σ 0.75 m along the
line of sight) takes half a second to a second of fixes to show the turn,
and the vehicle then needs about as long again to stop at the pilot's
profile: from straight ahead no controller keeps the horizontal range open
(an ideal one, knowing the turn at once, gets 1.8 m against a walker and
0.25 m against a runner). What keeps the subject safe is the height: the
vehicle never follows lower than MIN_FOLLOW_HEIGHT_M, so a head-on reversal
passes under it with the rotors at least OVERHEAD_CLEARANCE_M above a
SUBJECT_HEIGHT_M tall subject. On top of that the follow stops closing on
the subject, and backs away, once the subject comes within the safe
distance (:func:`follow_me.standoff.keep_clear`).
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from follow_sim import BODY_HEIGHT_M, BODY_RADIUS_M, VEHICLE_RADIUS_M, fly

from follow_me.standoff import SAFE_DISTANCE_M, keep_clear, safe_distance
from follow_me.state import (
    MIN_FOLLOW_HEIGHT_M,
    OVERHEAD_CLEARANCE_M,
    ROTOR_ENVELOPE_M,
    SUBJECT_HEIGHT_M,
    FollowConfig,
)

_TURN_S = 25.0
# DISTANCE_SENSOR reports whole centimetres.
_RANGEFINDER_RESOLUTION_M = 0.01


def _reversal(angle_deg: float, speed: float, turn_s: float) -> Any:
    """A walker at 1.4 m/s who, at _TURN_S, turns ``angle_deg`` (180: back
    straight at the vehicle) to ``speed``: the velocity goes linearly from
    the walk to the new one over ``turn_s`` (through a stop, head-on)."""
    a = math.radians(angle_deg)
    v0 = (1.4, 0.0)
    v1 = (speed * math.cos(a), speed * math.sin(a))
    p0 = (30.0 + 1.4 * _TURN_S, 0.0)

    def subject(t: float) -> tuple[float, float]:
        if t <= _TURN_S:
            return (30.0 + 1.4 * t, 0.0)
        dt = t - _TURN_S
        if turn_s <= 0.0 or dt >= turn_s:
            tt = min(dt, turn_s) if turn_s > 0.0 else 0.0
            return (
                p0[0] + 0.5 * (v0[0] + v1[0]) * tt + v1[0] * (dt - tt),
                p0[1] + 0.5 * (v0[1] + v1[1]) * tt + v1[1] * (dt - tt),
            )
        k = dt / turn_s
        vn, ve = v0[0] + (v1[0] - v0[0]) * k, v0[1] + (v1[1] - v0[1]) * k
        return (p0[0] + 0.5 * (v0[0] + vn) * dt, p0[1] + 0.5 * (v0[1] + ve) * dt)

    return subject


def test_the_lowest_follow_height_clears_a_tall_subject() -> None:
    # One documented number: a 2 m subject, the rotors' reach, 1 m between.
    assert MIN_FOLLOW_HEIGHT_M == pytest.approx(SUBJECT_HEIGHT_M + ROTOR_ENVELOPE_M + OVERHEAD_CLEARANCE_M)
    assert MIN_FOLLOW_HEIGHT_M == pytest.approx(3.4)
    # The closed-loop sim measures clearance with the same body and rotors.
    assert (BODY_HEIGHT_M, VEHICLE_RADIUS_M) == (SUBJECT_HEIGHT_M, ROTOR_ENVELOPE_M)


@pytest.mark.parametrize("asked", [0.0, 1.5, 2.5, 3.3])
def test_a_follow_height_below_the_lowest_is_held_at_it_and_said_so(asked: float) -> None:
    c = FollowConfig.resolve({"follow_height_m": asked}, {})
    assert c.follow_height_m == pytest.approx(MIN_FOLLOW_HEIGHT_M)
    assert c.follow_height_raised


@pytest.mark.parametrize("asked", [3.4, 4.0, 12.0])
def test_a_follow_height_at_or_above_the_lowest_is_flown_as_set(asked: float) -> None:
    c = FollowConfig.resolve({"follow_height_m": asked}, {})
    assert c.follow_height_m == pytest.approx(asked)
    assert not c.follow_height_raised


@pytest.mark.asyncio
async def test_a_low_follow_height_is_flown_at_the_lowest_and_read_back_raised(monkeypatch: Any) -> None:
    f = await fly(
        monkeypatch,
        subject=lambda t: (30.0 + 1.4 * t, 0.0),
        start=(24.0, 0.0),
        duration_s=15.0,
        config={"follow_height_m": 1.5},
        fix_hz=10.0,
    )
    assert min(f.height[i] for i in f.after(10.0)) >= MIN_FOLLOW_HEIGHT_M - 0.05
    assert f.state[-1]["height_raised"] is True
    assert f.state[-1]["height_setpoint_m"] == pytest.approx(MIN_FOLLOW_HEIGHT_M)


def test_the_safe_distance_comes_from_the_standoff() -> None:
    assert safe_distance(6.0) == SAFE_DISTANCE_M == 2.5
    # A short standoff keeps a metre inside it.
    assert safe_distance(3.0) == pytest.approx(2.0)


def _along(v: tuple[float, float], u: tuple[float, float]) -> float:
    return v[0] * u[0] + v[1] * u[1]


def test_closing_on_the_subject_stops_short_of_the_safe_distance_or_passes_it() -> None:
    # The subject stands 4 m north; the follow asks 2 m/s straight at it.
    # The nearest admissible velocity sidesteps: its path passes the subject
    # at the safe distance, not nearer.
    out, side, limited = keep_clear(
        rel_ne=(4.0, 0.0),
        subject_v_ne=(0.0, 0.0),
        pace_v_ne=(0.0, 0.0),
        want_ne=(2.0, 0.0),
        command_v_ne=(2.0, 0.0),
        range_sigma_m=0.0,
        safe_m=2.5,
        side=0,
    )
    assert limited and side != 0
    speed = math.hypot(*out)
    miss = 4.0 * abs(out[1]) / speed
    assert miss >= 2.5 - 1e-6


def test_inside_the_safe_distance_the_vehicle_backs_away_from_an_oncoming_subject() -> None:
    # 1.5 m away and walking at the vehicle at 1.4 m/s; the follow asks to
    # hold still.
    out, _side, limited = keep_clear(
        rel_ne=(1.5, 0.0),
        subject_v_ne=(-1.4, 0.0),
        pace_v_ne=(-1.4, 0.0),
        want_ne=(0.0, 0.0),
        command_v_ne=(0.0, 0.0),
        range_sigma_m=0.0,
        safe_m=2.5,
        side=0,
    )
    assert limited
    assert _along(out, (1.0, 0.0)) <= -1.4 - 0.9


def test_an_uncertain_range_is_taken_short() -> None:
    # 3.5 m away, the follow asking a slow 0.3 m/s in: clear on exact range,
    # limited when the estimate carries 0.75 m of error.
    kw: dict[str, Any] = {
        "rel_ne": (3.5, 0.0),
        "subject_v_ne": (0.0, 0.0),
        "pace_v_ne": (0.0, 0.0),
        "want_ne": (0.3, 0.0),
        "command_v_ne": (0.3, 0.0),
        "safe_m": 2.5,
        "side": 0,
    }
    assert not keep_clear(range_sigma_m=0.0, **kw)[2]
    assert keep_clear(range_sigma_m=0.75, **kw)[2]


def test_the_newest_fixes_nearer_than_the_estimate_are_kept_clear_of() -> None:
    # The estimate trails a subject that turned back: it still reads 6 m
    # and walking away, while the newest fixes put the subject at 2 m.
    out, _side, limited = keep_clear(
        rel_ne=(6.0, 0.0),
        subject_v_ne=(1.0, 0.0),
        pace_v_ne=(1.0, 0.0),
        want_ne=(1.2, 0.0),
        command_v_ne=(1.2, 0.0),
        range_sigma_m=0.0,
        safe_m=2.5,
        side=0,
        near_m=2.0,
    )
    assert limited
    assert out[0] < 0.0


def test_a_path_that_passes_the_subject_is_not_limited() -> None:
    # 5 m north, the vehicle flying east past it: never within 2.5 m.
    out, _side, limited = keep_clear(
        rel_ne=(5.0, 0.0),
        subject_v_ne=(0.0, 0.0),
        pace_v_ne=(0.0, 0.0),
        want_ne=(0.0, 3.0),
        command_v_ne=(0.0, 3.0),
        range_sigma_m=0.0,
        safe_m=2.5,
        side=0,
    )
    assert not limited and out == (0.0, 3.0)


def _clearance_3d(f: Any, i: int) -> float:
    out = max(0.0, f.range(i) - BODY_RADIUS_M)
    up = max(0.0, f.height[i] - BODY_HEIGHT_M)
    return math.hypot(out, up) - VEHICLE_RADIUS_M


# Turns back from straight ahead to 60° off, at a walk, a jog and a run;
# at once and over a second. The follow height asked is 1.5 m, below the
# lowest: it is flown at the lowest.
@pytest.mark.asyncio
@pytest.mark.parametrize("fix_hz", [10.0, 6.0])
@pytest.mark.parametrize("speed", [1.4, 3.0, 5.0])
@pytest.mark.parametrize("angle", [180.0, 165.0, 150.0, -150.0, 120.0])
async def test_a_subject_turning_back_never_comes_within_a_metre_of_the_rotors(
    monkeypatch: Any, angle: float, speed: float, fix_hz: float
) -> None:
    least, least_h = math.inf, math.inf
    for turn_s in (0.0, 1.0):
        for seed in (1, 2):
            f = await fly(
                monkeypatch,
                subject=_reversal(angle, speed, turn_s),
                start=(24.0, 0.0),
                duration_s=_TURN_S + 12.0,
                config={"follow_height_m": 1.5},
                miss=0.3,
                seed=seed,
                fix_hz=fix_hz,
            )
            idx = f.after(_TURN_S)
            least = min(least, min(_clearance_3d(f, i) for i in idx))
            least_h = min(least_h, min(f.range(i) for i in idx))
    print(f"{angle:.0f}° {speed} m/s {fix_hz:.0f} Hz: least 3-D clearance {least:.2f} m, horizontal {least_h:.2f} m")
    # The body passes under the rotors at the lowest follow height or
    # further off, to within the rangefinder's 1 cm the height is held on.
    assert least >= OVERHEAD_CLEARANCE_M - _RANGEFINDER_RESOLUTION_M
