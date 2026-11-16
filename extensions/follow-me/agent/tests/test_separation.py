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
from follow_sim import (
    BODY_HEIGHT_M,
    BODY_RADIUS_M,
    SPORT_ACCEL_MPS2,
    SPORT_JERK_MPS3,
    VEHICLE_RADIUS_M,
    fly,
)

import follow_me
import follow_me.loop as loop
import follow_me.standoff as standoff
from follow_me.speed_limit import NEAR_FIXES, NEAR_SPAN_S, SubjectMotion
from follow_me.standoff import (
    COMMAND_ACCEL_MPS2,
    SAFE_DISTANCE_M,
    SAFE_STANDOFF_MARGIN_M,
    StandoffController,
    clear_distance,
    keep_clear,
    safe_distance,
)
from follow_me.state import (
    FOLLOW_DISTANCE_RANGE,
    MIN_FOLLOW_HEIGHT_M,
    ORBIT_RADIUS_RANGE,
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
    # A short standoff keeps two metres inside it.
    assert safe_distance(4.0) == pytest.approx(2.0)
    assert safe_distance(3.0) == pytest.approx(1.0)


@pytest.mark.parametrize("standoff_m", [FOLLOW_DISTANCE_RANGE[0], ORBIT_RADIUS_RANGE[0], 3.5, 4.0, 5.0, 6.0, 8.0])
@pytest.mark.parametrize("sigma", [0.0, 0.4, 0.75, 2.0])
def test_the_clear_distance_stays_inside_the_standoff(standoff_m: float, sigma: float) -> None:
    # Range margin included, never nearer the standoff than its margin: a
    # vehicle held at the standoff is not inside what it keeps clear of.
    clear = clear_distance(standoff_m, sigma)
    assert safe_distance(standoff_m) <= clear <= standoff_m - SAFE_STANDOFF_MARGIN_M + 1e-9


def test_the_clear_distance_adds_the_range_margin_at_the_default_standoff() -> None:
    assert clear_distance(6.0, 0.0) == pytest.approx(2.5)
    assert clear_distance(6.0, 0.5) == pytest.approx(3.5)
    assert clear_distance(6.0, 0.75) == pytest.approx(4.0)
    assert clear_distance(6.0, 3.0) == pytest.approx(4.0)


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
        standoff_m=6.0,
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
        standoff_m=6.0,
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
        "standoff_m": 6.0,
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
        standoff_m=6.0,
        side=0,
        near_m=2.0,
    )
    assert limited
    assert out[0] < 0.0


def test_newest_fixes_farther_than_the_estimate_do_not_relax_the_limit() -> None:
    # 5 m off and closing at 1.5 m/s: the limit is the estimate's; newest
    # fixes that put the subject farther leave it as it is, nearer ones
    # tighten it.
    kw: dict[str, Any] = {
        "rel_ne": (5.0, 0.0),
        "subject_v_ne": (0.0, 0.0),
        "pace_v_ne": (0.0, 0.0),
        "want_ne": (1.5, 0.0),
        "command_v_ne": (1.5, 0.0),
        "range_sigma_m": 0.0,
        "standoff_m": 6.0,
        "side": 0,
    }
    own = keep_clear(**kw)
    assert own[2]
    assert keep_clear(near_m=9.0, **kw) == own
    assert keep_clear(near_m=5.0, **kw) == own
    nearer = keep_clear(near_m=3.5, **kw)
    assert nearer[2] and _along(nearer[0], (1.0, 0.0)) < _along(own[0], (1.0, 0.0))


def test_a_cut_steps_only_along_the_line_of_sight() -> None:
    # Flying 2 m/s north toward a goal far ahead, the subject comes into
    # view 3.5 m ahead: the closing speed is cut in one tick (not at the
    # 1 m/s² slew), the sideways part of the sidestep is still slewed.
    c = StandoffController()
    kw: dict[str, Any] = {
        "goal_ne": (30.0, 0.5),
        "goal_v_ne": (0.0, 0.0),
        "drone_ne": (0.0, 0.0),
        "drone_v_ne": (2.0, 0.0),
        "drone_yaw": 0.0,
        "yaw_target": 0.0,
        "still": False,
        "latch_goal": False,
        "max_speed_mps": 2.0,
        "subject_v_ne": (0.0, 0.0),
        "standoff_m": 6.0,
    }
    before = c.command(now=0.0, subject_ne=(30.0, 0.0), **kw)
    assert not c.limited
    dt = 0.05
    after = c.command(now=dt, subject_ne=(3.5, 0.0), **kw)
    assert c.limited
    # Along the line of sight: down by 20 slew steps or more.
    assert before.vn_mps - after.vn_mps >= 20.0 * COMMAND_ACCEL_MPS2 * dt
    # Its path no longer reaches the safe distance, ahead or beside.
    assert after.vn_mps <= 1.0
    # Sideways: one slew step at most.
    assert abs(after.ve_mps - before.ve_mps) <= COMMAND_ACCEL_MPS2 * dt + 1e-9


def _motion(fixes: list[tuple[float, float]]) -> SubjectMotion:
    m = SubjectMotion()
    for t, r in fixes:
        m.record(t, r, 0.0, r)
    return m


def test_the_near_range_is_the_median_of_the_newest_fixes() -> None:
    assert NEAR_FIXES == 5
    # Fewer than NEAR_FIXES fixes: none.
    assert _motion([(0.1 * k, 6.0) for k in range(NEAR_FIXES - 1)]).near_range(0.4) is None
    # The newest NEAR_FIXES only: older fixes do not hold it out.
    m = _motion([(0.1 * k, r) for k, r in enumerate([9.0, 9.0, 9.0, 3.0, 3.1, 2.9, 3.0, 3.2])])
    assert m.near_range(0.7) == pytest.approx(3.0)
    # One outlier, either way, does not move it.
    assert _motion([(0.1 * k, r) for k, r in enumerate([6.0, 6.1, 1.0, 5.9, 6.0])]).near_range(0.4) == 6.0
    assert _motion([(0.1 * k, r) for k, r in enumerate([6.0, 6.1, 12.0, 5.9, 6.0])]).near_range(0.4) == 6.0


def test_the_near_range_takes_only_fixes_of_the_last_span() -> None:
    fixes = [(0.0, 6.0), (0.3, 6.0), (0.6, 6.0), (0.9, 6.0), (1.2, 6.0)]
    assert _motion(fixes).near_range(1.2) == 6.0
    assert _motion(fixes).near_range(NEAR_SPAN_S + 1e-6) is None
    assert _motion([*fixes, (1.5, 2.0), (1.6, 2.0)]).near_range(1.6) == 6.0
    assert _motion([*fixes, (1.5, 2.0), (1.6, 2.0), (1.7, 2.0)]).near_range(1.7) == 2.0


@pytest.mark.asyncio
@pytest.mark.parametrize("short_m", [0.0, 3.0])
async def test_goals_carry_the_subject_range_error(monkeypatch: Any, short_m: float) -> None:
    # Every goal names the subject's range error for the pilot's subject
    # brake: at least the estimate's own, and half what the newest fixes
    # put the subject short of the estimate by (here, made 3 m short).
    sigmas: list[float] = []
    nears: list[float | None] = []
    seen: list[tuple[dict[str, Any], float, float | None, float]] = []
    error, near_range, goal = loop._estimate_error, SubjectMotion.near_range, follow_me.FollowMePlugin._goal

    def spy_error(*a: Any, **k: Any) -> Any:
        out = error(*a, **k)
        sigmas.append(out[0])
        return out

    def spy_near(self: SubjectMotion, now: float) -> float | None:
        r = near_range(self, now)
        r = None if r is None else max(0.5, r - short_m)
        nears.append(r)
        return r

    def spy_goal(self: Any, *a: Any, **k: Any) -> dict[str, Any]:
        nears.append(None)
        g = goal(self, *a, **k)
        est = math.hypot(g["subject_ne"][0] - self._pose.north_m, g["subject_ne"][1] - self._pose.east_m)
        seen.append((g, sigmas[-1], nears[-1], est))
        return g

    monkeypatch.setattr(loop, "_estimate_error", spy_error)
    monkeypatch.setattr(SubjectMotion, "near_range", spy_near)
    monkeypatch.setattr(follow_me.FollowMePlugin, "_goal", spy_goal)
    await fly(
        monkeypatch,
        subject=lambda t: (8.0 + 1.4 * t, 0.0),
        start=(2.0, 0.0),
        duration_s=20.0,
        miss=0.3,
        seed=3,
        fix_hz=10.0,
    )
    assert len(seen) > 100
    for g, range_sigma, near, est in seen:
        assert g["subject_sigma_m"] >= range_sigma - 1e-12
        if near is not None:
            assert g["subject_sigma_m"] >= (est - near) / 2.0 - 1e-9
    with_near = [g["subject_sigma_m"] for g, _s, near, _e in seen if near is not None]
    assert with_near
    if short_m:
        assert min(with_near) >= 1.0


# A jogger (2.5 m/s) followed at the default 6 m under the pilot's Sport
# profile (2.5 m/s²), who stops dead. The speed law alone slews its
# command at 1 m/s² and the vehicle comes over the subject; the guard cuts
# the closing speed at once, which the pilot flies at Sport's limits.
@pytest.mark.asyncio
@pytest.mark.parametrize("fix_hz", [10.0, 6.0])
async def test_a_jogger_stopping_dead_is_not_overflown_under_the_sport_profile(
    monkeypatch: Any, fix_hz: float
) -> None:
    stop_s = 20.0

    def subject(t: float) -> tuple[float, float]:
        return (8.0 + 2.5 * min(t, stop_s), 0.0)

    async def least_ranges() -> list[float]:
        out = []
        for seed in range(1, 7):
            f = await fly(
                monkeypatch,
                subject=subject,
                start=(2.0, 0.0),
                duration_s=stop_s + 12.0,
                config={"max_speed_mps": 8.0},
                miss=0.3,
                seed=seed,
                fix_hz=fix_hz,
                accel_mps2=SPORT_ACCEL_MPS2,
                jerk_mps3=SPORT_JERK_MPS3,
            )
            out.append(min(f.range(i) for i in f.after(stop_s)))
        return out

    guarded = await least_ranges()
    monkeypatch.setattr(standoff, "keep_clear", lambda **kw: (kw["want_ne"], 0, False))
    unguarded = await least_ranges()
    print(
        f"{fix_hz:g} Hz least range: guarded {[round(r, 2) for r in guarded]},"
        f" unguarded {[round(r, 2) for r in unguarded]}"
    )
    # The rotors never come over the body.
    over = BODY_RADIUS_M + VEHICLE_RADIUS_M
    assert min(guarded) >= over
    # The control: without the guard they do.
    assert min(unguarded) < over


def test_a_path_that_passes_the_subject_is_not_limited() -> None:
    # 5 m north, the vehicle flying east past it: never within 2.5 m.
    out, _side, limited = keep_clear(
        rel_ne=(5.0, 0.0),
        subject_v_ne=(0.0, 0.0),
        pace_v_ne=(0.0, 0.0),
        want_ne=(0.0, 3.0),
        command_v_ne=(0.0, 3.0),
        range_sigma_m=0.0,
        standoff_m=6.0,
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
