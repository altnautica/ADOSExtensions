"""The vehicle never comes onto the subject, whatever a false lock, a coast
or the subject's own manoeuvre does to the estimate.

* A coast (no fix for a while) lets the velocity flown on decay the less it
  is known (:meth:`follow_me.target_filter.TargetFilter.coasted`).
* A box after a gap lying beyond what the subject can reach is held out
  (:data:`follow_me.target_filter.SUBJECT_ACCEL_MAX_MPS2`), and held-out
  fixes expire.
* The hard guard keeps the safe distance from the subject's worst case (the
  nearer of the estimate flown on and its undecayed prediction, or a
  standing subject's centre; on a coast a sigma of its velocity's error
  faster toward the vehicle), widened on a coast, against the vehicle's own
  velocity as well as the command, braking at the slowest profile's goal
  deceleration after the pilot's lag.

The flights (:mod:`safety_cases`) fly with the pilot's own subject brake off
unless named, so the follow's guard is all there is. Each asserts the least
true horizontal separation over every seed.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
import safety_cases as sc
from follow_sim import SPORT, fly

from follow_me.standoff import (
    COAST_CLEAR_INSIDE_M,
    SEPARATION_DECEL_MPS2,
    SEPARATION_LAG_S,
    clear_distance,
    keep_clear,
)
from follow_me.target_filter import COAST_GRACE_S, TargetFilter

_R = ((0.75**2, 0.0), (0.0, 0.2**2))
STANDOFF_M = 6.0


def _walk(f: TargetFilter, fixes: int, hz: float = 10.0) -> float:
    t = 0.0
    for i in range(fixes):
        t = i / hz
        f.update(t, 10.0 + 1.4 * t, 0.0, _R)
    return t


def test_a_coast_lets_the_velocity_go_the_sooner_the_less_it_is_known() -> None:
    long_track, young = TargetFilter(), TargetFilter()
    t_long = _walk(long_track, 60)
    t_young = _walk(young, 5)
    for f, t in ((long_track, t_long), (young, t_young)):
        # Inside the grace a missed frame or two flies the prediction as it is.
        assert f.coasted(t + COAST_GRACE_S) == f.predicted(t + COAST_GRACE_S)
    late = 3.0
    v_long = long_track.coasted(t_long + late)[0][2]  # type: ignore[index]
    v_young = young.coasted(t_young + late)[0][2]  # type: ignore[index]
    assert young.coast_tau_s() < 0.5 * long_track.coast_tau_s()
    assert 0.0 < v_young < 0.1 * young.last_estimate()[2]  # type: ignore[index]
    assert v_young < v_long < long_track.last_estimate()[2]  # type: ignore[index]
    pred, coast = young.predicted(t_young + late), young.coasted(t_young + late)
    assert pred is not None and coast is not None
    assert young.last_estimate()[0] < coast[0][0] < pred[0][0]  # type: ignore[index]
    assert young.coast_spread_m(t_young + 0.4) == 0.0
    assert young.coast_spread_m(t_young + late) > 1.0


def test_a_box_beyond_the_subjects_reach_after_a_gap_is_held_out() -> None:
    # 1.9 s after the last fix a box 8 m beyond a walker: past what it can
    # reach (1.5 m/s² for 1.9 s is 2.7 m), held out. A second one agreeing
    # with it starts the estimate over there, at no speed (the old bound
    # fused both and ran the estimate on at the 5 m/s cap); the walker's own
    # boxes after them bring it back.
    f = TargetFilter()
    f.max_speed_mps = 5.0
    t = _walk(f, 30) + 1.9
    assert not f.update(t, 10.0 + 1.4 * t + 8.0, 0.0, _R)
    f.update(t + 0.1, 10.0 + 1.4 * (t + 0.1) + 8.0, 0.0, _R)
    est = f.last_estimate()
    assert est is not None and abs(est[2]) < 1.5
    for k in range(2, 8):
        tk = t + k / 10.0
        f.update(tk, 10.0 + 1.4 * tk, 0.0, _R)
    est = f.last_estimate()
    assert est is not None and abs(est[0] - (10.0 + 1.4 * (t + 0.7))) < 1.0 and abs(est[2]) < 3.0


def test_held_out_fixes_expire() -> None:
    # A box held out as jumped confirms nothing it is more than a second
    # from, nor anything after a hold: one later false box does not start
    # the estimate over.
    f = TargetFilter()
    t = _walk(f, 30)
    assert not f.update(t + 1.0, 30.0, 4.0, _R)
    assert not f.update(t + 2.5, 30.0, 4.0, _R)
    f2 = TargetFilter()
    t = _walk(f2, 30)
    assert not f2.update(t + 1.0, 30.0, 4.0, _R)
    f2.stop(t + 1.1)
    assert not f2.update(t + 1.2, 30.0, 4.0, _R)


@pytest.mark.parametrize("coast_sigma", [0.0, 0.5, 1.5, 4.0])
def test_no_closing_speed_runs_past_the_widened_clear_distance(coast_sigma: float) -> None:
    # Toward a standing subject, at any range and closing speed: a velocity
    # let through that does not speed the closing up either passes the
    # subject outside the clear distance or stops short of it, braking at
    # the slowest profile's 0.6 m/s² after the pilot's lag. The clear
    # distance widens with the coast's spread, never past 1 m inside the
    # standoff.
    clear = clear_distance(STANDOFF_M, 0.4, coast_sigma)
    assert clear <= STANDOFF_M - COAST_CLEAR_INSIDE_M + 1e-9
    if coast_sigma > 0.0:
        assert clear > clear_distance(STANDOFF_M, 0.4)
    for r10 in range(int(clear * 10) + 1, 400):
        r = r10 / 10.0
        for closing10 in range(0, 81, 5):
            closing = closing10 / 10.0
            (vn, ve), _side, _limited = keep_clear(
                rel_ne=(r, 0.0),
                subject_v_ne=(0.0, 0.0),
                pace_v_ne=(0.0, 0.0),
                want_ne=(8.0, 0.0),
                command_v_ne=(closing, 0.0),
                range_sigma_m=0.4,
                standoff_m=STANDOFF_M,
                side=0,
                coast_sigma_m=coast_sigma,
                vehicle_v_ne=(closing, 0.0),
            )
            speed = math.hypot(vn, ve)
            if vn <= 0.0 or vn > closing + 1e-9 or r * abs(ve) / speed >= clear - 1e-6:
                continue
            stop = vn * SEPARATION_LAG_S + vn * vn / (2.0 * SEPARATION_DECEL_MPS2)
            assert stop <= r - clear + 1e-6, (r, closing, vn, stop, clear)


def test_the_guard_sees_the_vehicle_running_ahead_of_its_command() -> None:
    # The command already slowed to 1 m/s, but the vehicle, 0.8 s behind it,
    # still closes at 3 m/s: the guard cuts on what the vehicle flies.
    kw: dict[str, Any] = {
        "rel_ne": (6.0, 0.0),
        "subject_v_ne": (0.0, 0.0),
        "pace_v_ne": (0.0, 0.0),
        "want_ne": (1.0, 0.0),
        "command_v_ne": (1.0, 0.0),
        "range_sigma_m": 0.0,
        "standoff_m": STANDOFF_M,
        "side": 0,
    }
    _v, _s, on_command = keep_clear(**kw)
    (vn, _ve), _s, on_vehicle = keep_clear(**kw, vehicle_v_ne=(3.0, 0.0))
    assert not on_command
    assert on_vehicle and vn < 1.0


async def _least(mp: Any, case: Any, seeds: int) -> float:
    return min([await case(mp, s) for s in range(1, seeds + 1)])


# (case, seeds, least true separation asserted, m)
_FLIGHTS = {
    "false relock 4 m short, young track": (lambda mp, s: sc.young_coast(mp, s, True, False), 200, 5.0),
    "coast on a young track": (lambda mp, s: sc.young_coast(mp, s, False, False), 200, 5.0),
    "false boxes 8 m beyond, 3 Hz": (lambda mp, s: sc.false_box_beyond(mp, s, 3.0, False), 40, 3.5),
    "false boxes 8 m beyond, 6 Hz": (lambda mp, s: sc.false_box_beyond(mp, s, 6.0, False), 40, 3.5),
    "false boxes 8 m beyond, 10 Hz": (lambda mp, s: sc.false_box_beyond(mp, s, 10.0, False), 40, 3.5),
    "lead, jogger, 3 Hz, 2.9 s unsure": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=3.0, speed=2.5, spell_s=2.9, angle_deg=0.0, brake=False),
        40,
        3.0,
    ),
    "lead, jogger, 10 Hz, 2.9 s unsure": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=10.0, speed=2.5, spell_s=2.9, angle_deg=0.0, brake=False),
        40,
        3.0,
    ),
    "lead, walker, 3 Hz, 2.9 s unsure": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=3.0, speed=1.4, spell_s=2.9, angle_deg=0.0, brake=False),
        40,
        3.0,
    ),
    "side, jogger, 3 Hz, 2.9 s unsure": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=3.0, speed=2.5, spell_s=2.9, angle_deg=90.0, brake=False),
        40,
        3.0,
    ),
    "lead, walker, 10 Hz, 5 s unsure (lost)": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=10.0, speed=1.4, spell_s=5.0, angle_deg=0.0, brake=False),
        40,
        3.0,
    ),
    "lead, jogger, 10 Hz, 5 s unsure (lost)": (
        lambda mp, s: sc.coast_ahead(mp, s, hz=10.0, speed=2.5, spell_s=5.0, angle_deg=0.0, brake=False),
        40,
        3.0,
    ),
}


@pytest.mark.asyncio
@pytest.mark.parametrize("name", list(_FLIGHTS))
async def test_the_follows_own_guard_keeps_the_subject_clear(monkeypatch: Any, name: str) -> None:
    case, seeds, bound = _FLIGHTS[name]
    least = await _least(monkeypatch, case, seeds)
    print(f"{name}: least separation {least:.2f} m over {seeds} seeds (asserted {bound} m)")
    assert least >= bound


@pytest.mark.asyncio
async def test_with_the_pilots_brake_too_the_young_track_keeps_its_margin(monkeypatch: Any) -> None:
    least = await _least(monkeypatch, lambda mp, s: sc.young_coast(mp, s, True, True), 50)
    assert least >= 5.0


@pytest.mark.asyncio
@pytest.mark.parametrize("hz", [10.0, 6.0])
async def test_a_walker_turning_back_passes_under_the_rotors(monkeypatch: Any, hz: float) -> None:
    # A walker 6 m ahead turns back at 1.5 m/s² to 1.4 m/s toward the
    # vehicle. No follow at a 6 m standoff keeps the horizontal range open
    # against it: the reversal shows in the fixes half a second to a second
    # in, the pilot answers 0.8 s later and needs a second more to reverse,
    # and the walker closes 2.8 m/s meanwhile. What keeps the walker safe is
    # the height: the vehicle never follows below the 3.4 m floor, the
    # rotors a metre over a 2 m walker's head (test_separation). The
    # horizontal least is printed, not held to a bound.
    from follow_me.state import MIN_FOLLOW_HEIGHT_M

    least = 99.0
    for seed in range(1, 11):
        f = await fly(
            monkeypatch,
            subject=sc._turning_back,
            start=(4.0, 0.0),
            duration_s=20.0,
            fix_hz=hz,
            seed=seed,
            box_jitter=sc.BOX_JITTER,
            outlier_share=0.0,
            profile=SPORT,
            lag_s=sc.LAG_S,
            config={"follow_height_m": MIN_FOLLOW_HEIGHT_M},
        )
        idx = f.after(9.0)
        least = min(least, min(f.range(i) for i in idx))
        assert min(f.height[i] for i in idx) >= MIN_FOLLOW_HEIGHT_M - 0.1
    print(f"{hz:g} Hz: least horizontal separation {least:.2f} m")


@pytest.mark.asyncio
async def test_no_goal_lies_nearer_the_subject_than_the_standoff_less_a_metre(monkeypatch: Any) -> None:
    for seed in range(1, 21):
        f = await fly(
            monkeypatch,
            subject=lambda t: (10.0 + 1.4 * t, 0.0),
            start=(0.0, 0.0),
            duration_s=12.0,
            fix_hz=10.0,
            seed=seed,
            box_jitter=sc.BOX_JITTER,
            outlier_share=0.0,
            profile=SPORT,
            lag_s=sc.LAG_S,
            miss_frames=sc._UNSURE_YOUNG,
            range_errors={15: -4.0},
        )
        for g in f.goals:
            if g.get("kind") == "follow":
                gap = math.hypot(g["n"] - g["subject_ne"][0], g["e"] - g["subject_ne"][1])
                assert gap >= STANDOFF_M - 1.0 - 1e-6
