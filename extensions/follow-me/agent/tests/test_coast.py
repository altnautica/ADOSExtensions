"""The follow on a coast: no fix for a while (an unsure lock, a lock the
filter held a jumped box out of). The velocity it flies on decays the less
it is known (:meth:`follow_me.target_filter.TargetFilter.coasted`), and the
distance kept clear of the subject widens by the spread the estimate gained
(:func:`follow_me.standoff.clear_distance`), so a velocity a few fixes
measured, flown on for seconds, never carries the vehicle onto the subject.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from follow_sim import SPORT, fly

import follow_me.target_filter as target_filter
from follow_me.standoff import clear_distance, keep_clear
from follow_me.target_filter import COAST_GRACE_S, TargetFilter

_R = ((0.75**2, 0.0), (0.0, 0.2**2))
STANDOFF_M = 6.0
# The least true separation every flight below keeps, set distance 6 m.
LEAST_SEPARATION_M = 3.0
# The pilot's response when the follow cuts its closing speed: the flight
# controller's guided deceleration after its lag.
PILOT_DECEL_MPS2 = 2.5
PILOT_LAG_S = 0.8


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
    print(f"after {late:g} s: {v_long:.2f} m/s on the long track, {v_young:.2f} on five fixes")
    assert young.coast_tau_s() < 0.5 * long_track.coast_tau_s()
    assert 0.0 < v_young < 0.1 * young.last_estimate()[2]  # type: ignore[index]
    assert v_young < v_long < long_track.last_estimate()[2]  # type: ignore[index]
    # The position runs on by the decaying velocity, never past the prediction.
    pred, coast = young.predicted(t_young + late), young.coasted(t_young + late)
    assert pred is not None and coast is not None
    assert young.last_estimate()[0] < coast[0][0] < pred[0][0]  # type: ignore[index]
    assert young.coast_spread_m(t_young + 0.4) == 0.0
    assert young.coast_spread_m(t_young + late) > 1.0


@pytest.mark.parametrize("coast_sigma", [0.0, 0.5, 1.5, 4.0])
def test_no_closing_speed_runs_past_the_widened_clear_distance(coast_sigma: float) -> None:
    # Toward a standing subject, at any range and any speed the command is
    # closing at: a velocity let through either passes the subject outside
    # the clear distance or stops, at the pilot's deceleration after its
    # lag, short of it. The clear distance widens with the coast's spread
    # (never past 1 m inside the standoff).
    clear = clear_distance(STANDOFF_M, 0.4, coast_sigma)
    assert clear <= STANDOFF_M - 1.0 + 1e-9
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
            )
            speed = math.hypot(vn, ve)
            if vn <= 0.0 or r * abs(ve) / speed >= clear - 1e-6:
                continue
            stop = vn * PILOT_LAG_S + vn * vn / (2.0 * PILOT_DECEL_MPS2)
            assert stop <= r - clear + 1e-6, (r, closing, vn, stop, clear)


_UNSURE = tuple(range(6, 15)) + tuple(range(17, 47))


@pytest.mark.asyncio
@pytest.mark.parametrize("false_relock", [True, False], ids=["false relock", "coast only"])
async def test_a_coast_on_a_young_track_never_brings_the_vehicle_onto_the_walker(
    monkeypatch: Any, false_relock: bool
) -> None:
    # Engaged 10 m behind a 1.4 m/s walker at 6 m: five locked frames at
    # 10 Hz, 0.9 s unsure, a box (4 m short of the walker: a false re-lock,
    # or the walker's own), the walker's box, three seconds unsure, then the
    # walker again. Ranged from box height 5 % off, the pilot at the flight
    # controller's guided 2.5 m/s² answering 0.8 s late. Over 200 noise
    # draws the vehicle keeps 3 m from the walker every time, and no goal
    # names a point nearer the subject estimate than the set distance less a
    # metre; flown on the undecayed prediction, one draw put the vehicle
    # 0.3 m from the walker and three under 3 m.
    async def flights() -> list[tuple[float, float]]:
        out = []
        for seed in range(1, 201):
            f = await fly(
                monkeypatch,
                subject=lambda t: (10.0 + 1.4 * t, 0.0),
                start=(0.0, 0.0),
                duration_s=12.0,
                fix_hz=10.0,
                seed=seed,
                box_jitter=0.05,
                outlier_share=0.0,
                profile=SPORT,
                lag_s=0.8,
                miss_frames=_UNSURE if false_relock else _UNSURE + (15,),
                range_errors={15: -4.0} if false_relock else None,
                subject_brake=True,
            )
            goal_gap = min(
                math.hypot(g["n"] - g["subject_ne"][0], g["e"] - g["subject_ne"][1])
                for g in f.goals
                if g.get("kind") == "follow"
            )
            out.append((min(f.range(i) for i in range(len(f.t))), goal_gap))
        return out

    coasted = await flights()
    least = sorted(r for r, _g in coasted)
    print(f"least separation {least[0]:.2f} m, 1st percentile {least[2]:.2f} m, median {least[100]:.2f} m")
    assert least[0] >= LEAST_SEPARATION_M
    assert min(g for _r, g in coasted) >= STANDOFF_M - 1.0
    if false_relock:
        monkeypatch.setattr(target_filter, "COAST_GRACE_S", math.inf)
        flown_on = sorted(r for r, _g in await flights())
        print(f"undecayed: least {flown_on[0]:.2f} m, {sum(r < LEAST_SEPARATION_M for r in flown_on)} under 3 m")
        assert flown_on[0] < LEAST_SEPARATION_M
