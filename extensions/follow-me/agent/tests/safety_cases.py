"""Person-safety flights of the whole plugin (see :mod:`follow_sim`): the
cases a false lock, a coast or the subject's own manoeuvre could bring the
vehicle onto the subject in. Each returns the least true horizontal
separation flown (from ``after_s`` on).

Every case ranges the subject from its box height (5 % off), flies the pilot
at the flight controller's guided 2.5 m/s² answering 0.8 s late, and takes
whether the pilot's own subject brake flies on the goals (``brake``): off,
the follow's own guard is all there is.
"""

from __future__ import annotations

import math
from typing import Any

from follow_sim import SPORT, SUBJECT_JUMP_MIN_S, SUBJECT_JUMP_MPS, Flight, fly

BOX_JITTER = 0.05
LAG_S = 0.8
STANDOFF_M = 6.0


def least_range(f: Flight, after_s: float = 0.0) -> float:
    return min(f.range(i) for i in f.after(after_s))


def _frames(hz: float, t0: float, seconds: float) -> tuple[int, ...]:
    """The batch numbers (frame k at (k - 1) / hz) from ``t0`` for
    ``seconds``."""
    k0 = int(round(t0 * hz)) + 1
    return tuple(range(k0, k0 + int(round(seconds * hz))))


async def false_box_beyond(mp: Any, seed: int, hz: float, brake: bool) -> float:
    # Followed 6 m behind a 1.4 m/s walker for 10 s, then unsure for
    # 1.67 s (3 Hz) or 1.9 s, then two boxes locked on something 8 m beyond
    # the walker, then the walker again.
    gap = _frames(hz, 10.0, 1.67 if hz < 4.0 else 1.9)
    after = gap[-1] + 1
    f = await fly(
        mp,
        subject=lambda t: (10.0 + 1.4 * t, 0.0),
        start=(4.0, 0.0),
        duration_s=25.0,
        fix_hz=hz,
        seed=seed,
        box_jitter=BOX_JITTER,
        outlier_share=0.0,
        profile=SPORT,
        lag_s=LAG_S,
        miss_frames=gap,
        range_errors={after: 8.0, after + 1: 8.0},
        subject_brake=brake,
    )
    return least_range(f, 9.0)


def _straight(speed: float) -> Any:
    return lambda t: (speed * t, 0.0)


STILL_S = 30.0
STILL_DRIFT_MPS = 0.45
STILL_SEARCH_S = 60.0


async def still_then_lost(
    mp: Any, seed: int, brake: bool, hz: float = 3.0, toward: bool = False
) -> Flight:
    # A subject standing at the origin, followed from 6 m south, for
    # STILL_S; over its last 3 s the boxes put it drifting away (``toward``:
    # toward the vehicle) at STILL_DRIFT_MPS (range noise the filter reads
    # as a slow walk), then every frame misses it: the lock is lost 3 s on,
    # and the search runs its full 60 s with the vantage points the
    # stand-in pilot picks.
    drift = _frames(hz, STILL_S - 3.0, 3.0)
    sign = -1.0 if toward else 1.0
    errors = {k: sign * STILL_DRIFT_MPS * (i + 1) / hz for i, k in enumerate(drift)}
    return await _lost_after(mp, seed, brake, hz, lambda _t: (0.0, 0.0), errors)


def _stand_then_walk(speed: float, start_s: float, walk_s: float, accel: float = 1.5) -> Any:
    """Standing at the origin until ``start_s``, then walking north at
    ``speed`` (negative: south, toward a vehicle south of it) for
    ``walk_s``, setting off at ``accel``."""
    ramp = abs(speed) / accel
    sign = math.copysign(1.0, speed)

    def at(t: float) -> tuple[float, float]:
        d = min(max(0.0, t - start_s), walk_s)
        if d <= ramp:
            return (sign * 0.5 * accel * d * d, 0.0)
        return (sign * (0.5 * accel * ramp * ramp + abs(speed) * (d - ramp)), 0.0)

    return at


async def stand_then_walk_toward(
    mp: Any, seed: int, brake: bool, hz: float = 3.0, speed: float = 0.5, before_s: float = 2.0
) -> Flight:
    # The subject of :func:`still_then_lost`, but it really sets off toward
    # the vehicle at ``speed`` ``before_s`` before the boxes stop, and walks
    # on 5 m: what the estimate sees reads much like the drift.
    walk = _stand_then_walk(-speed, STILL_S - before_s, 5.0 / speed)
    return await _lost_after(mp, seed, brake, hz, walk, None)


async def _lost_after(
    mp: Any, seed: int, brake: bool, hz: float, subject: Any, errors: dict[int, float] | None
) -> Flight:
    return await fly(
        mp,
        subject=subject,
        start=(-STANDOFF_M, 0.0),
        duration_s=STILL_S + 3.0 + STILL_SEARCH_S,
        config={"search_budget_s": STILL_SEARCH_S},
        fix_hz=hz,
        seed=seed,
        box_jitter=BOX_JITTER,
        profile=SPORT,
        lag_s=LAG_S,
        miss_frames=_frames(hz, STILL_S, STILL_SEARCH_S + 10.0),
        range_errors=errors,
        subject_brake=brake,
        vantage_pilot=True,
    )


def subject_jumps(f: Flight, after_s: float) -> int:
    """How many times the goals from ``after_s`` on name the subject
    somewhere else (a step the pilot's subject brake takes for a new
    subject and re-arms on)."""
    jumps = 0
    prev: tuple[float, list[float]] | None = None
    for i in f.after(after_s):
        s = f.goals[i].get("subject_ne")
        if s is None:
            continue
        if prev is not None:
            dt = max(f.t[i] - prev[0], SUBJECT_JUMP_MIN_S)
            if math.hypot(s[0] - prev[1][0], s[1] - prev[1][1]) > SUBJECT_JUMP_MPS * dt:
                jumps += 1
        prev = (f.t[i], s)
    return jumps


def backing_away_share(f: Flight, after_s: float) -> float:
    """The share of the search's goals (repositioning and searching) that
    back away from the subject rather than ask for a vantage."""
    kinds = [
        f.goals[i].get("kind")
        for i in f.after(after_s)
        if f.phases[i] in ("repositioning", "searching") and f.goals[i]
    ]
    back = sum(k == "position" for k in kinds)
    return back / max(1, len(kinds))


async def coast_ahead(
    mp: Any, seed: int, *, hz: float, speed: float, spell_s: float, angle_deg: float, brake: bool
) -> float:
    # Followed at 6 m ahead (0°) or beside (90°) a subject walking or
    # jogging north, then unsure for ``spell_s`` while it keeps coming.
    rad = math.radians(angle_deg)
    f = await fly(
        mp,
        subject=_straight(speed),
        start=(STANDOFF_M * math.cos(rad), STANDOFF_M * math.sin(rad)),
        duration_s=10.0 + spell_s + 8.0,
        config={"follow_angle_deg": angle_deg},
        fix_hz=hz,
        seed=seed,
        box_jitter=BOX_JITTER,
        outlier_share=0.0,
        profile=SPORT,
        lag_s=LAG_S,
        miss_frames=_frames(hz, 10.0, spell_s),
        subject_brake=brake,
    )
    return least_range(f, 5.0)


def _turning_back(t: float) -> tuple[float, float]:
    # 1.4 m/s away for 10 s, then a 1.5 m/s² reversal to 1.4 m/s back.
    t1 = 10.0
    t2 = t1 + 2.8 / 1.5
    if t <= t1:
        return (10.0 + 1.4 * t, 0.0)
    x1 = 10.0 + 1.4 * t1
    if t <= t2:
        d = t - t1
        return (x1 + 1.4 * d - 0.75 * d * d, 0.0)
    d = t2 - t1
    x2 = x1 + 1.4 * d - 0.75 * d * d
    return (x2 - 1.4 * (t - t2), 0.0)


async def turn_back(mp: Any, seed: int, hz: float, brake: bool) -> float:
    f = await fly(
        mp,
        subject=_turning_back,
        start=(4.0, 0.0),
        duration_s=25.0,
        fix_hz=hz,
        seed=seed,
        box_jitter=BOX_JITTER,
        outlier_share=0.0,
        profile=SPORT,
        lag_s=LAG_S,
        subject_brake=brake,
    )
    return least_range(f, 9.0)


_UNSURE_YOUNG = tuple(range(6, 15)) + tuple(range(17, 47))


async def young_coast(mp: Any, seed: int, false_relock: bool, brake: bool) -> float:
    # Engaged 10 m behind a 1.4 m/s walker: five locked frames at 10 Hz,
    # 0.9 s unsure, a box (4 m short: a false re-lock, or the walker's
    # own), the walker's box, three seconds unsure, then the walker again.
    f = await fly(
        mp,
        subject=lambda t: (10.0 + 1.4 * t, 0.0),
        start=(0.0, 0.0),
        duration_s=12.0,
        fix_hz=10.0,
        seed=seed,
        box_jitter=BOX_JITTER,
        outlier_share=0.0,
        profile=SPORT,
        lag_s=LAG_S,
        miss_frames=_UNSURE_YOUNG if false_relock else _UNSURE_YOUNG + (15,),
        range_errors={15: -4.0} if false_relock else None,
        subject_brake=brake,
    )
    return least_range(f)


async def catch_up(mp: Any, seed: int, accel: float = 2.5) -> tuple[float, float]:
    """10 m behind a 1.4 m/s walker at 6 m, 30 % of frames missed: the time
    to the ±2 m band and the least separation."""
    from follow_sim import Profile

    f = await fly(
        mp,
        subject=lambda t: (10.0 + 1.4 * t, 0.0),
        start=(0.0, 0.0),
        duration_s=20.0,
        fix_hz=10.0,
        miss=0.3,
        seed=seed,
        box_jitter=BOX_JITTER,
        profile=Profile(accel_mps2=accel, jerk_mps3=2.0 * accel, stick_accel_mps2=6.0),
        lag_s=LAG_S,
        subject_brake=True,
    )
    first = next(i for i, g in enumerate(f.goals) if g.get("kind") == "follow")
    inside = next(i for i in range(len(f.t)) if f.range(i) <= STANDOFF_M + 2.0)
    return f.t[inside] - f.t[first], least_range(f)
