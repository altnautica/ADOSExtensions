"""A fix after a detection gap that jumped is not taken for the subject's
motion (see :data:`follow_me.target_filter.REACQUIRE_GAP_S`).

The recorded case: a rendered forest walk, a walker going straight away from
a nearly hovering vehicle (it moved under a metre in the 1.6 s below), 8.4 m
off at the designation. The tracker held the walker four frames, then lost
it for 0.95 s and locked onto a tree trunk taller in the frame, then back
onto the walker, then lost it again for three seconds. Each locked box below
is the published one: its frame time after the designation, its height in
pixels and the walker's true range then. It is ranged from its height alone
with the camera's focal length times the walker's height (1300 px m, from
the walker's true range and box), and the false box and the true one after
it also as the follow ranged them with every cue (6.3 and 12.3 m).
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from follow_sim import SPORT, fly

import follow_me.target_filter as target_filter
from follow_me.target_filter import REACQUIRE_GAP_S, TargetFilter

_HEIGHT_M = 5.3
_FOCAL_HEIGHT_PX_M = 1300.0
# (s after the designation, box height px, the walker's true range m)
_LOCKED = (
    (0.08, 120.0, 8.4),
    (0.16, 129.0, 8.6),
    (0.25, 118.0, 8.7),
    (0.50, 114.0, 8.9),
    (1.45, 186.0, 10.3),  # the tree
    (1.55, 96.0, 10.3),
)
_EVERY_CUE = {1.45: 6.3, 1.55: 12.3}
_RANGINGS = {"box height": {}, "every cue": _EVERY_CUE}
_R = ((0.75**2, 0.0), (0.0, 0.2**2))


def _ranged(t: float, h_px: float, ranging: str) -> float:
    if t in _RANGINGS[ranging]:
        return _RANGINGS[ranging][t]
    slant = _FOCAL_HEIGHT_PX_M / h_px
    return math.sqrt(slant * slant - _HEIGHT_M * _HEIGHT_M)


def _fly_recorded(ranging: str) -> tuple[TargetFilter, list[bool]]:
    f = TargetFilter()
    f.max_speed_mps = 5.0
    fused = [f.update(t, _ranged(t, h, ranging), 0.0, _R) for t, h, _true in _LOCKED]
    return f, fused


@pytest.mark.parametrize("ranging", sorted(_RANGINGS))
def test_the_recorded_false_relock_after_a_gap_is_held_out(ranging: str) -> None:
    f, fused = _fly_recorded(ranging)
    # The tree is held out; the walker's own box after it is fused.
    assert fused == [True, True, True, True, False, True]
    est = f.last_estimate()
    assert est is not None
    print(f"{ranging}: estimate {est[0]:.2f} m at {est[2]:.2f} m/s")
    assert est[2] >= 0.0
    assert abs(est[0] - _LOCKED[-1][2]) <= 2.5
    # Three seconds unsure after it: the prediction never brings the walker
    # toward the vehicle (fused, the tree put it 6.3 m off walking in at
    # 2.4 m/s).
    pred = f.predicted(4.5)
    assert pred is not None and pred[0][0] >= _LOCKED[-1][2] - 1.0


@pytest.mark.parametrize("ranging", sorted(_RANGINGS))
def test_without_the_reacquire_rule_the_false_relock_sets_the_velocity(monkeypatch: Any, ranging: str) -> None:
    # The control: fused as any other fix, the tree turns the estimate
    # around toward the vehicle.
    monkeypatch.setattr(target_filter, "REACQUIRE_GAP_S", math.inf)
    f, fused = _fly_recorded(ranging)
    assert fused[4]
    est = f.last_estimate()
    assert est is not None and est[2] < -1.0


def _steady(f: TargetFilter, seconds: float, speed: float = 1.4) -> float:
    """A walker going north at ``speed`` from 10 m, 10 Hz clean fixes; the
    time of the last."""
    t = 0.0
    for i in range(int(round(seconds * 10.0)) + 1):
        t = i / 10.0
        f.update(t, 10.0 + speed * t, 0.0, _R)
    return t


def test_a_jump_the_fixes_after_it_confirm_starts_the_estimate_over() -> None:
    # The subject really is elsewhere (the estimate's velocity, or the
    # track, was wrong): the second jumped fix that agrees with the first
    # starts the estimate over on them, with no velocity carried over.
    f = TargetFilter()
    t0 = _steady(f, 3.0) + REACQUIRE_GAP_S + 0.5
    fused = [f.update(t0 + k / 10.0, 25.0, 4.0, _R) for k in range(3)]
    assert fused == [False, True, True]
    est = f.last_estimate()
    assert est is not None
    assert math.hypot(est[0] - 25.0, est[1] - 4.0) <= 0.5
    assert math.hypot(est[2], est[3]) <= 1.0


def test_a_checked_reacquire_that_jumped_starts_the_estimate_over_on_it() -> None:
    # A recovery's re-acquire (gated=False) passed its own checks: when it
    # jumped, the estimate starts over on it rather than fusing the jump
    # into a velocity.
    f = TargetFilter()
    t = _steady(f, 3.0) + 1.0
    assert f.update(t, 4.0, 3.0, _R, gated=False)
    est = f.last_estimate()
    assert est is not None
    assert (est[0], est[1], est[2], est[3]) == (4.0, 3.0, 0.0, 0.0)


# The recorded boxes as 10 Hz frames (frame k at (k - 1) / 10 s): the range
# error each carried, and every other frame up to 4.6 s unsure.
def _recorded_frames(ranging: str) -> tuple[dict[int, float], tuple[int, ...]]:
    errors = {
        int(round(t * 10.0)) + 1: _ranged(t, h, ranging) - true for t, h, true in _LOCKED
    }
    return errors, tuple(k for k in range(2, 47) if k not in errors)


@pytest.mark.asyncio
@pytest.mark.parametrize("ranging", sorted(_RANGINGS))
async def test_the_recorded_false_relock_does_not_bring_the_goal_onto_the_vehicle(
    monkeypatch: Any, ranging: str
) -> None:
    # The recorded boxes flown closed-loop: engaged 8.4 m behind a 1.4 m/s
    # walker at the 6 m default, each box carrying the range error it did,
    # then from 4.6 s the walker seen again (box height 5 % off). The pilot
    # flies at the flight controller's guided 2.5 m/s² and answers 0.8 s
    # late. Through the three unsure seconds the goal names the walker no
    # more than 1 m nearer than it is and the walker stays inside 10.5 m,
    # the detector's reach there; fused as any other fix, the tree had it
    # 4 m and more nearer, and the vehicle hung back 12-17 m behind.
    # more than 1 m nearer than it is and the walker stays inside 11 m
    # (about the detector's 10.5 m reach there; the coast's guard holds the
    # vehicle a little back while the walker is unseen); fused as any other
    # fix, the tree had it 4 m and more nearer, and the vehicle hung back
    # 12-25 m behind.
    errors, unsure = _recorded_frames(ranging)

    async def flights() -> list[tuple[float, float]]:
        out = []
        for seed in range(1, 7):
            f = await fly(
                monkeypatch,
                subject=lambda t: (8.4 + 1.4 * t, 0.0),
                start=(0.0, 0.0),
                duration_s=12.0,
                fix_hz=10.0,
                seed=seed,
                box_jitter=0.05,
                outlier_share=0.0,
                profile=SPORT,
                lag_s=0.8,
                miss_frames=unsure,
                range_errors=errors,
                subject_brake=True,
            )
            short = []
            for i in f.after(1.5):
                if f.t[i] > 4.6:
                    break
                s = f.goals[i].get("subject_ne")
                if s is not None:
                    short.append(f.range(i) - math.hypot(s[0] - f.pos[i][0], s[1] - f.pos[i][1]))
            out.append((max(short), max(f.range(i) for i in f.after(1.5))))
        return out

    held = await flights()
    monkeypatch.setattr(target_filter, "REACQUIRE_GAP_S", math.inf)
    fused = await flights()
    print(f"{ranging} with the rule: (named short, farthest) {[(round(a, 2), round(b, 2)) for a, b in held]}")
    print(f"{ranging} without it: {[(round(a, 2), round(b, 2)) for a, b in fused]}")
    assert all(short <= 1.0 and far <= 10.5 for short, far in held)
    assert all(short <= 1.0 and far <= 11.0 for short, far in held)
    # The control (the box-height ranging also had the outlier gate catch
    # the tree on some flights).
    assert sum(short >= 4.0 and far >= 12.0 for short, far in fused) >= 3
