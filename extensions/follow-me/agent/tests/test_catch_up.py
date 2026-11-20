"""Engaging a walker from behind on a camera's range.

A rendered forest walk engaged a walker 10 m off at a 6 m set distance: the
follow asked 0.4 m/s 0.6 s after the designation and 0.8 m/s 1.1 s after it
(its command slewed at 1 m/s²), the walker opened the range, and the
detector, which loses a walker beyond about 10.5 m there, lost it. Here the
range is a monocular camera's: read from the walker's box height, the box
5 % off (1 sigma; that walk's detector boxes were 4.6-4.9 % off from frame
to frame), so the range error grows with the range. The pilot flies at the
flight controller's guided limits (2.5 and 3 m/s²) and answers 0.8 s late.
"""

from __future__ import annotations

import math
import random
import statistics
from typing import Any

import pytest
from follow_sim import NORMAL, SPORT, Flight, Profile, fly, percentile

from follow_me import projection
from follow_me.loop import fix_covariance
from follow_me.projection import CameraIntrinsics, TargetFix
from follow_me.target_filter import TargetFilter

BOX_JITTER = 0.05
LAG_S = 0.8
GUIDED = {"2.5": SPORT, "3.0": Profile(accel_mps2=3.0, jerk_mps3=6.0, stick_accel_mps2=6.0)}
WALK_MPS = 1.4
STANDOFF_M = 6.0
BAND_M = 2.0


def _walker(t: float) -> tuple[float, float]:
    return (10.0 + WALK_MPS * t, 0.0)


_FRAME_WH = (1280, 720)
_INTR = CameraIntrinsics.from_fov(*_FRAME_WH, 87.0)


def _box_fix(rnd: random.Random, r: float, height: float) -> TargetFix:
    """The fix the box-height cue alone gives (the feet hidden) for a
    subject ``r`` away seen from ``height`` up, its box BOX_JITTER off."""
    depression = math.atan2(height, r)
    box_h = _INTR.fy * projection.SUBJECT_HEIGHT_M / math.hypot(r, height) * (1.0 + rnd.gauss(0.0, BOX_JITTER))
    cue = projection.box_height_cue(
        bbox_x=600.0,
        bbox_y=200.0,
        bbox_w=0.4 * box_h,
        bbox_h=box_h,
        frame_width=_FRAME_WH[0],
        frame_height=_FRAME_WH[1],
        intr=_INTR,
        depression_rad=depression,
    )
    assert cue is not None
    return TargetFix(
        offset_north_m=cue.range_m,
        offset_east_m=0.0,
        range_m=cue.range_m,
        range_sigma_m=cue.sigma_m,
        range_noise_m=cue.noise_m,
        bearing_rad=0.0,
        depression_rad=depression,
        cues=(cue,),
        agl_m=height,
        agl_source=projection.AGL_RNG,
        cross_sigma_m=0.2,
    )


def _velocity_after(seed: int, seconds: float, height: float = 5.0) -> float:
    """The filter's speed estimate of a walker going straight away from a
    hovering vehicle, 10 m off at the first fix, after ``seconds`` of 10 Hz
    locked fixes ranged from its box height."""
    rnd = random.Random(seed)
    f = TargetFilter()
    f.max_speed_mps = 5.0
    for i in range(int(round(seconds * 10.0)) + 1):
        t = i / 10.0
        fix = _box_fix(rnd, 10.0 + WALK_MPS * t, height)
        f.update(t, fix.offset_north_m, 0.0, fix_covariance(fix))
    est = f.last_estimate()
    assert est is not None
    return est[2]


def test_a_walkers_velocity_is_known_within_a_second_of_locked_fixes() -> None:
    # Straight away from the vehicle, the direction monocular range sees
    # worst. A second of 10 Hz fixes at 5 % box jitter fixes a walk's speed
    # to about 0.55 m/s (1 sigma), a straight line fitted through them as
    # well: the estimate is that good, and centred on the walk.
    for seconds, mean_off, spread, sd_max in ((1.0, 0.15, 0.9, 0.6), (2.0, 0.1, 0.4, 0.3)):
        v = [_velocity_after(seed, seconds) for seed in range(200)]
        mean, sd = statistics.fmean(v), statistics.pstdev(v)
        p10, p90 = percentile(v, 0.1), percentile(v, 0.9)
        print(f"{seconds:g} s: mean {mean:.2f} sd {sd:.2f} p10 {p10:.2f} p90 {p90:.2f}")
        assert abs(mean - WALK_MPS) <= mean_off
        assert sd <= sd_max
        assert WALK_MPS - spread <= p10 and p90 <= WALK_MPS + spread


def _relative_reversals(f: Flight, idx: range) -> int:
    """Reversals of the vehicle's speed along the line of sight relative to
    the walker's (beyond 0.3 m/s either way): overtaking, then dropping back."""
    flips, sign = 0, 0.0
    for i in idx:
        dn, de = f.subject[i][0] - f.pos[i][0], f.subject[i][1] - f.pos[i][1]
        x = (f.vel[i][0] * dn + f.vel[i][1] * de) / max(1e-6, math.hypot(dn, de)) - WALK_MPS
        if abs(x) > 0.3:
            if sign and math.copysign(1.0, x) != sign:
                flips += 1
            sign = math.copysign(1.0, x)
    return flips


@pytest.mark.asyncio
@pytest.mark.parametrize("accel", sorted(GUIDED))
async def test_a_walker_engaged_4_m_short_is_closed_on_within_5_s(monkeypatch: Any, accel: str) -> None:
    # 10 m behind a 1.4 m/s walker at a 6 m set distance, the agent's 10 Hz
    # detector missing 30 % of its frames, 5 % range outliers: inside the
    # ±2 m band within 5 s of the first goal, never past 4 m, and held there
    # without surging and dropping back.
    bands = []
    for seed in range(1, 9):
        f = await fly(
            monkeypatch,
            subject=_walker,
            start=(0.0, 0.0),
            duration_s=25.0,
            fix_hz=10.0,
            miss=0.3,
            seed=seed,
            box_jitter=BOX_JITTER,
            profile=GUIDED[accel],
            lag_s=LAG_S,
            subject_brake=True,
        )
        ranges = [f.range(i) for i in range(len(f.t))]
        first = next(i for i, g in enumerate(f.goals) if g.get("kind") == "follow")
        inside = next(i for i, r in enumerate(ranges) if r <= STANDOFF_M + BAND_M)
        t_band = f.t[inside] - f.t[first]
        held = [ranges[i] for i in range(inside, len(ranges))]
        cmd = [math.hypot(g.get("vn", 0.0), g.get("ve", 0.0)) for g in f.goals]
        t_fast = next(f.t[i] for i in range(first, len(cmd)) if cmd[i] >= 2.0) - f.t[first]
        flips = _relative_reversals(f, range(inside, len(ranges)))
        print(
            f"{accel} m/s² seed {seed}: in the band {t_band:.2f} s, asked 2 m/s at {t_fast:.2f} s,"
            f" least {min(ranges):.2f} m, held {min(held):.2f}-{max(held):.2f} m, reversals {flips}"
        )
        bands.append(t_band)
        # The follow is not the slow link: it asks for 2 m/s within 1.5 s
        # (slewed at 1 m/s² it could not before 2 s).
        assert t_fast <= 1.5
        assert t_band <= 5.0
        assert min(ranges) >= STANDOFF_M - BAND_M
        assert all(abs(r - STANDOFF_M) <= BAND_M for r in held)
        assert flips <= 1
    assert statistics.fmean(bands) <= 4.6


@pytest.mark.asyncio
@pytest.mark.parametrize("profile", ["normal", "guided"])
async def test_a_walker_who_speeds_up_stays_inside_the_detectors_reach(monkeypatch: Any, profile: str) -> None:
    # Followed at 6 m, the walker speeds up from 1.4 to 2.2 m/s over 15 s:
    # the range stays under 9 m, short of the 10.5 m past which the forest
    # walk's detector lost the walker, on the Normal profile too.
    def walker(t: float) -> tuple[float, float]:
        return (10.0 + WALK_MPS * t + 0.8 * max(0.0, min(t, 30.0) - 15.0), 0.0)

    for seed in range(1, 7):
        f = await fly(
            monkeypatch,
            subject=walker,
            start=(4.0, 0.0),
            duration_s=40.0,
            fix_hz=10.0,
            miss=0.3,
            seed=seed,
            box_jitter=BOX_JITTER,
            profile=NORMAL if profile == "normal" else SPORT,
            lag_s=LAG_S,
            subject_brake=True,
        )
        ranges = [f.range(i) for i in f.after(15.0)]
        print(f"{profile} seed {seed}: range {min(ranges):.2f}-{max(ranges):.2f} m")
        assert max(ranges) <= 9.0
        assert sum(abs(r - STANDOFF_M) <= BAND_M for r in ranges) / len(ranges) >= 0.9

