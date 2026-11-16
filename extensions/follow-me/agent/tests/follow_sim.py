"""A closed-loop flight of the whole plugin: its goals flown by a stand-in for
the pilot, its fixes drawn from a scripted subject with recorded noise.

* **Pilot.** A goal with ``vel_mode`` ``command`` is flown at its velocity; any
  other goal at its velocity plus the position error (the flight
  controller's position gain of 1), capped at the goal's speed limit. The
  velocity is smoothed on 0.15 s against tracker noise and then shaped as the
  pilot's Normal profile shapes it: acceleration at most 1.0 m/s², jerk at
  most 2.0 m/s³, by a square-root law so the shaped velocity never
  overshoots its target. The yaw turns toward the goal's at no more than
  30°/s.
* **Subject.** A scripted path. Each fix is the true offset plus a range
  error along the line of sight (sigma 0.75 m, the median 0.72 m step
  jitter a recorded forest follow measured with a standing subject) and,
  on 5 % of fixes, an outlier 2-6 m long or short (that run's 95th
  percentile was 3.5 m). The fix claims the 0.4 m sigma the range cues
  claim.
* **Detector.** Batches at ``fix_hz``; a fraction ``miss`` of them carry
  the subject as ``uncertain`` with no fresh box (the engine's tracker on a
  missed frame).

Pure and seeded: the same arguments fly the same flight.
"""

from __future__ import annotations

import dataclasses
import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch
from follow_fakes import (
    _arm_guided,
    _attitude,
    _Ctx,
    _distance_sensor,
    _global_position,
    _install_clock,
    _local_position,
    _make_plugin,
    _state_events,
)

from follow_me import projection

DT = 0.05
_SUBSTEPS = 10
SHAPER_ACCEL_MPS2 = 1.0
SHAPER_JERK_MPS3 = 2.0
SHAPER_YAW_DPS = 30.0
_FC_POS_GAIN = 1.0
# The time constant the pilot smooths an asked velocity on before shaping it.
ASK_SMOOTH_S = 0.15
RANGE_SIGMA_M = 0.75
OUTLIER_SHARE = 0.05
_CLAIMED_SIGMA_M = 0.4
_HEIGHT_M = 4.0

Point = tuple[float, float]


@dataclass
class Flight:
    """Per-tick samples of one flight."""

    t: list[float] = field(default_factory=list)
    pos: list[Point] = field(default_factory=list)
    vel: list[Point] = field(default_factory=list)
    subject: list[Point] = field(default_factory=list)
    goals: list[dict[str, Any]] = field(default_factory=list)
    phases: list[str] = field(default_factory=list)
    # Whether the plugin took the subject as standing (False before it had
    # a standing-subject mode).
    still: list[bool] = field(default_factory=list)

    def speed(self, i: int) -> float:
        return math.hypot(*self.vel[i])

    def range(self, i: int) -> float:
        p, s = self.pos[i], self.subject[i]
        return math.hypot(p[0] - s[0], p[1] - s[1])

    def after(self, t0: float) -> range:
        start = next(i for i, t in enumerate(self.t) if t >= t0)
        return range(start, len(self.t))


def percentile(values: list[float], q: float) -> float:
    s = sorted(values)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def _toward(v: Point, want: Point, step: float) -> Point:
    dn, de = want[0] - v[0], want[1] - v[1]
    d = math.hypot(dn, de)
    if d <= step:
        return want
    return (v[0] + dn / d * step, v[1] + de / d * step)


def _clamp(v: Point, limit: float) -> Point:
    n = math.hypot(*v)
    return v if n <= limit or n <= 1e-9 else (v[0] * limit / n, v[1] * limit / n)


class _Pilot:
    """The stand-in for the pilot's shaper and the flight controller."""

    def __init__(self, pos: Point, yaw: float) -> None:
        self.pos = pos
        self.vel: Point = (0.0, 0.0)
        self.acc: Point = (0.0, 0.0)
        self.yaw = yaw
        self.ask: Point = (0.0, 0.0)

    def fly(self, goal: dict[str, Any] | None) -> None:
        if goal is None:
            want: Point = (0.0, 0.0)
            limit = 0.0
        else:
            limit = float(goal["max_speed_mps"])
            v_goal = (float(goal["vn"]), float(goal["ve"]))
            if goal.get("vel_mode") == "command":
                want = v_goal
            else:
                to = (float(goal["n"]) - self.pos[0], float(goal["e"]) - self.pos[1])
                want = (v_goal[0] + _FC_POS_GAIN * to[0], v_goal[1] + _FC_POS_GAIN * to[1])
            want = _clamp(want, limit)
        k = 1.0 - math.exp(-DT / ASK_SMOOTH_S)
        self.ask = (self.ask[0] + (want[0] - self.ask[0]) * k, self.ask[1] + (want[1] - self.ask[1]) * k)
        want = self.ask
        h = DT / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            # The acceleration that brings the velocity onto its target with
            # the jerk limit, never past it.
            dv = (want[0] - self.vel[0], want[1] - self.vel[1])
            e = math.hypot(*dv)
            a_mag = min(SHAPER_ACCEL_MPS2, math.sqrt(2.0 * SHAPER_JERK_MPS3 * e))
            a_want = (dv[0] / e * a_mag, dv[1] / e * a_mag) if e > 1e-9 else (0.0, 0.0)
            self.acc = _toward(self.acc, a_want, SHAPER_JERK_MPS3 * h)
            self.vel = (self.vel[0] + self.acc[0] * h, self.vel[1] + self.acc[1] * h)
            self.pos = (self.pos[0] + self.vel[0] * h, self.pos[1] + self.vel[1] * h)
        if goal is not None and goal.get("yaw") is not None:
            err = (float(goal["yaw"]) - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            step = math.radians(SHAPER_YAW_DPS) * DT
            self.yaw += max(-step, min(step, err))


def _fix(
    rnd: random.Random, drone: Point, subject: Point, sigma: float, outliers: float
) -> projection.TargetFix:
    dn, de = subject[0] - drone[0], subject[1] - drone[1]
    r = math.hypot(dn, de)
    b = math.atan2(de, dn)
    err = rnd.gauss(0.0, sigma)
    if rnd.random() < outliers:
        err += rnd.choice((-1.0, 1.0)) * rnd.uniform(2.0, 6.0)
    r_meas = max(0.5, r + err)
    b_meas = b + rnd.gauss(0.0, 0.01)
    values = {
        "offset_north_m": r_meas * math.cos(b_meas),
        "offset_east_m": r_meas * math.sin(b_meas),
        "range_m": r_meas,
        "range_sigma_m": _CLAIMED_SIGMA_M,
        "bearing_rad": b_meas,
        "depression_rad": math.atan2(_HEIGHT_M, r_meas),
        "cues": (),
        "agl_m": _HEIGHT_M,
        "agl_source": "rangefinder",
        "cross_sigma_m": 0.2,
    }
    names = {f.name for f in dataclasses.fields(projection.TargetFix)}
    return projection.TargetFix(**{k: v for k, v in values.items() if k in names})


async def fly(
    monkeypatch: Any,
    *,
    subject: Callable[[float], Point],
    start: Point,
    duration_s: float,
    config: dict[str, Any] | None = None,
    fix_hz: float = 3.0,
    miss: float = 0.0,
    seed: int = 1,
    range_sigma_m: float = RANGE_SIGMA_M,
    outlier_share: float = OUTLIER_SHARE,
    miss_frames: tuple[int, ...] = (),
) -> Flight:
    """Fly the plugin for ``duration_s`` from ``start``, facing the subject.
    ``miss`` is the share of batches that miss the subject at random;
    ``miss_frames`` the batch numbers (from 1) that miss it besides."""
    rnd = random.Random(seed)
    clock = _install_clock(monkeypatch)
    cfg = {
        "active": True,
        "gimbal_point": False,
        "designate_camera": "uvc-0",
        "follow_distance_m": 6.0,
        "follow_height_m": _HEIGHT_M,
        "max_speed_mps": 6.0,
    }
    cfg.update(config or {})
    ctx = _Ctx(config=cfg)
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    s0 = subject(0.0)
    pilot = _Pilot(start, math.atan2(s0[1] - start[1], s0[0] - start[0]))
    pending: list[projection.TargetFix] = []

    def measure(*_a: Any, **_k: Any) -> projection.TargetFix | None:
        return pending.pop() if pending else None

    monkeypatch.setattr(plugin, "_measure_box", measure)
    flight = Flight()
    t0 = clock.now
    next_fix = 0.0
    frame = 0
    sent = 0
    t = 0.0
    while t < duration_s:
        plugin._on_attitude(_attitude(0.0, 0.0, pilot.yaw))
        plugin._on_local_position(
            _local_position(pilot.pos[0], pilot.pos[1], -_HEIGHT_M, pilot.vel[0], pilot.vel[1])
        )
        plugin._on_global_position(_global_position(12.0, 77.0, _HEIGHT_M))
        plugin._on_distance_sensor(_distance_sensor(_HEIGHT_M))
        _arm_guided(plugin)
        here = subject(t)
        if t + 1e-9 >= next_fix:
            next_fix += 1.0 / fix_hz
            frame += 1
            missed = frame > 1 and (rnd.random() < miss or frame in miss_frames)
            if not missed:
                pending[:] = [_fix(rnd, pilot.pos, here, range_sigma_m, outlier_share)]
            plugin._on_batch(
                DetectionBatch(
                    model_id="coco-person",
                    camera_id="uvc-0",
                    frame_id=frame,
                    ts_ms=0,
                    frame_width=640,
                    frame_height=480,
                    detections=[
                        Detection(
                            # A fresh box object per sighting: the loop
                            # measures each new one once.
                            bbox=BoundingBox(x=300.0, y=200.0, width=30.0, height=80.0),
                            class_label="person",
                            confidence=0.9,
                            track_id=7,
                            lock_state="uncertain" if missed else "locked",
                        )
                    ],
                )
            )
        await plugin._tick()
        goal = ctx.flight.setpoints[-1] if len(ctx.flight.setpoints) > sent else None
        sent = len(ctx.flight.setpoints)
        pilot.fly(goal)
        states = _state_events(ctx)
        flight.t.append(t)
        flight.pos.append(pilot.pos)
        flight.vel.append(pilot.vel)
        flight.subject.append(subject(t + DT))
        flight.goals.append(goal or {})
        flight.phases.append(states[-1]["phase"] if states else "")
        still = getattr(plugin, "_still", None)
        flight.still.append(bool(getattr(still, "still", False)))
        clock.advance(DT)
        t = clock.now - t0
    return flight
