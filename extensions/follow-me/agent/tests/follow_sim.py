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
# The Sport profile's goal limits.
SPORT_ACCEL_MPS2 = 2.5
SPORT_JERK_MPS3 = 5.0
SHAPER_YAW_DPS = 30.0
_FC_POS_GAIN = 1.0
# The time constant the pilot smooths an asked velocity on before shaping it.
ASK_SMOOTH_S = 0.15
# How the flight controller flies a velocity the pilot steps (a safety cut):
# ArduCopter's guided defaults WP_ACC 2.5 m/s² and PSC_NE_JERK 5 m/s³.
FC_ACCEL_MPS2 = 2.5
FC_JERK_MPS3 = 5.0
# Its height loop on a position target: PSC_POSZ_P 1, at most 2.5 m/s up
# and 1.5 m/s down (WPNAV_SPEED_UP/DN), 1.0 m/s² (WPNAV_ACCEL_Z).
_FC_Z_GAIN = 1.0
_FC_UP_MPS = 2.5
_FC_DOWN_MPS = 1.5
_FC_Z_ACCEL_MPS2 = 1.0
_FLOOR_AGL_M = 0.5
RANGE_SIGMA_M = 0.75
OUTLIER_SHARE = 0.05
_CLAIMED_SIGMA_M = 0.4
_HEIGHT_M = 4.0

# The subject's body: a cylinder this wide and tall, and the vehicle's
# outermost extent (the pilot's default body radius).
BODY_RADIUS_M = 0.4
BODY_HEIGHT_M = 2.0
VEHICLE_RADIUS_M = 0.4

# The pilot's subject brake (the agent pilot's, modelled): the subject a
# goal names (`subject_ne`) is an obstacle the vehicle keeps
# SUBJECT_MARGIN_M clear of, body to body, plus two sigmas of the goal's
# `subject_sigma_m` (at most SUBJECT_SIGMA_MAX_M). The velocity toward it
# is held to what still stops outside that distance, after the FC's lag, at
# the profile's stick acceleration; inside, it backs away at
# SUBJECT_PUSH_PER_S per metre, never slower than the subject comes on.
# Within SUBJECT_CLIMB_ZONE_M of that distance the vehicle climbs to
# SUBJECT_CLEAR_M over the body (its height floor), and holds it until the
# subject is SUBJECT_CLIMB_HYST_M further off.
SUBJECT_MARGIN_M = 0.5
SUBJECT_SIGMAS = 2.0
SUBJECT_SIGMA_MAX_M = 1.5
SUBJECT_PUSH_PER_S = 1.0
SUBJECT_CLIMB_ZONE_M = 3.0
SUBJECT_CLIMB_HYST_M = 1.0
SUBJECT_CLEAR_M = 1.0
SUBJECT_VEL_SMOOTH_S = 0.3
STICK_ACCEL_MPS2 = 3.0
BRAKE_LAG_S = 0.8
_CREEP_SLACK_M = 0.1 * BRAKE_LAG_S

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
    # The vehicle's height above the (flat) ground, and whether the pilot's
    # subject brake cut the velocity that tick.
    height: list[float] = field(default_factory=list)
    braked: list[bool] = field(default_factory=list)
    # The follow's published state each tick.
    state: list[dict[str, Any]] = field(default_factory=list)

    def speed(self, i: int) -> float:
        return math.hypot(*self.vel[i])

    def range(self, i: int) -> float:
        p, s = self.pos[i], self.subject[i]
        return math.hypot(p[0] - s[0], p[1] - s[1])

    def clearance(self, i: int) -> float:
        """From the vehicle's outermost extent to the subject's body (a
        BODY_RADIUS_M by BODY_HEIGHT_M cylinder on the ground), in 3-D."""
        out = max(0.0, self.range(i) - BODY_RADIUS_M)
        up = max(0.0, self.height[i] - BODY_HEIGHT_M)
        return math.hypot(out, up) - VEHICLE_RADIUS_M

    def closing(self, i: int) -> float:
        """The vehicle's own speed toward the subject (+ closing)."""
        p, s, v = self.pos[i], self.subject[i], self.vel[i]
        r = self.range(i)
        return 0.0 if r < 1e-6 else ((s[0] - p[0]) * v[0] + (s[1] - p[1]) * v[1]) / r

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


def _stop_cap(d: float, closing: float, a: float, keep: float) -> float:
    """The pilot's stop cap: the speed toward a point ``d`` away that still
    stops ``keep`` short of it when the FC answers after BRAKE_LAG_S."""
    t = BRAKE_LAG_S
    room = max(0.0, d - keep - _CREEP_SLACK_M)
    reacting = -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * room)
    coasting = math.sqrt(2.0 * a * max(0.0, room - closing * t))
    return min(reacting, coasting)


class _Pilot:
    """The stand-in for the pilot's shaper and the flight controller."""

    def __init__(
        self,
        pos: Point,
        yaw: float,
        *,
        height: float = _HEIGHT_M,
        accel: float = SHAPER_ACCEL_MPS2,
        jerk: float = SHAPER_JERK_MPS3,
        subject_brake: bool = False,
    ) -> None:
        self.pos = pos
        self.vel: Point = (0.0, 0.0)
        self.acc: Point = (0.0, 0.0)
        self.yaw = yaw
        self.ask: Point = (0.0, 0.0)
        self.z = height
        self.vz = 0.0
        self.accel = accel
        self.jerk = jerk
        self.subject_brake = subject_brake
        self.braked = False
        # The subject as the goals name it, when, its velocity from them,
        # and whether the vehicle is climbing clear of it.
        self._subject: Point | None = None
        self._subject_t = 0.0
        self._subject_v: Point = (0.0, 0.0)
        self._climbing = False
        self.t = 0.0

    def _brake(self, goal: dict[str, Any], want: Point) -> tuple[Point, float | None]:
        """The subject brake on ``want``: the velocity it lets through and,
        while climbing clear, the height floor."""
        s = goal.get("subject_ne")
        if s is None:
            self._subject = None
            return want, None
        s = (float(s[0]), float(s[1]))
        if self._subject is not None and 0.0 < self.t - self._subject_t <= 0.5:
            dt = self.t - self._subject_t
            if dt >= 1e-3:
                k = min(1.0, dt / SUBJECT_VEL_SMOOTH_S)
                raw = ((s[0] - self._subject[0]) / dt, (s[1] - self._subject[1]) / dt)
                sv = self._subject_v
                self._subject_v = (sv[0] + (raw[0] - sv[0]) * k, sv[1] + (raw[1] - sv[1]) * k)
        else:
            self._subject_v = (0.0, 0.0)
        self._subject, self._subject_t = s, self.t
        sigma = float(goal.get("subject_sigma_m") or 0.0)
        keep = (
            BODY_RADIUS_M
            + VEHICLE_RADIUS_M
            + SUBJECT_MARGIN_M
            + min(SUBJECT_SIGMA_MAX_M, SUBJECT_SIGMAS * max(0.0, sigma))
        )
        rel = (s[0] - self.pos[0], s[1] - self.pos[1])
        d = math.hypot(*rel)
        if d > 1e-6:
            u = (rel[0] / d, rel[1] / d)
        else:
            n = math.hypot(*self.vel)
            u = (self.vel[0] / n, self.vel[1] / n) if n > 1e-6 else (1.0, 0.0)
        approach = max(0.0, -(self._subject_v[0] * u[0] + self._subject_v[1] * u[1]))
        if d > keep:
            closing = self.vel[0] * u[0] + self.vel[1] * u[1] + approach
            cap = _stop_cap(d, closing, STICK_ACCEL_MPS2, keep) - approach
        else:
            cap = -approach - SUBJECT_PUSH_PER_S * (keep - d)
        toward = want[0] * u[0] + want[1] * u[1]
        if toward > cap:
            want = (want[0] - (toward - cap) * u[0], want[1] - (toward - cap) * u[1])
            self.braked = True
        if d < keep + SUBJECT_CLIMB_ZONE_M:
            self._climbing = True
        elif d > keep + SUBJECT_CLIMB_ZONE_M + SUBJECT_CLIMB_HYST_M:
            self._climbing = False
        floor = BODY_HEIGHT_M + VEHICLE_RADIUS_M + SUBJECT_CLEAR_M
        return want, floor if self._climbing else None

    def fly(self, goal: dict[str, Any] | None) -> None:
        self.braked = False
        z_target = self.z
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
            if goal.get("d") is not None:
                z_target = max(_FLOOR_AGL_M, -float(goal["d"]))
        k = 1.0 - math.exp(-DT / ASK_SMOOTH_S)
        self.ask = (self.ask[0] + (want[0] - self.ask[0]) * k, self.ask[1] + (want[1] - self.ask[1]) * k)
        want = self.ask
        accel, jerk = self.accel, self.jerk
        if goal is not None and self.subject_brake:
            want, floor = self._brake(goal, want)
            if floor is not None:
                z_target = max(z_target, floor)
            if self.braked:
                # A safety cut: the FC flies the cut velocity at its own
                # limits, and the shaper restarts from it.
                accel, jerk = FC_ACCEL_MPS2, FC_JERK_MPS3
                self.ask = want
        h = DT / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            # The acceleration that brings the velocity onto its target with
            # the jerk limit, never past it.
            dv = (want[0] - self.vel[0], want[1] - self.vel[1])
            e = math.hypot(*dv)
            a_mag = min(accel, math.sqrt(2.0 * jerk * e))
            a_want = (dv[0] / e * a_mag, dv[1] / e * a_mag) if e > 1e-9 else (0.0, 0.0)
            self.acc = _toward(self.acc, a_want, jerk * h)
            self.vel = (self.vel[0] + self.acc[0] * h, self.vel[1] + self.acc[1] * h)
            self.pos = (self.pos[0] + self.vel[0] * h, self.pos[1] + self.vel[1] * h)
            vz = max(-_FC_DOWN_MPS, min(_FC_UP_MPS, _FC_Z_GAIN * (z_target - self.z)))
            step = _FC_Z_ACCEL_MPS2 * h
            self.vz += max(-step, min(step, vz - self.vz))
            self.z += self.vz * h
        self.t += DT
        if goal is not None and goal.get("yaw") is not None:
            err = (float(goal["yaw"]) - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            step = math.radians(SHAPER_YAW_DPS) * DT
            self.yaw += max(-step, min(step, err))


def _fix(
    rnd: random.Random, drone: Point, subject: Point, sigma: float, outliers: float, height: float
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
        "depression_rad": math.atan2(height, r_meas),
        "cues": (),
        "agl_m": height,
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
    accel_mps2: float = SHAPER_ACCEL_MPS2,
    jerk_mps3: float = SHAPER_JERK_MPS3,
    subject_brake: bool = False,
) -> Flight:
    """Fly the plugin for ``duration_s`` from ``start``, facing the subject,
    at the follow height. ``miss`` is the share of batches that miss the
    subject at random; ``miss_frames`` the batch numbers (from 1) that miss
    it besides. ``accel_mps2``/``jerk_mps3`` are the pilot profile's goal
    limits (Normal by default); ``subject_brake`` flies the pilot's subject
    brake."""
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
    pilot = _Pilot(
        start,
        math.atan2(s0[1] - start[1], s0[0] - start[0]),
        height=float(cfg["follow_height_m"]),
        accel=accel_mps2,
        jerk=jerk_mps3,
        subject_brake=subject_brake,
    )
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
            _local_position(pilot.pos[0], pilot.pos[1], -pilot.z, pilot.vel[0], pilot.vel[1])
        )
        plugin._on_global_position(_global_position(12.0, 77.0, pilot.z))
        plugin._on_distance_sensor(_distance_sensor(pilot.z))
        _arm_guided(plugin)
        here = subject(t)
        if t + 1e-9 >= next_fix:
            next_fix += 1.0 / fix_hz
            frame += 1
            missed = frame > 1 and (rnd.random() < miss or frame in miss_frames)
            if not missed:
                pending[:] = [_fix(rnd, pilot.pos, here, range_sigma_m, outlier_share, pilot.z)]
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
        flight.height.append(pilot.z)
        flight.braked.append(pilot.braked)
        flight.state.append(states[-1] if states else {})
        clock.advance(DT)
        t = clock.now - t0
    return flight
