"""A closed-loop flight of the whole plugin: its goals flown by a stand-in for
the pilot, its fixes drawn from a scripted subject with recorded noise.

* **Pilot.** A goal with ``vel_mode`` ``command`` is flown at its velocity; any
  other goal at its velocity plus the position error (the flight
  controller's position gain of 1), capped at the goal's speed limit. The
  velocity is smoothed on 0.15 s against tracker noise and then shaped as the
  pilot's Normal profile shapes it: acceleration at most 1.0 m/s², jerk at
  most 2.0 m/s³ (Sport: 2.5 m/s² and 5.0 m/s³), by a square-root law so the
  shaped velocity never overshoots its target. The yaw turns toward the
  goal's at no more than 30°/s. The height is held on the goal's by the
  flight controller's height loop, measured by a rangefinder. On request the
  pilot's subject brake cuts the shaped velocity, flown by the FC at its own
  limits.
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


@dataclass(frozen=True)
class Profile:
    """A pilot profile: the goal shaper's acceleration and jerk, and the
    stick acceleration its subject brake stops at (`limits(p).a_h`)."""

    accel_mps2: float
    jerk_mps3: float
    stick_accel_mps2: float


NORMAL = Profile(accel_mps2=1.0, jerk_mps3=2.0, stick_accel_mps2=3.0)
SPORT = Profile(accel_mps2=2.5, jerk_mps3=5.0, stick_accel_mps2=6.0)
SHAPER_YAW_DPS = 30.0
_FC_POS_GAIN = 1.0
# How the flight controller flies a velocity the pilot steps (a safety
# cut): ArduCopter's guided defaults WPNAV_ACCEL 2.5 m/s² and PSC_NE_JERK
# 5 m/s³.
FC_ACCEL_MPS2 = 2.5
FC_JERK_MPS3 = 5.0
# The time constant the pilot smooths an asked velocity on before shaping it.
ASK_SMOOTH_S = 0.15
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

# The pilot's subject brake (`subject_guard.rs`): the subject a goal names
# is kept SUBJECT_MARGIN_M clear body to body, plus two sigmas of the
# goal's `subject_sigma_m` (at most SUBJECT_SIGMA_MAX_M). The subject's
# velocity is differenced from successive goals and smoothed on
# SUBJECT_VEL_SMOOTH_S; `s_u` is its part along the line of sight (positive
# away). Outside the kept distance the velocity toward the subject is held
# to `s_u` plus what still stops outside it, closing at the vehicle's own
# speed less `s_u`, after the FC's BRAKE_LAG_S; inside, to `min(s_u, 0)`
# less SUBJECT_PUSH_PER_S per metre inside. Two tiers, as the obstacle
# brake's: a soft one on the asked velocity before the shaper, stopping
# SUBJECT_SOFT_AIM_M outside the kept distance at the profile's goal
# acceleration (flown by the shaper, no step), and a hard one after it,
# stopping at the kept distance at the profile's stick acceleration and
# flown by the FC as a step; outside the kept distance the hard one acts
# only within SUBJECT_HARD_ZONE_M of it (the push inside the kept distance
# is hard only). A cut keeps the sideways part within the speed cap. No cut for
# SUBJECT_ARM_S after the subject is first named or its velocity restarts
# (a gap of more than _SUBJECT_VEL_GAP_S). (Its climb clear flies the body
# at 1 m below the rotors: the follow height floor already holds that, so
# it is not modelled.)
SUBJECT_MARGIN_M = 0.5
SUBJECT_SIGMAS = 2.0
SUBJECT_SIGMA_MAX_M = 1.5
SUBJECT_PUSH_PER_S = 1.0
SUBJECT_VEL_SMOOTH_S = 0.5
SUBJECT_ARM_S = 3.0
SUBJECT_HARD_ZONE_M = 1.0
SUBJECT_SOFT_AIM_M = 0.4
_SUBJECT_VEL_GAP_S = 0.5
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
    # The vehicle's height above the (flat) ground.
    height: list[float] = field(default_factory=list)
    # Whether the pilot's subject brake cut the velocity that tick.
    braked: list[bool] = field(default_factory=list)
    # The follow's published state each tick.
    state: list[dict[str, Any]] = field(default_factory=list)

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


def _stop_cap(d: float, closing: float, a: float, keep: float) -> float:
    """The pilot's `stop_cap`: the speed toward a point ``d`` away that
    still stops ``keep`` short of it when the FC answers after BRAKE_LAG_S."""
    t = BRAKE_LAG_S
    room = max(0.0, d - keep - _CREEP_SLACK_M)
    reacting = -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * room)
    coasting = math.sqrt(2.0 * a * max(0.0, room - closing * t))
    return min(reacting, coasting)


class _Pilot:
    """The stand-in for the pilot's shaper and the flight controller."""

    def __init__(
        self, pos: Point, yaw: float, *, height: float, profile: Profile, subject_brake: bool
    ) -> None:
        self.pos = pos
        self.vel: Point = (0.0, 0.0)
        self.acc: Point = (0.0, 0.0)
        self.yaw = yaw
        self.ask: Point = (0.0, 0.0)
        self.z = height
        self.vz = 0.0
        self.profile = profile
        self.subject_brake = subject_brake
        self.braked = False
        self.t = 0.0
        # The subject as the goals name it, when, its velocity, and since
        # when that velocity has run without a restart.
        self._subject: tuple[Point, float] | None = None
        self._subject_v: Point = (0.0, 0.0)
        self._subject_since = 0.0

    def _track_subject(self, goal: dict[str, Any]) -> tuple[Point, float, float, Point] | None:
        """The armed subject this tick: the unit vector toward it, its
        distance, the distance kept and its velocity; ``None`` when no
        subject is named or the brake is not yet armed."""
        s = goal.get("subject_ne")
        if s is None:
            self._subject = None
            return None
        ne = (float(s[0]), float(s[1]))
        if self._subject is not None and 0.0 < self.t - self._subject[1] <= _SUBJECT_VEL_GAP_S:
            prev, at = self._subject
            dt = self.t - at
            raw = ((ne[0] - prev[0]) / dt, (ne[1] - prev[1]) / dt)
            k = min(1.0, dt / SUBJECT_VEL_SMOOTH_S)
            sv = self._subject_v
            self._subject_v = (sv[0] + (raw[0] - sv[0]) * k, sv[1] + (raw[1] - sv[1]) * k)
        elif self._subject is None or self.t - self._subject[1] > _SUBJECT_VEL_GAP_S:
            self._subject_v = (0.0, 0.0)
            self._subject_since = self.t
        self._subject = (ne, self.t)
        if self.t - self._subject_since < SUBJECT_ARM_S:
            return None
        sigma = max(0.0, float(goal.get("subject_sigma_m") or 0.0))
        keep = BODY_RADIUS_M + VEHICLE_RADIUS_M + SUBJECT_MARGIN_M + min(SUBJECT_SIGMA_MAX_M, SUBJECT_SIGMAS * sigma)
        rel = (ne[0] - self.pos[0], ne[1] - self.pos[1])
        d = math.hypot(*rel)
        speed = math.hypot(*self.vel)
        if d > 1e-6:
            u = (rel[0] / d, rel[1] / d)
        elif speed > 1e-6:
            u = (self.vel[0] / speed, self.vel[1] / speed)
        else:
            u = (1.0, 0.0)
        return u, d, keep, self._subject_v

    def _subject_cap(self, u: Point, d: float, keep: float, sv: Point, a: float) -> float:
        """The most velocity toward the subject the brake lets through,
        stopping at ``a`` outside the kept distance."""
        s_u = sv[0] * u[0] + sv[1] * u[1]
        if d > keep:
            closing = max(0.0, self.vel[0] * u[0] + self.vel[1] * u[1] - s_u)
            return s_u + _stop_cap(d, closing, a, keep)
        return min(s_u, 0.0) - SUBJECT_PUSH_PER_S * (keep - d)

    @staticmethod
    def _cut(ask: Point, u: Point, cap: float, v_cap: float) -> Point:
        """``ask`` with its motion toward the subject cut to ``cap``, the
        sideways part kept within ``v_cap``."""
        toward = ask[0] * u[0] + ask[1] * u[1]
        along = min(toward, cap, v_cap)
        side = (ask[0] - u[0] * toward, ask[1] - u[1] * toward)
        side = _clamp(side, math.sqrt(max(0.0, v_cap * v_cap - along * along)))
        return (u[0] * along + side[0], u[1] * along + side[1])

    def fly(self, goal: dict[str, Any] | None) -> None:
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
        accel, jerk = self.profile.accel_mps2, self.profile.jerk_mps3
        self.braked = False
        armed = self._track_subject(goal) if goal is not None and self.subject_brake else None
        if armed is not None:
            u, d, keep, sv = armed
            v_cap = float(goal["max_speed_mps"])  # type: ignore[index]
            if d > keep:
                # Soft: the shaper flies the cut at the profile's goal
                # acceleration.
                s_u = sv[0] * u[0] + sv[1] * u[1]
                closing = max(0.0, self.vel[0] * u[0] + self.vel[1] * u[1] - s_u)
                soft = s_u + _stop_cap(d, closing, self.profile.accel_mps2, keep + SUBJECT_SOFT_AIM_M)
                if want[0] * u[0] + want[1] * u[1] > soft:
                    want = self._cut(want, u, soft, v_cap)
            hard = self._subject_cap(u, d, keep, sv, self.profile.stick_accel_mps2)
            near = d <= keep or d < keep + SUBJECT_HARD_ZONE_M
            if near and (want[0] * u[0] + want[1] * u[1] > hard or self.vel[0] * u[0] + self.vel[1] * u[1] > hard):
                # Hard: a safety cut the FC flies at its own limits, and the
                # shaper restarts from it.
                want = self.ask = self._cut(want, u, hard, v_cap)
                accel, jerk = FC_ACCEL_MPS2, FC_JERK_MPS3
                self.braked = True
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
    profile: Profile = NORMAL,
    subject_brake: bool = False,
) -> Flight:
    """Fly the plugin for ``duration_s`` from ``start``, facing the subject,
    at the follow height. ``miss`` is the share of batches that miss the
    subject at random; ``miss_frames`` the batch numbers (from 1) that miss
    it besides. ``profile`` is the pilot's (Normal by default);
    ``subject_brake`` flies the pilot's subject brake on the goals."""
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
        profile=profile,
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
