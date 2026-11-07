"""Follow and orbit goal geometry in the local NED frame.

The vehicle is flown by the agent's pilot, which owns the obstacle model, the
planner and the speed limits. This module only says where the follow wants
the vehicle to be:

* **Follow.** ``goal = subject + d·(cos θ, sin θ)`` with
  ``θ = ref_heading + follow_angle``. ``ref_heading`` is the subject's
  (slewed) travel heading, so 180° trails it, 90° sits on its right and 0°
  leads it.
* **Orbit.** ``goal = centre + R·(cos φ, sin φ)``; the phase ``φ`` advances
  at the orbit rate (``s = +1`` clockwise, bearing increasing north to east).
* **Velocity feed-forward.** The subject's velocity (plus, for an orbit, the
  ring's tangential velocity), the total clamped to ``max_speed_mps`` and
  scaled by ``ff_scale`` (1 on a live lock, at most 0.5 while coasting).
* **Yaw.** Toward the subject.
* **Height.** Terrain-following: ``d_goal = d_now + (agl_now - follow_height)``,
  so the vehicle holds ``follow_height_m`` above the ground under it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Below this subject speed the follow reference heading is held.
MOVING_SPEED_MPS = 0.5
# The follow reference heading slews toward the travel heading at most this
# fast, so a subject turning on the spot does not swing the vehicle round.
REF_SLEW_DPS = 45.0
# The orbit phase advances only while the vehicle is this close to the ring
# point, so it converges onto the ring before it starts to circle.
ORBIT_CAPTURE_M = 2.0


@dataclass(frozen=True)
class Goal:
    north_m: float
    east_m: float
    vn_mps: float
    ve_mps: float
    yaw_rad: float


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def bearing(from_n: float, from_e: float, to_n: float, to_e: float) -> float | None:
    dn = to_n - from_n
    de = to_e - from_e
    if math.hypot(dn, de) <= 1e-6:
        return None
    return math.atan2(de, dn)


def _clamp_velocity(vn: float, ve: float, max_speed_mps: float, ff_scale: float) -> tuple[float, float]:
    speed = math.hypot(vn, ve)
    scale = max(0.0, min(1.0, ff_scale))
    limit = max(0.0, max_speed_mps)
    k = scale * (min(1.0, limit / speed) if speed > 1e-6 else 0.0)
    return vn * k, ve * k


def _yaw_to(drone_n: float, drone_e: float, to_n: float, to_e: float, fallback: float) -> float:
    b = bearing(drone_n, drone_e, to_n, to_e)
    return fallback if b is None else b


def follow_goal(
    *,
    target_n: float,
    target_e: float,
    target_vn: float,
    target_ve: float,
    ref_heading_rad: float,
    follow_distance_m: float,
    follow_angle_deg: float,
    max_speed_mps: float,
    ff_scale: float,
    drone_n: float,
    drone_e: float,
    drone_yaw_rad: float,
) -> Goal:
    theta = ref_heading_rad + math.radians(follow_angle_deg)
    gn = target_n + follow_distance_m * math.cos(theta)
    ge = target_e + follow_distance_m * math.sin(theta)
    vn, ve = _clamp_velocity(target_vn, target_ve, max_speed_mps, ff_scale)
    return Goal(
        north_m=gn,
        east_m=ge,
        vn_mps=vn,
        ve_mps=ve,
        yaw_rad=_yaw_to(drone_n, drone_e, target_n, target_e, drone_yaw_rad),
    )


def orbit_goal(
    *,
    center_n: float,
    center_e: float,
    center_vn: float,
    center_ve: float,
    phase_rad: float,
    radius_m: float,
    rate_dps: float,
    direction: str,
    max_speed_mps: float,
    ff_scale: float,
) -> Goal:
    s = 1.0 if direction == "cw" else -1.0
    gn = center_n + radius_m * math.cos(phase_rad)
    ge = center_e + radius_m * math.sin(phase_rad)
    omega = math.radians(rate_dps)
    tn = radius_m * omega * s * -math.sin(phase_rad)
    te = radius_m * omega * s * math.cos(phase_rad)
    vn, ve = _clamp_velocity(center_vn + tn, center_ve + te, max_speed_mps, ff_scale)
    # Yaw toward the centre: the reverse of the centre-to-goal bearing.
    return Goal(north_m=gn, east_m=ge, vn_mps=vn, ve_mps=ve, yaw_rad=wrap_pi(phase_rad + math.pi))


def height_down(drone_down_m: float, agl_m: float, follow_height_m: float) -> float:
    return drone_down_m + (agl_m - max(0.0, follow_height_m))


def slew_ref_heading(
    ref_rad: float, target_vn: float, target_ve: float, dt_s: float
) -> float:
    """The follow reference heading after ``dt_s``: toward the subject's
    travel heading at ≤ REF_SLEW_DPS while it moves, held otherwise."""
    if math.hypot(target_vn, target_ve) <= MOVING_SPEED_MPS:
        return ref_rad
    travel = math.atan2(target_ve, target_vn)
    err = wrap_pi(travel - ref_rad)
    step = math.radians(REF_SLEW_DPS) * max(0.0, dt_s)
    return wrap_pi(ref_rad + max(-step, min(step, err)))


def advance_orbit_phase(
    phase_rad: float,
    *,
    center_n: float,
    center_e: float,
    radius_m: float,
    rate_dps: float,
    direction: str,
    drone_n: float,
    drone_e: float,
    dt_s: float,
) -> float:
    """The orbit phase after ``dt_s``: advanced at the orbit rate while the
    vehicle is within ORBIT_CAPTURE_M of the ring point, held otherwise."""
    rn = center_n + radius_m * math.cos(phase_rad)
    re = center_e + radius_m * math.sin(phase_rad)
    if math.hypot(drone_n - rn, drone_e - re) > ORBIT_CAPTURE_M:
        return phase_rad
    s = 1.0 if direction == "cw" else -1.0
    return wrap_pi(phase_rad + s * math.radians(rate_dps) * max(0.0, dt_s))


__all__ = [
    "Goal",
    "MOVING_SPEED_MPS",
    "ORBIT_CAPTURE_M",
    "REF_SLEW_DPS",
    "advance_orbit_phase",
    "bearing",
    "follow_goal",
    "height_down",
    "orbit_goal",
    "slew_ref_heading",
    "wrap_pi",
]
