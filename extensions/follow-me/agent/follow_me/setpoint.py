"""Standoff follow setpoint geometry in the local NED frame.

Given the vehicle position, its current height above ground and the filtered
subject position and velocity, this computes the position, velocity
feed-forward and yaw the vehicle should be commanded to:

* **Standoff.** ``target - follow_distance_m * u``. While the subject moves
  faster than 0.5 m/s, ``u`` is its direction of travel, so the vehicle trails
  behind it on its path; slower than that, ``u`` points from the vehicle to the
  subject, so a standing subject is held at range along the current line of
  sight without the vehicle circling.
* **Velocity feed-forward.** The subject's velocity, its magnitude clamped to
  ``max_speed_mps`` and scaled by ``ff_scale`` (1 while following on a live
  lock, at most 0.5 while coasting on a prediction). The flight controller's
  position loop corrects the residual error; the feed-forward removes the lag
  a position-only setpoint has on a moving subject.
* **Yaw.** The bearing from the vehicle to the subject, so a forward camera
  keeps it framed.
* **Height.** Terrain-following: ``z_sp = z_now + (agl_now - follow_height)``,
  so the vehicle holds ``follow_height_m`` above the ground under it rather
  than above the take-off point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Below this subject speed the standoff direction is the line of sight.
_MOVING_SPEED_MPS = 0.5


@dataclass(frozen=True)
class LocalSetpoint:
    north_m: float
    east_m: float
    down_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    yaw_rad: float


def _unit(n: float, e: float) -> tuple[float, float] | None:
    norm = math.hypot(n, e)
    if norm <= 1e-6:
        return None
    return (n / norm, e / norm)


def compute_setpoint(
    *,
    drone_north_m: float,
    drone_east_m: float,
    drone_down_m: float,
    drone_yaw_rad: float,
    agl_m: float,
    target_north_m: float,
    target_east_m: float,
    target_vn_mps: float,
    target_ve_mps: float,
    follow_distance_m: float,
    follow_height_m: float,
    max_speed_mps: float,
    ff_scale: float = 1.0,
) -> LocalSetpoint:
    speed = math.hypot(target_vn_mps, target_ve_mps)
    los = _unit(target_north_m - drone_north_m, target_east_m - drone_east_m)
    if speed > _MOVING_SPEED_MPS:
        u = (target_vn_mps / speed, target_ve_mps / speed)
    elif los is not None:
        u = los
    else:
        # The subject is directly under the vehicle and still: back off along
        # the current heading's reverse rather than pick an arbitrary axis.
        u = (math.cos(drone_yaw_rad), math.sin(drone_yaw_rad))
    set_n = target_north_m - follow_distance_m * u[0]
    set_e = target_east_m - follow_distance_m * u[1]

    scale = max(0.0, min(1.0, ff_scale))
    limit = max(0.0, max_speed_mps)
    k = scale * (min(1.0, limit / speed) if speed > 1e-6 else 0.0)
    vn = target_vn_mps * k
    ve = target_ve_mps * k

    yaw = drone_yaw_rad if los is None else math.atan2(los[1], los[0])
    down = drone_down_m + (agl_m - max(0.0, follow_height_m))
    return LocalSetpoint(
        north_m=set_n,
        east_m=set_e,
        down_m=down,
        vn_mps=vn,
        ve_mps=ve,
        vd_mps=0.0,
        yaw_rad=yaw,
    )


def hold_setpoint(
    *, north_m: float, east_m: float, down_m: float, yaw_rad: float
) -> LocalSetpoint:
    """Zero-velocity hold at a position: what every hold sends so the vehicle
    stops where it is instead of flying on along the last feed-forward."""
    return LocalSetpoint(
        north_m=north_m,
        east_m=east_m,
        down_m=down_m,
        vn_mps=0.0,
        ve_mps=0.0,
        vd_mps=0.0,
        yaw_rad=yaw_rad,
    )


__all__ = ["LocalSetpoint", "compute_setpoint", "hold_setpoint"]
