"""The pilot motion lease and the operator's stick nudges.

The agent's pilot is the only producer of motion setpoints. A skill that
flies the vehicle holds a motion lease and sends goals; the pilot runs them
through its obstacle planner, speed limits and geofence, and revokes the
lease when the operator pauses, takes the sticks, lands, returns home or the
flight controller leaves its guided mode.

While the lease is held the pilot forwards the operator's shaped sticks
(``flight.sticks``, 20 Hz). They do not move the vehicle; they nudge the
follow parameters:

=======  ==============================  =========================  ====================
mode     right X (roll)                  right Y (pitch)            left Y (throttle)
=======  ==============================  =========================  ====================
follow   ``follow_angle_deg`` ±60 °/s     ``follow_distance_m``       ``follow_height_m``
                                         ±2 m/s                     ±1 m/s
orbit    ``orbit_rate_dps`` ±10 °/s²,     ``orbit_radius_m`` ±2 m/s   ``follow_height_m``
         below the minimum flips                                    ±1 m/s
=======  ==============================  =========================  ====================

Pitch forward (positive) brings the vehicle closer (distance and radius
shrink); throttle up raises it. Every value is clamped to its config range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

from follow_me.state import (
    FOLLOW_DISTANCE_RANGE,
    FOLLOW_HEIGHT_RANGE,
    FOLLOW_MODE_ORBIT,
    ORBIT_CCW,
    ORBIT_CW,
    ORBIT_RADIUS_RANGE,
    ORBIT_RATE_RANGE,
    FollowConfig,
)

SKILL_ID = "follow-me"
STICKS_TOPIC = "flight.sticks"
LEASE_REVOKED_TOPIC = "flight.lease.revoked"

# A refused lease is asked for again at most this often.
LEASE_RETRY_S = 1.0
# Nudged values are written back to the per-drone config at most this often.
NUDGE_WRITE_PERIOD_S = 1.0
# Stick frames further apart than this are treated as a fresh start.
_MAX_STICK_DT_S = 0.2
# Axis magnitudes under this are ignored (the pilot already applies the
# surface deadzone; this only stops float noise writing config).
_NUDGE_DEADBAND = 0.02

ANGLE_RATE_DPS = 60.0
DISTANCE_RATE_MPS = 2.0
HEIGHT_RATE_MPS = 1.0
ORBIT_RATE_ACCEL_DPS2 = 10.0
RADIUS_RATE_MPS = 2.0
ANGLE_STEP_DEG = 5


def _clamp(v: float, lo_hi: tuple[float, float]) -> float:
    lo, hi = lo_hi
    return lo if v < lo else hi if v > hi else v


def lease_error_reason(reply: Any) -> str:
    """The pilot's refusal word from an acquire reply, ``"unavailable"`` when
    the reply carries none."""
    if isinstance(reply, dict):
        err = reply.get("err") or reply.get("error")
        if err:
            return str(err)
    return "unavailable"


@dataclass
class Nudges:
    """Operator nudges accumulated from the pilot's stick stream.

    ``values`` overrides the resolved config until the per-drone config reads
    back the written value, so the loop flies the nudged value at once.
    """

    values: dict[str, object] = field(default_factory=dict)
    dirty: bool = False
    nudged: bool = False
    last_stick_s: float | None = None
    last_write_s: float = -math.inf

    def reset_session(self) -> None:
        self.values.clear()
        self.dirty = False
        self.nudged = False
        self.last_stick_s = None

    def apply_to(self, cfg: FollowConfig) -> FollowConfig:
        if not self.values:
            return cfg
        return replace(cfg, **self.values)  # type: ignore[arg-type]

    def settle(self, cfg_read: FollowConfig) -> None:
        """Drop overrides the config has caught up with, once written."""
        if self.dirty:
            return
        for key in list(self.values):
            have = getattr(cfg_read, key)
            want = self.values[key]
            if isinstance(want, float) and isinstance(have, float):
                if abs(have - want) < 1e-6:
                    del self.values[key]
            elif have == want:
                del self.values[key]

    def on_sticks(self, cfg: FollowConfig, axes: Any, now: float) -> None:
        """Integrate one stick frame against ``cfg`` (already nudged)."""
        try:
            roll, pitch, thr = (float(axes[0]), float(axes[1]), float(axes[2]))
        except (TypeError, ValueError, IndexError):
            return
        if not all(math.isfinite(a) for a in (roll, pitch, thr)):
            return
        prev = self.last_stick_s
        self.last_stick_s = now
        if prev is None:
            return
        dt = now - prev
        if not (0.0 < dt <= _MAX_STICK_DT_S):
            return
        roll = max(-1.0, min(1.0, roll))
        pitch = max(-1.0, min(1.0, pitch))
        thr = max(-1.0, min(1.0, thr))
        changes: dict[str, object] = {}
        if abs(thr) > _NUDGE_DEADBAND:
            changes["follow_height_m"] = _clamp(
                cfg.follow_height_m + thr * HEIGHT_RATE_MPS * dt, FOLLOW_HEIGHT_RANGE
            )
        if cfg.follow_mode == FOLLOW_MODE_ORBIT:
            if abs(pitch) > _NUDGE_DEADBAND:
                changes["orbit_radius_m"] = _clamp(
                    cfg.orbit_radius_m - pitch * RADIUS_RATE_MPS * dt, ORBIT_RADIUS_RANGE
                )
            if abs(roll) > _NUDGE_DEADBAND:
                # Roll + is right. Facing the subject, a clockwise orbit
                # carries the drone to its left, so right stick turns the
                # orbit counter-clockwise.
                rate, direction = nudge_orbit_rate(
                    cfg.orbit_rate_dps, cfg.orbit_direction, -roll * ORBIT_RATE_ACCEL_DPS2 * dt
                )
                changes["orbit_rate_dps"] = rate
                changes["orbit_direction"] = direction
        else:
            if abs(pitch) > _NUDGE_DEADBAND:
                changes["follow_distance_m"] = _clamp(
                    cfg.follow_distance_m - pitch * DISTANCE_RATE_MPS * dt,
                    FOLLOW_DISTANCE_RANGE,
                )
            if abs(roll) > _NUDGE_DEADBAND:
                # Roll + is right. The angle grows clockwise around the
                # subject, which moves a drone facing the subject to its
                # left, so right stick lowers the angle.
                changes["follow_angle_deg"] = (
                    cfg.follow_angle_deg - roll * ANGLE_RATE_DPS * dt
                ) % 360.0
        changed = {k: v for k, v in changes.items() if getattr(cfg, k) != v}
        if changed:
            self.values.update(changed)
            self.dirty = True
            self.nudged = True

    async def write_back(self, ctx: Any, now: float) -> None:
        """Write the nudged values to the per-drone config, ≤ once a second,
        so the surfaces' sliders follow the sticks."""
        if not self.dirty or (now - self.last_write_s) < NUDGE_WRITE_PERIOD_S:
            return
        setter = getattr(ctx.config_kv, "set", None)
        if setter is None:
            return
        self.last_write_s = now
        self.dirty = False
        for key, value in list(self.values.items()):
            if key == "follow_angle_deg" and isinstance(value, (int, float)):
                value = wire_angle(float(value))
            elif isinstance(value, float):
                value = round(value, 2)
            self.values[key] = value
            await setter(key, value)


def wire_angle(deg: float) -> float:
    """A follow angle as the config holds it: a multiple of 5 in 0–355
    (360 wraps to 0)."""
    return float(int(round((deg % 360.0) / ANGLE_STEP_DEG)) * ANGLE_STEP_DEG % 360)


def nudge_orbit_rate(rate_dps: float, direction: str, delta_dps: float) -> tuple[float, str]:
    """Add ``delta_dps`` to the signed orbit rate (clockwise positive).
    Pushing the magnitude below the minimum rate flips the direction and lands
    on the minimum on the other side; magnitudes clamp to the range."""
    lo, _hi = ORBIT_RATE_RANGE
    signed = rate_dps if direction == ORBIT_CW else -rate_dps
    new = signed + delta_dps
    if signed > 0 and new < lo:
        new = -lo
    elif signed < 0 and new > -lo:
        new = lo
    return _clamp(abs(new), ORBIT_RATE_RANGE), ORBIT_CW if new > 0 else ORBIT_CCW


__all__ = [
    "LEASE_RETRY_S",
    "LEASE_REVOKED_TOPIC",
    "NUDGE_WRITE_PERIOD_S",
    "Nudges",
    "SKILL_ID",
    "STICKS_TOPIC",
    "lease_error_reason",
    "nudge_orbit_rate",
]
