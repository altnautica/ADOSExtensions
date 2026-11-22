"""The goal the follow sends the pilot.

A pilot at ``pilot.v1`` version 6 or later plans the view itself: the
follow sends a ``follow_intent`` goal naming the standoff band round its
follow distance, the bearing it prefers from the subject's heading (the
follow angle: 180 behind, 90 on its right, 0 leading; none with the angle
set to auto, for the bearing that sees the subject best), the height band
from its follow height, the subject (its point, its velocity and its id on
``vision.subject.v1``) and that the line of sight matters. The pilot picks
the viewpoint, flies it through its planner and keeps the subject clear.

The follow's own hard guard stays as an independent check on that flight:
whenever the vehicle is inside the guard's clear distance from the
subject's worst case, or closing on it fast enough to be there within the
guard's lag, the goal is the follow's own guarded command (the ring goal,
its velocity backing away) instead of the intent. An orbit is an intent
too, its bearing the orbit's phase over the ground (``bearing_frame``
``world``) as it advances. A pilot older than version 6 gets the ring goal.
"""

from __future__ import annotations

import functools
import math
from typing import Any

from .standoff import SAFE_DISTANCE_M, SEPARATION_LAG_S, clear_distance
from .state import MIN_FOLLOW_HEIGHT_M, FollowConfig

# The pilot.v1 version that takes follow_intent goals.
PILOT_INTENT_VERSION = 6
# The standoff band reaches this far either side of the follow distance, m,
# its near edge at least _BAND_NEAR_EXTRA_M beyond the guard's safe distance.
BAND_HALF_M = 1.0
_BAND_NEAR_EXTRA_M = 1.0
# The height band reaches this far above the follow height, m (not when the
# height is lowered under leaves: the pilot keeps to it there).
HEIGHT_BAND_UP_M = 1.0

VEL_MODE_FEED_FORWARD = "feed_forward"
# Goals in which ``vn``/``ve`` are the follow's complete desired velocity:
# the pilot shapes and caps it and adds no position loop toward ``n``/``e``
# (pilot.v1 ``vel_mode``). A zero velocity stops on the pilot's profile and
# holds where the vehicle stops.
VEL_MODE_COMMAND = "command"
KIND_FOLLOW_INTENT = "follow_intent"
# What an intent's bearing is measured from: the subject's heading, or north.
BEARING_FRAME_SUBJECT = "subject"
BEARING_FRAME_WORLD = "world"


@functools.cache
def pilot_takes_intents() -> bool:
    """Whether the agent's pilot takes ``follow_intent`` goals: its
    registered ``pilot.v1`` version is at least :data:`PILOT_INTENT_VERSION`.
    An agent without the contract registry takes none."""
    try:
        from ados.core.contracts import contract_version
    except ImportError:
        return False
    version = contract_version("pilot.v1")
    return version is not None and version >= PILOT_INTENT_VERSION


def guard_trips(
    *,
    drone_ne: tuple[float, float],
    drone_v_ne: tuple[float, float],
    guard_ne: tuple[float, float],
    guard_v_ne: tuple[float, float],
    standoff_m: float,
    coast_sigma_m: float,
) -> bool:
    """The hard guard's check on the pilot's flight: the vehicle is within
    the clear distance (no range margin, widened on a coast) of the
    subject's worst case ``guard_ne``, moving at ``guard_v_ne``, or closes
    on it fast enough to be within :data:`SEPARATION_LAG_S`."""
    rn, re = guard_ne[0] - drone_ne[0], guard_ne[1] - drone_ne[1]
    r = math.hypot(rn, re)
    clear = clear_distance(standoff_m, 0.0, coast_sigma_m)
    if r <= 1e-6:
        return True
    un, ue = rn / r, re / r
    closing = (drone_v_ne[0] - guard_v_ne[0]) * un + (
        drone_v_ne[1] - guard_v_ne[1]
    ) * ue
    return r - max(0.0, closing) * SEPARATION_LAG_S <= clear


def bearing_pref_deg(cfg: FollowConfig) -> float | None:
    """The bearing the follow prefers from the subject's heading, degrees
    in [-180, 180); ``None`` with the angle set to auto."""
    if cfg.follow_angle_auto:
        return None
    return (cfg.follow_angle_deg + 180.0) % 360.0 - 180.0


def standoff_band_m(standoff_m: float) -> list[float]:
    """The standoff band round the follow distance."""
    near = max(SAFE_DISTANCE_M + _BAND_NEAR_EXTRA_M, standoff_m - BAND_HALF_M)
    return [near, max(near, standoff_m + BAND_HALF_M)]


def height_band_m(cfg: FollowConfig) -> list[float]:
    """The height band from the follow height flown."""
    low = max(MIN_FOLLOW_HEIGHT_M, cfg.follow_height_m)
    if cfg.follow_height_lowered:
        return [low, low]
    return [low, low + HEIGHT_BAND_UP_M]


def intent_goal(
    goal: dict[str, Any],
    *,
    cfg: FollowConfig,
    pace_ne: tuple[float, float],
    standoff_m: float,
    subject_ref: int | None,
) -> dict[str, Any]:
    """The ``follow_intent`` for the follow or orbit goal ``goal``: its
    point, height, heading, subject and margin as they are (the point is
    what the pilot flies when it picks no viewpoint), the subject's
    velocity ``pace_ne`` fed forward, and the bands, bearing and subject
    id. A follow's bearing is the follow angle from the subject's heading;
    an orbit's, its ring's phase over the ground."""
    if goal.get("kind") == "orbit":
        phase = float(goal["candidates"]["angle_deg"])
        bearing: float | None = (phase + 180.0) % 360.0 - 180.0
        frame = BEARING_FRAME_WORLD
    else:
        bearing, frame = bearing_pref_deg(cfg), BEARING_FRAME_SUBJECT
    g = dict(goal)
    g.update(
        kind=KIND_FOLLOW_INTENT,
        vn=pace_ne[0],
        ve=pace_ne[1],
        vel_mode=VEL_MODE_FEED_FORWARD,
        candidates=None,
        standoff_band_m=standoff_band_m(standoff_m),
        bearing_pref_deg=bearing,
        bearing_frame=frame,
        height_band_m=height_band_m(cfg),
        subject_ref=subject_ref,
        require_los=True,
    )
    return g


def ring_goal(
    kind: str,
    ned: tuple[float, float, float],
    cmd: Any,
    cfg: FollowConfig,
    *,
    ring: dict[str, Any],
    subject_ne: tuple[float, float],
    subject_sigma_m: float,
) -> dict[str, Any]:
    """The follow (or orbit) goal flown on the follow's own command
    ``cmd`` (a :class:`follow_me.standoff.Command`) toward ``ned``, with
    the ring of detour spots round the subject."""
    return {
        "kind": kind,
        "n": ned[0],
        "e": ned[1],
        "d": ned[2],
        "vn": cmd.vn_mps,
        "ve": cmd.ve_mps,
        "vd": 0.0,
        "vel_mode": VEL_MODE_COMMAND,
        "yaw": cmd.yaw_rad,
        "yaw_rate": None,
        "subject_ne": [subject_ne[0], subject_ne[1]],
        "subject_sigma_m": subject_sigma_m,
        "max_speed_mps": cfg.max_speed_mps,
        "candidates": ring,
        "margin_m": cfg.obstacle_margin_m,
    }


def follow_goal(
    goal: dict[str, Any],
    *,
    cfg: FollowConfig,
    pace_ne: tuple[float, float],
    standoff_m: float,
    subject_ref: int | None,
    drone: tuple[float, float, float, float],
    guard: tuple[tuple[float, float], tuple[float, float], float],
) -> dict[str, Any]:
    """The goal to send for ``goal`` (:func:`ring_goal`): the intent when
    the pilot takes intents and the hard guard does not trip (``drone`` is
    the vehicle's ``(n, e, vn, ve)``, ``guard`` the subject's worst case,
    its velocity and the coast's spread); else ``goal``."""
    if not pilot_takes_intents():
        return goal
    dn, de, dvn, dve = drone
    guard_ne, guard_v, coast = guard
    if guard_trips(
        drone_ne=(dn, de),
        drone_v_ne=(dvn, dve),
        guard_ne=guard_ne,
        guard_v_ne=guard_v,
        standoff_m=standoff_m,
        coast_sigma_m=coast,
    ):
        return goal
    return intent_goal(
        goal, cfg=cfg, pace_ne=pace_ne, standoff_m=standoff_m, subject_ref=subject_ref
    )


__all__ = [
    "BAND_HALF_M",
    "HEIGHT_BAND_UP_M",
    "KIND_FOLLOW_INTENT",
    "PILOT_INTENT_VERSION",
    "VEL_MODE_COMMAND",
    "bearing_pref_deg",
    "follow_goal",
    "guard_trips",
    "height_band_m",
    "intent_goal",
    "pilot_takes_intents",
    "ring_goal",
    "standoff_band_m",
]
