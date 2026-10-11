"""Flight-controller, pose and gimbal state the follow loop projects through.

Each input is aged rather than latched (see :class:`FreshnessGate`): the
loop holds when the pose, the HEARTBEAT or the gimbal report stops arriving.
:class:`FcStateMixin` holds the MAVLink handlers that fill this state; it is a
mixin of :class:`follow_me.FollowMePlugin`.
"""

from __future__ import annotations

import time
from typing import Any

from follow_me import mavlink_frames
from follow_me.freshness import FreshnessGate

# Freshness windows for the flight-controller telemetry the loop projects
# through. Every one of these inputs can stop arriving while the write path
# still works — a stream-rate change, a lagged broadcast receiver, a router
# reconnect, an FC that simply stops emitting one message — so each is aged
# rather than latched, and each gets a window sized from the cadence that
# message is actually expected at. One shared window would be wrong for all
# three: they are requested at 10 Hz, 5 Hz and 1 Hz respectively.
#
# Pose. The agent's MAVLink router requests ATTITUDE at 10 Hz and
# GLOBAL_POSITION_INT at 5 Hz, so the slower of the two nominally arrives
# every 0.2 s. 0.5 s is three follow-loop periods (the loop runs at 6 Hz)
# and two and a half position periods: one dropped message plus link jitter
# still reads current, while a real stall is caught within about three
# ticks. Deliberately much tighter than the 1.5 s detection coast window,
# because a frozen pose corrupts the ORIGIN and heading the projection
# measures from, which grows the setpoint error without bound as the
# aircraft flies on. A frozen bounding box only misplaces the subject
# within the frame, and the loop already holds its last setpoint for that.
_POSE_MAX_AGE_S = 0.5

# Flight-controller arm + mode. HEARTBEAT is requested at 1 Hz, so a window
# sized like the pose one would report stale between every message. Three
# seconds is three missed heartbeats — the same "the link is gone" rule of
# thumb ground stations apply to a MAVLink system.
_FC_MAX_AGE_S = 3.0

# Gimbal boresight. MOUNT_ORIENTATION is NOT one of the streams the router
# requests, so its cadence is whatever the firmware happens to emit; this
# window is correspondingly generous. Expiring here is benign rather than a
# stop: the projection falls back to the angle the plugin last commanded the
# gimbal to, which is a better estimate than an attitude report old enough
# to predate the current aim.
_GIMBAL_MAX_AGE_S = 2.0

# HEARTBEAT decode. MAV_MODE_FLAG_SAFETY_ARMED is bit 7 of base_mode.
_BASE_MODE_ARMED = 0x80
# MAV_AUTOPILOT_INVALID: the heartbeat of a component that is not a flight
# controller (a GCS, a gimbal, a companion). Only the autopilot's heartbeat
# says whether the vehicle is armed and in a guided mode.
_AP_INVALID = 8
# MAV_AUTOPILOT ids we can decode a guided/offboard mode for.
_AP_ARDUPILOTMEGA = 3
_AP_PX4 = 12
# ArduPilot custom_mode numbers are per vehicle firmware, so the guided
# check picks the table from the heartbeat's MAV_TYPE. Copter: GUIDED (4)
# and GUIDED_NOGPS (20). Plane (incl. QuadPlane): GUIDED (15); Plane's 4 is
# ACRO and 20 is QLAND. Rover/boat: GUIDED (15).
_ARDUCOPTER_GUIDED_MODES = frozenset({4, 20})
_ARDUPLANE_GUIDED_MODES = frozenset({15})
_ARDUROVER_GUIDED_MODES = frozenset({15})
# MAV_TYPE groups. Multirotor/heli: quadrotor, coaxial, helicopter,
# hexarotor, octorotor, tricopter, dodecarotor, decarotor.
_MAV_TYPES_COPTER = frozenset({2, 3, 4, 13, 14, 15, 29, 35})
# Fixed wing and the VTOL types ArduPlane reports for a QuadPlane.
_MAV_TYPES_PLANE = frozenset({1, 19, 20, 21, 22, 23, 24, 25})
# Ground rover and surface boat.
_MAV_TYPES_ROVER = frozenset({10, 11})
# PX4 packs the main mode into bits 16-23 of custom_mode; OFFBOARD is 6.
_PX4_MAIN_MODE_OFFBOARD = 6


def monotonic() -> float:
    """The one clock every freshness mark and window check in this plugin
    reads. Routed through a single function so the whole staleness surface
    can be driven from a test without sleeping through real windows."""
    return time.monotonic()


def is_guided_mode(autopilot: int, custom_mode: int, mav_type: int) -> bool:
    """Whether ``custom_mode`` is a guided/offboard mode for this vehicle.

    Firmware- and vehicle-aware: ArduPilot picks the Copter, Plane or Rover
    mode table from ``mav_type``; PX4 decodes OFFBOARD from its main mode.
    An autopilot or vehicle class we do not decode returns False so the
    follow loop reports ``fc_guided`` honestly rather than claiming to
    command an FC that would ignore the setpoints.
    """
    if autopilot == _AP_PX4:
        return ((custom_mode >> 16) & 0xFF) == _PX4_MAIN_MODE_OFFBOARD
    if autopilot == _AP_ARDUPILOTMEGA:
        if mav_type in _MAV_TYPES_COPTER:
            return custom_mode in _ARDUCOPTER_GUIDED_MODES
        if mav_type in _MAV_TYPES_PLANE:
            return custom_mode in _ARDUPLANE_GUIDED_MODES
        if mav_type in _MAV_TYPES_ROVER:
            return custom_mode in _ARDUROVER_GUIDED_MODES
    return False


class Pose:
    """Latest FC pose cached from the MAVLink telemetry stream.

    Attitude and position arrive on two different messages, at two different
    rates, and can stop independently — an FC can keep emitting ATTITUDE long
    after GLOBAL_POSITION_INT dries up. They therefore carry two separate
    freshness gates rather than one shared mark, which whichever message was
    still flowing would keep refreshing, hiding the partial stall entirely.
    """

    __slots__ = (
        "roll",
        "pitch",
        "yaw",
        "lat_deg",
        "lon_deg",
        "rel_alt_m",
        "attitude",
        "position",
    )

    def __init__(self, max_age_s: float = _POSE_MAX_AGE_S) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        self.attitude = FreshnessGate(max_age_s)
        self.position = FreshnessGate(max_age_s)

    def ready(self, now_monotonic_s: float) -> bool:
        """Whether BOTH halves of the pose are current enough to project
        through. Not "have we ever had a pose" — a pose the aircraft has
        since flown away from is worse than no pose, because it looks
        usable."""
        return self.attitude.is_fresh(now_monotonic_s) and self.position.is_fresh(
            now_monotonic_s
        )


class FcState:
    """Latest flight-controller arm + mode cached from HEARTBEAT.

    ``guided`` is True only when the autopilot is positively identified
    (ArduPilot or PX4) AND reports a mode that accepts companion position
    setpoints. An unknown autopilot or a non-guided mode leaves it False so
    the follow loop never claims to command an FC that would ignore it.

    Both bits are readings, not facts: they describe the FC as of the last
    HEARTBEAT. Once that heartbeat ages out they are no longer evidence of
    anything, so the accessors below report False rather than continuing to
    assert an arm state nothing has confirmed for seconds.
    """

    __slots__ = ("armed", "guided", "heartbeat")

    def __init__(self, max_age_s: float = _FC_MAX_AGE_S) -> None:
        self.armed = False
        self.guided = False
        self.heartbeat = FreshnessGate(max_age_s)

    def is_fresh(self, now_monotonic_s: float) -> bool:
        return self.heartbeat.is_fresh(now_monotonic_s)

    def armed_at(self, now_monotonic_s: float) -> bool:
        return self.armed and self.heartbeat.is_fresh(now_monotonic_s)

    def guided_at(self, now_monotonic_s: float) -> bool:
        return self.guided and self.heartbeat.is_fresh(now_monotonic_s)


class GimbalState:
    """Gimbal boresight attitude reported by the FC, in the PROJECTION's
    convention (pitch degrees positive = down, yaw degrees positive = right of
    the nose). MOUNT_ORIENTATION reports pitch negative = down, so the handler
    negates it on the way in.

    The report is only the boresight while it is current; a report old enough
    to predate the gimbal's current aim is a worse estimate than the angle the
    plugin itself last commanded. The gate therefore decays back to that
    fallback (and then to the fixed mount) instead of latching."""

    __slots__ = ("pitch_deg", "yaw_deg", "report")

    def __init__(self, max_age_s: float = _GIMBAL_MAX_AGE_S) -> None:
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.report = FreshnessGate(max_age_s)


class FcStateMixin:
    """The MAVLink handlers that keep the cached FC, pose and gimbal state."""

    _pose: Pose
    _fc: FcState
    _gimbal: GimbalState

    def _on_heartbeat(self, delivery: dict[str, Any]) -> None:
        msg = mavlink_frames.decode_fields(delivery)
        if msg is None or int(msg.get("autopilot", _AP_INVALID)) == _AP_INVALID:
            return
        base_mode = int(msg.get("base_mode", 0) or 0)
        custom_mode = int(msg.get("custom_mode", 0) or 0)
        autopilot = int(msg["autopilot"])
        mav_type = int(msg.get("type", 0) or 0)
        self._fc.armed = bool(base_mode & _BASE_MODE_ARMED)
        self._fc.guided = is_guided_mode(autopilot, custom_mode, mav_type)
        self._fc.heartbeat.mark(monotonic())

    def _on_mount_orientation(self, delivery: dict[str, Any]) -> None:
        # MOUNT_ORIENTATION carries degrees: pitch negative = down, yaw
        # relative to the vehicle heading (positive to the right). The
        # projection wants pitch positive = down, so negate it; the yaw
        # convention already matches.
        msg = mavlink_frames.decode_fields(delivery)
        if msg is None:
            return
        self._gimbal.pitch_deg = -float(msg["pitch"])
        self._gimbal.yaw_deg = float(msg["yaw"])
        self._gimbal.report.mark(monotonic())

    def _on_attitude(self, delivery: dict[str, Any]) -> None:
        msg = mavlink_frames.decode_fields(delivery)
        if msg is None:
            return
        self._pose.roll = float(msg["roll"])
        self._pose.pitch = float(msg["pitch"])
        self._pose.yaw = float(msg["yaw"])
        # Marked separately from position: ATTITUDE and GLOBAL_POSITION_INT
        # are different messages at different rates and either can stall on
        # its own.
        self._pose.attitude.mark(monotonic())

    def _on_global_position(self, delivery: dict[str, Any]) -> None:
        # lat/lon are 1e7 integer degrees; relative_alt is millimetres.
        msg = mavlink_frames.decode_fields(delivery)
        if msg is None:
            return
        self._pose.lat_deg = float(msg["lat"]) / 1e7
        self._pose.lon_deg = float(msg["lon"]) / 1e7
        self._pose.rel_alt_m = float(msg["relative_alt"]) / 1000.0
        self._pose.position.mark(monotonic())
