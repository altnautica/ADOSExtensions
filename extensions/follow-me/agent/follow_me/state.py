"""Follow-loop configuration defaults and the published state shape.

The agent reads live per-drone config from ``ctx.config_kv`` each loop,
falling back to the static manifest config and finally these defaults so
a fresh install behaves sensibly before any value is written from the
GCS settings tab. The values mirror ``config-schema.json``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

# The topic the agent publishes the follow read-back on and the GCS skill
# + tab subscribe to. Must equal the manifest skill ``state.topic``.
FOLLOW_STATE_TOPIC = "follow.state"
# One-shot events: a smart re-acquire designated a new track, and the follow
# gave the subject up.
FOLLOW_REACQUIRED_TOPIC = "follow.reacquired"
FOLLOW_LOST_TOPIC = "follow.lost"
# The follow stopped because the pilot revoked its motion lease (operator
# pause or stick takeover, a land/RTH, an FC mode change, ...).
FOLLOW_STOPPED_TOPIC = "follow.stopped"

# Follow modes. ``follow``: hold a distance and an angle from the subject's
# travel heading (180 behind, 90 right side, 0 lead). ``orbit``: circle the
# subject at a radius and an angular rate.
FOLLOW_MODE_FOLLOW = "follow"
FOLLOW_MODE_ORBIT = "orbit"
_FOLLOW_MODES = (FOLLOW_MODE_FOLLOW, FOLLOW_MODE_ORBIT)
ORBIT_CW = "cw"
ORBIT_CCW = "ccw"
_ORBIT_DIRECTIONS = (ORBIT_CW, ORBIT_CCW)

# Config ranges, shared by the resolver, the stick nudges and the schema.
FOLLOW_DISTANCE_RANGE = (3.0, 30.0)
FOLLOW_HEIGHT_RANGE = (0.0, 20.0)
ORBIT_RADIUS_RANGE = (3.0, 30.0)
ORBIT_RATE_RANGE = (2.0, 30.0)
OBSTACLE_MARGIN_RANGE = (0.8, 5.0)

# Lock-state words on the wire (lowercase, matching the vision contract).
LOCK_LOCKED = "locked"
LOCK_UNCERTAIN = "uncertain"
LOCK_LOST = "lost"

# The follow phase on the wire. ``idle``: the skill is off. ``acquiring``:
# armed, waiting for a lock and a first range fix. ``following``: commanding
# on a live lock. ``coasting``: the lock dropped and a smart re-acquire flies
# toward the predicted standoff while the prediction is still tight.
# ``searching``: the same window once the prediction has spread too far to
# draw a box. ``lost``: the subject was given up. ``holding``: locked but a
# gate (telemetry, flight controller, range, uncertain lock) holds.
PHASE_IDLE = "idle"
PHASE_ACQUIRING = "acquiring"
PHASE_FOLLOWING = "following"
PHASE_COASTING = "coasting"
PHASE_SEARCHING = "searching"
PHASE_LOST = "lost"
PHASE_HOLDING = "holding"

# Re-acquire modes. ``strict`` gives the subject up on the first loss and
# waits for the operator. ``smart`` coasts on the prediction for a window and
# re-designates a track that matches the subject's appearance and predicted
# position, announcing it.
REACQUIRE_STRICT = "strict"
REACQUIRE_SMART = "smart"
_REACQUIRE_MODES = (REACQUIRE_STRICT, REACQUIRE_SMART)

# Why the loop is not commanding, on the wire. ``commanding: false`` on its
# own tells an operator that the follow is not flying the aircraft but not
# which of several very different causes is responsible — a disarmed FC is a
# normal pre-flight state, while telemetry that stopped arriving mid-follow
# is a fault. Every non-commanding branch names itself with one of these.
HOLD_INACTIVE = "inactive"
HOLD_NO_LOCK = "no-lock"
HOLD_LOCK_UNCERTAIN = "lock-uncertain"
HOLD_LOCK_LOST = "lock-lost"
HOLD_POSE_STALE = "pose-stale"
HOLD_FC_STALE = "fc-stale"
HOLD_FC_DISARMED = "fc-disarmed"
HOLD_NO_RANGE = "no-range"
# The pilot refused the motion lease: another skill holds it
# (``lease-busy``) or the vehicle cannot be flown by a skill right now
# (``lease-refused``: disarmed, wrong vehicle class, no guided mode).
HOLD_LEASE_BUSY = "lease-busy"
HOLD_LEASE_REFUSED = "lease-refused"


@dataclass(frozen=True)
class FollowConfig:
    """Resolved per-drone follow configuration."""

    active: bool = False
    follow_distance_m: float = 8.0
    follow_height_m: float = 4.0
    gimbal_point: bool = True
    # A camera-selector value: "auto" / "" resolves by requirement (the first
    # detect camera → any camera for the detection filter), or a pinned camera
    # id. Resolved by the plugin; not a device string.
    designate_camera: str = "auto"
    camera_hfov_deg: float = 70.0
    # Fixed downward tilt of the designate camera below the horizon. A
    # forward camera needs a positive tilt for its ground projection to
    # intersect the ground plane at all, so the default is a modest
    # forward-down angle rather than 0 (a level camera never resolves a
    # ground point and the follow loop would never command).
    mount_pitch_deg: float = 30.0
    reacquire_mode: str = REACQUIRE_SMART
    reacquire_window_s: float = 4.0
    reacquire_min_similarity: float = 0.75
    max_speed_mps: float = 6.0
    follow_mode: str = FOLLOW_MODE_FOLLOW
    follow_angle_deg: float = 180.0
    orbit_radius_m: float = 8.0
    orbit_rate_dps: float = 10.0
    orbit_direction: str = ORBIT_CW
    # The skill's obstacle margin, passed to the pilot's planner in each goal.
    obstacle_margin_m: float = 1.5

    @classmethod
    def resolve(
        cls,
        live: dict[str, object],
        static: dict[str, object],
    ) -> "FollowConfig":
        """Resolve config with precedence live -> static -> default."""

        def pick(key: str, default: object) -> object:
            if key in live and live[key] is not None:
                return live[key]
            if key in static and static[key] is not None:
                return static[key]
            return default

        mode = str(pick("reacquire_mode", cls.reacquire_mode)).strip().lower()
        follow_mode = str(pick("follow_mode", cls.follow_mode)).strip().lower()
        direction = str(pick("orbit_direction", cls.orbit_direction)).strip().lower()
        return cls(
            active=bool(pick("active", cls.active)),
            follow_distance_m=_clamped(
                pick("follow_distance_m", cls.follow_distance_m),
                cls.follow_distance_m,
                *FOLLOW_DISTANCE_RANGE,
            ),
            follow_height_m=_clamped(
                pick("follow_height_m", cls.follow_height_m),
                cls.follow_height_m,
                *FOLLOW_HEIGHT_RANGE,
            ),
            gimbal_point=bool(pick("gimbal_point", cls.gimbal_point)),
            designate_camera=str(
                pick("designate_camera", cls.designate_camera)
            ),
            camera_hfov_deg=_as_float(
                pick("camera_hfov_deg", cls.camera_hfov_deg),
                cls.camera_hfov_deg,
            ),
            mount_pitch_deg=_as_float(
                pick("mount_pitch_deg", cls.mount_pitch_deg),
                cls.mount_pitch_deg,
            ),
            # An unrecognised mode resolves to strict: the conservative
            # behaviour that never re-designates on its own.
            reacquire_mode=mode if mode in _REACQUIRE_MODES else REACQUIRE_STRICT,
            reacquire_window_s=_clamped(
                pick("reacquire_window_s", cls.reacquire_window_s),
                cls.reacquire_window_s,
                0.5,
                15.0,
            ),
            reacquire_min_similarity=_clamped(
                pick("reacquire_min_similarity", cls.reacquire_min_similarity),
                cls.reacquire_min_similarity,
                0.5,
                0.99,
            ),
            max_speed_mps=_clamped(
                pick("max_speed_mps", cls.max_speed_mps),
                cls.max_speed_mps,
                0.5,
                15.0,
            ),
            follow_mode=follow_mode if follow_mode in _FOLLOW_MODES else FOLLOW_MODE_FOLLOW,
            follow_angle_deg=_as_float(
                pick("follow_angle_deg", cls.follow_angle_deg), cls.follow_angle_deg
            )
            % 360.0,
            orbit_radius_m=_clamped(
                pick("orbit_radius_m", cls.orbit_radius_m),
                cls.orbit_radius_m,
                *ORBIT_RADIUS_RANGE,
            ),
            orbit_rate_dps=_clamped(
                pick("orbit_rate_dps", cls.orbit_rate_dps),
                cls.orbit_rate_dps,
                *ORBIT_RATE_RANGE,
            ),
            orbit_direction=direction if direction in _ORBIT_DIRECTIONS else ORBIT_CW,
            obstacle_margin_m=_clamped(
                pick("obstacle_margin_m", cls.obstacle_margin_m),
                cls.obstacle_margin_m,
                *OBSTACLE_MARGIN_RANGE,
            ),
        )


def _as_float(value: object, default: float) -> float:
    try:
        v = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return v if v == v and abs(v) != float("inf") else default


def _clamped(value: object, default: float, lo: float, hi: float) -> float:
    v = _as_float(value, default)
    return lo if v < lo else hi if v > hi else v


@dataclass
class FollowState:
    """The published follow read-back. ``commanding`` is the honest bit:
    True only while active, the vehicle pose is CURRENT, the flight
    controller is armed and in a guided/offboard mode that accepts the
    setpoints, AND the loop is flying a follow (on a live lock or a smart
    coast). ``fc_armed`` / ``fc_guided`` are reported False once the
    HEARTBEAT they came from goes stale — a remembered arm state is not an
    observed one. ``hold_reason`` names which gate is holding, so
    ``commanding: false`` is never a cause-free reading."""

    active: bool = False
    lock_state: str | None = None
    target_id: int | None = None
    range_m: float | None = None
    distance_setpoint_m: float | None = None
    height_setpoint_m: float | None = None
    commanding: bool = False
    fc_armed: bool = False
    fc_guided: bool = False
    hold_reason: str | None = None
    phase: str = PHASE_IDLE
    range_sigma_m: float | None = None
    range_cues: list[dict[str, object]] = field(default_factory=list)
    target_speed_mps: float | None = None
    bearing_deg: float | None = None
    depression_deg: float | None = None
    target_ned: list[float] | None = None
    target_vel_ned: list[float] | None = None
    predicted_bbox: dict[str, object] | None = None
    setpoint_rate_hz: float = 0.0
    agl_m: float | None = None
    agl_source: str | None = None
    reacquire_mode: str = REACQUIRE_SMART
    coast_remaining_s: float = 0.0
    pipeline_latency_ms: int | None = None
    track_id: int | None = None
    confidence: float | None = None
    follow_mode: str = FOLLOW_MODE_FOLLOW
    follow_angle_deg: float = 180.0
    orbit_radius_m: float = 8.0
    orbit_rate_dps: float = 10.0
    orbit_direction: str = ORBIT_CW
    orbit_phase_deg: float | None = None
    # The pilot motion lease this follow holds, or None.
    lease_id: str | None = None
    # The operator nudged the follow parameters with the sticks this session.
    nudged: bool = False

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def changed_from(self, other: "FollowState") -> bool:
        return self.to_dict() != other.to_dict()


__all__ = [
    "FOLLOW_STATE_TOPIC",
    "FOLLOW_REACQUIRED_TOPIC",
    "FOLLOW_LOST_TOPIC",
    "FOLLOW_STOPPED_TOPIC",
    "FOLLOW_MODE_FOLLOW",
    "FOLLOW_MODE_ORBIT",
    "ORBIT_CW",
    "ORBIT_CCW",
    "FOLLOW_DISTANCE_RANGE",
    "FOLLOW_HEIGHT_RANGE",
    "ORBIT_RADIUS_RANGE",
    "ORBIT_RATE_RANGE",
    "OBSTACLE_MARGIN_RANGE",
    "LOCK_LOCKED",
    "LOCK_UNCERTAIN",
    "LOCK_LOST",
    "PHASE_IDLE",
    "PHASE_ACQUIRING",
    "PHASE_FOLLOWING",
    "PHASE_COASTING",
    "PHASE_SEARCHING",
    "PHASE_LOST",
    "PHASE_HOLDING",
    "REACQUIRE_STRICT",
    "REACQUIRE_SMART",
    "HOLD_INACTIVE",
    "HOLD_NO_LOCK",
    "HOLD_LOCK_UNCERTAIN",
    "HOLD_LOCK_LOST",
    "HOLD_POSE_STALE",
    "HOLD_FC_STALE",
    "HOLD_FC_DISARMED",
    "HOLD_NO_RANGE",
    "HOLD_LEASE_BUSY",
    "HOLD_LEASE_REFUSED",
    "FollowConfig",
    "FollowState",
]
