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
# on a live lock. ``holding``: locked but a gate (telemetry, flight
# controller, range, uncertain lock) holds. Once the lock is lost:
# ``predicting``: flying toward the predicted standoff while the prediction
# is tight. ``repositioning``: flying to a vantage point the pilot chose to
# see the predicted position from. ``searching``: at the vantage point,
# sweeping the camera, widening in steps. ``holding-await-operator``:
# stopped at a latched hover point, waiting for the operator to resume,
# re-designate or stop.
PHASE_IDLE = "idle"
PHASE_ACQUIRING = "acquiring"
PHASE_FOLLOWING = "following"
PHASE_HOLDING = "holding"
PHASE_PREDICTING = "predicting"
PHASE_REPOSITIONING = "repositioning"
PHASE_SEARCHING = "searching"
PHASE_AWAIT_OPERATOR = "holding-await-operator"
# The phases in which the lock is lost and the subject is being sought.
RECOVERY_PHASES = (PHASE_PREDICTING, PHASE_REPOSITIONING, PHASE_SEARCHING)

# Re-acquire modes. ``strict`` gives the subject up on the first loss and
# waits for the operator. ``smart`` predicts, repositions and searches within
# a budget and re-designates a person that matches the subject's appearance
# and predicted position, announcing it.
REACQUIRE_STRICT = "strict"
REACQUIRE_SMART = "smart"
_REACQUIRE_MODES = (REACQUIRE_STRICT, REACQUIRE_SMART)

# What an uncertain lock does. ``follow``: keep following the prediction at
# reduced feed-forward. ``hold``: hold where it is. Either way an unsure
# spell longer than UNCERTAIN_FOLLOW_MAX_S is handled as a loss.
UNCERTAIN_FOLLOW = "follow"
UNCERTAIN_HOLD = "hold"
_UNCERTAIN_MODES = (UNCERTAIN_FOLLOW, UNCERTAIN_HOLD)
UNCERTAIN_FOLLOW_MAX_S = 3.0

SEARCH_BUDGET_RANGE = (5.0, 60.0)
SEARCH_RADIUS_RANGE = (3.0, 12.0)
SEARCH_SPEED_RANGE = (0.5, 4.0)

# Why the loop is not commanding, on the wire. ``commanding: false`` on its
# own tells an operator that the follow is not flying the aircraft but not
# which of several very different causes is responsible — a disarmed FC is a
# normal pre-flight state, while telemetry that stopped arriving mid-follow
# is a fault. Every non-commanding branch names itself with one of these.
HOLD_INACTIVE = "inactive"
HOLD_NO_LOCK = "no-lock"
HOLD_LOCK_UNCERTAIN = "lock-uncertain"
# Holding for the operator: after a strict loss, after the search budget ran
# out, or after a loss with predictive search switched off.
HOLD_AWAIT_OPERATOR = "awaiting-operator"
HOLD_SEARCH_EXHAUSTED = "search-exhausted"
HOLD_PREDICTIVE_OFF = "predictive-search-off"
HOLD_POSE_STALE = "pose-stale"
HOLD_FC_STALE = "fc-stale"
HOLD_FC_DISARMED = "fc-disarmed"
HOLD_NO_RANGE = "no-range"
# The pilot refused the motion lease: another skill holds it
# (``lease-busy``) or the vehicle cannot be flown by a skill right now
# (``lease-refused``: disarmed, wrong vehicle class, no guided mode).
HOLD_LEASE_BUSY = "lease-busy"
HOLD_LEASE_REFUSED = "lease-refused"
# The follow camera's detection batches stopped arriving. The engine
# publishes a batch for every inferred frame, an empty one too, so silence
# means the camera or its inference stopped, not that nobody is in view.
HOLD_CAMERA_LOST = "camera-lost"


@dataclass(frozen=True)
class FollowConfig:
    """Resolved per-drone follow configuration."""

    active: bool = False
    follow_distance_m: float = 6.0
    follow_height_m: float = 4.0
    gimbal_point: bool = True
    # A camera-selector value: "auto" / "" resolves by requirement (the first
    # detect camera → any camera for the detection filter), or a pinned camera
    # id. Resolved by the plugin; not a device string.
    designate_camera: str = "auto"
    # The operator's follow-camera geometry, used where the detection batches
    # carry none (an agent without the camera roster on its batches). None is
    # unset: the built-in reference camera applies. ``mount_pitch_deg`` is
    # the tilt below the horizon, positive down; 90 is straight down.
    camera_hfov_deg: float | None = None
    mount_pitch_deg: float | None = None
    reacquire_mode: str = REACQUIRE_SMART
    reacquire_min_similarity: float = 0.75
    max_speed_mps: float = 6.0
    uncertain_mode: str = UNCERTAIN_FOLLOW
    # Turned back on at every arm, so a search switched off for one flight
    # never carries into the next unnoticed.
    predictive_search: bool = True
    search_budget_s: float = 25.0
    search_radius_m: float = 12.0
    search_speed_mps: float = 2.0
    # A surface writes ``current + 1`` to resume a search that is holding
    # for the operator.
    operator_resume_seq: int = 0
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
        uncertain = str(pick("uncertain_mode", cls.uncertain_mode)).strip().lower()
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
            camera_hfov_deg=_opt_float(pick("camera_hfov_deg", None)),
            mount_pitch_deg=_opt_float(pick("mount_pitch_deg", None)),
            # An unrecognised mode resolves to strict: the conservative
            # behaviour that never re-designates on its own.
            reacquire_mode=mode if mode in _REACQUIRE_MODES else REACQUIRE_STRICT,
            reacquire_min_similarity=_clamped(
                pick("reacquire_min_similarity", cls.reacquire_min_similarity),
                cls.reacquire_min_similarity,
                0.5,
                0.99,
            ),
            uncertain_mode=uncertain if uncertain in _UNCERTAIN_MODES else UNCERTAIN_FOLLOW,
            predictive_search=bool(pick("predictive_search", cls.predictive_search)),
            search_budget_s=_clamped(
                pick("search_budget_s", cls.search_budget_s),
                cls.search_budget_s,
                *SEARCH_BUDGET_RANGE,
            ),
            search_radius_m=_clamped(
                pick("search_radius_m", cls.search_radius_m),
                cls.search_radius_m,
                *SEARCH_RADIUS_RANGE,
            ),
            search_speed_mps=_clamped(
                pick("search_speed_mps", cls.search_speed_mps),
                cls.search_speed_mps,
                *SEARCH_SPEED_RANGE,
            ),
            operator_resume_seq=int(
                _clamped(pick("operator_resume_seq", 0), 0.0, 0.0, float(2**53))
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


def _opt_float(value: object) -> float | None:
    if value is None:
        return None
    v = _as_float(value, float("nan"))
    return v if v == v else None


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
    recovery). ``fc_armed`` / ``fc_guided`` are reported False once the
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
    # Seconds left of the recovery's search budget.
    reacquire_remaining_s: float = 0.0
    search_budget_s: float = 25.0
    # 0-based widening step while searching, else None.
    search_step: int | None = None
    # {center_ne, semi_major_m, semi_minor_m, heading_deg} while recovering.
    search_area: dict[str, object] | None = None
    # At most 10 points 0.5 s apart while recovering.
    predicted_path_ne: list[list[float]] | None = None
    vantage_ne: list[float] | None = None
    # enroute | arrived | no-vantage, as the pilot reports it.
    vantage_state: str | None = None
    # {ne, kind: narrow | wide}: what hides the subject from the vantage.
    occluder: dict[str, object] | None = None
    # The latched hover point while holding for the operator.
    hold_ne: list[float] | None = None
    uncertain_mode: str = UNCERTAIN_FOLLOW
    predictive_search: bool = True
    operator_resume_seq: int = 0
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
    # The follow camera geometry in use: horizontal field of view, mount
    # pitch in the roster's sign (negative below the nose), and whether they
    # came from the camera roster or the built-in default.
    camera_hfov_deg: float | None = None
    camera_mount_pitch_deg: float | None = None
    camera_geometry_source: str | None = None

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
    "PHASE_HOLDING",
    "PHASE_PREDICTING",
    "PHASE_REPOSITIONING",
    "PHASE_SEARCHING",
    "PHASE_AWAIT_OPERATOR",
    "RECOVERY_PHASES",
    "REACQUIRE_STRICT",
    "REACQUIRE_SMART",
    "UNCERTAIN_FOLLOW",
    "UNCERTAIN_HOLD",
    "UNCERTAIN_FOLLOW_MAX_S",
    "SEARCH_BUDGET_RANGE",
    "SEARCH_RADIUS_RANGE",
    "SEARCH_SPEED_RANGE",
    "HOLD_INACTIVE",
    "HOLD_NO_LOCK",
    "HOLD_LOCK_UNCERTAIN",
    "HOLD_AWAIT_OPERATOR",
    "HOLD_SEARCH_EXHAUSTED",
    "HOLD_PREDICTIVE_OFF",
    "HOLD_POSE_STALE",
    "HOLD_FC_STALE",
    "HOLD_FC_DISARMED",
    "HOLD_NO_RANGE",
    "HOLD_LEASE_BUSY",
    "HOLD_LEASE_REFUSED",
    "HOLD_CAMERA_LOST",
    "FollowConfig",
    "FollowState",
]
