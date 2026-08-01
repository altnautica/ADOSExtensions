"""Follow-loop configuration defaults and the published state shape.

The agent reads live per-drone config from ``ctx.config_kv`` each loop,
falling back to the static manifest config and finally these defaults so
a fresh install behaves sensibly before any value is written from the
GCS settings tab. The values mirror ``config-schema.json``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

# The topic the agent publishes the follow read-back on and the GCS skill
# + tab subscribe to. Must equal the manifest skill ``state.topic``.
FOLLOW_STATE_TOPIC = "follow.state"

# Lock-state words on the wire (lowercase, matching the vision contract).
LOCK_LOCKED = "locked"
LOCK_UNCERTAIN = "uncertain"
LOCK_LOST = "lost"

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
HOLD_FC_NOT_GUIDED = "fc-not-guided"
HOLD_NO_GROUND_FIX = "no-ground-fix"


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

        return cls(
            active=bool(pick("active", cls.active)),
            follow_distance_m=_as_float(
                pick("follow_distance_m", cls.follow_distance_m),
                cls.follow_distance_m,
            ),
            follow_height_m=_as_float(
                pick("follow_height_m", cls.follow_height_m),
                cls.follow_height_m,
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
        )


def _as_float(value: object, default: float) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


@dataclass
class FollowState:
    """The published follow read-back. ``commanding`` is the honest bit:
    True only while active, locked, the vehicle pose is CURRENT, AND the
    flight controller is armed and in a guided/offboard mode that accepts the
    setpoints. ``fc_armed`` / ``fc_guided`` carry the flight-controller state,
    and are reported False once the HEARTBEAT they came from goes stale — a
    remembered arm state is not an observed one. ``hold_reason`` names which
    gate is holding, so ``commanding: false`` is never a cause-free reading."""

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

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def changed_from(self, other: "FollowState") -> bool:
        return self.to_dict() != other.to_dict()


__all__ = [
    "FOLLOW_STATE_TOPIC",
    "LOCK_LOCKED",
    "LOCK_UNCERTAIN",
    "LOCK_LOST",
    "HOLD_INACTIVE",
    "HOLD_NO_LOCK",
    "HOLD_LOCK_UNCERTAIN",
    "HOLD_LOCK_LOST",
    "HOLD_POSE_STALE",
    "HOLD_FC_STALE",
    "HOLD_FC_DISARMED",
    "HOLD_FC_NOT_GUIDED",
    "HOLD_NO_GROUND_FIX",
    "FollowConfig",
    "FollowState",
]
