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

# The topic the GCS overlay publishes the operator's designate click on.
DESIGNATE_TOPIC = "follow-me/designate"

# Lock-state words on the wire (lowercase, matching the vision contract).
LOCK_LOCKED = "locked"
LOCK_UNCERTAIN = "uncertain"
LOCK_LOST = "lost"


@dataclass(frozen=True)
class FollowConfig:
    """Resolved per-drone follow configuration."""

    active: bool = False
    follow_distance_m: float = 8.0
    follow_height_m: float = 4.0
    gimbal_point: bool = True
    designate_camera: str = "uvc-0"
    camera_hfov_deg: float = 70.0
    mount_pitch_deg: float = 0.0

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
    True only while active, locked, and actually emitting setpoints."""

    active: bool = False
    lock_state: str | None = None
    target_id: int | None = None
    range_m: float | None = None
    distance_setpoint_m: float | None = None
    height_setpoint_m: float | None = None
    commanding: bool = False

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def changed_from(self, other: "FollowState") -> bool:
        return self.to_dict() != other.to_dict()


__all__ = [
    "FOLLOW_STATE_TOPIC",
    "DESIGNATE_TOPIC",
    "LOCK_LOCKED",
    "LOCK_UNCERTAIN",
    "LOCK_LOST",
    "FollowConfig",
    "FollowState",
]
