"""Shared fakes for the Follow-Me plugin tests: a fake plugin context that
records every MAVLink frame, setpoint and event the plugin emits, MAVLink
delivery builders, and a controllable clock.
"""

from __future__ import annotations

import time
from typing import Any

from pymavlink.dialects.v20 import common as mavlink2

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

import follow_me
from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    LOCK_LOCKED,
)


class _Log:
    def info(self, *_a: Any, **_k: Any) -> None:
        pass

    def warning(self, *_a: Any, **_k: Any) -> None:
        pass

    def debug(self, *_a: Any, **_k: Any) -> None:
        pass


class _ConfigKv:
    def __init__(self, values: dict[str, Any]) -> None:
        self._values = values

    async def get(self, key: str, default: Any = None) -> Any:
        return self._values.get(key, default)

    async def set(self, key: str, value: Any, scope: str = "drone") -> dict:
        self._values[key] = value
        return {"ok": True}


class _Events:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict[str, Any]]] = []
        self.subscriptions: dict[str, Any] = {}

    async def publish(self, topic: str, payload: dict[str, Any]) -> int:
        self.published.append((topic, payload))
        return 1

    async def subscribe(self, topic: str, cb: Any) -> None:
        self.subscriptions[topic] = cb


class _MAVLink:
    def __init__(self) -> None:
        self.sent: list[bytes] = []
        self.subscriptions: dict[str, Any] = {}

    async def send(self, frame: bytes, component_id: int | None = None) -> dict:
        self.sent.append(frame)
        return {"ok": True}

    async def subscribe(self, msg_name: str, cb: Any) -> None:
        self.subscriptions[msg_name] = cb


class _Flight:
    """The pilot's motion surface: a lease, goals under it, the stick feed.

    ``setpoints`` records every goal sent (the follow's only motion output);
    ``acquire_reply`` is what the next ``lease_acquire`` returns.
    """

    def __init__(self) -> None:
        self.setpoints: list[dict[str, Any]] = []
        self.goal_leases: list[str] = []
        self.acquire_reply: dict[str, Any] = {"ok": True, "lease_id": 7}
        self.acquires: list[dict[str, Any]] = []
        self.releases: list[str] = []
        self.sticks_subscribed = False

    async def lease_acquire(self, *, kind: str, skill_id: str) -> dict:
        self.acquires.append({"kind": kind, "skill_id": skill_id})
        return dict(self.acquire_reply)

    async def lease_release(self, lease_id: int) -> dict:
        self.releases.append(lease_id)
        return {"ok": True}

    async def goal_send(self, lease_id: int, goal: dict[str, Any]) -> dict:
        self.goal_leases.append(lease_id)
        self.setpoints.append(dict(goal))
        return {"ok": True}

    async def sticks_subscribe(self) -> dict:
        self.sticks_subscribed = True
        return {"ok": True}


class _Vision:
    def __init__(self, designate_result: dict[str, Any] | None = None) -> None:
        self._designate_result = designate_result or {
            "designated": True,
            "track_id": 7,
            "camera_id": "uvc-0",
        }
        self.detection_cb: Any = None
        self.designate_calls: list[dict[str, Any]] = []

    async def subscribe_detections(
        self, cb: Any, *, camera_id: str | None = None, embeddings: bool = False
    ) -> None:
        self.detection_cb = cb
        self.embeddings = embeddings

    async def designate_track(
        self,
        camera_id: str,
        bbox: BoundingBox,
        *,
        class_label: str = "",
        confidence: float = 1.0,
    ) -> dict:
        self.designate_calls.append(
            {
                "camera_id": camera_id,
                "bbox": bbox,
                "class_label": class_label,
                "confidence": confidence,
            }
        )
        return dict(self._designate_result)


class _Ctx:
    """A fake plugin context covering the surface the plugin uses."""

    def __init__(
        self,
        config: dict[str, Any] | None = None,
        designate_result: dict[str, Any] | None = None,
    ) -> None:
        self.config = dict(config or {})
        self.config_kv = _ConfigKv(dict(config or {}))
        self.events = _Events()
        self.mavlink = _MAVLink()
        self.flight = _Flight()
        self.vision = _Vision(designate_result)
        self.log = _Log()


# The host delivers each subscribed message as {msg_name, frame, timestamp_ms}
# with ``frame`` the raw wire bytes; these build exactly that from real packed
# frames, so the handlers are exercised through the decoder they run on.
_FC = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)


def _delivery(msg: Any) -> dict[str, Any]:
    return {
        "msg_name": msg.get_type(),
        "frame": bytes(msg.pack(_FC)),
        "timestamp_ms": 0,
    }


def _attitude(roll: float, pitch: float, yaw: float) -> dict[str, Any]:
    return _delivery(mavlink2.MAVLink_attitude_message(0, roll, pitch, yaw, 0.0, 0.0, 0.0))


def _global_position(lat_deg: float, lon_deg: float, rel_alt_m: float) -> dict[str, Any]:
    return _delivery(
        mavlink2.MAVLink_global_position_int_message(
            0,
            int(round(lat_deg * 1e7)),
            int(round(lon_deg * 1e7)),
            0,
            int(round(rel_alt_m * 1000)),
            0,
            0,
            0,
            0,
        )
    )


def _heartbeat(
    *, base_mode: int, custom_mode: int, autopilot: int, mav_type: int = 2
) -> dict[str, Any]:
    return _delivery(
        mavlink2.MAVLink_heartbeat_message(
            mav_type, autopilot, base_mode, custom_mode, 4, 3
        )
    )


def _mount_orientation(pitch_deg: float, yaw_deg: float) -> dict[str, Any]:
    return _delivery(mavlink2.MAVLink_mount_orientation_message(0, 0.0, pitch_deg, yaw_deg, 0.0))


def _local_position(
    n: float, e: float, d: float, vn: float = 0.0, ve: float = 0.0
) -> dict[str, Any]:
    return _delivery(
        mavlink2.MAVLink_local_position_ned_message(0, n, e, d, vn, ve, 0.0)
    )


def _distance_sensor(distance_m: float, orientation: int = 25) -> dict[str, Any]:
    return _delivery(
        mavlink2.MAVLink_distance_sensor_message(
            0, 5, 1200, int(round(distance_m * 100)), 10, 0, orientation, 0
        )
    )


def _level_pose(plugin: follow_me.FollowMePlugin) -> None:
    """Give the plugin a usable, level vehicle pose at altitude."""
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    plugin._on_local_position(_local_position(0.0, 0.0, -20.0))
    plugin._on_global_position(_global_position(12.0, 77.0, 20.0))


def _arm_guided(
    plugin: follow_me.FollowMePlugin,
    *,
    autopilot: int = 3,  # MAV_AUTOPILOT_ARDUPILOTMEGA
    custom_mode: int = 4,  # ArduCopter GUIDED
) -> None:
    """Feed a HEARTBEAT that reports the FC armed and in a guided mode, the
    precondition for the loop to actually command."""
    plugin._on_heartbeat(
        _heartbeat(base_mode=0x81, custom_mode=custom_mode, autopilot=autopilot)
    )


def _bbox_center_frame() -> BoundingBox:
    # A box near the frame centre so the projection has a downward ray when
    # a mount pitch is configured.
    return BoundingBox(x=315.0, y=235.0, width=10.0, height=10.0)


def _make_plugin(ctx: _Ctx) -> follow_me.FollowMePlugin:
    plugin = follow_me.FollowMePlugin()
    plugin._ctx = ctx
    return plugin


class _Clock:
    """A controllable stand-in for the plugin's monotonic clock, so a test can
    let a telemetry stream go stale without sleeping through a real window."""

    def __init__(self) -> None:
        self.now = time.monotonic()

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _install_clock(monkeypatch: Any) -> _Clock:
    """Point the plugin's clock seam at a controllable clock.

    Every module of the plugin reads ``follow_me.fc_state.monotonic`` at call
    time, so one patch drives the whole staleness surface.
    """
    clock = _Clock()
    monkeypatch.setattr(follow_me.fc_state, "monotonic", clock)
    return clock


def _seed_lock(
    plugin: follow_me.FollowMePlugin,
    *,
    track_id: int = 7,
    lock_state: str = LOCK_LOCKED,
    bbox: BoundingBox | None = None,
    aged: bool = False,
    at: float | None = None,
) -> None:
    """Seed the plugin's locked-target tracker as if a detection batch had
    just arrived carrying the engine's designated target. ``aged=True``
    records the sighting far in the past so the coast window treats it as
    lost on the next tick. ``at`` records it at a specific monotonic time,
    for tests running on a controllable clock."""
    base = time.monotonic() if at is None else at
    seen_at = base - 1e6 if aged else base
    batch = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=0,
        detections=[
            Detection(
                bbox=bbox if bbox is not None else _bbox_center_frame(),
                class_label="person",
                confidence=0.9,
                track_id=track_id,
                lock_state=lock_state,
            )
        ],
    )
    plugin._tracker.record(batch, seen_at)


def _state_events(ctx: _Ctx) -> list[dict[str, Any]]:
    return [p for (t, p) in ctx.events.published if t == FOLLOW_STATE_TOPIC]


def _telemetry_config() -> dict[str, Any]:
    return {
        "active": True,
        "gimbal_point": False,
        "designate_camera": "uvc-0",
        "camera_hfov_deg": 70.0,
        "mount_pitch_deg": 45.0,
        "follow_distance_m": 6.0,
        "follow_height_m": 4.0,
    }


def _spy_projection(monkeypatch: Any) -> dict[str, Any]:
    """Record what the follow loop passes to measure_target (the camera pose
    flattened into its angle fields), while still calling the real geometry."""
    captured: dict[str, Any] = {}
    real = follow_me.projection.measure_target

    def spy(**kwargs: Any) -> Any:
        captured.clear()
        captured.update(kwargs)
        cam = kwargs["cam"]
        captured["gimbal_pitch_deg"] = cam.gimbal_pitch_deg
        captured["gimbal_yaw_deg"] = cam.gimbal_yaw_deg
        captured["mount_pitch_deg"] = cam.mount_pitch_deg
        return real(**kwargs)

    monkeypatch.setattr(follow_me.projection, "measure_target", spy)
    return captured


class _Tools:
    def __init__(self) -> None:
        self.handlers: dict[str, Any] = {}

    def register(self, name: str, handler: Any) -> None:
        self.handlers[name] = handler
