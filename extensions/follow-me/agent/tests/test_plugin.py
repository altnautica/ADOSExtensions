"""Plugin behaviour tests: the lock-state safety gate and designation.

These tests drive the plugin against a fake plugin context that records
every MAVLink frame the follow loop emits and every state read-back it
publishes. The geometry itself is covered by ``test_projection``; here the
focus is the control logic the operator's safety depends on:

* a setpoint is sent only while the behaviour is active, a track is
  locked, the tracker reports ``locked``, and a vehicle pose exists;
* ``uncertain`` and ``lost`` immediately stop commanding (no frame);
* a ``lost`` track drops the lock so the plugin never silently follows a
  different subject after re-acquisition; the operator must re-designate;
* an operator designate click stores the engine-returned track id and the
  loop then follows that id only.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

import follow_me
from follow_me.state import (
    DESIGNATE_TOPIC,
    FOLLOW_STATE_TOPIC,
    LOCK_LOCKED,
    LOCK_LOST,
    LOCK_UNCERTAIN,
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
        self, cb: Any, *, camera_id: str | None = None
    ) -> None:
        self.detection_cb = cb

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
        self.vision = _Vision(designate_result)
        self.log = _Log()


def _level_pose(plugin: follow_me.FollowMePlugin) -> None:
    """Give the plugin a usable, level vehicle pose at altitude."""
    plugin._on_attitude({"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    plugin._on_global_position(
        {
            "lat": int(round(12.0 * 1e7)),
            "lon": int(round(77.0 * 1e7)),
            "relative_alt": 20_000,  # 20 m in millimetres
        }
    )


def _bbox_center_frame() -> BoundingBox:
    # A box near the frame centre so the projection has a downward ray when
    # a mount pitch is configured.
    return BoundingBox(x=315.0, y=235.0, width=10.0, height=10.0)


def _make_plugin(ctx: _Ctx) -> follow_me.FollowMePlugin:
    plugin = follow_me.FollowMePlugin()
    plugin._ctx = ctx
    return plugin


def _state_events(ctx: _Ctx) -> list[dict[str, Any]]:
    return [p for (t, p) in ctx.events.published if t == FOLLOW_STATE_TOPIC]


# ---------------------------------------------------------------------------
# Lock-state safety gate
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_locked_and_active_sends_a_setpoint() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "follow_distance_m": 6.0,
            "follow_height_m": 4.0,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOCKED
    plugin._last_seen_monotonic = math.inf  # never aged out for this tick

    await plugin._tick()

    assert ctx.mavlink.sent, "a locked, active follow must emit a setpoint"
    states = _state_events(ctx)
    assert states, "the loop must publish a follow.state read-back"
    assert states[-1]["commanding"] is True


@pytest.mark.asyncio
async def test_uncertain_lock_sends_no_setpoint() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_UNCERTAIN
    plugin._last_seen_monotonic = math.inf

    await plugin._tick()

    assert not ctx.mavlink.sent, "uncertain lock must not command the FC"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["lock_state"] == LOCK_UNCERTAIN
    # The lock id is retained on uncertain (a recoverable state).
    assert plugin._locked_id == 7


@pytest.mark.asyncio
async def test_lost_lock_sends_no_setpoint_and_drops_lock() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOST
    plugin._last_seen_monotonic = math.inf

    await plugin._tick()

    assert not ctx.mavlink.sent, "lost lock must not command the FC"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["lock_state"] == LOCK_LOST
    # A lost track drops the lock: no silent re-acquisition onto another
    # subject; the operator must designate again.
    assert plugin._locked_id is None


@pytest.mark.asyncio
async def test_coast_window_expiry_is_treated_as_lost() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOCKED
    # Seen far enough in the past that the coast window has elapsed.
    plugin._last_seen_monotonic = -1e9

    await plugin._tick()

    assert not ctx.mavlink.sent, "a stale lock past the coast window is lost"
    assert plugin._locked_id is None


@pytest.mark.asyncio
async def test_inactive_sends_no_setpoint() -> None:
    ctx = _Ctx(
        config={
            "active": False,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOCKED
    plugin._last_seen_monotonic = math.inf

    await plugin._tick()

    assert not ctx.mavlink.sent, "an inactive behaviour never commands"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False


@pytest.mark.asyncio
async def test_no_pose_yet_holds_without_commanding() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    # No pose set: have_attitude / have_position are both False.

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOCKED
    plugin._last_seen_monotonic = math.inf

    await plugin._tick()

    assert not ctx.mavlink.sent, "without a pose there is nothing to project"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False


@pytest.mark.asyncio
async def test_gimbal_point_emits_a_second_frame() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "follow_distance_m": 6.0,
            "follow_height_m": 4.0,
            "gimbal_point": True,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)

    plugin._locked_id = 7
    plugin._last_bbox = _bbox_center_frame()
    plugin._last_lock_state = LOCK_LOCKED
    plugin._last_seen_monotonic = math.inf

    await plugin._tick()

    # One position-target frame plus one gimbal command frame.
    assert len(ctx.mavlink.sent) == 2


# ---------------------------------------------------------------------------
# Detection routing: the plugin follows the engine's designated track.
# The operator designates through the vision engine (the GCS overlay calls the
# engine's designate route); the engine's single-object tracker stamps the one
# designated detection with a track id + lock state, and the plugin adopts that
# track. The plugin never picks a target itself, so there is no plugin-side
# designate path.
# ---------------------------------------------------------------------------


def test_batch_adopts_the_engine_tracked_detection() -> None:
    ctx = _Ctx(config={"designate_camera": "uvc-0"})
    plugin = _make_plugin(ctx)
    assert plugin._locked_id is None

    tracked = Detection(
        bbox=BoundingBox(100, 100, 20, 20),
        class_label="person",
        confidence=0.9,
        track_id=7,
        lock_state=LOCK_LOCKED,
    )
    batch = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=0,
        detections=[tracked],
    )
    plugin._on_batch(batch)

    assert plugin._locked_id == 7
    assert plugin._last_bbox == tracked.bbox
    assert plugin._last_lock_state == LOCK_LOCKED


def test_batch_picks_the_tracked_detection_among_untracked() -> None:
    ctx = _Ctx(config={"designate_camera": "uvc-0"})
    plugin = _make_plugin(ctx)

    untracked = Detection(
        bbox=BoundingBox(0, 0, 5, 5),
        class_label="person",
        confidence=0.8,
        track_id=None,
        lock_state=None,
    )
    tracked = Detection(
        bbox=BoundingBox(100, 100, 20, 20),
        class_label="person",
        confidence=0.9,
        track_id=4,
        lock_state=LOCK_UNCERTAIN,
    )
    batch = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=0,
        detections=[untracked, tracked],
    )
    plugin._on_batch(batch)

    assert plugin._locked_id == 4
    assert plugin._last_bbox == tracked.bbox
    assert plugin._last_lock_state == LOCK_UNCERTAIN


def test_batch_with_no_tracked_detection_adopts_nothing() -> None:
    ctx = _Ctx(config={"designate_camera": "uvc-0"})
    plugin = _make_plugin(ctx)

    # Untracked detections (the engine has nothing designated) never start a
    # follow: the operator must designate through the engine first.
    batch = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=0,
        detections=[
            Detection(
                bbox=BoundingBox(0, 0, 5, 5),
                class_label="person",
                confidence=0.8,
                track_id=None,
                lock_state=None,
            )
        ],
    )
    plugin._on_batch(batch)

    assert plugin._locked_id is None
    assert plugin._last_bbox is None


def test_designate_topic_constant_matches_state_module() -> None:
    # The overlay publishes on DESIGNATE_TOPIC and the read-back lands on
    # FOLLOW_STATE_TOPIC; both must be the literals the manifest declares.
    assert DESIGNATE_TOPIC == "follow-me/designate"
    assert FOLLOW_STATE_TOPIC == "follow.state"
