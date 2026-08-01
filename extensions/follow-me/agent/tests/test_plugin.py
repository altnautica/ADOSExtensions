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

import time
from typing import Any

import pytest

from ados.sdk.tracking import EffectiveLock
from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

import follow_me
from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    HOLD_FC_DISARMED,
    HOLD_FC_NOT_GUIDED,
    HOLD_FC_STALE,
    HOLD_POSE_STALE,
    LOCK_LOCKED,
    LOCK_LOST,
    LOCK_UNCERTAIN,
    FollowConfig,
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
    def __init__(self) -> None:
        self.setpoints: list[dict[str, Any]] = []

    async def guided_setpoint(self, **kwargs: Any) -> dict:
        self.setpoints.append(dict(kwargs))
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
        self.flight = _Flight()
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


def _arm_guided(
    plugin: follow_me.FollowMePlugin,
    *,
    autopilot: int = 3,  # MAV_AUTOPILOT_ARDUPILOTMEGA
    custom_mode: int = 4,  # ArduCopter GUIDED
) -> None:
    """Feed a HEARTBEAT that reports the FC armed and in a guided mode, the
    precondition for the loop to actually command."""
    plugin._on_heartbeat(
        {
            "base_mode": 0x81,  # MAV_MODE_FLAG_SAFETY_ARMED | CUSTOM_MODE
            "custom_mode": custom_mode,
            "autopilot": autopilot,
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

    ``raising=False`` on purpose: against a build with no seam this patch is
    a no-op and the plugin keeps reading real time, so a staleness test still
    runs and fails on its behavioural assertion (a setpoint that should never
    have been emitted) rather than erroring on a missing attribute.
    """
    clock = _Clock()
    monkeypatch.setattr(follow_me, "_monotonic", clock, raising=False)
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
    _arm_guided(plugin)

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert ctx.flight.setpoints, "a locked, active follow must emit a setpoint"
    states = _state_events(ctx)
    assert states, "the loop must publish a follow.state read-back"
    assert states[-1]["commanding"] is True
    assert states[-1]["fc_armed"] is True
    assert states[-1]["fc_guided"] is True


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

    _seed_lock(plugin, track_id=7, lock_state=LOCK_UNCERTAIN)

    await plugin._tick()

    assert not ctx.flight.setpoints, "uncertain lock must not command the FC"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["lock_state"] == LOCK_UNCERTAIN
    # The lock id is retained on uncertain (a recoverable state).
    assert plugin._tracker.track_id == 7


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

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOST)

    await plugin._tick()

    assert not ctx.flight.setpoints, "lost lock must not command the FC"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["lock_state"] == LOCK_LOST
    # A lost track drops the lock: no silent re-acquisition onto another
    # subject; the operator must designate again.
    assert plugin._tracker.has_lock is False


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

    # Seen far enough in the past that the coast window has elapsed.
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, aged=True)

    await plugin._tick()

    assert not ctx.flight.setpoints, "a stale lock past the coast window is lost"
    assert plugin._tracker.has_lock is False


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

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert not ctx.flight.setpoints, "an inactive behaviour never commands"
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

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert not ctx.flight.setpoints, "without a pose there is nothing to project"
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
    _arm_guided(plugin)

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    # One position setpoint (via the scoped flight sender) plus one gimbal
    # command frame (raw MAVLink).
    assert len(ctx.flight.setpoints) == 1
    assert len(ctx.mavlink.sent) == 1


# ---------------------------------------------------------------------------
# Flight-controller command gate: commanding is honest about the FC state.
# A locked, active, posed follow only commands when the FC is armed AND in a
# guided/offboard mode that accepts the setpoints.
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_armed_but_not_guided_holds_without_commanding() -> None:
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
    # Armed, but in a non-guided mode (ArduCopter LOITER == 5).
    _arm_guided(plugin, custom_mode=5)

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert not ctx.flight.setpoints, "a non-guided FC must not be commanded"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["fc_armed"] is True
    assert states[-1]["fc_guided"] is False
    # The lock is retained; only the FC state blocks commanding.
    assert states[-1]["lock_state"] == LOCK_LOCKED


@pytest.mark.asyncio
async def test_guided_but_disarmed_holds_without_commanding() -> None:
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
    # Guided mode selected, but the disarmed bit is clear in base_mode.
    plugin._on_heartbeat(
        {"base_mode": 0x01, "custom_mode": 4, "autopilot": 3}
    )

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert not ctx.flight.setpoints, "a disarmed FC must not be commanded"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["fc_armed"] is False
    assert states[-1]["fc_guided"] is True


@pytest.mark.asyncio
async def test_px4_offboard_is_treated_as_guided() -> None:
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
    # PX4 (autopilot 12) packs the main mode into bits 16-23; OFFBOARD == 6.
    _arm_guided(plugin, autopilot=12, custom_mode=(6 << 16))

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert ctx.flight.setpoints, "PX4 OFFBOARD must be commanded like AP GUIDED"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is True
    assert states[-1]["fc_guided"] is True


# ---------------------------------------------------------------------------
# Config surface: mount_pitch_deg is a real, resolvable per-drone key.
# ---------------------------------------------------------------------------


def test_mount_pitch_default_is_a_usable_forward_tilt() -> None:
    # A fresh install with nothing written must resolve a non-zero downward
    # tilt, or a forward camera's ray never intersects the ground.
    cfg = FollowConfig.resolve({}, {})
    assert cfg.mount_pitch_deg == 30.0


def test_mount_pitch_live_value_overrides_the_default() -> None:
    cfg = FollowConfig.resolve({"mount_pitch_deg": 55.0}, {})
    assert cfg.mount_pitch_deg == 55.0


# ---------------------------------------------------------------------------
# Projection inputs: real frame size, real gimbal attitude, and the coast hold.
# ---------------------------------------------------------------------------


def _spy_projection(monkeypatch: Any) -> dict[str, Any]:
    """Record the kwargs the follow loop passes to project_follow_setpoint,
    while still calling through to the real geometry."""
    captured: dict[str, Any] = {}
    real = follow_me.projection.project_follow_setpoint

    def spy(**kwargs: Any) -> Any:
        captured.clear()
        captured.update(kwargs)
        return real(**kwargs)

    monkeypatch.setattr(follow_me.projection, "project_follow_setpoint", spy)
    return captured


@pytest.mark.asyncio
async def test_real_frame_size_from_the_batch_feeds_the_projection(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 45.0,
        }
    )
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    _level_pose(plugin)
    _arm_guided(plugin)

    # A v2 batch carrying its real source-frame size, delivered through the
    # real _on_batch path so the plugin caches the dimensions.
    batch = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=0,
        frame_width=1920,
        frame_height=1080,
        detections=[
            Detection(
                bbox=BoundingBox(955.0, 535.0, 10.0, 10.0),
                class_label="person",
                confidence=0.9,
                track_id=7,
                lock_state=LOCK_LOCKED,
            )
        ],
    )
    plugin._on_batch(batch)

    captured = _spy_projection(monkeypatch)
    await plugin._tick()

    # The projection is fed the batch's real dimensions, not a bbox guess.
    assert captured["frame_width"] == 1920
    assert captured["frame_height"] == 1080


@pytest.mark.asyncio
async def test_reported_gimbal_attitude_feeds_the_projection(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": True,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 30.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    _arm_guided(plugin)
    # MOUNT_ORIENTATION reports the gimbal pitched 40 deg down (negative = down)
    # and yawed 15 deg to the right of the nose.
    plugin._on_mount_orientation({"pitch": -40.0, "yaw": 15.0})

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    captured = _spy_projection(monkeypatch)
    await plugin._tick()

    # The reported attitude (converted to projection convention: +40 down) is
    # the boresight; the fixed mount pitch does not stack on top of it.
    assert captured["gimbal_pitch_deg"] == 40.0
    assert captured["gimbal_yaw_deg"] == 15.0
    assert captured["mount_pitch_deg"] == 0.0


@pytest.mark.asyncio
async def test_falls_back_to_the_last_commanded_gimbal_angle(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": True,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 30.0,
            "follow_distance_m": 6.0,
            "follow_height_m": 4.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    _arm_guided(plugin)

    # Tick 1: no gimbal report yet, so the loop bootstraps on the fixed mount
    # pitch, then commands the gimbal and caches that angle.
    _seed_lock(
        plugin,
        track_id=7,
        lock_state=LOCK_LOCKED,
        bbox=BoundingBox(315.0, 235.0, 10.0, 10.0),
    )
    await plugin._tick()
    assert plugin._commanded_gimbal is not None
    last_commanded = plugin._commanded_gimbal

    # Tick 2 on a FRESH sighting (a new bbox, so not coasting): still no report,
    # so the projection uses the last commanded gimbal angle, not the mount.
    captured = _spy_projection(monkeypatch)
    _seed_lock(
        plugin,
        track_id=7,
        lock_state=LOCK_LOCKED,
        bbox=BoundingBox(320.0, 240.0, 12.0, 12.0),
    )
    await plugin._tick()

    assert (captured["gimbal_pitch_deg"], captured["gimbal_yaw_deg"]) == (
        last_commanded
    )
    assert captured["mount_pitch_deg"] == 0.0


@pytest.mark.asyncio
async def test_coasting_holds_the_last_setpoint_without_recomputing(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
            "camera_hfov_deg": 70.0,
            "mount_pitch_deg": 45.0,
            "follow_distance_m": 6.0,
            "follow_height_m": 4.0,
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    _arm_guided(plugin)

    # One sighting, one fresh command.
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)
    await plugin._tick()
    assert len(ctx.flight.setpoints) == 1
    first_setpoint = ctx.flight.setpoints[-1]

    # Tick again with NO new detection (the tracker holds the same frozen bbox
    # within the coast window) and a CHANGED vehicle attitude. The loop must
    # hold: re-send the same setpoint, never re-project the stale bbox.
    plugin._on_attitude({"roll": 0.0, "pitch": 0.0, "yaw": 1.0})
    captured = _spy_projection(monkeypatch)
    await plugin._tick()

    assert captured == {}, "coasting must not re-project the stale bbox"
    assert len(ctx.flight.setpoints) == 2
    assert ctx.flight.setpoints[-1] == first_setpoint, "the held setpoint is re-sent"
    states = _state_events(ctx)
    assert states[-1]["commanding"] is True


# ---------------------------------------------------------------------------
# Telemetry freshness: the loop projects only through CURRENT vehicle state.
#
# The detection side has always been aged (the coast window). The vehicle side
# was not: a boolean said a pose had once arrived, and once set it never
# cleared. A telemetry subscription can stall while the write path still works
# — a stream-rate change, a lagged receiver, a router reconnect, an FC that
# stops emitting one message — and the loop would then keep commanding at
# 6 Hz, projecting fresh bounding boxes through a frozen attitude and lat/lon,
# with the error growing for as long as the stall lasted and the read-back
# reporting commanding: true throughout.
# ---------------------------------------------------------------------------


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


@pytest.mark.asyncio
async def test_a_stalled_pose_stops_commanding_and_names_the_reason(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    # Telemetry stops. Time passes: longer than the pose window, still inside
    # the detection coast window, so the subject stays locked and the ONLY
    # thing wrong is that the vehicle state is no longer current.
    clock.advance(1.0)

    await plugin._tick()

    assert not ctx.flight.setpoints, (
        "a pose that stopped updating must not be projected through: the "
        "setpoint would be computed from where the aircraft used to be"
    )
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["hold_reason"] == HOLD_POSE_STALE, (
        "commanding: false with no cause is indistinguishable from a normal "
        "pre-arm hold; the read-back must name the stall"
    )


@pytest.mark.asyncio
async def test_a_stalled_attitude_alone_stops_commanding(
    monkeypatch: Any,
) -> None:
    # ATTITUDE and GLOBAL_POSITION_INT are separate messages at separate
    # rates and either can stop on its own. A single shared freshness mark
    # would be refreshed by whichever one still flowed, hiding the stall
    # completely — so each half is aged independently.
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)

    clock.advance(1.0)
    # Position keeps arriving; attitude does not.
    plugin._on_global_position(
        {
            "lat": int(round(12.0 * 1e7)),
            "lon": int(round(77.0 * 1e7)),
            "relative_alt": 20_000,
        }
    )
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    assert not ctx.flight.setpoints, (
        "a stalled attitude alone must stop the follow; a still-flowing "
        "position message must not mask it"
    )
    states = _state_events(ctx)
    assert states and states[-1]["hold_reason"] == HOLD_POSE_STALE


@pytest.mark.asyncio
async def test_a_stalled_position_alone_stops_commanding(
    monkeypatch: Any,
) -> None:
    # The mirror image: attitude keeps flowing, position dries up.
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)

    clock.advance(1.0)
    plugin._on_attitude({"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    assert not ctx.flight.setpoints, "a stalled position alone must stop the follow"
    states = _state_events(ctx)
    assert states and states[-1]["hold_reason"] == HOLD_POSE_STALE


@pytest.mark.asyncio
async def test_the_follow_resumes_when_pose_telemetry_comes_back(
    monkeypatch: Any,
) -> None:
    # Staleness is a hold, not a latch of its own: the loop must command
    # again as soon as current telemetry resumes, with no re-designate.
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    clock.advance(1.0)
    await plugin._tick()
    assert not ctx.flight.setpoints

    # Telemetry resumes.
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(
        plugin,
        track_id=7,
        lock_state=LOCK_LOCKED,
        bbox=BoundingBox(316.0, 236.0, 10.0, 10.0),
        at=clock.now,
    )
    await plugin._tick()

    assert ctx.flight.setpoints, "current telemetry must resume the follow"
    states = _state_events(ctx)
    assert states[-1]["commanding"] is True
    assert states[-1]["hold_reason"] is None


@pytest.mark.asyncio
async def test_a_stalled_heartbeat_stops_commanding_and_clears_the_arm_readback(
    monkeypatch: Any,
) -> None:
    # armed/guided are readings of the FC as of the last HEARTBEAT, not
    # standing facts. Once it stops arriving nothing confirms the aircraft is
    # still armed or still in a mode that accepts setpoints, so the loop must
    # hold AND the read-back must stop asserting an arm state.
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    # Past the heartbeat window. Pose and lock are refreshed so the heartbeat
    # is the only stale input.
    clock.advance(4.0)
    _level_pose(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    assert not ctx.flight.setpoints, "a silent flight controller must not be commanded"
    states = _state_events(ctx)
    assert states and states[-1]["commanding"] is False
    assert states[-1]["hold_reason"] == HOLD_FC_STALE
    assert states[-1]["fc_armed"] is False, (
        "a remembered arm state is not an observed one"
    )
    assert states[-1]["fc_guided"] is False


@pytest.mark.asyncio
async def test_a_stale_heartbeat_is_reported_apart_from_a_disarmed_one(
    monkeypatch: Any,
) -> None:
    # Both read fc_armed: false, and they mean very different things — one is
    # a normal pre-flight state, the other is a lost link mid-follow.
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    plugin._on_heartbeat({"base_mode": 0x01, "custom_mode": 4, "autopilot": 3})
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    states = _state_events(ctx)
    assert states[-1]["hold_reason"] == HOLD_FC_DISARMED
    assert states[-1]["fc_guided"] is True, "a live heartbeat still reports its mode"


@pytest.mark.asyncio
async def test_a_non_guided_mode_is_named_distinctly(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin, custom_mode=5)  # ArduCopter LOITER
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    states = _state_events(ctx)
    assert states[-1]["hold_reason"] == HOLD_FC_NOT_GUIDED


@pytest.mark.asyncio
async def test_a_stalled_gimbal_report_falls_back_to_the_commanded_angle(
    monkeypatch: Any,
) -> None:
    # A MOUNT_ORIENTATION report is the boresight only while it is current.
    # Once it stops arriving, the angle the plugin itself last commanded
    # tracks the gimbal's real aim better than a report that predates it.
    cfg = _telemetry_config()
    cfg["gimbal_point"] = True
    cfg["mount_pitch_deg"] = 30.0
    ctx = _Ctx(config=cfg)
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    plugin._on_mount_orientation({"pitch": -40.0, "yaw": 15.0})
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()
    assert plugin._commanded_gimbal is not None
    commanded = plugin._commanded_gimbal
    assert commanded != (40.0, 15.0), "the test needs the two angles to differ"

    # The gimbal stops reporting. Everything else is refreshed.
    clock.advance(2.5)
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(
        plugin,
        track_id=7,
        lock_state=LOCK_LOCKED,
        bbox=BoundingBox(320.0, 240.0, 12.0, 12.0),
        at=clock.now,
    )

    captured = _spy_projection(monkeypatch)
    await plugin._tick()

    assert (captured["gimbal_pitch_deg"], captured["gimbal_yaw_deg"]) == commanded, (
        "a stale gimbal report must decay to the commanded angle, not be "
        "believed indefinitely"
    )


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
    assert plugin._tracker.has_lock is False

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

    now = time.monotonic()
    assert plugin._tracker.track_id == 7
    assert plugin._tracker.effective_lock(now) is EffectiveLock.LOCKED
    target = plugin._tracker.locked_target(now)
    assert target is not None and target.bbox == tracked.bbox


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

    now = time.monotonic()
    assert plugin._tracker.track_id == 4
    assert plugin._tracker.effective_lock(now) is EffectiveLock.UNCERTAIN


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

    assert plugin._tracker.has_lock is False


def test_batch_from_a_different_camera_is_filtered_out() -> None:
    # The plugin subscribes to every camera and filters to the configured
    # designate camera locally, so a batch from another camera never locks.
    ctx = _Ctx(config={"designate_camera": "uvc-0"})
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"

    other = DetectionBatch(
        model_id="coco-person",
        camera_id="uvc-1",  # not the configured designate camera
        frame_id=1,
        ts_ms=0,
        detections=[
            Detection(
                bbox=BoundingBox(100, 100, 20, 20),
                class_label="person",
                confidence=0.9,
                track_id=7,
                lock_state=LOCK_LOCKED,
            )
        ],
    )
    plugin._on_batch(other)
    assert plugin._tracker.has_lock is False

    # Retargeting to that camera at runtime lets its batches through with no
    # re-subscription.
    plugin._active_camera = "uvc-1"
    plugin._on_batch(other)
    assert plugin._tracker.track_id == 7


def test_follow_state_topic_constant_matches_manifest() -> None:
    # The agent publishes its read-back on FOLLOW_STATE_TOPIC; it must be the
    # literal the manifest skill state.topic declares. Designation is engine-
    # owned now (the GCS overlay locks the engine via the host vision.designate
    # command), so the agent half carries no designate topic.
    assert FOLLOW_STATE_TOPIC == "follow.state"


class _Tools:
    def __init__(self) -> None:
        self.handlers: dict[str, Any] = {}

    def register(self, name: str, handler: Any) -> None:
        self.handlers[name] = handler


def test_camera_selector_resolution() -> None:
    plugin = _make_plugin(_Ctx())
    # By-requirement (auto / empty / None) accepts any camera (None filter).
    assert plugin._resolve_designate_camera("auto") is None
    assert plugin._resolve_designate_camera("") is None
    assert plugin._resolve_designate_camera(None) is None
    # A pinned id filters to that camera.
    assert plugin._resolve_designate_camera("uvc-1") == "uvc-1"
    assert plugin._resolve_designate_camera(" uvc-2 ") == "uvc-2"


def test_default_designate_camera_accepts_any_camera() -> None:
    # The manifest default is "auto", so a fresh install follows the designated
    # subject on whatever camera the engine feeds (no hard-coded uvc-0 filter).
    assert FollowConfig().designate_camera == "auto"
    plugin = _make_plugin(_Ctx())
    assert plugin._resolve_designate_camera(FollowConfig().designate_camera) is None


@pytest.mark.asyncio
async def test_stop_follow_tool_disarms_only() -> None:
    ctx = _Ctx(config={"active": True})
    plugin = _make_plugin(ctx)
    plugin._register_tools(ctx)  # ctx has no tools surface -> no-op
    # Call the handler directly (the host would route tool.invoke to it).
    result = await plugin._tool_stop_follow({})
    assert result == {"ok": True, "active": False}
    assert await ctx.config_kv.get("active", None) is False


@pytest.mark.asyncio
async def test_follow_status_tool_reports_state() -> None:
    ctx = _Ctx()
    plugin = _make_plugin(ctx)
    status = await plugin._tool_follow_status({})
    # The read-back shape the GCS + an assistant consume.
    assert "active" in status
    assert "commanding" in status
    assert "lock_state" in status


def test_tools_registered_when_ctx_exposes_them() -> None:
    ctx = _Ctx()
    ctx.tools = _Tools()
    plugin = _make_plugin(ctx)
    plugin._register_tools(ctx)
    assert set(ctx.tools.handlers) == {"follow_status", "stop_follow"}
