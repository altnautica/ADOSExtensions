"""Plugin tests: detection routing, camera selection, MCP tools and the MAVLink
handlers.
"""

from __future__ import annotations

import time

import pytest

from ados.sdk.tracking import EffectiveLock
from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    LOCK_LOCKED,
    LOCK_UNCERTAIN,
    FollowConfig,
)
from follow_fakes import (
    _Ctx,
    _attitude,
    _global_position,
    _heartbeat,
    _mount_orientation,
    _local_position,
    _distance_sensor,
    _make_plugin,
    _Tools,
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


def test_mavlink_handlers_read_the_delivered_wire_frame() -> None:
    # The host delivers raw frames, not decoded fields: every pose handler has
    # to decode what it is given.
    plugin = _make_plugin(_Ctx())
    plugin._on_attitude(_attitude(0.1, -0.2, 1.5))
    plugin._on_global_position(_global_position(12.5, 77.25, 18.0))
    plugin._on_local_position(_local_position(3.0, -4.0, -18.0, vn=1.0))
    plugin._on_distance_sensor(_distance_sensor(17.5))
    assert plugin._pose.north_m == pytest.approx(3.0)
    assert plugin._pose.down_m == pytest.approx(-18.0)
    assert plugin._pose.vn_mps == pytest.approx(1.0)
    assert plugin._rangefinder.distance_m == pytest.approx(17.5)
    assert plugin._rangefinder.max_m == pytest.approx(12.0)
    plugin._on_mount_orientation(_mount_orientation(-30.0, 10.0))
    plugin._on_heartbeat(_heartbeat(base_mode=0x81, custom_mode=4, autopilot=3))
    assert plugin._pose.yaw == pytest.approx(1.5)
    assert plugin._pose.lat_deg == pytest.approx(12.5)
    assert plugin._pose.lon_deg == pytest.approx(77.25)
    assert plugin._pose.rel_alt_m == pytest.approx(18.0)
    assert plugin._gimbal.pitch_deg == pytest.approx(30.0)
    assert plugin._fc.armed and plugin._fc.guided


def test_a_non_autopilot_heartbeat_does_not_change_the_arm_state() -> None:
    plugin = _make_plugin(_Ctx())
    plugin._on_heartbeat(_heartbeat(base_mode=0x81, custom_mode=4, autopilot=3))
    # A ground station or gimbal heartbeat (MAV_AUTOPILOT_INVALID) says nothing
    # about the vehicle; it must not read as disarmed.
    plugin._on_heartbeat(_heartbeat(base_mode=0, custom_mode=0, autopilot=8))
    assert plugin._fc.armed and plugin._fc.guided


def test_an_undecodable_frame_is_ignored() -> None:
    plugin = _make_plugin(_Ctx())
    plugin._on_attitude(_attitude(0.0, 0.0, 0.4))
    truncated = _attitude(0.0, 0.0, 2.0)
    truncated["frame"] = truncated["frame"][:-3]
    plugin._on_attitude(truncated)
    assert plugin._pose.yaw == pytest.approx(0.4)


@pytest.mark.parametrize(
    ("mav_type", "custom_mode", "guided"),
    [
        (2, 4, True),  # Copter GUIDED
        (2, 20, True),  # Copter GUIDED_NOGPS
        (2, 15, False),  # Copter AUTOROTATE, not guided
        (1, 15, True),  # Plane GUIDED
        (20, 15, True),  # QuadPlane (VTOL quadrotor) GUIDED
        (1, 4, False),  # Plane ACRO
        (1, 20, False),  # Plane QLAND
        (20, 20, False),  # QuadPlane QLAND
        (10, 15, True),  # Rover GUIDED
        (10, 4, False),  # Rover HOLD
        (12, 4, False),  # Submarine: no table decoded
        (0, 4, False),  # MAV_TYPE_GENERIC: unknown vehicle
    ],
)
def test_guided_mode_is_read_from_the_vehicle_mode_table(
    mav_type: int, custom_mode: int, guided: bool
) -> None:
    plugin = _make_plugin(_Ctx())
    plugin._on_heartbeat(
        _heartbeat(
            base_mode=0x81, custom_mode=custom_mode, autopilot=3, mav_type=mav_type
        )
    )
    assert plugin._fc.guided is guided
