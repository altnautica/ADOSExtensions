"""Plugin tests: projection inputs (frame size, gimbal attitude, frozen boxes)
and telemetry freshness: the loop projects only through current vehicle state.
"""

from __future__ import annotations

from typing import Any

import pytest

from ados.sdk.vision import BoundingBox, Detection, DetectionBatch

from follow_me.state import (
    HOLD_FC_DISARMED,
    HOLD_LEASE_REFUSED,
    HOLD_FC_STALE,
    HOLD_POSE_STALE,
    LOCK_LOCKED,
)
from follow_fakes import (
    _Ctx,
    _attitude,
    _heartbeat,
    _mount_orientation,
    _local_position,
    _level_pose,
    _arm_guided,
    _make_plugin,
    _install_clock,
    _seed_lock,
    _state_events,
    _telemetry_config,
    _spy_projection,
)


# ---------------------------------------------------------------------------
# Projection inputs: real frame size, real gimbal attitude, and the coast hold.
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_real_frame_size_from_the_batch_feeds_the_projection(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
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
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    _arm_guided(plugin)
    # MOUNT_ORIENTATION reports the gimbal pitched 40 deg down (negative = down)
    # and yawed 15 deg to the right of the nose.
    plugin._on_mount_orientation(_mount_orientation(-40.0, 15.0))

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    captured = _spy_projection(monkeypatch)
    await plugin._tick()

    # The reported attitude (converted to projection convention: +40 down) is
    # the boresight; the fixed mount pitch does not stack on top of it.
    assert captured["gimbal_pitch_deg"] == 40.0
    assert captured["gimbal_yaw_deg"] == 15.0
    assert captured["mount_pitch_deg"] == 0.0


def _gimbal_point_ctx() -> _Ctx:
    return _Ctx(
        config={
            "active": True,
            "gimbal_point": True,
            "designate_camera": "uvc-0",
            "follow_distance_m": 6.0,
            "follow_height_m": 4.0,
        }
    )


async def _two_sightings(plugin: Any, clock: Any, monkeypatch: Any) -> dict[str, Any]:
    """Tick on one sighting (which commands the gimbal), then project a
    second, fresh one (a new bbox, so not coasting); what that projection
    was given."""
    _seed_lock(
        plugin, track_id=7, lock_state=LOCK_LOCKED, bbox=BoundingBox(315.0, 235.0, 10.0, 10.0), at=clock.now
    )
    await plugin._tick()
    assert plugin._commanded_gimbal is not None
    clock.advance(0.05)
    captured = _spy_projection(monkeypatch)
    _seed_lock(
        plugin, track_id=7, lock_state=LOCK_LOCKED, bbox=BoundingBox(320.0, 240.0, 12.0, 12.0), at=clock.now
    )
    await plugin._tick()
    return captured


@pytest.mark.asyncio
async def test_falls_back_to_the_last_commanded_gimbal_angle(
    monkeypatch: Any,
) -> None:
    clock = _install_clock(monkeypatch)
    plugin = _make_plugin(_gimbal_point_ctx())
    # A gimbal is on board: it reported once, then the report aged out.
    plugin._on_mount_orientation(_mount_orientation(-30.0, 0.0))
    clock.advance(2.5)
    _level_pose(plugin)
    _arm_guided(plugin)

    # The first sighting bootstraps on the fixed mount pitch and commands the
    # gimbal; the second, with still no current report, is projected through
    # the commanded angle, not the mount.
    captured = await _two_sightings(plugin, clock, monkeypatch)

    assert (captured["gimbal_pitch_deg"], captured["gimbal_yaw_deg"]) == (
        plugin._commanded_gimbal
    )
    assert captured["mount_pitch_deg"] == 0.0


@pytest.mark.asyncio
async def test_a_fixed_camera_is_projected_through_its_mount_whatever_was_commanded(
    monkeypatch: Any,
) -> None:
    # No gimbal has ever reported: the pointing command goes nowhere and the
    # camera still looks along its mount. Projecting through the commanded
    # angle would aim each measurement at the previous estimate.
    clock = _install_clock(monkeypatch)
    plugin = _make_plugin(_gimbal_point_ctx())
    _level_pose(plugin)
    _arm_guided(plugin)

    captured = await _two_sightings(plugin, clock, monkeypatch)

    # The reference front camera's mount, 15° below the nose.
    assert captured["mount_pitch_deg"] == 15.0
    assert (captured["gimbal_pitch_deg"], captured["gimbal_yaw_deg"]) == (0.0, 0.0)


@pytest.mark.asyncio
async def test_coasting_holds_the_last_setpoint_without_recomputing(
    monkeypatch: Any,
) -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
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
    plugin._on_attitude(_attitude(0.0, 0.0, 1.0))
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
    plugin._on_local_position(_local_position(0.0, 0.0, -20.0))
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
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
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
    plugin._on_heartbeat(_heartbeat(base_mode=0x01, custom_mode=4, autopilot=3))
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)

    await plugin._tick()

    states = _state_events(ctx)
    assert states[-1]["hold_reason"] == HOLD_FC_DISARMED
    assert states[-1]["fc_guided"] is True, "a live heartbeat still reports its mode"


@pytest.mark.asyncio
async def test_a_refused_lease_is_named_distinctly(monkeypatch: Any) -> None:
    # The pilot owns the mode; when it cannot hand the vehicle to a skill the
    # refusal is the hold reason, not a silent stall.
    ctx = _Ctx(config=_telemetry_config())
    ctx.flight.acquire_reply = {"ok": False, "err": "not_guided_capable"}
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin, custom_mode=5)  # ArduCopter LOITER
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED, at=clock.now)
    plugin._filter.update(clock.now, 10.0, 0.0, [[0.25, 0.0], [0.0, 0.25]])

    await plugin._tick()

    states = _state_events(ctx)
    assert states[-1]["hold_reason"] == HOLD_LEASE_REFUSED


@pytest.mark.asyncio
async def test_a_stalled_gimbal_report_falls_back_to_the_commanded_angle(
    monkeypatch: Any,
) -> None:
    # A MOUNT_ORIENTATION report is the boresight only while it is current.
    # Once it stops arriving, the angle the plugin itself last commanded
    # tracks the gimbal's real aim better than a report that predates it.
    cfg = _telemetry_config()
    cfg["gimbal_point"] = True
    ctx = _Ctx(config=cfg)
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    plugin._on_mount_orientation(_mount_orientation(-40.0, 15.0))
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
