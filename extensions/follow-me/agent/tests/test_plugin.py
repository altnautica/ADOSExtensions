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

import pytest

from follow_me.state import (
    LOCK_LOCKED,
    LOCK_LOST,
    LOCK_UNCERTAIN,
)
from follow_fakes import (
    _Ctx,
    _heartbeat,
    _level_pose,
    _arm_guided,
    _make_plugin,
    _seed_lock,
    _state_events,
)


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
# A locked, active, posed follow only commands when the FC is armed AND the
# pilot grants the motion lease.
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_refused_lease_holds_without_commanding() -> None:
    ctx = _Ctx(
        config={
            "active": True,
            "gimbal_point": False,
            "designate_camera": "uvc-0",
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    # Armed in LOITER (5); the pilot refuses the lease.
    _arm_guided(plugin, custom_mode=5)
    ctx.flight.acquire_reply = {"ok": False, "err": "not_guided_capable"}

    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)

    await plugin._tick()

    assert not ctx.flight.setpoints, "no goal without the pilot's lease"
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
        }
    )
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    # Guided mode selected, but the disarmed bit is clear in base_mode.
    plugin._on_heartbeat(_heartbeat(base_mode=0x01, custom_mode=4, autopilot=3))

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
