"""The distance setpoint in the read-back belongs to the follow mode only.

In orbit the geometry flown is the orbit radius, so the read-back must not
also carry the follow distance as a "distance setpoint".
"""

from __future__ import annotations

import pytest

from follow_me.state import LOCK_LOCKED
from follow_fakes import (
    _Ctx,
    _arm_guided,
    _level_pose,
    _make_plugin,
    _seed_lock,
    _state_events,
)


def _config(mode: str) -> dict:
    return {
        "active": True,
        "follow_mode": mode,
        "follow_distance_m": 8.0,
        "follow_height_m": 4.0,
        "orbit_radius_m": 10.5,
        "gimbal_point": False,
        "designate_camera": "uvc-0",
    }


async def _commanding_state(mode: str) -> dict:
    ctx = _Ctx(config=_config(mode))
    plugin = _make_plugin(ctx)
    _level_pose(plugin)
    _arm_guided(plugin)
    _seed_lock(plugin, track_id=7, lock_state=LOCK_LOCKED)
    await plugin._tick()
    state = _state_events(ctx)[-1]
    assert state["commanding"] is True
    return state


@pytest.mark.asyncio
async def test_follow_mode_publishes_the_distance_setpoint() -> None:
    state = await _commanding_state("follow")
    assert state["distance_setpoint_m"] == 8.0
    assert state["height_setpoint_m"] == 4.0


@pytest.mark.asyncio
async def test_orbit_mode_publishes_no_follow_distance_setpoint() -> None:
    state = await _commanding_state("orbit")
    assert state.get("distance_setpoint_m") is None
    assert state["orbit_radius_m"] == 10.5
    assert state["height_setpoint_m"] == 4.0
