"""The camera is aimed along the latest sighting, and an unsure lock stops the
estimate.

The follow goal's position comes from the subject estimate, which carries
the range's noise; its yaw comes from the latest sighting's measured line of
sight, so a wrong range or velocity cannot turn the subject out of the frame.
While the lock is unsure the vehicle holds, still facing that sighting, and
the estimate stops where it is instead of extrapolating its last velocity.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from ados.sdk.vision import BoundingBox
from follow_fakes import (
    _arm_guided,
    _attitude,
    _Ctx,
    _distance_sensor,
    _global_position,
    _install_clock,
    _local_position,
    _make_plugin,
    _seed_lock,
)
from follow_me.state import LOCK_LOCKED

_CONFIG = {
    "active": True,
    "gimbal_point": False,
    "designate_camera": "uvc-0",
    "follow_distance_m": 8.0,
    "follow_height_m": 2.5,
}
# A walker about 8 m ahead, left of the image centre (frame 640x480 by
# default when the batch carries no size).
_LEFT = BoundingBox(x=180.0, y=200.0, width=28.0, height=70.0)
_LEFT_MOVED = BoundingBox(x=200.0, y=200.0, width=28.0, height=70.0)


def _pose(plugin: Any, yaw: float = 0.0) -> None:
    plugin._on_attitude(_attitude(0.0, 0.0, yaw))
    plugin._on_local_position(_local_position(0.0, 0.0, -2.5))
    plugin._on_global_position(_global_position(12.0, 77.0, 2.5))
    plugin._on_distance_sensor(_distance_sensor(2.5))
    _arm_guided(plugin)


async def _sight(plugin: Any, clock: Any, bbox: BoundingBox, lock_state: str = LOCK_LOCKED) -> None:
    _seed_lock(plugin, track_id=7, lock_state=lock_state, bbox=bbox, at=clock.now)
    await plugin._tick()


def _estimate_bearing(plugin: Any, now: float) -> float:
    x, _p = plugin._filter.predicted(now)
    return math.atan2(x[1] - plugin._pose.east_m, x[0] - plugin._pose.north_m)


@pytest.mark.asyncio
async def test_the_goal_yaw_follows_the_sighting_not_the_estimate(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _Ctx(config=_CONFIG)
    plugin = _make_plugin(ctx)
    _pose(plugin)
    # An estimate from earlier sightings that sits well right of the subject.
    plugin._filter_track_id = 7
    plugin._filter.update(clock.now, 4.0, 6.9, ((0.01, 0.0), (0.0, 0.01)))
    clock.advance(0.05)
    _pose(plugin)
    await _sight(plugin, clock, _LEFT)

    goal = ctx.flight.setpoints[-1]
    fix = plugin._last_fix
    assert goal["kind"] == "follow" and fix is not None
    assert goal["yaw"] == pytest.approx(fix.bearing_rad)
    # The subject is left of the nose; the estimate is still pulled right.
    assert fix.bearing_rad < 0.0
    assert _estimate_bearing(plugin, clock.now) - fix.bearing_rad > 0.2


@pytest.mark.asyncio
async def test_an_unsure_hold_faces_the_sighting_and_stops_the_estimate(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _Ctx(config=_CONFIG)
    plugin = _make_plugin(ctx)
    _pose(plugin)
    await _sight(plugin, clock, _LEFT)
    clock.advance(0.33)
    _pose(plugin)
    await _sight(plugin, clock, _LEFT_MOVED)
    bearing = plugin._last_fix.bearing_rad
    x, _p = plugin._filter.predicted(clock.now)
    assert math.hypot(x[2], x[3]) > 0.1

    # The lock turns unsure while the vehicle has yawed away.
    clock.advance(0.33)
    _pose(plugin, yaw=1.0)
    at_hold, _p = plugin._filter.predicted(clock.now)
    await _sight(plugin, clock, _LEFT_MOVED, lock_state="uncertain")
    hold = ctx.flight.setpoints[-1]
    assert hold["kind"] == "position"
    assert hold["yaw"] == pytest.approx(bearing)
    x, _p = plugin._filter.predicted(clock.now + 1.0)
    assert x[2:] == [0.0, 0.0]
    assert x[0] == pytest.approx(at_hold[0]) and x[1] == pytest.approx(at_hold[1])

    # A sighting older than the aiming window no longer steers the hold.
    clock.advance(3.1)
    _pose(plugin, yaw=1.0)
    await _sight(plugin, clock, _LEFT_MOVED, lock_state="uncertain")
    hold = ctx.flight.setpoints[-1]
    assert hold["kind"] == "position"
    assert hold["yaw"] == pytest.approx(1.0)
