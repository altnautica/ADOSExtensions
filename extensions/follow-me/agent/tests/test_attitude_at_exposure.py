"""A box is projected through the vehicle attitude its frame was exposed at.

A detection batch reaches the follow loop about 0.4 s after its frame was
exposed. A following drone pitches by 10° or more in that time; projecting the
box through the newer attitude moves the subject's ground range by metres, the
follow goal jumps with it, and the drone pitches again: the follow pulses.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from ados.sdk.vision import BoundingBox
from follow_fakes import (
    _attitude,
    _Ctx,
    _global_position,
    _install_clock,
    _local_position,
    _make_plugin,
)

from follow_me.fc_state import Pose
from follow_me.loop import Sighting
from follow_me.state import FollowConfig

# The bench's front camera: 87° wide, 15° below the nose, 1280x720.
_CFG = FollowConfig(camera_hfov_deg=87.0, mount_pitch_deg=15.0)
_W, _H = 1280, 720
# A walker 8 m ahead of a drone 2.5 m up: feet 2.4° below the optical axis.
_WALKER = BoundingBox(x=615.0, y=250.0, width=55.0, height=138.0)


def _follow_pose(plugin: Any) -> None:
    """Level, 2.5 m above the ground, as the follow flights fly."""
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    plugin._on_local_position(_local_position(0.0, 0.0, -2.5))
    plugin._on_global_position(_global_position(12.0, 77.0, 2.5))


def test_the_pose_interpolates_its_attitude_history() -> None:
    p = Pose()
    assert p.attitude_at(5.0) == (0.0, 0.0, 0.0)
    p.set_attitude(0.0, -0.1, 3.10, 10.00)
    p.set_attitude(0.1, -0.2, -3.12, 10.05)
    roll, pitch, yaw = p.attitude_at(10.025)
    assert roll == pytest.approx(0.05) and pitch == pytest.approx(-0.15)
    # Across the ±π seam the short way.
    assert yaw == pytest.approx(3.10 + 0.5 * (2 * math.pi - 6.22))
    latest = (0.1, -0.2, -3.12)
    # Before the history, or past the newest sample by more than the hold:
    # the latest attitude.
    assert p.attitude_at(9.0) == latest
    assert p.attitude_at(10.12) == pytest.approx(latest)
    assert p.attitude_at(10.2) == latest
    # Inside a gap no sample bounds closely: the latest.
    p.set_attitude(0.3, 0.0, 0.0, 10.5)
    assert p.attitude_at(10.3) == (0.3, 0.0, 0.0)
    # The clock went back (a restart of the plugin's clock): start over.
    p.set_attitude(0.4, 0.0, 0.0, 1.0)
    assert p.attitude_at(10.05) == (0.4, 0.0, 0.0)


def _pitch_down(plugin: Any, clock: Any, seconds: float, rad: float) -> None:
    """Pitch the nose down to ``rad`` over ``seconds``, ATTITUDE at 20 Hz."""
    steps = round(seconds / 0.05)
    for k in range(1, steps + 1):
        clock.advance(0.05)
        plugin._on_attitude(_attitude(0.0, -rad * k / steps, 0.0))


def test_a_box_is_projected_through_the_attitude_at_its_exposure(
    monkeypatch: Any,
) -> None:
    clock = _install_clock(monkeypatch)
    plugin = _make_plugin(_Ctx())
    _follow_pose(plugin)
    clock.advance(0.05)
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    exposed = clock.now
    level = plugin._measure_box(_CFG, clock.now, _WALKER, _W, _H)
    assert level is not None
    # The batch with that frame's box arrives 0.4 s later; the nose went
    # 0.2 rad (11.5°) down meanwhile.
    _pitch_down(plugin, clock, 0.4, 0.2)
    stale = plugin._measure_box(_CFG, clock.now, _WALKER, _W, _H)
    at_exposure = plugin._measure_box(_CFG, clock.now, _WALKER, _W, _H, exposed)
    assert stale is not None and at_exposure is not None
    assert at_exposure.offset_north_m == pytest.approx(level.offset_north_m, abs=1e-6)
    assert at_exposure.offset_east_m == pytest.approx(level.offset_east_m, abs=1e-6)
    assert at_exposure.range_m == pytest.approx(level.range_m, abs=1e-6)

    # Through the newer attitude the feet's ground intersection is off by
    # metres, and the fused range by a metre.
    def ground(fix: Any) -> float:
        return next(c.range_m for c in fix.cues if c.kind == "ground")

    assert abs(ground(stale) - ground(level)) > 2.0, (ground(stale), ground(level))
    assert abs(stale.range_m - level.range_m) > 0.8, (stale.range_m, level.range_m)


def test_a_sighting_is_measured_at_its_frames_exposure(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    plugin = _make_plugin(_Ctx())
    _follow_pose(plugin)
    clock.advance(0.05)
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    level = plugin._measure_box(_CFG, clock.now, _WALKER, _W, _H)
    assert level is not None
    ts_ms = int(clock.now * 1000.0)
    _pitch_down(plugin, clock, 0.4, 0.2)
    plugin._sighting = Sighting(
        track_id=3,
        bbox=_WALKER,
        camera_id="main",
        frame_wh=(_W, _H),
        ts_ms=ts_ms,
        received_s=clock.now,
        confidence=0.9,
        embedding=None,
    )
    plugin._measure_sighting(_CFG, clock.now, _WALKER)
    fix = plugin._last_fix
    assert fix is not None
    assert fix.range_m == pytest.approx(level.range_m, abs=1e-3)
