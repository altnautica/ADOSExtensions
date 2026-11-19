"""A subject faster than the follow can fly, judged against the limit
actually in force (the follow's speed cap, or the pilot's GNSS-denied cap)
on the subject's measured speed; the reach bound along the track; and a
frame-reset check that a hard turn between sparse samples does not trip."""

from __future__ import annotations

from typing import Any

import pytest
from follow_fakes import _Ctx, _gps_raw, _install_clock, _local_position, _state_events
from follow_me.speed_limit import (
    GNSS_DENIED_MAX_SPEED_MPS,
    NavFeed,
    SubjectMotion,
    effective_limit,
)
from follow_me.state import PHASE_FOLLOWING
from follow_me.target_filter import TargetFilter
from test_follow_v03 import _follow_one
from test_recovery import _cfg, _plugin

_R = ((0.25, 0.0), (0.0, 0.25))


# -- the limit in force ---------------------------------------------------------


def test_without_a_3d_fix_the_limit_is_the_gnss_denied_cap() -> None:
    feed = NavFeed()
    assert effective_limit(6.0, feed, 10.0) == GNSS_DENIED_MAX_SPEED_MPS == 2.0
    feed.on_gps_fix(2, 10.0)
    assert effective_limit(6.0, feed, 10.0) == 2.0
    feed.on_gps_fix(3, 10.0)
    assert effective_limit(6.0, feed, 10.0) == 6.0
    # A fix that stopped arriving no longer counts.
    assert effective_limit(6.0, feed, 13.5) == 2.0
    # The follow's own cap still applies when it is the lower.
    feed.on_gps_fix(3, 20.0)
    assert effective_limit(1.5, feed, 20.0) == 1.5


def test_external_nav_on_the_gps_source_set_is_uncapped_as_the_pilot_flies_it() -> None:
    # the navigation extension in its external-navigation mode with the EKF on source set 1
    # and a 3-D fix: the pilot flies uncapped, so the limit is the follow's.
    feed = NavFeed()
    feed.on_gps_fix(6, 10.0)
    feed.on_nav_state({"mode": "external_nav", "active_set": 1, "gps_fix": 6}, 10.0)
    assert effective_limit(6.0, feed, 10.0) == 6.0


def test_the_source_set_is_the_last_one_named_and_a_set_outside_1_to_3_is_capped() -> None:
    feed = NavFeed()
    feed.on_gps_fix(3, 10.0)
    # Before any nav.state, the boot set 1.
    assert effective_limit(6.0, feed, 10.0) == 6.0
    feed.on_nav_state({"mode": "external_nav", "active_set": 2}, 10.0)
    assert effective_limit(6.0, feed, 10.0) == 6.0
    feed.on_nav_state({"mode": "external_nav", "active_set": 4}, 10.5)
    assert effective_limit(6.0, feed, 10.5) == 2.0
    # A nav.state without a set keeps the last one named, even once it is old.
    feed.on_nav_state({"mode": "off", "active_set": None}, 11.0)
    feed.on_gps_fix(3, 20.0)
    assert effective_limit(6.0, feed, 20.0) == 2.0
    feed.on_nav_state({"mode": "off", "active_set": 1}, 20.0)
    assert effective_limit(6.0, feed, 20.0) == 6.0
    # Without a fresh 3-D fix the cap applies on any set.
    assert effective_limit(6.0, feed, 23.5) == 2.0


# -- the subject's measured motion ---------------------------------------------


def test_measured_speed_is_not_clamped_by_the_person_cap() -> None:
    m = SubjectMotion()
    for k in range(7):
        m.record(k * 0.33, 3.8 * k * 0.33, 0.0, 8.0)
    speed, opening = m.rates(2.0)
    assert speed == pytest.approx(3.8, abs=0.05)
    assert opening == pytest.approx(0.0, abs=1e-9)
    # Too short a window, or one gone stale, gives nothing.
    short = SubjectMotion()
    short.record(0.0, 0.0, 0.0, 8.0)
    short.record(0.3, 1.0, 0.0, 8.0)
    assert short.rates(0.3) is None
    assert m.rates(2.0 + 5.0) is None


async def _fixes(
    plugin: Any,
    clock: Any,
    *,
    speed: float,
    opening: float = 0.0,
    seconds: float = 4.0,
    gps_fix: int | None = None,
) -> None:
    """Feed measured subject positions moving north at ``speed`` and a range
    opening at ``opening`` at 3 Hz, publishing the read-back as the loop
    would; with ``gps_fix``, a GPS_RAW_INT of that fix type each time."""
    plugin._subject_motion.clear()
    n0 = 10.0
    t0 = clock.now
    for k in range(int(seconds / 0.33) + 1):
        t = t0 + k * 0.33
        clock.now = t
        if gps_fix is not None:
            plugin._on_gps_raw(_gps_raw(gps_fix))
        plugin._subject_motion.record(t, n0 + speed * (t - t0), 0.0, 8.0 + opening * (t - t0))
        await plugin._publish_state(
            plugin._cfg, now=t, active=True, phase=PHASE_FOLLOWING, commanding=True
        )


@pytest.mark.asyncio
async def test_a_runner_faster_than_the_gnss_denied_cap_is_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=6.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    # No GPS fix: the pilot flies at 2 m/s at most, the subject runs 3.8.
    await _fixes(plugin, clock, speed=3.8)
    st = _state_events(ctx)[-1]
    assert st["speed_limit_mps"] == 2.0 and st["outpaced"] is True


@pytest.mark.asyncio
async def test_a_runner_within_the_limit_on_gnss_is_not_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=6.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _fixes(plugin, clock, speed=3.8, gps_fix=3)
    st = _state_events(ctx)[-1]
    assert st["speed_limit_mps"] == 6.0 and st["outpaced"] is False


@pytest.mark.asyncio
async def test_a_sprinter_above_the_follow_cap_is_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=3.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _fixes(plugin, clock, speed=4.5, gps_fix=3)
    st = _state_events(ctx)[-1]
    # The person filter would clamp its estimate at 3 m/s; the measured speed
    # is not clamped.
    assert st["speed_limit_mps"] == 3.0 and st["outpaced"] is True


@pytest.mark.asyncio
async def test_the_range_opening_while_flying_at_the_limit_is_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=6.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    # Flying north at the 2 m/s cap; the subject's measured speed looks like
    # 1.9 m/s (range noise), yet the range keeps opening.
    plugin._pose.vn_mps = 2.0
    await _fixes(plugin, clock, speed=1.9, opening=0.6)
    assert _state_events(ctx)[-1]["outpaced"] is True


@pytest.mark.asyncio
async def test_a_brief_burst_is_not_reported(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg(max_speed_mps=6.0))
    plugin = _plugin(ctx)
    clock = _install_clock(monkeypatch)
    await _follow_one(plugin, clock)
    await _fixes(plugin, clock, speed=3.8, seconds=2.0)
    assert _state_events(ctx)[-1]["outpaced"] is False


# -- the reach bound along the track -------------------------------------------


def test_the_gate_rejects_a_point_beyond_reach_along_the_track() -> None:
    f = TargetFilter()
    f.max_speed_mps = 3.0
    for k in range(10):
        f.update(k * 0.33, 1.4 * k * 0.33, 0.0, _R)
    last = f.last_estimate()
    assert last is not None
    reach = 3.0 * 10.0 + 3.0 * f.last_sigma_m
    t = f.last_update_s + 10.0
    d = f.mahalanobis(t, last[0] + reach + 10.0, last[1], _R)
    assert d is not None and d > 3.0


# -- a hard turn between sparse pose samples -----------------------------------


@pytest.mark.asyncio
async def test_a_hard_turn_between_sparse_samples_is_not_a_reset(monkeypatch: Any) -> None:
    ctx = _Ctx(config=_cfg())
    plugin = _plugin(ctx)
    _install_clock(monkeypatch)
    plugin._filter.update(0.0, 5.0, 5.0, _R)
    before = plugin._filter.last_estimate()
    plugin._on_local_position(_local_position(0.0, 0.0, -20.0, 6.0, 0.0, t_ms=10_000))
    # One second later, after a 90-degree turn at 6 m/s: about 1.2 m off the
    # straight average of the two velocities.
    plugin._on_local_position(_local_position(3.85, 3.85, -20.0, 0.0, 6.0, t_ms=11_000))
    assert plugin._filter.last_estimate() == before
