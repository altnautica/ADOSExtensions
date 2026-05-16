"""Tests for the rangefinder-free MAVLink altitude ladder.

The ladder is the substrate that lets vision-nav operate without a
rangefinder. These tests pin the rung selection logic, the staleness
gates, the GPS outdoor-flag gate, and the static fallback.
"""

from __future__ import annotations

import time
from typing import Any, Callable

import pytest

from altnautica_vision_nav.scale import (
    MavlinkScaleLadder,
    NullScaleSource,
    ScalePick,
)
from altnautica_vision_nav.scale.mavlink_ladder import (
    MAX_DISTANCE_M,
    MIN_DISTANCE_M,
    QM_GPS,
    QM_RAW_BARO,
    QM_RELATIVE_ALT,
    QM_STATIC,
)


class _FakeMavlink:
    def __init__(self) -> None:
        self.handlers: dict[str, Callable[[Any], None]] = {}

    def subscribe(
        self, msg: str, handler: Callable[[Any], None]
    ) -> Callable[[], None]:
        self.handlers[msg] = handler

        def _unsub() -> None:
            self.handlers.pop(msg, None)

        return _unsub

    def deliver(self, msg: str, payload: Any) -> None:
        handler = self.handlers.get(msg)
        if handler is None:
            raise AssertionError(f"no subscriber for {msg!r}")
        handler(payload)


class _FakeCtx:
    def __init__(self) -> None:
        self.mavlink = _FakeMavlink()


def test_null_scale_source_always_returns_none() -> None:
    """``NullScaleSource`` is the explicit "no scale" placeholder."""

    src = NullScaleSource()
    src.start()
    assert src.pick() is None
    src.stop()


def test_relative_alt_is_primary_rung() -> None:
    """``GLOBAL_POSITION_INT.relative_alt`` wins when fresh."""

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx)
    ladder.start()

    # relative_alt is in mm on the wire. 1500 mm -> 1.5 m.
    ctx.mavlink.deliver("GLOBAL_POSITION_INT", {"relative_alt": 1500})
    pick = ladder.pick()
    assert isinstance(pick, ScalePick)
    assert pick.source == "baro"
    assert pick.distance_m == pytest.approx(1.5)
    assert pick.quality_multiplier == QM_RELATIVE_ALT


def test_vfr_hud_is_secondary_rung() -> None:
    """``VFR_HUD.alt`` is used when ``relative_alt`` is absent.

    The take-off altitude is captured on the first VFR_HUD sample so
    subsequent samples report AGL rather than AMSL.
    """

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx)
    ladder.start()

    # First VFR sample captures take-off altitude; pick reports 0 m
    # (clamped up to MIN_DISTANCE_M because zero altitude is nonsense
    # for an OF scale).
    ctx.mavlink.deliver("VFR_HUD", {"alt": 100.0})
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "baro"
    assert pick.distance_m == pytest.approx(MIN_DISTANCE_M)
    assert pick.quality_multiplier == QM_RAW_BARO

    # Second VFR sample at +1.2 m above take-off.
    ctx.mavlink.deliver("VFR_HUD", {"alt": 101.2})
    pick = ladder.pick()
    assert pick is not None
    assert pick.distance_m == pytest.approx(1.2)


def test_gps_rung_requires_outdoor_flag_and_3d_fix() -> None:
    """GPS rung is gated on operator outdoor flag + 3D fix + HDOP."""

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx, outdoor_flag=False)
    ladder.start()

    # Indoor: GPS message arrives but the rung is gated off; we should
    # land on the static fallback.
    ctx.mavlink.deliver(
        "GPS_RAW_INT",
        {"alt": 5000, "fix_type": 3, "eph": 150},  # alt in mm
    )
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "static"
    assert pick.quality_multiplier == QM_STATIC

    # Flip outdoor flag on; now the GPS rung is consulted.
    ladder.set_outdoor_flag(True)
    ctx.mavlink.deliver(
        "GPS_RAW_INT",
        {"alt": 5000, "fix_type": 3, "eph": 150},
    )
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "gps"
    assert pick.quality_multiplier == QM_GPS


def test_gps_rejected_when_fix_quality_poor() -> None:
    """GPS with no 3D fix or high HDOP must not feed the EKF."""

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx, outdoor_flag=True)
    ladder.start()

    # 2D fix only.
    ctx.mavlink.deliver(
        "GPS_RAW_INT",
        {"alt": 5000, "fix_type": 2, "eph": 100},
    )
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "static"

    # 3D fix but HDOP too high.
    ctx.mavlink.deliver(
        "GPS_RAW_INT",
        {"alt": 5000, "fix_type": 3, "eph": 500},
    )
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "static"


def test_static_fallback_with_no_messages() -> None:
    """No baro / GPS yet -> static fallback at 1.5 m, lowest quality."""

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx, static_fallback_m=1.5)
    ladder.start()
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "static"
    assert pick.distance_m == pytest.approx(1.5)
    assert pick.quality_multiplier == QM_STATIC


def test_distance_clamped_to_valid_range() -> None:
    """Pathological values are clamped, never propagated raw."""

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx)
    ladder.start()

    # 100 km altitude (sensor glitch); clamp to MAX.
    ctx.mavlink.deliver("GLOBAL_POSITION_INT", {"relative_alt": 100_000_000})
    pick = ladder.pick()
    assert pick is not None
    assert pick.distance_m == pytest.approx(MAX_DISTANCE_M)

    # Negative altitude (vehicle below take-off in a basin); clamp to MIN.
    ctx.mavlink.deliver("GLOBAL_POSITION_INT", {"relative_alt": -5000})
    pick = ladder.pick()
    assert pick is not None
    assert pick.distance_m == pytest.approx(MIN_DISTANCE_M)


def test_stale_relative_alt_drops_to_next_rung() -> None:
    """A relative_alt sample older than 2 s should not be used.

    The implementation reads ``time.monotonic_ns()`` directly so this
    test sleeps just past the staleness threshold to exercise the gate.
    The 2 s sleep is unfortunate for test wall-time but the gate is
    structurally important for safety; skipping it would let a
    hung-FC sample silently feed the EKF.
    """

    ctx = _FakeCtx()
    ladder = MavlinkScaleLadder(ctx)
    ladder.start()

    ctx.mavlink.deliver("GLOBAL_POSITION_INT", {"relative_alt": 1500})
    ctx.mavlink.deliver("VFR_HUD", {"alt": 100.0})  # capture take-off
    ctx.mavlink.deliver("VFR_HUD", {"alt": 101.0})  # 1 m AGL

    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "baro"
    assert pick.quality_multiplier == QM_RELATIVE_ALT  # rung 1 alive

    # Sleep past the stale threshold; both samples now stale.
    time.sleep(2.05)
    pick = ladder.pick()
    assert pick is not None
    assert pick.source == "static"
    assert pick.quality_multiplier == QM_STATIC


def test_quality_multipliers_match_research() -> None:
    """Per-rung quality multipliers match the rangefinder-free spec."""

    assert QM_RELATIVE_ALT == 0.7
    assert QM_RAW_BARO == 0.6
    assert QM_GPS == 0.4
    assert QM_STATIC == 0.2
