"""Tests for the IMU subscribers and the frame-IMU time aligner.

The IMU package is the substrate every estimator reads through. These
tests pin the SI translation, the ring-buffer semantics, the rate
estimate, and the drift-band logic so a future estimator that swaps
in a different source observes a stable contract.
"""

from __future__ import annotations

import time
from typing import Any, Callable, List

import pytest

from altnautica_vision_nav.imu import (
    DRIFT_BAND_GREEN_MS,
    DRIFT_BAND_YELLOW_MS,
    ImuSample,
    MavlinkRawImu,
    MavlinkScaledImu2,
    TimeAligner,
)
from altnautica_vision_nav.imu.base import BaseImuSource


class _FakeMavlink:
    """Minimal stand-in for ``ctx.mavlink``.

    Tests push payloads through ``deliver`` and verify the source
    records them. The recorded handlers are kept in ``handlers`` so the
    test can assert which messages each source subscribed to.
    """

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
    """Wraps a fake mavlink so ``ctx.mavlink.subscribe`` works."""

    def __init__(self) -> None:
        self.mavlink = _FakeMavlink()


def test_raw_imu_translates_to_si_units() -> None:
    """RAW_IMU mrad/s + mg become rad/s + m/s²."""

    ctx = _FakeCtx()
    source = MavlinkRawImu(ctx)
    source.start()

    # 100 mrad/s = 0.1 rad/s. 1000 mg = 9.80665 m/s² (one g).
    ctx.mavlink.deliver(
        "RAW_IMU",
        {
            "xgyro": 100,
            "ygyro": -50,
            "zgyro": 0,
            "xacc": 0,
            "yacc": 0,
            "zacc": 1000,
        },
    )

    latest = source.latest()
    assert latest is not None
    assert latest.xgyro == pytest.approx(0.1)
    assert latest.ygyro == pytest.approx(-0.05)
    assert latest.zgyro == 0.0
    assert latest.zacc == pytest.approx(9.80665)


def test_scaled_imu2_subscribes_to_distinct_message() -> None:
    """SCALED_IMU2 source must not steal RAW_IMU traffic and vice-versa."""

    ctx = _FakeCtx()
    raw = MavlinkRawImu(ctx)
    raw.start()
    scaled = MavlinkScaledImu2(ctx)
    scaled.start()

    assert "RAW_IMU" in ctx.mavlink.handlers
    assert "SCALED_IMU2" in ctx.mavlink.handlers
    assert raw.source_id == "mavlink-raw-imu"
    assert scaled.source_id == "mavlink-scaled-imu2"


def test_source_buffers_recent_samples_in_order() -> None:
    """``recent()`` returns oldest-first; capacity caps the buffer."""

    ctx = _FakeCtx()
    source = MavlinkRawImu(ctx, buffer_capacity=3)
    source.start()

    for i in range(5):
        ctx.mavlink.deliver(
            "RAW_IMU",
            {
                "xgyro": i,
                "ygyro": 0,
                "zgyro": 0,
                "xacc": 0,
                "yacc": 0,
                "zacc": 0,
            },
        )

    samples = list(source.recent())
    # Capacity 3 keeps the last three samples (xgyro 2,3,4 in millirad).
    assert [round(s.xgyro * 1000) for s in samples] == [2, 3, 4]


def test_source_rate_hz_converges() -> None:
    """EMA rate converges on the cadence after a few samples."""

    ctx = _FakeCtx()
    source = MavlinkRawImu(ctx)
    source.start()

    # Inject a synthetic 200 Hz cadence. We can't drive time.monotonic_ns
    # directly so we settle for "rate is positive and roughly bounded".
    for _ in range(40):
        ctx.mavlink.deliver(
            "RAW_IMU",
            {
                "xgyro": 1,
                "ygyro": 0,
                "zgyro": 0,
                "xacc": 0,
                "yacc": 0,
                "zacc": 0,
            },
        )
        time.sleep(0.005)  # 5 ms => 200 Hz target

    rate = source.rate_hz()
    assert rate is not None
    assert 100.0 < rate < 600.0, f"rate {rate} Hz unrealistic for ~200 Hz cadence"


def test_time_aligner_returns_none_when_buffer_empty() -> None:
    """No IMU samples yet => alignment lookup returns ``None``."""

    ctx = _FakeCtx()
    source = MavlinkRawImu(ctx)
    aligner = TimeAligner(source)
    assert aligner.lookup(123_456) is None
    assert aligner.mean_residual_ms() is None
    assert aligner.drift_band() == "green"  # treated as "not yet wired"


def test_time_aligner_interpolates_between_bracket() -> None:
    """Linear interpolation between two bracketing samples is correct."""

    class _StaticSource(BaseImuSource):
        source_id = "static"

        def __init__(self, samples: List[ImuSample]) -> None:
            super().__init__()
            for s in samples:
                self._record(s)

        def start(self) -> None:  # pragma: no cover - never called
            pass

        def stop(self) -> None:  # pragma: no cover - never called
            pass

    before = ImuSample(
        ts_ns=1_000_000_000,
        xgyro=0.0,
        ygyro=0.0,
        zgyro=0.0,
        xacc=1.0,
        yacc=0.0,
        zacc=0.0,
    )
    after = ImuSample(
        ts_ns=1_010_000_000,
        xgyro=0.0,
        ygyro=0.0,
        zgyro=0.0,
        xacc=3.0,
        yacc=0.0,
        zacc=0.0,
    )
    source = _StaticSource([before, after])
    aligner = TimeAligner(source)

    # Frame timestamp halfway between -> interpolated xacc 2.0.
    aligned = aligner.lookup(1_005_000_000)
    assert aligned is not None
    assert aligned.imu_sample.xacc == pytest.approx(2.0)
    # Residual is the distance to the nearer real sample (5 ms either way).
    assert aligned.residual_ms == pytest.approx(5.0)


def test_time_aligner_applies_timeshift() -> None:
    """A non-zero ``timeshift_cam_imu`` shifts the lookup target.

    Kalibr convention: ``t_imu = t_cam + timeshift_cam_imu``. A
    positive offset means the IMU clock is ahead of the camera clock;
    a frame stamped at ``t_cam`` should pair with an IMU sample at
    ``t_cam + offset``.
    """

    class _StaticSource(BaseImuSource):
        source_id = "static"

        def __init__(self, samples: List[ImuSample]) -> None:
            super().__init__()
            for s in samples:
                self._record(s)

        def start(self) -> None:  # pragma: no cover - never called
            pass

        def stop(self) -> None:  # pragma: no cover - never called
            pass

    # IMU sample at exactly 1.020 s.
    s = ImuSample(
        ts_ns=1_020_000_000,
        xgyro=0.42,
        ygyro=0.0,
        zgyro=0.0,
        xacc=0.0,
        yacc=0.0,
        zacc=0.0,
    )
    source = _StaticSource([s])
    aligner = TimeAligner(source, timeshift_cam_imu_s=0.020)

    # Frame stamped at 1.000 s + 20 ms timeshift => target 1.020 s.
    aligned = aligner.lookup(1_000_000_000)
    assert aligned is not None
    assert aligned.imu_sample.xgyro == pytest.approx(0.42)
    assert aligned.residual_ms == pytest.approx(0.0, abs=1e-3)


def test_drift_band_thresholds_match_revised_spec() -> None:
    """Green ≤ 10 ms, yellow ≤ 30 ms, red > 30 ms."""

    class _StaticSource(BaseImuSource):
        source_id = "static"

        def __init__(self, samples: List[ImuSample]) -> None:
            super().__init__()
            for s in samples:
                self._record(s)

        def start(self) -> None:  # pragma: no cover - never called
            pass

        def stop(self) -> None:  # pragma: no cover - never called
            pass

    s_at_zero = ImuSample(
        ts_ns=0,
        xgyro=0.0,
        ygyro=0.0,
        zgyro=0.0,
        xacc=0.0,
        yacc=0.0,
        zacc=0.0,
    )
    source = _StaticSource([s_at_zero])
    aligner = TimeAligner(source, window_size=3)

    # Three lookups at increasing residuals: 5 ms, 25 ms, 50 ms.
    aligner.lookup(5_000_000)
    aligner.lookup(25_000_000)
    aligner.lookup(50_000_000)
    mean = aligner.mean_residual_ms()
    assert mean is not None
    assert mean == pytest.approx((5.0 + 25.0 + 50.0) / 3.0)
    # Mean ~26.6 ms falls in the yellow band.
    assert aligner.drift_band() == "yellow"

    # One huge residual flips it red.
    aligner.lookup(100_000_000)
    assert aligner.drift_band() == "red"


def test_thresholds_match_constants() -> None:
    """The exposed constants match the documented bands."""

    assert DRIFT_BAND_GREEN_MS == 10.0
    assert DRIFT_BAND_YELLOW_MS == 30.0


def test_set_timeshift_clears_history() -> None:
    """Updating the static offset clears the residual window.

    Old residuals were measured against the previous offset; keeping
    them would make the band reflect a calibration that no longer
    applies.
    """

    class _StaticSource(BaseImuSource):
        source_id = "static"

        def __init__(self, samples: List[ImuSample]) -> None:
            super().__init__()
            for s in samples:
                self._record(s)

        def start(self) -> None:  # pragma: no cover - never called
            pass

        def stop(self) -> None:  # pragma: no cover - never called
            pass

    source = _StaticSource(
        [
            ImuSample(
                ts_ns=0,
                xgyro=0.0,
                ygyro=0.0,
                zgyro=0.0,
                xacc=0.0,
                yacc=0.0,
                zacc=0.0,
            )
        ]
    )
    aligner = TimeAligner(source, window_size=4)
    aligner.lookup(40_000_000)  # 40 ms residual
    assert aligner.mean_residual_ms() == pytest.approx(40.0)
    aligner.set_timeshift(0.005)
    assert aligner.mean_residual_ms() is None
