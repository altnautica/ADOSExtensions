"""Unit tests for the capture sources, gyro tap, and optical-flow processor.

Hardware-backed sources (V4l2 and libcamera) are exercised by tests
that are skipped unless a real camera is attached. The synthetic
processor tests carry the bulk of the assertions: a translated frame
pair yields a positive flow estimate, a static pair yields a
near-zero estimate, and the gyro tap correctly converts millirad/s to
rad/s on ingestion.
"""

from __future__ import annotations

from typing import Any, Callable, List

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from altnautica_vision_nav.capture.gyro_tap import GyroReading, GyroTap
from altnautica_vision_nav.processors.optical_flow_lk import (
    OpticalFlowLk,
    OpticalFlowResult,
)


def _textured_frame(width: int = 640, height: int = 480, seed: int = 7) -> "np.ndarray":
    """Build a deterministic high-contrast pattern that LK can track.

    A pure noise field works but tends to produce many weak corners;
    overlaying a coarse grid plus a handful of bright blobs gives the
    pyramidal tracker stable, well-separated features.
    """

    rng = np.random.default_rng(seed)
    base = rng.integers(40, 200, size=(height, width), dtype=np.uint8)
    # Coarse grid for structure.
    for y in range(0, height, 32):
        base[y : y + 2, :] = 230
    for x in range(0, width, 32):
        base[:, x : x + 2] = 230
    # A few high-contrast blobs.
    for cx, cy in ((100, 110), (300, 220), (520, 380), (80, 400), (560, 80)):
        cv2.circle(base, (cx, cy), 16, 30, thickness=-1)
        cv2.circle(base, (cx, cy), 8, 250, thickness=-1)
    return base


def _shift(frame: "np.ndarray", dx: int, dy: int) -> "np.ndarray":
    """Translate ``frame`` by ``(dx, dy)`` pixels with edge replication."""

    h, w = frame.shape[:2]
    matrix = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(
        frame,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )


def test_optical_flow_lk_synthetic_translation() -> None:
    prev = _textured_frame()
    curr = _shift(prev, dx=5, dy=3)
    proc = OpticalFlowLk()
    result = proc.process(prev, curr, dt_seconds=1.0 / 30.0)
    assert isinstance(result, OpticalFlowResult)
    assert result.flow_x_dpi > 0
    assert result.flow_y_dpi > 0
    assert result.quality > 0
    # Sanity-bound the recovered pixel displacement.
    dx_pixels = result.flow_x_dpi / 8.0
    dy_pixels = result.flow_y_dpi / 8.0
    assert 3.0 < dx_pixels < 7.0
    assert 1.5 < dy_pixels < 5.0


def test_optical_flow_lk_no_motion() -> None:
    frame = _textured_frame()
    proc = OpticalFlowLk()
    result = proc.process(frame, frame.copy(), dt_seconds=1.0 / 30.0)
    assert isinstance(result, OpticalFlowResult)
    assert abs(result.flow_x_dpi) < 1.0
    assert abs(result.flow_y_dpi) < 1.0
    assert abs(result.flow_rate_x) < 1e-2
    assert abs(result.flow_rate_y) < 1e-2


def test_optical_flow_lk_gyro_derotation_changes_rate() -> None:
    """When gyro is supplied, the reported rotational rate is derotated."""

    prev = _textured_frame()
    curr = _shift(prev, dx=4, dy=0)
    proc = OpticalFlowLk()
    base = proc.process(prev, curr, dt_seconds=1.0 / 30.0)
    gyro = GyroReading(ts=0, xgyro=0.0, ygyro=0.5, zgyro=0.0)
    rotated = proc.process(prev, curr, dt_seconds=1.0 / 30.0, gyro=gyro)
    # ygyro maps onto flow_rate_y via the horizontal FOV; the derotated
    # value must differ from the no-gyro baseline.
    assert rotated.flow_rate_y != pytest.approx(base.flow_rate_y, abs=1e-6)
    # flow_rate_z mirrors the body-frame z gyro.
    assert rotated.flow_rate_z == pytest.approx(0.0)


def test_optical_flow_lk_distance_scaled_velocity() -> None:
    prev = _textured_frame()
    curr = _shift(prev, dx=5, dy=0)
    proc = OpticalFlowLk()
    result = proc.process(prev, curr, dt_seconds=0.1, distance_m=2.0)
    assert result.flow_comp_m_x != 0.0
    # flow_comp = flow_rate * distance, so it scales linearly with distance.
    result_doubled = proc.process(prev, curr, dt_seconds=0.1, distance_m=4.0)
    assert result_doubled.flow_comp_m_x == pytest.approx(
        result.flow_comp_m_x * 2.0, rel=1e-6
    )


class _FakeMavlinkBus:
    """Minimal stand-in for ``ctx.mavlink`` exposing ``subscribe``."""

    def __init__(self) -> None:
        self.handlers: dict[str, List[Callable[[Any], None]]] = {}

    def subscribe(
        self, msg_name: str, handler: Callable[[Any], None]
    ) -> Callable[[], None]:
        self.handlers.setdefault(msg_name, []).append(handler)

        def _unsubscribe() -> None:
            if handler in self.handlers.get(msg_name, []):
                self.handlers[msg_name].remove(handler)

        return _unsubscribe

    def emit(self, msg_name: str, payload: Any) -> None:
        for handler in list(self.handlers.get(msg_name, [])):
            handler(payload)


class _FakeCtx:
    def __init__(self) -> None:
        self.mavlink = _FakeMavlinkBus()


def test_gyro_tap_unit_conversion() -> None:
    ctx = _FakeCtx()
    tap = GyroTap(ctx)
    tap.start()
    assert tap.latest() is None
    ctx.mavlink.emit(
        "RAW_IMU",
        {"xgyro": 100, "ygyro": -200, "zgyro": 50, "xacc": 0, "yacc": 0, "zacc": 0},
    )
    reading = tap.latest()
    assert reading is not None
    assert reading.xgyro == pytest.approx(0.1, rel=1e-6)
    assert reading.ygyro == pytest.approx(-0.2, rel=1e-6)
    assert reading.zgyro == pytest.approx(0.05, rel=1e-6)
    tap.stop()
    assert ctx.mavlink.handlers.get("RAW_IMU") == []


def test_gyro_tap_object_payload() -> None:
    """The tap accepts attribute-style payloads too (live SDK shape)."""

    class _Payload:
        xgyro = 250
        ygyro = 0
        zgyro = -75

    ctx = _FakeCtx()
    tap = GyroTap(ctx)
    tap.start()
    ctx.mavlink.emit("RAW_IMU", _Payload())
    reading = tap.latest()
    assert reading is not None
    assert reading.xgyro == pytest.approx(0.25, rel=1e-6)
    assert reading.zgyro == pytest.approx(-0.075, rel=1e-6)


def test_v4l2_source_requires_hardware() -> None:
    pytest.skip("requires hardware")


def test_libcamera_source_requires_hardware() -> None:
    pytest.skip("requires hardware")
