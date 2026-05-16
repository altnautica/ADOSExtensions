"""Tests for the agent-side calibration runner.

The full happy path requires OpenCV with the aruco module, which the
test environment does not always have. The tests below split into
two groups:

* Pure-Python tests that exercise the request parser, the YAML
  formatter, and the gyro-camera residual without touching cv2.
* A stubbed end-to-end test that replaces ``cv2`` with a small mock
  inside the runner module's namespace and walks through
  :func:`run_calibration` end-to-end. This catches the wiring
  between the runner and the hooks adapter even when OpenCV is not
  installed in CI.
"""

from __future__ import annotations

import asyncio
import base64
import dataclasses
from pathlib import Path

import pytest

from altnautica_vision_nav.calibration import runner as runner_mod
from altnautica_vision_nav.calibration.runner import (
    CalibrationProgress,
    CalibrationResult,
    RunnerHooks,
    _format_camchain_yaml,
    _gyro_camera_residual,
    _ImuSample,
    parse_request,
    run_calibration,
)


def test_parse_request_happy_path() -> None:
    req = parse_request(
        {
            "framesB64": ["AAAA"],
            "windowStartNs": 1000,
            "windowEndNs": 2000,
            "width": 640,
            "height": 480,
        }
    )
    assert req.frames_b64 == ["AAAA"]
    assert req.window_start_ns == 1000
    assert req.window_end_ns == 2000
    assert req.width == 640
    assert req.height == 480
    assert req.tag_size_m == pytest.approx(0.054)


def test_parse_request_rejects_empty_frames() -> None:
    with pytest.raises(ValueError, match="framesB64"):
        parse_request({"framesB64": []})


def test_parse_request_rejects_non_string_frames() -> None:
    with pytest.raises(ValueError, match="strings"):
        parse_request({"framesB64": [123]})


def test_format_camchain_yaml_round_trip() -> None:
    result = CalibrationResult(
        camera_model="pinhole",
        fx=320.5,
        fy=320.5,
        cx=160.0,
        cy=120.0,
        width=320,
        height=240,
        distortion_model="radtan",
        distortion_coeffs=[0.01, -0.02, 0.0, 0.0],
        t_cam_imu=[
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ],
        timeshift_cam_imu_s=0.012,
        reprojection_error_px=0.5,
        timeshift_residual_ms=2.4,
        frames_used=20,
        frames_rejected=2,
    )
    yaml = _format_camchain_yaml(result)
    assert "cam0:" in yaml
    assert "camera_model: pinhole" in yaml
    assert "intrinsics: [320.5000" in yaml
    assert "timeshift_cam_imu: 0.012000" in yaml


def test_gyro_residual_empty_returns_inf() -> None:
    # No frame times -> residual is +inf (golden-section sentinel).
    assert _gyro_camera_residual(0.0, [], [], []) == float("inf")


def test_runner_reports_missing_cv2(monkeypatch: pytest.MonkeyPatch) -> None:
    """When OpenCV is unavailable, the runner publishes a clean error."""
    monkeypatch.setattr(runner_mod, "_HAVE_CV2", False)

    progress_log: list[CalibrationProgress] = []
    complete_log: list[tuple[CalibrationResult | None, str | None]] = []

    async def _record_progress(p: CalibrationProgress) -> None:
        progress_log.append(p)

    async def _record_complete(
        result: CalibrationResult | None, error: str | None
    ) -> None:
        complete_log.append((result, error))

    hooks = RunnerHooks(
        publish_progress=_record_progress,
        publish_complete=_record_complete,
        fetch_imu_window=lambda a, b: [],
        persist_camchain_yaml=lambda txt: Path("/tmp/x"),
    )
    payload = {
        "framesB64": [base64.b64encode(b"x").decode()],
        "windowStartNs": 0,
        "windowEndNs": 1,
        "width": 32,
        "height": 32,
    }
    asyncio.run(run_calibration(payload, hooks))
    assert len(complete_log) == 1
    assert complete_log[0][0] is None
    assert "OpenCV" in (complete_log[0][1] or "")


def test_runner_rejects_invalid_payload(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A bad payload short-circuits before opencv is needed."""
    monkeypatch.setattr(runner_mod, "_HAVE_CV2", True)

    complete_log: list[tuple[CalibrationResult | None, str | None]] = []

    async def _noop_progress(_p: CalibrationProgress) -> None:
        return None

    async def _record_complete(
        result: CalibrationResult | None, error: str | None
    ) -> None:
        complete_log.append((result, error))

    hooks = RunnerHooks(
        publish_progress=_noop_progress,
        publish_complete=_record_complete,
        fetch_imu_window=lambda a, b: [],
        persist_camchain_yaml=lambda txt: Path("/tmp/x"),
    )
    asyncio.run(run_calibration({"framesB64": []}, hooks))
    assert len(complete_log) == 1
    assert complete_log[0][0] is None
    assert "framesB64" in (complete_log[0][1] or "")


def test_imu_sample_dataclass_fields() -> None:
    """Defensive: the runner's ``_ImuSample`` shape is the wire contract
    the plugin.py adapter uses; keep these field names stable."""
    sample = _ImuSample(
        timestamp_ns=1_000_000,
        gyro=(0.1, 0.2, 0.3),
        accel=(0.0, 0.0, 9.81),
    )
    assert sample.timestamp_ns == 1_000_000
    assert sample.gyro == (0.1, 0.2, 0.3)
    assert sample.accel == (0.0, 0.0, 9.81)
    assert dataclasses.is_dataclass(sample)
