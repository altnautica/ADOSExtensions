"""Tests for the Kalibr-compatible calibration loaders.

Both loaders must accept the canonical Kalibr ``camchain.yaml`` block
and surface validation errors with operator-readable messages. These
tests round-trip a hand-rolled YAML through each loader and exercise
each documented validation rule.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from altnautica_vision_nav.calibration import (
    CameraImuExtrinsics,
    CameraImuExtrinsicsError,
    CameraIntrinsics,
    CameraIntrinsicsError,
    load_extrinsics,
    load_intrinsics,
)

# Identity SE(3): IMU and camera frames are co-located + co-oriented.
IDENTITY_SE3 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


_BASE_YAML = """
cam0:
  camera_model: pinhole
  intrinsics: [1385.4, 1384.1, 1014.2, 760.7]
  distortion_model: radtan
  distortion_coeffs: [-0.3142, 0.1051, 0.00018, -0.00021]
  resolution: [2028, 1520]
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.01]
    - [0.0, 0.0, 1.0, -0.02]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -0.042
  rostopic: /cam0/image_raw
"""


def _write(tmp_path: Path, content: str) -> Path:
    p = tmp_path / "camchain.yaml"
    p.write_text(content, encoding="utf-8")
    return p


def test_load_intrinsics_round_trip(tmp_path: Path) -> None:
    """A canonical Kalibr file parses into a typed CameraIntrinsics."""

    intrinsics = load_intrinsics(_write(tmp_path, _BASE_YAML))
    assert isinstance(intrinsics, CameraIntrinsics)
    assert intrinsics.camera_model == "pinhole"
    assert intrinsics.fx == pytest.approx(1385.4)
    assert intrinsics.fy == pytest.approx(1384.1)
    assert intrinsics.cx == pytest.approx(1014.2)
    assert intrinsics.cy == pytest.approx(760.7)
    assert intrinsics.width == 2028
    assert intrinsics.height == 1520
    assert intrinsics.distortion_model == "radtan"
    assert intrinsics.has_suspicious_focal_length is False


def test_load_extrinsics_round_trip(tmp_path: Path) -> None:
    """The same file yields a valid CameraImuExtrinsics."""

    extr = load_extrinsics(_write(tmp_path, _BASE_YAML))
    assert isinstance(extr, CameraImuExtrinsics)
    assert extr.timeshift_cam_imu == pytest.approx(-0.042)
    assert extr.translation == pytest.approx([0.0, 0.01, -0.02])
    assert extr.has_suspicious_timeshift is False


def test_load_intrinsics_rejects_negative_focal_length(tmp_path: Path) -> None:
    bad = _BASE_YAML.replace("1385.4", "-1385.4")
    with pytest.raises(CameraIntrinsicsError, match="fx and fy must be positive"):
        load_intrinsics(_write(tmp_path, bad))


def test_load_intrinsics_rejects_principal_point_outside_frame(tmp_path: Path) -> None:
    # cx > width
    bad = _BASE_YAML.replace("1014.2", "3000.0")
    with pytest.raises(CameraIntrinsicsError, match="outside frame width"):
        load_intrinsics(_write(tmp_path, bad))


def test_load_intrinsics_rejects_mismatched_distortion_coeff_count(
    tmp_path: Path,
) -> None:
    bad = _BASE_YAML.replace(
        "[-0.3142, 0.1051, 0.00018, -0.00021]", "[-0.3142, 0.1051]"
    )
    with pytest.raises(CameraIntrinsicsError, match="expects 4 coeffs"):
        load_intrinsics(_write(tmp_path, bad))


def test_load_intrinsics_rejects_unsupported_model(tmp_path: Path) -> None:
    # ``omni`` is valid Kalibr but explicitly out of scope for v1.
    bad = _BASE_YAML.replace("camera_model: pinhole", "camera_model: omni")
    with pytest.raises(CameraIntrinsicsError):
        load_intrinsics(_write(tmp_path, bad))


def test_load_intrinsics_rejects_radtan_coeff_out_of_range(tmp_path: Path) -> None:
    bad = _BASE_YAML.replace("-0.3142", "-5.0")
    with pytest.raises(CameraIntrinsicsError, match="out of plausible range"):
        load_intrinsics(_write(tmp_path, bad))


def test_load_intrinsics_accepts_none_distortion(tmp_path: Path) -> None:
    """``distortion_model: none`` with empty coeffs is a valid v1 config."""

    raw = _BASE_YAML.replace("distortion_model: radtan", "distortion_model: none")
    raw = raw.replace(
        "distortion_coeffs: [-0.3142, 0.1051, 0.00018, -0.00021]",
        "distortion_coeffs: []",
    )
    intrinsics = load_intrinsics(_write(tmp_path, raw))
    assert intrinsics.distortion_model == "none"
    assert intrinsics.distortion_coeffs == []


def test_load_extrinsics_rejects_non_orthonormal_rotation(tmp_path: Path) -> None:
    """A non-SE(3) transform must be rejected.

    The rotation block of an SE(3) is orthonormal with det = +1; a
    matrix that does not satisfy ``R @ R.T = I`` cannot represent a
    rigid rotation in 3-space.
    """

    bad_rot = _BASE_YAML.replace(
        "- [1.0, 0.0, 0.0, 0.0]\n    - [0.0, 1.0, 0.0, 0.01]",
        "- [1.0, 0.5, 0.0, 0.0]\n    - [0.0, 1.0, 0.0, 0.01]",
    )
    with pytest.raises(CameraImuExtrinsicsError, match="not orthonormal"):
        load_extrinsics(_write(tmp_path, bad_rot))


def test_load_extrinsics_rejects_bottom_row_drift(tmp_path: Path) -> None:
    """SE(3) bottom row must be [0,0,0,1]."""

    bad = _BASE_YAML.replace(
        "- [0.0, 0.0, 0.0, 1.0]", "- [0.0, 0.0, 0.0, 2.0]"
    )
    with pytest.raises(CameraImuExtrinsicsError, match="bottom row"):
        load_extrinsics(_write(tmp_path, bad))


def test_load_extrinsics_rejects_giant_translation(tmp_path: Path) -> None:
    """A >1 m lever arm is almost always a metres-vs-mm unit error."""

    bad = _BASE_YAML.replace("0.01]", "1000.0]")
    with pytest.raises(CameraImuExtrinsicsError, match="exceeds 1.0 m"):
        load_extrinsics(_write(tmp_path, bad))


def test_load_extrinsics_rejects_huge_timeshift(tmp_path: Path) -> None:
    """A >0.5 s offset is rejected outright; the warning band is up to 0.2 s."""

    bad = _BASE_YAML.replace("timeshift_cam_imu: -0.042", "timeshift_cam_imu: -1.0")
    with pytest.raises(CameraImuExtrinsicsError, match="sanity bound"):
        load_extrinsics(_write(tmp_path, bad))


def test_load_extrinsics_warning_band(tmp_path: Path) -> None:
    """0.25 s parses cleanly but ``has_suspicious_timeshift`` is true."""

    suspicious = _BASE_YAML.replace(
        "timeshift_cam_imu: -0.042", "timeshift_cam_imu: 0.25"
    )
    extr = load_extrinsics(_write(tmp_path, suspicious))
    assert extr.has_suspicious_timeshift is True


def test_loader_missing_file(tmp_path: Path) -> None:
    """A non-existent path raises with a clear message."""

    with pytest.raises(CameraIntrinsicsError, match="not found"):
        load_intrinsics(tmp_path / "missing.yaml")
    with pytest.raises(CameraImuExtrinsicsError, match="not found"):
        load_extrinsics(tmp_path / "missing.yaml")


def test_loader_accepts_bare_block_layout(tmp_path: Path) -> None:
    """Both layouts (``{cam0: ...}`` and bare block) parse cleanly."""

    bare = _BASE_YAML.replace("cam0:\n", "")
    # The bare layout drops two leading-space indent levels per line.
    bare = bare.replace("\n  ", "\n").strip()
    intrinsics = load_intrinsics(_write(tmp_path, bare))
    assert intrinsics.camera_model == "pinhole"
