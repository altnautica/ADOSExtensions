"""Tests for the hardware auto-detect package."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Sequence

import pytest

from altnautica_vision_nav.autodetect.camera import (
    DetectedCamera,
    detect_cameras,
    pick_camera_for_mode,
)
from altnautica_vision_nav.autodetect.imu import detect_preferred_imu_source
from altnautica_vision_nav.autodetect.mode import derive_suggested_mode
from altnautica_vision_nav.autodetect.profile import (
    detect_host_profile,
    is_drone_profile,
)
from altnautica_vision_nav.autodetect.rangefinder import (
    DetectedRangefinder,
    detect_rangefinder,
)


# ---------------------------------------------------------------------------
# profile
# ---------------------------------------------------------------------------


def test_profile_from_file(tmp_path: Path) -> None:
    conf = tmp_path / "profile.conf"
    conf.write_text("profile = drone\n", encoding="utf-8")
    result = detect_host_profile(conf_path=conf, env_var="UNSET")
    assert result.profile == "drone"
    assert result.source == "file"


def test_profile_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("VN_TEST_PROFILE", "ground-station-relay")
    missing = tmp_path / "missing.conf"
    result = detect_host_profile(conf_path=missing, env_var="VN_TEST_PROFILE")
    assert result.profile == "ground-station-relay"
    assert result.source == "env"


def test_profile_default_unknown(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv("VN_TEST_PROFILE", raising=False)
    missing = tmp_path / "missing.conf"
    result = detect_host_profile(conf_path=missing, env_var="VN_TEST_PROFILE")
    assert result.profile == "unknown"
    assert result.source == "default"


def test_is_drone_profile_accepts_drone_and_unknown(
    tmp_path: Path,
) -> None:
    drone = detect_host_profile(
        conf_path=_write(tmp_path, "drone"), env_var="UNSET"
    )
    assert is_drone_profile(drone) is True
    unknown = detect_host_profile(
        conf_path=tmp_path / "missing.conf", env_var="UNSET"
    )
    assert is_drone_profile(unknown) is True


def test_is_drone_profile_rejects_ground_station_variants(
    tmp_path: Path,
) -> None:
    for variant in (
        "ground-station",
        "ground-station-direct",
        "ground-station-relay",
        "ground-station-receiver",
    ):
        profile = detect_host_profile(
            conf_path=_write(tmp_path, variant), env_var="UNSET"
        )
        assert is_drone_profile(profile) is False, variant


def _write(tmp_path: Path, profile: str) -> Path:
    p = tmp_path / f"profile_{profile}.conf"
    p.write_text(f"profile = {profile}\n", encoding="utf-8")
    return p


# ---------------------------------------------------------------------------
# camera
# ---------------------------------------------------------------------------


def test_detect_cameras_empty(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("glob.glob", lambda _pattern: [])
    assert detect_cameras() == []


def test_detect_cameras_marks_csi(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    csi_node = tmp_path / "media0"
    csi_node.write_text("", encoding="utf-8")
    monkeypatch.setattr("glob.glob", lambda _pattern: ["/dev/video0", "/dev/video1"])
    cameras = detect_cameras(csi_paths=(str(csi_node),))
    kinds = [c.kind for c in cameras]
    assert kinds == ["csi", "uvc"]


def test_pick_camera_off_returns_none() -> None:
    cams = [DetectedCamera(device_path="/dev/video0", kind="uvc")]
    assert pick_camera_for_mode(cams, "off") is None


def test_pick_camera_optical_flow_takes_first_uvc() -> None:
    cams = [
        DetectedCamera(device_path="/dev/video0", kind="uvc"),
        DetectedCamera(device_path="/dev/video1", kind="csi"),
    ]
    pick = pick_camera_for_mode(cams, "optical_flow")
    assert pick is not None and pick.device_path == "/dev/video0"


def test_pick_camera_vio_prefers_csi() -> None:
    cams = [
        DetectedCamera(device_path="/dev/video0", kind="uvc"),
        DetectedCamera(device_path="/dev/video1", kind="csi"),
    ]
    pick = pick_camera_for_mode(cams, "vio_openvins")
    assert pick is not None and pick.kind == "csi"


def test_pick_camera_empty_returns_none() -> None:
    assert pick_camera_for_mode([], "optical_flow") is None


# ---------------------------------------------------------------------------
# rangefinder
# ---------------------------------------------------------------------------


def test_rangefinder_returns_none_when_no_bus(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("glob.glob", lambda _pattern: [])
    assert (
        detect_rangefinder(
            i2c_probe=lambda _bus, _addr: False,
            uart_probe=lambda _device: False,
        )
        is None
    )


def test_rangefinder_picks_lidar_lite(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "glob.glob",
        lambda pattern: ["/dev/i2c-1"] if "i2c" in pattern else [],
    )
    probed = []

    def i2c(bus: int, addr: int) -> bool:
        probed.append((bus, addr))
        return addr == 0x62

    result = detect_rangefinder(
        i2c_probe=i2c,
        uart_probe=lambda _device: False,
    )
    assert result is not None
    assert result.driver == "garmin_lidarlite_i2c"
    assert result.topology == "companion"
    assert (1, 0x62) in probed


def test_rangefinder_picks_vl53l1x_when_lidar_absent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "glob.glob",
        lambda pattern: ["/dev/i2c-1"] if "i2c" in pattern else [],
    )
    result = detect_rangefinder(
        i2c_probe=lambda _bus, addr: addr == 0x29,
        uart_probe=lambda _device: False,
    )
    assert result is not None
    assert result.driver == "vl53l1x_i2c"


def test_rangefinder_picks_tfluna_uart_fallback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # No I2C buses but a UART node responds.
    def fake_glob(pattern: str) -> list[str]:
        if "i2c" in pattern:
            return []
        if "ttyUSB" in pattern:
            return ["/dev/ttyUSB0"]
        return []

    monkeypatch.setattr("glob.glob", fake_glob)
    result = detect_rangefinder(
        i2c_probe=lambda _bus, _addr: False,
        uart_probe=lambda device: device == "/dev/ttyUSB0",
    )
    assert result is not None
    assert result.driver == "tfluna_uart"
    assert result.bus == "/dev/ttyUSB0"


# ---------------------------------------------------------------------------
# imu
# ---------------------------------------------------------------------------


def test_imu_default_is_mavlink_raw_imu() -> None:
    result = detect_preferred_imu_source()
    assert result.source_id == "mavlink-raw-imu"


def test_imu_prefers_direct_i2c_when_present() -> None:
    result = detect_preferred_imu_source(has_direct_i2c=True)
    assert result.source_id == "direct-i2c-bmi088"


def test_imu_prefers_dronecan_over_mavlink() -> None:
    result = detect_preferred_imu_source(
        has_direct_i2c=False, has_dronecan=True
    )
    assert result.source_id == "direct-dronecan"


# ---------------------------------------------------------------------------
# suggested mode
# ---------------------------------------------------------------------------


def test_suggested_mode_off_with_no_hardware() -> None:
    assert (
        derive_suggested_mode(
            has_camera=False, has_rangefinder=False
        ).mode
        == "off"
    )


def test_suggested_mode_optical_flow_with_rangefinder() -> None:
    assert (
        derive_suggested_mode(
            has_camera=True, has_rangefinder=True
        ).mode
        == "optical_flow"
    )


def test_suggested_mode_degraded_without_rangefinder() -> None:
    assert (
        derive_suggested_mode(
            has_camera=True, has_rangefinder=False
        ).mode
        == "optical_flow_degraded"
    )


def test_suggested_mode_vio_when_forward_cam_and_npu() -> None:
    assert (
        derive_suggested_mode(
            has_camera=False,
            has_rangefinder=False,
            has_forward_camera=True,
            has_npu_board=True,
        ).mode
        == "vio_openvins"
    )
