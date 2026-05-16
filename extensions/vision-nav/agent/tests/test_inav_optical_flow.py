"""Tests for the iNav firmware branch.

iNav 7.0+ consumes the MAVLink `OPTICAL_FLOW_RAD` (#106) message when
the FC is configured with `opflow_hardware = MAVLINK` and a MAVLink rx
UART. The plugin emits the same message it already emits for ArduPilot
and PX4. These tests cover the config validation gates and the runtime
guards that prevent VIO modes from running on iNav.
"""

from __future__ import annotations

import pytest

from altnautica_vision_nav.config import (
    CameraConfig,
    FirmwareConfig,
    RangefinderConfig,
    VisionNavConfig,
    load_config,
)


# ---------------------------------------------------------------------------
# Schema-level acceptance
# ---------------------------------------------------------------------------


def test_inav_optical_flow_config_accepted() -> None:
    """A downward camera + iNav + optical_flow loads cleanly."""

    cfg = load_config(
        {
            "mode": "optical_flow",
            "camera": {
                "device_path": "/dev/video0",
                "bus_type": "uvc",
                "orientation": "downward",
            },
            "rangefinder": {
                "topology": "companion",
                "driver": "tfluna_uart",
                "device": "/dev/ttyUSB0",
                "baud": 115200,
            },
            "firmware": {"type": "inav"},
        }
    )
    assert cfg.firmware.type == "inav"
    assert cfg.mode == "optical_flow"
    assert cfg.camera.orientation == "downward"


def test_inav_optical_flow_degraded_config_accepted() -> None:
    """iNav + degraded OF (no rangefinder) loads — the scale ladder
    handles altitude inference."""

    cfg = load_config(
        {
            "mode": "optical_flow_degraded",
            "camera": {"orientation": "downward"},
            "rangefinder": {"topology": "none", "driver": "fc_relay"},
            "firmware": {"type": "inav"},
        }
    )
    assert cfg.firmware.type == "inav"
    assert cfg.mode == "optical_flow_degraded"


# ---------------------------------------------------------------------------
# Schema-level rejection: VIO on iNav
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "vio_mode",
    ["vio_openvins", "vio_vins_fusion", "hybrid_of_plus_vio"],
)
def test_inav_vio_modes_rejected(vio_mode: str) -> None:
    """The validator refuses VIO modes on iNav. iNav's external
    position-injection EKF integration is not VIO-grade in 7.x; the
    operator gets a clear message instead of silent drift."""

    with pytest.raises(ValueError, match="VIO modes are not supported on iNav"):
        load_config(
            {
                "mode": vio_mode,
                "camera": {"orientation": "forward"},
                "firmware": {"type": "inav"},
            }
        )


# ---------------------------------------------------------------------------
# Plugin-level guard: VIO estimator never instantiates on iNav
# ---------------------------------------------------------------------------


class _StubCtx:
    """Minimal plugin ctx for unit tests."""

    def __init__(self) -> None:
        self.log = None
        self.mavlink = None
        self.event = None
        self.events = None
        self.process = None
        self.install_dir = None
        self.plugin_install_dir = None
        self.data_dir = None


def test_build_estimator_returns_null_for_vio_on_inav() -> None:
    """If a programmatic set-mode or a hand-edited config slips past
    the validator and asks for VIO on iNav, the estimator factory must
    return NullEstimator. Belt-and-suspenders for the validator."""

    from altnautica_vision_nav.estimators import NullEstimator
    from altnautica_vision_nav.plugin import VisionNavPlugin

    plugin = VisionNavPlugin()
    plugin._config = VisionNavConfig(
        mode="optical_flow",  # Pass validator first
        camera=CameraConfig(orientation="downward"),
        rangefinder=RangefinderConfig(topology="none", driver="fc_relay"),
        firmware=FirmwareConfig(type="inav"),
    )
    ctx = _StubCtx()

    # Now ask the factory directly for a VIO mode (bypasses validator).
    result = plugin._build_estimator(ctx, "vio_vins_fusion", scale_source=None)
    assert isinstance(result, NullEstimator)


def test_build_estimator_returns_null_for_hybrid_on_inav() -> None:
    """Hybrid mode also blocked on iNav because the VIO half cannot
    run."""

    from altnautica_vision_nav.estimators import NullEstimator
    from altnautica_vision_nav.plugin import VisionNavPlugin

    plugin = VisionNavPlugin()
    plugin._config = VisionNavConfig(
        mode="optical_flow",
        camera=CameraConfig(orientation="downward"),
        rangefinder=RangefinderConfig(topology="none", driver="fc_relay"),
        firmware=FirmwareConfig(type="inav"),
    )
    ctx = _StubCtx()
    result = plugin._build_estimator(ctx, "hybrid_of_plus_vio", scale_source=None)
    assert isinstance(result, NullEstimator)


# ---------------------------------------------------------------------------
# Firmware literal coverage
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("fw_type", ["ardupilot", "px4", "inav"])
def test_supported_firmware_types(fw_type: str) -> None:
    cfg = load_config(
        {
            "mode": "optical_flow",
            "camera": {"orientation": "downward"},
            "rangefinder": {"topology": "none", "driver": "fc_relay"},
            "firmware": {"type": fw_type},
        }
    )
    assert cfg.firmware.type == fw_type


def test_betaflight_not_a_supported_firmware_type() -> None:
    """The schema is closed: any string outside the literal is rejected.
    Betaflight is intentionally absent because the firmware has no
    position estimator."""

    with pytest.raises(Exception):
        load_config(
            {
                "mode": "optical_flow",
                "camera": {"orientation": "downward"},
                "firmware": {"type": "betaflight"},
            }
        )
