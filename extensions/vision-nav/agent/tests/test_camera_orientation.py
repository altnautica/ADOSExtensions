"""Tests for the camera-orientation schema + hybrid dual-camera rules.

Covers:
* The new ``CameraConfig.orientation`` literal.
* Optical-flow modes rejecting explicitly-forward cameras.
* Hybrid mode requiring both cameras with opposed orientations.
* Hybrid mode rejecting same-device-path duplicates.
"""

from __future__ import annotations

import pytest

from altnautica_vision_nav.config import load_config


# ---------------------------------------------------------------------------
# Orientation field
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "orientation",
    ["forward", "downward", "side", "auto"],
)
def test_orientation_accepts_each_literal(orientation: str) -> None:
    """All four literals load under a mode that does not constrain
    orientation (VIO accepts any direction)."""
    cfg = load_config(
        {
            "mode": "vio_vins_fusion",
            "camera": {"orientation": orientation},
            "firmware": {"type": "ardupilot"},
        }
    )
    assert cfg.camera.orientation == orientation


def test_orientation_defaults_to_auto() -> None:
    cfg = load_config({"mode": "optical_flow_degraded", "camera": {}})
    assert cfg.camera.orientation == "auto"


def test_orientation_rejects_garbage() -> None:
    with pytest.raises(Exception):
        load_config(
            {
                "mode": "optical_flow",
                "camera": {"orientation": "upside_down"},
            }
        )


# ---------------------------------------------------------------------------
# Optical-flow mode rejects non-downward camera
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("of_mode", ["optical_flow", "optical_flow_degraded"])
def test_optical_flow_rejects_forward_camera(of_mode: str) -> None:
    with pytest.raises(ValueError, match="downward-facing camera"):
        load_config(
            {
                "mode": of_mode,
                "camera": {"orientation": "forward"},
            }
        )


@pytest.mark.parametrize("of_mode", ["optical_flow", "optical_flow_degraded"])
def test_optical_flow_rejects_side_camera(of_mode: str) -> None:
    with pytest.raises(ValueError, match="downward-facing camera"):
        load_config(
            {
                "mode": of_mode,
                "camera": {"orientation": "side"},
            }
        )


def test_optical_flow_accepts_auto() -> None:
    """Auto-orientation passes the validator; the wizard refuses
    ``auto`` separately when the board profile cannot resolve it."""
    cfg = load_config(
        {
            "mode": "optical_flow",
            "camera": {"orientation": "auto"},
            "rangefinder": {"topology": "none", "driver": "fc_relay"},
        }
    )
    assert cfg.camera.orientation == "auto"


# ---------------------------------------------------------------------------
# VIO modes accept any orientation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "orientation",
    ["forward", "downward", "auto"],
)
def test_vio_accepts_forward_downward_auto(orientation: str) -> None:
    cfg = load_config(
        {
            "mode": "vio_vins_fusion",
            "camera": {"orientation": orientation},
            "firmware": {"type": "ardupilot"},
        }
    )
    assert cfg.camera.orientation == orientation


# ---------------------------------------------------------------------------
# Hybrid mode: dual-camera requirements
# ---------------------------------------------------------------------------


def test_hybrid_requires_secondary_camera() -> None:
    with pytest.raises(ValueError, match="secondary_camera"):
        load_config(
            {
                "mode": "hybrid_of_plus_vio",
                "camera": {"orientation": "downward"},
                "firmware": {"type": "ardupilot"},
            }
        )


def test_hybrid_requires_opposed_orientations() -> None:
    """Both cameras pointing the same way is a misconfig; the hybrid
    estimator depends on one downward (OF) + one forward (VIO)."""
    with pytest.raises(ValueError, match="downward.*forward|forward.*downward"):
        load_config(
            {
                "mode": "hybrid_of_plus_vio",
                "camera": {
                    "device_path": "/dev/video0",
                    "orientation": "downward",
                },
                "secondary_camera": {
                    "device_path": "/dev/video1",
                    "orientation": "downward",
                },
                "firmware": {"type": "ardupilot"},
            }
        )


def test_hybrid_requires_distinct_device_paths() -> None:
    with pytest.raises(ValueError, match="distinct device_path"):
        load_config(
            {
                "mode": "hybrid_of_plus_vio",
                "camera": {
                    "device_path": "/dev/video0",
                    "orientation": "downward",
                },
                "secondary_camera": {
                    "device_path": "/dev/video0",
                    "orientation": "forward",
                },
                "firmware": {"type": "ardupilot"},
            }
        )


def test_hybrid_accepts_valid_dual_camera() -> None:
    cfg = load_config(
        {
            "mode": "hybrid_of_plus_vio",
            "camera": {
                "device_path": "/dev/video0",
                "orientation": "downward",
            },
            "secondary_camera": {
                "device_path": "/dev/video1",
                "orientation": "forward",
            },
            "firmware": {"type": "ardupilot"},
        }
    )
    assert cfg.secondary_camera is not None
    assert cfg.camera.orientation == "downward"
    assert cfg.secondary_camera.orientation == "forward"


def test_hybrid_accepts_reversed_assignment() -> None:
    """Operator passes forward in camera + downward in secondary_camera
    by accident; the validator does not care about which slot holds
    which orientation as long as both are present."""
    cfg = load_config(
        {
            "mode": "hybrid_of_plus_vio",
            "camera": {
                "device_path": "/dev/video1",
                "orientation": "forward",
            },
            "secondary_camera": {
                "device_path": "/dev/video0",
                "orientation": "downward",
            },
            "firmware": {"type": "ardupilot"},
        }
    )
    assert cfg.camera.orientation == "forward"
    assert cfg.secondary_camera is not None
    assert cfg.secondary_camera.orientation == "downward"


def test_secondary_camera_ignored_for_non_hybrid_modes() -> None:
    """An accidental secondary_camera on a non-hybrid config does not
    blow up; it gets carried through but is ignored at runtime."""
    cfg = load_config(
        {
            "mode": "optical_flow",
            "camera": {"orientation": "downward"},
            "secondary_camera": {
                "device_path": "/dev/video1",
                "orientation": "forward",
            },
            "rangefinder": {"topology": "none", "driver": "fc_relay"},
        }
    )
    assert cfg.mode == "optical_flow"
    assert cfg.secondary_camera is not None
