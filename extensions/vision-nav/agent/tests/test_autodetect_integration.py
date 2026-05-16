"""Plugin-level auto-detect wiring tests.

These tests exercise the two private helpers the plugin runs at
start-up: ``_auto_detect_hardware`` and ``_auto_flip_mode_for_hardware``.
Together they encode the contract the GCS sees: an enriched heartbeat
with the detected hardware summary plus a mode that always matches
the hardware reality.
"""

from __future__ import annotations

from typing import Any

import pytest

from altnautica_vision_nav.config import load_config
from altnautica_vision_nav.plugin import VisionNavPlugin, _AutoDetectSummary


class _StubCtx:
    """Minimal context stub for the log helpers."""

    def __init__(self) -> None:
        self.logs: list[tuple[str, str, dict[str, Any]]] = []
        self.logger = self
        self.event_publisher = self
        self.data_dir = "/tmp/vision-nav-test"

    def info(self, event: str, **fields: Any) -> None:
        self.logs.append(("info", event, fields))

    def warning(self, event: str, **fields: Any) -> None:
        self.logs.append(("warning", event, fields))


def _make_plugin_with_mode(mode: str) -> VisionNavPlugin:
    plugin = VisionNavPlugin()
    plugin._config = load_config({"mode": mode})
    return plugin


def test_auto_detect_returns_summary_shape(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The summary dataclass has the four fields the heartbeat needs."""
    plugin = _make_plugin_with_mode("optical_flow")
    monkeypatch.setattr("glob.glob", lambda _p: [])
    summary = plugin._auto_detect_hardware(_StubCtx())
    assert isinstance(summary, _AutoDetectSummary)
    assert summary.camera_count == 0
    assert summary.picked_camera_path is None
    assert summary.rangefinder_driver is None
    assert summary.suggested_mode in {
        "off",
        "optical_flow",
        "optical_flow_degraded",
        "vio_openvins",
    }
    assert isinstance(summary.suggested_reason, str)


def test_auto_flip_optical_flow_to_degraded_without_rangefinder(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The single auto-flip rule fires when no rangefinder is found."""
    plugin = _make_plugin_with_mode("optical_flow")
    summary = _AutoDetectSummary(
        camera_count=1,
        picked_camera_path="/dev/video0",
        rangefinder_driver=None,
        suggested_mode="optical_flow_degraded",
        suggested_reason="no rangefinder",
    )
    plugin._auto_flip_mode_for_hardware(_StubCtx(), summary)
    assert plugin._config is not None
    assert plugin._config.mode == "optical_flow_degraded"


def test_auto_flip_leaves_mode_alone_with_rangefinder() -> None:
    """When the rangefinder is detected, the operator's mode stays."""
    plugin = _make_plugin_with_mode("optical_flow")
    summary = _AutoDetectSummary(
        camera_count=1,
        picked_camera_path="/dev/video0",
        rangefinder_driver="garmin_lidarlite_i2c",
        suggested_mode="optical_flow",
        suggested_reason="cam+rangefinder",
    )
    plugin._auto_flip_mode_for_hardware(_StubCtx(), summary)
    assert plugin._config is not None
    assert plugin._config.mode == "optical_flow"


def test_auto_flip_never_touches_vio_modes() -> None:
    """VIO modes do not auto-flip even if rangefinder is absent."""
    plugin = _make_plugin_with_mode("vio_openvins")
    summary = _AutoDetectSummary(
        camera_count=1,
        picked_camera_path="/dev/video0",
        rangefinder_driver=None,
        suggested_mode="vio_openvins",
        suggested_reason="forward cam",
    )
    plugin._auto_flip_mode_for_hardware(_StubCtx(), summary)
    assert plugin._config is not None
    assert plugin._config.mode == "vio_openvins"


def test_auto_detect_merges_picked_camera_into_default_config(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The default /dev/video0 config picks up an enumerated path."""
    plugin = _make_plugin_with_mode("optical_flow")
    # Plant a single UVC node at a non-default path.
    monkeypatch.setattr(
        "glob.glob",
        lambda pattern: ["/dev/video4"] if "video" in pattern else [],
    )
    summary = plugin._auto_detect_hardware(_StubCtx())
    assert summary.picked_camera_path == "/dev/video4"
    assert plugin._config is not None
    assert plugin._config.camera.device_path == "/dev/video4"
