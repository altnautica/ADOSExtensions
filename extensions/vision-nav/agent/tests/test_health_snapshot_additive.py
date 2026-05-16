"""Tests for the additive heartbeat fields from the estimator framework.

The new ``mode``, ``availableEstimators``, ``estimatorState`` and
``flowScaleSource`` keys land on every snapshot. Existing fields keep
the same names and types so the cloud relay and GCS normalizer both
stay backward-compatible.
"""

from __future__ import annotations

import pytest

from altnautica_vision_nav.health import HealthPublisher
from altnautica_vision_nav.mavlink.comp_status import CompanionState


def test_snapshot_carries_phase1_additive_fields() -> None:
    """Snapshot includes the four new keys with sensible defaults."""

    pub = HealthPublisher(
        rangefinder_topology="fc",
        recommended_camera_id="/dev/video0",
        mode="optical_flow",
        available_estimators=["off", "optical_flow"],
    )
    snap = pub.snapshot()
    assert snap["mode"] == "optical_flow"
    assert snap["availableEstimators"] == ["off", "optical_flow"]
    assert snap["estimatorState"] == "off"  # default companion is INACTIVE
    assert snap["flowScaleSource"] == "rangefinder"  # fc topology => rangefinder


def test_snapshot_flow_scale_none_when_topology_none() -> None:
    """``rangefinder.topology = "none"`` maps to ``flowScaleSource = None``.

    Today no other scale source is wired. ``"baro"`` / ``"gps"`` /
    ``"vision"`` join once the rangefinder-free OF estimator lands.
    """

    pub = HealthPublisher(
        rangefinder_topology=None,
        mode="optical_flow",
        available_estimators=["off", "optical_flow"],
    )
    assert pub.snapshot()["flowScaleSource"] is None


@pytest.mark.parametrize(
    "companion,expected",
    [
        (CompanionState.INACTIVE, "off"),
        (CompanionState.ACTIVE, "converged"),
        (CompanionState.CRITICAL, "degraded"),
        (CompanionState.TERMINATING, "failed"),
    ],
)
def test_estimator_state_maps_from_companion(
    companion: CompanionState, expected: str
) -> None:
    """The companion-state -> estimator-state mapping is total."""

    pub = HealthPublisher(rangefinder_topology="fc")
    pub.update_companion_state(companion)
    assert pub.snapshot()["estimatorState"] == expected


def test_set_mode_updates_snapshot() -> None:
    """``set_mode`` mutates the field surfaced on the next snapshot."""

    pub = HealthPublisher()
    assert pub.snapshot()["mode"] is None
    pub.set_mode("optical_flow")
    assert pub.snapshot()["mode"] == "optical_flow"
    pub.set_mode("off")
    assert pub.snapshot()["mode"] == "off"


def test_set_available_estimators_replaces_list() -> None:
    """``set_available_estimators`` overwrites the previous list."""

    pub = HealthPublisher(available_estimators=["off"])
    assert pub.snapshot()["availableEstimators"] == ["off"]
    pub.set_available_estimators(["off", "optical_flow", "vio_openvins"])
    assert pub.snapshot()["availableEstimators"] == [
        "off",
        "optical_flow",
        "vio_openvins",
    ]


def test_existing_fields_unchanged() -> None:
    """Backward-compat: the legacy keys keep their shape and defaults."""

    pub = HealthPublisher(
        rangefinder_topology="companion",
        recommended_camera_id="/dev/video1",
    )
    snap = pub.snapshot()
    assert snap["opticalFlowSupported"] is True
    assert snap["vioSupported"] is False
    assert snap["rangefinderTopology"] == "companion"
    assert snap["recommendedCameraId"] == "/dev/video1"
    assert snap["flowQuality"] is None
    assert snap["flowRateHz"] is None
    assert snap["flowDistanceM"] is None
    assert snap["vioState"] == "absent"
    assert snap["vioResetCounter"] == 0
    assert snap["vioQuality"] is None
    assert snap["companionState"] == "inactive"
