"""Tests for the pre-arm gate evaluator.

Pins the per-mode check set and the severity ladder so a future
refactor that splits the gate into multiple files (or shifts a check
between blocking and pending) is caught at PR time.
"""

from __future__ import annotations

import pytest

from altnautica_vision_nav.pre_arm_gate import (
    PreArmCheck,
    PreArmGate,
    PreArmInputs,
    PreArmReport,
)


def _ids(report: PreArmReport) -> list[str]:
    return [c.id for c in report.checks]


def _severity(report: PreArmReport, check_id: str) -> str:
    for c in report.checks:
        if c.id == check_id:
            return c.severity
    raise AssertionError(f"check {check_id!r} missing from report")


def test_off_mode_armable_with_no_checks() -> None:
    gate = PreArmGate()
    report = gate.evaluate(PreArmInputs(mode="off"))
    assert report.armable is True
    assert report.checks == ()


def test_unknown_mode_blocks_arm() -> None:
    gate = PreArmGate()
    report = gate.evaluate(PreArmInputs(mode="invented_xyz"))
    assert report.armable is False
    assert _severity(report, "mode_unknown") == "blocking"


def test_optical_flow_requires_rangefinder() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="optical_flow",
            companion_state="active",
            flow_quality=180,
            rangefinder_topology=None,
        )
    )
    assert _ids(report) == ["companion_active", "flow_quality", "rangefinder"]
    assert _severity(report, "rangefinder") == "blocking"
    assert report.armable is False


def test_optical_flow_passes_with_all_inputs_healthy() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="optical_flow",
            companion_state="active",
            flow_quality=180,
            rangefinder_topology="companion",
        )
    )
    assert report.armable is True
    assert all(c.severity == "ok" for c in report.checks)


def test_optical_flow_degraded_accepts_baro_scale() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="optical_flow_degraded",
            companion_state="active",
            flow_quality=120,
            flow_scale_source="baro",
            rangefinder_topology=None,
        )
    )
    assert _ids(report) == ["companion_active", "flow_quality", "scale_source"]
    assert report.armable is True


def test_optical_flow_degraded_blocks_without_any_scale() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="optical_flow_degraded",
            companion_state="active",
            flow_quality=120,
            flow_scale_source=None,
        )
    )
    assert _severity(report, "scale_source") == "blocking"
    assert report.armable is False


def test_vio_blocks_until_intrinsics_loaded() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=False,
            extrinsics_loaded=True,
            sync_offset_ms=5.0,
            feature_count=30,
        )
    )
    assert _severity(report, "intrinsics_loaded") == "blocking"
    assert report.armable is False


def test_vio_blocks_until_extrinsics_loaded() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_vins_fusion",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=False,
            sync_offset_ms=5.0,
            feature_count=30,
        )
    )
    assert _severity(report, "extrinsics_loaded") == "blocking"
    assert report.armable is False


def test_vio_blocks_when_sync_offset_in_red_band() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=45.0,  # > 30 ms red threshold
            feature_count=30,
        )
    )
    assert _severity(report, "sync_offset") == "blocking"
    assert report.armable is False


def test_vio_pending_when_sync_offset_unknown() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=None,
            feature_count=30,
        )
    )
    assert _severity(report, "sync_offset") == "pending"
    assert report.armable is False  # pending != ok


def test_vio_blocks_when_feature_count_low() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=8.0,
            feature_count=10,  # below default floor 20
        )
    )
    assert _severity(report, "feature_count") == "blocking"


def test_vio_pending_when_estimator_converging() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converging",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=8.0,
            feature_count=30,
        )
    )
    assert _severity(report, "estimator_converged") == "pending"
    assert report.armable is False


def test_vio_passes_with_all_inputs_healthy() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="vio_openvins",
            companion_state="active",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=8.0,
            feature_count=30,
        )
    )
    assert report.armable is True
    assert all(c.severity == "ok" for c in report.checks)


def test_hybrid_mode_combines_of_plus_vio_checks() -> None:
    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(
            mode="hybrid_of_plus_vio",
            companion_state="active",
            flow_quality=180,
            flow_scale_source="baro",
            estimator_state="converged",
            intrinsics_loaded=True,
            extrinsics_loaded=True,
            sync_offset_ms=8.0,
            feature_count=30,
        )
    )
    ids = _ids(report)
    # OF set (companion + flow_quality + scale_source) plus VIO set
    # (companion + estimator + intrinsics + extrinsics + sync +
    # feature). companion appears twice because hybrid pulls both
    # check tuples; this is fine — the GCS dedupes by id.
    for required in (
        "companion_active",
        "flow_quality",
        "scale_source",
        "estimator_converged",
        "intrinsics_loaded",
        "extrinsics_loaded",
        "sync_offset",
        "feature_count",
    ):
        assert required in ids
    assert report.armable is True


def test_report_as_dict_round_trip() -> None:
    """The wire dict shape matches the GCS pre-arm expected fields."""

    gate = PreArmGate()
    report = gate.evaluate(
        PreArmInputs(mode="optical_flow", companion_state="inactive")
    )
    payload = report.as_dict()
    assert payload["mode"] == "optical_flow"
    assert "armable" in payload
    assert isinstance(payload["checks"], list)
    for check in payload["checks"]:
        assert set(check.keys()) == {"id", "severity", "detail"}
