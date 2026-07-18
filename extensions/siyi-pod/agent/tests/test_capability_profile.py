"""Per-model capability-negotiation tests across the whole SIYI lineup."""

from __future__ import annotations

import pytest

from altnautica_siyi_pod import capability_profile as CP


@pytest.mark.parametrize(
    "hw_id,model,zoom,thermal,laser,ai,gimbal",
    [
        (CP.HW_A2_MINI, "A2 mini", False, False, False, False, False),
        (CP.HW_A8_MINI, "A8 mini", True, False, False, True, True),
        (CP.HW_ZR10, "ZR10", True, False, True, False, True),
        (CP.HW_ZR30, "ZR30", True, False, True, False, True),
        (CP.HW_ZT6, "ZT6", False, True, False, True, True),
        (CP.HW_ZT30, "ZT30", True, True, True, True, True),
    ],
)
def test_profiles(hw_id, model, zoom, thermal, laser, ai, gimbal):
    p = CP.profile_for(hw_id)
    assert p.model == model
    assert p.known is True
    assert p.supports("zoom") is zoom
    assert p.supports("thermal") is thermal
    assert p.supports("laser") is laser
    assert p.supports("ai_track") is ai
    assert p.supports("gimbal") is gimbal


def test_zt30_is_the_full_pod():
    p = CP.profile_for(CP.HW_ZT30)
    assert set(p.sensors) == {"eo_zoom", "eo_wide", "ir"}
    assert set(p.streams) == {"eo_zoom", "eo_wide", "ir", "split"}
    assert p.supports_pip is True
    assert p.max_zoom == 180.0
    assert p.yaw_max_deg == 360.0  # limitless


def test_streams_are_the_assignable_source_roles():
    # Single-EO pods advertise only eo_zoom; the ZT6 adds ir + the on-pod split
    # composite; the ZT30 adds the wide EO too.
    assert CP.profile_for(CP.HW_A2_MINI).streams == ("eo_zoom",)
    assert CP.profile_for(CP.HW_A8_MINI).streams == ("eo_zoom",)
    assert set(CP.profile_for(CP.HW_ZT6).streams) == {"eo_zoom", "ir", "split"}
    # can_stream gates a source against the model's assignable set.
    assert CP.profile_for(CP.HW_ZT30).can_stream("split") is True
    assert CP.profile_for(CP.HW_A8_MINI).can_stream("split") is False
    assert CP.profile_for(CP.HW_A8_MINI).can_stream("ir") is False


def test_resolve_leading_code():
    assert CP.resolve_hardware_code(bytes([CP.HW_ZT30, 0x00])) == CP.HW_ZT30


def test_resolve_scans_for_code():
    # A firmware that prefixes a header byte before the model code.
    assert CP.resolve_hardware_code(bytes([0xAA, CP.HW_A8_MINI, 0x00])) == CP.HW_A8_MINI


def test_resolve_unknown_returns_none():
    assert CP.resolve_hardware_code(bytes([0x01, 0x02, 0x03])) is None
    assert CP.resolve_hardware_code(b"") is None


def test_unknown_falls_back_conservatively():
    p = CP.profile_for(None)
    assert p is CP.FALLBACK_PROFILE
    assert p.known is False
    assert p.supports("gimbal") is True
    assert p.supports("zoom") is False
    assert p.supports("thermal") is False
    assert p.supports("laser") is False
