"""Per-model capability negotiation for the SIYI optical-pod line.

Every SIYI pod answers the same hardware-id request (command 0x02) with a code
that identifies the model. The driver resolves that code to a
:class:`CapabilityProfile` and then gates every control, UI surface, and MAVLink
message on the profile, so one plugin drives an A2 mini (a fixed EO camera) and a
ZT30 (a four-sensor pod) from the same code with no per-model branches in the
call sites.

An unknown or future code resolves to a conservative gimbal-plus-EO fallback so a
new pod still connects and streams video while the operator is told to report the
model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Hardware-id codes (confirmed against the SIYI SDK model table).
HW_A2_MINI = 0x75
HW_A8_MINI = 0x73
HW_ZR10 = 0x6B
HW_ZR30 = 0x78
HW_ZT6 = 0x82
HW_ZT30 = 0x7A

_KNOWN_CODES = (HW_A2_MINI, HW_A8_MINI, HW_ZR10, HW_ZR30, HW_ZT6, HW_ZT30)


@dataclass(frozen=True)
class CapabilityProfile:
    """What one pod model can physically do."""

    model: str
    hw_id: int | None
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float
    has_gimbal_control: bool
    has_optical_zoom: bool
    max_zoom: float
    has_thermal: bool
    has_laser: bool
    has_ai_track: bool
    # Stream roles the pod can output (drive the video-source configuration).
    sensors: tuple[str, ...] = field(default_factory=tuple)
    supports_split_pip: bool = False
    known: bool = True

    def supports(self, feature: str) -> bool:
        """Feature gate used by the pod facade and the surfaces.

        ``feature`` is one of: gimbal, zoom, thermal, laser, ai_track.
        """
        return {
            "gimbal": self.has_gimbal_control,
            # Any zoom (optical or digital); the console distinguishes them via
            # ``has_optical_zoom``.
            "zoom": self.max_zoom > 1.0,
            "thermal": self.has_thermal,
            "laser": self.has_laser,
            "ai_track": self.has_ai_track,
        }.get(feature, False)


# The lineup. Ranges and sensor sets follow the SIYI product specifications;
# the ZT30 (the bench reference) is validated on real hardware (Rule 25).
PROFILES: dict[int, CapabilityProfile] = {
    HW_A2_MINI: CapabilityProfile(
        model="A2 mini",
        hw_id=HW_A2_MINI,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,  # fixed mount
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=False,
        has_optical_zoom=False,
        max_zoom=1.0,
        has_thermal=False,
        has_laser=False,
        has_ai_track=False,
        sensors=("eo",),
    ),
    HW_A8_MINI: CapabilityProfile(
        model="A8 mini",
        hw_id=HW_A8_MINI,
        yaw_min_deg=-135.0,
        yaw_max_deg=135.0,
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=True,
        has_optical_zoom=False,  # digital only
        max_zoom=6.0,
        has_thermal=False,
        has_laser=False,
        has_ai_track=True,
        sensors=("eo",),
    ),
    HW_ZR10: CapabilityProfile(
        model="ZR10",
        hw_id=HW_ZR10,
        yaw_min_deg=-135.0,
        yaw_max_deg=135.0,
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=True,
        has_optical_zoom=True,
        max_zoom=30.0,
        has_thermal=False,
        has_laser=True,
        has_ai_track=False,
        sensors=("zoom",),
    ),
    HW_ZR30: CapabilityProfile(
        model="ZR30",
        hw_id=HW_ZR30,
        yaw_min_deg=-270.0,
        yaw_max_deg=270.0,
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=True,
        has_optical_zoom=True,
        max_zoom=180.0,
        has_thermal=False,
        has_laser=True,
        has_ai_track=False,
        sensors=("zoom",),
    ),
    HW_ZT6: CapabilityProfile(
        model="ZT6",
        hw_id=HW_ZT6,
        yaw_min_deg=-135.0,
        yaw_max_deg=135.0,
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=True,
        has_optical_zoom=False,
        max_zoom=1.0,
        has_thermal=True,
        has_laser=False,
        has_ai_track=True,
        sensors=("eo", "thermal"),
        supports_split_pip=True,
    ),
    HW_ZT30: CapabilityProfile(
        model="ZT30",
        hw_id=HW_ZT30,
        yaw_min_deg=-360.0,  # limitless yaw
        yaw_max_deg=360.0,
        pitch_min_deg=-90.0,
        pitch_max_deg=25.0,
        has_gimbal_control=True,
        has_optical_zoom=True,
        max_zoom=180.0,
        has_thermal=True,
        has_laser=True,
        has_ai_track=True,
        sensors=("zoom", "wide", "thermal"),
        supports_split_pip=True,
    ),
}

# The fallback for an unrecognised pod: control the gimbal and the EO stream,
# hide every model-specific feature, and flag the model as unknown so the GCS can
# ask the operator to report it.
FALLBACK_PROFILE = CapabilityProfile(
    model="Unknown SIYI pod",
    hw_id=None,
    yaw_min_deg=-135.0,
    yaw_max_deg=135.0,
    pitch_min_deg=-90.0,
    pitch_max_deg=25.0,
    has_gimbal_control=True,
    has_optical_zoom=False,
    max_zoom=1.0,
    has_thermal=False,
    has_laser=False,
    has_ai_track=False,
    sensors=("eo",),
    known=False,
)


def resolve_hardware_code(reply_data: bytes) -> int | None:
    """Extract the model code from a hardware-id (0x02) reply payload.

    The reply leads with the model code. If the leading byte is not a known
    code the payload is scanned for the first known code (tolerant of a
    firmware that prefixes a header), and ``None`` is returned when nothing
    matches so the caller uses the fallback profile.
    """
    if not reply_data:
        return None
    if reply_data[0] in _KNOWN_CODES:
        return reply_data[0]
    for byte in reply_data:
        if byte in _KNOWN_CODES:
            return byte
    return None


def profile_for(hw_code: int | None) -> CapabilityProfile:
    """Resolve a hardware code to its capability profile (or the fallback)."""
    if hw_code is None:
        return FALLBACK_PROFILE
    return PROFILES.get(hw_code, FALLBACK_PROFILE)
