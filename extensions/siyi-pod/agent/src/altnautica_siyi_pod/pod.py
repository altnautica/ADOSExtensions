"""The capability-negotiated SIYI pod facade.

:class:`SiyiPod` is what the plugin, the MAVLink bridge, the tracker bridge, and
the geolocation path talk to. It runs one negotiation on connect (firmware +
hardware-id), resolves the model's :class:`CapabilityProfile`, and then exposes
typed control methods that gate on the profile: a call to an absent feature
(zoom on an A2 mini, thermal on a ZR10) raises :class:`PodUnsupported` rather
than sending a command the pod cannot honour. Gimbal angles are clamped to the
model's mechanical range before they go out.
"""

from __future__ import annotations

import logging

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.capability_profile import (
    CapabilityProfile,
    FALLBACK_PROFILE,
    profile_for,
    resolve_hardware_code,
)
from altnautica_siyi_pod.session import SiyiSession

log = logging.getLogger(__name__)


class PodUnsupported(RuntimeError):
    """A control was called that the negotiated model does not support."""


class SiyiPod:
    """Typed, capability-gated control surface over a :class:`SiyiSession`."""

    def __init__(self, session: SiyiSession) -> None:
        self._session = session
        self.profile: CapabilityProfile = FALLBACK_PROFILE
        self.firmware: str | None = None
        self.negotiated = False

    # -- negotiation ------------------------------------------------------
    async def negotiate(self) -> CapabilityProfile:
        """Query firmware + hardware-id and resolve the capability profile."""
        try:
            fw = await self._session.request(C.request_firmware())
            self.firmware = "".join(f"{b}" for b in fw.data[:3]) or None
        except Exception:  # noqa: BLE001
            # Firmware is informational; a pod that answers the hardware-id but
            # not the firmware query still negotiates.
            log.info("siyi firmware query failed; continuing", exc_info=True)
        hw = await self._session.request(C.request_hardware_id())
        code = resolve_hardware_code(hw.data)
        self.profile = profile_for(code)
        self.negotiated = True
        log.info(
            "siyi pod negotiated: model=%s known=%s",
            self.profile.model,
            self.profile.known,
        )
        return self.profile

    def _require(self, feature: str) -> None:
        if not self.profile.supports(feature):
            raise PodUnsupported(
                f"{feature} is not supported by {self.profile.model}"
            )

    # -- gimbal -----------------------------------------------------------
    async def set_attitude(self, yaw_deg: float, pitch_deg: float) -> None:
        self._require("gimbal")
        yaw = _clamp(yaw_deg, self.profile.yaw_min_deg, self.profile.yaw_max_deg)
        pitch = _clamp(
            pitch_deg, self.profile.pitch_min_deg, self.profile.pitch_max_deg
        )
        await self._session.request(C.set_gimbal_attitude(yaw, pitch))

    async def command_rate(self, yaw: int, pitch: int) -> None:
        self._require("gimbal")
        await self._session.command(C.gimbal_speed(yaw, pitch))

    async def center(self) -> None:
        self._require("gimbal")
        await self._session.request(C.center())

    async def set_mode(self, mode: str) -> None:
        self._require("gimbal")
        await self._session.request(C.set_gimbal_mode(mode))

    async def read_attitude(self) -> C.GimbalAttitude:
        reply = await self._session.request(C.request_gimbal_attitude())
        return C.decode_gimbal_attitude(reply.data)

    # -- camera -----------------------------------------------------------
    async def set_zoom(self, zoom: float) -> None:
        self._require("zoom")
        zoom = _clamp(zoom, 1.0, self.profile.max_zoom)
        await self._session.request(C.absolute_zoom(zoom))

    async def zoom_rocker(self, direction: int) -> None:
        self._require("zoom")
        await self._session.command(C.manual_zoom(direction))

    async def autofocus(self) -> None:
        await self._session.request(C.autofocus())

    async def read_zoom(self) -> float:
        reply = await self._session.request(C.request_current_zoom())
        return C.decode_current_zoom(reply.data)

    async def take_photo(self) -> None:
        await self._session.request(C.take_photo())

    async def toggle_record(self) -> None:
        await self._session.request(C.record_toggle())

    # -- thermal ----------------------------------------------------------
    async def set_palette(self, palette_code: int) -> None:
        self._require("thermal")
        await self._session.request(C.set_thermal_palette(palette_code))

    async def set_gain(self, high: bool) -> None:
        self._require("thermal")
        await self._session.request(C.set_thermal_gain(high))

    # -- laser rangefinder ------------------------------------------------
    async def read_laser_range(self) -> float:
        """Return the pod's slant range to the pointed subject, in metres."""
        self._require("laser")
        reply = await self._session.request(C.request_laser_range())
        return C.decode_laser_range(reply.data)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))
