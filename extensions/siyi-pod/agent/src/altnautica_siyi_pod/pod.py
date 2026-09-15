"""The capability-negotiated SIYI pod facade.

:class:`SiyiPod` is what the plugin, the MAVLink bridge and the geolocation path
talk to. It runs one negotiation on connect (firmware + hardware-id), resolves
the model's :class:`CapabilityProfile`, and then exposes typed control methods
that gate on it: a call to an absent feature (zoom on an A2 mini, thermal on a
ZR10), or any call at all before the pod has identified itself, raises
:class:`PodUnsupported` rather than sending a command the pod may not honour.
Gimbal angles are clamped to the model's mechanical range before they go out.
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
        """Query firmware + hardware-id and resolve the capability profile.

        Never raises on a slow / absent / unreachable pod: a firmware or
        hardware-id timeout leaves the conservative fallback profile in place
        with ``negotiated`` still False, so the caller retries rather than the
        whole plugin failing to start. Negotiation is idempotent and
        re-runnable — it re-queries on every call, so a pod that appears late
        (or a hot-swapped model) resolves cleanly on a later attempt.
        """
        try:
            fw = await self._session.request(C.request_firmware())
            self.firmware = "".join(f"{b}" for b in fw.data[:3]) or None
        except Exception:  # noqa: BLE001
            # Firmware is informational; a pod that answers the hardware-id but
            # not the firmware query still negotiates.
            log.info("siyi firmware query failed; continuing", exc_info=True)
        try:
            hw = await self._session.request(C.request_hardware_id())
        except Exception:  # noqa: BLE001
            # The pod did not answer the identity query (late boot, unpowered,
            # unreachable). Stay on the fallback profile; the caller re-runs
            # negotiation until the pod appears.
            log.info("siyi hardware-id query failed; will retry", exc_info=True)
            return self.profile
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
        """Gate a control on a completed negotiation and on the model's profile.

        The negotiation check comes first: until the pod has said what it is,
        nothing is known about it — not its mechanical range, not its sensor
        set, not that it is a SIYI pod at all — so no command may be sent to
        it. The profile check is the second line of defence for a pod that has
        identified itself as a model without the feature.
        """
        if not self.negotiated:
            raise PodUnsupported(
                f"{feature} is unavailable: the pod has not identified itself"
            )
        if not self.profile.supports(feature):
            raise PodUnsupported(
                f"{feature} is not supported by {self.profile.model}"
            )

    def _require_negotiated(self, control: str) -> None:
        """Gate a control that every model carries on a completed negotiation.

        A device that has not answered the identity query is not known to be a
        SIYI pod at all, so even a base camera command must not be sent to it.
        """
        if not self.negotiated:
            raise PodUnsupported(
                f"{control} is unavailable: the pod has not identified itself"
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
        """Rate command, both axes as a percentage of the model's maximum rate.

        Clamped at the facade as well as in the encoder, so the bound the
        facade advertises is the bound it enforces.
        """
        self._require("gimbal")
        yaw_pct = max(-100, min(100, int(yaw)))
        pitch_pct = max(-100, min(100, int(pitch)))
        await self._session.command(C.gimbal_speed(yaw_pct, pitch_pct))

    async def center(self) -> None:
        self._require("gimbal")
        await self._session.request(C.center())

    async def set_mode(self, mode: str) -> None:
        self._require("gimbal")
        await self._session.request(C.set_gimbal_mode(mode))

    async def read_attitude(self) -> C.GimbalAttitude:
        self._require_negotiated("gimbal attitude")
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
        self._require_negotiated("autofocus")
        await self._session.request(C.autofocus())

    async def read_zoom(self) -> float:
        self._require_negotiated("zoom read-back")
        reply = await self._session.request(C.request_current_zoom())
        return C.decode_current_zoom(reply.data)

    async def take_photo(self) -> None:
        self._require_negotiated("photo")
        await self._session.request(C.take_photo())

    async def toggle_record(self) -> None:
        self._require_negotiated("recording")
        await self._session.request(C.record_toggle())

    # -- thermal ----------------------------------------------------------
    async def set_palette(self, palette_code: int) -> None:
        """Select a thermal palette by code, refusing an out-of-range value.

        A code outside the accepted range is rejected here rather than wrapped
        into a different palette by the encoder.
        """
        self._require("thermal")
        try:
            code = int(palette_code)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"palette code must be an integer, got {palette_code!r}"
            ) from exc
        await self._session.request(C.set_thermal_palette(code))

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
