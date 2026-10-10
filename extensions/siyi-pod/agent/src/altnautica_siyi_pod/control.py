"""The config-driven control loop of the SIYI pod plugin.

The loop reads the per-drone config keys the GCS half writes and applies them
to the pod: declarative state keys (zoom, gimbal mode, palette, thermal gain,
recording) plus one-shot command-nonce keys (photo, record, recenter, fire
laser) fired once per new nonce, and the cockpit Skills' one-shot boolean
action keys. Every call is gated first on the pod having identified itself and
then on the negotiated capability profile.

Each config read is an IPC round-trip to the host, so the declarative state
keys are re-read in full at most once per ``_CONFIG_REFRESH_S`` and served from
cache in between; only the one-shot nonce and action keys are polled every
tick, so a press is never missed or delayed.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.geolocation import geolocate
from altnautica_siyi_pod.pod import PodUnsupported

log = logging.getLogger(__name__)

_CONTROL_HZ = 5.0

# How often the declarative state keys are re-read in full.
_CONFIG_REFRESH_S = 1.0

# Declarative state keys the control loop diffs each tick.
_STATE_KEYS = (
    "zoom",
    "gimbal_mode",
    "palette",
    "thermal_gain",
    "recording",
)

# One-shot keys: an action fires once each time its integer nonce increases.
_NONCE_KEYS = (
    "photo_nonce",
    "record_nonce",
    "recenter_nonce",
    "laser_fire_nonce",
)

# One-shot boolean action keys the cockpit Skills flip to true. The control
# loop fires the matching command on the rising edge and clears the key, so a
# re-press fires again (the Skill Bar writes the flag true each press with no
# nonce). Each command is gated on the negotiated capability profile, so a
# Skill for a control the model lacks (zoom on an A2 mini) is a safe no-op.
_ACTION_KEYS = (
    "palette_cycle",
    "laser_fire",
    "center",
    "nadir",
    "zoom_in",
    "zoom_out",
    "photo",
)

# Per-press zoom step, in absolute-zoom units.
_ZOOM_STEP = 1.0


def _monotonic() -> float:
    return time.monotonic()


class ControlMixin:
    """Applies the per-drone config to the pod.

    Mixed into :class:`~altnautica_siyi_pod.plugin.SiyiPodPlugin`, which owns
    ``_ctx``, ``_pod``, ``_bridge``, ``_pose``, ``_state``, ``_applied``,
    ``_nonces``, ``_state_cfg``, ``_state_cfg_read_at`` and
    ``_last_laser_target`` and supplies ``_cfg`` and ``_try_renegotiate``.
    """

    async def _control_loop(self) -> None:
        period = 1.0 / _CONTROL_HZ
        while True:
            await asyncio.sleep(period)
            try:
                await self.apply_config_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                log.warning("siyi control tick failed: %s", exc)

    async def apply_config_once(self) -> None:
        """Read the config keys and apply any change / fire any new nonce.

        Nothing is applied until the pod has identified itself. A device that
        has not answered the identity query has no known mechanical range and
        no known feature set, and may not be a SIYI pod at all — so the loop
        retries the negotiation and otherwise sends nothing, rather than
        commanding an unidentified device against assumed limits.

        Deterministic and idempotent so tests can drive it directly.
        """
        pod = self._pod
        if pod is None:
            return
        # Bring an unreachable-at-boot pod online once it answers. The retry is
        # unbounded and on a fixed interval: a pod that appears late, or is
        # power-cycled in flight, comes online with no plugin restart.
        if not pod.negotiated:
            await self._try_renegotiate()
        if not pod.negotiated:
            return
        # Declarative state keys, from the 1 Hz cache.
        state_cfg = await self._config_for_tick()
        for key in _STATE_KEYS:
            value = state_cfg.get(key)
            if value is None or self._applied.get(key) == value:
                continue
            self._applied[key] = value
            await self._apply_state(key, value)
        # One-shot nonce keys, polled every tick.
        for key in _NONCE_KEYS:
            nonce = await self._cfg(key, 0)
            try:
                nonce = int(nonce)
            except (TypeError, ValueError):
                continue
            if nonce > self._nonces.get(key, 0):
                self._nonces[key] = nonce
                await self._fire_nonce(key)
        # One-shot boolean action keys (the cockpit Skills), polled every tick:
        # fire on the rising edge and clear so a re-press fires again.
        for key in _ACTION_KEYS:
            if bool(await self._cfg(key, False)):
                await self._fire_action(key)
                await self._reset_key(key)

    async def _config_for_tick(self) -> dict[str, Any]:
        """The declarative state keys for one control tick.

        The full key set is re-read only every ``_CONFIG_REFRESH_S``; between
        refreshes the cached values are served. The plugin's own config writes
        (a tool, a zoom step, a palette cycle) update the cache directly, so
        they never wait for the next refresh.
        """
        now = _monotonic()
        if (
            self._state_cfg is None
            or (now - self._state_cfg_read_at) >= _CONFIG_REFRESH_S
        ):
            fresh: dict[str, Any] = {}
            for key in _STATE_KEYS:
                fresh[key] = await self._cfg(key, None)
            self._state_cfg = fresh
            self._state_cfg_read_at = now
        return self._state_cfg

    async def _apply_state(self, key: str, value: Any) -> None:
        pod = self._pod
        assert pod is not None
        try:
            if key == "zoom":
                await pod.set_zoom(float(value))
            elif key == "gimbal_mode":
                await pod.set_mode(str(value))
            elif key == "palette":
                await pod.set_palette(int(value))
            elif key == "thermal_gain":
                await pod.set_gain(bool(value))
            elif key == "recording":
                # The record toggle drives the pod's start/stop through its
                # single toggle command: send it only when the desired state
                # differs from what the pod is doing, and track the result so
                # the Skill Bar reflects it.
                desired = bool(value)
                if desired != self._state.recording:
                    await pod.toggle_record()
                    self._state.recording = desired
        except PodUnsupported as exc:
            log.info("ignoring unsupported control %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("control %s failed: %s", key, exc)

    async def _fire_nonce(self, key: str) -> None:
        pod = self._pod
        assert pod is not None
        try:
            if key == "photo_nonce":
                await pod.take_photo()
            elif key == "record_nonce":
                await pod.toggle_record()
                # Keep the read-back in step with the toggle so the record
                # Skill reflects it whether it was fired by the tool or the
                # Skill Bar.
                self._state.recording = not self._state.recording
            elif key == "recenter_nonce":
                await pod.center()
            elif key == "laser_fire_nonce":
                await self._measure_laser()
        except PodUnsupported as exc:
            log.info("ignoring unsupported action %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("action %s failed: %s", key, exc)

    async def _measure_laser(self) -> float:
        """Fire the rangefinder, mirror the range to the flight controller, and
        (when a vehicle pose is available) resolve the geolocated target the
        geolocate tool returns. Returns the measured slant range. Shared by the
        laser-fire one-shot and the laser/geolocate tools."""
        pod = self._pod
        assert pod is not None
        range_m = await pod.read_laser_range()
        self._state.laser_range_m = range_m
        if self._bridge is not None:
            await self._bridge.send_distance(range_m)
        self._last_laser_target = None
        if self._pose.ready:
            att = self._state
            target = geolocate(
                vehicle_lat_deg=self._pose.lat_deg,
                vehicle_lon_deg=self._pose.lon_deg,
                vehicle_rel_alt_m=self._pose.rel_alt_m,
                vehicle_yaw_deg=self._pose.yaw_deg,
                gimbal_yaw_deg=att.yaw_deg or 0.0,
                gimbal_pitch_deg=att.pitch_deg or 0.0,
                slant_range_m=range_m,
            )
            self._last_laser_target = {
                "lat_deg": target.lat_deg,
                "lon_deg": target.lon_deg,
                "rel_alt_m": target.rel_alt_m,
                "slant_range_m": target.slant_range_m,
                "bearing_deg": target.bearing_deg,
            }
        return range_m

    # -- one-shot Skill actions -------------------------------------------
    async def _fire_action(self, key: str) -> None:
        pod = self._pod
        if pod is None:
            return
        try:
            if key == "photo":
                await pod.take_photo()
            elif key == "center":
                await pod.center()
            elif key == "nadir":
                # Point straight down: recenter yaw, drive pitch to the model's
                # lower mechanical limit.
                await pod.set_attitude(0.0, pod.profile.pitch_min_deg)
            elif key == "laser_fire":
                await self._measure_laser()
            elif key == "palette_cycle":
                await self._cycle_palette()
            elif key == "zoom_in":
                await self._step_zoom(_ZOOM_STEP)
            elif key == "zoom_out":
                await self._step_zoom(-_ZOOM_STEP)
        except PodUnsupported as exc:
            log.info("ignoring unsupported skill %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("skill %s failed: %s", key, exc)

    async def _cycle_palette(self) -> None:
        pod = self._pod
        assert pod is not None
        current = self._applied.get("palette")
        if current is None:
            current = self._state.palette if self._state.palette is not None else 0
        nxt = (int(current) + 1) % C.THERMAL_PALETTE_COUNT
        await pod.set_palette(nxt)
        self._applied["palette"] = nxt
        self._state.palette = nxt
        await self._set_cfg("palette", nxt)

    async def _step_zoom(self, delta: float) -> None:
        pod = self._pod
        assert pod is not None
        current = self._applied.get("zoom")
        if current is None:
            current = self._state.zoom if self._state.zoom is not None else 1.0
        nxt = max(1.0, min(pod.profile.max_zoom, float(current) + delta))
        await pod.set_zoom(nxt)
        self._applied["zoom"] = nxt
        self._state.zoom = nxt
        await self._set_cfg("zoom", nxt)

    async def _reset_key(self, key: str) -> None:
        await self._set_cfg(key, False)

    async def _set_cfg(self, key: str, value: Any) -> None:
        # Keep the state-key cache in step with the plugin's own writes.
        if self._state_cfg is not None and key in _STATE_KEYS:
            self._state_cfg[key] = value
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is None:
            return
        try:
            await setter(key, value)
        except Exception:  # noqa: BLE001
            log.debug("siyi config write failed: %s", key, exc_info=True)

    async def _bump_nonce(self, key: str) -> None:
        try:
            current = int(await self._cfg(key, 0) or 0)
        except (TypeError, ValueError):
            current = 0
        await self._set_cfg(key, current + 1)
