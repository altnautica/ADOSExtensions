"""The gimbal plugin's control loop: one-shot Skill actions, the Gimbal tab's
point / ROI commands, and the 1 Hz refresh of the aim and rate-mode flags.

A mixin of :class:`altnautica_gimbal_v2.plugin.GimbalV2Plugin`; it reads and
writes the plugin's own attributes (``_ctx``, ``_driver``, ``_session``,
``_aim_active``, ``_rate_mode`` and friends) and calls its ``_point_at``.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Awaitable, Callable
from typing import Any

log = logging.getLogger("altnautica_gimbal_v2.plugin")

# How often the control loop reads the one-shot keys (the Skills' actions and
# the panel's point / ROI commands). Each read is one host round-trip.
_CONTROL_HZ = 5.0

# How often the control loop refreshes the persistent state keys (``aim``,
# ``rate_mode``). They change only on an operator toggle, so reading them once
# a second instead of every tick (or every detection batch) keeps the idle
# IPC load to the one-shot keys alone.
_STATE_REFRESH_S = 1.0
_STATE_KEYS = ("aim", "rate_mode")

# One-shot action keys the cockpit Skills flip to true. The control loop fires
# the matching command on the rising edge and resets the key so a re-press
# fires again (the Skill Bar has no nonce; it writes the flag true each press).
_ACTION_KEYS = ("recenter", "nadir")

# Panel command keys. The Gimbal tab writes ``point`` ({pitch_deg, yaw_deg}),
# ``roi`` ({lat_deg, lon_deg, alt_m}) or ``roi_clear`` (true); the control loop
# executes each once and resets it, like the one-shot Skills.
_POINT_KEY = "point"
_ROI_KEY = "roi"
_ROI_CLEAR_KEY = "roi_clear"


def finite_numbers(*values: Any) -> bool:
    """True when every value is a real, finite number (bools excluded)."""
    return all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in values
    )


class GimbalControlMixin:
    """Control-loop half of the gimbal plugin."""

    _ctx: Any
    _driver: Any
    _session: Any
    _aim_active: bool
    _rate_mode: bool
    _nadir_pitch: float
    _state_refreshed_at: float | None
    _last_rate_published: bool | None
    # Provided by the plugin class.
    _point_at: Callable[..., Awaitable[dict]]


    async def _control_loop(self) -> None:
        period = 1.0 / _CONTROL_HZ
        while True:
            await asyncio.sleep(period)
            try:
                await self.poll_control_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                log.warning("gimbal control tick failed: %s", exc)

    async def _refresh_state(self, ctx: Any) -> None:
        """Re-read the persistent state keys and publish a rate-mode change."""
        for key in _STATE_KEYS:
            try:
                value = await ctx.config_kv.get(key, None)
            except Exception:  # noqa: BLE001
                log.debug("gimbal config refresh failed: %s", key, exc_info=True)
                continue
            if value is None:
                continue
            if key == "aim":
                self._aim_active = bool(value)
            else:
                self._rate_mode = bool(value)
        # Publish the rate-mode Skill state on change, so the Skill Bar toggle
        # reflects the mode the loop is actually in rather than a permanent idle.
        if self._rate_mode != self._last_rate_published:
            self._last_rate_published = self._rate_mode
            try:
                await ctx.events.publish(
                    "sensor.gimbal.rate_mode",
                    {"state": "active" if self._rate_mode else "idle"},
                )
            except Exception:  # noqa: BLE001
                log.debug("gimbal rate-mode state publish failed", exc_info=True)

    async def poll_control_once(self, now: float | None = None) -> None:
        """Fire any pending one-shot action and, at most once per
        ``_STATE_REFRESH_S``, refresh the aim and rate-mode flags.

        Deterministic and idempotent so tests can drive it directly; ``now``
        (monotonic seconds) defaults to the clock.
        """
        ctx = self._ctx
        if ctx is None:
            return
        now = time.monotonic() if now is None else now
        last = self._state_refreshed_at
        if last is None or now - last >= _STATE_REFRESH_S:
            self._state_refreshed_at = now
            await self._refresh_state(ctx)
        for key in _ACTION_KEYS:
            try:
                fired = bool(await ctx.config_kv.get(key, False))
            except Exception:  # noqa: BLE001
                fired = False
            if not fired:
                continue
            await self._fire_action(key)
            await self._reset_key(key)
        await self._run_panel_commands(ctx)

    async def _run_panel_commands(self, ctx: Any) -> None:
        """Execute the Gimbal tab's point / ROI commands once each and reset
        them. A value that is not the documented shape is dropped (reset) with
        a warning rather than sent."""
        point = await ctx.config_kv.get(_POINT_KEY, None)
        if isinstance(point, dict):
            pitch, yaw = point.get("pitch_deg"), point.get("yaw_deg")
            if finite_numbers(pitch, yaw):
                await self._point_at(float(pitch), float(yaw))
            else:
                log.warning("gimbal point command is malformed: %r", point)
            await self._reset_key(_POINT_KEY)
        roi = await ctx.config_kv.get(_ROI_KEY, None)
        if isinstance(roi, dict):
            lat, lon, alt = roi.get("lat_deg"), roi.get("lon_deg"), roi.get("alt_m")
            driver, session = self._driver, self._session
            if not finite_numbers(lat, lon, alt) or abs(lat) > 90 or abs(lon) > 180:
                log.warning("gimbal ROI command is malformed: %r", roi)
            elif driver is not None and session is not None:
                try:
                    await driver.set_roi_location(session, float(lat), float(lon), float(alt))
                except Exception as exc:  # noqa: BLE001
                    log.warning("gimbal ROI lock failed: %s", exc)
            await self._reset_key(_ROI_KEY)
        if bool(await ctx.config_kv.get(_ROI_CLEAR_KEY, False)):
            driver, session = self._driver, self._session
            if driver is not None and session is not None:
                try:
                    await driver.clear_roi(session)
                except Exception as exc:  # noqa: BLE001
                    log.warning("gimbal ROI release failed: %s", exc)
            await self._reset_key(_ROI_CLEAR_KEY)

    async def _fire_action(self, key: str) -> None:
        if key == "recenter":
            await self._point_at(0.0, 0.0, recenter_model=True)
        elif key == "nadir":
            await self._point_at(self._nadir_pitch, 0.0)

    async def _reset_key(self, key: str) -> None:
        """Reset a one-shot action key so a re-press fires again. The Skill Bar
        writes the flag true each press with no nonce, so the agent clears it
        after firing."""
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is None:
            return
        try:
            await setter(key, False)
        except Exception:  # noqa: BLE001
            log.debug("gimbal action key reset failed: %s", key, exc_info=True)
