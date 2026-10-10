"""MCP tools the SIYI pod plugin registers.

The tools write the same per-drone config keys the GCS half writes, so the
control loop is the one path that drives the pod; the laser and geolocate tools
measure directly and return the result.
"""

from __future__ import annotations

import math
from typing import Any

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.pod import PodUnsupported


class ToolsMixin:
    """Registers and serves the plugin's MCP tools.

    Mixed into :class:`~altnautica_siyi_pod.plugin.SiyiPodPlugin`, which owns
    ``_pod``, ``_state`` and ``_last_laser_target`` and supplies ``_set_cfg``,
    ``_bump_nonce`` and ``_measure_laser``.
    """

    def _register_tools(self, ctx: Any) -> None:
        # Guarded: the host injects ctx.tools only when the mcp.expose
        # capability is granted; tests pass a ctx without it.
        tools = getattr(ctx, "tools", None)
        if tools is None or not hasattr(tools, "register"):
            return
        tools.register("status", self._tool_status)
        tools.register("set_zoom", self._tool_set_zoom)
        tools.register("set_palette", self._tool_set_palette)
        tools.register("capture_photo", self._tool_capture_photo)
        tools.register("record", self._tool_record)
        tools.register("laser_range", self._tool_laser_range)
        tools.register("geolocate_target", self._tool_geolocate_target)

    async def _tool_status(self, _args: dict) -> dict:
        return self._state.to_dict()

    async def _tool_set_zoom(self, args: dict) -> dict:
        # The tool contract is {ok, reason}: a non-numeric argument must be a
        # refusal the caller can read, not an exception out of the handler.
        try:
            zoom = float(args.get("zoom", 1.0))
        except (TypeError, ValueError):
            return {"ok": False, "reason": "zoom must be a number"}
        if not math.isfinite(zoom) or zoom < 1.0:
            return {"ok": False, "reason": "zoom must be a finite value of at least 1"}
        await self._set_cfg("zoom", zoom)
        return {"ok": True, "zoom": zoom}

    async def _tool_set_palette(self, args: dict) -> dict:
        reason = f"palette must be an integer 0-{C.THERMAL_PALETTE_COUNT - 1}"
        try:
            palette = int(args.get("palette", 0))
        except (TypeError, ValueError):
            return {"ok": False, "reason": reason}
        if not 0 <= palette < C.THERMAL_PALETTE_COUNT:
            return {"ok": False, "reason": reason}
        await self._set_cfg("palette", palette)
        return {"ok": True, "palette": palette}

    async def _tool_capture_photo(self, _args: dict) -> dict:
        await self._bump_nonce("photo_nonce")
        return {"ok": True}

    async def _tool_record(self, _args: dict) -> dict:
        await self._bump_nonce("record_nonce")
        return {"ok": True}

    async def _tool_laser_range(self, _args: dict) -> dict:
        pod = self._pod
        if pod is None:
            return {"ok": False, "reason": "pod not connected"}
        try:
            range_m = await self._measure_laser()
        except PodUnsupported as exc:
            return {"ok": False, "reason": str(exc)}
        return {"ok": True, "range_m": range_m}

    async def _tool_geolocate_target(self, _args: dict) -> dict:
        pod = self._pod
        if pod is None:
            return {"ok": False, "reason": "pod not connected"}
        try:
            await self._measure_laser()
        except PodUnsupported as exc:
            return {"ok": False, "reason": str(exc)}
        if self._last_laser_target is None:
            return {"ok": False, "reason": "no vehicle pose for geolocation"}
        return {"ok": True, **self._last_laser_target}
