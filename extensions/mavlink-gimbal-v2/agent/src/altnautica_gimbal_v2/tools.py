"""The gimbal plugin's MCP tools (``status``, ``point_at``, ``recenter``).

A mixin of :class:`altnautica_gimbal_v2.plugin.GimbalV2Plugin`; it reads the
plugin's state and calls its ``_point_at``.
"""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable
from typing import Any


class GimbalToolsMixin:
    """MCP-tool half of the gimbal plugin."""

    _transport: str | None
    _session: Any
    _rate_mode: bool
    _last_aim_state: tuple[Any, ...] | None

    # Provided by the plugin class.
    _point_at: Callable[..., Awaitable[dict]]

    def _register_tools(self, ctx: Any) -> None:
        tools = getattr(ctx, "tools", None)
        if tools is None or not hasattr(tools, "register"):
            return
        tools.register("status", self._tool_status)
        tools.register("point_at", self._tool_point_at)
        tools.register("recenter", self._tool_recenter)

    async def _tool_status(self, _args: dict) -> dict:
        aim = self._last_aim_state
        return {
            "transport": self._transport,
            "connected": self._session is not None,
            "rate_mode": self._rate_mode,
            "aim": {
                "aim_active": aim[0] if aim else False,
                "commanding": aim[1] if aim else False,
                "lock_state": aim[2] if aim else None,
                "target_id": aim[3] if aim else None,
            },
        }

    async def _tool_point_at(self, args: dict) -> dict:
        # Every other tool here answers with the {ok, reason} contract, so a
        # non-numeric or non-finite argument must too: raising out of the
        # handler would show the caller a transport error instead of a usable
        # refusal, and a NaN angle means "do not change" on the wire.
        angles: dict[str, float] = {}
        for name, default in (("pitch_deg", 0.0), ("yaw_deg", 0.0)):
            try:
                value = float(args.get(name, default))
            except (TypeError, ValueError):
                return {"ok": False, "reason": f"{name} must be a number"}
            if not math.isfinite(value):
                return {"ok": False, "reason": f"{name} must be finite"}
            angles[name] = value
        return await self._point_at(angles["pitch_deg"], angles["yaw_deg"])

    async def _tool_recenter(self, _args: dict) -> dict:
        return await self._point_at(0.0, 0.0, recenter_model=True)
