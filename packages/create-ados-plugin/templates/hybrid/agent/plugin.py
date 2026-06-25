"""Hello hybrid agent plugin.

Runs as a subprocess under ados-supervisor. Demonstrates the contract the
GCS half pairs with:

- ``active`` is the per-node config key the cockpit Skill toggles. The loop
  reads it live each cycle, so arming/disarming takes effect with no restart.
- ``hello.state`` is the one-way read-back event the GCS tab subscribes to.
- ``greeting`` / ``rate_hz`` are the native settings the GCS renders from the
  manifest's ``contributes.parameters``; the agent reads the same keys.
- ``detector`` is the shared vision-detector selection (bound to
  ``engine.detector`` in the manifest); the contributed ``coco-person`` model
  is what the picker offers.

The plugin host injects a context object with ``config_kv`` (per-node config
reads), ``events`` (the host event bus), and ``log`` (structured logging).
See the SDK reference at ``docs.altnautica.com/developers/sdk-python``.
"""

from __future__ import annotations

import asyncio
from typing import Any

PLUGIN_ID = "__PLUGIN_ID__"

# The read-back event topic. Must equal the manifest skill `state.topic`
# and the GCS half's STATE_TOPIC.
STATE_TOPIC = "hello.state"

# Defaults mirror the manifest `contributes.parameters` defaults so the loop
# behaves the same before the operator changes anything.
_DEFAULT_RATE_HZ = 5.0
_DEFAULT_GREETING = "hello"


class HelloPlugin:
    """Lifecycle hooks the plugin host calls."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._task: asyncio.Task[None] | None = None
        self._ticks = 0

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        ctx.log.info("hello_plugin_started")
        self._task = asyncio.create_task(self._loop())

    async def on_stop(self, ctx: Any) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        ctx.log.info("hello_plugin_stopped")

    async def _read_config(self) -> dict[str, Any]:
        """Read the per-node config the GCS native settings write."""
        async def pick(key: str, default: Any) -> Any:
            try:
                value = await self._ctx.config_kv.get(key, None)
            except Exception:  # noqa: BLE001 - config read is best-effort
                value = None
            return default if value is None else value

        return {
            "active": bool(await pick("active", False)),
            "rate_hz": _as_float(await pick("rate_hz", _DEFAULT_RATE_HZ)),
            "greeting": str(await pick("greeting", _DEFAULT_GREETING)),
        }

    async def _loop(self) -> None:
        while True:
            cfg = await self._read_config()
            if cfg["active"]:
                self._ticks += 1
            await self._ctx.events.publish(
                STATE_TOPIC,
                {
                    "active": cfg["active"],
                    "greeting": cfg["greeting"],
                    "ticks": self._ticks,
                },
            )
            period = 1.0 / max(cfg["rate_hz"], 1.0)
            await asyncio.sleep(period)


def _as_float(value: Any, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback
