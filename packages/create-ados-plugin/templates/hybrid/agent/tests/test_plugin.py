"""Unit test for the hello hybrid agent half.

Drives the plugin against a fake host context: a config store the GCS native
settings would write, an event bus the GCS tab subscribes to, and a no-op
logger. Asserts the loop reads the live config and publishes the read-back.
"""

from __future__ import annotations

import asyncio

import pytest

from plugin import STATE_TOPIC, HelloPlugin


class FakeConfigKv:
    def __init__(self, values: dict[str, object]) -> None:
        self._values = values

    async def get(self, key: str, default: object = None) -> object:
        return self._values.get(key, default)


class FakeEvents:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict[str, object]]] = []

    async def publish(self, topic: str, payload: dict[str, object]) -> None:
        self.published.append((topic, payload))


class FakeLog:
    def info(self, *_args: object, **_kwargs: object) -> None:
        return None


class FakeCtx:
    def __init__(self, config: dict[str, object]) -> None:
        self.config_kv = FakeConfigKv(config)
        self.events = FakeEvents()
        self.log = FakeLog()


@pytest.mark.asyncio
async def test_publishes_state_and_counts_ticks_while_active() -> None:
    ctx = FakeCtx({"active": True, "greeting": "namaste", "rate_hz": 50})
    plugin = HelloPlugin()
    await plugin.on_start(ctx)
    # Let the high-rate loop publish a few times, then stop it.
    await asyncio.sleep(0.06)
    await plugin.on_stop(ctx)

    assert ctx.events.published, "expected at least one read-back publish"
    topic, payload = ctx.events.published[-1]
    assert topic == STATE_TOPIC
    assert payload["active"] is True
    assert payload["greeting"] == "namaste"
    assert isinstance(payload["ticks"], int) and payload["ticks"] >= 1


@pytest.mark.asyncio
async def test_does_not_count_ticks_while_inactive() -> None:
    ctx = FakeCtx({"active": False, "rate_hz": 50})
    plugin = HelloPlugin()
    await plugin.on_start(ctx)
    await asyncio.sleep(0.06)
    await plugin.on_stop(ctx)

    assert ctx.events.published
    _topic, payload = ctx.events.published[-1]
    assert payload["active"] is False
    assert payload["ticks"] == 0
