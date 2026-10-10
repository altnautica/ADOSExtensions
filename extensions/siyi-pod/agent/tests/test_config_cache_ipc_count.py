"""Config IPC per control tick: one-shot keys every tick, state keys at 1 Hz."""

from __future__ import annotations

from altnautica_siyi_pod import control
from altnautica_siyi_pod.capability_profile import HW_ZT30
from altnautica_siyi_pod.control import _ACTION_KEYS, _NONCE_KEYS, _STATE_KEYS
from altnautica_siyi_pod.plugin import SiyiPodPlugin
from altnautica_siyi_pod.transport import MockTransport


class _CountingConfigKV:
    def __init__(self) -> None:
        self._d: dict = {}
        self.gets: list[str] = []

    def static(self, key, default):
        return default

    async def get(self, key, default=None):
        self.gets.append(key)
        return self._d.get(key, default)

    async def set(self, key, value) -> None:
        self._d[key] = value


class _Host:
    async def register_component(self, *_a):
        return None

    async def subscribe(self, *_a):
        return None

    async def send(self, *_a, **_k):
        return None

    async def extend(self, *_a):
        return None

    async def publish(self, *_a):
        return None


class _Ctx:
    def __init__(self) -> None:
        self.config_kv = _CountingConfigKV()
        self.mavlink = _Host()
        self.telemetry = _Host()
        self.events = _Host()


_ONE_SHOT = set(_NONCE_KEYS) | set(_ACTION_KEYS)


async def test_state_keys_refresh_at_most_once_a_second(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(control, "_monotonic", lambda: now[0])
    ctx = _Ctx()
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: MockTransport(model=HW_ZT30)
    )
    await plugin.on_start(ctx)
    # Drive the ticks by hand.
    plugin._control_task.cancel()

    per_tick: list[list[str]] = []
    for _ in range(5):  # one tick every 0.25 s
        ctx.config_kv.gets.clear()
        await plugin.apply_config_once()
        per_tick.append(list(ctx.config_kv.gets))
        now[0] += 0.25

    # The first tick reads the full key set.
    assert set(per_tick[0]) == set(_STATE_KEYS) | _ONE_SHOT
    # Ticks inside the refresh window poll only the one-shot keys, once each.
    for gets in per_tick[1:4]:
        assert sorted(gets) == sorted(_ONE_SHOT)
    # One second after the last refresh the state keys are read again.
    assert set(per_tick[4]) == set(_STATE_KEYS) | _ONE_SHOT
    assert len(per_tick[4]) == len(_STATE_KEYS) + len(_ONE_SHOT)
    await plugin.on_stop(ctx)


async def test_a_state_change_is_applied_on_the_next_refresh(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(control, "_monotonic", lambda: now[0])
    ctx = _Ctx()
    transport = MockTransport(model=HW_ZT30)
    plugin = SiyiPodPlugin(transport_factory=lambda _cfg: transport)
    await plugin.on_start(ctx)
    plugin._control_task.cancel()

    await plugin.apply_config_once()
    await ctx.config_kv.set("zoom", 6.0)
    now[0] += 0.25
    await plugin.apply_config_once()
    assert transport.zoom == 1.0  # served from cache inside the window
    now[0] += 0.75
    await plugin.apply_config_once()
    assert transport.zoom == 6.0
    await plugin.on_stop(ctx)
