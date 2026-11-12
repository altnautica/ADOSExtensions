"""The laser rangefinder fires only while the operator has armed it
(``laser_enabled``), and every range is published with the time it was taken."""

from __future__ import annotations

import time

from altnautica_siyi_pod.plugin import SiyiPodPlugin
from test_plugin import _Ctx, _zt30_factory


async def _started(*, armed: bool):
    ctx = _Ctx(with_tools=True)
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    if armed:
        ctx.config_kv.set("laser_enabled", True)
    await plugin.apply_config_once()
    return ctx, plugin


async def test_disarmed_laser_does_not_fire_from_panel_or_skill():
    ctx, plugin = await _started(armed=False)
    ctx.config_kv.set("laser_fire_nonce", 1)
    ctx.config_kv.set("laser_fire", True)
    await plugin.apply_config_once()
    assert plugin.state.laser_range_m is None
    assert plugin.state.laser_range_at is None
    await plugin.on_stop(ctx)


async def test_disarmed_laser_tools_refuse_with_reason():
    ctx, plugin = await _started(armed=False)
    lr = await ctx.tools.handlers["laser_range"]({})
    assert lr["ok"] is False
    assert "armed" in lr["reason"]
    geo = await ctx.tools.handlers["geolocate_target"]({})
    assert geo["ok"] is False
    assert "armed" in geo["reason"]
    await plugin.on_stop(ctx)


async def test_disarmed_laser_skill_is_published_disabled():
    ctx, plugin = await _started(armed=False)
    await plugin.poll_telemetry_once()
    events = dict(ctx.events.published)
    assert events["siyi.pod.laser"]["state"] == "disabled"
    assert "settings" in events["siyi.pod.laser"]["reason"]
    assert events["siyi.pod.state"]["laser_enabled"] is False

    ctx.config_kv.set("laser_enabled", True)
    plugin._state_cfg = None  # force the next tick to re-read the config
    await plugin.apply_config_once()
    await plugin.poll_telemetry_once()
    events = dict(ctx.events.published)
    assert events["siyi.pod.laser"]["state"] == "idle"
    assert events["siyi.pod.state"]["laser_enabled"] is True
    await plugin.on_stop(ctx)


async def test_armed_range_is_published_with_its_time():
    ctx, plugin = await _started(armed=True)
    before = int(time.time() * 1000)
    lr = await ctx.tools.handlers["laser_range"]({})
    assert lr == {"ok": True, "range_m": 42.0}
    await plugin.poll_telemetry_once()
    state = dict(ctx.events.published)["siyi.pod.state"]
    assert state["laser_range_m"] == 42.0
    assert before <= state["laser_range_at"] <= time.time() * 1000
    await plugin.on_stop(ctx)
