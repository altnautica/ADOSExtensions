"""Idle config IPC: the control tick reads only the one-shot keys; the aim and
rate-mode flags refresh once a second and never per detection batch."""

from __future__ import annotations

import pytest

from altnautica_gimbal_v2.plugin import GimbalV2Plugin

from .test_plugin import FakeConfigKv, FakeCtx, _locked_batch, _settle

ONE_SHOT = ["recenter", "nadir", "point", "roi", "roi_clear"]


class CountingConfigKv(FakeConfigKv):
    def __init__(self, live=None) -> None:
        super().__init__(live=live)
        self.gets: list[str] = []

    async def get(self, key, default=None):
        self.gets.append(key)
        return await super().get(key, default)


@pytest.mark.asyncio
async def test_tick_reads_one_shot_keys_and_refreshes_state_at_1hz() -> None:
    ctx = FakeCtx(live={"aim": False})
    ctx.config_kv = CountingConfigKv(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        per_tick = []
        for i in range(6):  # 5 Hz ticks over 1.2 s
            ctx.config_kv.gets.clear()
            await plugin.poll_control_once(now=1000.0 + i * 0.2)
            per_tick.append(list(ctx.config_kv.gets))

        assert per_tick[0] == ["aim", "rate_mode", *ONE_SHOT]
        assert per_tick[1:5] == [ONE_SHOT] * 4
        assert per_tick[5] == ["aim", "rate_mode", *ONE_SHOT]
    finally:
        await plugin.on_stop(ctx)


@pytest.mark.asyncio
async def test_detection_batch_reads_no_config_and_aim_follows_refresh() -> None:
    ctx = FakeCtx(live={"aim": False})
    ctx.config_kv = CountingConfigKv(live={"aim": False})
    plugin = GimbalV2Plugin()
    await plugin.on_start(ctx)
    try:
        await _settle()
        ctx.mavlink.sent.clear()
        ctx.config_kv.gets.clear()
        await ctx.vision.callback(_locked_batch("locked", track_id=7))
        await _settle()
        assert ctx.config_kv.gets == []
        assert ctx.mavlink.sent == []

        # The operator arms aim; the next state refresh picks it up.
        await ctx.config_kv.set("aim", True)
        await plugin.poll_control_once(now=5000.0)
        ctx.mavlink.sent.clear()
        await ctx.vision.callback(_locked_batch("locked", track_id=7))
        await _settle()
        assert len(ctx.mavlink.sent) == 1
    finally:
        await plugin.on_stop(ctx)
