"""Plugin lifecycle + config-driven control tests against a fake host context."""

from __future__ import annotations

from altnautica_siyi_pod.capability_profile import HW_A2_MINI, HW_ZT30
from altnautica_siyi_pod.plugin import SiyiPodPlugin
from altnautica_siyi_pod.transport import MockTransport


class _ConfigKV:
    def __init__(self) -> None:
        self._d: dict = {}

    def static(self, key, default):
        return default

    async def get(self, key, default=None):
        return self._d.get(key, default)

    def set(self, key, value) -> None:
        self._d[key] = value


class _Mavlink:
    def __init__(self) -> None:
        self.components: list = []
        self.subscriptions: dict = {}
        self.sent: list = []

    async def register_component(self, cid, kind):
        self.components.append((cid, kind))

    async def subscribe(self, msg, handler):
        self.subscriptions[msg] = handler

    async def send(self, frame, component_id=None):
        self.sent.append((frame, component_id))


class _Vision:
    def __init__(self) -> None:
        self.published: list = []

    async def publish_detection(self, batch):
        self.published.append(batch)
        return {}


class _Telemetry:
    def __init__(self) -> None:
        self.extended: list = []

    async def extend(self, channel, payload):
        self.extended.append((channel, payload))


class _Events:
    def __init__(self) -> None:
        self.published: list = []

    async def publish(self, topic, payload):
        self.published.append((topic, payload))


class _Video:
    def __init__(self) -> None:
        self.sources: list = []

    async def set_source(self, cameras):
        self.sources.append(list(cameras))
        return {"ok": True, "count": len(list(cameras))}


class _Ctx:
    def __init__(self, *, with_video: bool = False) -> None:
        self.config_kv = _ConfigKV()
        self.mavlink = _Mavlink()
        self.vision = _Vision()
        self.telemetry = _Telemetry()
        self.events = _Events()
        if with_video:
            self.video = _Video()


def _zt30_factory(_cfg):
    return MockTransport(model=HW_ZT30, laser_range_m=42.0)


async def test_configure_video_advertises_two_legs_zt30():
    # A multi-sensor pod advertises exactly two concurrent legs (main + sub) on
    # real RTSP paths; there is no phantom /ir path. Main carries EO-zoom, sub
    # carries thermal by default. The cockpit switches / reassigns them.
    ctx = _Ctx(with_video=True)
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    legs = ctx.video.sources[-1]
    ids = [leg["id"] for leg in legs]
    assert ids == ["main", "sub"]
    assert legs[0]["source"] == "rtsp://192.168.144.25:8554/main"
    assert legs[0]["role"] == "eo"
    assert legs[1]["source"] == "rtsp://192.168.144.25:8554/sub"
    assert legs[1]["role"] == "ir"
    await plugin.on_stop(ctx)


async def test_start_assigns_main_eo_sub_ir_zt30():
    # On start the plugin routes distinct sensors: main = EO-zoom, sub = IR.
    from altnautica_siyi_pod import commands as C

    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport
    assert transport.image_sources == {
        C.STREAM_MAIN: C.IMG_SOURCE_EO_ZOOM,
        C.STREAM_SUB: C.IMG_SOURCE_IR,
    }
    await plugin.on_stop(ctx)


async def test_reassign_stream_source_to_wide():
    # The GCS reaches EO-wide by reassigning a leg's source; the plugin re-routes
    # the pod and re-advertises the leg with the new role.
    from altnautica_siyi_pod import commands as C

    ctx = _Ctx(with_video=True)
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport

    ctx.config_kv.set("stream_assignment", {"sub": "eo_wide"})
    await plugin.apply_config_once()
    assert transport.image_sources[C.STREAM_SUB] == C.IMG_SOURCE_EO_WIDE
    legs = ctx.video.sources[-1]
    sub = next(leg for leg in legs if leg["id"] == "sub")
    assert sub["role"] == "eo_wide"
    await plugin.on_stop(ctx)


async def test_reassign_stream_source_to_split_enables_composite():
    # Selecting the split source enables the pod's on-pod split/PiP composite and
    # advertises the leg with the split role.
    from altnautica_siyi_pod import commands as C

    ctx = _Ctx(with_video=True)
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport

    ctx.config_kv.set("stream_assignment", {"sub": "split"})
    await plugin.apply_config_once()
    assert transport.split_mode is True
    assert transport.image_sources[C.STREAM_SUB] == C.IMG_SOURCE_SPLIT
    legs = ctx.video.sources[-1]
    sub = next(leg for leg in legs if leg["id"] == "sub")
    assert sub["role"] == "split"
    await plugin.on_stop(ctx)


async def test_configure_video_single_leg_for_a_single_sensor_pod():
    ctx = _Ctx(with_video=True)
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: MockTransport(model=HW_A2_MINI)
    )
    await plugin.on_start(ctx)
    legs = ctx.video.sources[0]
    assert [leg["id"] for leg in legs] == ["main"]
    await plugin.on_stop(ctx)


async def test_start_negotiates_and_registers_components():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    assert plugin.pod is not None
    assert plugin.pod.profile.model == "ZT30"
    assert (154, "gimbal") in ctx.mavlink.components
    assert (100, "camera") in ctx.mavlink.components
    assert "ATTITUDE" in ctx.mavlink.subscriptions
    assert any(ch == "siyi" for ch, _ in ctx.telemetry.extended)
    await plugin.on_stop(ctx)


async def test_control_applies_state_and_fires_nonces_once():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport  # the MockTransport

    ctx.config_kv.set("zoom", 5.0)
    ctx.config_kv.set("palette", 3)
    ctx.config_kv.set("gimbal_mode", "lock")
    ctx.config_kv.set("photo_nonce", 1)
    await plugin.apply_config_once()
    assert transport.zoom == 5.0
    assert transport.palette == 3
    assert transport.gimbal_mode == "lock"
    assert transport.photos_taken == 1

    # A nonce fires exactly once until it increases again.
    await plugin.apply_config_once()
    assert transport.photos_taken == 1
    ctx.config_kv.set("photo_nonce", 2)
    await plugin.apply_config_once()
    assert transport.photos_taken == 2

    await plugin.on_stop(ctx)


async def test_laser_fire_publishes_a_geolocated_target():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    # Feed a valid FC pose so geolocation runs.
    plugin._on_global_position(
        {"lat": int(12.9716 * 1e7), "lon": int(77.5946 * 1e7), "relative_alt": 50000}
    )
    plugin._on_attitude({"yaw": 0.0})

    ctx.config_kv.set("laser_fire_nonce", 1)
    await plugin.apply_config_once()

    topics = [t for t, _ in ctx.events.published]
    assert "siyi.pod.laser_target" in topics
    assert plugin.state.laser_range_m == 42.0
    await plugin.on_stop(ctx)


async def test_a2_mini_unsupported_controls_are_ignored_not_raised():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: MockTransport(model=HW_A2_MINI)
    )
    await plugin.on_start(ctx)
    assert plugin.pod.profile.model == "A2 mini"
    # Zoom, thermal, and laser are absent on the A2 mini; the control loop must
    # skip them without raising.
    ctx.config_kv.set("zoom", 5.0)
    ctx.config_kv.set("palette", 2)
    ctx.config_kv.set("laser_fire_nonce", 1)
    await plugin.apply_config_once()  # must not raise
    await plugin.on_stop(ctx)
