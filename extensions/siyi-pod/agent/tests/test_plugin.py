"""Plugin lifecycle + config-driven control tests against a fake host context."""

from __future__ import annotations

from altnautica_siyi_pod.capability_profile import HW_A2_MINI, HW_ZT30
from altnautica_siyi_pod.plugin import SiyiPodPlugin
from altnautica_siyi_pod.transport import MockTransport
from pymavlink.dialects.v20 import common as mavlink2


# The host delivers each subscribed message as {msg_name, frame, timestamp_ms}
# with ``frame`` the raw wire bytes; these build exactly that from real packed
# frames, so the handlers are exercised through the decoder they run on.
_FC = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)


def _delivery(msg):
    return {"msg_name": msg.get_type(), "frame": bytes(msg.pack(_FC)), "timestamp_ms": 0}


def _attitude(*, yaw):
    return _delivery(mavlink2.MAVLink_attitude_message(0, 0.0, 0.0, yaw, 0.0, 0.0, 0.0))


def _global_position(lat_deg, lon_deg, rel_alt_m):
    return _delivery(
        mavlink2.MAVLink_global_position_int_message(
            0, int(round(lat_deg * 1e7)), int(round(lon_deg * 1e7)), 0,
            int(round(rel_alt_m * 1000)), 0, 0, 0, 0,
        )
    )


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


class _Tools:
    def __init__(self) -> None:
        self.handlers: dict = {}

    def register(self, name, handler) -> None:
        self.handlers[name] = handler


class _Ctx:
    def __init__(self, *, with_video: bool = False, with_tools: bool = False) -> None:
        self.config_kv = _ConfigKV()
        self.mavlink = _Mavlink()
        self.vision = _Vision()
        self.telemetry = _Telemetry()
        self.events = _Events()
        if with_video:
            self.video = _Video()
        if with_tools:
            self.tools = _Tools()


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


async def test_video_source_apply_failure_is_surfaced(caplog):
    # The host reports ok=False when the pipeline restart failed (config saved,
    # streams not live). The plugin must warn, not swallow it: a failed apply
    # has to be visible.
    import logging

    class _FailVideo:
        async def set_source(self, cameras):
            return {
                "ok": False,
                "count": len(list(cameras)),
                "persisted": True,
                "restarted": False,
            }

    ctx = _Ctx()
    ctx.video = _FailVideo()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    with caplog.at_level(logging.WARNING):
        await plugin.on_start(ctx)
    assert "video source apply did not go live" in caplog.text
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
    plugin._on_global_position(_global_position(12.9716, 77.5946, 50.0))
    plugin._on_attitude(_attitude(yaw=0.0))

    ctx.config_kv.set("laser_fire_nonce", 1)
    await plugin.apply_config_once()

    topics = [t for t, _ in ctx.events.published]
    assert plugin._last_laser_target is not None
    assert plugin.state.laser_range_m == 42.0
    await plugin.on_stop(ctx)


async def test_start_survives_unreachable_pod_and_recovers():
    # The pod is unreachable at boot: on_start must not raise, and the model +
    # video legs must come up once the pod answers.
    ctx = _Ctx(with_video=True)
    transport = MockTransport(model=HW_ZT30, answer_identity=False)
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: transport, session_timeout_s=0.02
    )
    await plugin.on_start(ctx)  # must not raise on an unreachable pod
    assert plugin.pod.negotiated is False
    assert plugin.state.connected is False

    # The pod appears; the control loop re-negotiates and brings it online.
    transport.answer_identity = True
    await plugin.apply_config_once()
    assert plugin.pod.negotiated is True
    assert plugin.pod.profile.model == "ZT30"
    assert plugin.state.connected is True
    legs = ctx.video.sources[-1]
    assert [leg["id"] for leg in legs] == ["main", "sub"]
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


async def test_skill_photo_center_and_nadir():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport

    ctx.config_kv.set("photo", True)
    await plugin.apply_config_once()
    assert transport.photos_taken == 1
    # The one-shot key is cleared so a re-press fires again.
    assert await ctx.config_kv.get("photo") is False

    # Nadir points straight down (the model's lower pitch limit); center then
    # returns the gimbal to zero.
    ctx.config_kv.set("nadir", True)
    await plugin.apply_config_once()
    assert transport.pitch_deg == plugin.pod.profile.pitch_min_deg
    ctx.config_kv.set("center", True)
    await plugin.apply_config_once()
    assert transport.pitch_deg == 0.0
    await plugin.on_stop(ctx)


async def test_skill_zoom_step_and_palette_cycle():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport

    # Zoom in steps the absolute zoom up and writes it back so the picker
    # reflects it; zoom out steps it back, clamped at 1.0.
    ctx.config_kv.set("zoom_in", True)
    await plugin.apply_config_once()
    assert transport.zoom == 2.0
    assert await ctx.config_kv.get("zoom") == 2.0
    ctx.config_kv.set("zoom_out", True)
    await plugin.apply_config_once()
    assert transport.zoom == 1.0

    # Cycle palette advances the thermal palette and writes it back.
    ctx.config_kv.set("palette_cycle", True)
    await plugin.apply_config_once()
    assert transport.palette == 1
    assert await ctx.config_kv.get("palette") == 1
    await plugin.on_stop(ctx)


async def test_skill_laser_fire_measures_range():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    plugin._on_global_position(_global_position(12.9716, 77.5946, 50.0))
    plugin._on_attitude(_attitude(yaw=0.0))

    ctx.config_kv.set("laser_fire", True)
    await plugin.apply_config_once()
    assert plugin.state.laser_range_m == 42.0
    assert plugin._last_laser_target is not None
    await plugin.on_stop(ctx)


async def test_a2_mini_skills_are_safe_noops():
    ctx = _Ctx()
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: MockTransport(model=HW_A2_MINI)
    )
    await plugin.on_start(ctx)
    transport = plugin._session._transport
    # Zoom / thermal / laser are absent on the A2 mini; the Skills must be safe
    # no-ops, not raises.
    for key in ("zoom_in", "palette_cycle", "laser_fire"):
        ctx.config_kv.set(key, True)
    await plugin.apply_config_once()
    assert transport.zoom == 1.0  # zoom-in was a no-op
    await plugin.on_stop(ctx)


async def test_skill_record_toggles_recording_via_the_pod():
    # The record Skill's config key must fire the real recording path: the pod's
    # record-toggle command flips its recording state, and the read-back tracks
    # it so the Skill Bar shows the true on/off.
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport
    assert transport.recording is False

    ctx.config_kv.set("recording", True)
    await plugin.apply_config_once()
    assert transport.recording is True
    assert plugin.state.recording is True

    # Idempotent: re-applying the same desired state does not toggle it back.
    await plugin.apply_config_once()
    assert transport.recording is True
    assert plugin.state.recording is True

    ctx.config_kv.set("recording", False)
    await plugin.apply_config_once()
    assert transport.recording is False
    assert plugin.state.recording is False
    await plugin.on_stop(ctx)


async def test_a2_mini_publishes_disabled_skill_states():
    # An A2 mini has no zoom / thermal / laser / tracking / gimbal, so those
    # Skills publish a disabled state instead of offering a silent
    # no-op; photo (a base camera feature) stays available.
    ctx = _Ctx()
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: MockTransport(model=HW_A2_MINI)
    )
    await plugin.on_start(ctx)
    events = dict(ctx.events.published)  # latest payload per topic
    for topic in (
        "siyi.pod.zoom",
        "siyi.pod.palette",
        "siyi.pod.laser",
        "siyi.pod.gimbal",
    ):
        assert events[topic]["state"] == "disabled", (topic, events.get(topic))
        assert events[topic].get("reason")
    assert events["siyi.pod.photo"]["state"] == "idle"
    assert events["siyi.pod.record"]["state"] == "idle"
    await plugin.on_stop(ctx)


async def test_zt30_publishes_enabled_skill_states():
    # Every capability-gated Skill is available on the ZT30, so each publishes
    # an idle (enabled) state.
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    events = dict(ctx.events.published)
    for topic in (
        "siyi.pod.zoom",
        "siyi.pod.palette",
        "siyi.pod.laser",
        "siyi.pod.gimbal",
        "siyi.pod.photo",
        "siyi.pod.record",
    ):
        assert events[topic]["state"] == "idle", (topic, events.get(topic))
    await plugin.on_stop(ctx)


async def test_mcp_tools_registered_and_callable():
    ctx = _Ctx(with_tools=True)
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport
    assert set(ctx.tools.handlers) == {
        "status",
        "set_zoom",
        "set_palette",
        "capture_photo",
        "record",
        "laser_range",
        "geolocate_target",
    }

    status = await ctx.tools.handlers["status"]({})
    assert status["model"] == "ZT30"

    # set_zoom / set_palette write config the control loop applies.
    await ctx.tools.handlers["set_zoom"]({"zoom": 5.0})
    await ctx.tools.handlers["set_palette"]({"palette": 3})
    await plugin.apply_config_once()
    assert transport.zoom == 5.0
    assert transport.palette == 3

    # capture_photo / record bump the same nonces the panel writes.
    await ctx.tools.handlers["capture_photo"]({})
    await ctx.tools.handlers["record"]({})
    await plugin.apply_config_once()
    assert transport.photos_taken == 1
    assert transport.recording is True

    # laser_range returns the measured range.
    lr = await ctx.tools.handlers["laser_range"]({})
    assert lr == {"ok": True, "range_m": 42.0}

    # geolocate_target needs a vehicle pose; feed one, then it returns a fix.
    plugin._on_global_position(_global_position(12.9716, 77.5946, 50.0))
    plugin._on_attitude(_attitude(yaw=0.0))
    geo = await ctx.tools.handlers["geolocate_target"]({})
    assert geo["ok"] is True
    assert "lat_deg" in geo and "lon_deg" in geo
    await plugin.on_stop(ctx)


async def test_unidentified_pod_is_never_commanded():
    # A device that never answers the identity query is not known to be a SIYI
    # pod at all, so nothing may be sent to it. Every declarative key, nonce and
    # one-shot action is armed here; a control tick must still produce no write
    # beyond the identity/firmware queries the negotiation retry itself sends.
    ctx = _Ctx(with_video=True)
    transport = MockTransport(model=HW_ZT30, answer_identity=False)
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: transport, session_timeout_s=0.02
    )
    await plugin.on_start(ctx)
    assert plugin.pod.negotiated is False

    ctx.config_kv.set("zoom", 5.0)
    ctx.config_kv.set("gimbal_mode", "lock")
    ctx.config_kv.set("palette", 3)
    ctx.config_kv.set("thermal_gain", True)
    ctx.config_kv.set("recording", True)
    ctx.config_kv.set("photo_nonce", 7)
    ctx.config_kv.set("laser_fire_nonce", 4)
    ctx.config_kv.set("center", True)
    ctx.config_kv.set("nadir", True)
    ctx.config_kv.set("zoom_in", True)

    await plugin.apply_config_once()

    # The mock carries the pod's own power-on defaults; none of them moved.
    assert transport.zoom == 1.0
    assert transport.palette == 0
    assert transport.gimbal_mode == "follow"
    assert transport.photos_taken == 0
    assert transport.recording is False
    assert plugin.state.connected is False
    # No stream leg is advertised for a device that has not said it serves one.
    assert ctx.video.sources == []
    await plugin.on_stop(ctx)


async def test_stale_nonce_does_not_refire_when_the_pod_comes_online():
    # A one-shot nonce left in config by an earlier session is history, not a
    # fresh operator press: it must not fire a photo or the rangefinder the
    # moment the pod identifies itself.
    ctx = _Ctx()
    transport = MockTransport(model=HW_ZT30, answer_identity=False)
    plugin = SiyiPodPlugin(
        transport_factory=lambda _cfg: transport, session_timeout_s=0.02
    )
    ctx.config_kv.set("photo_nonce", 9)
    await plugin.on_start(ctx)
    assert plugin.pod.negotiated is False

    transport.answer_identity = True
    await plugin.apply_config_once()
    assert plugin.pod.negotiated is True
    assert transport.photos_taken == 0

    # A genuinely new press does fire.
    ctx.config_kv.set("photo_nonce", 10)
    await plugin.apply_config_once()
    assert transport.photos_taken == 1
    await plugin.on_stop(ctx)


async def test_a_position_that_does_not_decode_leaves_geolocation_off():
    # A GLOBAL_POSITION_INT delivery the decoder cannot read must not mark the
    # vehicle pose ready: geolocating from an unset pose would put the subject
    # marker somewhere the vehicle never was.
    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    broken = _global_position(12.9716, 77.5946, 50.0)
    broken["frame"] = broken["frame"][:-4]
    plugin._on_global_position(broken)
    plugin._on_attitude(_attitude(yaw=0.0))

    ctx.config_kv.set("laser_fire_nonce", 1)
    await plugin.apply_config_once()

    assert plugin._last_laser_target is None
    await plugin.on_stop(ctx)
