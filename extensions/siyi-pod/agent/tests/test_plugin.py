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
    # The live assignment is published so the console's per-leg selector reflects
    # it (main = EO-zoom, sub = IR).
    assert plugin.state.assignment == {"main": "eo_zoom", "sub": "ir"}
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
    assert plugin.state.assignment["sub"] == "eo_wide"
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


async def test_video_source_apply_failure_is_surfaced(caplog):
    # The host reports ok=False when the pipeline restart failed (config saved,
    # streams not live). The plugin must warn, not swallow it (Rule 44).
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
    # The tracker stamps its box with the primary advertised leg id (not the
    # old "siyi-pod" placeholder), so the overlay renders it on the shown leg.
    assert plugin._tracker.camera_id == "main"
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


async def test_pod_track_box_is_republished_to_vision():
    # The pod owns the track loop; the plugin mirrors its box onto the shared
    # detection bus, stamped with the primary advertised leg id so the cockpit
    # overlay renders it. A drop publishes one "lost" batch, not a silent stall.
    ctx = _Ctx()
    transport = MockTransport(model=HW_ZT30, track_box=(7, 100, 120, 40, 60, True))
    plugin = SiyiPodPlugin(transport_factory=lambda _cfg: transport)
    await plugin.on_start(ctx)

    await plugin.poll_track_once()
    assert len(ctx.vision.published) >= 1
    batch = ctx.vision.published[-1]
    assert batch.camera_id == "main"
    assert len(batch.detections) == 1
    det = batch.detections[0]
    assert det.track_id == 7
    assert det.lock_state == "locked"
    assert (det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height) == (100, 120, 40, 60)
    assert plugin.state.track_active is True
    assert plugin.state.track_id == 7

    # The track drops: exactly one empty batch, then nothing further.
    transport.track_box = None
    await plugin.poll_track_once()
    assert ctx.vision.published[-1].detections == []
    assert plugin.state.track_active is False
    published_after_lost = len(ctx.vision.published)
    await plugin.poll_track_once()
    assert len(ctx.vision.published) == published_after_lost  # no repeat empty batch
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
    plugin._on_global_position(
        {"lat": int(12.9716 * 1e7), "lon": int(77.5946 * 1e7), "relative_alt": 50000}
    )
    plugin._on_attitude({"yaw": 0.0})

    ctx.config_kv.set("laser_fire", True)
    await plugin.apply_config_once()
    assert plugin.state.laser_range_m == 42.0
    assert "siyi.pod.laser_target" in [t for t, _ in ctx.events.published]
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


async def test_skill_track_toggle_starts_then_stops_tracking():
    # The track toggle's rising edge designates a subject (starts the pod
    # tracker); the falling edge stops it. Neither edge is a silent no-op.
    from altnautica_siyi_pod import commands as C
    from altnautica_siyi_pod.framing import parse_frame

    ctx = _Ctx()
    plugin = SiyiPodPlugin(transport_factory=_zt30_factory)
    await plugin.on_start(ctx)
    transport = plugin._session._transport

    transport.sent.clear()
    ctx.config_kv.set("track_active", True)
    await plugin.apply_config_once()
    cmds = [parse_frame(f).cmd_id for f in transport.sent]
    assert C.CMD_AI_TRACK in cmds  # a designate went out (tracking started)

    transport.sent.clear()
    ctx.config_kv.set("track_active", False)
    await plugin.apply_config_once()
    cmds = [parse_frame(f).cmd_id for f in transport.sent]
    assert C.CMD_AI_TRACK in cmds  # a stop went out
    await plugin.on_stop(ctx)


async def test_a2_mini_publishes_disabled_skill_states():
    # An A2 mini has no zoom / thermal / laser / tracking / gimbal, so those
    # Skills publish a disabled state (Rule 44) instead of offering a silent
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
        "siyi.pod.point_at",
        "siyi.pod.center",
        "siyi.pod.nadir",
        "siyi.pod.track",
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
        "siyi.pod.point_at",
        "siyi.pod.center",
        "siyi.pod.nadir",
        "siyi.pod.photo",
        "siyi.pod.track",
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
        "point_at",
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

    # point_at writes a designate box + bumps the designate nonce.
    result = await ctx.tools.handlers["point_at"]({})
    assert result["ok"] is True
    assert isinstance(await ctx.config_kv.get("track_designate"), dict)

    # laser_range returns the measured range.
    lr = await ctx.tools.handlers["laser_range"]({})
    assert lr == {"ok": True, "range_m": 42.0}

    # geolocate_target needs a vehicle pose; feed one, then it returns a fix.
    plugin._on_global_position(
        {"lat": int(12.9716 * 1e7), "lon": int(77.5946 * 1e7), "relative_alt": 50000}
    )
    plugin._on_attitude({"yaw": 0.0})
    geo = await ctx.tools.handlers["geolocate_target"]({})
    assert geo["ok"] is True
    assert "lat_deg" in geo and "lon_deg" in geo
    await plugin.on_stop(ctx)
