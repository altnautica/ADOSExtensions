"""Plugin entry point.

The agent runner instantiates ``SiyiPodPlugin`` with no arguments and drives
``on_start`` / ``on_stop``. On start the plugin opens the transport, negotiates
the pod's capability profile, registers the MAVLink gimbal + camera components,
auto-configures the pod's video source (when the host exposes ``ctx.video``),
and runs two loops:

* a control loop that reads the per-drone config keys the GCS half writes and
  applies them to the pod — declarative state keys (zoom, sensor mode, gimbal
  mode, palette, thermal gain, laser arm, track arm) plus one-shot command-nonce
  keys (photo, record, recenter, fire laser, designate) fired once per new
  nonce, every call gated on the negotiated capability profile;
* a telemetry loop that reads attitude / zoom / range from the pod and publishes
  the ``siyi.pod.state`` read-back the GCS renders, mirroring gimbal attitude and
  any laser range up to the flight controller.

The pod owns its AI tracker; the plugin republishes its box onto the shared
detection bus so the cockpit click-to-track and the locked-target gate work.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable

from altnautica_siyi_pod.geolocation import geolocate
from altnautica_siyi_pod.mavlink_bridge import (
    COMP_CAMERA,
    COMP_GIMBAL,
    SiyiMavlinkBridge,
)
from altnautica_siyi_pod.pod import PodUnsupported, SiyiPod
from altnautica_siyi_pod.session import SiyiSession
from altnautica_siyi_pod.state import PodState
from altnautica_siyi_pod.tracker_bridge import SiyiTrackerBridge
from altnautica_siyi_pod.transport import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    TcpTransport,
    UartTransport,
    UdpTransport,
)

log = logging.getLogger(__name__)

_CONTROL_HZ = 5.0
_TELEMETRY_HZ = 5.0

# Declarative state keys the control loop diffs each tick.
_STATE_KEYS = (
    "zoom",
    "gimbal_mode",
    "palette",
    "thermal_gain",
    "track_active",
    "recording",
    "stream_assignment",
)

# Each capability-gated Skill's state-read-back topic mapped to the capability
# it needs. A Skill for a control the negotiated model lacks publishes a
# disabled state so the cockpit Skill Bar greys it out (Rule 44) instead of
# offering a silent no-op; a supported Skill publishes idle. Photo is a base
# camera feature every model has, so it has no capability gate.
_SKILL_TOPIC_CAP: dict[str, str | None] = {
    "siyi.pod.point_at": "ai_track",
    "siyi.pod.palette": "thermal",
    "siyi.pod.laser": "laser",
    "siyi.pod.zoom": "zoom",
    "siyi.pod.center": "gimbal",
    "siyi.pod.nadir": "gimbal",
    "siyi.pod.photo": None,
}

# The reason string the Skill Bar shows on a greyed-out control.
_CAP_UNAVAILABLE_REASON = {
    "ai_track": "Tracking is unavailable on this pod model",
    "thermal": "Thermal is unavailable on this pod model",
    "laser": "The rangefinder is unavailable on this pod model",
    "zoom": "Zoom is unavailable on this pod model",
    "gimbal": "The gimbal is fixed on this pod model",
}

# SIYI RTSP stream layout: the pod serves exactly two concurrent streams,
# `main` and `sub`, each on its own :8554 path and each assignable to a sensor
# (EO-zoom / EO-wide / thermal-IR) or the on-pod split/PiP composite. A
# multi-sensor pod (ZT6 / ZT30) advertises both legs with distinct sources
# (main = EO-zoom, sub = IR by default); a single-sensor pod advertises only
# `main`. The GCS reaches EO-wide / split by reassigning a leg's source.
#
# The advertised leg `role` uses the GCS label-map vocabulary (eo / eo_wide /
# ir / split); the assignable source vocabulary the pod facade + capability
# profile speak is eo_zoom / eo_wide / ir / split.
_ROLE_FOR_SOURCE = {
    "eo_zoom": "eo",
    "eo_wide": "eo_wide",
    "ir": "ir",
    "split": "split",
}
# One-shot keys: an action fires once each time its integer nonce increases.
_NONCE_KEYS = (
    "photo_nonce",
    "record_nonce",
    "recenter_nonce",
    "laser_fire_nonce",
    "track_designate_nonce",
)

# One-shot boolean action keys the cockpit Skills flip to true. The control
# loop fires the matching command on the rising edge and clears the key, so a
# re-press fires again (the Skill Bar writes the flag true each press with no
# nonce). Each command is gated on the negotiated capability profile, so a
# Skill for a control the model lacks (zoom on an A2 mini) is a safe no-op.
_ACTION_KEYS = (
    "point_at",
    "palette_cycle",
    "laser_fire",
    "center",
    "nadir",
    "zoom_in",
    "zoom_out",
    "photo",
)

# Per-press zoom step (absolute-zoom units) and the number of thermal palettes
# the cycle-palette Skill rotates through.
_ZOOM_STEP = 1.0
_THERMAL_PALETTE_COUNT = 8


class _Pose:
    """Latest flight-controller pose, for laser geolocation."""

    __slots__ = ("yaw_deg", "lat_deg", "lon_deg", "rel_alt_m", "ready")

    def __init__(self) -> None:
        self.yaw_deg = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        self.ready = False


class SiyiPodPlugin:
    """Lifecycle-hook plugin the runner instantiates with no args."""

    def __init__(
        self,
        transport_factory: Callable[[dict], Any] | None = None,
        *,
        session_timeout_s: float | None = None,
    ) -> None:
        # A test injects a factory returning a MockTransport; production uses the
        # config-selected real transport (the thermal-extension mock-seam pattern).
        self._transport_factory = transport_factory
        # A test seam to shorten the session command timeout (production uses the
        # session default); keeps the unreachable-pod retry test fast.
        self._session_timeout_s = session_timeout_s
        self._ctx: Any = None
        self._session: SiyiSession | None = None
        self._pod: SiyiPod | None = None
        self._bridge: SiyiMavlinkBridge | None = None
        self._tracker: SiyiTrackerBridge | None = None
        self._pose = _Pose()
        self._state = PodState()
        self._applied: dict[str, Any] = {}
        self._nonces: dict[str, int] = {}
        # Last per-skill state published on each read-back topic, so the Skill
        # Bar states are republished only on change (enabled/disabled/active).
        self._last_skill_states: dict[str, dict] = {}
        # Which sensor each physical leg (main/sub) currently carries.
        self._assignment: dict[str, str] = {}
        # The last geolocated laser target, returned by the geolocate tool.
        self._last_laser_target: dict[str, Any] | None = None
        # Whether the pod is currently reporting an AI-track box (so a drop
        # publishes one "lost" batch rather than an empty batch every tick).
        self._track_present = False
        self._host = DEFAULT_HOST
        self._control_task: asyncio.Task | None = None
        self._telemetry_task: asyncio.Task | None = None

    # -- lifecycle --------------------------------------------------------
    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        system_id = int(await self._cfg("system_id", 1))
        self._host = str(await self._cfg("host", DEFAULT_HOST))

        transport = self._build_transport(
            {
                "transport": str(await self._cfg("transport", "udp")),
                "host": self._host,
                "port": int(await self._cfg("port", DEFAULT_PORT)),
                "serial_port": str(await self._cfg("serial_port", "/dev/ttyUSB0")),
            }
        )
        if self._session_timeout_s is not None:
            self._session = SiyiSession(
                transport, timeout_s=self._session_timeout_s, retries=0
            )
        else:
            self._session = SiyiSession(transport)
        await self._session.start()

        self._pod = SiyiPod(self._session)
        # Never hard-raise if the pod is unreachable at boot: negotiate resolves
        # the fallback profile and the control loop re-negotiates until the pod
        # answers (the gimbal, telemetry, and video come up once it appears).
        profile = await self._pod.negotiate()

        self._bridge = SiyiMavlinkBridge(ctx, system_id=system_id)
        # The pod tracks on its primary leg; stamp its republished box with that
        # advertised leg id so the cockpit overlay (which keys detection boxes by
        # cameraId to the shown leg) actually renders it.
        self._tracker = SiyiTrackerBridge(ctx, camera_id=self._primary_leg())

        # Register the pod's MAVLink components so the standard gimbal/camera
        # surfaces light up (interop bonus; the plugin's own GCS half is the
        # primary control path).
        await self._safe(ctx.mavlink.register_component(COMP_GIMBAL, "gimbal"))
        await self._safe(ctx.mavlink.register_component(COMP_CAMERA, "camera"))

        # FC pose for laser geolocation.
        await self._safe(ctx.mavlink.subscribe("ATTITUDE", self._on_attitude))
        await self._safe(
            ctx.mavlink.subscribe("GLOBAL_POSITION_INT", self._on_global_position)
        )

        # Route the default sensor assignment to the pod's streams, advertise the
        # legs to the video pipeline, and publish the read-back.
        await self._post_negotiation_setup()

        # Register the MCP tools (guarded: the host injects ctx.tools only when
        # the mcp.expose capability is granted; tests pass a ctx without it).
        self._register_tools(ctx)

        self._control_task = asyncio.create_task(self._control_loop())
        self._telemetry_task = asyncio.create_task(self._telemetry_loop())
        log.info("siyi pod started: model=%s", profile.model)

    async def _post_negotiation_setup(self) -> None:
        """Apply the default source assignment, advertise the pod's stream legs,
        and refresh the published state.

        Idempotent and re-runnable so the video legs + source routing re-resolve
        once a (re)negotiation lands the model.
        """
        await self._apply_stream_assignment(self._default_assignment())
        await self._configure_video(self._host)
        self._refresh_state()
        await self._publish_state()

    def _refresh_state(self) -> None:
        pod = self._pod
        if pod is None:
            return
        p = pod.profile
        self._state.model = p.model
        self._state.known = p.known
        self._state.connected = pod.negotiated
        self._state.firmware = pod.firmware
        self._state.capabilities = self._capabilities_dict()
        self._state.assignment = dict(self._assignment)
        self._state.link_ok = pod.negotiated

    async def _try_renegotiate(self) -> None:
        """Re-run negotiation while the pod is unresolved, and bring it online.

        On the attempt that resolves the model, the video legs, source
        assignment, and published state are set up — so a pod that was
        unreachable at boot comes fully online once it answers, with no plugin
        restart.
        """
        pod = self._pod
        if pod is None:
            return
        try:
            await pod.negotiate()
        except Exception:  # noqa: BLE001
            return
        if pod.negotiated:
            log.info("siyi pod negotiated on retry: model=%s", pod.profile.model)
            await self._post_negotiation_setup()

    async def on_stop(self, ctx: Any) -> None:
        for task in (self._control_task, self._telemetry_task):
            if task is not None:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._control_task = None
        self._telemetry_task = None
        if self._pod is not None:
            await self._safe(self._pod.center())
        if self._session is not None:
            await self._session.stop()
        self._session = None
        self._pod = None

    async def on_disable(self, ctx: Any) -> None:
        await self.on_stop(ctx)

    # -- transport selection ---------------------------------------------
    def _build_transport(self, cfg: dict) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory(cfg)
        kind = cfg["transport"]
        if kind == "tcp":
            return TcpTransport(cfg["host"], cfg["port"])
        if kind == "uart":
            return UartTransport(cfg["serial_port"])
        return UdpTransport(cfg["host"], cfg["port"])

    def _stream_layout(self) -> list[str]:
        """The pod's concurrent physical RTSP legs.

        A multi-sensor pod (ZT6 / ZT30) serves two concurrent streams —
        ``main`` and ``sub``; every other pod serves a single ``main`` leg.
        """
        profile = self._pod.profile if self._pod is not None else None
        if profile is not None and len(profile.sensors) >= 2:
            return ["main", "sub"]
        return ["main"]

    def _primary_leg(self) -> str:
        """The leg the pod tracks on — its primary EO stream (always ``main``)."""
        return self._stream_layout()[0]

    def _default_assignment(self) -> dict[str, str]:
        """Which sensor each physical leg carries by default.

        A multi-sensor pod shows two distinct sensors at once — the primary EO
        on ``main`` and thermal on ``sub`` (the observation default); a
        single-sensor pod shows its one EO stream on ``main``.
        """
        profile = self._pod.profile if self._pod is not None else None
        if profile is not None and len(profile.sensors) >= 2:
            secondary = "ir" if "ir" in profile.sensors else "eo_wide"
            return {"main": "eo_zoom", "sub": secondary}
        return {"main": "eo_zoom"}

    def _video_legs(self, host: str) -> list[dict]:
        """Build the pipeline's stream-source list from the current assignment.

        Exactly the pod's concurrent physical legs (``main`` [+ ``sub``]), each
        pointing at its own RTSP path on the pod, with the leg ``role`` set to
        whatever sensor that leg currently carries (Rule 44 — advertise only the
        streams the pod actually serves; no phantom ``/ir`` path).
        """
        legs: list[dict] = []
        for leg_id in self._stream_layout():
            source = self._assignment.get(leg_id, "eo_zoom")
            legs.append(
                {
                    "id": leg_id,
                    "source": f"rtsp://{host}:8554/{leg_id}",
                    "role": _ROLE_FOR_SOURCE.get(source, "eo"),
                    "codec": "h264",
                }
            )
        return legs

    async def _configure_video(self, host: str) -> None:
        video = getattr(self._ctx, "video", None)
        if video is None or not hasattr(video, "set_source"):
            return
        cameras = self._video_legs(host)
        try:
            reply = await video.set_source(cameras)
        except Exception as exc:  # noqa: BLE001
            log.warning("video source auto-config failed: %s", exc)
            return
        # The host reports ok=False when the config saved but the pipeline
        # restart failed (streams not live). Surface it — a failed apply must be
        # visible, not silently swallowed.
        if isinstance(reply, dict) and reply.get("ok") is not True:
            log.warning("video source apply did not go live: %s", reply)

    async def _apply_stream_assignment(self, assignment: dict[str, str]) -> None:
        """Route each physical leg to its assigned sensor on the pod.

        Only a multi-stream pod routes distinct sensors; a single-EO pod has one
        sensor and issues no assignment command. The pod's split/PiP composite is
        left enabled only while a leg is assigned the ``split`` source.
        """
        pod = self._pod
        self._assignment = dict(assignment)
        if pod is None or len(self._stream_layout()) < 2:
            return
        want_split = "split" in self._assignment.values()
        for leg_id, source in self._assignment.items():
            try:
                await pod.set_image_source(leg_id, source)
            except PodUnsupported as exc:
                log.info("ignoring unsupported source %s=%s: %s", leg_id, source, exc)
            except Exception as exc:  # noqa: BLE001
                log.warning("source assign %s=%s failed: %s", leg_id, source, exc)
        if not want_split and pod.profile.supports_pip:
            await self._safe(pod.set_split_mode(False))

    async def _reassign_streams(self, value: Any) -> None:
        """Apply a GCS-driven leg-source change (a ``{leg: source}`` delta)."""
        if not isinstance(value, dict):
            return
        layout = self._stream_layout()
        merged = dict(self._assignment)
        changed = False
        for leg, source in value.items():
            if leg in layout and isinstance(source, str) and source in _ROLE_FOR_SOURCE:
                merged[leg] = source
                changed = True
        if not changed:
            return
        await self._apply_stream_assignment(merged)
        await self._configure_video(self._host)
        self._refresh_state()
        await self._publish_state()

    # -- config -----------------------------------------------------------
    async def _cfg(self, key: str, default: Any) -> Any:
        static = self._ctx.config_kv.static(key, default)
        return await self._ctx.config_kv.get(key, static)

    def _capabilities_dict(self) -> dict[str, object]:
        p = self._pod.profile if self._pod is not None else None
        if p is None:
            return {}
        return {
            "gimbal": p.supports("gimbal"),
            "zoom": p.supports("zoom"),
            "optical_zoom": p.has_optical_zoom,
            "max_zoom": p.max_zoom,
            "thermal": p.supports("thermal"),
            "laser": p.supports("laser"),
            "ai_track": p.supports("ai_track"),
            "sensors": list(p.sensors),
            "streams": list(p.streams),
            "supports_pip": p.supports_pip,
            "yaw_min": p.yaw_min_deg,
            "yaw_max": p.yaw_max_deg,
            "pitch_min": p.pitch_min_deg,
            "pitch_max": p.pitch_max_deg,
        }

    # -- FC pose ----------------------------------------------------------
    def _on_attitude(self, msg: dict[str, Any]) -> None:
        import math

        self._pose.yaw_deg = math.degrees(float(msg.get("yaw", 0.0)))

    def _on_global_position(self, msg: dict[str, Any]) -> None:
        lat = msg.get("lat")
        lon = msg.get("lon")
        rel = msg.get("relative_alt")
        if lat is not None:
            self._pose.lat_deg = float(lat) / 1e7
        if lon is not None:
            self._pose.lon_deg = float(lon) / 1e7
        if rel is not None:
            self._pose.rel_alt_m = float(rel) / 1000.0
        self._pose.ready = True

    # -- control loop -----------------------------------------------------
    async def _control_loop(self) -> None:
        period = 1.0 / _CONTROL_HZ
        while True:
            await asyncio.sleep(period)
            try:
                await self.apply_config_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                log.warning("siyi control tick failed: %s", exc)

    async def apply_config_once(self) -> None:
        """Read the config keys and apply any change / fire any new nonce.

        Deterministic and idempotent so tests can drive it directly.
        """
        pod = self._pod
        if pod is None:
            return
        # Bring an unreachable-at-boot pod online once it answers.
        if not pod.negotiated:
            await self._try_renegotiate()
        # Declarative state keys.
        for key in _STATE_KEYS:
            value = await self._cfg(key, None)
            if value is None or self._applied.get(key) == value:
                continue
            self._applied[key] = value
            await self._apply_state(key, value)
        # One-shot nonce keys.
        for key in _NONCE_KEYS:
            nonce = await self._cfg(key, 0)
            try:
                nonce = int(nonce)
            except (TypeError, ValueError):
                continue
            if nonce > self._nonces.get(key, 0):
                self._nonces[key] = nonce
                await self._fire_nonce(key)
        # One-shot boolean action keys (the cockpit Skills): fire on the rising
        # edge and clear so a re-press fires again.
        for key in _ACTION_KEYS:
            if bool(await self._cfg(key, False)):
                await self._fire_action(key)
                await self._reset_key(key)

    async def _apply_state(self, key: str, value: Any) -> None:
        pod = self._pod
        assert pod is not None
        try:
            if key == "zoom":
                await pod.set_zoom(float(value))
            elif key == "gimbal_mode":
                await pod.set_mode(str(value))
            elif key == "palette":
                await pod.set_palette(int(value))
            elif key == "thermal_gain":
                await pod.set_gain(bool(value))
            elif key == "track_active":
                # The toggle's rising edge starts the pod tracker on the last
                # designated subject (or the frame centre); the falling edge
                # stops it. No silent no-op on either edge.
                if value:
                    await self._start_tracking()
                else:
                    await pod.ai_track_stop()
            elif key == "recording":
                # The record toggle drives the pod's start/stop through its
                # single toggle command: send it only when the desired state
                # differs from what the pod is doing, and track the result so
                # the Skill Bar reflects it.
                desired = bool(value)
                if desired != self._state.recording:
                    await pod.toggle_record()
                    self._state.recording = desired
            elif key == "stream_assignment":
                await self._reassign_streams(value)
        except PodUnsupported as exc:
            log.info("ignoring unsupported control %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("control %s failed: %s", key, exc)

    async def _fire_nonce(self, key: str) -> None:
        pod = self._pod
        assert pod is not None
        try:
            if key == "photo_nonce":
                await pod.take_photo()
            elif key == "record_nonce":
                await pod.toggle_record()
                # Keep the read-back in step with the toggle so the record
                # Skill reflects it whether it was fired by the tool or the
                # Skill Bar.
                self._state.recording = not self._state.recording
            elif key == "recenter_nonce":
                await pod.center()
            elif key == "laser_fire_nonce":
                await self._fire_laser()
            elif key == "track_designate_nonce":
                await self._designate_from_config()
        except PodUnsupported as exc:
            log.info("ignoring unsupported action %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("action %s failed: %s", key, exc)

    async def _fire_laser(self) -> None:
        await self._measure_laser()

    async def _measure_laser(self) -> float:
        """Fire the rangefinder, mirror the range to the flight controller, and
        (when a vehicle pose is available) resolve + publish the geolocated
        target. Returns the measured slant range. Shared by the laser-fire
        one-shot and the laser/geolocate tools."""
        pod = self._pod
        assert pod is not None
        range_m = await pod.read_laser_range()
        self._state.laser_range_m = range_m
        if self._bridge is not None:
            await self._bridge.send_distance(range_m)
        self._last_laser_target = None
        if self._pose.ready:
            att = self._state
            target = geolocate(
                vehicle_lat_deg=self._pose.lat_deg,
                vehicle_lon_deg=self._pose.lon_deg,
                vehicle_rel_alt_m=self._pose.rel_alt_m,
                vehicle_yaw_deg=self._pose.yaw_deg,
                gimbal_yaw_deg=att.yaw_deg or 0.0,
                gimbal_pitch_deg=att.pitch_deg or 0.0,
                slant_range_m=range_m,
            )
            self._last_laser_target = {
                "lat_deg": target.lat_deg,
                "lon_deg": target.lon_deg,
                "rel_alt_m": target.rel_alt_m,
                "slant_range_m": target.slant_range_m,
                "bearing_deg": target.bearing_deg,
            }
            await self._safe(
                self._ctx.events.publish(
                    "siyi.pod.laser_target", self._last_laser_target
                )
            )
        return range_m

    async def _designate_from_config(self) -> None:
        pod = self._pod
        assert pod is not None
        box = await self._cfg("track_designate", None)
        if not isinstance(box, dict):
            return
        await pod.ai_track_designate(
            int(box.get("x", 0)),
            int(box.get("y", 0)),
            int(box.get("width", 0)),
            int(box.get("height", 0)),
        )

    async def _start_tracking(self) -> None:
        """Start the pod's tracker for the track toggle's rising edge.

        Re-designates the last operator-designated box when there is one, else
        the frame centre, so arming the toggle actually locks the pod onto a
        subject (gated on ai_track: a model without it raises PodUnsupported,
        which the caller treats as a safe no-op)."""
        pod = self._pod
        assert pod is not None
        box = await self._cfg("track_designate", None)
        if not isinstance(box, dict):
            box = await self._center_box()
        await pod.ai_track_designate(
            int(box.get("x", 0)),
            int(box.get("y", 0)),
            int(box.get("width", 0)),
            int(box.get("height", 0)),
        )

    # -- one-shot Skill actions -------------------------------------------
    async def _fire_action(self, key: str) -> None:
        pod = self._pod
        if pod is None:
            return
        try:
            if key == "photo":
                await pod.take_photo()
            elif key == "center":
                await pod.center()
            elif key == "nadir":
                # Point straight down: recenter yaw, drive pitch to the model's
                # lower mechanical limit.
                await pod.set_attitude(0.0, pod.profile.pitch_min_deg)
            elif key == "laser_fire":
                await self._measure_laser()
            elif key == "point_at":
                await self._designate_center()
            elif key == "palette_cycle":
                await self._cycle_palette()
            elif key == "zoom_in":
                await self._step_zoom(_ZOOM_STEP)
            elif key == "zoom_out":
                await self._step_zoom(-_ZOOM_STEP)
        except PodUnsupported as exc:
            log.info("ignoring unsupported skill %s: %s", key, exc)
        except Exception as exc:  # noqa: BLE001
            log.warning("skill %s failed: %s", key, exc)

    async def _center_box(self) -> dict[str, int]:
        """A centred designation box in pod-frame pixels for the point-at Skill.
        The frame dimensions are configurable so a bench can match the pod's
        actual stream resolution; the box is a tenth of the frame."""
        fw = int(await self._cfg("track_frame_width", 1920))
        fh = int(await self._cfg("track_frame_height", 1080))
        bw = max(1, fw // 10)
        bh = max(1, fh // 10)
        return {"x": (fw - bw) // 2, "y": (fh - bh) // 2, "width": bw, "height": bh}

    async def _designate_center(self) -> None:
        pod = self._pod
        assert pod is not None
        box = await self._center_box()
        await pod.ai_track_designate(
            box["x"], box["y"], box["width"], box["height"]
        )

    async def _cycle_palette(self) -> None:
        pod = self._pod
        assert pod is not None
        current = self._applied.get("palette")
        if current is None:
            current = self._state.palette if self._state.palette is not None else 0
        nxt = (int(current) + 1) % _THERMAL_PALETTE_COUNT
        await pod.set_palette(nxt)
        self._applied["palette"] = nxt
        self._state.palette = nxt
        await self._set_cfg("palette", nxt)

    async def _step_zoom(self, delta: float) -> None:
        pod = self._pod
        assert pod is not None
        current = self._applied.get("zoom")
        if current is None:
            current = self._state.zoom if self._state.zoom is not None else 1.0
        nxt = max(1.0, min(pod.profile.max_zoom, float(current) + delta))
        await pod.set_zoom(nxt)
        self._applied["zoom"] = nxt
        self._state.zoom = nxt
        await self._set_cfg("zoom", nxt)

    async def _reset_key(self, key: str) -> None:
        await self._set_cfg(key, False)

    async def _set_cfg(self, key: str, value: Any) -> None:
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is None:
            return
        try:
            await setter(key, value)
        except Exception:  # noqa: BLE001
            log.debug("siyi config write failed: %s", key, exc_info=True)

    async def _bump_nonce(self, key: str) -> None:
        try:
            current = int(await self._cfg(key, 0) or 0)
        except (TypeError, ValueError):
            current = 0
        await self._set_cfg(key, current + 1)

    # -- MCP tools --------------------------------------------------------
    def _register_tools(self, ctx: Any) -> None:
        tools = getattr(ctx, "tools", None)
        if tools is None or not hasattr(tools, "register"):
            return
        tools.register("status", self._tool_status)
        tools.register("set_zoom", self._tool_set_zoom)
        tools.register("set_palette", self._tool_set_palette)
        tools.register("capture_photo", self._tool_capture_photo)
        tools.register("record", self._tool_record)
        tools.register("point_at", self._tool_point_at)
        tools.register("laser_range", self._tool_laser_range)
        tools.register("geolocate_target", self._tool_geolocate_target)

    async def _tool_status(self, _args: dict) -> dict:
        return self._state.to_dict()

    async def _tool_set_zoom(self, args: dict) -> dict:
        zoom = float(args.get("zoom", 1.0))
        await self._set_cfg("zoom", zoom)
        return {"ok": True, "zoom": zoom}

    async def _tool_set_palette(self, args: dict) -> dict:
        palette = int(args.get("palette", 0))
        await self._set_cfg("palette", palette)
        return {"ok": True, "palette": palette}

    async def _tool_capture_photo(self, _args: dict) -> dict:
        await self._bump_nonce("photo_nonce")
        return {"ok": True}

    async def _tool_record(self, _args: dict) -> dict:
        await self._bump_nonce("record_nonce")
        return {"ok": True}

    async def _tool_point_at(self, args: dict) -> dict:
        box = args.get("box")
        if not isinstance(box, dict):
            box = await self._center_box()
        await self._set_cfg("track_designate", box)
        await self._bump_nonce("track_designate_nonce")
        return {"ok": True, "box": box}

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

    # -- telemetry loop ---------------------------------------------------
    async def _telemetry_loop(self) -> None:
        period = 1.0 / _TELEMETRY_HZ
        while True:
            await asyncio.sleep(period)
            try:
                await self.poll_telemetry_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                log.warning("siyi telemetry tick failed: %s", exc)

    async def poll_telemetry_once(self) -> None:
        pod = self._pod
        session = self._session
        if pod is None or session is None:
            return
        try:
            att = await pod.read_attitude()
            self._state.yaw_deg = att.yaw_deg
            self._state.pitch_deg = att.pitch_deg
            self._state.roll_deg = att.roll_deg
            if self._bridge is not None:
                await self._bridge.send_attitude(
                    att.yaw_deg, att.pitch_deg, att.roll_deg
                )
        except Exception:  # noqa: BLE001
            pass
        await self.poll_track_once()
        self._state.frames_received = session.frames_received
        self._state.link_ok = True
        await self._publish_state()

    async def poll_track_once(self) -> None:
        """Read the pod's AI-track box and republish it onto the shared bus.

        The pod owns the track loop (it detects and self-slews with no companion
        NPU); this mirrors its current box onto ``vision.detection`` so the
        cockpit click-to-track overlay, the locked-target safety gate, and track
        geolocation all work. Publishes an empty batch once when a track drops,
        so consumers see the loss (never a silent stall).
        """
        pod = self._pod
        tracker = self._tracker
        if pod is None or tracker is None:
            return
        if not pod.negotiated or not pod.profile.supports("ai_track"):
            return
        try:
            box = await pod.read_track_box()
        except PodUnsupported:
            return
        except Exception:  # noqa: BLE001
            return
        if box is not None:
            self._track_present = True
            self._state.track_active = True
            self._state.track_id = box.track_id
            await tracker.publish_box(
                x=box.x,
                y=box.y,
                width=box.width,
                height=box.height,
                track_id=box.track_id,
                locked=box.locked,
            )
        elif self._track_present:
            self._track_present = False
            self._state.track_active = False
            self._state.track_id = None
            await tracker.publish_lost()

    async def _publish_state(self) -> None:
        payload = self._state.to_dict()
        # Rides the heartbeat under the "siyi" channel (GCS subscribes
        # telemetry.subscribe.siyi) and the event bus under siyi.pod.state (the
        # overlay + console read-back).
        await self._safe(self._ctx.telemetry.extend("siyi", payload))
        await self._safe(self._ctx.events.publish("siyi.pod.state", payload))
        await self._publish_skill_states()

    async def _publish_skill_states(self) -> None:
        """Publish each Skill's enabled / disabled / active state on its own
        read-back topic so the cockpit Skill Bar reflects the negotiated model.

        A capability-gated Skill the model lacks (zoom / thermal / laser / track
        / gimbal on an A2 mini) shows disabled with a reason instead of a silent
        no-op (Rule 44); the two toggles carry their live on/off state.
        Republished only on change so steady state is silent."""
        pod = self._pod
        if pod is None:
            return
        profile = pod.profile
        states: dict[str, dict] = {}
        for topic, cap in _SKILL_TOPIC_CAP.items():
            if cap is None or profile.supports(cap):
                states[topic] = {"state": "idle"}
            else:
                states[topic] = {
                    "state": "disabled",
                    "reason": _CAP_UNAVAILABLE_REASON.get(
                        cap, "Unavailable on this pod model"
                    ),
                }
        if profile.supports("ai_track"):
            states["siyi.pod.track"] = {
                "state": "active" if self._state.track_active else "idle"
            }
        else:
            states["siyi.pod.track"] = {
                "state": "disabled",
                "reason": _CAP_UNAVAILABLE_REASON["ai_track"],
            }
        states["siyi.pod.record"] = {
            "state": "active" if self._state.recording else "idle"
        }
        for topic, payload in states.items():
            if self._last_skill_states.get(topic) != payload:
                self._last_skill_states[topic] = payload
                await self._safe(self._ctx.events.publish(topic, payload))

    # -- helpers ----------------------------------------------------------
    async def _safe(self, awaitable) -> None:
        try:
            await awaitable
        except Exception:  # noqa: BLE001
            log.debug("siyi optional host call failed", exc_info=True)

    # -- accessors (tests) ------------------------------------------------
    @property
    def pod(self) -> SiyiPod | None:
        return self._pod

    @property
    def state(self) -> PodState:
        return self._state
