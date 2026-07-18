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
    "stream_assignment",
)

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
        # Which sensor each physical leg (main/sub) currently carries.
        self._assignment: dict[str, str] = {}
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
                if not value:
                    await pod.ai_track_stop()
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
        pod = self._pod
        assert pod is not None
        range_m = await pod.read_laser_range()
        self._state.laser_range_m = range_m
        if self._bridge is not None:
            await self._bridge.send_distance(range_m)
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
            await self._safe(
                self._ctx.events.publish(
                    "siyi.pod.laser_target",
                    {
                        "lat_deg": target.lat_deg,
                        "lon_deg": target.lon_deg,
                        "rel_alt_m": target.rel_alt_m,
                        "slant_range_m": target.slant_range_m,
                        "bearing_deg": target.bearing_deg,
                    },
                )
            )

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
        # overlay + skill read-back).
        await self._safe(self._ctx.telemetry.extend("siyi", payload))
        await self._safe(self._ctx.events.publish("siyi.pod.state", payload))

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
