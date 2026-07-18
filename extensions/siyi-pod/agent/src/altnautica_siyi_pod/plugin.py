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

from altnautica_siyi_pod import commands as C
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
)

# SIYI RTSP stream layout: the pod serves each active sensor on its own 8554
# path. Each entry is (leg id, role, rtsp path) keyed by the negotiated sensor
# name; a pod advertises exactly the legs for the sensors it has, so a single-EO
# pod publishes one leg and a three-sensor pod (zoom + wide + thermal) publishes
# three. The cockpit stream switcher then flips between them client-side, which
# is why the pod-side sensor mux is gone.
_SENSOR_STREAMS: dict[str, tuple[str, str, str]] = {
    "eo": ("main", "eo", "main"),
    "zoom": ("main", "eo", "main"),
    "wide": ("eo_wide", "eo_wide", "sub"),
    "thermal": ("ir", "ir", "ir"),
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

    def __init__(self, transport_factory: Callable[[dict], Any] | None = None) -> None:
        # A test injects a factory returning a MockTransport; production uses the
        # config-selected real transport (the thermal-extension mock-seam pattern).
        self._transport_factory = transport_factory
        self._ctx: Any = None
        self._session: SiyiSession | None = None
        self._pod: SiyiPod | None = None
        self._bridge: SiyiMavlinkBridge | None = None
        self._tracker: SiyiTrackerBridge | None = None
        self._pose = _Pose()
        self._state = PodState()
        self._applied: dict[str, Any] = {}
        self._nonces: dict[str, int] = {}
        self._control_task: asyncio.Task | None = None
        self._telemetry_task: asyncio.Task | None = None

    # -- lifecycle --------------------------------------------------------
    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        camera_id = str(await self._cfg("camera_id", "siyi-pod"))
        system_id = int(await self._cfg("system_id", 1))

        transport = self._build_transport(
            {
                "transport": str(await self._cfg("transport", "udp")),
                "host": str(await self._cfg("host", DEFAULT_HOST)),
                "port": int(await self._cfg("port", DEFAULT_PORT)),
                "serial_port": str(await self._cfg("serial_port", "/dev/ttyUSB0")),
            }
        )
        self._session = SiyiSession(transport)
        await self._session.start()

        self._pod = SiyiPod(self._session)
        profile = await self._pod.negotiate()

        self._bridge = SiyiMavlinkBridge(ctx, system_id=system_id)
        self._tracker = SiyiTrackerBridge(ctx, camera_id=camera_id)

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

        # Auto-configure the pod's video source when the host exposes the video
        # facade; otherwise the operator points the pipeline at the pod (the
        # plugin still works, it just does not self-configure the stream).
        await self._configure_video(await self._cfg("host", DEFAULT_HOST))

        self._state = PodState(
            model=profile.model,
            known=profile.known,
            connected=True,
            firmware=self._pod.firmware,
            capabilities=self._capabilities_dict(),
            link_ok=True,
        )
        await self._publish_state()

        self._control_task = asyncio.create_task(self._control_loop())
        self._telemetry_task = asyncio.create_task(self._telemetry_loop())
        log.info("siyi pod started: model=%s", profile.model)

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

    def _video_legs(self, host: str) -> list[dict]:
        """Build the pipeline's stream-source list from the negotiated sensors.

        One leg per sensor the pod actually has, each pointing at its own RTSP
        path on the pod (Rule 44 — advertise only the streams the pod serves).
        Falls back to a single ``main`` leg when the profile is unknown.
        """
        profile = self._pod.profile if self._pod is not None else None
        sensors = tuple(profile.sensors) if profile is not None else ("eo",)
        legs: list[dict] = []
        seen: set[str] = set()
        for sensor in sensors:
            entry = _SENSOR_STREAMS.get(sensor)
            if entry is None:
                continue
            leg_id, role, path = entry
            if leg_id in seen:
                continue
            seen.add(leg_id)
            legs.append(
                {
                    "id": leg_id,
                    "source": f"rtsp://{host}:8554/{path}",
                    "role": role,
                    "codec": "h264",
                }
            )
        if not legs:
            legs.append(
                {
                    "id": "main",
                    "source": f"rtsp://{host}:8554/main",
                    "role": "eo",
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
            await video.set_source(cameras)
        except Exception:  # noqa: BLE001
            log.info("video source auto-config unavailable; using operator config")

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
            "split_pip": p.supports_split_pip,
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
        self._state.frames_received = session.frames_received
        self._state.link_ok = True
        await self._publish_state()

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
