"""Plugin entry point.

The agent runner instantiates ``SiyiPodPlugin`` with no arguments and drives
``on_start`` / ``on_stop``. On start the plugin opens the transport, negotiates
the pod's capability profile, registers the MAVLink gimbal + camera components,
auto-configures the pod's video source (when the host exposes ``ctx.video``),
and runs two loops:

* a control loop (``control.py``) that reads the per-drone config keys the GCS
  half writes and applies them to the pod, every call gated first on the pod
  having identified itself and then on the negotiated capability profile;
* a telemetry loop that reads attitude / zoom / range from the pod and publishes
  the ``siyi.pod.state`` read-back the GCS renders, mirroring gimbal attitude and
  any laser range up to the flight controller.

The video legs live in ``video.py`` and the MCP tools in ``tools.py``.

Until the pod answers the hardware-id query the plugin sends it nothing at all:
an unidentified device has no known mechanical range and no known feature set.
The session reopens a lost or never-opened transport on a fixed interval, and
the control loop retries the negotiation on a fixed interval, both with no
attempt cap, so a pod that is unpowered at boot or power-cycled in flight comes
online with no plugin restart, on any transport.
"""

from __future__ import annotations

import asyncio
import logging
import math
from typing import Any, Callable

from altnautica_siyi_pod.control import _NONCE_KEYS, ControlMixin
from altnautica_siyi_pod.mavlink_bridge import (
    COMP_CAMERA,
    COMP_GIMBAL,
    SiyiMavlinkBridge,
    decode_fields,
)
from altnautica_siyi_pod.pod import SiyiPod
from altnautica_siyi_pod.session import SiyiSession
from altnautica_siyi_pod.state import PodState
from altnautica_siyi_pod.tools import ToolsMixin
from altnautica_siyi_pod.transport import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    TcpTransport,
    UartTransport,
    UdpTransport,
)
from altnautica_siyi_pod.video import VideoMixin

log = logging.getLogger(__name__)

_TELEMETRY_HZ = 5.0


# Each capability-gated Skill's state-read-back topic mapped to the capability
# it needs. A Skill for a control the negotiated model lacks publishes a
# disabled state so the cockpit Skill Bar greys it out, instead of offering a
# silent no-op; a supported Skill publishes idle. Photo is a base camera
# feature every model has, so it has no capability gate. Center and nadir both
# gate on the gimbal and share one topic, as zoom in and out do: the agent keeps
# a small fixed number of state topics per plugin, and this set plus the state
# topic, the record toggle and the telemetry channel fills it exactly.
_SKILL_TOPIC_CAP: dict[str, str | None] = {
    "siyi.pod.palette": "thermal",
    "siyi.pod.laser": "laser",
    "siyi.pod.zoom": "zoom",
    "siyi.pod.gimbal": "gimbal",
    "siyi.pod.photo": None,
}

# The reason string the Skill Bar shows on a greyed-out control.
_CAP_UNAVAILABLE_REASON = {
    "thermal": "Thermal is unavailable on this pod model",
    "laser": "The rangefinder is unavailable on this pod model",
    "zoom": "Zoom is unavailable on this pod model",
    "gimbal": "The gimbal is fixed on this pod model",
}


class _Pose:
    """Latest flight-controller pose, for laser geolocation."""

    __slots__ = ("yaw_deg", "lat_deg", "lon_deg", "rel_alt_m", "ready")

    def __init__(self) -> None:
        self.yaw_deg = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        self.ready = False


class SiyiPodPlugin(ControlMixin, VideoMixin, ToolsMixin):
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
        self._pose = _Pose()
        self._state = PodState()
        self._applied: dict[str, Any] = {}
        self._nonces: dict[str, int] = {}
        # Last per-skill state published on each read-back topic, so the Skill
        # Bar states are republished only on change (enabled/disabled/active).
        self._last_skill_states: dict[str, dict] = {}
        # Which sensor each physical leg (main/sub) carries, from the resolved
        # model's factory stream layout.
        self._assignment: dict[str, str] = {}
        # The last geolocated laser target, returned by the geolocate tool.
        self._last_laser_target: dict[str, Any] | None = None
        self._host = DEFAULT_HOST
        # The declarative state keys, cached and re-read in full at 1 Hz by the
        # control loop (``ControlMixin._config_for_tick``).
        self._state_cfg: dict[str, Any] | None = None
        self._state_cfg_read_at = 0.0

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
        # A reopened link may front a power-cycled or swapped pod: re-identify
        # it before commanding it again.
        self._session.add_reconnect_listener(self._on_link_restored)
        # Never raises on an absent pod; the session keeps reopening the link.
        await self._session.start()

        self._pod = SiyiPod(self._session)
        # Never hard-raise if the pod is unreachable at boot: negotiate resolves
        # the fallback profile and the control loop re-negotiates until the pod
        # answers (the gimbal, telemetry, and video come up once it appears).
        profile = await self._pod.negotiate()

        self._bridge = SiyiMavlinkBridge(ctx, system_id=system_id)

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

        # Advertise the pod's stream legs and publish the read-back, but only
        # once it has identified itself; otherwise publish the unidentified
        # state and let the control loop keep retrying the negotiation.
        if self._pod.negotiated:
            await self._post_negotiation_setup()
        else:
            self._refresh_state()
            await self._publish_state()

        self._register_tools(ctx)

        self._control_task = asyncio.create_task(self._control_loop())
        self._telemetry_task = asyncio.create_task(self._telemetry_loop())
        log.info("siyi pod started: model=%s", profile.model)

    async def _post_negotiation_setup(self) -> None:
        """Advertise the identified pod's stream legs and refresh the read-back.

        Only called once the pod has identified itself: the leg layout comes
        from the resolved model, so advertising a stream before then would
        publish an RTSP path on a device that has not said it serves one.

        Idempotent and re-runnable so the video legs re-resolve once a
        (re)negotiation lands the model.
        """
        self._assignment = self._stream_sources()
        await self._configure_video(self._host)
        # Seed the one-shot nonce baseline from what is already stored, so a
        # nonce left in config by an earlier session does not re-fire a photo
        # or the rangefinder the moment the pod comes online.
        for key in _NONCE_KEYS:
            try:
                self._nonces[key] = int(await self._cfg(key, 0) or 0)
            except (TypeError, ValueError):
                self._nonces[key] = 0
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

    def _on_link_restored(self) -> None:
        """The transport came back after being lost.

        The pod behind it may have been power-cycled or swapped, so it must
        identify itself again before it is commanded: clearing ``negotiated``
        sends the control loop back through the negotiation retry. The applied
        state and the config cache are dropped so the operator's settings are
        re-applied to a pod that came back on its power-on defaults.
        """
        if self._pod is not None:
            self._pod.negotiated = False
        self._applied.clear()
        self._state_cfg = None
        self._refresh_state()

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
            "sensors": list(p.sensors),
            "yaw_min": p.yaw_min_deg,
            "yaw_max": p.yaw_max_deg,
            "pitch_min": p.pitch_min_deg,
            "pitch_max": p.pitch_max_deg,
        }

    # -- FC pose ----------------------------------------------------------
    def _on_attitude(self, delivery: dict[str, Any]) -> None:
        msg = decode_fields(delivery)
        if msg is None:
            return
        self._pose.yaw_deg = math.degrees(float(msg["yaw"]))

    def _on_global_position(self, delivery: dict[str, Any]) -> None:
        # lat/lon are 1e7 integer degrees; relative_alt is millimetres. The
        # pose is ready for geolocation only once a position actually decoded.
        msg = decode_fields(delivery)
        if msg is None:
            return
        self._pose.lat_deg = float(msg["lat"]) / 1e7
        self._pose.lon_deg = float(msg["lon"]) / 1e7
        self._pose.rel_alt_m = float(msg["relative_alt"]) / 1000.0
        self._pose.ready = True

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
        answered = False
        try:
            att = await pod.read_attitude()
            self._state.yaw_deg = att.yaw_deg
            self._state.pitch_deg = att.pitch_deg
            self._state.roll_deg = att.roll_deg
            answered = True
            if self._bridge is not None:
                await self._bridge.send_attitude(
                    att.yaw_deg, att.pitch_deg, att.roll_deg
                )
        except Exception:  # noqa: BLE001
            log.debug("siyi attitude read failed", exc_info=True)
        self._state.frames_received = session.frames_received
        # Link health is what this tick actually observed. Reporting it as up
        # regardless of whether the pod answered would make the GCS show a
        # healthy link to a pod that has gone silent, and the published
        # attitude would be the last value read before it did.
        self._state.link_ok = answered
        await self._publish_state()

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

        A capability-gated Skill the model lacks (zoom / thermal / laser /
        gimbal on an A2 mini) shows disabled with a reason rather than offering
        a silent no-op; the record toggle carries its live on/off state.
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
