"""Plugin entry point for the ADOS Thermal Camera FLIR Lepton USB UVC.

The agent half ships a single class, :class:`ThermalUsbPlugin`, that the host
instantiates in a subprocess hosting environment. The host calls
:meth:`on_start` after capability tokens are issued and :meth:`on_stop` when
the plugin is torn down.

The plugin owns its device: on start it constructs the injected UVC backend,
builds a :class:`LeptonUvcDriver`, opens the device, and runs a control loop
that reads the per-drone config keys the GCS half writes and applies them to
the Lepton (palette, high/low gain via the radiometric linear resolution,
one-shot flat field correction). It also auto-configures the agent's video
pipeline with the thermal stream leg via ``ctx.video.set_source`` when a
colorized stream endpoint is configured (hardware-gated: the Lepton feed is
per-pixel Y16 that must be colorized before it can be a stream leg, so a leg
is advertised only when an endpoint exists to serve it — see the extension
README).

A concrete UVC backend is not part of this package: the host injects one
through ``backend_factory``. The plugin fails closed whenever it has no open
radiometric session — no injected backend, no device on the bus, a failed
open, or a frame stream that will not start. In each case it logs the error,
publishes ``{"connected": false, "reason": ...}`` on the ``thermal`` state
channel, and starts neither the control loop nor the frame stream, so no
temperature is ever published without a device behind it.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable

from altnautica_thermal_camera.driver import LeptonUvcDriver
from altnautica_thermal_camera.palettes import list_palettes
from altnautica_thermal_camera.tlinear import (
    DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
    celsius_from_y16,
    celsius_grid_extrema,
)
from altnautica_thermal_camera.uvc_backend import LibUvcBackend

log = logging.getLogger(__name__)

# How often the control loop reads the config keys and applies changes.
_CONTROL_HZ = 5.0

# The telemetry channel the GCS overlay subscribes to for the live spot
# temperature. The colorized image itself rides the video pipeline (the
# thermal stream leg); this channel carries only the lightweight radiometric
# read-back the overlay draws over the video, so the overlay reads the video
# leg for the picture and this channel for temperatures. Nothing else is
# published here: the channel has exactly one payload shape, a frame
# read-back.
_FRAME_CHANNEL = "camera.thermal.frame"

# The state channel carrying the thermal read-back (connectivity, palette,
# gain). Connectivity lives here and never on the frame channel.
_STATE_CHANNEL = "thermal"

# Machine-readable reasons the plugin reports when it has no open session.
# The GCS half maps each to operator-facing text.
REASON_NO_BACKEND = "no-backend"
REASON_NO_DEVICE = "no-device"
REASON_OPEN_FAILED = "open-failed"
REASON_STREAM_FAILED = "stream-failed"
REASON_NOT_STARTED = "not-started"

# Skill-state topics for the two cockpit Skills the manifest contributes. The
# Skill Bar reads these to show whether a Skill can fire.
_SKILL_STATE_TOPICS = {
    "palette_cycle": "camera.thermal.palette",
    "ffc": "camera.thermal.ffc",
}

# The radiometric linear resolution (kelvin per count) each gain setting maps
# to: high gain trades range for temperature sensitivity, low gain the reverse.
_GAIN_TLINEAR = {True: 0.01, False: 0.1}

BackendFactory = Callable[[], LibUvcBackend]


def _frame_readout(frame: Any) -> dict[str, Any] | None:
    """Compute the centre-reticle spot temperature and frame extrema from a
    radiometric frame, as the lightweight overlay read-back.

    Returns ``None`` when the frame carries no usable Y16 grid."""
    import array
    import sys

    data = getattr(frame, "data", None)
    width = int(getattr(frame, "width", 0) or 0)
    height = int(getattr(frame, "height", 0) or 0)
    if data is None or width <= 0 or height <= 0:
        return None
    grid = array.array("H")
    try:
        grid.frombytes(bytes(data))
    except (ValueError, TypeError):
        return None
    if sys.byteorder == "big":
        grid.byteswap()
    if len(grid) < width * height:
        return None
    meta = getattr(frame, "metadata", None) or {}
    res = float(
        meta.get(
            "tlinear_resolution_k_per_count",
            DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
        )
    )
    # Centre reticle: the conventional fixed spot-meter position. The operator
    # aims the camera to place the subject under it.
    sx = width // 2
    sy = height // 2
    spot_c = celsius_from_y16(grid[sy * width + sx], res)
    min_c, max_c = celsius_grid_extrema(grid, res)
    return {
        "width": width,
        "height": height,
        "spot": {"x": sx, "y": sy, "temperatureC": round(spot_c, 2)},
        "minC": round(min_c, 2),
        "maxC": round(max_c, 2),
        "resolutionKPerCount": res,
    }


class ThermalUsbPlugin:
    """Entry point for the thermal camera plugin.

    ``backend_factory`` is the injection seam for a UVC backend satisfying
    :class:`LibUvcBackend`. It has no default: this package contains no
    concrete backend, and fabricating one would put invented temperatures on
    a live telemetry channel. With no factory the plugin has no sensor, so it
    publishes an explicit unavailable state and captures nothing.
    """

    plugin_id = "com.altnautica.thermal-flir-lepton-usb"
    version = "1.3.0"

    def __init__(self, backend_factory: BackendFactory | None = None) -> None:
        self._backend_factory: BackendFactory | None = backend_factory
        self._ctx: Any = None
        self._driver: LeptonUvcDriver | None = None
        self._session: Any = None
        # The open frame stream; the control loop pulls one frame per tick to
        # compute the spot temperature the overlay renders.
        self._frames: Any = None
        self._control_task: asyncio.Task | None = None
        # Declarative-key cache so a control tick only re-applies a changed key.
        self._applied: dict[str, Any] = {}
        # Last published thermal state, so the read-back is emitted on change.
        self._last_state: dict[str, Any] | None = None
        # Why there is no open session, reported with the unavailable state.
        self._reason: str = REASON_NOT_STARTED
        # Last published skill state per topic, so they are emitted on change.
        self._last_skill_states: dict[str, dict[str, Any]] = {}

    async def on_start(self, ctx: Any) -> None:
        """Open the device and start the control loop.

        Every path that leaves the plugin without an open radiometric session
        returns through :meth:`_fail_closed`, which reports the reason and
        leaves the control loop and the frame stream shut down. No reading
        reaches a telemetry channel unless a device is behind it.
        """
        self._ctx = ctx

        if self._backend_factory is None:
            await self._fail_closed(
                REASON_NO_BACKEND,
                "thermal-flir-lepton-usb has no UVC backend: the host injected "
                "no backend factory, so no thermal camera can be opened",
            )
            return
        self._driver = LeptonUvcDriver(self._backend_factory())

        try:
            candidates = await self._driver.discover()
        except Exception as exc:  # noqa: BLE001
            # A backend that cannot enumerate the bus has established no
            # device, which is indistinguishable from an empty bus here.
            await self._fail_closed(
                REASON_NO_DEVICE,
                f"thermal-flir-lepton-usb device enumeration failed: {exc}",
            )
            return
        if not candidates:
            await self._fail_closed(
                REASON_NO_DEVICE,
                "thermal-flir-lepton-usb found no device on the bus",
            )
            return

        initial_palette = str(await self._cfg("palette", "ironbow"))
        try:
            self._session = await self._driver.open(
                candidates[0], {"palette": initial_palette}
            )
        except Exception as exc:  # noqa: BLE001
            await self._fail_closed(
                REASON_OPEN_FAILED,
                f"thermal-flir-lepton-usb open failed: {exc}",
            )
            return
        self._applied["palette"] = initial_palette

        # Open the radiometric frame stream so the control loop can read a live
        # spot temperature off the Y16 grid and publish it for the overlay. A
        # session that cannot stream frames produces no temperature at all, so
        # it is released rather than left reporting a connected camera.
        try:
            self._frames = await self._driver.frame_iterator(self._session)
        except Exception as exc:  # noqa: BLE001
            await self._release_session()
            await self._fail_closed(
                REASON_STREAM_FAILED,
                f"thermal frame stream open failed: {exc}",
            )
            return

        # Advertise the thermal stream leg to the video pipeline when a
        # colorized stream endpoint is configured (hardware-gated: with no
        # endpoint to serve the colorized feed, no leg is advertised).
        await self._maybe_declare_source()

        self._reason = ""
        self._control_task = asyncio.create_task(self._control_loop())
        await self._publish_state()
        self._log(
            "thermal-flir-lepton-usb started",
            driver_id=self._driver.driver_id,
        )

    async def _fail_closed(self, reason: str, message: str) -> None:
        """Report an unavailable thermal camera and capture nothing.

        The caller returns immediately after this: no control loop, no frame
        stream, and therefore no reading on the frame channel. The reason is
        published on the state channel so the GCS renders why the camera is
        unavailable instead of a plausible-looking default.
        """
        self._reason = reason
        self._log(message, error=True, reason=reason)
        await self._publish_state()

    async def _release_session(self) -> None:
        """Close an open session and drop the frame stream."""
        if self._frames is not None:
            try:
                await self._frames.aclose()
            except Exception:  # noqa: BLE001
                pass
            self._frames = None
        if self._driver is not None and self._session is not None:
            try:
                await self._driver.close(self._session)
            except Exception:  # noqa: BLE001
                log.debug("thermal session release failed", exc_info=True)
        self._session = None

    async def on_stop(self, ctx: Any) -> None:
        """Stop the control loop and release the device."""
        if self._control_task is not None:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
            self._control_task = None
        await self._release_session()
        self._driver = None
        self._reason = REASON_NOT_STARTED
        self._ctx = None

    async def on_disable(self, ctx: Any) -> None:
        await self.on_stop(ctx)

    # -- config -----------------------------------------------------------

    async def _cfg(self, key: str, default: Any) -> Any:
        kv = self._ctx.config_kv
        static = kv.static(key, default) if hasattr(kv, "static") else default
        return await kv.get(key, static)

    # -- video source (hardware-gated) -----------------------------------

    async def _maybe_declare_source(self) -> None:
        """Declare the thermal stream leg to the video pipeline when a stream
        endpoint is configured. The Lepton output is per-pixel Y16 that the
        colorize step turns into a viewable image, so the pipeline can only
        serve it once a colorized RTSP/MJPEG endpoint exists; until then no leg
        is advertised, because a stream leg is offered only when something
        actually serves it."""
        video = getattr(self._ctx, "video", None)
        if video is None or not hasattr(video, "set_source"):
            return
        source = str(await self._cfg("stream_source", "")).strip()
        if not source:
            return
        cameras = [
            {"id": "thermal", "source": source, "role": "ir", "codec": "h264"}
        ]
        try:
            reply = await video.set_source(cameras)
        except Exception as exc:  # noqa: BLE001
            self._log(f"thermal video source config failed: {exc}")
            return
        if isinstance(reply, dict) and reply.get("ok") is not True:
            self._log(f"thermal video source apply did not go live: {reply}")

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
                self._log(f"thermal control tick failed: {exc}")

    async def apply_config_once(self) -> None:
        """Read the config keys and apply any change / fire any one-shot.

        Deterministic and idempotent so tests can drive it directly.
        """
        driver = self._driver
        session = self._session
        if driver is None or session is None:
            return

        # Declarative: palette.
        palette = await self._cfg("palette", None)
        if palette is not None and self._applied.get("palette") != palette:
            self._applied["palette"] = palette
            await self._safe_set_param("palette", palette)

        # Declarative: high/low gain -> radiometric linear resolution.
        gain = await self._cfg("gain", None)
        if gain is not None and self._applied.get("gain") != gain:
            self._applied["gain"] = gain
            await self._safe_set_param(
                "tlinear_resolution", _GAIN_TLINEAR[bool(gain)]
            )

        # One-shot: cycle to the next palette (the cycle-palette Skill).
        if bool(await self._cfg("palette_cycle", False)):
            await self._cycle_palette()
            await self._reset_key("palette_cycle")

        # One-shot: flat field correction (the FFC Skill).
        if bool(await self._cfg("ffc", False)):
            await self._safe_set_param("ffc", None)
            await self._reset_key("ffc")

        await self._publish_state()
        await self._publish_frame_readout()

    # -- telemetry --------------------------------------------------------

    async def _publish_frame_readout(self) -> None:
        """Read one radiometric frame and publish the spot temperature the
        overlay renders on the ``camera.thermal.frame`` channel.

        The colorized picture rides the video pipeline (the stream leg); this
        carries only the lightweight radiometric read-back (spot temperature at
        the centre reticle plus the frame extrema), so the overlay has a live
        temperature to draw over the video without shipping the whole Y16 grid
        over the heartbeat."""
        frames = self._frames
        if frames is None:
            return
        try:
            frame = await anext(frames, None)
        except Exception as exc:  # noqa: BLE001
            self._log(f"thermal frame read failed: {exc}")
            return
        if frame is None:
            return
        readout = _frame_readout(frame)
        if readout is None:
            return
        tel = getattr(self._ctx, "telemetry", None)
        if tel is None or not hasattr(tel, "extend"):
            return
        try:
            await tel.extend(_FRAME_CHANNEL, readout)
        except Exception:  # noqa: BLE001
            log.debug("thermal frame telemetry extend failed", exc_info=True)

    async def _publish_state(self) -> None:
        """Publish the thermal read-back (connectivity, palette, gain) on the
        state channel, on change, so the GCS shows the true state without
        spamming the heartbeat.

        ``connected`` is derived from the open session and nothing else. When
        there is no session the payload carries the machine-readable ``reason``
        the GCS renders; an open session clears it.
        """
        connected = self._session is not None
        payload: dict[str, Any] = {
            "connected": connected,
            "palette": self._applied.get("palette"),
            "gain": bool(self._applied.get("gain", True)),
        }
        if not connected:
            payload["reason"] = self._reason or REASON_NOT_STARTED
        await self._publish_skill_states(connected)
        if payload == self._last_state:
            return
        self._last_state = payload
        tel = getattr(self._ctx, "telemetry", None)
        if tel is None or not hasattr(tel, "extend"):
            return
        try:
            await tel.extend(_STATE_CHANNEL, payload)
        except Exception:  # noqa: BLE001
            log.debug("thermal telemetry extend failed", exc_info=True)

    async def _publish_skill_states(self, connected: bool) -> None:
        """Publish each cockpit Skill's state on its declared topic, on change.

        Both Skills are one-shots that need an open session to do anything, so
        without one they report ``disabled`` plus the reason rather than
        offering the operator a button that cannot fire.
        """
        events = getattr(self._ctx, "events", None)
        if events is None or not hasattr(events, "publish"):
            return
        if connected:
            state: dict[str, Any] = {"state": "idle"}
        else:
            state = {
                "state": "disabled",
                "reason": self._reason or REASON_NOT_STARTED,
            }
        for topic in _SKILL_STATE_TOPICS.values():
            if self._last_skill_states.get(topic) == state:
                continue
            self._last_skill_states[topic] = state
            try:
                await events.publish(topic, state)
            except Exception:  # noqa: BLE001
                log.debug(
                    "thermal skill state publish failed: %s", topic, exc_info=True
                )

    async def _cycle_palette(self) -> None:
        palettes = list_palettes()
        current = getattr(self._session, "palette", palettes[0])
        try:
            idx = palettes.index(current)
        except ValueError:
            idx = -1
        nxt = palettes[(idx + 1) % len(palettes)]
        await self._safe_set_param("palette", nxt)
        self._applied["palette"] = nxt
        # Write the cycled palette back so the settings picker reflects it.
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is not None:
            try:
                await setter("palette", nxt)
            except Exception:  # noqa: BLE001
                log.debug("thermal palette write-back failed", exc_info=True)

    async def _safe_set_param(self, param: str, value: Any) -> None:
        try:
            await self._driver.set_param(self._session, param, value)
        except Exception as exc:  # noqa: BLE001
            self._log(f"thermal set_param {param} failed: {exc}")

    async def _reset_key(self, key: str) -> None:
        """Clear a one-shot key so a re-press fires again (the Skill Bar writes
        the flag true each press with no nonce)."""
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is None:
            return
        try:
            await setter(key, False)
        except Exception:  # noqa: BLE001
            log.debug("thermal action key reset failed: %s", key, exc_info=True)

    # -- helpers ----------------------------------------------------------

    def _log(self, message: str, *, error: bool = False, **fields: Any) -> None:
        logger = getattr(self._ctx, "log", None) if self._ctx is not None else None
        if logger is not None:
            emit = getattr(logger, "error", None) if error else None
            if emit is None:
                emit = getattr(logger, "info", None)
            if emit is not None:
                try:
                    emit(message, extra=fields or None)
                    return
                except Exception:  # noqa: BLE001
                    pass
        (log.error if error else log.info)(message)

    # -- accessors (tests) ------------------------------------------------

    @property
    def driver(self) -> LeptonUvcDriver | None:
        return self._driver

    @property
    def session(self) -> Any:
        return self._session

    @property
    def unavailable_reason(self) -> str:
        """Why there is no open session; empty while one is open."""
        return self._reason
