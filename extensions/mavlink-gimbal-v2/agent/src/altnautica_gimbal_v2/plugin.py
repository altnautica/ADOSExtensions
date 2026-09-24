"""Plugin entry point.

The agent's plugin runner instantiates ``GimbalV2Plugin`` with NO
constructor arguments and drives the lifecycle hooks (``on_start`` /
``on_stop``) with a rich :class:`PluginContext`. This module reads its
config, selects a driver from the ``transport`` field, opens a session,
registers the gimbal-manager component, and -- for the MAVLink transport
-- subscribes to the vision detection stream to run an "Aim at this
target" visual servo.

The heavy MAVLink framing lives in ``mavlink_driver.py`` (the driver) and
``ctx_router.py`` (the adapter that packs the driver's commands onto
``ctx.mavlink.send``). The visual-servo math lives in ``aim.py``. This
module is the lifecycle + safety-gate glue.

Aim safety gate: the loop commands the gimbal ONLY when the vision engine
reports a target that is both tracked (``track_id`` present) and
``locked``. On any other state -- uncertain, lost, or no tracked target
-- it stops commanding and the gimbal holds its last angle. The operator
owns designation through the vision engine; this plugin never picks a
target itself, and it never silently follows an unlocked box.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from ados.sdk.cameras import CAMERA_SELECTOR_AUTO
from ados.sdk.tracking import LOCK_LOCKED, EffectiveLock, LockedTargetTracker

from altnautica_gimbal_v2 import attitude as gimbal_attitude
from altnautica_gimbal_v2.aim import AimConfig, GimbalAimController
from altnautica_gimbal_v2.ctx_router import ONBOARD_COMPUTER_COMP_ID, _CtxRouter
from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver

log = logging.getLogger(__name__)

# The extension speaks the open MAVLink Gimbal Manager Protocol v2 only; the
# transport label reported on the health event and the status tool.
_TRANSPORT = "mavlink"

# How often the control loop reads the one-shot action keys (recenter / nadir)
# and refreshes the rate-mode flag.
_CONTROL_HZ = 5.0

# One-shot action keys the cockpit Skills flip to true. The control loop fires
# the matching command on the rising edge and resets the key so a re-press
# fires again (the Skill Bar has no nonce; it writes the flag true each press).
_ACTION_KEYS = ("recenter", "nadir")

# Panel command keys. The Gimbal tab writes ``point`` ({pitch_deg, yaw_deg}),
# ``roi`` ({lat_deg, lon_deg, alt_m}) or ``roi_clear`` (true); the control loop
# executes each once and resets it, like the one-shot Skills.
_POINT_KEY = "point"
_ROI_KEY = "roi"
_ROI_CLEAR_KEY = "roi_clear"

# The ``gimbal`` telemetry channel is republished at most this often. The
# gimbal reports its attitude far faster, and every publish is a host write.
_TELEMETRY_MIN_INTERVAL_S = 0.2


def _finite_numbers(*values: Any) -> bool:
    """True when every value is a real, finite number (bools excluded)."""
    return all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in values
    )


class GimbalV2Plugin:
    """Lifecycle-hook plugin the runner instantiates with no args."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._router: _CtxRouter | None = None
        self._driver: Any = None
        self._session: Any = None
        self._controller: GimbalAimController | None = None
        self._transport: str | None = None
        self._last_aim_state: tuple[Any, ...] | None = None
        # Rate-control mode: when on, the aim loop commands the gimbal by
        # angular rate instead of an absolute angle. The Skill toggles it.
        self._rate_mode = False
        # The pitch (degrees) the nadir Skill points the gimbal to (straight
        # down): the driver's lower pitch limit, resolved from its caps.
        self._nadir_pitch = -90.0
        self._control_task: asyncio.Task | None = None
        # Last rate-mode value published as a state event, so the rate-mode
        # Skill Bar reflects the true on/off state without spamming the bus.
        self._last_rate_published: bool | None = None
        # The shared locked-target safety gate: adopt only the engine's
        # designated target, coast briefly, stop on uncertain/lost, never
        # silently re-lock. Identical gate to Follow-Me, one implementation.
        self._tracker = LockedTargetTracker()
        # Monotonic time of the last ``gimbal`` telemetry publish.
        self._last_telemetry_at: float | None = None

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        self._loop = asyncio.get_running_loop()

        target_system = int(await self._cfg("target_system", 1))
        target_component = int(await self._cfg("target_component", 154))
        limits = await self._cfg("limits", {})
        # The aim-loop geometry comes from per-drone config, which the GCS
        # widget bounds but an MCP client, a restored config file or a direct
        # write does not. config-schema.json is the public contract, so each
        # value is checked against the range it declares and falls back to the
        # documented default when it is outside — an out-of-range gain or field
        # of view otherwise scales straight into the commanded gimbal rate.
        frame_width = await self._cfg_bounded(
            "camera_frame_width", default=1280.0, low=1.0, high=None
        )
        frame_height = await self._cfg_bounded(
            "camera_frame_height", default=720.0, low=1.0, high=None
        )
        hfov_deg = await self._cfg_bounded(
            "camera_hfov_deg", default=66.0, low=1.0, high=180.0
        )
        vfov_deg = await self._cfg_bounded(
            "camera_vfov_deg", default=41.0, low=1.0, high=180.0
        )
        aim_gain = await self._cfg_bounded(
            "aim_gain", default=0.3, low=0.05, high=1.0
        )
        aim_deadband = await self._cfg_bounded(
            "aim_deadband_frac", default=0.04, low=0.0, high=0.3
        )
        invert_pitch = bool(await self._cfg("aim_invert_pitch", False))
        invert_yaw = bool(await self._cfg("aim_invert_yaw", False))
        # Empty string (the schema default) means "all cameras": the
        # vision subscription wants None for that, not a literal "" filter
        # that would match a camera named "".
        designate_camera = self._resolve_designate_camera(
            await self._cfg("designate_camera", "")
        )
        aim_default = bool(await self._cfg("aim", False))
        self._rate_mode = bool(await self._cfg("rate_mode", False))

        self._router = _CtxRouter(
            ctx_mavlink=ctx.mavlink,
            src_system=target_system,
            src_component=ONBOARD_COMPUTER_COMP_ID,
            loop=self._loop,
        )
        self._transport = _TRANSPORT
        self._driver = MavlinkGimbalDriver(router=self._router)

        candidates = await self._driver.discover()
        if not candidates:
            log.warning("gimbal driver reported no candidates")
            return

        driver_config = {
            "target_system": target_system,
            "target_component": target_component,
            "limits": limits,
            # The component the router transmits as, so the driver announces
            # primary control for the sender the gimbal will actually see.
            "src_component": ONBOARD_COMPUTER_COMP_ID,
        }
        self._session = await self._driver.open(candidates[0], driver_config)

        # Resolve the nadir (straight-down) pitch from the driver's lower pitch
        # limit so the nadir Skill points to the real axis bound, not a guess.
        try:
            self._nadir_pitch = float(
                self._driver.capabilities(self._session).pitch_min_deg
            )
        except Exception:  # noqa: BLE001
            self._nadir_pitch = -90.0

        await ctx.mavlink.register_component(target_component, "gimbal")
        # The gimbal's own attitude report drives the driver state and the
        # ``gimbal`` telemetry channel the Gimbal tab renders.
        await ctx.mavlink.subscribe(
            gimbal_attitude.ATTITUDE_STATUS_MSG, self._on_attitude_status
        )
        # Take the clamp limits from the driver's capabilities so the
        # controller model and the driver agree on the axis bounds.
        caps = self._driver.capabilities(self._session)
        self._controller = GimbalAimController(
            AimConfig(
                frame_width=frame_width,
                frame_height=frame_height,
                hfov_deg=hfov_deg,
                vfov_deg=vfov_deg,
                gain=aim_gain,
                deadband_frac=aim_deadband,
                invert_pitch=invert_pitch,
                invert_yaw=invert_yaw,
                pitch_min_deg=caps.pitch_min_deg,
                pitch_max_deg=caps.pitch_max_deg,
                yaw_min_deg=caps.yaw_min_deg,
                yaw_max_deg=caps.yaw_max_deg,
            )
        )
        await ctx.vision.subscribe_detections(
            self._on_batch, camera_id=designate_camera
        )

        await ctx.events.publish(
            "sensor.gimbal.health",
            {"transport": _TRANSPORT, "responsive": True},
        )

        # Register the MCP tools (guarded: the host injects ctx.tools only when
        # the mcp.expose capability is granted; tests pass a ctx without it).
        self._register_tools(ctx)

        # The control loop fires the one-shot Skills (recenter / nadir) and
        # refreshes the rate-mode flag; it runs whenever a session is open.
        self._control_task = asyncio.create_task(self._control_loop())

        log.info(
            "gimbal started (aim_default=%s, rate_mode=%s)",
            aim_default,
            self._rate_mode,
        )

    async def on_stop(self, ctx: Any) -> None:
        if self._control_task is not None:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
            self._control_task = None
        driver = self._driver
        session = self._session
        if driver is not None and session is not None:
            try:
                await driver.close(session)
            except Exception:  # noqa: BLE001
                log.warning("gimbal close failed during on_stop", exc_info=True)
        self._driver = None
        self._session = None
        self._controller = None
        self._router = None
        self._loop = None
        self._last_aim_state = None
        self._tracker.drop()
        self._ctx = None

    # -- config -------------------------------------------------------

    async def _cfg(self, key: str, default: Any) -> Any:
        """Live per-drone config value, falling back to the manifest
        static config, then to ``default``."""
        static = self._ctx.config_kv.static(key, default)
        return await self._ctx.config_kv.get(key, static)

    async def _cfg_bounded(
        self,
        key: str,
        *,
        default: float,
        low: float,
        high: float | None,
    ) -> float:
        """A numeric config value validated against the range the extension's
        config schema declares for it.

        A value that is not a finite number, or that falls outside
        ``[low, high]``, is a config error: the documented default is used and
        the substitution is logged. Silently clamping instead would leave the
        aim model running on geometry the operator never chose.
        """
        raw = await self._cfg(key, default)
        try:
            value = float(raw)
        except (TypeError, ValueError):
            log.warning("gimbal config %s is not a number (%r); using %s", key, raw, default)
            return default
        if not math.isfinite(value):
            log.warning("gimbal config %s is not finite (%r); using %s", key, raw, default)
            return default
        if value < low or (high is not None and value > high):
            log.warning(
                "gimbal config %s=%s is outside the declared range [%s, %s]; using %s",
                key,
                value,
                low,
                "inf" if high is None else high,
                default,
            )
            return default
        return value

    # -- detections (the aim safety gate) -----------------------------

    async def _on_batch(self, batch: Any) -> None:
        ctx = self._ctx
        if (
            ctx is None
            or self._controller is None
            or self._driver is None
            or self._session is None
        ):
            return

        # Hand the batch to the shared safety gate. It adopts the engine's
        # designated target (never our own) and applies the coast window.
        now = time.monotonic()
        self._tracker.record(batch, now)

        # The target action flips the ``aim`` key live; read it each batch
        # so arming/disarming takes effect without a restart. The rate-mode
        # flag is refreshed by the control loop.
        try:
            aim_active = bool(await ctx.config_kv.get("aim", False))
        except Exception:  # noqa: BLE001
            aim_active = False

        if not aim_active:
            # Not armed: do nothing (no command, no drop).
            return

        lock = self._tracker.effective_lock(now)
        if lock is EffectiveLock.LOST:
            # Lost (tracker-reported, or the coast window elapsed): drop the
            # lock so a fresh operator designate is required, and send nothing
            # -- the gimbal holds its last physical angle. The controller model
            # is intentionally NOT reset: it stays equal to the held angle, so
            # a re-lock resumes smoothly instead of jumping toward centre.
            self._tracker.drop()
            await self._publish_aim_state(
                aim_active=True, commanding=False, lock_state="lost", target_id=None
            )
            return

        target = self._tracker.locked_target(now)
        if target is None:
            # Uncertain / coasting / nothing adopted: hold, emit no command.
            await self._publish_aim_state(
                aim_active=True,
                commanding=False,
                lock_state=lock.value,
                target_id=self._tracker.track_id,
            )
            return

        # Rate mode servos the gimbal by angular rate; position mode drives it
        # to an absolute setpoint. Both use one error model (see aim.py) and
        # both hold (no command) inside the dead-band.
        if self._rate_mode:
            result = self._controller.rate(target.bbox)
        else:
            result = self._controller.update(target.bbox)
        if result is None:
            # Inside the dead-band: on target, hold, emit no command.
            await self._publish_aim_state(
                aim_active=True,
                commanding=False,
                lock_state=LOCK_LOCKED,
                target_id=target.track_id,
            )
            return

        try:
            if self._rate_mode:
                await self._driver.command_rate(self._session, result[0], result[1])
            else:
                await self._driver.command_attitude(
                    self._session, result[0], result[1]
                )
        except Exception:  # noqa: BLE001
            log.warning("gimbal aim command failed", exc_info=True)
            return

        await self._publish_aim_state(
            aim_active=True,
            commanding=True,
            lock_state=LOCK_LOCKED,
            target_id=target.track_id,
        )

    async def _publish_aim_state(
        self,
        *,
        aim_active: bool,
        commanding: bool,
        lock_state: str | None,
        target_id: int | None,
    ) -> None:
        """Publish the aim read-back, gated on change to avoid spamming
        the event bus on every detection batch. Best-effort."""

        state = (aim_active, commanding, lock_state, target_id)
        if state == self._last_aim_state:
            return
        self._last_aim_state = state
        ctx = self._ctx
        if ctx is None:
            return
        try:
            await ctx.events.publish(
                "sensor.gimbal.aim",
                {
                    # `state` drives the aim Skill's Bar state (active when the
                    # aim behaviour is armed); the rest is the panel read-back.
                    "state": "active" if aim_active else "idle",
                    "aim_active": aim_active,
                    "commanding": commanding,
                    "lock_state": lock_state,
                    "target_id": target_id,
                },
            )
        except Exception:  # noqa: BLE001
            log.warning("gimbal aim-state publish failed", exc_info=True)

    # -- camera binding ----------------------------------------------

    def _resolve_designate_camera(self, selection: Any) -> str | None:
        """Resolve the camera-selector value to the detection-subscription
        filter. By-requirement (auto / empty) subscribes to every camera
        (``None``) and follows whichever target the vision engine designates; a
        pinned id filters to that camera. The full roster-based resolution runs
        host-side; the subscription filter only needs the pinned-vs-any
        distinction."""
        if selection is None:
            return None
        s = str(selection).strip()
        if s == "" or s == CAMERA_SELECTOR_AUTO:
            return None
        return s

    # -- control loop (one-shot Skills + rate-mode) -------------------

    async def _control_loop(self) -> None:
        period = 1.0 / _CONTROL_HZ
        while True:
            await asyncio.sleep(period)
            try:
                await self.poll_control_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                log.warning("gimbal control tick failed: %s", exc)

    async def poll_control_once(self) -> None:
        """Refresh the rate-mode flag and fire any pending one-shot action.

        Deterministic and idempotent so tests can drive it directly.
        """
        ctx = self._ctx
        if ctx is None:
            return
        try:
            self._rate_mode = bool(
                await ctx.config_kv.get("rate_mode", self._rate_mode)
            )
        except Exception:  # noqa: BLE001
            pass
        # Publish the rate-mode Skill state on change, so the Skill Bar toggle
        # reflects the mode the loop is actually in rather than a permanent idle.
        if self._rate_mode != self._last_rate_published:
            self._last_rate_published = self._rate_mode
            try:
                await ctx.events.publish(
                    "sensor.gimbal.rate_mode",
                    {"state": "active" if self._rate_mode else "idle"},
                )
            except Exception:  # noqa: BLE001
                log.debug("gimbal rate-mode state publish failed", exc_info=True)
        for key in _ACTION_KEYS:
            try:
                fired = bool(await ctx.config_kv.get(key, False))
            except Exception:  # noqa: BLE001
                fired = False
            if not fired:
                continue
            await self._fire_action(key)
            await self._reset_key(key)
        await self._run_panel_commands(ctx)

    async def _run_panel_commands(self, ctx: Any) -> None:
        """Execute the Gimbal tab's point / ROI commands once each and reset
        them. A value that is not the documented shape is dropped (reset) with
        a warning rather than sent."""
        point = await ctx.config_kv.get(_POINT_KEY, None)
        if isinstance(point, dict):
            pitch, yaw = point.get("pitch_deg"), point.get("yaw_deg")
            if _finite_numbers(pitch, yaw):
                await self._point_at(float(pitch), float(yaw))
            else:
                log.warning("gimbal point command is malformed: %r", point)
            await self._reset_key(_POINT_KEY)
        roi = await ctx.config_kv.get(_ROI_KEY, None)
        if isinstance(roi, dict):
            lat, lon, alt = roi.get("lat_deg"), roi.get("lon_deg"), roi.get("alt_m")
            driver, session = self._driver, self._session
            if not _finite_numbers(lat, lon, alt) or abs(lat) > 90 or abs(lon) > 180:
                log.warning("gimbal ROI command is malformed: %r", roi)
            elif driver is not None and session is not None:
                try:
                    await driver.set_roi_location(session, float(lat), float(lon), float(alt))
                except Exception as exc:  # noqa: BLE001
                    log.warning("gimbal ROI lock failed: %s", exc)
            await self._reset_key(_ROI_KEY)
        if bool(await ctx.config_kv.get(_ROI_CLEAR_KEY, False)):
            driver, session = self._driver, self._session
            if driver is not None and session is not None:
                try:
                    await driver.clear_roi(session)
                except Exception as exc:  # noqa: BLE001
                    log.warning("gimbal ROI release failed: %s", exc)
            await self._reset_key(_ROI_CLEAR_KEY)

    # -- attitude read-back --------------------------------------------

    async def _on_attitude_status(self, delivery: dict[str, Any]) -> None:
        """Feed the gimbal's attitude report into the driver state and publish
        the ``gimbal`` telemetry channel, rate-limited."""
        fields = gimbal_attitude.decode_fields(delivery)
        att = gimbal_attitude.attitude_from_status(fields) if fields else None
        driver, session, ctx = self._driver, self._session, self._ctx
        if att is None or driver is None or session is None or ctx is None:
            return
        driver.on_attitude_status(
            session,
            att.pitch_deg,
            att.yaw_deg,
            att.roll_deg,
            att.pitch_rate_dps,
            att.yaw_rate_dps,
            att.roll_rate_dps,
        )
        now = time.monotonic()
        last = self._last_telemetry_at
        if last is not None and now - last < _TELEMETRY_MIN_INTERVAL_S:
            return
        self._last_telemetry_at = now
        payload = gimbal_attitude.telemetry_payload(
            att, driver.get_state(session).mode, int(time.time() * 1000)
        )
        try:
            await ctx.telemetry.extend("gimbal", payload)
        except Exception:  # noqa: BLE001
            log.debug("gimbal telemetry extend failed", exc_info=True)

    async def _fire_action(self, key: str) -> None:
        if key == "recenter":
            await self._point_at(0.0, 0.0, recenter_model=True)
        elif key == "nadir":
            await self._point_at(self._nadir_pitch, 0.0)

    async def _reset_key(self, key: str) -> None:
        """Reset a one-shot action key so a re-press fires again. The Skill Bar
        writes the flag true each press with no nonce, so the agent clears it
        after firing."""
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is None:
            return
        try:
            await setter(key, False)
        except Exception:  # noqa: BLE001
            log.debug("gimbal action key reset failed: %s", key, exc_info=True)

    async def _point_at(
        self, pitch_deg: float, yaw_deg: float, *, recenter_model: bool = False
    ) -> dict:
        """Command the gimbal to an absolute pitch/yaw. The driver clamps to
        its axis limits. Returns a result dict for the MCP tool."""
        driver = self._driver
        session = self._session
        if driver is None or session is None:
            return {"ok": False, "reason": "no gimbal session"}
        try:
            await driver.command_attitude(session, float(pitch_deg), float(yaw_deg))
        except Exception as exc:  # noqa: BLE001
            log.warning("gimbal point_at failed: %s", exc)
            return {"ok": False, "reason": str(exc)}
        if recenter_model and self._controller is not None:
            # Keep the visual-servo model consistent with the physical recenter
            # so a subsequent aim resumes from centre, not the pre-recenter angle.
            self._controller.reset()
        return {"ok": True, "pitch_deg": float(pitch_deg), "yaw_deg": float(yaw_deg)}

    # -- MCP tools ----------------------------------------------------

    def _register_tools(self, ctx: Any) -> None:
        tools = getattr(ctx, "tools", None)
        if tools is None or not hasattr(tools, "register"):
            return
        tools.register("status", self._tool_status)
        tools.register("point_at", self._tool_point_at)
        tools.register("recenter", self._tool_recenter)

    async def _tool_status(self, _args: dict) -> dict:
        aim = self._last_aim_state
        return {
            "transport": self._transport,
            "connected": self._session is not None,
            "rate_mode": self._rate_mode,
            "aim": {
                "aim_active": aim[0] if aim else False,
                "commanding": aim[1] if aim else False,
                "lock_state": aim[2] if aim else None,
                "target_id": aim[3] if aim else None,
            },
        }

    async def _tool_point_at(self, args: dict) -> dict:
        # Every other tool here answers with the {ok, reason} contract, so a
        # non-numeric or non-finite argument must too: raising out of the
        # handler would show the caller a transport error instead of a usable
        # refusal, and a NaN angle means "do not change" on the wire.
        angles: dict[str, float] = {}
        for name, default in (("pitch_deg", 0.0), ("yaw_deg", 0.0)):
            try:
                value = float(args.get(name, default))
            except (TypeError, ValueError):
                return {"ok": False, "reason": f"{name} must be a number"}
            if not math.isfinite(value):
                return {"ok": False, "reason": f"{name} must be finite"}
            angles[name] = value
        return await self._point_at(angles["pitch_deg"], angles["yaw_deg"])

    async def _tool_recenter(self, _args: dict) -> dict:
        return await self._point_at(0.0, 0.0, recenter_model=True)

    # -- accessors (used by tests) ------------------------------------

    @property
    def driver(self) -> Any:
        return self._driver

    @property
    def session(self) -> Any:
        return self._session

    @property
    def controller(self) -> GimbalAimController | None:
        return self._controller
