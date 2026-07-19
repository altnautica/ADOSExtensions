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
import time
from typing import Any

from ados.sdk.cameras import CAMERA_SELECTOR_AUTO
from ados.sdk.tracking import LOCK_LOCKED, EffectiveLock, LockedTargetTracker

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

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        self._loop = asyncio.get_running_loop()

        target_system = int(await self._cfg("target_system", 1))
        target_component = int(await self._cfg("target_component", 154))
        limits = await self._cfg("limits", {})
        frame_width = float(await self._cfg("camera_frame_width", 1280))
        frame_height = float(await self._cfg("camera_frame_height", 720))
        hfov_deg = float(await self._cfg("camera_hfov_deg", 66.0))
        vfov_deg = float(await self._cfg("camera_vfov_deg", 41.0))
        aim_gain = float(await self._cfg("aim_gain", 0.3))
        aim_deadband = float(await self._cfg("aim_deadband_frac", 0.04))
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
        # Publish the rate-mode Skill state on change (Rule 44: the toggle
        # reflects the true mode, not a permanent idle).
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
        pitch = float(args.get("pitch_deg", 0.0))
        yaw = float(args.get("yaw_deg", 0.0))
        return await self._point_at(pitch, yaw)

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
