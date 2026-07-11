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
from typing import Any, Callable

from ados.sdk.tracking import LOCK_LOCKED, EffectiveLock, LockedTargetTracker

from altnautica_gimbal_v2.aim import AimConfig, GimbalAimController
from altnautica_gimbal_v2.ctx_router import ONBOARD_COMPUTER_COMP_ID, _CtxRouter
from altnautica_gimbal_v2.gremsy_driver import GremsyGimbalDriver
from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver
from altnautica_gimbal_v2.sbgc_driver import SimpleBgcGimbalDriver
from altnautica_gimbal_v2.storm32_driver import Storm32NtGimbalDriver

log = logging.getLogger(__name__)

# Only the MAVLink transport is wired for the vision-locked aim loop.
_MAVLINK_TRANSPORT = "mavlink"


_DRIVER_FACTORIES: dict[str, Callable[[Any], Any]] = {
    "mavlink": lambda router: MavlinkGimbalDriver(router=router),
    "sbgc-uart": lambda _router: SimpleBgcGimbalDriver(),
    "storm32-uart": lambda _router: Storm32NtGimbalDriver(),
    "gremsy-uart": lambda _router: GremsyGimbalDriver(),
}


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
        # The shared locked-target safety gate: adopt only the engine's
        # designated target, coast briefly, stop on uncertain/lost, never
        # silently re-lock. Identical gate to Follow-Me, one implementation.
        self._tracker = LockedTargetTracker()

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        self._loop = asyncio.get_running_loop()

        transport = str(await self._cfg("transport", _MAVLINK_TRANSPORT))
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
        designate_camera = (await self._cfg("designate_camera", "")) or None
        aim_default = bool(await self._cfg("aim", False))

        factory = _DRIVER_FACTORIES.get(transport)
        if factory is None:
            raise ValueError(
                f"unknown gimbal transport: {transport!r}; "
                f"expected one of {sorted(_DRIVER_FACTORIES)}"
            )

        self._router = _CtxRouter(
            ctx_mavlink=ctx.mavlink,
            src_system=target_system,
            src_component=ONBOARD_COMPUTER_COMP_ID,
            loop=self._loop,
        )
        self._transport = transport
        self._driver = factory(self._router)

        if transport != _MAVLINK_TRANSPORT:
            # The serial drivers can still open a session and drive manual
            # control, but the vision-locked aim loop is only wired for the
            # MAVLink transport, so it is skipped for the others.
            log.warning(
                "gimbal aim is only wired for the '%s' transport; "
                "'%s' will not run the aim-at-target loop",
                _MAVLINK_TRANSPORT,
                transport,
            )

        candidates = await self._driver.discover()
        if not candidates:
            log.warning(
                "gimbal driver for transport '%s' reported no candidates",
                transport,
            )
            return

        driver_config = {
            "target_system": target_system,
            "target_component": target_component,
            "limits": limits,
        }
        self._session = await self._driver.open(candidates[0], driver_config)

        if transport == _MAVLINK_TRANSPORT:
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
            {"transport": transport, "responsive": True},
        )
        log.info(
            "gimbal started (transport=%s, aim_default=%s)",
            transport,
            aim_default,
        )

    async def on_stop(self, ctx: Any) -> None:
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
        # so arming/disarming takes effect without a restart.
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
            await self._driver.command_attitude(self._session, result[0], result[1])
        except Exception:  # noqa: BLE001
            log.warning("gimbal aim command_attitude failed", exc_info=True)
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
                    "aim_active": aim_active,
                    "commanding": commanding,
                    "lock_state": lock_state,
                    "target_id": target_id,
                },
            )
        except Exception:  # noqa: BLE001
            log.warning("gimbal aim-state publish failed", exc_info=True)

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
