"""MAVLink Gimbal v2 driver.

The first-class driver. Talks to any device that speaks the open
Gimbal Manager Protocol v2: the ArduPilot SITL ``mount_servo`` model,
Storm32 NT, Gremsy, and any other spec-compliant gimbal that emits
``GIMBAL_DEVICE_ATTITUDE_STATUS`` and accepts the four manager commands.

The driver is constructed with a router handle that exposes a
``send_command`` callable. The agent's MAVLink router supplies this
handle at plugin start. Tests inject a mock router that records the
byte payloads the driver emits.
"""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import replace
from typing import Any, AsyncIterator, Callable, Protocol

from ados.sdk.drivers.gimbal import (
    GimbalCandidate,
    GimbalCapabilities,
    GimbalDriver,
    GimbalSession,
    GimbalState,
)

from altnautica_gimbal_v2.mavlink_messages import (
    CommandInt,
    CommandLong,
    encode_gimbal_manager_configure,
    encode_gimbal_manager_pitchyaw,
    encode_set_roi_location,
    encode_set_roi_none,
)


class RouterHandle(Protocol):
    """Minimal surface the driver needs from the agent's MAVLink router.

    The router is the agent process that owns the underlying serial or
    UDP transport. It accepts a ``CommandLong`` or ``CommandInt`` value
    and returns ``True`` if the command was queued. The driver does not
    block on ACK; the gimbal manager state machine handles re-emit on
    timeout.
    """

    def send_command(
        self, cmd: CommandLong | CommandInt
    ) -> bool:  # pragma: no cover - protocol
        ...


class _MavlinkSession(GimbalSession):
    """Per-open state for the MAVLink driver."""

    def __init__(
        self,
        candidate: GimbalCandidate,
        config: dict[str, Any],
        target_system: int,
        target_component: int,
    ) -> None:
        self.candidate = candidate
        self.config = config
        self.target_system = target_system
        self.target_component = target_component
        self.state = GimbalState(
            timestamp_ns=time.time_ns(),
            pitch_deg=0.0,
            yaw_deg=0.0,
            roll_deg=0.0,
            mode="neutral",
        )
        self.last_response_ns: int = 0
        self.queue: asyncio.Queue[GimbalState] = asyncio.Queue(maxsize=64)
        self.closed = False


class MavlinkGimbalDriver(GimbalDriver):
    """``GimbalDriver`` over the open MAVLink Gimbal v2 protocol."""

    driver_id = "mavlink-gimbal-v2"

    def __init__(
        self,
        router: RouterHandle,
        clock: Callable[[], int] = time.time_ns,
    ) -> None:
        self._router = router
        self._clock = clock
        self._sessions: list[_MavlinkSession] = []

    async def discover(self) -> list[GimbalCandidate]:
        """Return the single virtual candidate for the MAVLink path.

        Discovery is logical, not physical: any MAVLink network may
        carry a gimbal device, and the router multiplexes them. We
        emit one candidate that the supervisor can open with the
        target system and component overrides supplied via config.
        """

        return [
            GimbalCandidate(
                driver_id=self.driver_id,
                device_id="mavlink:primary",
                label="MAVLink Gimbal v2 (primary)",
                bus="mavlink",
                metadata={"component_id": 154},
            )
        ]

    async def open(
        self, candidate: GimbalCandidate, config: dict[str, Any]
    ) -> GimbalSession:
        target_system = int(config.get("target_system", 1))
        target_component = int(config.get("target_component", 154))
        # The component that will actually transmit every pitch/yaw command,
        # supplied by the plugin from the router it builds. It has to be the
        # component announced as holding primary control: a spec-compliant
        # manager that enforces the primary-control handshake is entitled to
        # ignore commands from any other sender, and would then refuse to move
        # while the plugin still reported itself as commanding.
        src_component = int(config.get("src_component", 191))
        session = _MavlinkSession(candidate, config, target_system, target_component)
        # Announce primary control by default. The configure call is
        # idempotent on the gimbal side; the manager replies with
        # GIMBAL_MANAGER_STATUS at its next 1 Hz tick.
        configure = encode_gimbal_manager_configure(
            primary_sysid=target_system,
            primary_compid=src_component,
            target_system=target_system,
            target_component=target_component,
        )
        self._router.send_command(configure)
        self._sessions.append(session)
        return session

    async def close(self, session: GimbalSession) -> None:
        sess = self._typed(session)
        sess.closed = True
        # Best-effort ROI release so a subsequent operator does not
        # inherit a stuck lock.
        release = encode_set_roi_none(
            target_system=sess.target_system,
            target_component=sess.target_component,
        )
        self._router.send_command(release)
        if sess in self._sessions:
            self._sessions.remove(sess)

    def capabilities(self, session: GimbalSession) -> GimbalCapabilities:
        sess = self._typed(session)
        limits = sess.config.get("limits", {})
        return GimbalCapabilities(
            has_pitch=True,
            has_yaw=True,
            has_roll=True,
            pitch_min_deg=float(limits.get("pitch_min_deg", -90.0)),
            pitch_max_deg=float(limits.get("pitch_max_deg", 30.0)),
            yaw_min_deg=float(limits.get("yaw_min_deg", -180.0)),
            yaw_max_deg=float(limits.get("yaw_max_deg", 180.0)),
            roll_min_deg=float(limits.get("roll_min_deg", -45.0)),
            roll_max_deg=float(limits.get("roll_max_deg", 45.0)),
            max_rate_dps=180.0,
            supports_follow_mode=True,
            supports_lock_mode=True,
        )

    async def command_attitude(
        self,
        session: GimbalSession,
        pitch_deg: float,
        yaw_deg: float,
        roll_deg: float = 0.0,
    ) -> None:
        sess = self._typed(session)
        caps = self.capabilities(sess)
        clamped_pitch = _clamp(
            _finite("pitch_deg", pitch_deg), caps.pitch_min_deg, caps.pitch_max_deg
        )
        clamped_yaw = _clamp(
            _finite("yaw_deg", yaw_deg), caps.yaw_min_deg, caps.yaw_max_deg
        )
        cmd = encode_gimbal_manager_pitchyaw(
            pitch_deg=clamped_pitch,
            yaw_deg=clamped_yaw,
            target_system=sess.target_system,
            target_component=sess.target_component,
        )
        self._router.send_command(cmd)
        sess.state = replace(
            sess.state,
            timestamp_ns=self._clock(),
            pitch_deg=clamped_pitch,
            yaw_deg=clamped_yaw,
            roll_deg=_clamp(
                _finite("roll_deg", roll_deg),
                caps.roll_min_deg,
                caps.roll_max_deg,
            ),
            mode="manual",
        )

    async def command_rate(
        self,
        session: GimbalSession,
        pitch_rate_dps: float,
        yaw_rate_dps: float,
        roll_rate_dps: float = 0.0,
    ) -> None:
        sess = self._typed(session)
        caps = self.capabilities(sess)
        # The driver advertises a max_rate_dps ceiling in its capabilities, so
        # it enforces it: whatever the visual-servo gain or camera field of
        # view in the per-drone config, no commanded rate leaves here above the
        # declared maximum.
        pitch_rate = _clamp(
            _finite("pitch_rate_dps", pitch_rate_dps),
            -caps.max_rate_dps,
            caps.max_rate_dps,
        )
        yaw_rate = _clamp(
            _finite("yaw_rate_dps", yaw_rate_dps),
            -caps.max_rate_dps,
            caps.max_rate_dps,
        )
        # The PITCHYAW frame carries no roll rate, so roll is validated but not
        # transmitted: a caller passing a non-finite roll rate gets an error
        # rather than the silent drop.
        _finite("roll_rate_dps", roll_rate_dps)
        # Rate commands are sent as PITCHYAW with the rate fields filled
        # and the position fields left at the current state. The gimbal
        # device integrates the rate against its own clock.
        cmd = encode_gimbal_manager_pitchyaw(
            pitch_deg=sess.state.pitch_deg,
            yaw_deg=sess.state.yaw_deg,
            pitch_rate_dps=pitch_rate,
            yaw_rate_dps=yaw_rate,
            target_system=sess.target_system,
            target_component=sess.target_component,
        )
        self._router.send_command(cmd)

    async def set_roi_location(
        self,
        session: GimbalSession,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
    ) -> None:
        """Lock the gimbal on a fixed lat/lon/alt target.

        Not part of the abstract base, but exposed so the plugin's
        REST surface and the GCS panel can route ROI commands through
        the same driver instance.
        """

        sess = self._typed(session)
        cmd = encode_set_roi_location(
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            alt_m=alt_m,
            target_system=sess.target_system,
            target_component=sess.target_component,
        )
        self._router.send_command(cmd)
        sess.state = replace(sess.state, mode="roi-lock", timestamp_ns=self._clock())

    async def clear_roi(self, session: GimbalSession) -> None:
        """Release any active ROI lock and return to manual mode."""

        sess = self._typed(session)
        cmd = encode_set_roi_none(
            target_system=sess.target_system,
            target_component=sess.target_component,
        )
        self._router.send_command(cmd)
        sess.state = replace(sess.state, mode="manual", timestamp_ns=self._clock())

    def get_state(self, session: GimbalSession) -> GimbalState:
        return self._typed(session).state

    async def state_iterator(
        self, session: GimbalSession
    ) -> AsyncIterator[GimbalState]:
        sess = self._typed(session)

        async def gen() -> AsyncIterator[GimbalState]:
            while not sess.closed:
                yield await sess.queue.get()

        return gen()

    def on_attitude_status(
        self,
        session: GimbalSession,
        pitch_deg: float,
        yaw_deg: float,
        roll_deg: float,
        pitch_rate_dps: float = 0.0,
        yaw_rate_dps: float = 0.0,
        roll_rate_dps: float = 0.0,
    ) -> None:
        """Inject a gimbal device attitude status into the session.

        The plugin's MAVLink subscriber calls this when the router
        delivers a ``GIMBAL_DEVICE_ATTITUDE_STATUS`` message. Tests
        call it directly to drive the round-trip path without a real
        router.
        """

        sess = self._typed(session)
        new_state = GimbalState(
            timestamp_ns=self._clock(),
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            roll_deg=roll_deg,
            pitch_rate_dps=pitch_rate_dps,
            yaw_rate_dps=yaw_rate_dps,
            roll_rate_dps=roll_rate_dps,
            mode=sess.state.mode,
        )
        sess.state = new_state
        sess.last_response_ns = new_state.timestamp_ns
        try:
            sess.queue.put_nowait(new_state)
        except asyncio.QueueFull:
            # Drop oldest sample under load. The state field is the
            # authoritative live readout; the iterator is a best-effort
            # stream.
            try:
                sess.queue.get_nowait()
                sess.queue.put_nowait(new_state)
            except asyncio.QueueEmpty:  # pragma: no cover - race
                pass

    def _typed(self, session: GimbalSession) -> _MavlinkSession:
        if not isinstance(session, _MavlinkSession):
            raise TypeError(
                "session was not produced by MavlinkGimbalDriver.open()"
            )
        return session


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp into ``[lo, hi]``.

    Comparison-based and therefore only meaningful for a finite value; callers
    put every axis value through :func:`_finite` first.
    """
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _finite(name: str, value: float) -> float:
    """Return ``value`` as a float, rejecting NaN and the infinities.

    Every comparison against NaN is false, so a comparison-based clamp returns
    NaN unchanged. In the Gimbal Manager v2 protocol a NaN parameter has the
    defined meaning "do not change", so an unguarded NaN is transmitted as a
    silent no-op while the plugin still reports the axis as commanded. An axis
    value that cannot be represented is an error, not something to clamp.
    """
    as_float = float(value)
    if not math.isfinite(as_float):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return as_float
