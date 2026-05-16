"""VIO engine abstraction.

A :class:`BaseVioEngine` represents one spawned vendor binary and the
two IPC channels (SHM ring for frames + msgpack UDS for everything
else). The estimator above talks only to this abstraction; the engine
hides the binary-spawning details and the message wire format.

Two concrete engines ship in v1: :class:`OpenVinsEngine` and
:class:`VinsFusionEngine`. Both use the same IPC contract; the only
difference is the basename passed to ``process.spawn`` and the
engine-id string they advertise on the heartbeat.
"""

from __future__ import annotations

import logging
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from altnautica_vision_nav.shim.ipc import MsgpackChannel, ShmFrameRing

log = logging.getLogger(__name__)


PROTOCOL_VERSION = 1
DEFAULT_HEARTBEAT_GRACE_S = 2.0
DEFAULT_FRAME_SLOT_COUNT = 4


class VioEngineError(RuntimeError):
    """Raised when the engine fails to start, dies, or rejects a message."""


@dataclass
class EngineConfig:
    """Per-session configuration the shim sends to the vendor on connect.

    Mirrors the Kalibr-style camera + extrinsics output so the operator
    can hand-roll one file via :func:`load_intrinsics` / :func:`load_extrinsics`
    and feed it straight in.
    """

    # Intrinsics (pinhole + radtan or none)
    camera_model: str
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion_model: str
    distortion_coeffs: tuple[float, ...]
    # Extrinsics (T_cam_imu, 16 row-major floats)
    t_cam_imu: tuple[float, ...]
    # Time offset (seconds, Kalibr convention t_imu = t_cam + ts)
    timeshift_cam_imu_s: float
    # IMU + camera rates
    imu_rate_hz: float
    camera_rate_hz: float


@dataclass(frozen=True)
class PoseMessage:
    """One pose return from the vendor binary.

    Mirrors the ``{type: "pose", ...}`` msgpack message. The estimator
    above translates this into an ``EstimatorOutput``.
    """

    ts_us: int
    position: tuple[float, float, float]
    orientation_quat: tuple[float, float, float, float]  # (w, x, y, z)
    velocity: tuple[float, float, float]
    covariance: tuple[float, ...]  # upper-triangular over [pos, vel]; 21 floats
    feature_count: int
    reset_counter: int
    state: str  # "init"|"converging"|"converged"|"degraded"|"failed"


class BaseVioEngine(ABC):
    """Common shape for every concrete VIO engine.

    Concrete engines override :attr:`engine_id` and :attr:`basename`
    (the binary name in the manifest's ``subprocess_spawn`` allowlist).
    The lifecycle is:

    1. :meth:`start` — spawn the binary, open the channels, send
       ``hello`` + ``config``.
    2. :meth:`send_imu` / :meth:`send_frame` per sensor sample. Pose
       returns drain from :meth:`drain_poses` (non-blocking).
    3. :meth:`is_alive` is polled by the watchdog; returns False if
       the vendor has not sent an ``alive`` message recently.
    4. :meth:`stop` — graceful shutdown; the watchdog respawns on a
       miss.

    The engine accepts an optional ``transport_factory`` so tests can
    inject in-memory MsgpackChannels instead of real UDS sockets.
    """

    engine_id: str = "base"
    basename: str = "ados_vio_shim"

    def __init__(
        self,
        *,
        spawn_callable: Optional[object] = None,
        transport_factory: Optional[object] = None,
        frame_slot_count: int = DEFAULT_FRAME_SLOT_COUNT,
        frame_slot_size: int = 1280 * 800 * 3,
        heartbeat_grace_s: float = DEFAULT_HEARTBEAT_GRACE_S,
    ) -> None:
        self._spawn_callable = spawn_callable
        self._transport_factory = transport_factory
        self._frame_slot_count = int(frame_slot_count)
        self._frame_slot_size = int(frame_slot_size)
        self._heartbeat_grace_s = float(heartbeat_grace_s)
        self._channel: Optional[MsgpackChannel] = None
        self._ring: Optional[ShmFrameRing] = None
        self._lock = threading.Lock()
        self._pose_queue: list[PoseMessage] = []
        self._last_alive_ts_ns: Optional[int] = None
        self._process_handle: object | None = None
        self._started: bool = False

    @property
    def started(self) -> bool:
        return self._started

    @property
    def frame_ring(self) -> Optional[ShmFrameRing]:
        return self._ring

    def start(self, *, config: EngineConfig) -> None:
        """Spawn the vendor binary and complete the handshake.

        In production ``spawn_callable`` is a closure around
        ``PluginContext.process.spawn`` so the supervisor enforces the
        manifest allowlist + cgroup slice. In tests the callable can
        be a stub that returns a mock process handle.
        """

        if self._started:
            return
        self._ring = ShmFrameRing(
            slot_count=self._frame_slot_count,
            slot_size=self._frame_slot_size,
        )
        self._channel = self._open_channel()
        self._spawn_binary()
        self._handshake(config)
        self._started = True

    def stop(self) -> None:
        """Send shutdown, close channels, terminate the binary."""

        if not self._started:
            return
        if self._channel is not None:
            try:
                self._channel.send({"type": "shutdown"})
            except Exception:  # noqa: BLE001 - vendor may be dead already
                pass
            try:
                self._channel.close()
            except Exception:  # noqa: BLE001
                pass
        self._terminate_binary()
        self._channel = None
        self._ring = None
        self._started = False

    def send_imu(
        self,
        *,
        ts_us: int,
        gyro: tuple[float, float, float],
        accel: tuple[float, float, float],
    ) -> None:
        """Forward one IMU sample to the vendor binary."""

        if not self._started or self._channel is None:
            raise VioEngineError("engine not started")
        self._channel.send(
            {
                "type": "imu",
                "ts_us": int(ts_us),
                "gyro": list(gyro),
                "accel": list(accel),
            }
        )

    def send_frame(
        self,
        *,
        ts_us: int,
        width: int,
        height: int,
        stride: int,
        pixel_format: int,
        pixels: bytes,
    ) -> None:
        """Push one frame into the SHM ring and notify the vendor."""

        if not self._started or self._channel is None or self._ring is None:
            raise VioEngineError("engine not started")
        slot = self._ring.write(
            ts_us=ts_us,
            width=width,
            height=height,
            stride=stride,
            pixel_format=pixel_format,
            pixels=pixels,
        )
        self._channel.send(
            {
                "type": "frame_ready",
                "ts_us": slot.ts_us,
                "seq": slot.seq,
                "slot": slot.slot_index,
                "width": slot.width,
                "height": slot.height,
                "stride": slot.stride,
                "format": slot.pixel_format,
            }
        )

    def poll(self) -> None:
        """Drain any pending messages on the channel.

        Pose messages land on the internal queue (see
        :meth:`drain_poses`). Alive messages refresh the watchdog.
        Log messages forward to the Python logger.
        """

        if not self._started or self._channel is None:
            return
        while True:
            try:
                msg = self._channel.recv()
            except (ConnectionError, ValueError):
                msg = None
            if msg is None:
                return
            self._dispatch(msg)

    def drain_poses(self) -> list[PoseMessage]:
        """Return and clear the accumulated pose messages."""

        with self._lock:
            poses = list(self._pose_queue)
            self._pose_queue.clear()
            return poses

    def is_alive(self, *, now_ns: int) -> bool:
        """Returns False when no alive message arrived within the grace.

        The watchdog uses this to decide when to restart the engine.
        Before the first ``alive`` message lands the function returns
        True so a slow-starting vendor is not aborted on its first
        millisecond.
        """

        if self._last_alive_ts_ns is None:
            return True
        delta_s = (now_ns - self._last_alive_ts_ns) / 1e9
        return delta_s <= self._heartbeat_grace_s

    # ------------------------------------------------------------------
    # Subclass hooks
    # ------------------------------------------------------------------

    @abstractmethod
    def _engine_spawn_args(self, *, socket_path: str, shm_name: str) -> list[str]:
        """Return the argv tail passed to ``process.spawn``."""

    def _open_channel(self) -> MsgpackChannel:
        """Open the IPC channel.

        Default uses the injected ``transport_factory`` when present,
        otherwise raises — production wires the factory up to the
        plugin SDK during plugin start.
        """

        if self._transport_factory is None:
            raise VioEngineError(
                "no transport_factory; production must wire ctx.process up"
            )
        transport = self._transport_factory()  # type: ignore[misc]
        return MsgpackChannel(transport)

    def _spawn_binary(self) -> None:
        """Spawn the vendor binary via the injected ``spawn_callable``.

        Default is a no-op so tests can drive the engine without a
        real subprocess; the test fixture sets ``_started`` semantics
        through the handshake instead.
        """

        if self._spawn_callable is None:
            return
        self._process_handle = self._spawn_callable(  # type: ignore[misc]
            self.basename,
            self._engine_spawn_args(
                socket_path="/run/ados/plugins/vio.sock",
                shm_name="/ados_vio_frames",
            ),
        )

    def _terminate_binary(self) -> None:
        handle = self._process_handle
        self._process_handle = None
        if handle is None:
            return
        terminate = getattr(handle, "terminate", None)
        if callable(terminate):
            try:
                terminate()
            except Exception:  # noqa: BLE001 - teardown must not raise
                pass

    def _handshake(self, config: EngineConfig) -> None:
        """Send hello + config; wait for hello_ack."""

        assert self._channel is not None
        self._channel.send(
            {
                "type": "hello",
                "protocol_version": PROTOCOL_VERSION,
                "engine": self.engine_id,
            }
        )
        ack = self._channel.recv()
        if not isinstance(ack, dict) or ack.get("type") != "hello_ack":
            raise VioEngineError(f"unexpected handshake reply: {ack!r}")
        ack_version = ack.get("protocol_version")
        if ack_version != PROTOCOL_VERSION:
            raise VioEngineError(
                f"protocol mismatch: shim={PROTOCOL_VERSION} vendor={ack_version}"
            )
        self._channel.send({"type": "config", **_config_to_wire(config)})

    def _dispatch(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "pose":
            try:
                pose = _pose_from_wire(msg)
            except (KeyError, TypeError, ValueError) as exc:
                log.warning("vio engine pose decode failed: %s", exc)
                return
            with self._lock:
                self._pose_queue.append(pose)
            return
        if kind == "alive":
            import time

            self._last_alive_ts_ns = time.monotonic_ns()
            return
        if kind == "log":
            level = msg.get("level", "info")
            text = msg.get("msg", "")
            if level == "error":
                log.error("vio engine: %s", text)
            elif level == "warn":
                log.warning("vio engine: %s", text)
            else:
                log.info("vio engine: %s", text)
            return
        log.debug("vio engine unknown message: %s", kind)


class OpenVinsEngine(BaseVioEngine):
    """OpenVINS (MSCKF) implementation.

    Spawns ``ados_openvins_shim`` from the manifest allowlist. The
    binary links ``libov_msckf_lib.so`` and runs the OpenVINS
    ``VioManager`` loop. Default for low-cost NPU boards.
    """

    engine_id = "openvins"
    basename = "ados_openvins_shim"

    def _engine_spawn_args(self, *, socket_path: str, shm_name: str) -> list[str]:
        return [
            "--socket",
            socket_path,
            "--shm",
            shm_name,
            "--slot-count",
            str(self._frame_slot_count),
            "--slot-size",
            str(self._frame_slot_size),
        ]


class VinsFusionEngine(BaseVioEngine):
    """VINS-Fusion (sliding-window BA) implementation.

    Spawns ``ados_vins_fusion_shim``. Higher CPU than OpenVINS;
    advanced option for boards with the headroom (Rock 5C Lite,
    CM4-class).
    """

    engine_id = "vins-fusion"
    basename = "ados_vins_fusion_shim"

    def _engine_spawn_args(self, *, socket_path: str, shm_name: str) -> list[str]:
        return [
            "--socket",
            socket_path,
            "--shm",
            shm_name,
            "--slot-count",
            str(self._frame_slot_count),
            "--slot-size",
            str(self._frame_slot_size),
        ]


# ----------------------------------------------------------------------
# Wire helpers
# ----------------------------------------------------------------------


def _config_to_wire(config: EngineConfig) -> dict:
    return {
        "intrinsics": {
            "model": f"{config.camera_model}-{config.distortion_model}",
            "fx": config.fx,
            "fy": config.fy,
            "cx": config.cx,
            "cy": config.cy,
            "width": config.width,
            "height": config.height,
            "distortion_coeffs": list(config.distortion_coeffs),
        },
        "extrinsics": {"T_cam_imu": list(config.t_cam_imu)},
        "timeshift_cam_imu_s": config.timeshift_cam_imu_s,
        "imu_rate_hz": config.imu_rate_hz,
        "camera_rate_hz": config.camera_rate_hz,
    }


def _pose_from_wire(msg: dict) -> PoseMessage:
    state = msg.get("state", "init")
    if state not in (
        "init",
        "converging",
        "converged",
        "degraded",
        "failed",
    ):
        raise ValueError(f"unknown pose state {state!r}")
    pos = tuple(float(x) for x in msg["position"])
    quat = tuple(float(x) for x in msg["orientation_quat"])
    vel = tuple(float(x) for x in msg["velocity"])
    cov = tuple(float(x) for x in msg.get("covariance", []))
    if len(pos) != 3 or len(vel) != 3 or len(quat) != 4:
        raise ValueError("pose vectors have wrong arity")
    return PoseMessage(
        ts_us=int(msg["ts_us"]),
        position=pos,  # type: ignore[arg-type]
        orientation_quat=quat,  # type: ignore[arg-type]
        velocity=vel,  # type: ignore[arg-type]
        covariance=cov,
        feature_count=int(msg.get("feature_count", 0)),
        reset_counter=int(msg.get("reset_counter", 0)),
        state=state,
    )
