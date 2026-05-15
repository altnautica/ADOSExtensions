"""ArduPilot vision-only pre-arm helper.

When a drone is flying purely on optical flow, the firmware refuses to
arm without a global origin. This helper pushes a synthetic origin via
the two messages ArduPilot accepts:

* ``SET_GPS_GLOBAL_ORIGIN`` (#48)  -- seeds the EKF's lat/lon/alt.
* ``SET_HOME_POSITION`` (#243)     -- declares the same point as
  "home" so RTL has a target and the GCS can render the home marker.

Both messages are fire-and-forget; ``GCS_MAVLINK::handle_set_gps_global_origin``
accepts the message and does not ACK it. The helper logs each send so
operators can confirm the pre-arm step happened.
"""

from __future__ import annotations

import asyncio
import itertools
import logging
import struct
from typing import Final, Sequence

from altnautica_vision_nav.mavlink._framing import pack_v2

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Message ids + CRC_EXTRA fingerprints (pymavlink common.xml, verified).
# ---------------------------------------------------------------------------

MSG_ID_SET_GPS_GLOBAL_ORIGIN: Final[int] = 48
CRC_SET_GPS_GLOBAL_ORIGIN: Final[int] = 41

MSG_ID_SET_HOME_POSITION: Final[int] = 243
CRC_SET_HOME_POSITION: Final[int] = 85


# Wire-order packers (largest-first MAVLink v2 layout, extension fields
# such as ``time_usec`` are intentionally omitted because we send zero
# and the v2 truncation rule strips trailing zero bytes).
_PACK_SET_GPS_GLOBAL_ORIGIN: Final[struct.Struct] = struct.Struct("<iiiB")
_PACK_SET_HOME_POSITION: Final[struct.Struct] = struct.Struct(
    "<iiiffffffffffB"
)


DEFAULT_SYS_ID: Final[int] = 1
DEFAULT_COMP_ID: Final[int] = 198


def _ed7(degrees: float) -> int:
    """Convert WGS-84 degrees to MAVLink ``degE7`` (int32 × 10^-7)."""
    return int(round(float(degrees) * 1e7))


def _alt_mm(metres: float) -> int:
    """Convert altitude in metres to MAVLink millimetres (int32)."""
    return int(round(float(metres) * 1000.0))


def build_set_gps_global_origin(
    *,
    sys_id: int,
    comp_id: int,
    seq: int,
    target_system: int,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
) -> bytes:
    """Pack a SET_GPS_GLOBAL_ORIGIN (#48) v2 frame."""

    payload = _PACK_SET_GPS_GLOBAL_ORIGIN.pack(
        _ed7(lat_deg),
        _ed7(lon_deg),
        _alt_mm(alt_m),
        int(target_system) & 0xFF,
    )
    return pack_v2(
        msg_id=MSG_ID_SET_GPS_GLOBAL_ORIGIN,
        crc_extra=CRC_SET_GPS_GLOBAL_ORIGIN,
        payload=payload,
        sys_id=sys_id,
        comp_id=comp_id,
        seq=seq,
    )


def build_set_home_position(
    *,
    sys_id: int,
    comp_id: int,
    seq: int,
    target_system: int,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    approach_vector: Sequence[float] = (0.0, 0.0, 0.0),
) -> bytes:
    """Pack a SET_HOME_POSITION (#243) v2 frame.

    The quaternion defaults to identity (level home attitude) and the
    approach vector defaults to zero (no preferred final approach).
    """

    if len(quaternion) != 4:
        raise ValueError(
            f"quaternion must have length 4, got {len(quaternion)}"
        )
    if len(approach_vector) != 3:
        raise ValueError(
            f"approach_vector must have length 3, got {len(approach_vector)}"
        )
    payload = _PACK_SET_HOME_POSITION.pack(
        _ed7(lat_deg),
        _ed7(lon_deg),
        _alt_mm(alt_m),
        0.0,  # x (local NED)
        0.0,  # y
        0.0,  # z
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
        float(approach_vector[0]),
        float(approach_vector[1]),
        float(approach_vector[2]),
        int(target_system) & 0xFF,
    )
    return pack_v2(
        msg_id=MSG_ID_SET_HOME_POSITION,
        crc_extra=CRC_SET_HOME_POSITION,
        payload=payload,
        sys_id=sys_id,
        comp_id=comp_id,
        seq=seq,
    )


class PreArmHelper:
    """Fire SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION pairs on demand."""

    def __init__(
        self,
        *,
        component_id: int = DEFAULT_COMP_ID,
        sys_id: int = DEFAULT_SYS_ID,
    ) -> None:
        self._component_id = int(component_id)
        self._sys_id = int(sys_id)
        self._seq_it = itertools.count()

    async def auto_set_origin(
        self,
        ctx: object,
        lat: float,
        lon: float,
        alt_m: float,
        *,
        target_system: int = 1,
    ) -> None:
        """Send SET_GPS_GLOBAL_ORIGIN followed by SET_HOME_POSITION.

        Both messages are fire-and-forget; ArduPilot accepts them
        without sending a MAVLink ACK. The helper logs each send via
        ``ctx.log`` (when present) and the module logger otherwise.
        """

        origin_frame = build_set_gps_global_origin(
            sys_id=self._sys_id,
            comp_id=self._component_id,
            seq=self._next_seq(),
            target_system=target_system,
            lat_deg=lat,
            lon_deg=lon,
            alt_m=alt_m,
        )
        await self._send(ctx, origin_frame, label="set_gps_global_origin")
        self._info(
            ctx,
            "vision_nav_pre_arm_origin_sent",
            target_system=target_system,
            lat=lat,
            lon=lon,
            alt_m=alt_m,
        )

        home_frame = build_set_home_position(
            sys_id=self._sys_id,
            comp_id=self._component_id,
            seq=self._next_seq(),
            target_system=target_system,
            lat_deg=lat,
            lon_deg=lon,
            alt_m=alt_m,
        )
        await self._send(ctx, home_frame, label="set_home_position")
        self._info(
            ctx,
            "vision_nav_pre_arm_home_sent",
            target_system=target_system,
            lat=lat,
            lon=lon,
            alt_m=alt_m,
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _next_seq(self) -> int:
        return next(self._seq_it) & 0xFF

    async def _send(self, ctx: object, frame: bytes, *, label: str) -> None:
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            log.warning("pre-arm %s dropped: ctx.mavlink missing", label)
            return
        send = getattr(mavlink, "send", None)
        if send is None:
            log.warning("pre-arm %s dropped: ctx.mavlink.send missing", label)
            return
        try:
            result = send(frame, component_id=self._component_id)
            if asyncio.iscoroutine(result):
                await result
        except Exception as exc:  # noqa: BLE001
            log.warning("pre-arm %s send failed: %s", label, exc)

    def _info(self, ctx: object, event: str, **fields: object) -> None:
        plugin_log = getattr(ctx, "log", None)
        if plugin_log is not None:
            info = getattr(plugin_log, "info", None)
            if callable(info):
                try:
                    info(event, **fields)
                    return
                except TypeError:
                    # stdlib logger: positional message only.
                    try:
                        info("%s %s", event, fields)
                        return
                    except Exception:  # noqa: BLE001
                        pass
                except Exception:  # noqa: BLE001
                    pass
        log.info("%s %s", event, fields)
