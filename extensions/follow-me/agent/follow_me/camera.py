"""The follow camera's geometry and the instant its frames were exposed.

The vision engine stamps each detection batch with the camera roster's
horizontal field of view (``hfov_deg``) and mount pitch (``mount_pitch_deg``,
negative below the nose), and with the frame's sensor-clock time
(``ts_sensor_ms``) when the camera has one. Each is optional: an older agent
sends none of them, and a camera without a roster entry sends no geometry.

Each geometry field is taken from the batch when it carries one, else from
the operator's ``camera_hfov_deg`` / ``mount_pitch_deg`` settings, else from
the built-in reference camera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

# The reference airframe's front camera: 87° wide, 15° below the nose. Used
# only where neither the batches nor the settings give a value.
DEFAULT_HFOV_DEG = 87.0
DEFAULT_MOUNT_PITCH_DEG = -15.0

GEOMETRY_ROSTER = "roster"
GEOMETRY_SETTINGS = "settings"
GEOMETRY_DEFAULT = "default"

# A geometry with fields from more than one source reports the least
# authoritative of them.
_SOURCE_RANK = {GEOMETRY_DEFAULT: 0, GEOMETRY_SETTINGS: 1, GEOMETRY_ROSTER: 2}

# A sensor stamp further than this from the host stamp mapped onto the
# flight-controller clock is on some other clock and is not used.
SENSOR_STAMP_MAX_SKEW_S = 1.0


@dataclass(frozen=True)
class CameraGeometry:
    """Horizontal field of view and mount pitch, in the roster's sign
    (negative below the nose), and where they came from."""

    hfov_deg: float = DEFAULT_HFOV_DEG
    mount_pitch_deg: float = DEFAULT_MOUNT_PITCH_DEG
    source: str = GEOMETRY_DEFAULT

    @property
    def mount_down_deg(self) -> float:
        """The mount tilt in the projection's sign: positive below the
        horizon."""
        return -self.mount_pitch_deg


DEFAULT_GEOMETRY = CameraGeometry()


def _finite(v: Any) -> float | None:
    if v is None or isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _hfov(v: Any) -> float | None:
    f = _finite(v)
    return f if f is not None and 1.0 <= f <= 179.0 else None


def _pitch(v: Any) -> float | None:
    f = _finite(v)
    return f if f is not None and -90.0 <= f <= 90.0 else None


def _weakest(a: str, b: str) -> str:
    return a if _SOURCE_RANK[a] <= _SOURCE_RANK[b] else b


def configured_geometry(cfg: Any) -> CameraGeometry:
    """The geometry the operator's settings give (``cfg`` a FollowConfig, or
    None before the config is read). The ``mount_pitch_deg`` setting is the
    tilt below the horizon, positive down. Each field unset or out of range
    takes the default."""
    hfov = _hfov(getattr(cfg, "camera_hfov_deg", None))
    down = _pitch(getattr(cfg, "mount_pitch_deg", None))
    return CameraGeometry(
        hfov_deg=DEFAULT_HFOV_DEG if hfov is None else hfov,
        mount_pitch_deg=DEFAULT_MOUNT_PITCH_DEG if down is None else -down,
        source=_weakest(
            GEOMETRY_DEFAULT if hfov is None else GEOMETRY_SETTINGS,
            GEOMETRY_DEFAULT if down is None else GEOMETRY_SETTINGS,
        ),
    )


def geometry_of(batch: Any, fallback: CameraGeometry = DEFAULT_GEOMETRY) -> CameraGeometry:
    """The camera geometry for a batch (None for no batch): each field the
    batch carries in range, else the fallback's. The source is the least
    authoritative one a field came from."""
    hfov = _hfov(getattr(batch, "hfov_deg", None))
    pitch = _pitch(getattr(batch, "mount_pitch_deg", None))
    if hfov is None and pitch is None:
        return fallback
    return CameraGeometry(
        hfov_deg=fallback.hfov_deg if hfov is None else hfov,
        mount_pitch_deg=fallback.mount_pitch_deg if pitch is None else pitch,
        source=_weakest(
            fallback.source if hfov is None else GEOMETRY_ROSTER,
            fallback.source if pitch is None else GEOMETRY_ROSTER,
        ),
    )


def sensor_ts_ms(batch: Any) -> int | None:
    """The frame's sensor-clock stamp, or None."""
    v = _finite(getattr(batch, "ts_sensor_ms", None))
    return None if v is None or v <= 0 else int(v)


def exposure_fc_s(
    ts_ms: int, ts_sensor_ms: int | None, offset_s: float | None
) -> float | None:
    """A frame's exposure on the flight-controller clock, seconds.

    ``ts_ms`` is the host monotonic stamp; ``offset_s`` the host-minus-FC
    clock offset (None while unknown). The sensor stamp is used when it lies
    within :data:`SENSOR_STAMP_MAX_SKEW_S` of the host stamp mapped onto the
    FC clock (a simulator camera stamps its frames with the FC's time and its
    host stamp is the frame's late arrival); otherwise the mapped host stamp.
    """
    if offset_s is None or ts_ms <= 0:
        return None
    mapped = ts_ms / 1000.0 - offset_s
    if ts_sensor_ms is not None:
        sensor = ts_sensor_ms / 1000.0
        if abs(sensor - mapped) <= SENSOR_STAMP_MAX_SKEW_S:
            return sensor
    return mapped


__all__ = [
    "DEFAULT_HFOV_DEG",
    "DEFAULT_MOUNT_PITCH_DEG",
    "DEFAULT_GEOMETRY",
    "GEOMETRY_ROSTER",
    "GEOMETRY_SETTINGS",
    "GEOMETRY_DEFAULT",
    "CameraGeometry",
    "configured_geometry",
    "exposure_fc_s",
    "geometry_of",
    "sensor_ts_ms",
]
