"""MAVLink-derived scale ladder (rangefinder-free).

Subscribes to ``GLOBAL_POSITION_INT``, ``VFR_HUD``, ``GPS_RAW_INT`` and
walks the ladder on every :meth:`pick` call. Staleness is enforced
per-source. The take-off altitude is captured the first time
``VFR_HUD.alt`` arrives so the raw baro fallback can subtract it and
report AGL rather than AMSL.

Quality multipliers per the rangefinder-free scale research:

* ``GLOBAL_POSITION_INT.relative_alt`` — 0.7 (best, baro-EKF-fused)
* ``VFR_HUD.alt`` raw baro — 0.6 (still baro, but unfiltered)
* ``GPS_RAW_INT.alt`` — 0.4 (gated on outdoor flag + 3D fix + HDOP)
* static fallback — 0.2 (critical; GCS surfaces a red banner)

Altitude is clamped to ``[0.3, 50.0]`` m so a stale or pathological
reading does not produce a runaway scale factor.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional, Tuple

from altnautica_vision_nav.scale.base import BaseScaleSource, ScalePick

log = logging.getLogger(__name__)

STALE_THRESHOLD_NS = 2_000_000_000  # 2 seconds
MIN_DISTANCE_M = 0.3
MAX_DISTANCE_M = 50.0
DEFAULT_STATIC_FALLBACK_M = 1.5

QM_RELATIVE_ALT = 0.7
QM_RAW_BARO = 0.6
QM_GPS = 0.4
QM_STATIC = 0.2

# GPS gates (per research): 3D fix or better, horizontal accuracy
# better than ~2 m. ``eph`` is in centimetres on the wire; 200 cm = 2 m.
GPS_FIX_TYPE_3D = 3
GPS_EPH_MAX_CM = 200


class MavlinkScaleLadder(BaseScaleSource):
    """Walk the baro -> GPS -> static ladder for rangefinder-free OF.

    ``outdoor_flag`` gates the GPS rung. The GCS exposes an explicit
    operator toggle for this; an auto-detect heuristic on satellite
    count + HDOP is a later polish item left out of v1.
    """

    def __init__(
        self,
        ctx: Any,
        *,
        outdoor_flag: bool = False,
        static_fallback_m: float = DEFAULT_STATIC_FALLBACK_M,
    ) -> None:
        self._ctx = ctx
        self._outdoor = bool(outdoor_flag)
        self._static_fallback_m = float(static_fallback_m)
        self._lock = threading.Lock()
        # Each rung carries (value, monotonic_ns_at_receive). Values
        # are in metres for relative_alt and vfr_alt; gps_alt is also
        # metres after the AGL subtraction.
        self._relative_alt: Optional[Tuple[float, int]] = None
        self._vfr_alt: Optional[Tuple[float, int]] = None
        self._gps_alt: Optional[Tuple[float, int]] = None
        self._takeoff_alt: Optional[float] = None
        self._gps_meta: Optional[Tuple[int, int]] = None  # (fix_type, eph_cm)
        self._unsubs: list[Callable[[], None]] = []

    @property
    def outdoor_flag(self) -> bool:
        return self._outdoor

    def set_outdoor_flag(self, value: bool) -> None:
        """GCS toggle: enables / disables the GPS rung."""

        self._outdoor = bool(value)

    def start(self) -> None:
        if self._unsubs:
            return
        sub = self._ctx.mavlink.subscribe
        self._unsubs.append(
            sub("GLOBAL_POSITION_INT", self._on_global_position)
        )
        self._unsubs.append(sub("VFR_HUD", self._on_vfr_hud))
        self._unsubs.append(sub("GPS_RAW_INT", self._on_gps_raw))

    def stop(self) -> None:
        for unsub in self._unsubs:
            try:
                unsub()
            except Exception:  # noqa: BLE001 - teardown must not raise
                log.warning(
                    "scale ladder unsubscribe raised; ignoring",
                    exc_info=True,
                )
        self._unsubs.clear()

    def pick(self) -> Optional[ScalePick]:
        """Walk the ladder; return the first healthy rung or the static fallback."""

        now = time.monotonic_ns()

        with self._lock:
            relative = self._relative_alt
            vfr = self._vfr_alt
            gps = self._gps_alt
            gps_meta = self._gps_meta
            outdoor = self._outdoor

        # Rung 1: relative_alt (baro-EKF altitude, AGL from arm origin).
        if relative is not None and (now - relative[1]) <= STALE_THRESHOLD_NS:
            return ScalePick(
                distance_m=_clamp(relative[0]),
                source="baro",
                quality_multiplier=QM_RELATIVE_ALT,
            )

        # Rung 2: VFR_HUD raw baro minus captured takeoff alt.
        if vfr is not None and (now - vfr[1]) <= STALE_THRESHOLD_NS:
            return ScalePick(
                distance_m=_clamp(vfr[0]),
                source="baro",
                quality_multiplier=QM_RAW_BARO,
            )

        # Rung 3: GPS alt minus takeoff, gated on outdoor + fix + hdop.
        if (
            outdoor
            and gps is not None
            and (now - gps[1]) <= STALE_THRESHOLD_NS
            and gps_meta is not None
            and gps_meta[0] >= GPS_FIX_TYPE_3D
            and gps_meta[1] <= GPS_EPH_MAX_CM
        ):
            return ScalePick(
                distance_m=_clamp(gps[0]),
                source="gps",
                quality_multiplier=QM_GPS,
            )

        # Rung 4: static fallback. Always returns something so the
        # estimator can emit at the lowest quality rather than refusing
        # to feed the EKF entirely; the GCS surfaces the red banner.
        return ScalePick(
            distance_m=_clamp(self._static_fallback_m),
            source="static",
            quality_multiplier=QM_STATIC,
        )

    def _on_global_position(self, data: Any) -> None:
        relative_alt_mm = _field(data, "relative_alt")
        if relative_alt_mm is None:
            return
        metres = float(relative_alt_mm) / 1000.0
        with self._lock:
            self._relative_alt = (metres, time.monotonic_ns())

    def _on_vfr_hud(self, data: Any) -> None:
        alt_m = _field(data, "alt")
        if alt_m is None:
            return
        alt_m = float(alt_m)
        with self._lock:
            if self._takeoff_alt is None:
                # First sample becomes the take-off altitude reference
                # so the raw-baro rung reports AGL from arm rather than
                # AMSL. Subsequent samples are deltas against this.
                self._takeoff_alt = alt_m
                metres = 0.0
            else:
                metres = alt_m - self._takeoff_alt
            self._vfr_alt = (metres, time.monotonic_ns())

    def _on_gps_raw(self, data: Any) -> None:
        alt_mm = _field(data, "alt")
        fix_type = _field(data, "fix_type")
        eph = _field(data, "eph")
        if alt_mm is None or fix_type is None or eph is None:
            return
        alt_m = float(alt_mm) / 1000.0
        with self._lock:
            takeoff = self._takeoff_alt
            metres = alt_m - takeoff if takeoff is not None else alt_m
            self._gps_alt = (metres, time.monotonic_ns())
            self._gps_meta = (int(fix_type), int(eph))


def _clamp(value: float) -> float:
    return max(MIN_DISTANCE_M, min(MAX_DISTANCE_M, float(value)))


def _field(data: Any, name: str) -> Optional[Any]:
    if isinstance(data, dict):
        return data.get(name)
    return getattr(data, name, None)
