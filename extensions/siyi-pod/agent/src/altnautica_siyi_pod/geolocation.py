"""Turn a laser range into a subject ground position.

When the pod fires its rangefinder it returns the slant range to whatever the
gimbal is pointed at. Combined with the gimbal pointing angles and the
aircraft's position, that measured range fixes the subject in the world: the
boresight ray is projected out by the measured range (no flat-ground-plane
assumption, so it works pointing at a hillside, a rooftop, or the ground
equally).

Frame model: a SIYI pod stabilises its own camera, so the pitch it reports is
referenced to the horizon and the yaw is referenced to the aircraft body. The
earth-frame bearing is therefore the aircraft heading plus the gimbal yaw, and
the depression is the gimbal pitch directly. Aircraft roll/pitch are absorbed by
the gimbal's stabilisation and are not re-applied here. This is a spot-ranging
geolocation for map marking and hand-off, not an EKF input; its accuracy is
bounded by the time alignment of the range sample with the pose and by the
gimbal-attitude calibration, which is why the result carries the inputs it was
computed from.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Metres per degree of latitude (spherical-earth approximation, good to well
# under the pod's own pointing error at these ranges).
_M_PER_DEG_LAT = 111320.0


@dataclass(frozen=True)
class GroundTarget:
    """A geolocated subject position and the geometry it came from."""

    lat_deg: float
    lon_deg: float
    rel_alt_m: float
    slant_range_m: float
    bearing_deg: float
    depression_deg: float
    ground_range_m: float


def geolocate(
    *,
    vehicle_lat_deg: float,
    vehicle_lon_deg: float,
    vehicle_rel_alt_m: float,
    vehicle_yaw_deg: float,
    gimbal_yaw_deg: float,
    gimbal_pitch_deg: float,
    slant_range_m: float,
) -> GroundTarget:
    """Project the gimbal boresight out by the measured slant range."""
    bearing_deg = (vehicle_yaw_deg + gimbal_yaw_deg) % 360.0
    bearing = math.radians(bearing_deg)
    pitch = math.radians(gimbal_pitch_deg)  # negative looks down

    horizontal = slant_range_m * math.cos(pitch)
    north_m = horizontal * math.cos(bearing)
    east_m = horizontal * math.sin(bearing)
    down_m = -slant_range_m * math.sin(pitch)  # pitch<0 -> down>0

    d_lat_deg = north_m / _M_PER_DEG_LAT
    cos_lat = math.cos(math.radians(vehicle_lat_deg))
    # Guard the pole singularity so a bad pose never divides by zero.
    d_lon_deg = east_m / (_M_PER_DEG_LAT * cos_lat) if abs(cos_lat) > 1e-6 else 0.0

    return GroundTarget(
        lat_deg=vehicle_lat_deg + d_lat_deg,
        lon_deg=vehicle_lon_deg + d_lon_deg,
        rel_alt_m=vehicle_rel_alt_m - down_m,
        slant_range_m=slant_range_m,
        bearing_deg=bearing_deg,
        depression_deg=-gimbal_pitch_deg,
        ground_range_m=abs(horizontal),
    )
