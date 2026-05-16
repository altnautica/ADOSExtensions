"""Hardware auto-detect package.

The vision-nav plugin used to require every camera path, rangefinder
driver, and IMU source to be set in static config. That is fine for a
bench rig where the operator already knows what is wired up; it is
the wrong default for an OEM ship where the plugin should adapt to
whatever hardware the host actually has.

The submodules here probe each subsystem and produce a typed result
the plugin merges into its live config before opening capture, the
rangefinder, and the IMU source. The submodules are intentionally
side-effect-free except for the underlying device I/O so unit tests
can mock the bus access via the standard library facades.

Each detector returns ``None`` (or an empty list / a typed ``DetectionResult``
with an ``is_present=False`` flag) when the hardware is absent. The
plugin folds the results into the heartbeat so the GCS can render the
detected topology and the suggested mode.
"""

from __future__ import annotations

from altnautica_vision_nav.autodetect.camera import (
    DetectedCamera,
    detect_cameras,
)
from altnautica_vision_nav.autodetect.imu import (
    detect_preferred_imu_source,
)
from altnautica_vision_nav.autodetect.mode import (
    SuggestedMode,
    derive_suggested_mode,
)
from altnautica_vision_nav.autodetect.profile import (
    HostProfile,
    detect_host_profile,
    is_drone_profile,
)
from altnautica_vision_nav.autodetect.rangefinder import (
    DetectedRangefinder,
    detect_rangefinder,
)


__all__ = [
    "DetectedCamera",
    "DetectedRangefinder",
    "HostProfile",
    "SuggestedMode",
    "derive_suggested_mode",
    "detect_cameras",
    "detect_host_profile",
    "detect_preferred_imu_source",
    "detect_rangefinder",
    "is_drone_profile",
]
