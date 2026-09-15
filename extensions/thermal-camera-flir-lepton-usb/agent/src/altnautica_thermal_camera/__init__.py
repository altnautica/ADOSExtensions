"""Agent half of the ADOS Thermal Camera FLIR Lepton USB UVC extension.

The plugin entry point is :class:`altnautica_thermal_camera.plugin.ThermalUsbPlugin`.
The driver implementation lives in :mod:`altnautica_thermal_camera.driver` and
talks to a UVC backend behind the :class:`LibUvcBackend` Protocol. No
concrete backend ships in this package: the host injects one satisfying the
Protocol, and without it the plugin reports an explicit unavailable state
instead of capturing.
"""

from altnautica_thermal_camera.driver import LeptonUvcDriver, LeptonUvcSession
from altnautica_thermal_camera.palettes import (
    PALETTES,
    apply_palette,
    list_palettes,
    palette_lut,
)
from altnautica_thermal_camera.tlinear import (
    DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
    celsius_from_y16,
    kelvin_from_y16,
    y16_from_celsius,
    y16_from_kelvin,
)
from altnautica_thermal_camera.uvc_backend import (
    LibUvcBackend,
    UvcDeviceInfo,
    UvcFrame,
)

__all__ = [
    "LeptonUvcDriver",
    "LeptonUvcSession",
    "LibUvcBackend",
    "UvcDeviceInfo",
    "UvcFrame",
    "PALETTES",
    "apply_palette",
    "list_palettes",
    "palette_lut",
    "DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT",
    "celsius_from_y16",
    "kelvin_from_y16",
    "y16_from_celsius",
    "y16_from_kelvin",
]

__version__ = "1.3.0"
