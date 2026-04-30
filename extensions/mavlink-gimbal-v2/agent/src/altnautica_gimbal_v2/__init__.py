"""MAVLink Gimbal v2 Controller plugin (agent half).

Public re-exports cover the four ``GimbalDriver`` subclasses plus the
plugin entry point. Concrete drivers live in the per-protocol modules.
"""

from __future__ import annotations

__version__ = "1.0.0"

from altnautica_gimbal_v2.gremsy_driver import GremsyGimbalDriver
from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver
from altnautica_gimbal_v2.plugin import GimbalV2Plugin
from altnautica_gimbal_v2.sbgc_driver import SimpleBgcGimbalDriver
from altnautica_gimbal_v2.storm32_driver import Storm32NtGimbalDriver

__all__ = [
    "GimbalV2Plugin",
    "GremsyGimbalDriver",
    "MavlinkGimbalDriver",
    "SimpleBgcGimbalDriver",
    "Storm32NtGimbalDriver",
    "__version__",
]
