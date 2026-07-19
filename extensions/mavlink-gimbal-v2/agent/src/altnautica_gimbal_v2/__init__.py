"""MAVLink Gimbal v2 Controller plugin (agent half).

Public re-exports cover the MAVLink ``GimbalDriver`` and the plugin entry
point. The driver talks the open MAVLink Gimbal Manager Protocol v2.
"""

from __future__ import annotations

__version__ = "1.3.0"

from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver
from altnautica_gimbal_v2.plugin import GimbalV2Plugin

__all__ = [
    "GimbalV2Plugin",
    "MavlinkGimbalDriver",
    "__version__",
]
