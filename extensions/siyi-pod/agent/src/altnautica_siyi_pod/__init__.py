"""ADOS SIYI optical-pod driver (agent half).

Drives the SIYI optical-pod line (A2 mini, A8 mini, ZR10, ZR30, ZT6, ZT30)
over the SIYI Gimbal Camera External SDK. One transport session backs a
capability-negotiated facade that exposes gimbal, zoom, focus, photo/record,
thermal palette and gain, and laser-rangefinder control, and mirrors gimbal
attitude plus the laser range up to the flight controller over MAVLink.
"""

from __future__ import annotations

__version__ = "0.5.0"

PLUGIN_ID = "com.altnautica.siyi-pod"

__all__ = ["PLUGIN_ID", "__version__"]
