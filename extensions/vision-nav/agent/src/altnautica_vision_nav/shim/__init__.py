"""Python shim layer for the VIO vendor binaries.

The shim is the bridge between the plugin's :class:`VioEstimator` and
the spawned vendor binary (``ados_openvins_shim`` or
``ados_vins_fusion_shim``). The vendor binary owns the actual MSCKF or
bundle-adjustment work; the shim owns the IPC plumbing, the watchdog,
and the translation between the plugin's Python types and the
binary's msgpack wire format.

Two transports are used in parallel:

* a shared-memory ring buffer for camera frames (zero-copy)
* a Unix-domain SOCK_SEQPACKET (or SOCK_STREAM with length-prefix
  fallback) for IMU samples, pose returns, config, heartbeat, log
"""

from __future__ import annotations

from altnautica_vision_nav.shim.engine import (
    BaseVioEngine,
    EngineConfig,
    PoseMessage,
    VioEngineError,
)
from altnautica_vision_nav.shim.heartbeat import HeartbeatWatcher
from altnautica_vision_nav.shim.ipc import (
    FrameSlot,
    MsgpackChannel,
    ShmFrameRing,
)

__all__ = [
    "BaseVioEngine",
    "EngineConfig",
    "FrameSlot",
    "HeartbeatWatcher",
    "MsgpackChannel",
    "PoseMessage",
    "ShmFrameRing",
    "VioEngineError",
]
