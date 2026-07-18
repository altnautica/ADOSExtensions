"""The pod telemetry read-back the GCS renders.

Published on the ``siyi.pod.state`` event and mirrored onto the agent heartbeat
via ``ctx.telemetry.extend`` so the GCS console, cockpit panel, and video overlay
render live pod state (model, capabilities, attitude, zoom, range, temperatures,
tracker, link health). The GCS gates its controls on the ``capabilities`` block,
so one payload drives every model.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass
class PodState:
    """A snapshot of the pod the GCS renders. All fields JSON-serialisable."""

    model: str = "Unknown SIYI pod"
    known: bool = False
    connected: bool = False
    firmware: str | None = None
    # Which controls to show, mirrored from the negotiated capability profile.
    capabilities: dict[str, object] = field(default_factory=dict)
    # Live readings (None until first read / for unsupported sensors).
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    zoom: float | None = None
    sensor_mode: str = "eo"
    gimbal_mode: str = "follow"
    palette: int | None = None
    recording: bool = False
    laser_range_m: float | None = None
    spot_temp_c: float | None = None
    track_active: bool = False
    track_id: int | None = None
    # Link health (Rule 37): frames seen and whether the pod is answering.
    link_ok: bool = False
    frames_received: int = 0

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
