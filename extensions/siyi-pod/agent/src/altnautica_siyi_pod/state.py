"""The pod telemetry read-back the GCS renders.

Published on the ``siyi.pod.state`` event and mirrored onto the agent heartbeat
via ``ctx.telemetry.extend`` so the GCS console, cockpit panel, and video overlay
render live pod state (model, capabilities, attitude, zoom, range, temperatures,
link health). The GCS gates its controls on the ``capabilities`` block, so one
payload drives every model.
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
    # Which sensor source each physical leg (main/sub) carries, from the
    # resolved model's stream layout.
    assignment: dict[str, str] = field(default_factory=dict)
    # Live readings (None until first read / for unsupported sensors).
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    zoom: float | None = None
    gimbal_mode: str = "follow"
    palette: int | None = None
    recording: bool = False
    laser_range_m: float | None = None
    spot_temp_c: float | None = None
    # Link health: frames seen, and whether the pod answered the last poll. A
    # transport that is up but has stopped answering reads as not ok.
    link_ok: bool = False
    frames_received: int = 0

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
