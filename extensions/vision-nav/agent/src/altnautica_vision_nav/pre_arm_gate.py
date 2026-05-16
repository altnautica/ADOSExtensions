"""Vision-navigation pre-arm gate evaluator.

The gate runs every heartbeat tick and reports a structured set of
checks the GCS pre-arm card consumes. Each check returns ok / pending
/ blocking with an operator-readable reason; the aggregate ``armable``
flag is true only when every required check is ok.

Mode-specific rules:

* ``off`` — no checks fire; the gate reports armable as a passthrough.
* ``optical_flow`` — companion active, flow quality above gate,
  rangefinder healthy.
* ``optical_flow_degraded`` — companion active, flow quality above
  gate, ANY scale source healthy (rangefinder optional).
* ``vio_openvins`` / ``vio_vins_fusion`` — companion active,
  estimator converged, intrinsics + extrinsics loaded, sync offset
  within the red threshold, feature count above the minimum.
* ``hybrid_of_plus_vio`` — VIO checks AND OF checks (both inputs
  must be healthy before arm).

This module is intentionally pure: no MAVLink, no telemetry, no
async. The plugin calls :meth:`PreArmGate.evaluate` with the current
state and serialises the result onto the heartbeat. Tests can drive
every branch synchronously with stub inputs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

DEFAULT_FLOW_QUALITY_GATE = 50
DEFAULT_VIO_FEATURE_FLOOR = 20
# Per the camera-IMU time-sync research: green ≤10 ms, yellow ≤30 ms,
# red >30 ms. The pre-arm gate refuses to arm in the red band; the
# yellow band passes the gate but the GCS sensors card raises a
# warning banner.
SYNC_OFFSET_RED_MS = 30.0


@dataclass(frozen=True)
class PreArmCheck:
    """One pre-arm check result.

    ``id`` is a stable machine-readable key the GCS uses to render the
    row icon + i18n string. ``severity`` is "ok" / "pending" /
    "blocking" — pending means the check has not produced an answer
    yet (e.g. estimator still initialising) and is not strictly
    blocking but is not yet green either.
    """

    id: str
    severity: str  # "ok" | "pending" | "blocking"
    detail: str = ""


@dataclass(frozen=True)
class PreArmReport:
    """Aggregate pre-arm state for the heartbeat.

    ``armable`` is true iff every check is ``"ok"``. ``checks`` is
    the full list the GCS renders; ``mode`` echoes the active
    estimator mode for tooltip clarity. The current rangefinder
    policy treats a missing rangefinder as blocking in plain optical
    flow but not in the degraded path.
    """

    mode: str
    armable: bool
    checks: tuple[PreArmCheck, ...] = field(default_factory=tuple)

    def as_dict(self) -> dict:
        """Wire-friendly dict for the navigation heartbeat block."""

        return {
            "mode": self.mode,
            "armable": self.armable,
            "checks": [
                {"id": c.id, "severity": c.severity, "detail": c.detail}
                for c in self.checks
            ],
        }


@dataclass
class PreArmInputs:
    """Snapshot of the inputs the gate consults.

    Constructed by the plugin per tick from the active estimator state,
    rangefinder topology, sync aligner, and calibration loader. The
    gate's evaluation is deterministic over these inputs so the same
    snapshot always produces the same report — useful for tests and
    for explaining the row in the GCS.
    """

    mode: str
    companion_state: str = "inactive"
    estimator_state: str = "off"
    flow_quality: Optional[int] = None
    flow_scale_source: Optional[str] = None  # rangefinder|baro|gps|vision|None
    rangefinder_topology: Optional[str] = None  # companion|fc|both|None
    intrinsics_loaded: bool = False
    extrinsics_loaded: bool = False
    sync_offset_ms: Optional[float] = None
    feature_count: Optional[int] = None


class PreArmGate:
    """Evaluate the pre-arm checks for the current vision-nav mode."""

    def __init__(
        self,
        *,
        flow_quality_gate: int = DEFAULT_FLOW_QUALITY_GATE,
        vio_feature_floor: int = DEFAULT_VIO_FEATURE_FLOOR,
        sync_offset_red_ms: float = SYNC_OFFSET_RED_MS,
    ) -> None:
        self._flow_quality_gate = int(flow_quality_gate)
        self._vio_feature_floor = int(vio_feature_floor)
        self._sync_offset_red_ms = float(sync_offset_red_ms)

    def evaluate(self, inputs: PreArmInputs) -> PreArmReport:
        mode = inputs.mode
        if mode == "off":
            return PreArmReport(mode=mode, armable=True, checks=())
        if mode == "optical_flow":
            checks = self._of_checks(inputs, require_rangefinder=True)
        elif mode == "optical_flow_degraded":
            checks = self._of_checks(inputs, require_rangefinder=False)
        elif mode in ("vio_openvins", "vio_vins_fusion"):
            checks = self._vio_checks(inputs)
        elif mode == "hybrid_of_plus_vio":
            checks = tuple(
                list(self._of_checks(inputs, require_rangefinder=False))
                + list(self._vio_checks(inputs))
            )
        else:
            # Unknown mode -> single blocking check so the GCS surfaces
            # the misconfiguration rather than silently arming.
            checks = (
                PreArmCheck(
                    id="mode_unknown",
                    severity="blocking",
                    detail=f"Unknown mode {mode!r}",
                ),
            )
        armable = all(c.severity == "ok" for c in checks) if checks else True
        return PreArmReport(mode=mode, armable=armable, checks=checks)

    # ------------------------------------------------------------------
    # Per-mode check sets
    # ------------------------------------------------------------------

    def _of_checks(
        self, inputs: PreArmInputs, *, require_rangefinder: bool
    ) -> tuple[PreArmCheck, ...]:
        checks: list[PreArmCheck] = [
            self._companion_check(inputs),
            self._flow_quality_check(inputs),
        ]
        if require_rangefinder:
            checks.append(self._rangefinder_check(inputs))
        else:
            checks.append(self._scale_source_check(inputs))
        return tuple(checks)

    def _vio_checks(self, inputs: PreArmInputs) -> tuple[PreArmCheck, ...]:
        return (
            self._companion_check(inputs),
            self._estimator_converged_check(inputs),
            self._intrinsics_check(inputs),
            self._extrinsics_check(inputs),
            self._sync_offset_check(inputs),
            self._feature_count_check(inputs),
        )

    # ------------------------------------------------------------------
    # Individual checks
    # ------------------------------------------------------------------

    def _companion_check(self, inputs: PreArmInputs) -> PreArmCheck:
        if inputs.companion_state == "active":
            return PreArmCheck(id="companion_active", severity="ok")
        if inputs.companion_state == "inactive":
            return PreArmCheck(
                id="companion_active",
                severity="pending",
                detail="Companion has not reported active yet.",
            )
        return PreArmCheck(
            id="companion_active",
            severity="blocking",
            detail=f"Companion state {inputs.companion_state!r}.",
        )

    def _flow_quality_check(self, inputs: PreArmInputs) -> PreArmCheck:
        quality = inputs.flow_quality
        if quality is None:
            return PreArmCheck(
                id="flow_quality",
                severity="pending",
                detail="Awaiting first optical-flow sample.",
            )
        if quality >= self._flow_quality_gate:
            return PreArmCheck(
                id="flow_quality",
                severity="ok",
                detail=f"Quality {quality}/255.",
            )
        return PreArmCheck(
            id="flow_quality",
            severity="blocking",
            detail=f"Quality {quality} below gate {self._flow_quality_gate}.",
        )

    def _rangefinder_check(self, inputs: PreArmInputs) -> PreArmCheck:
        if inputs.rangefinder_topology in ("companion", "fc", "both"):
            return PreArmCheck(
                id="rangefinder",
                severity="ok",
                detail=f"Topology: {inputs.rangefinder_topology}",
            )
        return PreArmCheck(
            id="rangefinder",
            severity="blocking",
            detail=(
                "Rangefinder required in this mode. "
                "Switch to Optical Flow (degraded) to fly without one."
            ),
        )

    def _scale_source_check(self, inputs: PreArmInputs) -> PreArmCheck:
        source = inputs.flow_scale_source
        if source in ("rangefinder", "baro", "gps", "vision"):
            return PreArmCheck(
                id="scale_source",
                severity="ok",
                detail=f"Scale source: {source}.",
            )
        return PreArmCheck(
            id="scale_source",
            severity="blocking",
            detail="No altitude or depth source healthy.",
        )

    def _estimator_converged_check(
        self, inputs: PreArmInputs
    ) -> PreArmCheck:
        state = inputs.estimator_state
        if state == "converged":
            return PreArmCheck(id="estimator_converged", severity="ok")
        if state in ("init", "converging"):
            return PreArmCheck(
                id="estimator_converged",
                severity="pending",
                detail=f"Estimator {state}.",
            )
        return PreArmCheck(
            id="estimator_converged",
            severity="blocking",
            detail=f"Estimator state {state!r}.",
        )

    def _intrinsics_check(self, inputs: PreArmInputs) -> PreArmCheck:
        if inputs.intrinsics_loaded:
            return PreArmCheck(id="intrinsics_loaded", severity="ok")
        return PreArmCheck(
            id="intrinsics_loaded",
            severity="blocking",
            detail=(
                "Camera intrinsics not loaded. "
                "Upload a Kalibr camchain.yaml or run the calibration wizard."
            ),
        )

    def _extrinsics_check(self, inputs: PreArmInputs) -> PreArmCheck:
        if inputs.extrinsics_loaded:
            return PreArmCheck(id="extrinsics_loaded", severity="ok")
        return PreArmCheck(
            id="extrinsics_loaded",
            severity="blocking",
            detail=(
                "Camera-IMU extrinsics not loaded. "
                "Both T_cam_imu and timeshift_cam_imu are required for VIO."
            ),
        )

    def _sync_offset_check(self, inputs: PreArmInputs) -> PreArmCheck:
        offset = inputs.sync_offset_ms
        if offset is None:
            return PreArmCheck(
                id="sync_offset",
                severity="pending",
                detail="Time aligner has no measurements yet.",
            )
        if abs(offset) <= self._sync_offset_red_ms:
            return PreArmCheck(
                id="sync_offset",
                severity="ok",
                detail=f"Sync residual {offset:.1f} ms.",
            )
        return PreArmCheck(
            id="sync_offset",
            severity="blocking",
            detail=(
                f"Sync residual {offset:.1f} ms exceeds "
                f"{self._sync_offset_red_ms:.0f} ms. "
                "Re-run the camera-IMU calibration."
            ),
        )

    def _feature_count_check(self, inputs: PreArmInputs) -> PreArmCheck:
        count = inputs.feature_count
        if count is None:
            return PreArmCheck(
                id="feature_count",
                severity="pending",
                detail="No feature count reported yet.",
            )
        if count >= self._vio_feature_floor:
            return PreArmCheck(
                id="feature_count",
                severity="ok",
                detail=f"Tracking {count} features.",
            )
        return PreArmCheck(
            id="feature_count",
            severity="blocking",
            detail=(
                f"Only {count} features tracked "
                f"(minimum {self._vio_feature_floor}). "
                "Move to a more textured scene or improve lighting."
            ),
        )
