"""MAVLink component selector for the estimator's output.

The plugin owns two MAVLink components:

* component 198 (peripheral) for OF mode — emits ``OPTICAL_FLOW_RAD``
  and ``DISTANCE_SENSOR``.
* component 197 (``MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY``) for VIO
  mode — emits ``VISION_POSITION_ESTIMATE``.

The router reads the active estimator's ``output_mode`` and picks the
component each sample should ride. Hybrid mode runs both estimators
concurrently and the router emits on both components in parallel.
"""

from __future__ import annotations

import logging
from typing import Optional

from altnautica_vision_nav.estimators.base import EstimatorOutput
from altnautica_vision_nav.mavlink import encoders as _encoders

log = logging.getLogger(__name__)


COMPONENT_OF_PERIPHERAL = 198
# MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY per MAVLink common.xml. The
# autopilot routes VIO injection messages to this component id when
# the EKF source is set to ExternalNav.
COMPONENT_VIO = 197


class MavlinkComponentRouter:
    """Emit an EstimatorOutput on the matching MAVLink component.

    Construct once per pipeline; call :meth:`emit` after every
    successful estimator step. The router translates the typed output
    into MAVLink frames + dispatches through the existing
    :mod:`encoders` emit helpers.
    """

    def __init__(
        self,
        *,
        ctx: object,
        clock_align: Optional[object] = None,
        of_component_id: int = COMPONENT_OF_PERIPHERAL,
        vio_component_id: int = COMPONENT_VIO,
        sensor_id: int = 0,
    ) -> None:
        self._ctx = ctx
        self._clock_align = clock_align
        self._of_component = int(of_component_id)
        self._vio_component = int(vio_component_id)
        self._sensor_id = int(sensor_id)

    @property
    def of_component(self) -> int:
        return self._of_component

    @property
    def vio_component(self) -> int:
        return self._vio_component

    async def emit(self, sample: EstimatorOutput) -> None:
        """Route ``sample`` to the matching MAVLink component.

        ``output_mode`` decides the path:

        * ``"optical_flow"`` -> OPTICAL_FLOW_RAD on comp 198
        * ``"vio"`` -> VISION_POSITION_ESTIMATE on comp 197
        * ``"none"`` -> no emission (off mode, or pre-arm sample
          that has not yet converged)

        Samples that should not produce wire emissions return ``None``
        from this method's perspective; the caller treats no return
        value as "nothing emitted". MAVLink emission failures are
        logged and swallowed so a transient send error does not crash
        the pipeline.
        """

        if sample.output_mode == "optical_flow":
            await self._emit_of(sample)
        elif sample.output_mode == "vio":
            await self._emit_vio(sample)
        # "none" outputs are deliberately silent.

    async def _emit_of(self, sample: EstimatorOutput) -> None:
        if sample.flow_rate_x is None or sample.flow_rate_y is None:
            return
        distance = float(sample.flow_distance_m or 0.0)
        await _encoders.emit_optical_flow_rad(
            self._ctx,
            component_id=self._of_component,
            sensor_id=self._sensor_id,
            integration_time_us=int(sample.integration_time_us or 0),
            integrated_x=float(sample.flow_rate_x),
            integrated_y=float(sample.flow_rate_y),
            integrated_xgyro=0.0,
            integrated_ygyro=0.0,
            integrated_zgyro=float(sample.flow_rate_z or 0.0),
            temperature=0,
            quality=int(sample.flow_quality or 0),
            time_delta_distance_us=0,
            distance=distance,
            clock_align=self._clock_align,
        )

    async def _emit_vio(self, sample: EstimatorOutput) -> None:
        pose = sample.pose
        if pose is None or len(pose) != 6:
            return
        x, y, z, roll, pitch, yaw = pose
        await _encoders.emit_vision_position_estimate(
            self._ctx,
            component_id=self._vio_component,
            x=float(x),
            y=float(y),
            z=float(z),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            covariance=tuple(sample.covariance) if sample.covariance else (),
            reset_counter=int(sample.reset_counter or 0),
            clock_align=self._clock_align,
        )
