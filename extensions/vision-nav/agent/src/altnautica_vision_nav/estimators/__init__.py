"""Estimator framework for vision navigation.

Estimators are the swappable inner loop of the plugin: an
``OpticalFlowEstimator`` produces ``OPTICAL_FLOW_RAD`` samples, a future
``VioEstimator`` will produce ``VISION_POSITION_ESTIMATE`` samples, and
the pipeline doesn't need to know which kind it is holding. Every
estimator returns the same :class:`EstimatorOutput` shape and the
pipeline decides how that maps to MAVLink.

This package ships the seam without rewiring the live pipeline. The
abstractions land alongside the existing :class:`OFPipeline` so the
live behaviour is unchanged; future work retires that pipeline in
favour of the estimator-agnostic one that consumes
:class:`BaseEstimator` directly.
"""

from __future__ import annotations

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
    OutputMode,
    ScaleSource,
)
from altnautica_vision_nav.estimators.null_estimator import NullEstimator
from altnautica_vision_nav.estimators.optical_flow import OpticalFlowEstimator
from altnautica_vision_nav.estimators.optical_flow_no_range import (
    OpticalFlowNoRangeEstimator,
)
from altnautica_vision_nav.estimators.vio_engine import (
    VioEstimator,
    VioOpenvinsEstimator,
    VioVinsFusionEstimator,
)

# Registry: estimator key -> class. Keys mirror the config ``mode``
# values so a config flip selects the matching estimator without
# bespoke routing. Adding a new estimator is a single line here plus
# the implementation file; the pipeline picks it up automatically.
#
# The two VIO entries map to thin subclasses of :class:`VioEstimator`
# that only differ in ``estimator_id`` so the registry-key contract
# (key == class identifier) holds. The engine choice (OpenVINS vs
# VINS-Fusion) is still a runtime parameter constructed by the
# pipeline, not by these classes.
ESTIMATOR_REGISTRY: dict[str, type[BaseEstimator]] = {
    "off": NullEstimator,
    "optical_flow": OpticalFlowEstimator,
    "optical_flow_degraded": OpticalFlowNoRangeEstimator,
    "vio_openvins": VioOpenvinsEstimator,
    "vio_vins_fusion": VioVinsFusionEstimator,
}


def available_estimators() -> list[str]:
    """Return the list of estimator keys currently wired.

    Surfaced on the agent heartbeat so the GCS mode picker only shows
    estimators the running plugin can actually instantiate.
    """

    return sorted(ESTIMATOR_REGISTRY.keys())


__all__ = [
    "BaseEstimator",
    "EstimatorOutput",
    "EstimatorState",
    "ESTIMATOR_REGISTRY",
    "NullEstimator",
    "OpticalFlowEstimator",
    "OpticalFlowNoRangeEstimator",
    "OutputMode",
    "ScaleSource",
    "VioEstimator",
    "VioOpenvinsEstimator",
    "VioVinsFusionEstimator",
    "available_estimators",
]
