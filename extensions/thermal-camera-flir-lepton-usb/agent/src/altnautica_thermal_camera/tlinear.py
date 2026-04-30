"""TLinear conversion helpers for the FLIR Lepton 3.5 in radiometric mode.

The Lepton emits Y16 raw pixels. With the TLinear feature enabled over
the camera control interface (CCI), each pixel maps linearly to absolute
temperature with a fixed resolution per count. v1.0 ships a single
resolution constant (0.01 K/count); higher-resolution modes can pass an
override when the operator changes the TLinear setting at runtime.

The module is intentionally pure-Python with zero external dependencies
so the agent half stays cheap to import and the unit tests do not
require numpy. Hot-loop pixel math runs through numpy in the driver
where it matters; this module covers the scalar conversion semantics
that downstream consumers exercise per-frame for the spot-meter and
alarm paths.
"""

from __future__ import annotations

from typing import Iterable

#: Default TLinear resolution for the Lepton 3.5 in 0.01 K mode. Matches
#: the value the driver writes via the CCI extension unit at boot.
DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT: float = 0.01

#: Absolute zero offset between kelvin and celsius.
KELVIN_C_OFFSET: float = 273.15


def kelvin_from_y16(
    y16: int,
    resolution_k_per_count: float = DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
) -> float:
    """Convert a single Y16 raw pixel into absolute temperature (kelvin).

    Per the spec: ``temp_K = raw_y16 * resolution_k_per_count``. With the
    default 0.01 K/count, raw = 27315 maps to 273.15 K (zero deg C) and
    raw = 33315 maps to 333.15 K (60 deg C).

    Negative raw values are not physically meaningful for the Lepton in
    radiometric mode but the function does not clamp; callers that need
    a sanity bound should apply it themselves.
    """

    return float(y16) * float(resolution_k_per_count)


def celsius_from_y16(
    y16: int,
    resolution_k_per_count: float = DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
) -> float:
    """Convert a single Y16 raw pixel into temperature in deg C."""

    return kelvin_from_y16(y16, resolution_k_per_count) - KELVIN_C_OFFSET


def y16_from_kelvin(
    kelvin: float,
    resolution_k_per_count: float = DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
) -> int:
    """Inverse of :func:`kelvin_from_y16`. Returns the nearest Y16 count.

    Used by the driver when a fixed-AGC range is configured: the GCS
    panel passes celsius bounds, the agent converts both ends to Y16
    counts once, and the colorize step normalizes against those counts
    rather than re-computing per pixel.
    """

    if resolution_k_per_count <= 0:
        raise ValueError("resolution_k_per_count must be positive")
    return int(round(kelvin / resolution_k_per_count))


def y16_from_celsius(
    celsius: float,
    resolution_k_per_count: float = DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
) -> int:
    """Inverse of :func:`celsius_from_y16`."""

    return y16_from_kelvin(celsius + KELVIN_C_OFFSET, resolution_k_per_count)


def celsius_grid_extrema(
    y16_pixels: Iterable[int],
    resolution_k_per_count: float = DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
) -> tuple[float, float]:
    """Return ``(min_c, max_c)`` over an iterable of Y16 raw pixels.

    Pure-Python fallback for when numpy is not available or when the
    caller has a small region-of-interest sample. The driver's hot loop
    uses numpy directly for the full grid.
    """

    iterator = iter(y16_pixels)
    try:
        first = next(iterator)
    except StopIteration as exc:  # pragma: no cover - defensive
        raise ValueError("empty pixel iterable") from exc
    min_y16 = max_y16 = int(first)
    for value in iterator:
        v = int(value)
        if v < min_y16:
            min_y16 = v
        elif v > max_y16:
            max_y16 = v
    return (
        celsius_from_y16(min_y16, resolution_k_per_count),
        celsius_from_y16(max_y16, resolution_k_per_count),
    )
