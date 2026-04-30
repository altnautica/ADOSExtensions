"""Unit tests for the TLinear conversion helpers."""

from __future__ import annotations

import math

import pytest

from altnautica_thermal_camera.tlinear import (
    DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
    KELVIN_C_OFFSET,
    celsius_from_y16,
    celsius_grid_extrema,
    kelvin_from_y16,
    y16_from_celsius,
    y16_from_kelvin,
)


def test_zero_celsius_round_trips_to_27315_counts() -> None:
    raw = y16_from_celsius(0.0)
    # 0 deg C is 273.15 K. At 0.01 K/count that is 27315 counts.
    assert raw == 27315
    assert math.isclose(celsius_from_y16(raw), 0.0, abs_tol=1e-9)


def test_sixty_celsius_round_trip() -> None:
    raw = y16_from_celsius(60.0)
    assert raw == 33315
    assert math.isclose(celsius_from_y16(raw), 60.0, abs_tol=1e-9)


def test_kelvin_from_y16_uses_default_resolution() -> None:
    # 12345 counts at 0.01 K/count = 123.45 K
    assert math.isclose(
        kelvin_from_y16(12345),
        12345 * DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
        abs_tol=1e-9,
    )


def test_celsius_from_y16_subtracts_kelvin_offset() -> None:
    raw = 30000
    kelvin = kelvin_from_y16(raw)
    celsius = celsius_from_y16(raw)
    assert math.isclose(celsius, kelvin - KELVIN_C_OFFSET, abs_tol=1e-9)


def test_y16_from_kelvin_rounds_to_nearest_count() -> None:
    # The helper uses `int(round(...))` so values strictly above the
    # half-way point round up while values strictly below round down.
    # Float representation of 273.155 makes it slightly less than the
    # exact half-way point, so the helper should land on 27315.
    assert y16_from_kelvin(273.157) == 27316
    assert y16_from_kelvin(273.152) == 27315


def test_y16_from_kelvin_rejects_zero_resolution() -> None:
    with pytest.raises(ValueError):
        y16_from_kelvin(273.15, resolution_k_per_count=0.0)


def test_y16_from_kelvin_rejects_negative_resolution() -> None:
    with pytest.raises(ValueError):
        y16_from_kelvin(273.15, resolution_k_per_count=-0.01)


def test_celsius_grid_extrema_reports_min_and_max() -> None:
    pixels = [27000, 28000, 26000, 33000, 30000]
    lo, hi = celsius_grid_extrema(pixels)
    # 26000 -> 260 K -> -13.15 C; 33000 -> 330 K -> 56.85 C
    assert math.isclose(lo, -13.15, abs_tol=1e-9)
    assert math.isclose(hi, 56.85, abs_tol=1e-9)


def test_celsius_grid_extrema_rejects_empty_input() -> None:
    with pytest.raises(ValueError):
        celsius_grid_extrema([])


def test_alternate_resolution_changes_mapping() -> None:
    # 0.1 K/count alternative: 0 deg C maps to roughly 2731-2732 counts.
    # 273.15 K / 0.1 = 2731.5 which sits at the half-way point between
    # 2731 and 2732. Float representation makes the exact result either
    # 2731 or 2732 depending on rounding; both are acceptable.
    raw = y16_from_celsius(0.0, resolution_k_per_count=0.1)
    assert raw in (2731, 2732)
    # Round-trip a clearly above-mid value to lock the direction.
    assert y16_from_celsius(0.05, resolution_k_per_count=0.1) == 2732
