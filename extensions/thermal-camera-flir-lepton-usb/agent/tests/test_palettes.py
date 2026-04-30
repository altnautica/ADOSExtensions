"""Unit tests for the palette LUTs and frame application."""

from __future__ import annotations

import pytest

from altnautica_thermal_camera.palettes import (
    PALETTE_SIZE,
    PALETTES,
    apply_palette,
    list_palettes,
    palette_lut,
)


def test_built_in_palettes_have_three_entries() -> None:
    names = list_palettes()
    assert names == ["ironbow", "rainbow", "grayscale"]
    for name in names:
        assert name in PALETTES


def test_each_palette_lut_has_768_entries() -> None:
    for name in list_palettes():
        lut = palette_lut(name)
        assert len(lut) == PALETTE_SIZE * 3


def test_palette_lut_clips_components_to_byte_range() -> None:
    for name in list_palettes():
        lut = palette_lut(name)
        for value in lut:
            assert 0 <= value <= 255


def test_grayscale_is_strictly_monotonic_per_channel() -> None:
    lut = palette_lut("grayscale")
    for i in range(PALETTE_SIZE):
        r = lut[i * 3]
        g = lut[i * 3 + 1]
        b = lut[i * 3 + 2]
        assert r == i
        assert g == i
        assert b == i


def test_unknown_palette_raises() -> None:
    with pytest.raises(ValueError):
        palette_lut("not-a-palette")


def test_apply_palette_produces_rgba_buffer() -> None:
    pixels = [10, 20, 30, 40]
    out = apply_palette(pixels, width=2, height=2, palette="grayscale")
    assert len(out) == 4 * 4
    # Alpha is always 255.
    for i in range(4):
        assert out[i * 4 + 3] == 255


def test_apply_palette_grayscale_min_paints_black_max_paints_white() -> None:
    pixels = [100, 200, 300, 400]
    out = apply_palette(
        pixels,
        width=2,
        height=2,
        palette="grayscale",
        lo_y16=100,
        hi_y16=400,
    )
    # First pixel is the min, so it should land at LUT index 0 -> RGB 0,0,0.
    assert (out[0], out[1], out[2]) == (0, 0, 0)
    # Last pixel is the max -> LUT index 255 -> RGB 255,255,255.
    last = (4 - 1) * 4
    assert (out[last], out[last + 1], out[last + 2]) == (255, 255, 255)


def test_apply_palette_rejects_size_mismatch() -> None:
    with pytest.raises(ValueError):
        apply_palette([1, 2, 3], width=2, height=2, palette="ironbow")


def test_apply_palette_rejects_zero_dimensions() -> None:
    with pytest.raises(ValueError):
        apply_palette([], width=0, height=0, palette="ironbow")


def test_ironbow_low_end_is_dark_high_end_is_bright() -> None:
    lut = palette_lut("ironbow")
    low_brightness = sum(lut[0:3])
    high_brightness = sum(lut[(PALETTE_SIZE - 1) * 3 : PALETTE_SIZE * 3])
    assert high_brightness > low_brightness


def test_rainbow_low_end_is_blue_high_end_is_red() -> None:
    lut = palette_lut("rainbow")
    # Index 0: blue dominates; index 255: red dominates.
    r0, g0, b0 = lut[0], lut[1], lut[2]
    r1, g1, b1 = (
        lut[(PALETTE_SIZE - 1) * 3],
        lut[(PALETTE_SIZE - 1) * 3 + 1],
        lut[(PALETTE_SIZE - 1) * 3 + 2],
    )
    assert b0 > r0
    assert r1 > b1


def test_rainbow_lightness_is_monotonic_non_decreasing() -> None:
    """A spot at a higher temperature must not read visually darker
    than a cooler spot. Brightness is approximated by the channel sum.
    Allow tiny rounding wobble (<= 3) per step so anchor-stop
    quantisation does not fail us; the trend across the LUT must rise.
    """

    lut = palette_lut("rainbow")
    prev = lut[0] + lut[1] + lut[2]
    max_dip = 0
    for i in range(1, PALETTE_SIZE):
        cur = lut[i * 3] + lut[i * 3 + 1] + lut[i * 3 + 2]
        if cur < prev:
            max_dip = max(max_dip, prev - cur)
        prev = cur
    assert max_dip <= 3, f"rainbow palette has a {max_dip}-unit lightness dip"
    last = lut[(PALETTE_SIZE - 1) * 3] + lut[(PALETTE_SIZE - 1) * 3 + 1] + lut[(PALETTE_SIZE - 1) * 3 + 2]
    first = lut[0] + lut[1] + lut[2]
    assert last > first


def test_ironbow_lightness_is_monotonic_non_decreasing() -> None:
    lut = palette_lut("ironbow")
    prev = lut[0] + lut[1] + lut[2]
    max_dip = 0
    for i in range(1, PALETTE_SIZE):
        cur = lut[i * 3] + lut[i * 3 + 1] + lut[i * 3 + 2]
        if cur < prev:
            max_dip = max(max_dip, prev - cur)
        prev = cur
    assert max_dip <= 3, f"ironbow palette has a {max_dip}-unit lightness dip"
