"""False-color palette LUTs for thermal frame visualisation.

Each palette is a 256-entry RGB lookup table. A normalised intensity
``t`` in ``[0.0, 1.0]`` (typically computed by min/max scaling the Y16
grid) selects an index ``floor(t * 255)`` whose RGB triple paints the
output pixel. Three palettes ship in v1.0:

* ``ironbow`` is the SAR-default palette: black -> purple -> red ->
  yellow -> white. Hot is bright. Highest visual contrast for warm
  bodies in cool surroundings.

* ``rainbow`` is a quantitative palette: blue -> cyan -> green ->
  yellow -> red. Cool is blue, hot is red. Useful when the operator
  cares about absolute temperature mapping more than warm-body pop.

* ``grayscale`` is a linear black-to-white ramp. Useful for export and
  for documentation that prints in monochrome.

Tables are computed once at import time. The output type is a flat
``list[int]`` of length 768 (256 RGB triples) so consumers can choose
between numpy ``uint8`` views or plain integer indexing without
forcing a numpy dependency on every consumer.
"""

from __future__ import annotations

from typing import Sequence

PALETTE_SIZE = 256


def _ironbow_lut() -> list[int]:
    """Build the 256-entry ironbow LUT.

    Five anchor stops define a piecewise-linear ramp from black through
    purple, red, and yellow into white.
    """

    stops: list[tuple[int, int, int]] = [
        (0, 0, 0),
        (50, 0, 80),
        (170, 30, 0),
        (255, 160, 0),
        (255, 255, 255),
    ]
    return _gradient(stops)


def _rainbow_lut() -> list[int]:
    """Build the 256-entry rainbow LUT.

    Cool to hot ramp with strictly increasing perceived lightness so a
    spot at higher temperature reads visually brighter than a cooler
    spot. Endpoints stay blue-dominant (cool) and red-dominant (hot).
    Stops:

    * (0, 0, 80)      dark blue
    * (0, 60, 180)    medium blue
    * (60, 140, 200)  cyan-blue
    * (160, 180, 100) yellow-green
    * (240, 180, 80)  orange
    * (255, 220, 200) warm pink-white
    """

    stops: list[tuple[int, int, int]] = [
        (0, 0, 80),
        (0, 60, 180),
        (60, 140, 200),
        (160, 180, 100),
        (240, 180, 80),
        (255, 220, 200),
    ]
    return _gradient(stops)


def _grayscale_lut() -> list[int]:
    """Build the 256-entry grayscale LUT (black -> white)."""

    out: list[int] = []
    for i in range(PALETTE_SIZE):
        out.extend((i, i, i))
    return out


def _gradient(stops: Sequence[tuple[int, int, int]]) -> list[int]:
    if len(stops) < 2:
        raise ValueError("at least two color stops required")
    segments = len(stops) - 1
    out: list[int] = []
    for i in range(PALETTE_SIZE):
        position = i / (PALETTE_SIZE - 1) * segments
        seg_index = min(int(position), segments - 1)
        local = position - seg_index
        r0, g0, b0 = stops[seg_index]
        r1, g1, b1 = stops[seg_index + 1]
        r = int(round(r0 + (r1 - r0) * local))
        g = int(round(g0 + (g1 - g0) * local))
        b = int(round(b0 + (b1 - b0) * local))
        out.extend((_clip(r), _clip(g), _clip(b)))
    return out


def _clip(v: int) -> int:
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


PALETTES: dict[str, list[int]] = {
    "ironbow": _ironbow_lut(),
    "rainbow": _rainbow_lut(),
    "grayscale": _grayscale_lut(),
}


def list_palettes() -> list[str]:
    """Return the names of the built-in palettes in a stable order."""

    return ["ironbow", "rainbow", "grayscale"]


def palette_lut(name: str) -> list[int]:
    """Return the flat RGB LUT for a palette by name.

    Raises :class:`ValueError` for unknown names so the GCS panel can
    surface the rejection cleanly when an operator picks an invalid
    palette via the config form.
    """

    try:
        return PALETTES[name]
    except KeyError as exc:
        raise ValueError(f"unknown palette: {name!r}") from exc


def apply_palette(
    y16_pixels: Sequence[int],
    width: int,
    height: int,
    palette: str = "ironbow",
    lo_y16: int | None = None,
    hi_y16: int | None = None,
) -> bytearray:
    """Map a Y16 grid to an RGBA byte buffer using the named palette.

    Returns a flat ``bytearray`` of length ``width * height * 4`` with
    each pixel as ``(R, G, B, 255)``. The function is pure-Python and
    intentionally simple; the production driver replaces this with a
    numpy vectorised version. Tests use it for golden-frame checks.

    ``lo_y16`` and ``hi_y16`` give the range that maps to LUT indices 0
    and 255 respectively. When unset they default to the observed grid
    extrema (linear AGC over the current frame).
    """

    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    expected = width * height
    if len(y16_pixels) != expected:
        raise ValueError(
            f"pixel count mismatch: expected {expected}, got {len(y16_pixels)}"
        )

    lut = palette_lut(palette)

    if lo_y16 is None or hi_y16 is None:
        observed_lo = min(y16_pixels)
        observed_hi = max(y16_pixels)
        if lo_y16 is None:
            lo_y16 = observed_lo
        if hi_y16 is None:
            hi_y16 = observed_hi
    if hi_y16 <= lo_y16:
        hi_y16 = lo_y16 + 1

    span = float(hi_y16 - lo_y16)
    out = bytearray(expected * 4)
    for i, raw in enumerate(y16_pixels):
        clamped = raw
        if clamped < lo_y16:
            clamped = lo_y16
        elif clamped > hi_y16:
            clamped = hi_y16
        normalised = (clamped - lo_y16) / span
        idx = int(normalised * (PALETTE_SIZE - 1))
        if idx < 0:
            idx = 0
        elif idx > PALETTE_SIZE - 1:
            idx = PALETTE_SIZE - 1
        base = idx * 3
        dst = i * 4
        out[dst] = lut[base]
        out[dst + 1] = lut[base + 1]
        out[dst + 2] = lut[base + 2]
        out[dst + 3] = 255
    return out
