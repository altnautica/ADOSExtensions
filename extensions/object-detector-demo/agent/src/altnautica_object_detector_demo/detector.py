"""Cheap brightest-region heuristic over a frame's luma plane.

The detector does not run a neural network. It reads the luminance (luma)
plane of one frame, divides it into a grid of fixed-size cells, and picks the
cell whose mean luma is highest. That cell becomes one bounding box with a
class label of ``bright-region`` and a confidence equal to the cell's mean
luma normalized to 0..1. The point is to exercise the frame-read and
detection-publish path with a result a test can predict, not to detect
anything meaningful.

Luma extraction by pixel format:

* ``nv12`` / ``yuv420p``: the luma plane is the first ``width * height`` bytes
  (both formats store the full-resolution Y plane first, then chroma).
* ``rgb24``: there is no separate luma plane, so luma is derived per pixel
  with the Rec. 601 weights (0.299 R + 0.587 G + 0.114 B).

The grid cell size defaults to 64x64. When a frame is smaller than one cell in
either axis the cell is clamped to the frame so a single box still covers it.
"""

from __future__ import annotations

from dataclasses import dataclass

from ados.sdk.vision import BoundingBox, Detection, Frame, FrameFormat

# The class label every detection this plugin emits carries. It names the
# heuristic's output, not a real object class.
CLASS_LABEL = "bright-region"

# Default grid cell edge in pixels. Each cell is scored by its mean luma; the
# brightest cell becomes the bounding box.
DEFAULT_CELL = 64

# Rec. 601 luma weights for the rgb24 path, scaled to integers (sum 1000) so
# the per-pixel reduction stays in integer math.
_R_W = 299
_G_W = 587
_B_W = 114
_W_SUM = _R_W + _G_W + _B_W


@dataclass(frozen=True)
class BrightRegion:
    """The winning cell: its pixel-space box and mean luma (0..255)."""

    box: BoundingBox
    mean_luma: float


def luma_plane(frame: Frame) -> tuple[bytes, int, int]:
    """Return ``(luma, width, height)`` for ``frame``.

    For the 4:2:0 formats this is the first ``width * height`` bytes of the
    pixel buffer. For ``rgb24`` it is a freshly built per-pixel luma buffer.
    ``width`` and ``height`` come from the descriptor; when the descriptor does
    not carry them (the test harness leaves them at zero) they are inferred
    from the buffer length and the frame's format so the scan still runs.
    """
    desc = frame.descriptor
    fmt = FrameFormat(desc.format)
    width = int(desc.width)
    height = int(desc.height)
    if width <= 0 or height <= 0:
        width, height = _infer_dims(len(frame.pixels), fmt)

    if fmt is FrameFormat.RGB24:
        return _rgb24_to_luma(frame.pixels, width, height), width, height

    # nv12 and yuv420p both lead with the full-resolution Y (luma) plane.
    count = width * height
    return bytes(frame.pixels[:count]), width, height


def find_bright_region(
    frame: Frame, *, cell: int = DEFAULT_CELL
) -> BrightRegion | None:
    """Find the brightest ``cell`` x ``cell`` grid cell in ``frame``.

    Returns ``None`` only when the frame has no usable pixels (zero area or an
    empty buffer). Otherwise the brightest cell is returned as a
    :class:`BrightRegion` with a box clamped to the frame bounds.
    """
    luma, width, height = luma_plane(frame)
    if width <= 0 or height <= 0 or not luma:
        return None

    step = max(1, int(cell))
    cell_w = min(step, width)
    cell_h = min(step, height)

    best_sum = -1
    best_x = 0
    best_y = 0
    best_count = 1
    for y0 in range(0, height, step):
        y1 = min(y0 + cell_h, height)
        for x0 in range(0, width, step):
            x1 = min(x0 + cell_w, width)
            total = 0
            for y in range(y0, y1):
                row = y * width
                total += sum(luma[row + x0 : row + x1])
            count = (x1 - x0) * (y1 - y0)
            if count > 0 and total > best_sum:
                best_sum = total
                best_x = x0
                best_y = y0
                best_count = count

    if best_sum < 0:
        return None

    mean = best_sum / best_count
    box = BoundingBox(
        x=float(best_x),
        y=float(best_y),
        width=float(min(cell_w, width - best_x)),
        height=float(min(cell_h, height - best_y)),
    )
    return BrightRegion(box=box, mean_luma=mean)


def detection_for(frame: Frame, *, cell: int = DEFAULT_CELL) -> Detection | None:
    """Build the single :class:`Detection` for ``frame``, or ``None`` if the
    frame has no usable pixels. Confidence is the brightest cell's mean luma
    normalized to 0..1."""
    region = find_bright_region(frame, cell=cell)
    if region is None:
        return None
    confidence = max(0.0, min(1.0, region.mean_luma / 255.0))
    return Detection(
        bbox=region.box,
        class_label=CLASS_LABEL,
        confidence=confidence,
    )


def _infer_dims(byte_len: int, fmt: FrameFormat) -> tuple[int, int]:
    """Infer a near-square ``(width, height)`` from a buffer length and format.

    Used when a descriptor omits dimensions. The pixel count is the buffer
    length for ``rgb24`` divided by 3, or the buffer length scaled by 2/3 for
    the 4:2:0 formats (which carry 1.5 bytes per pixel). The result is the
    largest square that fits, with any remainder folded into the height so the
    whole buffer is covered.
    """
    if byte_len <= 0:
        return 0, 0
    if fmt is FrameFormat.RGB24:
        pixels = byte_len // 3
    else:
        pixels = (byte_len * 2) // 3
    if pixels <= 0:
        return 0, 0
    side = int(pixels**0.5)
    side = max(1, side)
    width = side
    height = max(1, pixels // width)
    return width, height


def _rgb24_to_luma(pixels: bytes, width: int, height: int) -> bytes:
    """Reduce packed 24-bit RGB to an 8-bit luma plane using Rec. 601 weights."""
    count = width * height
    out = bytearray(count)
    src = memoryview(pixels)
    for i in range(count):
        base = i * 3
        if base + 2 >= len(src):
            break
        r = src[base]
        g = src[base + 1]
        b = src[base + 2]
        out[i] = (r * _R_W + g * _G_W + b * _B_W) // _W_SUM
    return bytes(out)


__all__ = [
    "CLASS_LABEL",
    "DEFAULT_CELL",
    "BrightRegion",
    "luma_plane",
    "find_bright_region",
    "detection_for",
]
