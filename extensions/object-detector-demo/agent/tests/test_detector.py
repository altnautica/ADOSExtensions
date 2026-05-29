"""Unit tests for the brightest-region heuristic (no host, no async)."""

from __future__ import annotations

from ados.sdk.vision import Frame, FrameDescriptor, FrameFormat

from altnautica_object_detector_demo.detector import (
    CLASS_LABEL,
    detection_for,
    find_bright_region,
    luma_plane,
)

WIDTH = 256
HEIGHT = 128
CELL = 64

# A dark background and a bright patch, both inside the 0..255 luma range.
DARK = 16
BRIGHT = 240

# Origin of the planted bright patch, aligned to the 64-px grid so the
# brightest-cell scan lands its box exactly here.
PATCH_X = 128
PATCH_Y = 64


def _descriptor(fmt: FrameFormat, byte_len: int) -> FrameDescriptor:
    return FrameDescriptor(
        camera_id="cam0",
        frame_id=1,
        ts_ms=0,
        width=WIDTH,
        height=HEIGHT,
        format=fmt,
        shm_name="ados-vision-cam0",
        slot=0,
        seq=1,
        byte_len=byte_len,
    )


def _yuv420p_with_patch() -> Frame:
    """A planar 4:2:0 frame: dark luma plane with one bright CELL-sized patch,
    plus neutral chroma planes after it."""
    luma = bytearray([DARK] * (WIDTH * HEIGHT))
    for y in range(PATCH_Y, PATCH_Y + CELL):
        row = y * WIDTH
        for x in range(PATCH_X, PATCH_X + CELL):
            luma[row + x] = BRIGHT
    chroma = bytes([128] * (WIDTH * HEIGHT // 2))
    pixels = bytes(luma) + chroma
    return Frame(descriptor=_descriptor(FrameFormat.YUV420P, len(pixels)), pixels=pixels)


def _nv12_solid(value: int) -> Frame:
    """A semi-planar 4:2:0 frame filled to a flat luma value."""
    luma = bytes([value] * (WIDTH * HEIGHT))
    chroma = bytes([128] * (WIDTH * HEIGHT // 2))
    pixels = luma + chroma
    return Frame(descriptor=_descriptor(FrameFormat.NV12, len(pixels)), pixels=pixels)


def _rgb24_with_patch() -> Frame:
    """A packed RGB frame: dark grey background with one bright white patch."""
    px = bytearray([DARK, DARK, DARK] * (WIDTH * HEIGHT))
    for y in range(PATCH_Y, PATCH_Y + CELL):
        row = (y * WIDTH) * 3
        for x in range(PATCH_X, PATCH_X + CELL):
            base = row + x * 3
            px[base] = BRIGHT
            px[base + 1] = BRIGHT
            px[base + 2] = BRIGHT
    pixels = bytes(px)
    return Frame(descriptor=_descriptor(FrameFormat.RGB24, len(pixels)), pixels=pixels)


def test_luma_plane_yuv420p_is_the_first_wh_bytes() -> None:
    frame = _yuv420p_with_patch()
    luma, w, h = luma_plane(frame)
    assert (w, h) == (WIDTH, HEIGHT)
    assert len(luma) == WIDTH * HEIGHT
    # The planted patch is bright; a background pixel is dark.
    assert luma[PATCH_Y * WIDTH + PATCH_X] == BRIGHT
    assert luma[0] == DARK


def test_find_bright_region_lands_on_the_patch_cell() -> None:
    frame = _yuv420p_with_patch()
    region = find_bright_region(frame, cell=CELL)
    assert region is not None
    assert (region.box.x, region.box.y) == (float(PATCH_X), float(PATCH_Y))
    assert region.box.width == float(CELL)
    assert region.box.height == float(CELL)
    # The winning cell is fully inside the patch, so its mean luma is BRIGHT.
    assert region.mean_luma == float(BRIGHT)


def test_detection_for_patch_has_expected_label_and_confidence() -> None:
    frame = _yuv420p_with_patch()
    det = detection_for(frame, cell=CELL)
    assert det is not None
    assert det.class_label == CLASS_LABEL
    assert det.bbox.x == float(PATCH_X)
    assert det.bbox.y == float(PATCH_Y)
    # confidence == normalized mean luma of the brightest cell.
    assert abs(det.confidence - BRIGHT / 255.0) < 1e-6


def test_nv12_confidence_tracks_mean_luma() -> None:
    bright = detection_for(_nv12_solid(200), cell=CELL)
    dark = detection_for(_nv12_solid(10), cell=CELL)
    assert bright is not None and dark is not None
    assert bright.confidence > dark.confidence
    assert abs(bright.confidence - 200 / 255.0) < 1e-6


def test_rgb24_patch_is_found_via_derived_luma() -> None:
    det = detection_for(_rgb24_with_patch(), cell=CELL)
    assert det is not None
    assert det.class_label == CLASS_LABEL
    assert det.bbox.x == float(PATCH_X)
    assert det.bbox.y == float(PATCH_Y)
    # White patch reduces to ~BRIGHT luma; background to ~DARK. The box sits on
    # the patch and confidence is high.
    assert det.confidence > 0.5
