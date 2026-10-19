"""3DGUT routing: a camera SfM returns with refined distortion trains with the unscented path."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common  # noqa: E402


def _view(model: str, params: list[float]) -> common.View:
    cam = common.ColmapCamera(1, model, 1280, 800, np.asarray(params, dtype=float))
    return common.View("cam_fl/1.png", np.eye(4), np.eye(3), 1280, 800, cam)


def test_zero_distortion_opencv_is_a_pinhole():
    v = _view("OPENCV", [537, 537, 640, 400, 0, 0, 0, 0])
    assert not v.camera.is_distorted()
    assert not common.needs_ut([v], False)


def test_refined_opencv_distortion_needs_ut():
    # The coefficients bundle adjustment returned for an uncalibrated `opencv` rig camera.
    v = _view("OPENCV", [537.26, 397.66, 640, 400, 1.46e-4, -2.1e-5, 1.98e-4, -3.7e-5])
    assert v.camera.is_distorted()
    assert common.needs_ut([_view("PINHOLE", [537, 537, 640, 400]), v], False)


def test_fisheye_and_explicit_request_need_ut():
    assert common.needs_ut([_view("OPENCV_FISHEYE", [300, 300, 640, 400, 0, 0, 0, 0])], False)
    assert common.needs_ut([_view("PINHOLE", [537, 537, 640, 400])], True)
    assert not common.needs_ut([common.View("x.png", np.eye(4), np.eye(3), 8, 8)], False)
