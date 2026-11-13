from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the RTX")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import colorize  # noqa: E402

W, H, F = 64, 48, 40.0


def _view(image: np.ndarray) -> colorize.ColorView:
    K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
    return colorize.ColorView(K, np.eye(4), W, H, image)


def _split_image() -> np.ndarray:
    img = np.zeros((H, W, 3), np.uint8)
    img[:, : W // 2] = (255, 0, 0)
    img[:, W // 2:] = (0, 0, 255)
    return img


def test_points_take_the_colour_where_they_project():
    pts = np.array([[-0.5, 0.0, 3.0], [0.5, 0.0, 3.0]])
    rgb, seen = colorize.colorize_points(pts, [_view(_split_image())])
    assert rgb[0].tolist() == [255, 0, 0] and rgb[1].tolist() == [0, 0, 255]
    assert seen.tolist() == [1, 1]


def test_a_point_behind_a_surface_stays_uncoloured():
    xs = np.linspace(-1, 1, 81)
    X, Y = np.meshgrid(xs, xs)
    wall = np.stack([X.ravel(), Y.ravel(), np.full(X.size, 2.0)], 1)
    hidden = np.array([[0.0, 0.0, 5.0]])
    rgb, seen = colorize.colorize_points(np.vstack([wall, hidden]), [_view(_split_image())])
    assert seen[-1] == 0 and rgb[-1].tolist() == [0, 0, 0]


def test_points_behind_the_camera_or_too_far_are_unseen():
    pts = np.array([[0.0, 0.0, -3.0], [0.0, 0.0, 300.0]])
    _, seen = colorize.colorize_points(pts, [_view(_split_image())], max_dist=30.0)
    assert seen.tolist() == [0, 0]


def test_a_masked_pixel_colours_nothing():
    mask = np.full((H, W), 255, np.uint8)
    mask[:, : W // 2] = 0
    view = colorize.ColorView(_view(_split_image()).K, np.eye(4), W, H, _split_image(), mask)
    rgb, seen = colorize.colorize_points(np.array([[-0.5, 0.0, 3.0], [0.5, 0.0, 3.0]]), [view])
    assert seen.tolist() == [0, 1] and rgb[1].tolist() == [0, 0, 255]


def test_a_moved_camera_colours_from_its_pose():
    # The camera 10 m up world +z, looking back down -z (a half turn about y): camera x = -world x.
    w2c = np.eye(4)
    w2c[:3, :3] = np.diag([-1.0, 1.0, -1.0])
    w2c[:3, 3] = w2c[:3, :3] @ -np.array([0.0, 0.0, 10.0])
    view = colorize.ColorView(_view(_split_image()).K, w2c, W, H, _split_image())
    # World z = 7 is 3 m ahead; world x = +0.5 is camera x = -0.5 (the red half).
    rgb, seen = colorize.colorize_points(np.array([[0.5, 0.0, 7.0], [-0.5, 0.0, 7.0]]), [view])
    assert seen.tolist() == [1, 1]
    assert rgb[0].tolist() == [255, 0, 0] and rgb[1].tolist() == [0, 0, 255]
