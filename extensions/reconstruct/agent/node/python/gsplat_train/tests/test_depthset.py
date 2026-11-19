"""Depth set round trip: colour, metric depth and pose survive write and read."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import depthset  # noqa: E402


def test_round_trip(tmp_path):
    rgb = np.zeros((4, 6, 3), np.uint8)
    rgb[..., 0] = 200
    d = np.full((4, 6), 2.5, np.float32)
    K = np.array([[5.0, 0, 3], [0, 5.0, 2], [0, 0, 1]])
    w2c = np.eye(4)
    w2c[:3, 3] = [0.1, -0.2, 0.3]
    v = depthset.save_view(tmp_path, 0, "front/1.png", rgb, d, K, w2c)
    depthset.write_depthset(tmp_path, [v], frame="model")
    frame, views = depthset.read_depthset(tmp_path)
    assert frame == "model" and views[0].name == "front/1.png"
    assert (views[0].width, views[0].height) == (6, 4)
    assert np.allclose(views[0].K, K) and np.allclose(views[0].w2c, w2c)
    assert depthset.load_rgb(tmp_path, views[0])[0, 0].tolist() == [200, 0, 0]
    depth = depthset.load_depth(tmp_path, views[0])
    assert depth.dtype == np.float32 and float(depth[0, 0]) == 2.5
