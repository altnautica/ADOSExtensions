"""Monocular depth priors are used only where they agree with the model's own sparse points."""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common  # noqa: E402
from train import depth_prior_scale  # noqa: E402


def _scene(rng, n=400):
    xy = np.column_stack([rng.uniform(0, 64, n), rng.uniform(0, 48, n)])
    z = rng.uniform(1.0, 6.0, n)
    prior = np.zeros((48, 64), np.float32)
    prior[np.floor(xy[:, 1]).astype(int), np.floor(xy[:, 0]).astype(int)] = z
    return xy, z, prior


def test_an_agreeing_prior_gets_the_model_scale():
    xy, z, prior = _scene(np.random.default_rng(0))
    # A metric prior on a model at 0.4 units per metre.
    s = depth_prior_scale(prior, xy, 0.4 * z)
    assert s is not None and abs(s - 0.4) < 1e-6


def test_a_disagreeing_or_thin_prior_is_dropped():
    rng = np.random.default_rng(1)
    xy, z, prior = _scene(rng)
    noisy = prior * rng.uniform(0.3, 1.7, prior.shape).astype(np.float32)
    assert depth_prior_scale(noisy, xy, z) is None
    assert depth_prior_scale(prior, xy[:10], z[:10]) is None
    assert depth_prior_scale(prior, None, None) is None


def test_views_carry_their_sparse_observations(tmp_path):
    sp = tmp_path / "sparse" / "0"
    sp.mkdir(parents=True)
    with open(sp / "cameras.bin", "wb") as f:
        f.write(struct.pack("<Q", 1) + struct.pack("<iiQQ", 1, 1, 64, 48) + struct.pack("<4d", 40, 40, 32, 24))
    with open(sp / "images.bin", "wb") as f:
        f.write(struct.pack("<Q", 1) + struct.pack("<idddddddi", 1, 1, 0, 0, 0, 0, 0, 2, 1) + b"cam0/a.png\x00")
        f.write(struct.pack("<Q", 2) + struct.pack("<ddq", 10.5, 20.5, 7) + struct.pack("<ddq", 1.0, 2.0, -1))
    with open(sp / "points3D.bin", "wb") as f:
        f.write(struct.pack("<Q", 1) + struct.pack("<QdddBBBd", 7, 0.1, 0.2, 3.0, 1, 2, 3, 0.5) + struct.pack("<Q", 0))
    views, pts, _ = common.load_colmap_views(tmp_path)
    v = views[0]
    assert np.allclose(v.obs_xy, [[10.5, 20.5]]) and np.allclose(v.obs_xyz, [[0.1, 0.2, 3.0]])
    z = v.obs_xyz @ v.w2c[:3, :3].T[:, 2] + v.w2c[2, 3]
    assert np.allclose(z, [5.0])
