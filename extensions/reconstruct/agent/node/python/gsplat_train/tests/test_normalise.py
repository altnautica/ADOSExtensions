"""Training runs in a normalised frame whatever the scale and offset of the SfM model."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common  # noqa: E402
from train import denormalise_splats, normalise_views, scene_normalisation  # noqa: E402


def _rot(rng) -> np.ndarray:
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    return q * np.sign(np.linalg.det(q))


def _views(rng, offset, size, n=12):
    """Cameras along a walk of `size` units starting near `offset`, each seeing a few points."""
    K = np.array([[500.0, 0, 320], [0, 500.0, 200], [0, 0, 1]])
    views = []
    for i in range(n):
        R = _rot(rng)
        c = offset + size * np.array([i / n, np.sin(i), 0.1 * i / n])
        w2c = np.eye(4)
        w2c[:3, :3] = R
        w2c[:3, 3] = -R @ c
        pts = c + (np.array([[0.0, 0.0, 2.0], [0.3, -0.2, 4.0], [-1.0, 0.5, 9.0]]) * size / 10) @ R
        views.append(common.View(f"cam0/{i:03d}.png", w2c, K, 640, 400, None, None, pts))
    return views


def _project(v, X):
    cam = X @ v.w2c[:3, :3].T + v.w2c[:3, 3]
    uv = cam @ v.K.T
    return uv[:, :2] / uv[:, 2:], cam[:, 2]


def test_cameras_end_up_centred_at_unit_median_distance():
    views = _views(np.random.default_rng(0), np.array([5000.0, -300.0, 20.0]), 40.0)
    centre, s = scene_normalisation(views)
    out = normalise_views(views, centre, s)
    c = np.stack([np.linalg.inv(v.w2c)[:3, 3] for v in out])
    assert np.allclose(c.mean(0), 0, atol=1e-9)
    assert np.isclose(np.median(np.linalg.norm(c, axis=1)), 1.0)


def test_pixels_are_unchanged_and_depths_scale():
    views = _views(np.random.default_rng(1), np.array([12.0, 3.0, -7.0]), 25.0)
    centre, s = scene_normalisation(views)
    out = normalise_views(views, centre, s)
    for v, n in zip(views, out):
        uv, z = _project(v, v.obs_xyz)
        uv_n, z_n = _project(n, n.obs_xyz)
        assert np.allclose(uv, uv_n) and np.allclose(z * s, z_n)
        assert np.allclose(n.w2c[:3, :3], v.w2c[:3, :3])


def test_the_training_frame_does_not_depend_on_the_model_scale_or_offset():
    """A model 1000x larger and shifted trains on identical inputs, so learning rates and the MCMC
    position noise act the same; in the raw frame the noise grows with the square of the size."""
    a = _views(np.random.default_rng(2), np.zeros(3), 1.0)
    b = []
    for v in a:
        w2c = v.w2c.copy()
        c = 1000.0 * np.linalg.inv(v.w2c)[:3, 3] + np.array([3e4, -2e4, 50.0])
        w2c[:3, 3] = -w2c[:3, :3] @ c
        b.append(common.View(v.name, w2c, v.K, v.width, v.height, None, None,
                             1000.0 * v.obs_xyz + np.array([3e4, -2e4, 50.0])))
    na = normalise_views(a, *scene_normalisation(a))
    nb = normalise_views(b, *scene_normalisation(b))
    for x, y in zip(na, nb):
        assert np.allclose(x.w2c, y.w2c, atol=1e-6) and np.allclose(x.obs_xyz, y.obs_xyz, atol=1e-6)


def test_splats_are_written_back_in_the_model_frame():
    views = _views(np.random.default_rng(3), np.array([100.0, 0.0, 0.0]), 30.0)
    centre, s = scene_normalisation(views)
    means = np.array([[1.0, 2.0, 3.0], [-40.0, 7.0, 0.5]])
    log_scales = np.log(np.array([[0.05, 0.1, 0.2], [1.0, 1.0, 1.0]]))
    m_n, ls_n = s * (means - centre), log_scales + np.log(s)
    m, ls = denormalise_splats(m_n, ls_n, centre, s)
    assert np.allclose(m, means) and np.allclose(ls, log_scales)


def test_cameras_at_one_point_keep_a_finite_scale():
    v = _views(np.random.default_rng(4), np.zeros(3), 1.0, n=3)
    same = [common.View(x.name, v[0].w2c, x.K, x.width, x.height) for x in v]
    centre, s = scene_normalisation(same)
    assert np.isfinite(centre).all() and s == 1.0
