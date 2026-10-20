"""Lost gaussians (non-finite, far, huge) are recycled and not exported; MCMC noise is bounded."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common  # noqa: E402
import train as T  # noqa: E402


def _params(n=6):
    torch.manual_seed(0)
    return torch.nn.ParameterDict({
        "means": torch.nn.Parameter(torch.zeros(n, 3)),
        "scales": torch.nn.Parameter(torch.full((n, 3), math.log(0.01))),
        "quats": torch.nn.Parameter(torch.tensor([[1.0, 0, 0, 0]]).repeat(n, 1)),
        "opacities": torch.nn.Parameter(torch.zeros(n)),
        "sh0": torch.nn.Parameter(torch.zeros(n, 1, 3)),
        "shN": torch.nn.Parameter(torch.zeros(n, 15, 3)),
    })


def test_non_finite_far_and_huge_gaussians_are_invalid():
    p = _params()
    with torch.no_grad():
        p["means"][1, 0] = float("nan")
        p["means"][2] = torch.tensor([T.MAX_DISTANCE * 2.0 + 0.1, 0, 0])
        p["scales"][3, 2] = math.log(T.MAX_SIZE * 2.0 * 1.01)
        p["sh0"][4, 0, 1] = float("inf")
        p["means"][5] = torch.tensor([T.MAX_DISTANCE * 2.0 - 0.1, 0, 0])
    assert T.invalid_gaussians(p, scene_scale=2.0).tolist() == [False, True, True, True, True, False]


def test_killed_gaussians_are_dead_finite_and_forget_their_moments():
    p = _params()
    opts = T.make_optimizers(p, 1.0)
    (p["means"].sum() + p["opacities"].sum()).backward()
    for o in opts.values():
        o.step()
    with torch.no_grad():
        p["means"][1] = float("nan")
    mask = torch.tensor([False, True, False, False, False, True])
    assert T.kill_gaussians(p, opts, mask) == 2
    assert torch.isfinite(torch.cat([v.reshape(len(v), -1) for v in p.values()], 1)).all()
    op = torch.sigmoid(p["opacities"])
    assert torch.allclose(op[mask], torch.tensor(T.DEAD_OPACITY))
    assert (op[mask] < 0.005).all() and (op[~mask] > 0.4).all()  # below MCMC's relocation threshold
    for o in opts.values():
        for st in o.state.values():
            assert (st["exp_avg"][mask] == 0).all() and (st["exp_avg_sq"][mask] == 0).all()
            assert (st["exp_avg"][~mask] != 0).any()


def test_noise_moves_only_faint_visible_gaussians_and_is_clamped():
    p = _params(4)
    with torch.no_grad():
        p["opacities"][:] = torch.logit(torch.tensor([0.001, 0.001, 0.001, 0.9]))
        p["scales"][0] = math.log(10.0)  # huge and faint: unclamped noise would be ~1e2 * 5e5 * lr
        p["scales"][1] = math.log(1e-3)
    visible = torch.tensor([True, True, False, True])
    T.mcmc_noise(p, visible, scaler=5e5 * 1.6e-4, max_step=0.05)
    m = p["means"].detach()
    assert (m[0].abs() <= 0.05 + 1e-7).all() and (m[0].abs() > 0).any()
    assert (m[1].abs() < 1e-3).all() and (m[1].abs() > 0).any()  # small: covariance-shaped
    assert (m[2] == 0).all()  # not visible
    assert (m[3].abs() < 1e-6).all()  # opaque


def test_noise_covariance_follows_the_rotation():
    """Anisotropic gaussian rotated 90 degrees about z: its long axis (x) becomes y."""
    p = _params(1)
    with torch.no_grad():
        p["opacities"][:] = -20.0
        p["scales"][0] = torch.log(torch.tensor([0.1, 1e-4, 1e-4]))
        p["quats"][0] = torch.tensor([math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])
    torch.manual_seed(1)
    T.mcmc_noise(p, torch.tensor([True]), scaler=1.0, max_step=1.0)
    m = p["means"].detach()[0].abs()
    assert m[1] > 100 * m[0] and m[1] > 100 * m[2]


def test_far_sparse_points_are_not_used_and_initial_size_is_a_tenth_of_the_spacing():
    rng = np.random.default_rng(0)
    K = np.eye(3)
    views = []
    for x in (-1.0, 1.0):
        w2c = np.eye(4)
        w2c[0, 3] = -x
        views.append(common.View("a.png", w2c, K, 4, 4))
    grid = np.stack(np.meshgrid(*[np.arange(10) * 0.1] * 3), -1).reshape(-1, 3).astype(np.float32)
    far = np.array([[500.0, 0, 0], [np.nan, 0, 0]], np.float32)
    pts = np.concatenate([grid, far])
    params, scene_scale = T.init_params(pts, rng.uniform(0, 1, (len(pts), 3)).astype(np.float32), views, 10**6, "cpu")
    assert math.isclose(scene_scale, 1.1)
    assert len(params["means"]) == len(grid)
    assert np.allclose(np.exp(params["scales"].detach().numpy()).max(), 0.1 * 0.1, rtol=0.3)


def test_colour_correction_recovers_an_affine_colour_change():
    torch.manual_seed(0)
    gt = torch.rand(16, 16, 3)
    a = torch.tensor([[0.8, 0.1, 0.0], [0.0, 0.7, 0.1], [0.05, 0.0, 0.9]])
    render = (gt - 0.05) @ torch.linalg.inv(a)  # gt = render @ a + 0.05
    out = T.colour_corrected(render, gt)
    assert torch.allclose(out, gt, atol=1e-4)
    mask = torch.zeros(16, 16)
    mask[:8] = 1
    noisy = render.clone()
    noisy[8:] = 0.5
    assert torch.allclose(T.colour_corrected(noisy, gt, mask)[:8], gt[:8], atol=1e-4)
