"""Lost gaussians (non-finite, far, huge) are recycled and not exported; MCMC noise is bounded."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common  # noqa: E402
import strategy as S  # noqa: E402
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
        p["means"][2] = torch.tensor([S.MAX_DISTANCE * 2.0 + 0.1, 0, 0])
        p["scales"][3, 2] = math.log(S.MAX_SIZE * 2.0 * 1.01)
        p["sh0"][4, 0, 1] = float("inf")
        p["means"][5] = torch.tensor([S.MAX_DISTANCE * 2.0 - 0.1, 0, 0])
    assert S.invalid_gaussians(p, scene_scale=2.0).tolist() == [False, True, True, True, True, False]


def test_kill_pass_marks_nonfinite_huge_far():
    """At a refine step the strategy kills the lost gaussians (also after refinement stopped, so
    they stay dead) and leaves the healthy ones alone."""
    p = _params()
    opts = T.make_optimizers(p, 1.0)
    with torch.no_grad():
        p["opacities"][:] = 2.0
        p["means"][1, 2] = float("nan")
        p["scales"][2] = math.log(5.0)
        p["means"][3] = torch.tensor([0.0, 60.0, 0.0])
    strat = S.BoundedMCMCStrategy(cap_max=6, refine_start_iter=0, refine_stop_iter=150, refine_every=100,
                                  scene_scale=1.0)
    state = strat.initialize_state()
    info = {"radii": torch.ones(1, 6, 2, dtype=torch.int32)}
    strat.step_post_backward(p, opts, state, step=200, info=info, lr=0.0)
    assert state["killed"] == 3
    op = torch.sigmoid(p["opacities"]).detach()
    assert torch.allclose(op[1:4], torch.tensor(S.DEAD_OPACITY)) and (op[[0, 4, 5]] > 0.8).all()
    assert torch.isfinite(p["means"]).all() and not S.invalid_gaussians(p, 1.0).any()


def test_killed_gaussians_are_dead_finite_and_forget_their_moments():
    p = _params()
    opts = T.make_optimizers(p, 1.0)
    (p["means"].sum() + p["opacities"].sum()).backward()
    for o in opts.values():
        o.step()
    with torch.no_grad():
        p["means"][1] = float("nan")
    mask = torch.tensor([False, True, False, False, False, True])
    assert S.kill_gaussians(p, opts, mask) == 2
    assert torch.isfinite(torch.cat([v.reshape(len(v), -1) for v in p.values()], 1)).all()
    op = torch.sigmoid(p["opacities"])
    assert torch.allclose(op[mask], torch.tensor(S.DEAD_OPACITY))
    assert (op[mask] < 0.005).all() and (op[~mask] > 0.4).all()  # below MCMC's relocation threshold
    for o in opts.values():
        for st in o.state.values():
            assert (st["exp_avg"][mask] == 0).all() and (st["exp_avg_sq"][mask] == 0).all()
            assert (st["exp_avg"][~mask] != 0).any()


def test_bounded_noise_is_clamped_and_visible_only():
    p = _params(4)
    with torch.no_grad():
        p["opacities"][:] = torch.logit(torch.tensor([0.001, 0.001, 0.001, 0.9]))
        p["scales"][0] = math.log(10.0)  # huge and faint: unclamped noise would be ~1e2 * 5e5 * lr
        p["scales"][1] = math.log(1e-3)
    visible = torch.tensor([True, True, False, True])
    S.mcmc_noise(p, visible, scaler=5e5 * 1.6e-4, max_step=0.05)
    m = p["means"].detach()
    assert (m[0].abs() <= 0.05 + 1e-7).all() and (m[0].abs() > 0).any()
    assert (m[1].abs() < 1e-3).all() and (m[1].abs() > 0).any()  # small: covariance-shaped
    assert (m[2] == 0).all()  # not visible
    assert (m[3].abs() < 1e-6).all()  # opaque


def test_noise_follows_the_render_and_stops_with_refinement():
    p = _params(3)
    with torch.no_grad():
        p["opacities"][:] = -20.0
        p["scales"][:] = math.log(0.5)
    opts = T.make_optimizers(p, 1.0)
    strat = S.BoundedMCMCStrategy(cap_max=3, refine_start_iter=0, refine_stop_iter=100, refine_every=1000,
                                  scene_scale=1.0)
    state = strat.initialize_state()
    # The last render projected only the first two gaussians.
    info = {"radii": torch.tensor([[[3, 3], [2, 2], [0, 0]]], dtype=torch.int32)}
    strat.step_post_backward(p, opts, state, step=5, info=info, lr=1.0)
    m = p["means"].detach().clone()
    assert (m[:2].abs() > 0).all() and (m[2] == 0).all()
    assert (m.abs() <= S.MAX_NOISE_STEP + 1e-7).all()
    strat.step_post_backward(p, opts, state, step=100, info=info, lr=1.0)
    assert torch.equal(p["means"].detach(), m)


def test_noise_covariance_follows_the_rotation():
    """Anisotropic gaussian rotated 90 degrees about z: its long axis (x) becomes y."""
    p = _params(1)
    with torch.no_grad():
        p["opacities"][:] = -20.0
        p["scales"][0] = torch.log(torch.tensor([0.1, 1e-4, 1e-4]))
        p["quats"][0] = torch.tensor([math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])
    torch.manual_seed(1)
    S.mcmc_noise(p, torch.tensor([True]), scaler=1.0, max_step=1.0)
    m = p["means"].detach()[0].abs()
    assert m[1] > 100 * m[0] and m[1] > 100 * m[2]


def _two_views():
    views = []
    for x in (-1.0, 1.0):
        w2c = np.eye(4)
        w2c[0, 3] = -x
        views.append(common.View("a.png", w2c, np.eye(3), 4, 4))
    return views


def test_far_sparse_points_are_not_used_and_initial_size_is_a_tenth_of_the_spacing():
    rng = np.random.default_rng(0)
    grid = np.stack(np.meshgrid(*[np.arange(10) * 0.1] * 3), -1).reshape(-1, 3).astype(np.float32)
    # Beyond the bound, non-finite, and a far cluster inside the bound but past the 99th percentile.
    far = np.array([[500.0, 0, 0], [np.nan, 0, 0]] + [[10.0, 0, 0]] * 5, np.float32)
    pts = np.concatenate([grid, far])
    params, scene_scale = T.init_params(pts, rng.uniform(0, 1, (len(pts), 3)).astype(np.float32), _two_views(),
                                        10**6, "cpu")
    assert math.isclose(scene_scale, 1.1)
    means = params["means"].detach().numpy()
    assert len(grid) - 15 <= len(means) <= len(grid)
    assert np.linalg.norm(means, axis=1).max() < 2.0  # the cluster at 10 m is gone
    assert np.allclose(np.exp(params["scales"].detach().numpy()).max(), 0.1 * 0.1, rtol=0.3)


def test_export_cull_removes_floaters(tmp_path):
    """A gaussian a million metres out, a non-finite one and an invisible one are not written;
    the written ones are back in the model frame."""
    p = _params(5)
    with torch.no_grad():
        p["opacities"][:] = 2.0
        p["opacities"][4] = math.log((0.5 / 255) / (1 - 0.5 / 255))
        p["means"][0] = torch.tensor([0.5, 0.0, 0.0])
        p["means"][1] = torch.tensor([1e6, 0.0, 0.0])
        p["scales"][2, 0] = float("nan")
    centre, norm = np.array([10.0, 0.0, 0.0]), 0.5
    ply = tmp_path / "splat.ply"
    assert T.export_splat(ply, p, 1.1, centre, norm) == (2, 3)
    out = common.read_splat_ply(ply)
    assert len(out["means"]) == 2
    assert np.abs(out["means"]).max() < 100.0
    assert np.allclose(out["means"][0], [11.0, 0.0, 0.0])
    assert np.allclose(out["scales"][0], math.log(0.01) - math.log(0.5), atol=1e-5)


def test_nan_loss_skips_step():
    p = _params(2)
    opts = T.make_optimizers(p, 1.0)
    before = {k: v.detach().clone() for k, v in p.items()}
    loss = p["means"].sum() * float("nan")
    assert not T.apply_step(loss, list(opts.values()))
    for k, v in p.items():
        assert torch.equal(v.detach(), before[k]) and v.grad is None
    assert all(not o.state for o in opts.values())  # Adam moments untouched
    assert T.apply_step((p["means"] - 1.0).pow(2).sum(), list(opts.values()))
    assert not torch.equal(p["means"].detach(), before["means"])


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
