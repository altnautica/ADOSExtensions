"""Appearance models, their schedules, the masked photometric loss and the depth prior term."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import losses as L  # noqa: E402


def test_affine_identity_init():
    torch.manual_seed(0)
    rgb = torch.rand(8, 6, 3)
    app = L.make_appearance("affine", 4, 1000, "cpu")
    assert isinstance(app.module, L.AffineAppearance)
    assert torch.allclose(app(rgb, 3), rgb)
    assert float(app.regulariser().detach()) == 0.0
    assert app.opt.param_groups[0]["lr"] == pytest.approx(1e-3) and app.sched is None
    with torch.no_grad():
        app.module.affine[1, 0, 3] = 0.1  # image 1 gets a red offset; the others stay identity
    assert torch.allclose(app(rgb, 1)[..., 0], rgb[..., 0] + 0.1)
    assert torch.allclose(app(rgb, 2), rgb)
    assert float(app.regulariser().detach()) == pytest.approx(1e-3 * 0.01 / 4)
    assert L.make_appearance("none", 4, 1000, "cpu") is None
    with pytest.raises(ValueError):
        L.make_appearance("bilateral", 4, 1000, "cpu")


def test_affine_learns_a_per_image_exposure():
    torch.manual_seed(0)
    rgb = torch.rand(16, 16, 3)
    gt = 0.7 * rgb + 0.05
    app = L.make_appearance("affine", 2, 1000, "cpu")
    for g in app.opt.param_groups:
        g["lr"] = 1e-2
    for _ in range(500):
        loss = (app(rgb, 0) - gt).abs().mean()
        app.opt.zero_grad()
        loss.backward()
        app.opt.step()
    assert float((app(rgb, 0) - gt).abs().mean().detach()) < 0.01
    assert torch.allclose(app(rgb, 1), rgb)  # the other image's affine is untouched


def test_grid_lr_schedule_values():
    steps = 30000
    assert L.grid_lr_factor(0, steps) == pytest.approx(0.01)
    assert L.grid_lr_factor(500, steps) == pytest.approx(0.505)
    assert L.grid_lr_factor(1000, steps) == pytest.approx(1.0)
    assert L.grid_lr_factor(steps - 1, steps) == pytest.approx(0.01)
    mid = 1000 + (steps - 1 - 1000) // 2
    assert L.grid_lr_factor(mid, steps) == pytest.approx(0.1, rel=1e-3)
    app = L.make_appearance("grid", 2, steps, "cpu")
    lrs = []
    for s in range(1002):
        lrs.append(app.opt.param_groups[0]["lr"])
        app.opt.step()
        app.sched.step()
    assert lrs[0] == pytest.approx(2e-5) and lrs[1000] == pytest.approx(2e-3)
    assert lrs[1001] < lrs[1000]


def test_masked_ssim_ignores_masked_content():
    torch.manual_seed(0)
    gt = torch.rand(32, 32, 3)
    render = gt.clone()
    mask = torch.ones(32, 32)
    mask[:, 16:] = 0
    render[:, 16:] = torch.rand(32, 16, 3)  # garbage only where the mask is off
    assert float(L.photometric_loss(render, gt, mask)) == pytest.approx(0.0, abs=1e-6)
    assert float(L.photometric_loss(render, gt, None)) > 0.05


def test_depth_prior_is_inverse_depth_edge_aware_and_decays():
    h, w = 32, 32
    prior = torch.full((h, w), 2.0)
    alpha = torch.ones(h, w)
    rendered = torch.full((h, w), 4.0)  # twice the prior everywhere: the fitted scale absorbs it
    assert float(L.depth_loss(rendered, prior, alpha, None)) == pytest.approx(0.0, abs=1e-6)
    rendered = rendered.clone()
    rendered[:, 16:] = 8.0  # half the view is twice as far as the prior says
    # Scale = median(r / p) is 2 or 4; either way |1/r - 1/(s p)| is 1/8 on the wrong half.
    plain = float(L.depth_loss(rendered, prior, alpha, None))
    assert plain == pytest.approx(0.0625, abs=1e-6)
    gt = torch.zeros(h, w, 3)
    gt[:, 21:] = 1.0  # a hard colour edge inside the wrong half: its pixels count less
    edge = L.image_edge_weight(gt)
    assert float(edge[0, 20]) == pytest.approx(math.exp(-1.0)) and float(edge[0, 0]) == pytest.approx(1.0)
    assert float(L.depth_loss(rendered, prior, alpha, None, gt)) < plain
    assert L.depth_weight(0, 1000) == pytest.approx(0.05)
    assert L.depth_weight(999, 1000) == pytest.approx(0.005)
    assert L.depth_weight(500, 1001) == pytest.approx(math.sqrt(0.05 * 0.005))
