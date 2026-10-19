"""Photometric, depth and appearance terms used by train.py."""

from __future__ import annotations

import torch
import torch.nn.functional as F


def _gaussian_window(size: int = 11, sigma: float = 1.5, device=None) -> torch.Tensor:
    x = torch.arange(size, dtype=torch.float32, device=device) - size // 2
    g = torch.exp(-(x * x) / (2 * sigma * sigma))
    g = g / g.sum()
    return (g[:, None] @ g[None, :])[None, None].repeat(3, 1, 1, 1)


def ssim_map(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Per-pixel SSIM of two [H,W,3] images in 0..1, returned as [H,W]."""
    x = a.permute(2, 0, 1)[None]
    y = b.permute(2, 0, 1)[None]
    w = _gaussian_window(device=a.device)
    pad = w.shape[-1] // 2
    mu_x = F.conv2d(x, w, padding=pad, groups=3)
    mu_y = F.conv2d(y, w, padding=pad, groups=3)
    sxx = F.conv2d(x * x, w, padding=pad, groups=3) - mu_x * mu_x
    syy = F.conv2d(y * y, w, padding=pad, groups=3) - mu_y * mu_y
    sxy = F.conv2d(x * y, w, padding=pad, groups=3) - mu_x * mu_y
    c1, c2 = 0.01**2, 0.03**2
    s = ((2 * mu_x * mu_y + c1) * (2 * sxy + c2)) / ((mu_x * mu_x + mu_y * mu_y + c1) * (sxx + syy + c2))
    return s[0].mean(0)


def photometric_loss(render: torch.Tensor, gt: torch.Tensor, weight: torch.Tensor | None, ssim_lambda: float = 0.2) -> torch.Tensor:
    """(1-λ)·L1 + λ·(1-SSIM), both averaged over pixels with weight > 0."""
    l1 = (render - gt).abs().mean(-1)
    ds = 1.0 - ssim_map(render, gt)
    if weight is None:
        return (1 - ssim_lambda) * l1.mean() + ssim_lambda * ds.mean()
    denom = weight.sum().clamp_min(1.0)
    return ((1 - ssim_lambda) * (l1 * weight).sum() + ssim_lambda * (ds * weight).sum()) / denom


def depth_loss(rendered: torch.Tensor, prior: torch.Tensor, alpha: torch.Tensor, weight: torch.Tensor | None) -> torch.Tensor:
    """Relative L1 between rendered depth and a monocular prior aligned by a median scale.

    The prior is metric but its scale can differ from the reconstruction's, so a
    single detached scale per image is fitted before comparing.
    """
    valid = (prior > 0) & (alpha > 0.5) & (rendered > 0)
    if weight is not None:
        valid &= weight > 0
    if valid.sum() < 64:
        return rendered.sum() * 0.0
    r, p = rendered[valid], prior[valid]
    scale = torch.median(r.detach() / p)
    target = p * scale
    return ((r - target).abs() / target.clamp_min(1e-3)).mean()


class BilateralGrid(torch.nn.Module):
    """Per-image affine colour grids sliced by pixel position and luminance.

    Each image owns a grid of 3x4 affine colour transforms over
    (x, y, luminance). Slicing it trilinearly at every pixel and applying the
    transform absorbs per-image exposure, white balance and tone changes so the
    splat colours stay consistent across views.
    """

    def __init__(self, num_images: int, grid_xy: int = 16, grid_l: int = 8):
        super().__init__()
        ident = torch.tensor([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], dtype=torch.float32)
        grids = ident.view(1, 12, 1, 1, 1).repeat(num_images, 1, grid_l, grid_xy, grid_xy)
        self.grids = torch.nn.Parameter(grids)

    def forward(self, rgb: torch.Tensor, image_index: int) -> torch.Tensor:
        h, w, _ = rgb.shape
        lum = (rgb * torch.tensor([0.299, 0.587, 0.114], device=rgb.device)).sum(-1).clamp(0, 1)
        ys, xs = torch.meshgrid(
            torch.linspace(-1, 1, h, device=rgb.device), torch.linspace(-1, 1, w, device=rgb.device), indexing="ij"
        )
        coords = torch.stack([xs, ys, lum * 2 - 1], -1)[None, None]  # [1,1,H,W,3] (x->W, y->H, z->L)
        grid = self.grids[image_index : image_index + 1]
        affine = F.grid_sample(grid, coords, mode="bilinear", align_corners=True)  # [1,12,1,H,W]
        affine = affine[0, :, 0].permute(1, 2, 0).reshape(h, w, 3, 4)
        return (affine[..., :3] @ rgb[..., None]).squeeze(-1) + affine[..., 3]

    def total_variation(self) -> torch.Tensor:
        g = self.grids
        tv = (g[:, :, 1:] - g[:, :, :-1]).pow(2).mean()
        tv = tv + (g[:, :, :, 1:] - g[:, :, :, :-1]).pow(2).mean()
        return tv + (g[:, :, :, :, 1:] - g[:, :, :, :, :-1]).pow(2).mean()
