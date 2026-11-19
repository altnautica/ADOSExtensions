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
    """(1-λ)·L1 + λ·(1-SSIM), both averaged over pixels with weight > 0. Both images are zeroed
    where the weight is 0 before SSIM, so masked content does not leak into it through the window."""
    l1 = (render - gt).abs().mean(-1)
    if weight is None:
        return (1 - ssim_lambda) * l1.mean() + ssim_lambda * (1.0 - ssim_map(render, gt)).mean()
    keep = (weight > 0).to(render.dtype)[..., None]
    ds = 1.0 - ssim_map(render * keep, gt * keep)
    denom = weight.sum().clamp_min(1.0)
    return ((1 - ssim_lambda) * (l1 * weight).sum() + ssim_lambda * (ds * weight).sum()) / denom


DEPTH_WEIGHT_START = 0.05
DEPTH_WEIGHT_END = 0.005


def depth_weight(step: int, steps: int) -> float:
    """Depth-prior weight: DEPTH_WEIGHT_START at step 0 decaying exponentially to DEPTH_WEIGHT_END
    at the last step."""
    f = step / max(1, steps - 1)
    return DEPTH_WEIGHT_START * (DEPTH_WEIGHT_END / DEPTH_WEIGHT_START) ** f


def image_edge_weight(gt: torch.Tensor) -> torch.Tensor:
    """[H,W] exp(-|∇I|) of an [H,W,3] image: low across colour edges, where a depth prior's
    boundaries are least trustworthy."""
    gx = torch.zeros_like(gt)
    gy = torch.zeros_like(gt)
    gx[:, :-1] = (gt[:, 1:] - gt[:, :-1]).abs()
    gy[:-1] = (gt[1:] - gt[:-1]).abs()
    return torch.exp(-(gx + gy).mean(-1))


def depth_loss(rendered: torch.Tensor, prior: torch.Tensor, alpha: torch.Tensor, weight: torch.Tensor | None,
               gt: torch.Tensor | None = None) -> torch.Tensor:
    """Edge-aware L1 between the rendered and the prior inverse depth.

    The prior is metric but its scale can differ from the reconstruction's, so a single
    detached median scale per image is fitted before comparing. Inverse depth weighs near
    surfaces most, where the prior is reliable, and stays bounded for far ones; `gt` (the
    training image) down-weights pixels on colour edges.
    """
    valid = (prior > 0) & (alpha > 0.5) & (rendered > 0)
    if weight is not None:
        valid &= weight > 0
    if valid.sum() < 64:
        return rendered.sum() * 0.0
    r, p = rendered[valid], prior[valid]
    scale = torch.median(r.detach() / p)
    err = (1.0 / r.clamp_min(1e-6) - 1.0 / (p * scale).clamp_min(1e-6)).abs()
    if gt is None:
        return err.mean()
    w = image_edge_weight(gt)[valid]
    return (err * w).sum() / w.sum().clamp_min(1e-6)


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


APPEARANCES = ("affine", "grid", "none")
AFFINE_LR = 1e-3
AFFINE_PULL = 1e-3  # weight of the L2 pull of every image's affine to identity
GRID_LR = 2e-3
GRID_WARMUP = 1000
GRID_LR_FLOOR = 0.01  # the grid lr starts at and decays to this fraction of GRID_LR
GRID_TV = 10.0


def grid_lr_factor(step: int, steps: int) -> float:
    """Bilateral-grid lr as a fraction of GRID_LR: a linear warm-up from GRID_LR_FLOOR to 1 over
    GRID_WARMUP steps (the grids must not learn the images before the gaussians do), then an
    exponential decay to GRID_LR_FLOOR at the last step."""
    warm = min(GRID_WARMUP, max(1, steps - 1))
    if step < warm:
        return GRID_LR_FLOOR + (1.0 - GRID_LR_FLOOR) * step / warm
    span = max(1, steps - 1 - warm)
    return GRID_LR_FLOOR ** (min(step - warm, span) / span)


def _identity_affine(device=None) -> torch.Tensor:
    return torch.cat([torch.eye(3, device=device), torch.zeros(3, 1, device=device)], 1)


class AffineAppearance(torch.nn.Module):
    """One 3x4 colour affine per training image (identity at start): absorbs per-image exposure
    and white balance without the spatial freedom of a bilateral grid."""

    def __init__(self, num_images: int):
        super().__init__()
        self.affine = torch.nn.Parameter(_identity_affine()[None].repeat(num_images, 1, 1))

    def forward(self, rgb: torch.Tensor, image_index: int) -> torch.Tensor:
        a = self.affine[image_index]
        return rgb @ a[:, :3].T + a[:, 3]

    def regulariser(self) -> torch.Tensor:
        """Mean squared distance of the affines from identity."""
        return (self.affine - _identity_affine(self.affine.device)).pow(2).sum(dim=(1, 2)).mean()


class Appearance:
    """A per-image appearance model with its optimiser, lr schedule and regulariser weight."""

    def __init__(self, kind: str, num_images: int, steps: int, device):
        if kind == "affine":
            self.module = AffineAppearance(num_images).to(device)
            self.opt = torch.optim.Adam(self.module.parameters(), lr=AFFINE_LR, eps=1e-15)
            self.sched = None
            self.weight = AFFINE_PULL
        elif kind == "grid":
            self.module = BilateralGrid(num_images).to(device)
            self.opt = torch.optim.Adam(self.module.parameters(), lr=GRID_LR, eps=1e-15)
            self.sched = torch.optim.lr_scheduler.LambdaLR(self.opt, lambda s: grid_lr_factor(s, steps))
            self.weight = GRID_TV
        else:
            raise ValueError(f"unknown appearance {kind!r}; expected one of {APPEARANCES}")

    def __call__(self, rgb: torch.Tensor, image_index: int) -> torch.Tensor:
        return self.module(rgb, image_index)

    def regulariser(self) -> torch.Tensor:
        m = self.module
        return self.weight * (m.total_variation() if isinstance(m, BilateralGrid) else m.regulariser())


def make_appearance(kind: str, num_images: int, steps: int, device) -> Appearance | None:
    """The appearance model for `kind` (one of APPEARANCES), None for `none`."""
    return None if kind == "none" else Appearance(kind, num_images, steps, device)
