"""gsplat's MCMC densification with its position noise bounded and lost gaussians recycled.

Bounds are multiples of scene_scale (1.1 x the largest camera distance from the cameras'
centroid, in the normalised training frame). A gaussian outside them is a lost one: it is
recycled by MCMC relocation during training and not exported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch
from gsplat.strategy import MCMCStrategy

MAX_DISTANCE = 20.0  # its mean is farther than this from the cameras' centroid
MAX_SIZE = 1.0  # its largest axis is longer than the camera spread
MAX_NOISE_STEP = 0.01  # largest MCMC position-noise move of one gaussian in one step
DEAD_OPACITY = 0.001  # below MCMCStrategy.min_opacity, so relocation refills the slot


def invalid_gaussians(params, scene_scale: float) -> torch.Tensor:
    """[N] bool: gaussians with a non-finite parameter, a mean more than MAX_DISTANCE x scene_scale
    from the origin (the cameras' centroid) or a largest axis above MAX_SIZE x scene_scale."""
    with torch.no_grad():
        means = params["means"]
        bad = torch.zeros(len(means), dtype=torch.bool, device=means.device)
        for v in params.values():
            bad |= ~torch.isfinite(v.reshape(len(v), -1)).all(1)
        bad |= means.norm(dim=1) > MAX_DISTANCE * scene_scale
        bad |= params["scales"].max(1).values > math.log(MAX_SIZE * scene_scale)
    return bad


def kill_gaussians(params, optimizers, mask) -> int:
    """Make the `mask` gaussians dead (opacity DEAD_OPACITY, finite placeholder values, Adam moments
    cleared): invisible, and refilled from live gaussians at the next MCMC relocation."""
    n = int(mask.sum())
    if n == 0:
        return 0
    with torch.no_grad():
        params["means"][mask] = 0.0
        params["scales"][mask] = math.log(1e-6)
        params["quats"][mask] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=mask.device)
        params["opacities"][mask] = math.log(DEAD_OPACITY / (1.0 - DEAD_OPACITY))
        params["sh0"][mask] = 0.0
        params["shN"][mask] = 0.0
        for opt in optimizers.values():
            for st in opt.state.values():
                for key in ("exp_avg", "exp_avg_sq"):
                    if key in st:
                        st[key][mask] = 0.0
    return n


def mcmc_noise(params, visible, scaler: float, max_step: float) -> None:
    """MCMC position noise (gsplat's inject_noise_to_position: covariance-shaped, on faint
    gaussians), applied only to gaussians visible this step and clamped to `max_step` per axis.
    Unclamped, a large faint gaussian jumps many scene sizes in one step."""
    with torch.no_grad():
        op = torch.sigmoid(params["opacities"].flatten())
        s2 = torch.exp(2.0 * params["scales"])
        w, x, y, z = torch.nn.functional.normalize(params["quats"], dim=-1).unbind(-1)
        R = torch.stack([
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ], -1).reshape(-1, 3, 3)
        covar = (R * s2[:, None, :]) @ R.transpose(1, 2)
        gate = 1.0 / (1.0 + torch.exp(-100.0 * ((1.0 - op) - 0.995)))
        noise = torch.randn_like(params["means"]) * gate[:, None] * scaler
        noise = torch.einsum("bij,bj->bi", covar, noise).clamp(-max_step, max_step)
        noise[~visible] = 0.0
        params["means"].add_(noise)


def visible_gaussians(info, n: int) -> torch.Tensor:
    """[n] bool: gaussians the last render projected into the view (gaussians added after that
    render count as not visible)."""
    vis = (info["radii"] > 0).all(-1).reshape(-1, info["radii"].shape[-2]).any(0)
    if len(vis) < n:
        vis = torch.cat([vis, torch.zeros(n - len(vis), dtype=torch.bool, device=vis.device)])
    return vis


@dataclass
class BoundedMCMCStrategy(MCMCStrategy):
    """MCMCStrategy with the position noise visible-only, clamped to MAX_NOISE_STEP x scene_scale
    and stopped when refinement stops, and lost gaussians (invalid_gaussians) killed before every
    relocation so it reuses them; after refinement stops they stay dead and are not exported."""

    scene_scale: float = 1.0

    @classmethod
    def for_run(cls, cap_max: int, steps: int, scene_scale: float) -> "BoundedMCMCStrategy":
        return cls(
            cap_max=cap_max,
            refine_start_iter=min(500, steps // 10),
            refine_stop_iter=int(steps * 0.83),
            refine_every=100,
            scene_scale=scene_scale,
        )

    def kill_lost(self, params, optimizers, state) -> int:
        n = kill_gaussians(params, optimizers, invalid_gaussians(params, self.scene_scale))
        state["killed"] = state.get("killed", 0) + n
        return n

    def step_post_backward(self, params, optimizers, state, step, info, lr):
        state["binoms"] = state["binoms"].to(params["means"].device)
        if step > self.refine_start_iter and step % self.refine_every == 0:
            self.kill_lost(params, optimizers, state)
            if step < self.refine_stop_iter:
                self._relocate_gs(params, optimizers, state["binoms"])
                self._add_new_gs(params, optimizers, state["binoms"])
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
        if step < self.refine_stop_iter:
            mcmc_noise(params, visible_gaussians(info, len(params["means"])), lr * self.noise_lr,
                       MAX_NOISE_STEP * self.scene_scale)
