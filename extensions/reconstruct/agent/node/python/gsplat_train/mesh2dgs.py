"""The mesh's 2DGS pass: a second training run with 2D gaussians (surfels, gsplat's
rasterization_2dgs) over the same views, whose median depth mesh.py fuses with its TSDF.

Surfels lie flat on surfaces, so their depth is sharper than a 3D splat's. A normal-consistency
term (rendered normals against the normals of the rendered depth) and a depth-distortion term
(weights concentrated along each ray) pull them onto one surface; both start once the colours
have settled. Densification, bounds, appearance and the normalised frame are train.py's.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

import common

NORMAL_WEIGHT = 0.05
NORMAL_FROM = 7000
DIST_WEIGHT = 0.01
DIST_FROM = 3000


def surfel_terms(normals, normals_from_depth, alpha, distort):
    """(normal-consistency error, depth distortion) of one 2DGS render, each a mean over pixels.
    The depth normals are weighted by the (detached) alpha, so empty pixels do not count."""
    nd = normals_from_depth.reshape(normals.shape) * alpha.detach()[..., None]
    return (1.0 - (normals * nd).sum(-1)).mean(), distort.mean()


def surfel_weights(step: int) -> tuple[float, float]:
    """(normal-consistency weight, distortion weight) at `step`."""
    return (NORMAL_WEIGHT if step >= NORMAL_FROM else 0.0), (DIST_WEIGHT if step >= DIST_FROM else 0.0)


def render_2dgs(params, w2c, K, width: int, height: int, sh_degree: int, device, distloss: bool = False):
    """(rgb HxWx3, alpha HxW, normals HxWx3, normals from depth, distortion HxW, median depth HxW,
    info) of the surfels `params` (log scales, logit opacities) from one view."""
    import torch
    from gsplat import rasterization_2dgs

    viewmat = torch.as_tensor(w2c, dtype=torch.float32, device=device)[None]
    Ks = torch.as_tensor(K, dtype=torch.float32, device=device)[None]
    # The normals of the rendered depth exist only when depth is rendered.
    out, alpha, normals, normals_d, distort, median, info = rasterization_2dgs(
        params["means"], params["quats"], torch.exp(params["scales"]), torch.sigmoid(params["opacities"]),
        torch.cat([params["sh0"], params["shN"]], 1), viewmat, Ks, width, height,
        sh_degree=sh_degree, render_mode="RGB+ED", distloss=distloss,
    )
    return out[0, ..., :3], alpha[0, ..., 0], normals[0], normals_d, distort[0, ..., 0], median[0, ..., 0], info


class SurfelRenderer:
    """Renders trained surfels in the model frame for TSDF fusion: the median depth, which
    for surfels is the surface rather than a blend along the ray."""

    def __init__(self, params, sh_degree: int, device):
        self.params, self.sh_degree, self.device = params, sh_degree, device

    def render(self, w2c: np.ndarray, K: np.ndarray, width: int, height: int, mode: str = "RGB+ED"):
        import torch

        with torch.no_grad():
            rgb, alpha, _, _, _, median, _ = render_2dgs(self.params, w2c, K, width, height, self.sh_degree,
                                                         self.device)
        return (np.clip(rgb.float().cpu().numpy(), 0.0, 1.0), median.float().cpu().numpy(),
                alpha.float().cpu().numpy())


def train_2dgs(args, max_splats: int, device: str = "cuda") -> SurfelRenderer:
    """Train surfels for `args.steps` steps from the COLMAP model at `args.colmap` (images at
    `args.images`, optional masks at `args.masks`) and return their renderer."""
    import torch

    import losses
    import train as T
    from strategy import BoundedMCMCStrategy

    ns = argparse.Namespace(colmap=args.colmap, images=args.images, masks=args.masks, depths=None, ut=False)
    views, pts, cols, centre, norm = T.load_normalised(ns)
    if ns.ut:
        raise RuntimeError("the 2DGS mesh pass needs undistorted pinhole cameras")
    data = T.Dataset(ns, views)
    params, scene_scale = T.init_params(pts, cols, data.views, max_splats, device)
    optimizers = T.make_optimizers(params, scene_scale)
    steps = max(1, args.steps)
    sched = torch.optim.lr_scheduler.ExponentialLR(optimizers["means"], gamma=0.01 ** (1.0 / steps))
    strategy = BoundedMCMCStrategy.for_run(max_splats, steps, scene_scale)
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state()
    appearance = losses.make_appearance("affine", len(data.items), steps, device)
    all_opts = list(optimizers.values()) + [appearance.opt]
    order: list[int] = []
    report_every = max(1, steps // 50)
    for step in range(steps):
        if not order:
            order = np.random.permutation(len(data.items)).tolist()
        i = order.pop()
        b = data.get(i, device)
        it = b["item"]
        w_normal, w_dist = surfel_weights(step)
        rgb, alpha, normals, normals_d, distort, _, info = render_2dgs(
            params, it["w2c"], it["K"], it["w"], it["h"], min(step // 1000, T.SH_DEGREE), device,
            distloss=w_dist > 0)
        rgb = appearance(rgb, i) + torch.rand(3, device=device) * (1.0 - alpha[..., None])
        loss = losses.photometric_loss(rgb, b["rgb"], b.get("mask"))
        normal_err, dist = surfel_terms(normals, normals_d, alpha, distort)
        loss = loss + w_normal * normal_err + w_dist * dist + appearance.regulariser()
        loss = loss + 0.01 * torch.sigmoid(params["opacities"]).mean() + 0.01 * torch.exp(params["scales"]).mean()
        T.apply_step(loss, all_opts)
        sched.step()
        strategy.step_post_backward(params, optimizers, state, step, info, lr=sched.get_last_lr()[0])
        if step % report_every == 0 or step == steps - 1:
            common.progress(0.02 + 0.4 * (step + 1) / steps, f"2DGS step {step + 1}/{steps}")
    with torch.no_grad():
        strategy.kill_lost(params, optimizers, state)
        # Back to the model frame, so the views' own poses render metric depth.
        c = torch.as_tensor(centre, dtype=params["means"].dtype, device=params["means"].device)
        params["means"].div_(norm).add_(c)
        params["scales"].sub_(math.log(norm))
    return SurfelRenderer(params, T.SH_DEGREE, device)
