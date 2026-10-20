"""Train a 3D Gaussian splat from a COLMAP model with gsplat (MCMC densification).

Output: <out>/splat.ply (standard 3DGS PLY). Events: JSON lines on stdout.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

import numpy as np

import common

MAX_SIDE = 1600  # training images are downscaled so the long side fits this
RAM_CACHE_BYTES = 6 << 30
SH_DEGREE = 3
# A view's monocular depth prior is used only where it agrees with the model's own sparse points.
DEPTH_MIN_POINTS = 50
DEPTH_MAX_MEDIAN_REL = 0.05
DEPTH_MAX_P90_REL = 0.25


def depth_prior_scale(prior: np.ndarray, obs_xy: np.ndarray | None, z: np.ndarray | None) -> float | None:
    """The scale that puts a view's monocular depth `prior` (full resolution) onto the model's
    depths `z` of the sparse points observed at pixels `obs_xy`, or None when the prior disagrees
    with them (median relative error > 5 % or 90th percentile > 25 % after scaling, or too few
    points): a wrong prior pulls gaussians along the view rays, so such a view trains without a
    depth term."""
    if obs_xy is None or z is None or len(z) < DEPTH_MIN_POINTS:
        return None
    h, w = prior.shape
    u = np.clip(np.floor(obs_xy[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.floor(obs_xy[:, 1]).astype(int), 0, h - 1)
    p = prior[v, u].astype(np.float64)
    ok = np.isfinite(p) & (p > 0) & (z > 0)
    if ok.sum() < DEPTH_MIN_POINTS:
        return None
    s = float(np.median(z[ok] / p[ok]))
    rel = np.abs(s * p[ok] - z[ok]) / z[ok]
    if np.median(rel) > DEPTH_MAX_MEDIAN_REL or np.percentile(rel, 90) > DEPTH_MAX_P90_REL:
        return None
    return s


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--colmap", required=True, type=Path, help="dir containing sparse/0 (BIN model)")
    p.add_argument("--images", required=True, type=Path, help="images dir; COLMAP image names are relative to it")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--steps", type=int, default=30000)
    p.add_argument("--max-splats", type=int, default=1_000_000)
    p.add_argument("--masks", type=Path, help="<masks>/<image rel>.png, 0 = ignore, 255 = keep")
    p.add_argument("--depths", type=Path, help="<depths>/<image rel>.npy, float32 metres")
    p.add_argument("--ut", action="store_true", help="3DGUT: unscented transform for distorted/fisheye cameras")
    p.add_argument("--bilateral-grid", action="store_true", help="per-image bilateral colour grids")
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args(argv)


class Dataset:
    """Training views with downscaled images, optional masks and depth priors (CPU, uint8/float16)."""

    def __init__(self, args, views: list[common.View]):
        import cv2

        self.views, self.items = [], []
        self.depth_used = self.depth_rejected = 0
        budget = RAM_CACHE_BYTES
        for v in views:
            path = args.images / v.name
            if not path.is_file():
                print(f"missing image {path}, skipped", file=sys.stderr)
                continue
            s = min(1.0, MAX_SIDE / max(v.width, v.height))
            w, h = max(1, round(v.width * s)), max(1, round(v.height * s))
            K = v.K.copy()
            K[0, :] *= w / v.width
            K[1, :] *= h / v.height
            item = {"path": path, "w": w, "h": h, "K": K, "w2c": v.w2c, "camera": v.camera, "name": v.name}
            if args.masks and common.mask_path(args.masks, v.name).is_file():
                item["mask_path"] = common.mask_path(args.masks, v.name)
            if args.depths and common.depth_path(args.depths, v.name).is_file():
                prior = np.load(common.depth_path(args.depths, v.name)).astype(np.float32)
                z = None if v.obs_xyz is None else v.obs_xyz @ v.w2c[:3, :3].T[:, 2] + v.w2c[2, 3]
                scale = depth_prior_scale(prior, v.obs_xy, z)
                if scale is None:
                    self.depth_rejected += 1
                else:
                    item["depth_path"] = common.depth_path(args.depths, v.name)
                    item["depth_scale"] = scale
                    self.depth_used += 1
            need = w * h * 3
            if need <= budget:
                item["rgb"] = self._load_rgb(item, cv2)
                budget -= need
            self.views.append(v)
            self.items.append(item)
        if not self.items:
            raise RuntimeError("no training images found")

    @staticmethod
    def _load_rgb(item, cv2) -> np.ndarray:
        img = common.read_image(item["path"])
        if img.shape[1] != item["w"] or img.shape[0] != item["h"]:
            img = cv2.resize(img, (item["w"], item["h"]), interpolation=cv2.INTER_AREA)
        return img

    def get(self, i: int, device):
        import cv2
        import torch

        it = self.items[i]
        rgb = it.get("rgb")
        if rgb is None:
            rgb = self._load_rgb(it, cv2)
        out = {"rgb": torch.from_numpy(rgb).to(device).float() / 255.0, "item": it}
        if "mask_path" in it:
            m = common.read_image(it["mask_path"], gray=True)
            m = cv2.resize(m, (it["w"], it["h"]), interpolation=cv2.INTER_NEAREST)
            out["mask"] = torch.from_numpy((m > 127).astype(np.float32)).to(device)
        if "depth_path" in it:
            d = np.load(it["depth_path"]).astype(np.float32) * it["depth_scale"]
            d = cv2.resize(d, (it["w"], it["h"]), interpolation=cv2.INTER_NEAREST)
            out["depth"] = torch.from_numpy(np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)).to(device)
        return out


def camera_terms(item, use_ut: bool, device):
    """gsplat camera kwargs for one view; distortion only feeds the 3DGUT path."""
    import torch

    cam = item["camera"]
    model, radial, tangential = cam.distortion() if cam is not None else ("pinhole", [], [])
    kw = {"camera_model": model}
    if not use_ut:
        if cam is not None and cam.is_distorted():
            raise RuntimeError(f"{item['name']}: distorted/{model} camera needs --ut or undistorted input")
        return kw
    n = 4 if model == "fisheye" else 6
    r = (radial + [0.0] * n)[:n]
    kw["radial_coeffs"] = torch.tensor([r], dtype=torch.float32, device=device)
    if model == "pinhole":
        t = (tangential + [0.0, 0.0])[:2]
        kw["tangential_coeffs"] = torch.tensor([t], dtype=torch.float32, device=device)
    return kw


def init_params(pts: np.ndarray, cols: np.ndarray, views, max_splats: int, device):
    import torch
    from scipy.spatial import cKDTree

    centers = np.stack([np.linalg.inv(v.w2c)[:3, 3] for v in views])
    if len(pts) < 100:
        lo, hi = centers.min(0) - 1.0, centers.max(0) + 1.0
        n = min(100_000, max_splats)
        pts = np.random.uniform(lo, hi, (n, 3)).astype(np.float32)
        cols = np.random.uniform(0, 1, (n, 3)).astype(np.float32)
    if len(pts) > max_splats:
        sel = np.random.choice(len(pts), max_splats, replace=False)
        pts, cols = pts[sel], cols[sel]
    d, _ = cKDTree(pts).query(pts, k=4)
    dist = np.sqrt((d[:, 1:] ** 2).mean(1)).clip(1e-4, None)
    n = len(pts)
    t = lambda a: torch.tensor(a, dtype=torch.float32, device=device)  # noqa: E731
    params = {
        "means": t(pts),
        "scales": t(np.log(dist))[:, None].repeat(1, 3),
        "quats": torch.rand((n, 4), device=device),
        "opacities": torch.logit(torch.full((n,), 0.5, device=device)),
        "sh0": t((cols - 0.5) / common.SH_C0)[:, None, :],
        "shN": torch.zeros((n, (SH_DEGREE + 1) ** 2 - 1, 3), device=device),
    }
    scene_scale = float(np.linalg.norm(centers - centers.mean(0), axis=1).max()) * 1.1 or 1.0
    return torch.nn.ParameterDict({k: torch.nn.Parameter(v) for k, v in params.items()}), scene_scale


def make_optimizers(params, scene_scale: float):
    import torch

    lrs = {
        "means": 1.6e-4 * scene_scale,
        "scales": 5e-3,
        "quats": 1e-3,
        "opacities": 5e-2,
        "sh0": 2.5e-3,
        "shN": 2.5e-3 / 20,
    }
    return {k: torch.optim.Adam([{"params": params[k], "lr": lr, "name": k}], eps=1e-15) for k, lr in lrs.items()}


def render(params, item, sh_degree, use_ut, mode, device):
    import torch
    from gsplat import rasterization

    viewmat = torch.tensor(item["w2c"], dtype=torch.float32, device=device)[None]
    K = torch.tensor(item["K"], dtype=torch.float32, device=device)[None]
    colors = torch.cat([params["sh0"], params["shN"]], 1)
    out, alpha, info = rasterization(
        params["means"],
        params["quats"],
        torch.exp(params["scales"]),
        torch.sigmoid(params["opacities"]),
        colors,
        viewmat,
        K,
        item["w"],
        item["h"],
        sh_degree=sh_degree,
        packed=False,
        render_mode=mode,
        with_ut=use_ut,
        with_eval3d=use_ut,
        **camera_terms(item, use_ut, device),
    )
    return out[0], alpha[0, ..., 0], info


def evaluate(params, data: Dataset, use_ut: bool, device) -> float | None:
    import torch

    idx = np.linspace(0, len(data.items) - 1, min(32, len(data.items))).round().astype(int)
    vals = []
    with torch.no_grad():
        for i in sorted(set(idx.tolist())):
            b = data.get(i, device)
            out, _, _ = render(params, b["item"], SH_DEGREE, use_ut, "RGB", device)
            err = (out.clamp(0, 1) - b["rgb"]).pow(2).mean(-1)
            if "mask" in b:
                err = (err * b["mask"]).sum() / b["mask"].sum().clamp_min(1)
            else:
                err = err.mean()
            vals.append(float(-10 * torch.log10(err.clamp_min(1e-10))))
    return round(float(np.mean(vals)), 3) if vals else None


def train(args) -> None:
    import torch
    from gsplat.strategy import MCMCStrategy

    import losses

    common.progress(0.0, "loading model")
    common.require_gsplat()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = "cuda"

    views, pts, cols = common.load_colmap_views(args.colmap)
    # The model SfM produced decides, not the rig alone: an uncalibrated `opencv` camera comes
    # back with refined distortion, which only the 3DGUT path renders.
    ut = common.needs_ut(views, args.ut)
    if ut and not args.ut:
        common.progress(0.01, "distorted cameras in the SfM model: training with 3DGUT")
    args.ut = ut
    data = Dataset(args, views)
    common.progress(0.02, f"{len(data.items)} views, {len(pts)} sparse points, depth prior on "
                          f"{data.depth_used} views ({data.depth_rejected} disagree with the sparse points)")
    params, scene_scale = init_params(pts, cols, data.views, args.max_splats, device)
    optimizers = make_optimizers(params, scene_scale)
    steps = max(1, args.steps)
    gamma = 0.01 ** (1.0 / steps)
    sched = torch.optim.lr_scheduler.ExponentialLR(optimizers["means"], gamma=gamma)
    strategy = MCMCStrategy(
        cap_max=args.max_splats,
        refine_start_iter=min(500, steps // 10),
        refine_stop_iter=int(steps * 0.83),
        refine_every=100,
    )
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state()

    bilagrid = bil_opt = None
    if args.bilateral_grid:
        bilagrid = losses.BilateralGrid(len(data.items)).to(device)
        bil_opt = torch.optim.Adam(bilagrid.parameters(), lr=2e-3, eps=1e-15)

    use_depth = any("depth_path" in it for it in data.items)
    mode = "RGB+ED" if use_depth else "RGB"
    order: list[int] = []
    report_every = max(1, steps // 100)
    for step in range(steps):
        if not order:
            order = np.random.permutation(len(data.items)).tolist()
        i = order.pop()
        b = data.get(i, device)
        sh_degree = min(step // 1000, SH_DEGREE)
        out, alpha, info = render(params, b["item"], sh_degree, args.ut, mode, device)
        rgb = out[..., :3]
        if bilagrid is not None:
            rgb = bilagrid(rgb, i)
        weight = b.get("mask")
        loss = losses.photometric_loss(rgb, b["rgb"], weight)
        if use_depth and "depth" in b:
            w_depth = 0.05 * max(0.0, 1.0 - step / steps)
            loss = loss + w_depth * losses.depth_loss(out[..., 3], b["depth"], alpha, weight)
        loss = loss + 0.01 * torch.sigmoid(params["opacities"]).mean() + 0.01 * torch.exp(params["scales"]).mean()
        if bilagrid is not None:
            loss = loss + 10.0 * bilagrid.total_variation()
        loss.backward()
        for opt in optimizers.values():
            opt.step()
            opt.zero_grad(set_to_none=True)
        if bil_opt is not None:
            bil_opt.step()
            bil_opt.zero_grad(set_to_none=True)
        sched.step()
        strategy.step_post_backward(params, optimizers, state, step, info, lr=sched.get_last_lr()[0])
        if step % report_every == 0 or step == steps - 1:
            common.progress(
                0.03 + 0.9 * (step + 1) / steps,
                f"step {step + 1}/{steps} loss {float(loss):.4f} splats {len(params['means'])}",
            )

    common.progress(0.94, "evaluating")
    psnr = evaluate(params, data, args.ut, device)
    common.progress(0.97, "writing splat.ply")
    ply = args.out / "splat.ply"
    with torch.no_grad():
        q = torch.nn.functional.normalize(params["quats"], dim=-1)
        common.write_splat_ply(
            ply,
            params["means"].cpu().numpy(),
            params["scales"].cpu().numpy(),
            q.cpu().numpy(),
            params["opacities"].cpu().numpy(),
            params["sh0"].cpu().numpy(),
            params["shN"].cpu().numpy(),
        )
    psnr_out = psnr if psnr is not None and math.isfinite(psnr) else None
    common.done(ply=str(ply), gaussians=int(len(params["means"])), psnr=psnr_out, ut=bool(args.ut),
                depth_views=data.depth_used, depth_rejected=data.depth_rejected)


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: train(a))
