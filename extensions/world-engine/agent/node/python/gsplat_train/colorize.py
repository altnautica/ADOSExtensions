"""Colour a point cloud (LAZ) from posed camera frames on the GPU. Each point takes the distance-weighted mean
colour of the frames that see it; a point is seen when it lies in front of the camera, inside the image, within
`max_dist`, and not behind the nearest point projecting to the same coarse pixel (a z-buffer at 1/zbuf_down
resolution, so a sparse cloud still occludes). Output: the input LAZ with RGB (16-bit) and point source = 0.

    colorize.py --cloud <in.laz> --dataset <dataset dir> [--colmap <dir>] --out <out.laz>
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import common

Z_ABS, Z_REL = 0.05, 0.02


@dataclass
class ColorView:
    K: np.ndarray
    w2c: np.ndarray
    width: int
    height: int
    image: np.ndarray  # HxWx3 uint8


def colorize_points(xyz: np.ndarray, views: list[ColorView], device: str = "cuda", max_dist: float = 30.0,
                    zbuf_down: int = 4) -> tuple[np.ndarray, np.ndarray]:
    import torch

    P = torch.as_tensor(np.asarray(xyz, np.float32), device=device)
    acc = torch.zeros((len(P), 3), device=device)
    wsum = torch.zeros(len(P), device=device)
    seen = torch.zeros(len(P), dtype=torch.int32, device=device)
    for v in views:
        W2C = torch.as_tensor(v.w2c, dtype=torch.float32, device=device)
        pc = P @ W2C[:3, :3].T + W2C[:3, 3]
        z = pc[:, 2]
        zs = z.clamp(min=1e-6)
        u = float(v.K[0, 0]) * pc[:, 0] / zs + float(v.K[0, 2])
        vv = float(v.K[1, 1]) * pc[:, 1] / zs + float(v.K[1, 2])
        ok = (z > 0.05) & (z < max_dist) & (u >= 0) & (u < v.width) & (vv >= 0) & (vv < v.height)
        idx = torch.nonzero(ok).squeeze(1)
        if idx.numel() == 0:
            continue
        ui, vi = u[idx].long(), vv[idx].long()
        cw = (v.width + zbuf_down - 1) // zbuf_down
        cell = (vi // zbuf_down) * cw + ui // zbuf_down
        zb = torch.full(((v.height // zbuf_down + 1) * cw,), float("inf"), device=device)
        zb.scatter_reduce_(0, cell, z[idx], reduce="amin")
        front = z[idx] <= zb[cell] * (1 + Z_REL) + Z_ABS
        sel = idx[front]
        img = torch.as_tensor(v.image, device=device).float()
        col = img[vi[front], ui[front]]
        w = 1.0 / (1.0 + z[sel])
        acc.index_add_(0, sel, col * w[:, None])
        wsum.index_add_(0, sel, w)
        seen.index_add_(0, sel, torch.ones_like(sel, dtype=torch.int32))
    rgb = (acc / wsum.clamp(min=1e-9)[:, None]).round().clamp(0, 255).to(torch.uint8)
    rgb[wsum == 0] = 0
    return rgb.cpu().numpy(), seen.cpu().numpy()


def views_from_frames(dataset: Path) -> list[ColorView]:
    """The dataset's frames at their prior poses (`frames.json`, OpenCV `T_world_cam`); unposed frames are
    skipped."""
    return [ColorView(f["K"], f["w2c"], int(f["width"]), int(f["height"]),
                      common.read_image(dataset / "images" / f["name"]))
            for f in common.load_frames_json(dataset / "frames.json") if f["w2c"] is not None]


def views_from_colmap(colmap: Path, images: Path) -> list[ColorView]:
    views, _, _ = common.load_colmap_views(colmap)
    return [ColorView(v.K, v.w2c, v.width, v.height, common.read_image(images / v.name)) for v in views]


def main(a) -> None:
    import laspy

    import vram

    vram.require_free_vram(2000)
    common.progress(0.0, "reading the cloud")
    las = laspy.read(str(a.cloud))
    xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], 1)
    views = views_from_colmap(a.colmap, a.dataset / "images") if a.colmap else views_from_frames(a.dataset)
    common.progress(0.2, f"colouring {len(xyz)} points from {len(views)} frames")
    rgb, seen = colorize_points(xyz, views)
    out = laspy.LasData(laspy.LasHeader(point_format=7, version="1.4"))
    out.header.offsets, out.header.scales = las.header.offsets, las.header.scales
    out.x, out.y, out.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    out.red, out.green, out.blue = (rgb[:, i].astype(np.uint16) * 257 for i in range(3))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    out.write(str(a.out))
    common.done(points=int(len(xyz)), coloured=int((seen > 0).sum()), poses="colmap" if a.colmap else "frames",
                peak_vram_mb=vram.peak_vram_mb())


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cloud", required=True, type=Path)
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--colmap", type=Path)
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args()
    common.run_main(lambda: main(args))
