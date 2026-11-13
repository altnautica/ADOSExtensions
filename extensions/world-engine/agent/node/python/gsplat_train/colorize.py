"""Colour a point cloud (LAZ) from posed camera frames on the GPU. Each point takes the distance-weighted mean
colour of the frames that see it; a point is seen when it lies in front of the camera, inside the image, within
`max_dist`, on a kept pixel of the frame's mask (people, mirrors and glass are masked out), and not behind the
nearest point projecting to the same coarse pixel (a z-buffer at 1/zbuf_down resolution, so a sparse cloud still
occludes). The projection is an ideal pinhole, so frames of a distorted camera are skipped (and counted).
Output: the input LAZ with its points and attributes, plus RGB (16-bit).

A node without a CUDA device reports `done` with `skipped` and writes nothing.

    colorize.py --cloud <in.laz> --dataset <dataset dir> [--colmap <dir>] --out <out.laz>
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import common

Z_ABS, Z_REL = 0.05, 0.02
# Camera models that project as an ideal pinhole when their distortion coefficients are all zero.
PINHOLE_MODELS = ("pinhole", "opencv")


@dataclass
class ColorView:
    K: np.ndarray
    w2c: np.ndarray
    width: int
    height: int
    image: np.ndarray | Path  # HxWx3 uint8, or the file decoded when the view is coloured from
    mask: np.ndarray | Path | None = None  # HxW, 0 = ignore; a file is decoded like the image

    def pixels(self) -> np.ndarray:
        return self.image if isinstance(self.image, np.ndarray) else common.read_image(self.image)

    def keep(self) -> np.ndarray | None:
        if self.mask is None or isinstance(self.mask, np.ndarray):
            return self.mask
        return common.read_image(self.mask, gray=True)


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
        keep = v.keep()
        if keep is not None:
            front &= torch.as_tensor(keep, device=device)[vi, ui] > 127
        sel = idx[front]
        img = torch.as_tensor(v.pixels(), device=device).float()
        col = img[vi[front], ui[front]]
        w = 1.0 / (1.0 + z[sel])
        acc.index_add_(0, sel, col * w[:, None])
        wsum.index_add_(0, sel, w)
        seen.index_add_(0, sel, torch.ones_like(sel, dtype=torch.int32))
    rgb = (acc / wsum.clamp(min=1e-9)[:, None]).round().clamp(0, 255).to(torch.uint8)
    rgb[wsum == 0] = 0
    return rgb.cpu().numpy(), seen.cpu().numpy()


def _mask(dataset: Path, name: str) -> Path | None:
    path = common.mask_path(dataset / "masks", name)
    return path if path.is_file() else None


def views_from_frames(dataset: Path) -> tuple[list[ColorView], int]:
    """The dataset's frames at their prior poses (`frames.json`, OpenCV `T_world_cam`), and how many frames of
    a distorted camera were skipped. Unposed frames are left out."""
    views, distorted = [], 0
    for f in common.load_frames_json(dataset / "frames.json"):
        if f["w2c"] is None:
            continue
        if f.get("model", "pinhole") not in PINHOLE_MODELS or any(abs(d) > 1e-9 for d in f.get("dist") or []):
            distorted += 1
            continue
        views.append(ColorView(f["K"], f["w2c"], int(f["width"]), int(f["height"]),
                               dataset / "images" / f["name"], _mask(dataset, f["name"])))
    return views, distorted


def views_from_colmap(colmap: Path, dataset: Path) -> tuple[list[ColorView], int]:
    """The SfM model's registered frames, and how many of a distorted camera were skipped."""
    model, _, _ = common.load_colmap_views(colmap)
    views = [ColorView(v.K, v.w2c, v.width, v.height, dataset / "images" / v.name, _mask(dataset, v.name))
             for v in model if v.camera is None or not v.camera.is_distorted()]
    return views, len(model) - len(views)


def main(a) -> None:
    import laspy
    import torch

    import vram

    if not torch.cuda.is_available():
        common.done(skipped="no CUDA device")
        return
    poses = "colmap" if a.colmap else "frames"
    views, distorted = views_from_colmap(a.colmap, a.dataset) if a.colmap else views_from_frames(a.dataset)
    if not views:
        common.done(skipped="no posed pinhole frame", poses=poses, frames_distorted=distorted)
        return
    vram.require_free_vram(2000)
    common.progress(0.0, "reading the cloud")
    las = laspy.read(str(a.cloud))
    xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], 1)
    common.progress(0.2, f"colouring {len(xyz)} points from {len(views)} frames")
    rgb, seen = colorize_points(xyz, views)
    out = las if "red" in las.point_format.dimension_names else laspy.convert(las, point_format_id=7,
                                                                               file_version="1.4")
    out.red, out.green, out.blue = (rgb[:, i].astype(np.uint16) * 257 for i in range(3))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    out.write(str(a.out))
    common.done(points=int(len(xyz)), coloured=int((seen > 0).sum()), poses=poses, frames=len(views),
                frames_distorted=distorted, peak_vram_mb=vram.peak_vram_mb())


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cloud", required=True, type=Path)
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--colmap", type=Path)
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args()
    common.run_main(lambda: main(args))
