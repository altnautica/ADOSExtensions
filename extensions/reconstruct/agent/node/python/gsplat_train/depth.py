"""Monocular metric depth per image with MoGe-2 (MIT code and weights).

Writes <out>/<image rel>.npy (float32 metres, 0 = no estimate). Weights are
downloaded once into the gitignored tools cache (models/huggingface).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

import common

MODEL_REPO = "Ruicheng/moge-2-vitl-normal"
MODEL_REVISION = "cb0e8bbd6b1e243589717c78e750b1ba4c093acf"


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--images", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--frames", type=Path, help="optional frames.json; known fx gives the model its field of view")
    p.add_argument("--overwrite", action="store_true", help="recompute depths that already exist")
    return p.parse_args(argv)


def load_model(device: str):
    import os

    # Windows without developer mode cannot create symlinks; the hub then copies.
    os.environ.setdefault("HF_HUB_DISABLE_SYMLINKS_WARNING", "1")
    from huggingface_hub import hf_hub_download
    from moge.model.v2 import MoGeModel

    cache = common.tools_cache_dir() / "models" / "huggingface"
    ckpt = hf_hub_download(MODEL_REPO, filename="model.pt", revision=MODEL_REVISION, cache_dir=str(cache))
    return MoGeModel.from_pretrained(ckpt).to(device).eval()


def main(args) -> None:
    import torch

    names = common.list_images(args.images)
    if not names:
        raise RuntimeError(f"no images under {args.images}")
    fov = {}
    if args.frames:
        for fr in common.load_frames_json(args.frames):
            fov[fr["name"]] = math.degrees(2 * math.atan(fr["width"] / (2 * fr["fx"])))
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        print("CUDA unavailable; MoGe runs on the CPU", file=sys.stderr)
    common.progress(0.0, f"loading {MODEL_REPO}")
    model = load_model(device)
    written = 0
    for i, rel in enumerate(names):
        dst = common.depth_path(args.out, rel)
        if dst.is_file() and not args.overwrite:
            continue
        rgb = common.read_image(args.images / rel)
        x = torch.from_numpy(rgb).to(device).float().permute(2, 0, 1) / 255.0
        with torch.no_grad():
            res = model.infer(x, fov_x=fov.get(rel), use_fp16=device == "cuda")
        depth = res["depth"].float().cpu().numpy()
        if "mask" in res:
            depth = np.where(res["mask"].cpu().numpy() > 0, depth, 0.0)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
        dst.parent.mkdir(parents=True, exist_ok=True)
        np.save(dst, depth)
        written += 1
        common.progress((i + 1) / len(names), f"depth {i + 1}/{len(names)}")
    common.done(images=len(names), written=written, out=str(args.out))


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
