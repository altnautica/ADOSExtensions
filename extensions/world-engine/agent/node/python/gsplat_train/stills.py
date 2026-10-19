"""Render JPEG stills from a trained splat at given camera poses.

--poses is a JSON list of {name, T_world_cam:[16] row-major (OpenCV camera),
width, height, fx, fy, cx, cy}; each pose is written to <out>/<name>.jpg.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import common


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--splat", required=True, type=Path, help="3DGS PLY")
    p.add_argument("--poses", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--quality", type=int, default=92, help="JPEG quality")
    return p.parse_args(argv)


def main(args) -> None:
    poses = json.loads(args.poses.read_text())
    if not isinstance(poses, list) or not poses:
        raise ValueError("--poses must be a non-empty JSON list")
    common.require_gsplat()
    common.progress(0.0, "loading splat")
    renderer = common.SplatRenderer(common.read_splat_ply(args.splat))
    files = []
    for i, p in enumerate(poses):
        c2w = np.asarray(p["T_world_cam"], np.float64).reshape(4, 4)
        K = np.array([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]], np.float64)
        rgb, _, _ = renderer.render(np.linalg.inv(c2w), K, int(p["width"]), int(p["height"]), "RGB")
        dst = args.out / f"{p['name']}.jpg"
        common.write_jpeg(dst, (rgb * 255 + 0.5).astype(np.uint8), args.quality)
        files.append(str(dst))
        common.progress((i + 1) / len(poses), f"still {i + 1}/{len(poses)}")
    common.done(stills=len(files), files=files)


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
