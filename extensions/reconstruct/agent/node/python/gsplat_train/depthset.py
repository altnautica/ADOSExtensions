"""A depth set: posed colour + metric depth views that fuse.py integrates.

Layout: <root>/views.json, <root>/rgb/<i>.png (8-bit RGB), <root>/depth/<i>.npy (float32 metres,
0 = none). K is 3x3, w2c 4x4 (OpenCV convention).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


@dataclass
class DepthView:
    name: str
    K: np.ndarray
    w2c: np.ndarray
    width: int
    height: int
    rgb: str
    depth: str


def save_view(root: Path, i: int, name: str, rgb_u8: np.ndarray, depth_m: np.ndarray, K, w2c) -> DepthView:
    import cv2

    root = Path(root)
    (root / "rgb").mkdir(parents=True, exist_ok=True)
    (root / "depth").mkdir(parents=True, exist_ok=True)
    rgb_rel, depth_rel = f"rgb/{i:05d}.png", f"depth/{i:05d}.npy"
    if not cv2.imwrite(str(root / rgb_rel), cv2.cvtColor(np.ascontiguousarray(rgb_u8), cv2.COLOR_RGB2BGR)):
        raise IOError(f"cannot write {root / rgb_rel}")
    np.save(root / depth_rel, np.asarray(depth_m, np.float32))
    return DepthView(name, np.asarray(K, float), np.asarray(w2c, float), int(rgb_u8.shape[1]),
                     int(rgb_u8.shape[0]), rgb_rel, depth_rel)


def write_depthset(root: Path, views: list[DepthView], frame: str = "model") -> Path:
    rows = [{**asdict(v), "K": np.asarray(v.K).tolist(), "w2c": np.asarray(v.w2c).tolist()} for v in views]
    p = Path(root) / "views.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"frame": frame, "views": rows}))
    return p


def read_depthset(root: Path) -> tuple[str, list[DepthView]]:
    doc = json.loads((Path(root) / "views.json").read_text())
    return doc["frame"], [DepthView(r["name"], np.asarray(r["K"], float), np.asarray(r["w2c"], float),
                                    int(r["width"]), int(r["height"]), r["rgb"], r["depth"]) for r in doc["views"]]


def load_rgb(root: Path, v: DepthView) -> np.ndarray:
    import cv2

    bgr = cv2.imread(str(Path(root) / v.rgb), cv2.IMREAD_COLOR)
    if bgr is None:
        raise IOError(f"cannot read {Path(root) / v.rgb}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def load_depth(root: Path, v: DepthView) -> np.ndarray:
    return np.load(Path(root) / v.depth).astype(np.float32)
