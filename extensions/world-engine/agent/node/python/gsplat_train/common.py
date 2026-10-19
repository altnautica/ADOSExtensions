"""Shared helpers for the reconstruction scripts.

JSON-lines events on stdout, COLMAP binary model reading, image/mask/depth
path conventions, 3DGS PLY read/write and a gsplat render wrapper.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

NODE_DIR = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------- events


def emit(obj: dict) -> None:
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def progress(p: float, message: str) -> None:
    emit({"event": "progress", "progress": round(min(max(p, 0.0), 1.0), 4), "message": message})


def done(**fields) -> None:
    emit({"event": "done", **fields})


def run_main(fn: Callable[[], None]) -> None:
    """Run `fn`; any exception becomes an error event and exit code 1."""
    try:
        fn()
    except SystemExit:
        raise
    except BaseException as exc:  # noqa: BLE001 - reported to the caller
        traceback.print_exc(file=sys.stderr)
        emit({"event": "error", "message": f"{type(exc).__name__}: {exc}"})
        sys.exit(1)


def tools_cache_dir() -> Path:
    """Gitignored cache root (same as the fetch scripts' default)."""
    env = os.environ.get("ADOS_WE_TOOLS_DIR")
    root = Path(env) if env else NODE_DIR / "tools" / "cache"
    root.mkdir(parents=True, exist_ok=True)
    return root


# ---------------------------------------------------------------- COLMAP

# model_id -> (name, num_params)
CAMERA_MODELS = {
    0: ("SIMPLE_PINHOLE", 3),
    1: ("PINHOLE", 4),
    2: ("SIMPLE_RADIAL", 4),
    3: ("RADIAL", 5),
    4: ("OPENCV", 8),
    5: ("OPENCV_FISHEYE", 8),
    6: ("FULL_OPENCV", 12),
    7: ("FOV", 5),
    8: ("SIMPLE_RADIAL_FISHEYE", 4),
    9: ("RADIAL_FISHEYE", 5),
    10: ("THIN_PRISM_FISHEYE", 12),
    11: ("RAD_TAN_THIN_PRISM_FISHEYE", 16),
}


@dataclass
class ColmapCamera:
    id: int
    model: str
    width: int
    height: int
    params: np.ndarray

    def intrinsics(self) -> tuple[float, float, float, float]:
        p = self.params
        if self.model in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL", "SIMPLE_RADIAL_FISHEYE", "RADIAL_FISHEYE", "FOV"):
            return float(p[0]), float(p[0]), float(p[1]), float(p[2])
        return float(p[0]), float(p[1]), float(p[2]), float(p[3])

    def distortion(self) -> tuple[str, list[float], list[float]]:
        """(camera_model for gsplat, radial coeffs, tangential coeffs)."""
        p = [float(x) for x in self.params]
        m = self.model
        if m in ("SIMPLE_PINHOLE", "PINHOLE"):
            return "pinhole", [], []
        if m == "SIMPLE_RADIAL":
            return "pinhole", [p[3]], []
        if m == "RADIAL":
            return "pinhole", [p[3], p[4]], []
        if m == "OPENCV":
            return "pinhole", [p[4], p[5]], [p[6], p[7]]
        if m == "FULL_OPENCV":
            return "pinhole", [p[4], p[5], p[8], p[9], p[10], p[11]], [p[6], p[7]]
        if m == "OPENCV_FISHEYE":
            return "fisheye", p[4:8], []
        if m == "SIMPLE_RADIAL_FISHEYE":
            return "fisheye", [p[3]], []
        if m == "RADIAL_FISHEYE":
            return "fisheye", [p[3], p[4]], []
        raise ValueError(f"unsupported COLMAP camera model {m}")

    def is_distorted(self) -> bool:
        """True when this camera cannot be rendered as an ideal pinhole: a fisheye model, or any
        non-zero distortion coefficient (also one SfM refined on an `opencv` camera)."""
        model, radial, tangential = self.distortion()
        return model != "pinhole" or any(abs(x) > 1e-9 for x in radial + tangential)


@dataclass
class ColmapImage:
    id: int
    camera_id: int
    name: str
    w2c: np.ndarray  # 4x4 float64


def _qvec_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _read(f, fmt: str):
    size = struct.calcsize("<" + fmt)
    return struct.unpack("<" + fmt, f.read(size))


def read_cameras_bin(path: Path) -> dict[int, ColmapCamera]:
    cams = {}
    with open(path, "rb") as f:
        (n,) = _read(f, "Q")
        for _ in range(n):
            cid, model_id, w, h = _read(f, "iiQQ")
            name, npar = CAMERA_MODELS[model_id]
            params = np.array(_read(f, "d" * npar))
            cams[cid] = ColmapCamera(cid, name, int(w), int(h), params)
    return cams


def read_images_bin(path: Path) -> dict[int, ColmapImage]:
    images = {}
    with open(path, "rb") as f:
        (n,) = _read(f, "Q")
        for _ in range(n):
            iid, qw, qx, qy, qz, tx, ty, tz, cid = _read(f, "idddddddi")
            name = bytearray()
            while (c := f.read(1)) != b"\x00":
                name += c
            (npts,) = _read(f, "Q")
            f.seek(npts * 24, os.SEEK_CUR)  # x, y (double) + point3D id (int64)
            w2c = np.eye(4)
            w2c[:3, :3] = _qvec_to_rot(np.array([qw, qx, qy, qz]))
            w2c[:3, 3] = [tx, ty, tz]
            images[iid] = ColmapImage(iid, cid, name.decode("utf-8"), w2c)
    return images


def read_points3d_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """(xyz [N,3] float32, rgb [N,3] float32 in 0..1)."""
    xyz, rgb = [], []
    with open(path, "rb") as f:
        (n,) = _read(f, "Q")
        for _ in range(n):
            vals = _read(f, "QdddBBBd")
            xyz.append(vals[1:4])
            rgb.append(vals[4:7])
            (tl,) = _read(f, "Q")
            f.seek(tl * 8, os.SEEK_CUR)
    return np.asarray(xyz, np.float32).reshape(-1, 3), np.asarray(rgb, np.float32).reshape(-1, 3) / 255.0


def sparse_dir(colmap_dir: Path) -> Path:
    for cand in (colmap_dir / "sparse" / "0", colmap_dir / "0", colmap_dir):
        if (cand / "cameras.bin").is_file():
            return cand
    raise FileNotFoundError(f"no sparse/0/cameras.bin under {colmap_dir}")


@dataclass
class View:
    name: str  # image path relative to the images dir (forward slashes)
    w2c: np.ndarray  # 4x4
    K: np.ndarray  # 3x3
    width: int
    height: int
    camera: ColmapCamera | None = None


def load_colmap_views(colmap_dir: Path) -> tuple[list[View], np.ndarray, np.ndarray]:
    sp = sparse_dir(colmap_dir)
    cams = read_cameras_bin(sp / "cameras.bin")
    imgs = read_images_bin(sp / "images.bin")
    pts, cols = read_points3d_bin(sp / "points3D.bin") if (sp / "points3D.bin").is_file() else (np.zeros((0, 3), np.float32), np.zeros((0, 3), np.float32))
    views = []
    for im in sorted(imgs.values(), key=lambda i: i.name):
        cam = cams[im.camera_id]
        fx, fy, cx, cy = cam.intrinsics()
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
        views.append(View(im.name.replace("\\", "/"), im.w2c, K, cam.width, cam.height, cam))
    return views, pts, cols


def needs_ut(views: list[View], requested: bool) -> bool:
    """3DGUT when asked for, or when any camera of the model is distorted."""
    return requested or any(v.camera is not None and v.camera.is_distorted() for v in views)


def load_frames_json(path: Path) -> list[dict]:
    """frames.json entries with `w2c` (4x4) and `K` (3x3) added."""
    frames = json.loads(Path(path).read_text())
    for fr in frames:
        T = np.asarray(fr["T_world_cam"], np.float64).reshape(4, 4)
        fr["c2w"] = T
        fr["w2c"] = np.linalg.inv(T)
        fr["K"] = np.array([[fr["fx"], 0, fr["cx"]], [0, fr["fy"], fr["cy"]], [0, 0, 1]], np.float64)
    return frames


# ---------------------------------------------------------------- files


def read_image(path: Path, gray: bool = False) -> np.ndarray:
    """uint8 HxWx3 RGB (or HxW when gray); unicode-safe on Windows."""
    import cv2

    data = np.fromfile(str(path), np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise IOError(f"cannot decode {path}")
    if img.dtype != np.uint8:
        img = (img.astype(np.float32) / np.iinfo(img.dtype).max * 255).astype(np.uint8)
    if img.ndim == 3 and img.shape[2] == 4:
        img = img[:, :, :3]
    if gray:
        return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.ndim == 2:
        return np.repeat(img[:, :, None], 3, axis=2)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def write_png(path: Path, img: np.ndarray) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise IOError(f"cannot encode {path}")
    buf.tofile(str(path))


def write_jpeg(path: Path, rgb: np.ndarray, quality: int = 92) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise IOError(f"cannot encode {path}")
    buf.tofile(str(path))


def mask_path(masks_dir: Path, rel: str) -> Path:
    return masks_dir / (rel + ".png")


def depth_path(depths_dir: Path, rel: str) -> Path:
    return depths_dir / (rel + ".npy")


def list_images(images_dir: Path) -> list[str]:
    exts = {".png", ".jpg", ".jpeg", ".tif", ".tiff", ".bmp"}
    out = []
    for p in sorted(images_dir.rglob("*")):
        if p.is_file() and p.suffix.lower() in exts:
            out.append(p.relative_to(images_dir).as_posix())
    return out


# ---------------------------------------------------------------- splats

SH_C0 = 0.28209479177387814


def write_splat_ply(path: Path, means, scales, quats, opacities, sh0, shN) -> None:
    """Standard 3DGS PLY. scales are log, opacities logit, quats wxyz, sh0 [N,1,3], shN [N,K-1,3]."""
    from plyfile import PlyData, PlyElement

    n = means.shape[0]
    f_dc = sh0.reshape(n, 3)
    f_rest = shN.transpose(0, 2, 1).reshape(n, -1)  # channel-major like the reference format
    cols = (
        [("x", means[:, 0]), ("y", means[:, 1]), ("z", means[:, 2])]
        + [(f"n{a}", np.zeros(n, np.float32)) for a in "xyz"]
        + [(f"f_dc_{i}", f_dc[:, i]) for i in range(3)]
        + [(f"f_rest_{i}", f_rest[:, i]) for i in range(f_rest.shape[1])]
        + [("opacity", opacities.reshape(n))]
        + [(f"scale_{i}", scales[:, i]) for i in range(3)]
        + [(f"rot_{i}", quats[:, i]) for i in range(4)]
    )
    arr = np.empty(n, dtype=[(k, "f4") for k, _ in cols])
    for k, v in cols:
        arr[k] = np.asarray(v, np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    PlyData([PlyElement.describe(arr, "vertex")], text=False).write(str(path))


def read_splat_ply(path: Path) -> dict[str, np.ndarray]:
    from plyfile import PlyData

    v = PlyData.read(str(path))["vertex"].data
    names = v.dtype.names
    n = len(v)
    rest = sorted((k for k in names if k.startswith("f_rest_")), key=lambda k: int(k[7:]))
    shN = np.stack([v[k] for k in rest], 1).reshape(n, 3, -1).transpose(0, 2, 1) if rest else np.zeros((n, 0, 3), np.float32)
    return {
        "means": np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float32),
        "scales": np.stack([v[f"scale_{i}"] for i in range(3)], 1).astype(np.float32),
        "quats": np.stack([v[f"rot_{i}"] for i in range(4)], 1).astype(np.float32),
        "opacities": np.asarray(v["opacity"], np.float32),
        "sh0": np.stack([v[f"f_dc_{i}"] for i in range(3)], 1).reshape(n, 1, 3).astype(np.float32),
        "shN": np.ascontiguousarray(shN, np.float32),
    }


def sh_degree_for(shN_count: int) -> int:
    return int(round(math.sqrt(shN_count + 1))) - 1


def gsplat_unavailable_reason() -> str | None:
    """None when torch has a GPU and gsplat's CUDA kernels load, else why not.

    Loading the kernels (and a first-run JIT build) prints to stdout, which
    carries the JSON event stream, so that output goes to stderr instead.
    """
    import contextlib

    import torch

    if not torch.cuda.is_available():
        return "CUDA is not available to torch"
    with contextlib.redirect_stdout(sys.stderr):
        from gsplat.cuda._backend import _C

    if _C is None:
        return "gsplat CUDA kernels unavailable: no prebuilt kernels and no CUDA toolkit for the JIT build"
    return None


def require_gsplat() -> None:
    """Fail fast with a clear message when gsplat's CUDA kernels are unusable."""
    reason = gsplat_unavailable_reason()
    if reason:
        raise RuntimeError(reason)


class SplatRenderer:
    """Renders a trained splat (activated parameters on the GPU)."""

    def __init__(self, splat: dict[str, np.ndarray], device: str = "cuda"):
        import torch

        t = lambda a: torch.from_numpy(np.ascontiguousarray(a)).to(device)  # noqa: E731
        self.device = device
        self.means = t(splat["means"])
        self.quats = torch.nn.functional.normalize(t(splat["quats"]), dim=-1)
        self.scales = torch.exp(t(splat["scales"]))
        self.opacities = torch.sigmoid(t(splat["opacities"]))
        self.colors = torch.cat([t(splat["sh0"]), t(splat["shN"])], 1)
        self.sh_degree = sh_degree_for(splat["shN"].shape[1])

    def render(self, w2c: np.ndarray, K: np.ndarray, width: int, height: int, mode: str = "RGB+ED"):
        """(rgb HxWx3 float 0..1, depth HxW or None, alpha HxW) as numpy arrays."""
        import torch
        from gsplat import rasterization

        with torch.no_grad():
            out, alpha, _ = rasterization(
                self.means, self.quats, self.scales, self.opacities, self.colors,
                torch.tensor(w2c, dtype=torch.float32, device=self.device)[None],
                torch.tensor(K, dtype=torch.float32, device=self.device)[None],
                width, height, sh_degree=self.sh_degree, render_mode=mode,
            )
        out = out[0].float().cpu().numpy()
        rgb = np.clip(out[..., :3], 0.0, 1.0)
        depth = out[..., 3] if out.shape[-1] > 3 else None
        return rgb, depth, alpha[0, ..., 0].float().cpu().numpy()


def probe() -> None:
    """Exit 0 only when torch sees a GPU and the gsplat kernels run."""
    reason = gsplat_unavailable_reason()
    if reason:
        print(f"gsplat probe FAIL: {reason}", file=sys.stderr)
        sys.exit(1)
    import torch
    from gsplat import rasterization

    d = "cuda"
    n = 64
    means = torch.randn(n, 3, device=d) + torch.tensor([0.0, 0.0, 4.0], device=d)
    quats = torch.nn.functional.normalize(torch.randn(n, 4, device=d), dim=-1)
    scales = torch.full((n, 3), 0.05, device=d)
    opac = torch.full((n,), 0.8, device=d)
    cols = torch.rand(n, 3, device=d)
    K = torch.tensor([[50.0, 0, 32], [0, 50.0, 32], [0, 0, 1]], device=d)[None]
    img, _, _ = rasterization(means, quats, scales, opac, cols, torch.eye(4, device=d)[None], K, 64, 64)
    assert torch.isfinite(img).all()
    print("gsplat probe OK")


if __name__ == "__main__" and "--probe" in sys.argv:
    probe()
    sys.exit(0)
