"""Mirror/glass masking on a synthetic planar wall.

World: a wall on the plane z = WALL_Z, seen by three pinhole cameras looking
along +z (OpenCV axes). The LiDAR cloud samples the wall. The depth priors
agree with it except inside a rectangle on the wall, where they report the
surface 1 m deeper, as a mirror or a window does.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
W, H = 320, 240
F = 250.0
WALL_Z = 5.0
MIRROR_DEPTH = 1.0
# mirror rectangle on the wall, world metres
MX0, MX1, MY0, MY1 = -1.0, 1.0, -0.8, 0.6


def _rot_y(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def _poses() -> list[np.ndarray]:
    out = []
    for tx, yaw in [(-0.4, 4.0), (0.0, 0.0), (0.5, -3.0)]:
        T = np.eye(4)
        T[:3, :3] = _rot_y(yaw)
        T[:3, 3] = [tx, 0.1, 0.0]
        out.append(T)
    return out


def _render(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(z-depth prior HxW, mirror pixel mask HxW) for camera-to-world T."""
    R, c = T[:3, :3], T[:3, 3]
    us, vs = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    d_cam = np.stack([(us - W / 2) / F, (vs - H / 2) / F, np.ones_like(us)], -1)
    d_w = d_cam @ R.T
    t_wall = (WALL_Z - c[2]) / d_w[..., 2]  # ray length per unit camera z
    hit = c + t_wall[..., None] * d_w
    mirror = (hit[..., 0] >= MX0) & (hit[..., 0] <= MX1) & (hit[..., 1] >= MY0) & (hit[..., 1] <= MY1)
    t_far = (WALL_Z + MIRROR_DEPTH - c[2]) / d_w[..., 2]
    depth = np.where(mirror, t_far, t_wall)  # d_cam has unit z, so t is z-depth
    return depth.astype(np.float32), mirror


def _write_laz(path: Path) -> None:
    import laspy

    xs, ys = np.meshgrid(np.arange(-6.0, 6.0, 0.03), np.arange(-4.0, 4.0, 0.03))
    xyz = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, WALL_Z)], 1)
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.zeros(3)
    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.write(str(path))


def _build(root: Path) -> list[tuple[str, np.ndarray]]:
    import cv2

    images, depths = root / "images", root / "depths"
    rng = np.random.default_rng(0)
    frames, truth = [], []
    for i, T in enumerate(_poses()):
        name = f"cam0/{1000 + i}.png"
        (images / name).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(images / name), rng.integers(0, 255, (H, W, 3), np.uint8))
        depth, mirror = _render(T)
        (depths / name).parent.mkdir(parents=True, exist_ok=True)
        np.save(depths / (name + ".npy"), depth)
        frames.append({
            "name": name, "camera_id": "cam0", "t_ns": 1000 + i,
            "T_world_cam": T.ravel().tolist(),
            "fx": F, "fy": F, "cx": W / 2, "cy": H / 2, "width": W, "height": H,
        })
        truth.append((name, mirror))
    (root / "frames.json").write_text(json.dumps(frames))
    _write_laz(root / "lidar.laz")
    return truth


def test_mirror_rectangle_masked(tmp_path: Path) -> None:
    import cv2

    truth = _build(tmp_path)
    out = tmp_path / "masks"
    cmd = [
        sys.executable, str(HERE / "masks.py"),
        "--images", str(tmp_path / "images"), "--frames", str(tmp_path / "frames.json"),
        "--out", str(out), "--depths", str(tmp_path / "depths"),
        "--lidar", str(tmp_path / "lidar.laz"), "--no-motion",
    ]
    run = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE)
    assert run.returncode == 0, run.stdout + run.stderr
    done = json.loads(run.stdout.strip().splitlines()[-1])
    assert done["event"] == "done" and done["frames_with_mirror"] == len(truth)
    for name, mirror in truth:
        mask = cv2.imread(str(out / (name + ".png")), cv2.IMREAD_GRAYSCALE)
        assert mask is not None and mask.shape == (H, W)
        assert mirror.sum() > 0.05 * W * H and (~mirror).sum() > 0.3 * W * H
        zeroed = float((mask[mirror] == 0).mean())
        kept = float((mask[~mirror] == 255).mean())
        assert zeroed >= 0.90, f"{name}: only {zeroed:.3f} of the mirror masked"
        assert kept >= 0.95, f"{name}: only {kept:.3f} of the wall kept"
