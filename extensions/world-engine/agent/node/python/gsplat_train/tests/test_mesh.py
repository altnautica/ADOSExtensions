"""TSDF fusion of a synthetic flat wall seen from three views."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mesh  # noqa: E402

W, H = 160, 120
F = 120.0
WALL_Z = 3.0


def _rot_y(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def _view(tx: float, yaw: float):
    """(rgb, z-depth, K, w2c) for a camera at (tx, 0, 0) looking along +z at the wall z = WALL_Z."""
    c2w = np.eye(4)
    c2w[:3, :3] = _rot_y(yaw)
    c2w[:3, 3] = [tx, 0.0, 0.0]
    us, vs = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    d_cam = np.stack([(us - W / 2) / F, (vs - H / 2) / F, np.ones_like(us)], -1)
    d_w = d_cam @ c2w[:3, :3].T
    depth = ((WALL_Z - c2w[2, 3]) / d_w[..., 2]).astype(np.float32)  # d_cam has unit z
    rgb = np.full((H, W, 3), 0.5, np.float32)
    K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]], np.float64)
    return rgb, depth, K, np.linalg.inv(c2w)


def test_flat_wall_fuses_to_triangles() -> None:
    fuser = mesh.TsdfFuser(0.02)
    for tx, yaw in [(-0.3, 5.0), (0.0, 0.0), (0.3, -5.0)]:
        assert fuser.integrate(*_view(tx, yaw))
    tri = fuser.mesh()
    n = len(tri.triangles)
    assert n > 1000, f"only {n} triangles"
    z = np.asarray(tri.vertices)[:, 2]
    assert abs(float(np.median(z)) - WALL_Z) < 0.05


def test_tsdf_falls_back_to_the_cpu_when_the_gpu_grid_runs_out_of_memory(monkeypatch):
    calls = []

    def fake_fuse(args, renderer, views, device):
        calls.append(device)
        if device is None:
            raise RuntimeError("[Open3D Error] MemoryManagerCUDA.cpp:24 CUDA runtime error: out of memory")
        return "cpu-mesh"

    monkeypatch.setattr(mesh.common, "SplatRenderer", lambda splat: object())
    monkeypatch.setattr(mesh, "fuse", fake_fuse)
    assert mesh.tsdf_mesh(None, "splat.ply", []) == "cpu-mesh"
    assert calls == [None, "CPU:0"]

    def other_error(args, renderer, views, device):
        raise RuntimeError("no view rendered usable depth")

    monkeypatch.setattr(mesh, "fuse", other_error)
    import pytest

    with pytest.raises(RuntimeError, match="usable depth"):
        mesh.tsdf_mesh(None, "splat.ply", [])
