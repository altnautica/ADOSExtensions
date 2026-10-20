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


def test_tsdf_retries_coarser_on_the_gpu_then_capped_on_the_cpu(monkeypatch):
    import types

    import pytest

    calls = []
    oom = RuntimeError("[Open3D Error] MemoryManagerCUDA.cpp:24 CUDA runtime error: out of memory")

    def fake_fuse(renderer, views, device, voxel, max_blocks):
        calls.append((device, voxel, max_blocks))
        if device is None:
            raise oom
        return "cpu-mesh", types.SimpleNamespace(device="CPU:0", capped=3)

    args = types.SimpleNamespace(voxel=0.02)
    monkeypatch.setattr(mesh.common, "SplatRenderer", lambda splat: object())
    monkeypatch.setattr(mesh, "release_gpu", lambda: None)
    monkeypatch.setattr(mesh, "fuse", fake_fuse)
    out, fusion = mesh.tsdf_mesh(args, "splat.ply", [])
    assert out == "cpu-mesh"
    assert calls == [(None, 0.02, None), (None, 0.04, None), ("CPU:0", 0.04, mesh.CPU_MAX_BLOCKS)]
    assert fusion == {"device": "CPU:0", "voxel_m": 0.04, "attempt": 3, "views_capped": 3}

    def other_error(renderer, views, device, voxel, max_blocks):
        raise RuntimeError("no view rendered usable depth")

    monkeypatch.setattr(mesh, "fuse", other_error)
    with pytest.raises(RuntimeError, match="usable depth"):
        mesh.tsdf_mesh(args, "splat.ply", [])


def test_the_cpu_grid_stops_growing_at_its_block_cap() -> None:
    fuser = mesh.TsdfFuser(0.02, "CPU:0", max_blocks=1)
    assert not fuser.integrate(*_view(0.0, 0.0))
    assert fuser.capped == 1 and fuser.used == 0
