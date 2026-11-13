"""TSDF fusion of depth sets in its own process: a synthetic flat wall, the GPU retry ladder,
the block cap and cluster cleanup."""

from __future__ import annotations

import json
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import depthset  # noqa: E402
import fuse  # noqa: E402

W, H, F, WALL_Z = 160, 120, 120.0, 3.0


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


def _wall_depthset(root: Path) -> None:
    views = []
    for i, tx in enumerate((-0.3, 0.0, 0.3, 0.6)):
        w2c = np.eye(4)
        w2c[0, 3] = -tx
        rgb = np.full((H, W, 3), 180, np.uint8)
        depth = np.full((H, W), WALL_Z, np.float32)
        K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
        views.append(depthset.save_view(root, i, f"cam/{i}.png", rgb, depth, K, w2c))
    depthset.write_depthset(root, views)


def _done(stdout: str) -> dict:
    return [e for e in (json.loads(l) for l in stdout.splitlines() if l.startswith("{")) if e.get("event") == "done"][-1]


def _run_fuse(tmp_path: Path, device: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(HERE / "fuse.py"), "--depthset", str(tmp_path / "ds"), "--out",
                           str(tmp_path / "o"), "--voxel", "0.05", "--device", device],
                          capture_output=True, text=True, cwd=HERE)


def test_flat_wall_fuses_to_triangles_once_seen_by_more_than_three_views() -> None:
    poses = [(-0.3, 5.0), (-0.1, 2.0), (0.1, -2.0), (0.3, -5.0)]
    fuser = fuse.TsdfFuser(0.02, "CPU:0")
    for tx, yaw in poses[:3]:
        assert fuser.integrate(*_view(tx, yaw))
    assert len(fuser.mesh().triangles) == 0  # weight 3 is not above TSDF_MIN_WEIGHT
    assert fuser.integrate(*_view(*poses[3]))
    tri = fuser.mesh()
    n = len(tri.triangles)
    assert n > 1000, f"only {n} triangles"
    z = np.asarray(tri.vertices)[:, 2]
    assert abs(float(np.median(z)) - WALL_Z) < 0.05
    cloud = fuser.point_cloud()
    assert len(cloud.points) > 1000 and cloud.has_colors()
    assert np.allclose(np.asarray(cloud.colors), 0.5, atol=0.02)


def test_the_grid_stops_growing_at_its_block_cap() -> None:
    fuser = fuse.TsdfFuser(0.02, "CPU:0", max_blocks=1)
    assert not fuser.integrate(*_view(0.0, 0.0))
    assert fuser.capped == 1 and fuser.used == 0


def test_max_blocks_for_a_budget() -> None:
    assert fuse.max_blocks_for(5000) == 5000 * (1 << 20) // (16 ** 3 * 20)


def test_cuda_out_of_memory_retries_coarser_on_the_gpu_never_the_cpu(tmp_path, monkeypatch) -> None:
    _wall_depthset(tmp_path / "ds")
    calls = []

    class FakeFuser:
        def __init__(self, voxel, device, max_blocks):
            calls.append((voxel, device, max_blocks))
            self.device, self.used, self.capped = device, 0, 2
            self.oom = len(calls) < 3

        def integrate(self, rgb, depth, K, w2c):
            if self.oom:
                raise RuntimeError("[Open3D Error] MemoryManagerCUDA.cpp:24 CUDA runtime error: out of memory")
            self.used += 1
            return True

        def mesh(self):
            return "mesh"

        def point_cloud(self):
            return "cloud"

    monkeypatch.setattr(fuse, "TsdfFuser", FakeFuser)
    m, c, info = fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0", budget_mb=100)
    assert (m, c) == ("mesh", "cloud")
    cap = fuse.max_blocks_for(100)
    assert [c[0] for c in calls] == pytest.approx([0.02, 0.03, 0.04])
    assert [c[1:] for c in calls] == [("CUDA:0", cap)] * 3
    assert info == {"device": "CUDA:0", "voxel_m": pytest.approx(0.04), "attempt": 3, "views_capped": 2}

    calls.clear()
    with pytest.raises(RuntimeError, match="out of memory"):
        monkeypatch.setattr(FakeFuser, "integrate", lambda *a: (_ for _ in ()).throw(RuntimeError("out of memory")))
        fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0")
    assert len(calls) == 3

    monkeypatch.setattr(FakeFuser, "integrate", lambda *a: (_ for _ in ()).throw(RuntimeError("bad pose")))
    calls.clear()
    with pytest.raises(RuntimeError, match="bad pose"):
        fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0")
    assert len(calls) == 1


def _grid_patch(o3d, n: int, x0: float):
    """An n x n vertex grid of triangles in the z = 0 plane starting at x = x0: 2 (n-1)^2 triangles."""
    xs, ys = np.meshgrid(np.arange(n) * 0.01 + x0, np.arange(n) * 0.01)
    v = np.stack([xs.ravel(), ys.ravel(), np.zeros(n * n)], 1)
    tris = []
    for r in range(n - 1):
        for c in range(n - 1):
            a = r * n + c
            tris += [[a, a + 1, a + n], [a + 1, a + n + 1, a + n]]
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(np.array(tris)))


def test_small_clusters_removed() -> None:
    import open3d as o3d

    big = _grid_patch(o3d, 120, 0.0)  # 28322 triangles
    medium = _grid_patch(o3d, 8, 5.0)  # 98: above 50 and above 0.1 % of the largest (28)
    small = _grid_patch(o3d, 5, 9.0)  # 32: below the 50-triangle floor
    m = big + medium + small
    assert fuse.remove_small_clusters(m) == 32
    assert len(m.triangles) == 28322 + 98
    assert np.asarray(m.vertices)[:, 0].max() < 9.0
    huge = _grid_patch(o3d, 400, 0.0)  # 318402: 0.1 % is 318, so the 98-triangle patch goes too
    m = huge + _grid_patch(o3d, 8, 5.0)
    assert fuse.remove_small_clusters(m) == 98


def test_fuse_on_cpu_makes_a_wall(tmp_path):
    _wall_depthset(tmp_path / "ds")
    r = _run_fuse(tmp_path, "CPU:0")
    assert r.returncode == 0, r.stderr
    d = _done(r.stdout)
    assert d["triangles"] > 100 and d["points"] > 100
    assert d["device"] == "CPU:0" and d["method"] == "tsdf" and d["peak_vram_mb"] is None
    for f in ("mesh.ply", "mesh.glb", "dense.ply", "dense.laz"):
        assert (tmp_path / "o" / f).is_file()


def _cuda() -> bool:
    try:
        import torch
    except ImportError:
        return False
    return torch.cuda.is_available()


@pytest.mark.skipif(not _cuda(), reason="needs torch and a CUDA GPU")
def test_fuse_runs_on_cuda_after_the_parent_used_torch_cuda(tmp_path):
    import torch

    x = torch.ones(1 << 20, device="cuda")  # the splat renderer's CUDA context lives in the parent, as in mesh.py
    _wall_depthset(tmp_path / "ds")
    r = _run_fuse(tmp_path, "CUDA:0")
    del x
    assert r.returncode == 0, r.stderr
    d = _done(r.stdout)
    assert d["device"] == "CUDA:0" and d["triangles"] > 100
