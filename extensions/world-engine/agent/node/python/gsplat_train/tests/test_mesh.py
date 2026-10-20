"""TSDF fusion of a synthetic flat wall, cluster cleanup and the 3D-LiDAR Poisson mesh."""

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


def test_flat_wall_fuses_to_triangles_once_seen_by_more_than_three_views() -> None:
    poses = [(-0.3, 5.0), (-0.1, 2.0), (0.1, -2.0), (0.3, -5.0)]
    fuser = mesh.TsdfFuser(0.02)
    for tx, yaw in poses[:3]:
        assert fuser.integrate(*_view(tx, yaw))
    assert len(fuser.mesh().triangles) == 0  # weight 3 is not above TSDF_MIN_WEIGHT
    assert fuser.integrate(*_view(*poses[3]))
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
    monkeypatch.setattr(mesh, "release_gpu", lambda: None)
    monkeypatch.setattr(mesh, "fuse", fake_fuse)
    out, fusion = mesh.tsdf_mesh(args, object(), [])
    assert out == "cpu-mesh"
    assert calls == [(None, 0.02, None), (None, 0.04, None), ("CPU:0", 0.04, mesh.CPU_MAX_BLOCKS)]
    assert fusion == {"device": "CPU:0", "voxel_m": 0.04, "attempt": 3, "views_capped": 3}

    def other_error(renderer, views, device, voxel, max_blocks):
        raise RuntimeError("no view rendered usable depth")

    monkeypatch.setattr(mesh, "fuse", other_error)
    with pytest.raises(RuntimeError, match="usable depth"):
        mesh.tsdf_mesh(args, object(), [])


def test_the_cpu_grid_stops_growing_at_its_block_cap() -> None:
    fuser = mesh.TsdfFuser(0.02, "CPU:0", max_blocks=1)
    assert not fuser.integrate(*_view(0.0, 0.0))
    assert fuser.capped == 1 and fuser.used == 0


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
    assert mesh.remove_small_clusters(m) == 32
    assert len(m.triangles) == 28322 + 98
    assert np.asarray(m.vertices)[:, 0].max() < 9.0
    huge = _grid_patch(o3d, 400, 0.0)  # 318402: 0.1 % is 318, so the 98-triangle patch goes too
    m = huge + _grid_patch(o3d, 8, 5.0)
    assert mesh.remove_small_clusters(m) == 98


def test_lidar_cloud_meshes_by_poisson_with_splat_colours(tmp_path) -> None:
    import types

    import laspy

    rng = np.random.default_rng(0)
    d = rng.normal(size=(40000, 3))
    xyz = 1.5 * d / np.linalg.norm(d, axis=1, keepdims=True) + np.array([2.0, 0.0, 1.0])
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.zeros(3)
    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.write(str(tmp_path / "lidar.laz"))
    # One opaque red splat on the +x side, one opaque blue on the -x side, one faint green.
    splat = {
        "means": np.array([[4.0, 0, 1], [0.0, 0, 1], [2.0, 3, 1]], np.float32),
        "opacities": np.array([5.0, 5.0, -5.0], np.float32),
        "sh0": ((np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], np.float32) - 0.5) / mesh.common.SH_C0)[:, None, :],
    }
    args = types.SimpleNamespace(lidar=tmp_path / "lidar.laz", voxel=0.05)
    m = mesh.lidar_mesh(args, splat, [])
    v = np.asarray(m.vertices)
    assert len(m.triangles) > 1000
    r = np.linalg.norm(v - np.array([2.0, 0.0, 1.0]), axis=1)
    assert abs(float(np.median(r)) - 1.5) < 0.03
    c = np.asarray(m.vertex_colors)
    assert np.allclose(c[v[:, 0] > 3.0], [1, 0, 0], atol=1e-3)
    assert np.allclose(c[v[:, 0] < 1.0], [0, 0, 1], atol=1e-3)
    assert not np.isclose(c[:, 1], 1.0).any()  # the faint splat colours nothing
