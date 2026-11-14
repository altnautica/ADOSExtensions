"""TSDF fusion of depth sets in its own process: a synthetic flat wall, the GPU retry ladder,
the block cap and cluster cleanup."""

from __future__ import annotations

import json
import os
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


def test_an_open3d_allocation_failure_counts_as_out_of_memory() -> None:
    marching_cubes = RuntimeError(
        "[Open3D Error] VoxelBlockGridImpl.h:1423: Unable to allocate assistance mesh structure for "
        "Marching Cubes with 3480 active voxel blocks. Please consider using a larger voxel size")
    assert fuse.is_out_of_memory(marching_cubes)
    assert fuse.is_out_of_memory(RuntimeError("CUDA runtime error: out of memory"))
    assert not fuse.is_out_of_memory(RuntimeError("no view had usable depth"))


def test_surface_extraction_never_runs_on_the_gpu_grid() -> None:
    class Mesh:
        def to_legacy(self):
            return "mesh"

    class Hashmap:
        def __init__(self, cuda):
            self.is_cuda = cuda

        def size(self):
            return 1

    class CpuGrid:
        def hashmap(self):
            return Hashmap(False)

        def extract_triangle_mesh(self, weight_threshold):
            return Mesh()

    class CudaGrid:
        def hashmap(self):
            return Hashmap(True)

        def cpu(self):
            return CpuGrid()

        def extract_triangle_mesh(self, weight_threshold):
            raise AssertionError("marching cubes ran on the CUDA grid")

    fuser = fuse.TsdfFuser.__new__(fuse.TsdfFuser)
    fuser.o3d = fuser.o3c = None
    fuser.voxel = 0.02
    fuser.grid = CudaGrid()
    assert fuser.mesh() == "mesh"


def test_no_grid_grows_past_the_integrate_index_bound() -> None:
    assert fuse.TsdfFuser(0.02, "CPU:0").max_blocks == fuse.INTEGRATE_MAX_BLOCKS
    assert fuse.TsdfFuser(0.02, "CPU:0", max_blocks=10 ** 9).max_blocks == fuse.INTEGRATE_MAX_BLOCKS
    assert fuse.INTEGRATE_MAX_BLOCKS * 3 * 16 ** 3 < 2 ** 31
    assert fuse.MC_MAX_BLOCKS * 16 ** 3 * 16 < 2 ** 31


def test_extraction_chunks_stay_under_the_marching_cubes_bound() -> None:
    g = np.stack(np.meshgrid(np.arange(40), np.arange(40), np.arange(3), indexing="ij"), -1).reshape(-1, 3)
    chunks = fuse.chunk_blocks(g, limit=600, tile=4)
    assert len(chunks) > 1 and all(len(c) <= 600 for c in chunks)
    covered = np.zeros(len(g), int)
    for c in chunks:
        covered[c] += 1
    assert covered.min() >= 1  # every block is in some chunk, as core or halo


def test_chunked_extraction_matches_one_pass() -> None:
    fuser = fuse.TsdfFuser(0.02, "CPU:0")
    for tx, yaw in [(-0.3, 5.0), (-0.1, 2.0), (0.1, -2.0), (0.3, -5.0)]:
        assert fuser.integrate(*_view(tx, yaw))
    whole = fuse.extract_chunked(fuser.o3d, fuser.o3c, fuser.grid, 0.02)
    n = fuser.grid.hashmap().size()
    parts = fuse.extract_chunked(fuser.o3d, fuser.o3c, fuser.grid, 0.02, limit=n // 4)
    assert len(whole.triangles) > 1000
    assert len(parts.triangles) == len(whole.triangles)
    a = np.unique(np.round(np.asarray(whole.vertices), 6), axis=0)
    b = np.unique(np.round(np.asarray(parts.vertices), 6), axis=0)
    assert a.shape == b.shape and np.allclose(a, b)


def test_far_depth_is_cut_at_the_depth_range() -> None:
    assert fuse.depth_max_for(np.array([1.0, 2.0, 3.0]), 0.02) < 3.1
    assert fuse.depth_max_for(np.array([50.0, 90.0, 120.0]), 0.02) == fuse.DEPTH_MAX_M


def test_the_voxel_coarsens_until_the_estimated_grid_fits(monkeypatch) -> None:
    monkeypatch.setattr(fuse, "estimate_blocks", lambda sets, vxs: {vx: int(1e6 * (0.02 / vx) ** 2) for vx in vxs})
    vx, est = fuse.voxel_for([], 0.02, 170_000)
    assert est <= fuse.ESTIMATE_HEADROOM * 170_000
    assert vx == 0.02 * min(m for m in fuse.VOXEL_LADDER if 1e6 / m ** 2 <= fuse.ESTIMATE_HEADROOM * 170_000)


def test_the_block_estimate_tracks_the_real_grid(tmp_path) -> None:
    _wall_depthset(tmp_path / "ds")
    root, views = tmp_path / "ds", depthset.read_depthset(tmp_path / "ds")[1]
    est = fuse.estimate_blocks([(root, views)], [0.02])[0.02]
    fuser = fuse.TsdfFuser(0.02, "CPU:0")
    for v in views:
        fuser.integrate(depthset.load_rgb(root, v).astype(np.float32) / 255.0, depthset.load_depth(root, v), v.K, v.w2c)
    real = fuser.grid.hashmap().size()
    assert 0.5 * real <= est <= 2.0 * real, (est, real)


_PLANE_GRID = """
import numpy as np, open3d as o3d, open3d.core as o3c

def plane_grid(n):
    # n fully observed blocks in one z layer, a plane at z voxel 7.5 in each (as the RCA repro)
    g = o3d.t.geometry.VoxelBlockGrid(attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3c.float32, o3c.float32, o3c.float32), attr_channels=((1), (1), (3)),
        voxel_size=0.02, block_resolution=16, block_count=n, device=o3c.Device("CPU:0"))
    i = np.arange(n, dtype=np.int32)
    g.hashmap().activate(o3c.Tensor(np.stack([i % 256, i // 256, np.zeros_like(i)], 1).astype(np.int32)))
    g.attribute("tsdf").numpy()[:] = np.clip((np.arange(16, dtype=np.float32) - 7.5) / 4.0, -1, 1)[None, :, None, None, None]
    g.attribute("weight").numpy()[:] = 10.0
    g.attribute("color").numpy()[:] = 128.0
    return g
"""


@pytest.mark.skipif(not os.environ.get("FUSE_HEAVY_TESTS"), reason="about 6 GB of RAM; set FUSE_HEAVY_TESTS=1")
def test_marching_cubes_past_its_index_bound_is_chunked_not_crashed() -> None:
    n = 32_769  # one block past Open3D 0.20's 32-bit marching-cubes table offset
    one_pass = subprocess.run([sys.executable, "-c", _PLANE_GRID + f"plane_grid({n}).extract_triangle_mesh(weight_threshold=3.0)"],
                              capture_output=True, text=True)
    assert one_pass.returncode != 0  # SIGSEGV: why extract_chunked exists
    ns: dict = {}
    exec(_PLANE_GRID, ns)
    import open3d as o3d
    import open3d.core as o3c

    mesh = fuse.extract_chunked(o3d, o3c, ns["plane_grid"](n), 0.02)
    assert len(mesh.vertices) == n * 16 * 16  # one plane crossing per voxel column, none lost or doubled at seams


def test_cuda_out_of_memory_retries_coarser_on_the_gpu_never_the_cpu(tmp_path, monkeypatch) -> None:
    _wall_depthset(tmp_path / "ds")
    calls = []

    class FakeFuser:
        def __init__(self, voxel, device, max_blocks, block_count=None):
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

        def point_cloud(self, m=None):
            return "cloud"

    monkeypatch.setattr(fuse, "TsdfFuser", FakeFuser)
    monkeypatch.setattr(fuse, "_fuse_in_child", fuse._fuse_once)  # the ladder's logic, without child processes
    m, c, info = fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0", budget_mb=100)
    assert (m, c) == ("mesh", "cloud")
    cap = fuse.max_blocks_for(100)
    assert [c[0] for c in calls] == pytest.approx([0.02, 0.03, 0.04])
    assert [c[1:] for c in calls] == [("CUDA:0", cap)] * 3
    assert info == {"device": "CUDA:0", "voxel_m": pytest.approx(0.04), "attempt": 3, "views_capped": 2,
                    "budget_mb": 100, "estimated_blocks": info["estimated_blocks"]}
    assert 0 < info["estimated_blocks"] < fuse.max_blocks_for(100)

    # Without a budget each attempt's cap comes from the VRAM free just before it.
    free = iter([1000, 1000, 500, 250])  # the first is the estimate's GPU check
    monkeypatch.setattr(fuse.vram, "free_vram_mb", lambda device=0: next(free))
    calls.clear()
    with pytest.raises(RuntimeError, match="out of memory"):
        monkeypatch.setattr(FakeFuser, "integrate", lambda *a: (_ for _ in ()).throw(RuntimeError("out of memory")))
        fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0")
    assert [c[2] for c in calls] == [fuse.max_blocks_for(int(f * fuse.VRAM_GRID_FRACTION)) for f in (1000, 500, 250)]

    # A child that dies without a result (an abort) is retried coarser too.
    died = []

    def dies(sets, vx, device, max_blocks, n):
        died.append(vx)
        if len(died) < 2:
            raise fuse.GpuAttemptDied("GPU attempt died (exit code -6)")
        return "mesh", "cloud", {"device": device, "voxel_m": vx, "views_capped": 0}

    monkeypatch.setattr(fuse, "_fuse_in_child", dies)
    assert fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0", budget_mb=100)[2]["attempt"] == 2
    monkeypatch.setattr(fuse, "_fuse_in_child", fuse._fuse_once)

    monkeypatch.setattr(FakeFuser, "integrate", lambda *a: (_ for _ in ()).throw(RuntimeError("bad pose")))
    calls.clear()
    with pytest.raises(RuntimeError, match="bad pose"):
        fuse.fuse_depthset([tmp_path / "ds"], 0.02, "CUDA:0", budget_mb=100)
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


@pytest.mark.skipif(not _cuda(), reason="needs torch and a CUDA GPU")
def test_a_gpu_retry_after_a_real_cuda_out_of_memory_completes_on_cuda(tmp_path, monkeypatch):
    # The parent holds all but ~1 GiB of the GPU (a fresh CUDA context plus Open3D needs a few hundred MiB),
    # and a 0.05 mm voxel with no block cap puts every pixel
    # of every view in its own block, so attempt 1 runs out of GPU memory for real. Attempt 2 (1000x
    # coarser) fits in what is left, but only in a fresh CUDA context: in the same process Open3D fails
    # with "invalid device ordinal" after the OOM.
    import torch

    free, _ = torch.cuda.mem_get_info()
    hog = torch.empty(max(free - (1 << 30), 0), dtype=torch.uint8, device="cuda")
    _wall_depthset(tmp_path / "ds")
    monkeypatch.setattr(fuse, "COARSER", (1.0, 1000.0))
    monkeypatch.setattr(fuse, "voxel_for", lambda sets, voxel, cap: (voxel, 0))  # the forced OOM needs the fine voxel
    try:
        mesh, cloud, info = fuse.fuse_depthset([tmp_path / "ds"], 0.00005, "CUDA:0", budget_mb=1 << 20)
    finally:
        del hog
    assert info["attempt"] == 2 and info["device"] == "CUDA:0"
    assert info["voxel_m"] == pytest.approx(0.05)
    assert len(mesh.triangles) > 100 and len(cloud.points) > 100
