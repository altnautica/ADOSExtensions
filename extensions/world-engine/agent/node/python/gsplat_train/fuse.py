"""TSDF fusion of one or more depth sets (depthset.py) into a surface mesh and a dense coloured cloud.

Runs as its own process so it never shares a CUDA context with torch: Open3D's cached CUDA allocator
aborted ("Block ... should have been recorded") when it shared a process with the splat renderer.
The grid is an Open3D tensor VoxelBlockGrid on CUDA:0 when CUDA is available. On CUDA every attempt runs
in a child process of its own: a CUDA out-of-memory leaves the process's CUDA context unusable (the next
grid fails with "invalid device ordinal", or the process aborts), so the coarser retry starts a fresh one.
The block cap of each GPU attempt is sized from the VRAM free just before it. An out-of-memory grid retries
coarser on the GPU, never on the CPU; a node without CUDA fuses on the CPU, in this process.
Writes <out>/mesh.ply (binary), mesh.glb (surface), dense.ply and dense.laz (coloured surface points).

    fuse.py --depthset DIR [--depthset DIR ...] --out DIR [--voxel 0.03] [--max-tris 300000] [--device CUDA:0]
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import tempfile
from pathlib import Path

import numpy as np

import common
import depthset
import vram

BLOCK_RESOLUTION = 16
BLOCK_COUNT = 10_000  # initial hash capacity; the grid grows past it
TRUNC_VOXELS = 4.0  # TSDF truncation, in voxels
TSDF_MIN_WEIGHT = 3.0  # a surface voxel must be seen by more than this many views
CLUSTER_MIN_TRIANGLES = 50  # smaller disconnected clusters are dropped ...
CLUSTER_MIN_FRACTION = 0.001  # ... and so are those under this fraction of the largest
BYTES_PER_VOXEL = 20  # tsdf f32 + weight f32 + colour 3 x f32
COARSER = (1.0, 1.5, 2.0)  # voxel multipliers of the GPU attempts
# Share of the free VRAM a GPU attempt's blocks may take: the hash map doubles when it grows (old and new
# tables live together while it rehashes), and each view's image tensors and Open3D's cache sit beside it.
VRAM_GRID_FRACTION = 0.4


def max_blocks_for(budget_mb: int) -> int:
    """Blocks of BLOCK_RESOLUTION^3 voxels that fit in `budget_mb` MiB."""
    return int(budget_mb * (1 << 20) // (BLOCK_RESOLUTION ** 3 * BYTES_PER_VOXEL))


class TsdfFuser:
    """TSDF fusion of depth + colour views into a sparse voxel block grid."""

    def __init__(self, voxel: float, device: str | None = None, max_blocks: int | None = None):
        import open3d as o3d
        import open3d.core as o3c

        self.o3d, self.o3c = o3d, o3c
        self.device = o3c.Device(device or ("CUDA:0" if o3c.cuda.is_available() else "CPU:0"))
        self.grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1), (1), (3)),
            voxel_size=voxel,
            block_resolution=BLOCK_RESOLUTION,
            block_count=BLOCK_COUNT,
            device=self.device,
        )
        self.voxel = voxel
        self.max_blocks = max_blocks
        self.used = 0
        self.capped = 0  # views skipped because they would grow the grid past max_blocks

    def integrate(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, w2c: np.ndarray) -> bool:
        """rgb HxWx3 float 0..1, depth HxW metres (0 = none). False when the view has too little
        depth or would grow the grid past max_blocks."""
        o3d, o3c = self.o3d, self.o3c
        valid = depth[depth > 0]
        if valid.size < 100:
            return False
        # Open3D keeps depth strictly below depth_max; the voxel margin keeps a
        # view whose depths sit at the cut (a fronto-parallel plane) from losing them all.
        depth_max = float(np.percentile(valid, 98)) + self.voxel
        depth_t = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(depth, np.float32))).to(self.device)
        color_t = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(rgb, np.float32))).to(self.device)
        K_t = o3c.Tensor(np.asarray(K, np.float64))
        w2c_t = o3c.Tensor(np.asarray(w2c, np.float64))
        blocks = self.grid.compute_unique_block_coordinates(depth_t, K_t, w2c_t, 1.0, depth_max, TRUNC_VOXELS)
        if self.max_blocks is not None and self.grid.hashmap().size() + len(blocks) > self.max_blocks:
            self.capped += 1
            return False
        self.grid.integrate(blocks, depth_t, color_t, K_t, w2c_t, 1.0, depth_max, TRUNC_VOXELS)
        self.used += 1
        return True

    def mesh(self):
        return self.grid.extract_triangle_mesh(weight_threshold=TSDF_MIN_WEIGHT).cpu().to_legacy()

    def point_cloud(self):
        """The coloured surface points: the vertices of the extracted surface. The grid's own
        extract_point_cloud finds no strict sign change, so no points, on a surface lying exactly
        on a voxel layer, which marching cubes still meshes."""
        m = self.mesh()
        pcd = self.o3d.geometry.PointCloud(m.vertices)
        pcd.colors = m.vertex_colors
        return pcd


def is_out_of_memory(e: BaseException) -> bool:
    return "out of memory" in str(e).lower()


class GpuAttemptDied(RuntimeError):
    """A GPU attempt's child process ended without a result (killed by a signal, as an abort under
    CUDA memory pressure is); retried coarser like an out-of-memory."""


def _fuse_once(sets, vx: float, device: str | None, max_blocks: int | None, n: int):
    """`(mesh, cloud, fusion)` of one attempt at voxel `vx`, in this process."""
    fuser = TsdfFuser(vx, device, max_blocks)
    i = 0
    for root, views in sets:
        for v in views:
            rgb = depthset.load_rgb(root, v).astype(np.float32) / 255.0
            fuser.integrate(rgb, depthset.load_depth(root, v), v.K, v.w2c)
            i += 1
            common.progress(0.05 + 0.75 * i / n, f"fused view {i}/{n}")
    if fuser.used == 0:
        raise RuntimeError("no view had usable depth")
    common.progress(0.82, "extracting surface")
    return fuser.mesh(), fuser.point_cloud(), {"device": str(fuser.device), "voxel_m": vx,
                                               "views_capped": fuser.capped}


def _attempt_child(conn, sets, vx: float, device: str | None, max_blocks: int | None, n: int, mesh_path: str):
    """Child-process body of a GPU attempt: the mesh goes to `mesh_path`, the fusion fields or the
    error message down `conn`."""
    import open3d as o3d

    try:
        mesh, _, info = _fuse_once(sets, vx, device, max_blocks, n)
        if not o3d.io.write_triangle_mesh(mesh_path, mesh, write_ascii=False, compressed=False):
            raise IOError(f"cannot write {mesh_path}")
        conn.send(("ok", info))
    except BaseException as e:  # noqa: BLE001 - reported to the parent
        conn.send(("error", f"{e}"))
    finally:
        conn.close()


def _fuse_in_child(sets, vx: float, device: str | None, max_blocks: int | None, n: int):
    """`_fuse_once` in a fresh (spawned) process, so it gets a CUDA context of its own."""
    import open3d as o3d

    ctx = mp.get_context("spawn")
    with tempfile.TemporaryDirectory(prefix="fuse-") as tmp:
        mesh_path = str(Path(tmp) / "mesh.ply")
        rx, tx = ctx.Pipe(duplex=False)
        p = ctx.Process(target=_attempt_child, args=(tx, sets, vx, device, max_blocks, n, mesh_path))
        p.start()
        tx.close()
        try:
            msg = rx.recv()
        except EOFError:
            msg = None
        p.join()
        if msg is None:
            raise GpuAttemptDied(f"GPU attempt at voxel {vx:.3f} m died (exit code {p.exitcode})")
        if msg[0] == "error":
            raise RuntimeError(msg[1])
        mesh = o3d.io.read_triangle_mesh(mesh_path)
    cloud = o3d.geometry.PointCloud(mesh.vertices)
    cloud.colors = mesh.vertex_colors
    return mesh, cloud, msg[1]


def _cuda_index(device: str | None) -> int:
    return int(device.split(":")[1]) if device and ":" in device else 0


def gpu_budget_mb(device: str | None) -> int:
    """MiB the next GPU attempt's blocks may take: VRAM_GRID_FRACTION of the VRAM free now."""
    return int(vram.free_vram_mb(_cuda_index(device)) * VRAM_GRID_FRACTION)


def fuse_depthset(roots: list[Path], voxel: float, device: str | None = None, budget_mb: int | None = None):
    """`(mesh, cloud, fusion)`: the surface and coloured points fused from every view of every depth
    set in `roots`, and which attempt produced them. On CUDA each attempt runs in its own child process
    with its block cap from `budget_mb`, or from the VRAM free before it when None."""
    import open3d.core as o3c

    on_cuda = (device or "").upper().startswith("CUDA") or (device is None and o3c.cuda.is_available())
    sets = [(Path(r), depthset.read_depthset(Path(r))[1]) for r in roots]
    n = sum(len(views) for _, views in sets)
    if not on_cuda:
        mesh, cloud, info = _fuse_once(sets, voxel, device, None, n)
        return mesh, cloud, {**info, "attempt": 1}
    attempts = [m * voxel for m in COARSER]
    for k, vx in enumerate(attempts):
        budget = budget_mb if budget_mb is not None else gpu_budget_mb(device)
        try:
            mesh, cloud, info = _fuse_in_child(sets, vx, device, max_blocks_for(budget), n)
            return mesh, cloud, {**info, "attempt": k + 1, "budget_mb": budget}
        except RuntimeError as e:
            if not (is_out_of_memory(e) or isinstance(e, GpuAttemptDied)) or k == len(attempts) - 1:
                raise
            common.progress(0.05, f"TSDF grid out of GPU memory ({e}); retrying in a fresh process at voxel "
                                  f"{attempts[k + 1]:.3f} m")
    raise AssertionError("unreachable")


def remove_small_clusters(mesh) -> int:
    """Drop the triangles of connected clusters smaller than CLUSTER_MIN_TRIANGLES or than
    CLUSTER_MIN_FRACTION of the largest cluster (floaters and fusion debris). The number dropped."""
    if len(mesh.triangles) == 0:
        return 0
    ids, counts, _ = mesh.cluster_connected_triangles()
    ids, counts = np.asarray(ids), np.asarray(counts)
    floor = max(CLUSTER_MIN_TRIANGLES, CLUSTER_MIN_FRACTION * counts.max())
    drop = counts[ids] < floor
    mesh.remove_triangles_by_mask(drop)
    mesh.remove_unreferenced_vertices()
    return int(drop.sum())


def finish(out: Path, mesh, max_tris: int):
    """Clean, cluster-filter, decimate to `max_tris` and write <out>/mesh.ply (binary) and
    <out>/mesh.glb. `(mesh, triangles dropped as small clusters, ply, glb)`."""
    import open3d as o3d

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    dropped = remove_small_clusters(mesh)
    if len(mesh.triangles) == 0:
        raise RuntimeError("surface extraction produced no triangles")
    if len(mesh.triangles) > max_tris:
        common.progress(0.88, f"decimating {len(mesh.triangles)} -> {max_tris} triangles")
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=max_tris)
        mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    out.mkdir(parents=True, exist_ok=True)
    ply, glb = out / "mesh.ply", out / "mesh.glb"
    if not o3d.io.write_triangle_mesh(str(ply), mesh, write_ascii=False, compressed=False):
        raise IOError(f"cannot write {ply}")
    if not o3d.io.write_triangle_mesh(str(glb), mesh):
        raise IOError(f"cannot write {glb}")
    return mesh, dropped, ply, glb


def write_laz(path: Path, pcd) -> None:
    import laspy

    xyz = np.asarray(pcd.points)
    las = laspy.LasData(laspy.LasHeader(point_format=7, version="1.4"))
    las.header.offsets = xyz.min(0)
    las.header.scales = np.array([0.001, 0.001, 0.001])
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if pcd.has_colors():
        rgb = (np.clip(np.asarray(pcd.colors), 0, 1) * 65535).astype(np.uint16)
        las.red, las.green, las.blue = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    las.write(str(path))


def main(a) -> None:
    import open3d as o3d

    common.progress(0.0, f"fusing {len(a.depthset)} depth set(s)")
    mesh, pcd, fusion = fuse_depthset(a.depthset, a.voxel, a.device)
    if len(pcd.points) == 0:
        raise RuntimeError("surface extraction produced no points")
    mesh, dropped, ply, glb = finish(a.out, mesh, a.max_tris)
    dense_ply, dense_laz = a.out / "dense.ply", a.out / "dense.laz"
    if not o3d.io.write_point_cloud(str(dense_ply), pcd):
        raise IOError(f"cannot write {dense_ply}")
    write_laz(dense_laz, pcd)
    common.done(triangles=int(len(mesh.triangles)), points=int(len(pcd.points)), method=a.method_label,
                cluster_triangles_dropped=dropped, ply=str(ply), glb=str(glb), dense=str(dense_laz),
                peak_vram_mb=None, **fusion)


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--depthset", action="append", required=True, type=Path, help="depth set dir (repeatable)")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--voxel", type=float, default=0.03, help="TSDF voxel, metres")
    p.add_argument("--max-tris", type=int, default=300_000)
    p.add_argument("--device", help="Open3D device (CUDA:0, CPU:0); default CUDA:0 when available")
    p.add_argument("--method-label", default="tsdf", help="`method` of the done event")
    return p.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    common.run_main(lambda: main(args))
