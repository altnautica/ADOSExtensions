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
# Open3D 0.20 indexes grid memory with 32-bit offsets. Integrate addresses colour at 3 * block * 16^3 + voxel,
# which wraps past 174,762 blocks; marching cubes addresses its 16-byte per-voxel table at voxel * 16, which
# wraps past 32,768 blocks. Past either bound it writes out of bounds: a segfault on the CPU, an MMU fault
# (Xid 31) on the GPU. So a grid never grows past INTEGRATE_MAX_BLOCKS, and marching cubes runs on chunks of
# at most MC_MAX_BLOCKS blocks, halo included.
INTEGRATE_MAX_BLOCKS = 170_000
MC_MAX_BLOCKS = 30_000
MC_TILE_BLOCKS = 8  # chunks are built from cubes of this many blocks a side
# Depth past this is not fused: far returns add blocks by the square of range and little surface, and a
# 120 m warehouse view at 0.02 m alone passes the integrate bound.
DEPTH_MAX_M = 30.0
VOXEL_LADDER = (1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0)  # voxel multipliers tried until the estimate fits
ESTIMATE_HEADROOM = 0.85  # share of INTEGRATE_MAX_BLOCKS the estimated grid may take
ESTIMATE_STRIDE = 4  # pixel stride of the block estimate, as Open3D's DepthTouch
ESTIMATE_RAY_SAMPLES = 4


def depth_max_for(valid: np.ndarray, voxel: float) -> float:
    """The far depth cut of a view with depths `valid`: its 98th percentile plus a voxel (Open3D keeps
    depth strictly below depth_max, and the margin keeps a fronto-parallel plane at the cut), at most
    DEPTH_MAX_M."""
    return min(float(np.percentile(valid, 98)) + voxel, DEPTH_MAX_M)


def max_blocks_for(budget_mb: int) -> int:
    """Blocks of BLOCK_RESOLUTION^3 voxels that fit in `budget_mb` MiB."""
    return int(budget_mb * (1 << 20) // (BLOCK_RESOLUTION ** 3 * BYTES_PER_VOXEL))


class TsdfFuser:
    """TSDF fusion of depth + colour views into a sparse voxel block grid."""

    def __init__(self, voxel: float, device: str | None = None, max_blocks: int | None = None,
                 block_count: int = BLOCK_COUNT):
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
            block_count=block_count,
            device=self.device,
        )
        self.voxel = voxel
        self.max_blocks = INTEGRATE_MAX_BLOCKS if max_blocks is None else min(max_blocks, INTEGRATE_MAX_BLOCKS)
        self.used = 0
        self.capped = 0  # views skipped because they would grow the grid past max_blocks

    def integrate(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, w2c: np.ndarray) -> bool:
        """rgb HxWx3 float 0..1, depth HxW metres (0 = none). False when the view has too little
        depth or would grow the grid past max_blocks."""
        o3d, o3c = self.o3d, self.o3c
        valid = depth[depth > 0]
        if valid.size < 100:
            return False
        depth_max = depth_max_for(valid, self.voxel)
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
        """Marching cubes on a CPU copy of the grid, wherever it was fused (Open3D's CUDA marching cubes
        made an illegal memory access on large volumes), one chunk of at most MC_MAX_BLOCKS blocks at a
        time (see MC_MAX_BLOCKS)."""
        grid = self.grid.cpu() if self.grid.hashmap().is_cuda else self.grid
        return extract_chunked(self.o3d, self.o3c, grid, self.voxel)

    def point_cloud(self, m=None):
        """The coloured surface points: the vertices of the extracted surface `m` (extracted here when
        None). The grid's own extract_point_cloud finds no strict sign change, so no points, on a
        surface lying exactly on a voxel layer, which marching cubes still meshes."""
        m = self.mesh() if m is None else m
        pcd = self.o3d.geometry.PointCloud(m.vertices)
        pcd.colors = m.vertex_colors
        return pcd


def chunk_blocks(keys: np.ndarray, limit: int = MC_MAX_BLOCKS, tile: int = MC_TILE_BLOCKS) -> list[np.ndarray]:
    """Row indices into `keys` (N x 3 block coordinates) of each extraction chunk: whole tiles of `tile`^3
    blocks, gathered in tile order, each chunk with its 26-neighbour halo of active blocks at most
    `limit` blocks. A chunk's first rows are its own blocks, then its halo."""
    keys = np.asarray(keys, np.int64)
    if len(keys) == 0:
        return []
    lo = keys.min(0) - 1
    span = keys.max(0) + 2 - lo

    def code(k):
        k = k - lo
        return (k[:, 0] * span[1] + k[:, 1]) * span[2] + k[:, 2]

    codes = code(keys)
    order = np.argsort(codes)
    sorted_codes = codes[order]
    offsets = np.array([(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
                        if (x, y, z) != (0, 0, 0)], np.int64)

    def halo_of(rows: np.ndarray) -> np.ndarray:
        nb = code((keys[rows][:, None, :] + offsets[None]).reshape(-1, 3))
        pos = np.clip(np.searchsorted(sorted_codes, nb), 0, len(codes) - 1)
        hit = order[pos[sorted_codes[pos] == nb]]
        return np.setdiff1d(np.unique(hit), rows)

    t = keys // tile
    tile_ids, inverse = np.unique(t, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    by_tile = np.argsort(inverse, kind="stable")
    bounds = np.searchsorted(inverse[by_tile], np.arange(len(tile_ids) + 1))
    chunks, cur = [], []
    for i in range(len(tile_ids)):
        rows = by_tile[bounds[i]:bounds[i + 1]]
        trial = np.concatenate(cur + [rows])
        if cur and len(trial) + len(halo_of(trial)) > limit:
            core = np.concatenate(cur)
            chunks.append(np.concatenate([core, halo_of(core)]))
            cur = [rows]
        else:
            cur.append(rows)
    core = np.concatenate(cur)
    chunks.append(np.concatenate([core, halo_of(core)]))
    return chunks


def extract_chunked(o3d, o3c, grid, voxel: float, limit: int = MC_MAX_BLOCKS):
    """Marching cubes of a CPU VoxelBlockGrid as one legacy mesh, run on sub-grids of at most `limit`
    blocks (each chunk's blocks copied with their neighbours, so the seams mesh as in one pass; the
    triangles a halo repeats are dropped as duplicates)."""
    hm = grid.hashmap()
    if hm.size() <= limit:
        return grid.extract_triangle_mesh(weight_threshold=TSDF_MIN_WEIGHT).to_legacy()
    buf = hm.active_buf_indices().numpy().astype(np.int64)
    keys = hm.key_tensor().numpy()[buf]
    names = ("tsdf", "weight", "color")
    src = {a: grid.attribute(a).numpy() for a in names}
    out = o3d.geometry.TriangleMesh()
    for rows in chunk_blocks(keys, limit):
        sub = o3d.t.geometry.VoxelBlockGrid(
            attr_names=names, attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1), (1), (3)), voxel_size=voxel, block_resolution=BLOCK_RESOLUTION,
            block_count=len(rows), device=o3c.Device("CPU:0"))
        sub_buf, _ = sub.hashmap().activate(o3c.Tensor(np.ascontiguousarray(keys[rows], np.int32)))
        sub_buf = sub_buf.numpy().astype(np.int64)
        for a in names:
            sub.attribute(a).numpy()[sub_buf] = src[a][buf[rows]]
        out += sub.extract_triangle_mesh(weight_threshold=TSDF_MIN_WEIGHT).to_legacy()
    out.remove_duplicated_vertices()
    out.remove_duplicated_triangles()
    out.remove_unreferenced_vertices()
    return out


def is_out_of_memory(e: BaseException) -> bool:
    """A CUDA out-of-memory, or Open3D's own checked allocation failure ("Unable to allocate ...")."""
    msg = str(e).lower()
    return "out of memory" in msg or "unable to allocate" in msg


class GpuAttemptDied(RuntimeError):
    """A GPU attempt's child process ended without a result (killed by a signal, as an abort under
    CUDA memory pressure is); retried coarser like an out-of-memory."""


def _fuse_once(sets, vx: float, device: str | None, max_blocks: int | None, n: int,
                block_count: int = BLOCK_COUNT):
    """`(mesh, cloud, fusion)` of one attempt at voxel `vx`, in this process."""
    fuser = TsdfFuser(vx, device, max_blocks, block_count)
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
    mesh = fuser.mesh()
    return mesh, fuser.point_cloud(mesh), {"device": str(fuser.device), "voxel_m": vx,
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


def _view_block_codes(depth: np.ndarray, K: np.ndarray, w2c: np.ndarray, vx: float) -> np.ndarray:
    """Codes of the blocks one view's integration would touch, emulated as Open3D's DepthTouch does
    (every ESTIMATE_STRIDE-th pixel, ray samples across the truncation band), with fuse's depth range."""
    d = depth[::ESTIMATE_STRIDE, ::ESTIMATE_STRIDE]
    vs, us = np.nonzero(d > 0)
    z = d[vs, us].astype(np.float64)
    valid = depth[depth > 0]
    keep = z < depth_max_for(valid, vx)
    us, vs, z = us[keep] * ESTIMATE_STRIDE, vs[keep] * ESTIMATE_STRIDE, z[keep]
    rays = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(z)], 1)
    c2w = np.linalg.inv(w2c)
    trunc = TRUNC_VOXELS * vx
    block = vx * BLOCK_RESOLUTION
    out = []
    for s in np.linspace(-1.0, 1.0, ESTIMATE_RAY_SAMPLES):
        zz = np.clip(z + s * trunc, 1e-3, None)
        p = (rays * zz[:, None]) @ c2w[:3, :3].T + c2w[:3, 3]
        k = np.floor(p / block).astype(np.int64) + (1 << 20)
        out.append((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2])
    return np.unique(np.concatenate(out))


def estimate_blocks(sets, vxs: list[float]) -> dict[float, int]:
    """Estimated active blocks of a grid fused from every view of `sets`, at each voxel in `vxs`."""
    acc = {vx: np.empty(0, np.int64) for vx in vxs}
    for root, views in sets:
        for v in views:
            depth = depthset.load_depth(root, v)
            for vx in vxs:
                acc[vx] = np.union1d(acc[vx], _view_block_codes(depth, np.asarray(v.K, np.float64),
                                                                np.asarray(v.w2c, np.float64), vx))
    return {vx: len(c) for vx, c in acc.items()}


def voxel_for(sets, voxel: float, max_blocks: int) -> tuple[float, int]:
    """The finest of `voxel` x VOXEL_LADDER whose estimated block count fits ESTIMATE_HEADROOM of
    `max_blocks` (the coarsest when none fits; the grid cap still guards it), and its estimate."""
    vxs = [m * voxel for m in VOXEL_LADDER]
    est = estimate_blocks(sets, vxs)
    for vx in vxs:
        if est[vx] <= ESTIMATE_HEADROOM * max_blocks:
            return vx, est[vx]
    return vxs[-1], est[vxs[-1]]


def fuse_depthset(roots: list[Path], voxel: float, device: str | None = None, budget_mb: int | None = None):
    """`(mesh, cloud, fusion)`: the surface and coloured points fused from every view of every depth
    set in `roots`, and which attempt produced them. The voxel is first coarsened from `voxel` until the
    estimated grid fits INTEGRATE_MAX_BLOCKS. On CUDA each attempt runs in its own child process with its
    block cap from `budget_mb`, or from the VRAM free before it when None."""
    import open3d.core as o3c

    on_cuda = (device or "").upper().startswith("CUDA") or (device is None and o3c.cuda.is_available())
    sets = [(Path(r), depthset.read_depthset(Path(r))[1]) for r in roots]
    n = sum(len(views) for _, views in sets)
    common.progress(0.02, "estimating the grid size")
    vx0, est = voxel_for(sets, voxel, INTEGRATE_MAX_BLOCKS)
    if on_cuda and est > ESTIMATE_HEADROOM * max_blocks_for(budget_mb if budget_mb is not None
                                                            else gpu_budget_mb(device)):
        common.progress(0.03, f"an estimated {est} blocks exceed the GPU budget; fusing on the CPU")
        on_cuda, device = False, "CPU:0"
    if not on_cuda:
        # Preallocated so the map never rehashes: CPU growth doubles the capacity, and old and new tables
        # live together while it does.
        prealloc = min(int(est * 1.1) + BLOCK_COUNT, INTEGRATE_MAX_BLOCKS)
        mesh, cloud, info = _fuse_once(sets, vx0, device, None, n, prealloc)
        return mesh, cloud, {**info, "attempt": 1, "estimated_blocks": est}
    attempts = [m * vx0 for m in COARSER]
    for k, vx in enumerate(attempts):
        budget = budget_mb if budget_mb is not None else gpu_budget_mb(device)
        try:
            mesh, cloud, info = _fuse_in_child(sets, vx, device, max_blocks_for(budget), n)
            return mesh, cloud, {**info, "attempt": k + 1, "budget_mb": budget, "estimated_blocks": est}
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
