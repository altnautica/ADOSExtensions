"""Build a triangle mesh of the trained scene.

With gsplat kernels available ("tsdf"): render expected depth from the splat at every training
view, fuse into an Open3D tensor VoxelBlockGrid TSDF and extract the surface. "2dgs": train a
second pass with 2D gaussians (mesh2dgs.py) and fuse its median depth the same way. With
--lidar ("lidar_poisson"): screened Poisson reconstruction of a 3D LiDAR's registered cloud,
coloured by the nearest splat. Without gsplat kernels ("poisson"): screened Poisson over the
splat centres with normals oriented towards the nearest camera. All drop small disconnected
clusters, decimate to --max-tris and write <out>/mesh.ply (binary) and <out>/mesh.glb, world frame.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

import common

MAX_RENDER_SIDE = 1280
BLOCK_RESOLUTION = 16
BLOCK_COUNT = 10_000  # initial hash capacity; the grid grows past it
TRUNC_VOXELS = 4.0  # TSDF truncation, in voxels
TSDF_MIN_WEIGHT = 3.0  # a surface voxel must be seen by more than this many views
POISSON_DEPTH = 9
DENSITY_TRIM = 0.05
MIN_OPACITY = 0.1
LIDAR_POISSON_DEPTH = 10
LIDAR_DENSITY_TRIM = 0.02
LIDAR_NORMAL_NEIGHBOURS = 20
CLUSTER_MIN_TRIANGLES = 50  # smaller disconnected clusters are dropped ...
CLUSTER_MIN_FRACTION = 0.001  # ... and so are those under this fraction of the largest


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--colmap", required=True, type=Path, help="dir containing sparse/0 (BIN model)")
    p.add_argument("--images", required=True, type=Path, help="images dir (view list comes from the COLMAP model)")
    p.add_argument("--splat", required=True, type=Path, help="3DGS PLY")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--voxel", type=float, default=0.02, help="TSDF voxel / Poisson input sampling size, metres")
    p.add_argument("--max-tris", type=int, default=300_000)
    p.add_argument("--method", choices=("tsdf", "2dgs"), default="tsdf",
                   help="depth source of the TSDF: the splat, or a 2D-gaussian training pass")
    p.add_argument("--steps", type=int, default=30000, help="training steps of the 2DGS pass")
    p.add_argument("--masks", type=Path, help="masks of the 2DGS pass (<masks>/<image rel>.png)")
    p.add_argument("--lidar", type=Path, help="registered 3D LiDAR cloud (LAZ) to mesh by Poisson")
    return p.parse_args(argv)


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
        """rgb HxWx3 float 0..1, depth HxW metres (0 = none). False when the view has too little depth."""
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


def is_out_of_memory(e: BaseException) -> bool:
    return "out of memory" in str(e).lower()


# Fusion attempts in order: the GPU at the asked voxel, the GPU at twice the voxel (an
# eighth of the blocks), then the host at twice the voxel with a block cap
# (CPU_MAX_BLOCKS x 16^3 voxels x 20 B is about 8 GB).
CPU_MAX_BLOCKS = 100_000


def fusion_attempts(voxel: float) -> list[dict]:
    return [
        {"device": None, "voxel": voxel, "max_blocks": None},
        {"device": None, "voxel": 2 * voxel, "max_blocks": None},
        {"device": "CPU:0", "voxel": 2 * voxel, "max_blocks": CPU_MAX_BLOCKS},
    ]


def tsdf_mesh(args, renderer, views):
    """`(mesh, fusion)`: the surface fused from `renderer`'s depth (anything with
    common.SplatRenderer's `render`) and which fusion attempt produced it."""
    attempts = fusion_attempts(args.voxel)
    for k, a in enumerate(attempts):
        try:
            mesh, fuser = fuse(renderer, views, **a)
            return mesh, {"device": str(fuser.device), "voxel_m": a["voxel"], "attempt": k + 1,
                          "views_capped": fuser.capped}
        except RuntimeError as e:
            if not is_out_of_memory(e) or k == len(attempts) - 1:
                raise
        # Outside the handler, so the traceback no longer pins the GPU grid.
        release_gpu()
        nxt = attempts[k + 1]
        common.progress(0.05, f"TSDF grid out of GPU memory: retrying on {nxt['device'] or 'the GPU'} "
                              f"at voxel {nxt['voxel']:.3f} m")
    raise AssertionError("unreachable")


def release_gpu() -> None:
    import gc

    import open3d.core as o3c

    gc.collect()
    if o3c.cuda.is_available():
        o3c.cuda.release_cache()


def fuse(renderer, views, device: str | None, voxel: float, max_blocks: int | None):
    fuser = TsdfFuser(voxel, device, max_blocks)
    for i, v in enumerate(views):
        s = min(1.0, MAX_RENDER_SIDE / max(v.width, v.height))
        w, h = max(1, round(v.width * s)), max(1, round(v.height * s))
        K = v.K.copy()
        K[0, :] *= w / v.width
        K[1, :] *= h / v.height
        rgb, depth, alpha = renderer.render(v.w2c, K, w, h, "RGB+ED")
        depth = np.where((alpha > 0.5) & np.isfinite(depth), depth, 0.0).astype(np.float32)
        fuser.integrate(np.clip(rgb, 0.0, 1.0), depth, K, v.w2c)
        common.progress(0.45 + 0.4 * (i + 1) / len(views), f"fused view {i + 1}/{len(views)}")
    if fuser.used == 0:
        raise RuntimeError("no view rendered usable depth")
    common.progress(0.86, "extracting surface")
    return fuser.mesh(), fuser


def orient_towards_cameras(pcd, views) -> None:
    """Flip each normal of `pcd` to face the nearest camera centre."""
    import open3d as o3d
    from scipy.spatial import cKDTree

    if not views:
        return
    p = np.asarray(pcd.points)
    n = np.asarray(pcd.normals).copy()
    centers = np.stack([np.linalg.inv(v.w2c)[:3, 3] for v in views])
    _, idx = cKDTree(centers).query(p)
    flip = np.einsum("ij,ij->i", n, centers[idx] - p) < 0
    n[flip] *= -1
    pcd.normals = o3d.utility.Vector3dVector(n)


def splat_colours(splat) -> tuple[np.ndarray, np.ndarray]:
    """(centres, base colours 0..1) of the splats more opaque than MIN_OPACITY."""
    opac = 1.0 / (1.0 + np.exp(-splat["opacities"]))
    keep = opac > MIN_OPACITY
    rgb = np.clip(splat["sh0"][keep, 0, :] * common.SH_C0 + 0.5, 0, 1)
    return splat["means"][keep].astype(np.float64), rgb.astype(np.float64)


def poisson_mesh(args, splat, views):
    import open3d as o3d

    pts, rgb = splat_colours(splat)
    if len(pts) < 100:
        raise RuntimeError(f"only {len(pts)} splats above opacity {MIN_OPACITY}")
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    pcd = pcd.voxel_down_sample(args.voxel)
    common.progress(0.15, f"estimating normals on {len(pcd.points)} points")
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 6, max_nn=30))
    orient_towards_cameras(pcd, views)
    common.progress(0.35, f"poisson depth {POISSON_DEPTH}")
    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=POISSON_DEPTH)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, DENSITY_TRIM))
    return mesh


def lidar_mesh(args, splat, views):
    """Screened Poisson (depth LIDAR_POISSON_DEPTH) over the registered 3D LiDAR cloud with
    normals from LIDAR_NORMAL_NEIGHBOURS neighbours, the lowest-density LIDAR_DENSITY_TRIM of
    vertices cut, coloured by the nearest splat."""
    import laspy
    import open3d as o3d
    from scipy.spatial import cKDTree

    las = laspy.read(str(args.lidar))
    xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], 1).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(1)]
    if len(xyz) < 100:
        raise RuntimeError(f"the LiDAR cloud has only {len(xyz)} points")
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(args.voxel)
    common.progress(0.15, f"estimating normals on {len(pcd.points)} LiDAR points")
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=LIDAR_NORMAL_NEIGHBOURS))
    orient_towards_cameras(pcd, views)
    common.progress(0.35, f"poisson depth {LIDAR_POISSON_DEPTH}")
    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=LIDAR_POISSON_DEPTH)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, LIDAR_DENSITY_TRIM))
    centres, rgb = splat_colours(splat)
    if len(centres):
        _, idx = cKDTree(centres).query(np.asarray(mesh.vertices))
        mesh.vertex_colors = o3d.utility.Vector3dVector(rgb[idx])
    return mesh


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


def finish(args, mesh, method: str, **extra) -> None:
    import open3d as o3d

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    dropped = remove_small_clusters(mesh)
    if len(mesh.triangles) == 0:
        raise RuntimeError("surface extraction produced no triangles")
    if len(mesh.triangles) > args.max_tris:
        common.progress(0.88, f"decimating {len(mesh.triangles)} -> {args.max_tris} triangles")
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=args.max_tris)
        mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    args.out.mkdir(parents=True, exist_ok=True)
    ply, glb = args.out / "mesh.ply", args.out / "mesh.glb"
    if not o3d.io.write_triangle_mesh(str(ply), mesh, write_ascii=False, compressed=False):
        raise IOError(f"cannot write {ply}")
    if not o3d.io.write_triangle_mesh(str(glb), mesh):
        raise IOError(f"cannot write {glb}")
    common.done(triangles=int(len(mesh.triangles)), method=method, cluster_triangles_dropped=dropped,
                ply=str(ply), glb=str(glb), **extra)


def main(args) -> None:
    common.progress(0.0, "loading splat")
    splat = common.read_splat_ply(args.splat)
    views, _, _ = common.load_colmap_views(args.colmap)
    if args.lidar is not None:
        finish(args, lidar_mesh(args, splat, views), "lidar_poisson")
        return
    reason = common.gsplat_unavailable_reason()
    if reason is not None:
        print(f"{reason}; using Poisson over splat centres", file=sys.stderr)
        finish(args, poisson_mesh(args, splat, views), "poisson")
        return
    if args.method == "2dgs":
        import mesh2dgs

        renderer = mesh2dgs.train_2dgs(args, max_splats=len(splat["means"]))
    else:
        renderer = common.SplatRenderer(splat)
    mesh, fusion = tsdf_mesh(args, renderer, views)
    finish(args, mesh, args.method, fusion=fusion)


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
