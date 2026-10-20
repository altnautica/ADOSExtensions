"""Build a triangle mesh from a trained splat.

With gsplat kernels available ("tsdf"): render expected depth from the splat
at every training view, fuse into an Open3D tensor VoxelBlockGrid TSDF and
extract the surface. Without them ("poisson"): screened Poisson reconstruction over the
splat centres with normals oriented towards the nearest camera, trimming the
lowest-density 5 % of vertices. Both end with quadric decimation to
--max-tris and write <out>/mesh.ply (binary) and <out>/mesh.glb, world frame.
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
POISSON_DEPTH = 9
DENSITY_TRIM = 0.05
MIN_OPACITY = 0.1


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--colmap", required=True, type=Path, help="dir containing sparse/0 (BIN model)")
    p.add_argument("--images", required=True, type=Path, help="images dir (view list comes from the COLMAP model)")
    p.add_argument("--splat", required=True, type=Path, help="3DGS PLY")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--voxel", type=float, default=0.02, help="TSDF voxel / Poisson input sampling size, metres")
    p.add_argument("--max-tris", type=int, default=300_000)
    return p.parse_args(argv)


class TsdfFuser:
    """TSDF fusion of depth + colour views into a sparse voxel block grid."""

    def __init__(self, voxel: float, device: str | None = None):
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
        self.used = 0

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
        blocks = self.grid.compute_unique_block_coordinates(depth_t, K_t, w2c_t, 1.0, depth_max)
        self.grid.integrate(blocks, depth_t, color_t, K_t, w2c_t, 1.0, depth_max)
        self.used += 1
        return True

    def mesh(self):
        return self.grid.extract_triangle_mesh(weight_threshold=1.0).cpu().to_legacy()


def is_out_of_memory(e: BaseException) -> bool:
    return "out of memory" in str(e).lower()


def tsdf_mesh(args, splat, views):
    renderer = common.SplatRenderer(splat)
    try:
        return fuse(args, renderer, views, None)
    except RuntimeError as e:
        # A large site at a fine voxel outgrows the GPU next to the splat; host memory holds it.
        if not is_out_of_memory(e):
            raise
    # Outside the handler, so the traceback no longer pins the GPU grid.
    import gc

    gc.collect()
    import open3d.core as o3c

    if o3c.cuda.is_available():
        o3c.cuda.release_cache()
    common.progress(0.05, "TSDF grid out of GPU memory: fusing on the CPU")
    return fuse(args, renderer, views, "CPU:0")


def fuse(args, renderer, views, device: str | None):
    fuser = TsdfFuser(args.voxel, device)
    for i, v in enumerate(views):
        s = min(1.0, MAX_RENDER_SIDE / max(v.width, v.height))
        w, h = max(1, round(v.width * s)), max(1, round(v.height * s))
        K = v.K.copy()
        K[0, :] *= w / v.width
        K[1, :] *= h / v.height
        rgb, depth, alpha = renderer.render(v.w2c, K, w, h, "RGB+ED")
        depth = np.where((alpha > 0.5) & np.isfinite(depth), depth, 0.0).astype(np.float32)
        fuser.integrate(np.clip(rgb, 0.0, 1.0), depth, K, v.w2c)
        common.progress(0.05 + 0.75 * (i + 1) / len(views), f"fused view {i + 1}/{len(views)}")
    if fuser.used == 0:
        raise RuntimeError("no view rendered usable depth")
    common.progress(0.82, "extracting surface")
    return fuser.mesh()


def poisson_mesh(args, splat, views):
    import open3d as o3d
    from scipy.spatial import cKDTree

    opac = 1.0 / (1.0 + np.exp(-splat["opacities"]))
    keep = opac > MIN_OPACITY
    pts = splat["means"][keep].astype(np.float64)
    if len(pts) < 100:
        raise RuntimeError(f"only {len(pts)} splats above opacity {MIN_OPACITY}")
    rgb = np.clip(splat["sh0"][keep, 0, :] * common.SH_C0 + 0.5, 0, 1)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64))
    pcd = pcd.voxel_down_sample(args.voxel)
    common.progress(0.15, f"estimating normals on {len(pcd.points)} points")
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 6, max_nn=30))
    p = np.asarray(pcd.points)
    n = np.asarray(pcd.normals)
    if views:
        centers = np.stack([np.linalg.inv(v.w2c)[:3, 3] for v in views])
        _, idx = cKDTree(centers).query(p)
        flip = np.einsum("ij,ij->i", n, centers[idx] - p) < 0
        n[flip] *= -1
        pcd.normals = o3d.utility.Vector3dVector(n)
    common.progress(0.35, f"poisson depth {POISSON_DEPTH}")
    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=POISSON_DEPTH)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, DENSITY_TRIM))
    return mesh


def finish(args, mesh, method: str) -> None:
    import open3d as o3d

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
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
    common.done(triangles=int(len(mesh.triangles)), method=method, ply=str(ply), glb=str(glb))


def main(args) -> None:
    common.progress(0.0, "loading splat")
    splat = common.read_splat_ply(args.splat)
    views, _, _ = common.load_colmap_views(args.colmap)
    reason = common.gsplat_unavailable_reason()
    if reason is None:
        mesh, method = tsdf_mesh(args, splat, views), "tsdf"
    else:
        print(f"{reason}; using Poisson over splat centres", file=sys.stderr)
        mesh, method = poisson_mesh(args, splat, views), "poisson"
    finish(args, mesh, method)


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
