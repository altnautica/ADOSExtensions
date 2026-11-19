"""Build a triangle mesh of the trained scene.

With gsplat kernels available ("tsdf"): render colour and expected depth from the splat at every
training view into a depth set (depthset.py) and hand it, with any --extra-depthset, to fuse.py,
which runs TSDF fusion in its own process (no torch CUDA context) and writes the mesh and a dense
coloured cloud. "2dgs": train a second pass with 2D gaussians (mesh2dgs.py) and fuse its median
depth the same way. With --lidar ("lidar_poisson"): screened Poisson reconstruction of a 3D
LiDAR's registered cloud, coloured by the nearest splat. Without gsplat kernels ("poisson"):
screened Poisson over the splat centres with normals oriented towards the nearest camera. All drop
small disconnected clusters, decimate to --max-tris and write <out>/mesh.ply (binary) and
<out>/mesh.glb, world frame.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

import common

MAX_RENDER_SIDE = 1280
POISSON_DEPTH = 9
DENSITY_TRIM = 0.05
MIN_OPACITY = 0.1
LIDAR_POISSON_DEPTH = 10
LIDAR_DENSITY_TRIM = 0.02
LIDAR_NORMAL_NEIGHBOURS = 20


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
    p.add_argument("--extra-depthset", action="append", type=Path, help="more depth sets to fuse (MVS)")
    return p.parse_args(argv)


def splat_depthset(renderer, views, root: Path) -> Path:
    """Render `renderer`'s colour and expected depth (anything with common.SplatRenderer's
    `render`) at every training view into a depth set at `root`."""
    import depthset

    out = []
    for i, v in enumerate(views):
        s = min(1.0, MAX_RENDER_SIDE / max(v.width, v.height))
        w, h = max(1, round(v.width * s)), max(1, round(v.height * s))
        K = v.K.copy()
        K[0, :] *= w / v.width
        K[1, :] *= h / v.height
        rgb, depth, alpha = renderer.render(v.w2c, K, w, h, "RGB+ED")
        depth = np.where((alpha > 0.5) & np.isfinite(depth), depth, 0.0).astype(np.float32)
        rgb_u8 = np.round(np.clip(rgb, 0.0, 1.0) * 255).astype(np.uint8)
        out.append(depthset.save_view(root, i, v.name, rgb_u8, depth, K, v.w2c))
        common.progress(0.45 * (i + 1) / len(views), f"rendered depth {i + 1}/{len(views)}")
    depthset.write_depthset(root, out, frame="model")
    return root


def run_fuse(args, roots: list[Path], label: str, discard: tuple[Path, ...] = ()) -> None:
    """fuse.py over `roots`; its stdout (progress and the done event) is inherited. The depth sets in
    `discard` (rendered for this run only, about 4 MB a view) are deleted afterwards, fused or not."""
    cmd = [sys.executable, str(Path(__file__).with_name("fuse.py")), "--out", str(args.out), "--voxel",
           str(args.voxel), "--max-tris", str(args.max_tris), "--method-label", label]
    for r in roots:
        cmd += ["--depthset", str(r)]
    try:
        r = subprocess.run(cmd)
    finally:
        for d in discard:
            shutil.rmtree(d, ignore_errors=True)
    if r.returncode != 0:
        raise RuntimeError(f"fuse.py exited with {r.returncode}")



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


def main(args) -> None:
    import fuse

    common.progress(0.0, "loading splat")
    splat = common.read_splat_ply(args.splat)
    views, _, _ = common.load_colmap_views(args.colmap)
    if args.lidar is not None:
        mesh, method = lidar_mesh(args, splat, views), "lidar_poisson"
    elif (reason := common.gsplat_unavailable_reason()) is not None:
        print(f"{reason}; using Poisson over splat centres", file=sys.stderr)
        mesh, method = poisson_mesh(args, splat, views), "poisson"
    else:
        if args.method == "2dgs":
            import mesh2dgs

            renderer = mesh2dgs.train_2dgs(args, max_splats=len(splat["means"]))
        else:
            renderer = common.SplatRenderer(splat)
        splat_root = splat_depthset(renderer, views, args.out / "depthset-splat")
        roots = [splat_root] + list(args.extra_depthset or [])
        del renderer
        import torch

        torch.cuda.empty_cache()
        run_fuse(args, roots, args.method, discard=(splat_root,))
        return
    mesh, dropped, ply, glb = fuse.finish(args.out, mesh, args.max_tris)
    common.done(triangles=int(len(mesh.triangles)), method=method, cluster_triangles_dropped=dropped,
                ply=str(ply), glb=str(glb))


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
