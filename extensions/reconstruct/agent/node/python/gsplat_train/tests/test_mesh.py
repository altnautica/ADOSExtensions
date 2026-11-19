"""The splat depth set handed to fuse.py, and the 3D-LiDAR Poisson mesh."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mesh  # noqa: E402


def test_splat_depthset_renders_every_view_scaled_and_masked(tmp_path) -> None:
    import types

    import depthset

    class Renderer:
        def render(self, w2c, K, w, h, mode):
            assert mode == "RGB+ED"
            rgb = np.full((h, w, 3), 1.5, np.float32)  # out of range: clipped to white
            depth = np.full((h, w), 2.0, np.float32)
            depth[0, 0] = np.inf
            alpha = np.ones((h, w), np.float32)
            alpha[0, 1] = 0.2
            return rgb, depth, alpha

    K = np.array([[2000.0, 0, 1280], [0, 2000.0, 720], [0, 0, 1]])
    views = [types.SimpleNamespace(name="a.jpg", K=K, w2c=np.eye(4), width=2560, height=1440)]
    root = mesh.splat_depthset(Renderer(), views, tmp_path / "ds")
    frame, out = depthset.read_depthset(root)
    assert frame == "model" and out[0].name == "a.jpg"
    assert (out[0].width, out[0].height) == (1280, 720)
    assert np.allclose(out[0].K, K * np.array([[0.5], [0.5], [1]]))
    d = depthset.load_depth(root, out[0])
    assert d[0, 0] == 0 and d[0, 1] == 0 and d[1, 1] == 2.0
    assert depthset.load_rgb(root, out[0])[1, 1].tolist() == [255, 255, 255]


def test_the_rendered_depth_set_is_deleted_after_fusion_even_when_it_fails(tmp_path, monkeypatch) -> None:
    import subprocess
    import types

    import pytest

    rendered, extra = tmp_path / "depthset-splat", tmp_path / "mvs"
    for d in (rendered, extra):
        d.mkdir()
        (d / "view.png").write_bytes(b"x")
    args = types.SimpleNamespace(out=tmp_path, voxel=0.02, max_tris=10)
    for rc in (0, 1):
        monkeypatch.setattr(subprocess, "run", lambda cmd, rc=rc: types.SimpleNamespace(returncode=rc))
        rendered.mkdir(exist_ok=True)
        if rc:
            with pytest.raises(RuntimeError, match="exited with 1"):
                mesh.run_fuse(args, [rendered, extra], "tsdf", discard=(rendered,))
        else:
            mesh.run_fuse(args, [rendered, extra], "tsdf", discard=(rendered,))
        assert not rendered.exists() and (extra / "view.png").exists()


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
