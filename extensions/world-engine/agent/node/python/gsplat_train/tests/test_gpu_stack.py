"""Every GPU library the world-recreation pipeline needs runs on this GPU (sm_120)."""
from __future__ import annotations

import shutil
import subprocess

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs a CUDA GPU")


def test_torch_has_native_kernels_for_this_gpu():
    major, minor = torch.cuda.get_device_capability(0)
    assert f"sm_{major}{minor}" in torch.cuda.get_arch_list()


def test_gsplat_rasterises():
    from gsplat import rasterization

    d = "cuda"
    means = torch.tensor([[0.0, 0.0, 3.0]], device=d)
    img, alpha, _ = rasterization(means, torch.tensor([[1.0, 0, 0, 0]], device=d),
                                  torch.full((1, 3), 0.2, device=d), torch.tensor([0.9], device=d),
                                  torch.ones(1, 3, device=d), torch.eye(4, device=d)[None],
                                  torch.tensor([[[50.0, 0, 32], [0, 50.0, 32], [0, 0, 1]]], device=d), 64, 64)
    assert alpha[0, 32, 32, 0] > 0.5


def test_nvdiffrast_rasterises_a_triangle():
    dr = pytest.importorskip("nvdiffrast.torch", reason="nvdiffrast is not a dependency of this project")

    ctx = dr.RasterizeCudaContext()
    pos = torch.tensor([[[-0.8, -0.8, 0.5, 1.0], [0.8, -0.8, 0.5, 1.0], [0.0, 0.8, 0.5, 1.0]]], device="cuda")
    tri = torch.tensor([[0, 1, 2]], dtype=torch.int32, device="cuda")
    rast, _ = dr.rasterize(ctx, pos, tri, resolution=[32, 32])
    assert int((rast[0, ..., 3] > 0).sum()) > 100


def test_xatlas_unwraps_a_quad():
    import xatlas

    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], np.float32)
    f = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    vmap, idx, uv = xatlas.parametrize(v, f)
    assert idx.shape == (2, 3) and uv.min() >= 0.0 and uv.max() <= 1.0


def test_open3d_has_a_cuda_device():
    import open3d.core as o3c

    assert o3c.cuda.is_available()
    t = o3c.Tensor(np.ones((4, 3), np.float32)).to(o3c.Device("CUDA:0"))
    assert float(t.sum().item()) == 12.0


def test_torchmetrics_lpips_and_ssim_run_on_cuda():
    tm = pytest.importorskip("torchmetrics")
    from torchmetrics.functional.image import structural_similarity_index_measure as ssim
    from torchmetrics.image.lpip import LearnedPerceptualImagePatchSimilarity

    a = torch.rand(1, 3, 64, 64, device="cuda")
    assert float(ssim(a, a, data_range=1.0)) > 0.999
    lp = LearnedPerceptualImagePatchSimilarity(net_type="alex", normalize=True).cuda()
    assert float(lp(a, a)) < 1e-3
    assert tm.__version__


def test_nvenc_encodes_ten_frames(tmp_path):
    ff = shutil.which("ffmpeg")
    if ff is None:
        pytest.skip("ffmpeg is only in the evaluation image")
    out = tmp_path / "t.mp4"
    r = subprocess.run([ff, "-y", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=640x360:rate=30",
                        "-frames:v", "10", "-c:v", "h264_nvenc", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert out.stat().st_size > 0
