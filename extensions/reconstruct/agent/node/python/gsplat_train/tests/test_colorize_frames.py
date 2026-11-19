from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import colorize  # noqa: E402

# A camera at world (10, 2, 1) looking along world +x (OpenCV: x right = world -y, y down = world -z,
# z forward = world +x).
R_WORLD_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CENTRE = np.array([10.0, 2.0, 1.0])


def _frame(name: str, **over) -> dict:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_WORLD_CAM, CENTRE
    f = {"name": name, "camera_id": "cam0", "t_ns": 0, "T_world_cam": T.ravel().tolist(), "width": 64,
         "height": 48, "fx": 40.0, "fy": 40.0, "cx": 32.0, "cy": 24.0, "model": "pinhole", "dist": []}
    f.update(over)
    return f


def _dataset(tmp_path: Path, frames: list[dict]) -> Path:
    (tmp_path / "frames.json").write_text(json.dumps(frames))
    return tmp_path


def test_a_frame_pose_is_world_from_camera(tmp_path):
    views, distorted = colorize.views_from_frames(_dataset(tmp_path, [_frame("cam0/a.png")]))
    assert distorted == 0 and len(views) == 1
    v = views[0]
    # A point 3 m ahead of the camera and 0.5 m to its left (world +y) lands at z = 3, x = -0.5.
    p = v.w2c @ np.array([13.0, 2.5, 1.0, 1.0])
    assert np.allclose(p[:3], [-0.5, 0.0, 3.0])
    assert v.image == tmp_path / "images" / "cam0" / "a.png"
    assert v.mask is None


def test_distorted_and_unposed_frames_are_left_out(tmp_path):
    frames = [_frame("cam0/a.png"), _frame("cam1/b.png", model="opencv_fisheye", dist=[0.1, 0, 0, 0]),
              _frame("cam2/c.png", model="opencv", dist=[0.0, 0.01, 0, 0]), _frame("cam0/d.png", T_world_cam=None),
              _frame("cam3/e.png", model="opencv", dist=[0.0, 0.0, 0.0, 0.0])]
    views, distorted = colorize.views_from_frames(_dataset(tmp_path, frames))
    assert distorted == 2
    assert [v.image.name for v in views] == ["a.png", "e.png"]


def test_a_frame_takes_its_mask(tmp_path):
    ds = _dataset(tmp_path, [_frame("cam0/a.png")])
    mask = ds / "masks" / "cam0" / "a.png.png"
    mask.parent.mkdir(parents=True)
    mask.write_bytes(b"png")
    views, _ = colorize.views_from_frames(ds)
    assert views[0].mask == mask
