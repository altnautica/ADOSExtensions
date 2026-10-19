"""Per-image training masks: people, independently moving things, mirrors/glass.

Writes <out>/<image rel>.png with 0 = ignore, 255 = keep (the COLMAP
--ImageReader.mask_path convention). Sources, combined with a logical AND:
- person: an ONNX YOLO model trained on COCO (--person-model), class-0 boxes
  filled and dilated;
- motion: dense Farneback flow to the neighbouring frame of the same camera
  compared with the flow the camera motion predicts (from the depth prior and
  the two poses when --depths is given, else a RANSAC homography);
- mirror/glass (needs --depths and --lidar): planar regions of the depth prior
  whose LiDAR depth disagrees by more than 0.3 m.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import common

PERSON_CONF = 0.35
PERSON_PAD = 0.04  # box growth, fraction of box size
FLOW_SIDE = 640
MOTION_MIN_PX = 2.0
MIRROR_DIFF_M = 0.3
MIRROR_GRID = 4  # LiDAR/depth comparison runs at 1/4 resolution
LIDAR_VOXEL_M = 0.05
LIDAR_MAX_RANGE_M = 40.0


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--images", required=True, type=Path)
    p.add_argument("--frames", required=True, type=Path, help="frames.json (names relative to --images, poses, intrinsics)")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--person-model", type=Path, help="ONNX YOLO (COCO classes) detector")
    p.add_argument("--depths", type=Path, help="<depths>/<image rel>.npy depth priors")
    p.add_argument("--lidar", type=Path, help="world-frame LAZ/LAS point cloud")
    p.add_argument("--no-motion", action="store_true", help="skip the motion mask")
    return p.parse_args(argv)


# ---------------------------------------------------------------- person


class PersonDetector:
    def __init__(self, model: Path):
        import onnxruntime as ort

        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
        self.sess = ort.InferenceSession(str(model), providers=providers)
        inp = self.sess.get_inputs()[0]
        self.name = inp.name
        h, w = inp.shape[2], inp.shape[3]
        self.size = (int(w) if isinstance(w, int) else 640, int(h) if isinstance(h, int) else 640)

    def mask(self, rgb: np.ndarray) -> np.ndarray:
        """bool HxW, True where a person is."""
        import cv2

        H, W = rgb.shape[:2]
        tw, th = self.size
        s = min(tw / W, th / H)
        nw, nh = round(W * s), round(H * s)
        px, py = (tw - nw) // 2, (th - nh) // 2
        canvas = np.full((th, tw, 3), 114, np.uint8)
        canvas[py : py + nh, px : px + nw] = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        out = np.asarray(self.sess.run(None, {self.name: x})[0])[0]
        if out.shape[0] < out.shape[1]:
            out = out.T  # [4+C, N] -> [N, 4+C]
        if out.shape[1] == 85:  # box, objectness, 80 classes
            score = out[:, 4] * out[:, 5]
        else:  # box, 80 classes
            score = out[:, 4]
        mask = np.zeros((H, W), bool)
        for cx, cy, bw, bh in out[score > PERSON_CONF, :4]:
            bw, bh = bw * (1 + 2 * PERSON_PAD), bh * (1 + 2 * PERSON_PAD)
            x0 = int(max(0, (cx - bw / 2 - px) / s))
            y0 = int(max(0, (cy - bh / 2 - py) / s))
            x1 = int(min(W, (cx + bw / 2 - px) / s + 1))
            y1 = int(min(H, (cy + bh / 2 - py) / s + 1))
            mask[y0:y1, x0:x1] = True
        return mask


# ---------------------------------------------------------------- motion


def _gray_small(path: Path):
    import cv2

    g = common.read_image(path, gray=True)
    s = min(1.0, FLOW_SIDE / max(g.shape))
    small = cv2.resize(g, (round(g.shape[1] * s), round(g.shape[0] * s)), interpolation=cv2.INTER_AREA) if s < 1 else g
    return small, s, g.shape


def _pose_flow(cur: dict, other: dict, depth: np.ndarray, s: float, shape) -> np.ndarray | None:
    """Flow (small-image pixels) from cur to other predicted by depth and both poses."""
    import cv2

    h, w = shape
    d = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
    valid = d > 0
    if valid.mean() < 0.3:
        return None
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float64)
    K = cur["K"].copy()
    K[:2] *= s
    Ko = other["K"].copy()
    Ko[:2] *= s
    z = d.astype(np.float64)
    pc = np.stack([(xs - K[0, 2]) / K[0, 0] * z, (ys - K[1, 2]) / K[1, 1] * z, z], -1).reshape(-1, 3)
    T = other["w2c"] @ cur["c2w"]
    po = pc @ T[:3, :3].T + T[:3, 3]
    zo = np.where(po[:, 2] > 1e-3, po[:, 2], np.nan)
    uo = Ko[0, 0] * po[:, 0] / zo + Ko[0, 2]
    vo = Ko[1, 1] * po[:, 1] / zo + Ko[1, 2]
    flow = np.stack([uo.reshape(h, w) - xs, vo.reshape(h, w) - ys], -1)
    flow[~valid] = np.nan
    return flow


def _homography_flow(flow: np.ndarray) -> np.ndarray | None:
    import cv2

    h, w = flow.shape[:2]
    ys, xs = np.mgrid[0:h:8, 0:w:8]
    src = np.stack([xs.ravel(), ys.ravel()], 1).astype(np.float32)
    dst = src + flow[ys.ravel(), xs.ravel()]
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None:
        return None
    gy, gx = np.mgrid[0:h, 0:w].astype(np.float32)
    pts = np.stack([gx.ravel(), gy.ravel()], 1)[None]
    warped = cv2.perspectiveTransform(pts, H)[0].reshape(h, w, 2)
    return warped - np.stack([gx, gy], -1)


def motion_mask(cur: dict, other: dict, images: Path, depths: Path | None) -> np.ndarray:
    """bool at full resolution, True where the scene moved independently."""
    import cv2

    a, s, full = _gray_small(images / cur["name"])
    b, _, _ = _gray_small(images / other["name"])
    if a.shape != b.shape:
        return np.zeros(full[:2], bool)
    flow = cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 4, 21, 3, 5, 1.1, 0)
    pred = None
    if depths is not None and common.depth_path(depths, cur["name"]).is_file():
        pred = _pose_flow(cur, other, np.load(common.depth_path(depths, cur["name"])), s, a.shape)
    if pred is None:
        pred = _homography_flow(flow)
    if pred is None:
        return np.zeros(full[:2], bool)
    res = np.linalg.norm(flow - pred, axis=-1)
    finite = np.isfinite(res)
    if finite.mean() < 0.3:
        return np.zeros(full[:2], bool)
    thr = max(MOTION_MIN_PX, 4.0 * float(np.median(res[finite])))
    m = (np.where(finite, res, 0.0) > thr).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.dilate(m, np.ones((9, 9), np.uint8))
    return cv2.resize(m, (full[1], full[0]), interpolation=cv2.INTER_NEAREST) > 0


# ---------------------------------------------------------------- mirrors


def load_lidar(path: Path) -> np.ndarray:
    import laspy

    las = laspy.read(str(path))
    xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], 1)
    keys = np.floor(xyz / LIDAR_VOXEL_M).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return xyz[first]


def mirror_mask(fr: dict, lidar: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """bool at full resolution, True on planar depth-prior regions LiDAR contradicts."""
    import cv2

    H, W = fr["height"], fr["width"]
    h, w = max(1, H // MIRROR_GRID), max(1, W // MIRROR_GRID)
    K = fr["K"].copy()
    K[0] *= w / W
    K[1] *= h / H
    pc = lidar @ fr["w2c"][:3, :3].T + fr["w2c"][:3, 3]
    pc = pc[(pc[:, 2] > 0.1) & (pc[:, 2] < LIDAR_MAX_RANGE_M)]
    u = np.floor(K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]).astype(np.int64)
    v = np.floor(K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]).astype(np.int64)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    lid = np.full(h * w, np.inf)
    np.minimum.at(lid, v[ok] * w + u[ok], pc[ok, 2])
    lid = lid.reshape(h, w)
    mono = cv2.resize(depth.astype(np.float32), (w, h), interpolation=cv2.INTER_NEAREST).astype(np.float64)
    both = np.isfinite(lid) & (mono > 0)
    if both.sum() < 50:
        return np.zeros((H, W), bool)
    mono = mono * float(np.median(lid[both] / mono[both]))
    disagree = both & (np.abs(mono - lid) > MIRROR_DIFF_M)

    # planarity of the prior: back-projected normals agree with their neighbourhood
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float64)
    P = np.stack([(xs - K[0, 2]) / K[0, 0] * mono, (ys - K[1, 2]) / K[1, 1] * mono, mono], -1)
    n = np.cross(np.gradient(P, axis=1), np.gradient(P, axis=0))
    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-12
    nm = cv2.blur(n.astype(np.float32), (5, 5))
    nm /= np.linalg.norm(nm, axis=-1, keepdims=True) + 1e-12
    gz = np.hypot(*np.gradient(mono)) / np.maximum(mono, 1e-3)
    planar = ((n * nm).sum(-1) > 0.98) & (gz < 0.05) & (mono > 0)
    count, labels = cv2.connectedComponents(planar.astype(np.uint8), connectivity=4)
    out = np.zeros((h, w), bool)
    for lab in range(1, count):
        reg = labels == lab
        samples = both[reg].sum()
        if samples >= 20 and disagree[reg].sum() > 0.5 * samples:
            out |= reg
    return cv2.resize(out.astype(np.uint8), (W, H), interpolation=cv2.INTER_NEAREST) > 0


# ---------------------------------------------------------------- main


def main(args) -> None:
    frames = common.load_frames_json(args.frames)
    if not frames:
        raise RuntimeError("frames.json lists no frames")
    use_mirror = args.depths is not None and args.lidar is not None
    detector = PersonDetector(args.person_model) if args.person_model else None
    lidar = load_lidar(args.lidar) if use_mirror else None
    by_cam: dict[str, list[int]] = {}
    for i, fr in enumerate(frames):
        by_cam.setdefault(str(fr.get("camera_id")), []).append(i)
    neighbour = {}
    for idx in by_cam.values():
        idx.sort(key=lambda i: frames[i]["t_ns"])
        for k, i in enumerate(idx):
            if len(idx) > 1:
                neighbour[i] = idx[k - 1] if k > 0 else idx[1]
    counts = {"person": 0, "motion": 0, "mirror": 0}
    for i, fr in enumerate(frames):
        ignore = np.zeros((fr["height"], fr["width"]), bool)
        if detector is not None:
            m = detector.mask(common.read_image(args.images / fr["name"]))
            counts["person"] += int(m.any())
            ignore |= m[: ignore.shape[0], : ignore.shape[1]]
        if not args.no_motion and i in neighbour:
            m = motion_mask(fr, frames[neighbour[i]], args.images, args.depths)
            counts["motion"] += int(m.any())
            ignore |= m[: ignore.shape[0], : ignore.shape[1]]
        if use_mirror and common.depth_path(args.depths, fr["name"]).is_file():
            m = mirror_mask(fr, lidar, np.load(common.depth_path(args.depths, fr["name"])))
            counts["mirror"] += int(m.any())
            ignore |= m
        common.write_png(common.mask_path(args.out, fr["name"]), np.where(ignore, 0, 255).astype(np.uint8))
        common.progress((i + 1) / len(frames), f"mask {i + 1}/{len(frames)}")
    common.done(masks=len(frames), frames_with_person=counts["person"], frames_with_motion=counts["motion"], frames_with_mirror=counts["mirror"])


if __name__ == "__main__":
    a = parse_args()
    common.run_main(lambda: main(a))
