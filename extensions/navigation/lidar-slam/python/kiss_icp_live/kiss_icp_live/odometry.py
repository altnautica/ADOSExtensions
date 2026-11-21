"""KISS-ICP per scan, plus the translational ICP Hessian for degeneracy.

Hessian: with a point-to-point residual ``r = R p + t - q`` the translation
Jacobian is the identity, so its ``J^T J`` is ``n * I`` for any geometry and
says nothing about degeneracy. The geometry enters through the
point-to-plane linearisation of the same correspondences, ``r = n_q^T (R p +
t - q)``, whose translation Jacobian is the map normal ``n_q``; its Hessian
``sum n_q n_q^T`` has a near-zero eigenvalue along any direction no surface
constrains (the axis of a long corridor). Normals are the smallest principal
axis of the map points sharing the correspondence's cell of a grid 1.5 times
the map's voxel (at least ``normal_k`` of them), from per-cell moments, so no
search structure is built per scan. A correspondence is a scan point (final
pose) whose point-to-plane distance to its cell's points is within KISS-ICP's
own gate, ``3 * sigma`` of the adaptive threshold, against the map as it stood
before this scan.
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from kiss_icp.config.parser import KISSConfig
from kiss_icp.kiss_icp import KissICP

# Cell indices are offset into 21 bits per axis and packed into one int64.
_BIAS = 1 << 20
# The normals' cell, in map voxels: on a synthetic forest and corridor stream
# it classifies degeneracy as the former 8-nearest-neighbour normals did.
CELL_VOXELS = 1.5


@dataclass
class LiveConfig:
    normal_k: int = 8
    # Smallest / middle eigenvalue bound for a neighbourhood to count as a plane.
    planarity: float = 0.3
    min_points: int = 50


@dataclass
class Result:
    t_ns: int
    pose: np.ndarray  # 4x4 T_w_l
    eig: np.ndarray  # ascending
    n: int


def load_config(path: Optional[Path]) -> tuple[KISSConfig, LiveConfig]:
    raw = json.loads(Path(path).read_text()) if path else {}
    live = LiveConfig(**raw.pop("live", {}))
    cfg = KISSConfig(**raw)
    if cfg.data.max_range < cfg.data.min_range:
        cfg.data.min_range = 0.0
    if cfg.mapping.voxel_size is None:
        cfg.mapping.voxel_size = float(cfg.data.max_range / 100.0)
    return cfg, live


def _cell_keys(pts: np.ndarray, cell: float) -> np.ndarray:
    c = np.floor(pts / cell).astype(np.int64) + _BIAS
    return (c[:, 0] << 42) | (c[:, 1] << 21) | c[:, 2]


def translational_hessian(
    source_w: np.ndarray, map_pts: np.ndarray, gate: float, live: LiveConfig, cell: float
) -> tuple[np.ndarray, int]:
    """``sum n n^T`` over gated correspondences; returns (3x3, count)."""
    k = live.normal_k
    if len(map_pts) < k or len(source_w) == 0:
        return np.zeros((3, 3)), 0
    keys = _cell_keys(source_w, cell)
    mkeys = _cell_keys(map_pts, cell)
    near = np.isin(mkeys, keys)
    if near.sum() < k:
        return np.zeros((3, 3)), 0
    pts = map_pts[near]
    cells, inv, count = np.unique(mkeys[near], return_inverse=True, return_counts=True)
    m = len(cells)
    mean = np.stack([np.bincount(inv, pts[:, i], m) for i in range(3)], axis=1) / count[:, None]
    d = pts - mean[inv]
    cov = np.empty((m, 3, 3))
    for i in range(3):
        for j in range(i, 3):
            cov[:, i, j] = cov[:, j, i] = np.bincount(inv, d[:, i] * d[:, j], m) / count
    w, v = np.linalg.eigh(cov)  # ascending per cell
    plane = (count >= k) & (w[:, 0] <= live.planarity * np.maximum(w[:, 1], 1e-12))
    at = np.minimum(np.searchsorted(cells, keys), m - 1)
    ok = (cells[at] == keys) & plane[at]
    at = at[ok]
    normals = v[at, :, 0]
    residual = np.abs(np.einsum("mi,mi->m", source_w[ok] - mean[at], normals))
    normals = normals[residual <= gate]
    return normals.T @ normals, int(len(normals))


class LiveOdometry:
    def __init__(self, cfg: KISSConfig, live: LiveConfig):
        self.cfg = cfg
        self.live = live
        self.kiss = KissICP(cfg)

    def process(self, t_ns: int, points: np.ndarray, t_offset_ns: np.ndarray) -> Optional[Result]:
        if len(points) < self.live.min_points:
            print(f"kiss_icp_live: scan at {t_ns} has {len(points)} points, skipped", file=sys.stderr)
            return None
        offs = np.asarray(t_offset_ns, dtype=np.float64)
        span = float(offs.max() - offs.min())
        # KISS-ICP normalises the times to the scan; a zero span has nothing
        # to deskew (and would divide by zero), so it gets no times at all.
        stamps = offs if span > 0 else np.empty(0)
        map_before = self.kiss.local_map.point_cloud()
        gate = 3.0 * self.kiss.adaptive_threshold.get_threshold()
        _, source = self.kiss.register_frame(np.ascontiguousarray(points, dtype=np.float64), stamps)
        pose = np.array(self.kiss.last_pose, dtype=np.float64)
        source_w = source @ pose[:3, :3].T + pose[:3, 3]
        cell = CELL_VOXELS * float(self.cfg.mapping.voxel_size)
        h, n = translational_hessian(source_w, map_before, gate, self.live, cell)
        eig = np.clip(np.linalg.eigvalsh(h), 0.0, None)
        return Result(t_ns=int(t_ns + int(offs.max())), pose=pose, eig=eig, n=n)
