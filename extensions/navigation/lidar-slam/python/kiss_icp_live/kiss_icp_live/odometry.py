"""KISS-ICP per scan, plus the translational ICP Hessian for degeneracy.

Hessian: with a point-to-point residual ``r = R p + t - q`` the translation
Jacobian is the identity, so its ``J^T J`` is ``n * I`` for any geometry and
says nothing about degeneracy. The geometry enters through the
point-to-plane linearisation of the same correspondences, ``r = n_q^T (R p +
t - q)``, whose translation Jacobian is the map normal ``n_q``; its Hessian
``sum n_q n_q^T`` has a near-zero eigenvalue along any direction no surface
constrains (the axis of a long corridor). Normals are the smallest principal
axis of the ``normal_k`` nearest map points around each correspondence;
correspondences use the final pose and KISS-ICP's own gate, ``3 * sigma`` of
the adaptive threshold, against the map as it stood before this scan.
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
from scipy.spatial import cKDTree


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


def translational_hessian(
    source_w: np.ndarray, map_pts: np.ndarray, gate: float, live: LiveConfig, radius: float
) -> tuple[np.ndarray, int]:
    """``sum n n^T`` over gated correspondences; returns (3x3, count)."""
    k = live.normal_k
    if len(map_pts) < k or len(source_w) == 0:
        return np.zeros((3, 3)), 0
    tree = cKDTree(map_pts)
    dist, idx = tree.query(source_w, k=k, distance_upper_bound=max(radius, gate))
    ok = (dist[:, 0] <= gate) & np.isfinite(dist).all(axis=1)
    if not ok.any():
        return np.zeros((3, 3)), 0
    nb = map_pts[idx[ok]]  # (m, k, 3)
    centred = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("mki,mkj->mij", centred, centred) / k
    w, v = np.linalg.eigh(cov)  # ascending per neighbourhood
    planar = w[:, 0] <= live.planarity * np.maximum(w[:, 1], 1e-12)
    normals = v[planar, :, 0]
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
        radius = 3.0 * float(self.cfg.mapping.voxel_size)
        h, n = translational_hessian(source_w, map_before, gate, self.live, radius)
        eig = np.clip(np.linalg.eigvalsh(h), 0.0, None)
        return Result(t_ns=int(t_ns + int(offs.max())), pose=pose, eig=eig, n=n)
