"""Wire framing between the capture service and this process.

Capture -> odometry, one frame per scan, little-endian::

    u32  len            bytes after this field (= 12 + 16 * n)
    u64  t_ns           host monotonic ns of the first point
    u32  n              point count
    f32  xyz[n][3]      points, metres, LiDAR frame
    u32  t_offset[n]    per-point ns after t_ns

Odometry -> capture, one JSON object per line (UTF-8, ``\\n``-terminated)::

    {"t_ns": int, "T_w_l": [16 floats, row-major], "eig": [3 floats, ascending], "n": int}

``t_ns`` is the time the pose holds at (the scan's last point: KISS-ICP
deskews every scan to its end); ``T_w_l`` is the LiDAR pose in the odometry
frame (the LiDAR frame at the first scan); ``eig`` are the eigenvalues of the
translational ICP Hessian and ``n`` the correspondences it was summed over.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from typing import BinaryIO, Optional

import numpy as np

_LEN = struct.Struct("<I")
_HEAD = struct.Struct("<QI")
# A Mid-360 scan is ~20k points; this bounds a corrupt length field.
MAX_POINTS = 2_000_000


class FramingError(ValueError):
    pass


@dataclass
class Scan:
    t_ns: int
    points: np.ndarray  # (n, 3) float64
    t_offset_ns: np.ndarray  # (n,) uint32


def encode_scan(t_ns: int, points: np.ndarray, t_offset_ns: np.ndarray) -> bytes:
    pts = np.ascontiguousarray(points, dtype="<f4").reshape(-1, 3)
    offs = np.ascontiguousarray(t_offset_ns, dtype="<u4").reshape(-1)
    if len(pts) != len(offs):
        raise FramingError("points and t_offset_ns differ in length")
    body = _HEAD.pack(t_ns, len(pts)) + pts.tobytes() + offs.tobytes()
    return _LEN.pack(len(body)) + body


def _read_exact(stream: BinaryIO, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            if buf:
                raise FramingError("stream ended inside a frame")
            return None
        buf += chunk
    return bytes(buf)


def read_scan(stream: BinaryIO) -> Optional[Scan]:
    """The next scan, or None at a clean end of stream."""
    head = _read_exact(stream, _LEN.size)
    if head is None:
        return None
    (length,) = _LEN.unpack(head)
    if length < _HEAD.size or (length - _HEAD.size) % 16:
        raise FramingError(f"bad frame length {length}")
    body = _read_exact(stream, length)
    if body is None:
        raise FramingError("stream ended inside a frame")
    t_ns, n = _HEAD.unpack_from(body)
    if n > MAX_POINTS or length != _HEAD.size + 16 * n:
        raise FramingError(f"frame length {length} does not fit {n} points")
    off = _HEAD.size
    pts = np.frombuffer(body, dtype="<f4", count=3 * n, offset=off).reshape(n, 3)
    offs = np.frombuffer(body, dtype="<u4", count=n, offset=off + 12 * n)
    return Scan(t_ns=t_ns, points=pts.astype(np.float64), t_offset_ns=offs.copy())


def encode_result(t_ns: int, pose: np.ndarray, eig, n: int) -> bytes:
    line = {
        "t_ns": int(t_ns),
        "T_w_l": [float(v) for v in np.asarray(pose, dtype=float).reshape(16)],
        "eig": [float(v) for v in eig],
        "n": int(n),
    }
    return (json.dumps(line, separators=(",", ":")) + "\n").encode()
