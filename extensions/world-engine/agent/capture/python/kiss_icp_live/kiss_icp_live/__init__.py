"""Live KISS-ICP LiDAR odometry for the World Engine capture service.

``python -m kiss_icp_live --socket <path> --config <json>`` connects to the
capture service's Unix socket, reads scans framed as in :mod:`.framing`, runs
KISS-ICP (MIT, PRBonn/kiss-icp) on each with per-point deskew, and writes one
JSON line per scan back on the same socket.
"""

from __future__ import annotations

from typing import BinaryIO

from .framing import encode_result, read_scan
from .odometry import LiveOdometry


def serve(rfile: BinaryIO, wfile: BinaryIO, odom: LiveOdometry) -> int:
    """Process scans until the stream ends; returns the scans answered."""
    answered = 0
    while (scan := read_scan(rfile)) is not None:
        res = odom.process(scan.t_ns, scan.points, scan.t_offset_ns)
        if res is None:
            continue
        wfile.write(encode_result(res.t_ns, res.pose, res.eig, res.n))
        wfile.flush()
        answered += 1
    return answered
