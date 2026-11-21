#!/usr/bin/env python3
"""Run `kiss_icp_live` from a KISS-ICP runtime on a synthetic 3D LiDAR stream.

Plays the capture service's side of the socket (`kiss_icp_live/framing.py`):
binds the Unix socket, starts `<python> -s -E -B -m kiss_icp_live` exactly as
the lidar-slam service does, sends a synthetic Mid-360-like scan (20 000
points over 0.1 s, a sensor walking through a stand of trunks over flat
ground) at the given rate, and reads the poses back. Prints the scans
answered, the reply latency, the child's resident memory and its CPU share,
and exits non-zero when fewer than 90 % of the scans were answered or a pose
is not finite.

Stdlib only, so the host python runs it whatever the runtime's arch:

    stream_check.py --python <runtime>/bin/python3 --project <kiss_icp_live dir>
                    [--seconds 30] [--rate 10] [--points 20000]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import threading
import time
from array import array
from pathlib import Path

CLK_TCK = os.sysconf("SC_CLK_TCK")
SCAN_S = 0.1
SPEED_MPS = 1.0
SENSOR_Z_M = 1.5


def scene(seed: int) -> list[tuple[float, float, float]]:
    """Ground, trunks every ~4 m and two walls along a 60 m strip."""
    rng = random.Random(seed)
    pts: list[tuple[float, float, float]] = []
    for _ in range(60_000):
        pts.append((rng.uniform(-20, 80), rng.uniform(-25, 25), rng.gauss(0.0, 0.02)))
    for gx in range(-20, 80, 4):
        for gy in range(-24, 25, 4):
            cx, cy, r = gx + rng.uniform(-1, 1), gy + rng.uniform(-1, 1), rng.uniform(0.1, 0.3)
            if abs(cy) < 1.5:
                continue
            for _ in range(250):
                a = rng.uniform(0, 2 * math.pi)
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a), rng.uniform(0, 8)))
    for _ in range(30_000):
        x, z = rng.uniform(-20, 80), rng.uniform(0, 6)
        pts.append((x, rng.choice((-25.0, 25.0)), z))
    return pts


def scan_frame(world: list, t: float, t_ns: int, n: int, rng: random.Random) -> bytes:
    """One frame: `n` scene points in the sensor frame, timed over 0.1 s."""
    xyz = array("f")
    offs = array("I")
    for i, (wx, wy, wz) in enumerate(rng.sample(world, n)):
        dt = SCAN_S * i / n
        tt = t + dt
        cx, cy, yaw = SPEED_MPS * tt, 0.5 * math.sin(0.2 * tt), 0.1 * math.sin(0.3 * tt)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy, dz = wx - cx, wy - cy, wz - SENSOR_Z_M
        xyz.extend((c * dx + s * dy, -s * dx + c * dy, dz))
        offs.append(int(dt * 1e9))
    body = struct.pack("<QI", t_ns, n) + xyz.tobytes() + offs.tobytes()
    return struct.pack("<I", len(body)) + body


def proc_sample(pid: int) -> tuple[int, int]:
    """(resident kB, utime + stime ticks) of `pid`."""
    rss = 0
    with open(f"/proc/{pid}/status") as fh:
        for line in fh:
            if line.startswith("VmRSS:"):
                rss = int(line.split()[1])
    with open(f"/proc/{pid}/stat") as fh:
        fields = fh.read().rsplit(")", 1)[1].split()
    return rss, int(fields[11]) + int(fields[12])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--python", required=True, help="the runtime's interpreter")
    ap.add_argument("--project", required=True, help="the kiss_icp_live project directory (the child's cwd)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=10.0, help="scans per second")
    ap.add_argument("--points", type=int, default=20_000, help="points per scan")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    world = scene(args.seed)
    rng = random.Random(args.seed + 1)
    work = Path(tempfile.mkdtemp(prefix="kiss-stream-"))
    sock_path, config = work / "kiss.sock", work / "kiss.json"
    config.write_text(json.dumps({
        "data": {"max_range": 40.0, "min_range": 0.5, "deskew": True},
        "mapping": {"voxel_size": 0.4, "max_points_per_voxel": 20},
        "live": {"normal_k": 8, "planarity": 0.3, "min_points": 50},
    }))
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(sock_path))
    listener.listen(1)
    env = {"PATH": "/usr/bin:/bin", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}
    child = subprocess.Popen(
        [args.python, "-s", "-E", "-B", "-m", "kiss_icp_live", "--socket", str(sock_path), "--config", str(config)],
        cwd=args.project, env=env, stdin=subprocess.DEVNULL,
    )
    listener.settimeout(30)
    conn, _ = listener.accept()

    sent: dict[int, float] = {}
    lock = threading.Lock()
    latencies: list[float] = []
    bad = 0

    def reader() -> None:
        nonlocal bad
        for line in conn.makefile("rb"):
            msg = json.loads(line)
            if not all(math.isfinite(v) for v in msg["T_w_l"]):
                bad += 1
            with lock:
                key = min(sent, key=lambda k: abs(k - msg["t_ns"]), default=None)
                if key is not None:
                    latencies.append(time.monotonic() - sent.pop(key))

    rd = threading.Thread(target=reader, daemon=True)
    rd.start()
    rss: list[int] = []
    _, ticks0 = proc_sample(child.pid)
    wall0 = time.monotonic()
    n_scans = int(args.seconds * args.rate)
    for k in range(n_scans):
        due = wall0 + k / args.rate
        time.sleep(max(0.0, due - time.monotonic()))
        t = k / args.rate
        t_ns = int(t * 1e9)
        frame = scan_frame(world, t, t_ns, args.points, rng)
        with lock:
            sent[t_ns + int(SCAN_S * 1e9 * (args.points - 1) / args.points)] = time.monotonic()
        conn.sendall(frame)
        rss.append(proc_sample(child.pid)[0])
    time.sleep(1.0)
    _, ticks1 = proc_sample(child.pid)
    wall = time.monotonic() - wall0
    conn.shutdown(socket.SHUT_WR)
    child.wait(timeout=30)
    rd.join(timeout=5)

    answered = len(latencies)
    cpu = 100.0 * (ticks1 - ticks0) / CLK_TCK / wall
    lat = sorted(latencies) or [float("nan")]
    print(
        f"scans {n_scans} answered {answered} non-finite {bad}; "
        f"latency p50 {1e3 * lat[len(lat) // 2]:.0f} ms p95 {1e3 * lat[int(0.95 * (len(lat) - 1))]:.0f} ms; "
        f"RSS median {statistics.median(rss) / 1024:.0f} MB peak {max(rss) / 1024:.0f} MB; CPU {cpu:.0f} %"
    )
    return 0 if answered >= 0.9 * n_scans and bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
