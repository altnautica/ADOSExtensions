# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
# SPDX-License-Identifier: BSD-3-Clause
"""Synthetic end-to-end check of basalt-ados.

Renders a stereo rig looking at a textured wall, with a matching IMU stream,
feeds both through the basalt-ados socket protocol in real time, and checks
that the binary reaches `tracking` and that its trajectory follows the ground
truth.

Scene: world = gravity-aligned, z up; body = FLU, IMU = body; the rig starts
at the origin facing +x toward a wall at x = 3 m, accelerates smoothly along
+y (a half-cosine ramp over 2 s) to 0.3 m/s, then holds that velocity, with a
slow yaw sway of a few degrees. Cameras: 640x480 pinhole, fx = fy = 400,
baseline 0.10 m, 20 Hz. IMU: 200 Hz, noise-free.

    uv run synth_check.py <path/to/basalt-ados> [--seconds 12]
"""

import argparse
import json
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

W, H, F = 640, 480, 400.0
CX, CY = W / 2 - 0.5, H / 2 - 0.5
BASELINE = 0.10
WALL_X = 3.0
CAM_HZ, IMU_HZ = 20, 200
V_MAX, RAMP = 0.3, 2.0
YAW_AMP, YAW_W = 0.05, 0.8
G = 9.81
T0_NS = 1_000_000_000

# Body FLU -> camera OpenCV.
R_BC = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CAM_Y = [BASELINE / 2, -BASELINE / 2]  # left, right


def position(t):
    if t < RAMP:
        y = V_MAX * (t / 2 - RAMP / (2 * math.pi) * math.sin(math.pi * t / RAMP))
    else:
        y = V_MAX * (RAMP / 2 + (t - RAMP))
    return np.array([0.0, y, 0.0])


def accel_world(t):
    a = V_MAX * math.pi / (2 * RAMP) * math.sin(math.pi * t / RAMP) if t < RAMP else 0.0
    return np.array([0.0, a, 0.0])


def yaw(t):
    return YAW_AMP * (1 - math.cos(YAW_W * t))


def yaw_rate(t):
    return YAW_AMP * YAW_W * math.sin(YAW_W * t)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def texture(y, z):
    """Random grey blocks at two scales: strong corners for FAST + flow."""

    def blocks(size, seed):
        i = np.floor(y / size).astype(np.int64)
        j = np.floor(z / size).astype(np.int64)
        h = (i * 73856093) ^ (j * 19349663) ^ seed
        h = (h ^ (h >> 13)) * 1274126177
        return ((h >> 8) & 0xFF).astype(np.float64)

    return 0.65 * blocks(0.12, 11) + 0.35 * blocks(0.035, 97)


PIX = np.stack(
    np.meshgrid((np.arange(W) - CX) / F, (np.arange(H) - CY) / F), axis=-1
).reshape(-1, 2)
RAYS_CAM = np.concatenate([PIX, np.ones((PIX.shape[0], 1))], axis=1)


def render(t, cam):
    r_wb = rot_z(yaw(t))
    c = position(t) + r_wb @ np.array([0.0, CAM_Y[cam], 0.0])
    d = RAYS_CAM @ (r_wb @ R_BC).T
    s = (WALL_X - c[0]) / d[:, 0]
    hit = c[None, :] + s[:, None] * d
    img = texture(hit[:, 1], hit[:, 2])
    return np.clip(img, 0, 255).astype(np.uint8).reshape(H, W)


def imu_sample(t):
    r_wb = rot_z(yaw(t))
    f = r_wb.T @ (accel_world(t) + np.array([0.0, 0.0, G]))
    return np.array([0.0, 0.0, yaw_rate(t)]), f


def calib():
    def se3(y):
        # T_imu_cam: rotation R_BC, translation (0, y, 0). Quaternion of R_BC.
        return {"px": 0.0, "py": y, "pz": 0.0, "qx": -0.5, "qy": 0.5, "qz": -0.5, "qw": 0.5}

    intr = {"camera_type": "pinhole", "intrinsics": {"fx": F, "fy": F, "cx": CX, "cy": CY}}
    v3 = lambda x: [x, x, x]  # noqa: E731
    return {"value0": {
        "T_imu_cam": [se3(CAM_Y[0]), se3(CAM_Y[1])],
        "intrinsics": [intr, intr],
        "resolution": [[W, H], [W, H]],
        "calib_accel_bias": [0.0] * 9,
        "calib_gyro_bias": [0.0] * 12,
        "imu_update_rate": float(IMU_HZ),
        "accel_noise_std": v3(6.9e-3),
        "gyro_noise_std": v3(4.9e-4),
        "accel_bias_std": v3(2.0e-3),
        "gyro_bias_std": v3(4.0e-5),
        "cam_time_offset_ns": 0,
        "vignette": [],
    }}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binary")
    ap.add_argument("--config", help="Basalt VIO config JSON (default: alongside)")
    ap.add_argument("--seconds", type=float, default=12.0)
    # The live capture streams IMU from the hub well before the first stereo
    # pair (camera start-up, pairing). A lead-in longer than Basalt's stock
    # 0.3 s IMU queue deadlocked the reader before the queue fix.
    ap.add_argument("--imu-lead-s", type=float, default=2.0,
                    help="seconds of IMU sent before the first stereo pair")
    # A live frame arrives after the IMU of its exposure (seconds on the
    # simulator bench), so the first pair is older than the newest IMU.
    ap.add_argument("--frame-lag-s", type=float, default=0.0,
                    help="send each stereo pair this long after its exposure")
    # Host-mapped IMU stamps can step back by the clock mapping's jitter.
    ap.add_argument("--imu-stepback-every", type=int, default=0,
                    help="every Nth IMU sample, also send one stamped half a "
                         "period before the previous sample (0 = never)")
    # A stalled estimator (no further frames) must not stop the reader: past
    # the IMU queue capacity the oldest samples are evicted, never blocked on.
    ap.add_argument("--imu-burst", type=int, default=0,
                    help="after the run, send this many IMU samples at once "
                         "(above the 20000-sample queue to test eviction)")
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    config = args.config or os.path.join(here, "vio_config.json")

    work = tempfile.mkdtemp(prefix="basalt-synth-")
    calib_path = os.path.join(work, "calib.json")
    with open(calib_path, "w") as f:
        json.dump(calib(), f)
    sock_path = os.path.join(work, "in.sock")
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    child = subprocess.Popen([args.binary, sock_path, calib_path, config],
                             stdout=subprocess.PIPE, text=True)
    srv.settimeout(30)
    conn, _ = srv.accept()
    conn.sendall(struct.pack("<BII", 0, 4, 1))

    states, statuses, bad_lines = [], [], []

    def read_out():
        for line in child.stdout:
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print("FAIL: non-JSON stdout line:", line.rstrip(), flush=True)
                bad_lines.append(line)
                continue
            (statuses if "status" in msg else states).append(msg)
            if "status" in msg:
                print("status:", msg["status"], flush=True)

    reader = threading.Thread(target=read_out, daemon=True)
    reader.start()

    n_imu = int(args.seconds * IMU_HZ)
    lag_imu = round(args.frame_lag_s * IMU_HZ)
    start = time.monotonic()
    frames = 0
    for k in range(n_imu + lag_imu + 1):
        t = k / IMU_HZ
        lag = start + t - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        if k <= n_imu:
            g, a = imu_sample(t)
            t_ns = T0_NS + round(t * 1e9)
            conn.sendall(struct.pack("<BIQ6d", 1, 56, t_ns, *g, *a))
            if args.imu_stepback_every and k and k % args.imu_stepback_every == 0:
                back = t_ns - round(1.5e9 / IMU_HZ)
                conn.sendall(struct.pack("<BIQ6d", 1, 56, back, *g, *a))
        # The pair exposed lag_imu samples ago.
        e = k - lag_imu
        te = e / IMU_HZ
        if e > 0 and e % (IMU_HZ // CAM_HZ) == 0 and te >= args.imu_lead_s:
            left, right = render(te, 0), render(te, 1)
            te_ns = T0_NS + round(te * 1e9)
            conn.sendall(struct.pack("<BIQII", 2, 16 + 2 * W * H, te_ns, W, H)
                         + left.tobytes() + right.tobytes())
            frames += 1
    burst_s = None
    if args.imu_burst:
        conn.settimeout(10)
        t_end = (n_imu + 1) / IMU_HZ
        g, a = imu_sample(t_end)
        burst = b"".join(
            struct.pack("<BIQ6d", 1, 56, T0_NS + round((t_end + i / IMU_HZ) * 1e9), *g, *a)
            for i in range(args.imu_burst))
        t_burst = time.monotonic()
        try:
            conn.sendall(burst)
            burst_s = time.monotonic() - t_burst
        except TimeoutError:
            burst_s = float("inf")
    conn.close()
    child.wait(timeout=30)
    reader.join(timeout=5)

    print(f"frames sent {frames}, states {len(states)}, exit {child.returncode}")
    ok = True
    if burst_s is not None:
        print(f"IMU burst of {args.imu_burst} sent in {burst_s:.2f} s")
        if burst_s > 5.0:
            print("FAIL: the reader blocked on a full IMU queue")
            ok = False
    if bad_lines:
        ok = False
    if not any(s["status"] == "tracking" for s in statuses):
        print("FAIL: never reached tracking")
        ok = False
    if len(states) < 0.7 * frames:
        print("FAIL: fewer than 70% of frames produced a state")
        ok = False
    if states:
        est = np.array([[s["T_w_i"][3], s["T_w_i"][7], s["T_w_i"][11]] for s in states])
        gt = np.array([position((s["t_ns"] - T0_NS) / 1e9) for s in states])
        # Basalt's origin is where it initialised (the first stereo pair, after
        # any IMU lead-in), so ground truth is taken relative to that point.
        gt -= position((states[0]["t_ns"] - T0_NS) / 1e9)
        # Basalt's yaw is arbitrary: align about z (2-D Procrustes), origin fixed.
        h = est[:, :2].T @ gt[:, :2]
        u, _, vt = np.linalg.svd(h)
        r2 = (u @ vt).T
        if np.linalg.det(r2) < 0:
            vt[1] *= -1
            r2 = (u @ vt).T
        aligned = est.copy()
        aligned[:, :2] = est[:, :2] @ r2.T
        err = np.linalg.norm(aligned - gt, axis=1)
        travel = np.linalg.norm(gt[-1] - gt[0])
        feats = [s["features"] for s in states if s["features"] is not None]
        print(f"travel {travel:.2f} m, final error {err[-1]:.3f} m, max error {err.max():.3f} m, "
              f"features median {int(np.median(feats)) if feats else 0}")
        if err[-1] > 0.1 * max(travel, 1.0):
            print("FAIL: final position error above 10% of travel")
            ok = False
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
