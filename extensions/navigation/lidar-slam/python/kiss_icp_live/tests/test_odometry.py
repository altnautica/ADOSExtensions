import io
import json
import socket
import threading

import numpy as np
import pytest

from kiss_icp_live import serve
from kiss_icp_live.framing import FramingError, encode_scan, read_scan
from kiss_icp_live.odometry import LiveConfig, LiveOdometry, load_config

SCAN_NS = 100_000_000


def plane(rng, n, origin, u, v):
    """n points uniform on the rectangle origin + a*u + b*v, a, b in [0, 1]."""
    a, b = rng.random((2, n, 1))
    return np.asarray(origin) + a * np.asarray(u) + b * np.asarray(v)


def box_room(rng, n=12_000):
    # 10 x 8 x 3 m room, corner at (-5, -4, -1).
    x0, y0, z0, lx, ly, lz = -5.0, -4.0, -1.0, 10.0, 8.0, 3.0
    faces = [
        ((x0, y0, z0), (lx, 0, 0), (0, ly, 0)),
        ((x0, y0, z0 + lz), (lx, 0, 0), (0, ly, 0)),
        ((x0, y0, z0), (lx, 0, 0), (0, 0, lz)),
        ((x0, y0 + ly, z0), (lx, 0, 0), (0, 0, lz)),
        ((x0, y0, z0), (0, ly, 0), (0, 0, lz)),
        ((x0 + lx, y0, z0), (0, ly, 0), (0, 0, lz)),
    ]
    return np.vstack([plane(rng, n // 6, *f) for f in faces])


def corridor(rng, n=12_000):
    # 2 m wide, 2.5 m tall, 200 m long along x; the ends are out of range.
    faces = [
        ((-100, -1, -1), (200, 0, 0), (0, 2, 0)),
        ((-100, -1, 1.5), (200, 0, 0), (0, 2, 0)),
        ((-100, -1, -1), (200, 0, 0), (0, 0, 2.5)),
        ((-100, 1, -1), (200, 0, 0), (0, 0, 2.5)),
    ]
    world = np.vstack([plane(rng, n // 4, *f) for f in faces])
    # Only what a 20 m sensor sees around the origin region.
    return world[np.abs(world[:, 0]) < 25.0]


def run(scene, steps=6, speed=0.1):
    rng = np.random.default_rng(7)
    cfg, live = load_config(None)
    cfg.data.max_range = 20.0
    cfg.data.min_range = 0.3
    cfg.mapping.voxel_size = 0.25
    odom = LiveOdometry(cfg, live)
    results = []
    for i in range(steps):
        world = scene(rng)
        pos = np.array([speed * i, 0.0, 0.0])
        pts = world - pos
        pts = pts[np.linalg.norm(pts, axis=1) < cfg.data.max_range]
        offs = np.sort(rng.integers(0, SCAN_NS, len(pts))).astype(np.uint32)
        results.append(odom.process(i * SCAN_NS, pts, offs))
    return results


def ratio(res):
    return res.eig[0] / res.eig[2]


def test_box_room_is_well_conditioned_and_tracks():
    res = run(box_room)
    assert res[0].n == 0 and not res[0].eig.any(), "first scan has no map to match"
    last = res[-1]
    assert last.n > 500
    assert ratio(last) > 0.1, last.eig
    # Static scene, sensor moving +x 0.1 m per scan.
    assert abs(last.pose[0, 3] - 0.5) < 0.05, last.pose
    assert np.allclose(last.pose[:3, :3], np.eye(3), atol=0.02)
    assert last.t_ns >= 5 * SCAN_NS


def test_corridor_is_degenerate_along_its_axis():
    res = run(corridor)
    last = res[-1]
    assert last.n > 500
    assert ratio(last) < 0.05, last.eig
    assert ratio(run(box_room)[-1]) > 5 * ratio(last)


def test_zero_span_times_and_tiny_scans():
    cfg, live = load_config(None)
    cfg.data.max_range = 20.0
    cfg.mapping.voxel_size = 0.25
    odom = LiveOdometry(cfg, LiveConfig(min_points=50))
    rng = np.random.default_rng(1)
    pts = box_room(rng)
    assert odom.process(0, pts, np.zeros(len(pts), np.uint32)) is not None
    assert odom.process(1, pts[:10], np.zeros(10, np.uint32)) is None


def test_framing_round_trip_and_errors():
    pts = np.array([[1.0, 2.0, 3.0], [-1.5, 0.25, 0.0]])
    offs = np.array([0, 99_000_000], dtype=np.uint32)
    frame = encode_scan(123, pts, offs)
    assert len(frame) == 4 + 12 + 16 * 2
    scan = read_scan(io.BytesIO(frame))
    assert scan.t_ns == 123
    assert np.allclose(scan.points, pts)
    assert scan.t_offset_ns.tolist() == [0, 99_000_000]
    assert read_scan(io.BytesIO(b"")) is None
    with pytest.raises(FramingError):
        read_scan(io.BytesIO(frame[:-3]))
    bad = bytearray(frame)
    bad[0] += 16  # length no longer matches n
    with pytest.raises(FramingError):
        read_scan(io.BytesIO(bytes(bad) + b"\0" * 16))


def test_serve_answers_each_scan_over_a_socket(tmp_path):
    cfg_path = tmp_path / "cfg.json"
    cfg_path.write_text(json.dumps({"data": {"max_range": 20.0}, "mapping": {"voxel_size": 0.25}, "live": {"normal_k": 6}}))
    cfg, live = load_config(cfg_path)
    assert cfg.mapping.voxel_size == 0.25 and live.normal_k == 6
    a, b = socket.socketpair()
    rng = np.random.default_rng(3)
    frames = [
        encode_scan(i * SCAN_NS, pts, np.linspace(0, SCAN_NS - 1, len(pts)).astype(np.uint32))
        for i, pts in enumerate(box_room(rng) for _ in range(3))
    ]

    def send():
        # ~0.2 MB per scan: more than a socket buffer, so the writer must not
        # wait for the reader on the same thread.
        for f in frames:
            a.sendall(f)
        a.shutdown(socket.SHUT_WR)

    with a, b:
        sender = threading.Thread(target=send)
        sender.start()
        with b.makefile("rb") as r, b.makefile("wb") as w:
            assert serve(r, w, LiveOdometry(cfg, live)) == 3
        sender.join(timeout=10)
        b.shutdown(socket.SHUT_WR)
        lines = a.makefile("rb").read().decode().splitlines()
    assert len(lines) == 3
    first = json.loads(lines[0])
    assert set(first) == {"t_ns", "T_w_l", "eig", "n"}
    assert first["t_ns"] == SCAN_NS - 1
    assert len(first["T_w_l"]) == 16 and len(first["eig"]) == 3
