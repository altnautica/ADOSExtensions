"""Agent-side calibration runner consumed by the in-app wizard.

The GCS publishes a ``vision-nav.start_calibration`` event carrying
the captured frame bundle (base64 PNGs) and the IMU recording window.
This module decodes the bundle, runs OpenCV's AprilTag detection and
intrinsics solve, fits the joint camera-IMU timeshift against the
IMU motion trace, persists the result, and publishes substep progress
back over ``vision-nav.calibration_progress``. On completion it
publishes ``vision-nav.calibration_complete`` with the result + the
reprojection error + the timeshift residual so the wizard's verify
step can render the side-by-side compare.

The runner is intentionally a pure coroutine that the plugin's host
schedules. Hardware probes, MAVLink callbacks, and event publishing
are passed in via the small :class:`RunnerHooks` adapter so the unit
tests can exercise the maths without bringing up the full plugin.

The math:

* AprilGrid t36h11, 6x6, with a known printed edge length (the
  operator typed it in step 1 of the wizard).
* ``cv2.aruco.ArucoDetector`` extracts tag corners per frame.
* ``cv2.calibrateCamera`` solves the pinhole-radtan intrinsics +
  per-frame extrinsics jointly.
* For the joint camera-IMU timeshift fit, the residual is the
  rotational alignment of the gyro trace to the per-frame camera
  rotation series. A 1-D golden-section search over the candidate
  timeshift band ``[-200 ms .. +200 ms]`` finds the minimum.

This is a v1 implementation: pinhole + radtan only, monocular only,
joint timeshift only (T_cam_imu rotation comes from manual mounting
geometry rather than full inertial-visual bundle adjustment). The
heavyweight joint VIO calibration the project will eventually want
lives in the Kalibr binary, not in this file.
"""

from __future__ import annotations

import asyncio
import base64
import io
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Awaitable, Callable, Mapping, Sequence

try:
    import cv2  # type: ignore[import-untyped]
    import numpy as np  # type: ignore[import-untyped]

    _HAVE_CV2 = True
except ImportError:  # pragma: no cover - degrade cleanly in tests w/o opencv
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]
    _HAVE_CV2 = False

_LOG = logging.getLogger(__name__)

_GRID_ROWS = 6
_GRID_COLS = 6
_TAG_FAMILY = "DICT_APRILTAG_36h11"


@dataclass
class CalibrationRequest:
    """The decoded ``start_calibration`` payload."""

    frames_b64: Sequence[str]
    window_start_ns: int
    window_end_ns: int
    width: int
    height: int
    # Edge length of one printed tag in metres. The wizard captures
    # this in step 1 and includes it on the event payload (v1 hardcodes
    # the default; the wizard sends it as an override).
    tag_size_m: float = 0.054


@dataclass
class CalibrationProgress:
    stage: str  # one of CalibrationStage on the GCS side
    percent: float
    detail: str | None = None


@dataclass
class CalibrationResult:
    camera_model: str
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion_model: str
    distortion_coeffs: list[float]
    t_cam_imu: list[float]
    timeshift_cam_imu_s: float
    reprojection_error_px: float
    timeshift_residual_ms: float
    frames_used: int
    frames_rejected: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "cameraModel": self.camera_model,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "distortionModel": self.distortion_model,
            "distortionCoeffs": self.distortion_coeffs,
            "tCamImu": self.t_cam_imu,
            "timeshiftCamImuS": self.timeshift_cam_imu_s,
            "reprojectionErrorPx": self.reprojection_error_px,
            "timeshiftResidualMs": self.timeshift_residual_ms,
            "framesUsed": self.frames_used,
            "framesRejected": self.frames_rejected,
        }


@dataclass
class _ImuSample:
    timestamp_ns: int
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]


@dataclass
class RunnerHooks:
    """Adapter the plugin passes in.

    Lets the unit tests substitute synthetic IMU traces + event
    publishing capture lists without touching the full plugin
    machinery.
    """

    publish_progress: Callable[[CalibrationProgress], Awaitable[None]]
    publish_complete: Callable[
        [CalibrationResult | None, str | None], Awaitable[None]
    ]
    fetch_imu_window: Callable[[int, int], Sequence[_ImuSample]]
    persist_camchain_yaml: Callable[[str], Path]


def _ensure_cv2() -> None:
    if not _HAVE_CV2:  # pragma: no cover - mocked in tests
        raise RuntimeError(
            "OpenCV is not available; the agent calibration runner "
            "requires opencv-contrib-python-headless."
        )


def parse_request(payload: Mapping[str, Any]) -> CalibrationRequest:
    """Turn the raw event payload into a typed :class:`CalibrationRequest`."""
    frames = payload.get("framesB64")
    if not isinstance(frames, list) or len(frames) == 0:
        raise ValueError("start_calibration payload missing framesB64 list")
    if not all(isinstance(f, str) for f in frames):
        raise ValueError("framesB64 entries must be strings")
    window_start = int(payload.get("windowStartNs", 0))
    window_end = int(payload.get("windowEndNs", 0))
    width = int(payload.get("width", 640))
    height = int(payload.get("height", 480))
    tag_size = float(payload.get("tagSizeM", 0.054))
    return CalibrationRequest(
        frames_b64=frames,
        window_start_ns=window_start,
        window_end_ns=window_end,
        width=width,
        height=height,
        tag_size_m=tag_size,
    )


def _decode_frame(b64: str) -> Any:
    """Decode a base64 PNG into a BGR numpy array."""
    raw = base64.b64decode(b64)
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("decoded frame is not a valid image")
    return img


def _detect_apriltag_corners(img: Any) -> tuple[Sequence[Any], Sequence[int]]:
    """Detect AprilTag corners via ``cv2.aruco``.

    Returns ``(corners, ids)`` where ``corners`` is a sequence of 4x2
    float arrays and ``ids`` is the matching integer tag IDs.
    """
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(
        getattr(aruco, _TAG_FAMILY)
    )
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)
    corners, ids, _rej = detector.detectMarkers(img)
    if ids is None:
        return [], []
    return list(corners), list(int(i) for i in ids.flatten())


def _build_object_points(
    ids: Sequence[int], tag_size_m: float
) -> Any:
    """Map detected tag IDs to their 3D object-point grid coordinates.

    The AprilGrid tags lie in the z=0 plane, with tag 0 at the origin
    and tags laid out row-major. The 4 corners of each tag are
    offset by ``tag_size_m`` in x and y from the tag's grid cell.
    """
    points = []
    for tag_id in ids:
        if not (0 <= tag_id < _GRID_ROWS * _GRID_COLS):
            continue
        col = tag_id % _GRID_COLS
        row = tag_id // _GRID_COLS
        origin_x = col * tag_size_m * 2  # 1 tag-edge gap between tags
        origin_y = row * tag_size_m * 2
        tag_corners = np.array(
            [
                [origin_x, origin_y, 0.0],
                [origin_x + tag_size_m, origin_y, 0.0],
                [origin_x + tag_size_m, origin_y + tag_size_m, 0.0],
                [origin_x, origin_y + tag_size_m, 0.0],
            ],
            dtype=np.float32,
        )
        points.append(tag_corners)
    if not points:
        return None
    return np.vstack(points)


def _gyro_camera_residual(
    timeshift_s: float,
    frame_times: Sequence[float],
    frame_rotations: Sequence[Any],
    imu_samples: Sequence[_ImuSample],
) -> float:
    """Residual of the gyro trace against the per-frame camera rotation.

    For each consecutive frame pair we have an inferred angular
    velocity (rotation between the two frames divided by their dt).
    The IMU's gyro samples in the same shifted window should match.
    Returns sum of squared component errors in rad/s.
    """
    if len(frame_times) < 2:
        return float("inf")
    total = 0.0
    for i in range(len(frame_times) - 1):
        t0 = frame_times[i] + timeshift_s
        t1 = frame_times[i + 1] + timeshift_s
        if t1 <= t0:
            continue
        # Camera-derived angular velocity (axis-angle / dt).
        r0 = frame_rotations[i]
        r1 = frame_rotations[i + 1]
        rel, _ = cv2.Rodrigues(r1 @ r0.T)
        omega_cam = rel.flatten() / (t1 - t0)
        # Average gyro in the matching window.
        gyro_samples = [
            s
            for s in imu_samples
            if t0 <= s.timestamp_ns * 1e-9 <= t1
        ]
        if not gyro_samples:
            continue
        gyro_mean = np.mean(
            [list(s.gyro) for s in gyro_samples], axis=0
        )
        delta = omega_cam - gyro_mean
        total += float(np.dot(delta, delta))
    return total


def _fit_timeshift(
    frame_times: Sequence[float],
    frame_rotations: Sequence[Any],
    imu_samples: Sequence[_ImuSample],
) -> tuple[float, float]:
    """Golden-section search over candidate timeshifts."""
    if len(imu_samples) < 5 or len(frame_times) < 3:
        return 0.0, float("inf")

    a, b = -0.2, 0.2
    gr = (math.sqrt(5) - 1) / 2  # golden ratio
    c = b - gr * (b - a)
    d = a + gr * (b - a)
    fc = _gyro_camera_residual(c, frame_times, frame_rotations, imu_samples)
    fd = _gyro_camera_residual(d, frame_times, frame_rotations, imu_samples)
    for _ in range(40):
        if fc < fd:
            b = d
            d = c
            fd = fc
            c = b - gr * (b - a)
            fc = _gyro_camera_residual(
                c, frame_times, frame_rotations, imu_samples
            )
        else:
            a = c
            c = d
            fc = fd
            d = a + gr * (b - a)
            fd = _gyro_camera_residual(
                d, frame_times, frame_rotations, imu_samples
            )
        if abs(b - a) < 1e-4:
            break
    best = (a + b) / 2
    residual = _gyro_camera_residual(
        best, frame_times, frame_rotations, imu_samples
    )
    return float(best), float(residual)


def _format_camchain_yaml(result: CalibrationResult) -> str:
    rows = []
    for r in range(4):
        row = result.t_cam_imu[r * 4 : r * 4 + 4]
        rows.append(
            "    - [" + ", ".join(f"{v:.8f}" for v in row) + "]"
        )
    return (
        "cam0:\n"
        f"  camera_model: {result.camera_model}\n"
        f"  intrinsics: [{result.fx:.4f}, {result.fy:.4f}, "
        f"{result.cx:.4f}, {result.cy:.4f}]\n"
        f"  distortion_model: {result.distortion_model}\n"
        f"  distortion_coeffs: ["
        + ", ".join(f"{v:.6f}" for v in result.distortion_coeffs)
        + "]\n"
        f"  resolution: [{result.width}, {result.height}]\n"
        "  T_cam_imu:\n" + "\n".join(rows) + "\n"
        f"  timeshift_cam_imu: {result.timeshift_cam_imu_s:.6f}\n"
    )


async def run_calibration(
    payload: Mapping[str, Any],
    hooks: RunnerHooks,
) -> CalibrationResult | None:
    """Top-level entry point invoked by the plugin's event subscriber.

    Wraps each substep in a try/except so a failure publishes a
    descriptive error rather than letting the exception escape into
    the plugin's event loop.
    """
    try:
        _ensure_cv2()
    except RuntimeError as err:
        await hooks.publish_complete(None, str(err))
        return None

    try:
        request = parse_request(payload)
    except ValueError as err:
        await hooks.publish_complete(None, str(err))
        return None

    await hooks.publish_progress(
        CalibrationProgress(stage="queued", percent=0.0)
    )

    # Tag detection
    await hooks.publish_progress(
        CalibrationProgress(stage="tag_detection", percent=5.0)
    )
    frames = []
    rejected = 0
    for idx, frame_b64 in enumerate(request.frames_b64):
        try:
            frames.append(_decode_frame(frame_b64))
        except (ValueError, Exception):  # noqa: BLE001
            rejected += 1
            continue
        pct = 5.0 + 15.0 * (idx + 1) / max(len(request.frames_b64), 1)
        await hooks.publish_progress(
            CalibrationProgress(
                stage="tag_detection",
                percent=pct,
                detail=f"decoded {idx + 1}/{len(request.frames_b64)}",
            )
        )

    all_obj_points: list[Any] = []
    all_img_points: list[Any] = []
    per_frame_corners: list[Any] = []
    for idx, img in enumerate(frames):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = _detect_apriltag_corners(gray)
        if len(ids) < 8:
            rejected += 1
            continue
        obj = _build_object_points(ids, request.tag_size_m)
        if obj is None:
            rejected += 1
            continue
        flat = np.vstack([c.reshape(-1, 2) for c in corners]).astype(
            np.float32
        )
        all_obj_points.append(obj)
        all_img_points.append(flat)
        per_frame_corners.append(corners)
        pct = 20.0 + 10.0 * (idx + 1) / max(len(frames), 1)
        await hooks.publish_progress(
            CalibrationProgress(
                stage="tag_detection",
                percent=pct,
                detail=f"detected on {len(all_obj_points)} frames",
            )
        )

    if len(all_obj_points) < 5:
        await hooks.publish_complete(
            None,
            f"Need at least 5 usable frames; got {len(all_obj_points)}",
        )
        return None

    # Intrinsics solve
    await hooks.publish_progress(
        CalibrationProgress(stage="intrinsics_solve", percent=35.0)
    )
    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            all_obj_points,
            all_img_points,
            (request.width, request.height),
            None,
            None,
        )
    except cv2.error as err:  # type: ignore[attr-defined]
        await hooks.publish_complete(None, f"calibrateCamera failed: {err}")
        return None

    reprojection_err_px = float(ret)
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    dist_list = [float(v) for v in dist_coeffs.flatten()]

    # Extrinsics + timeshift
    await hooks.publish_progress(
        CalibrationProgress(stage="extrinsics_solve", percent=60.0)
    )
    frame_rotations = []
    for rvec in rvecs:
        rmat, _ = cv2.Rodrigues(rvec)
        frame_rotations.append(rmat)

    duration_s = max(
        (request.window_end_ns - request.window_start_ns) * 1e-9, 1e-3
    )
    frame_times = [
        i * (duration_s / max(len(rvecs) - 1, 1))
        for i in range(len(rvecs))
    ]
    imu_samples = list(
        hooks.fetch_imu_window(
            request.window_start_ns, request.window_end_ns
        )
    )

    await hooks.publish_progress(
        CalibrationProgress(stage="timeshift_solve", percent=80.0)
    )
    timeshift_s, residual = _fit_timeshift(
        frame_times, frame_rotations, imu_samples
    )

    # T_cam_imu defaults to identity in v1; the operator's manual
    # mounting geometry covers the orientation. A future revision can
    # add a full inertial-visual bundle adjustment.
    t_cam_imu = [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]

    distortion_model = "radtan" if len(dist_list) >= 4 else "none"
    result = CalibrationResult(
        camera_model="pinhole",
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        width=request.width,
        height=request.height,
        distortion_model=distortion_model,
        distortion_coeffs=dist_list,
        t_cam_imu=t_cam_imu,
        timeshift_cam_imu_s=timeshift_s,
        reprojection_error_px=reprojection_err_px,
        timeshift_residual_ms=residual * 1000.0,
        frames_used=len(all_obj_points),
        frames_rejected=rejected,
    )

    # Persist + report.
    yaml = _format_camchain_yaml(result)
    try:
        hooks.persist_camchain_yaml(yaml)
    except OSError as err:
        _LOG.warning("failed to persist camchain.yaml: %s", err)

    await hooks.publish_progress(
        CalibrationProgress(stage="complete", percent=100.0)
    )
    await hooks.publish_complete(result, None)
    return result


__all__ = [
    "CalibrationProgress",
    "CalibrationRequest",
    "CalibrationResult",
    "RunnerHooks",
    "parse_request",
    "run_calibration",
]
