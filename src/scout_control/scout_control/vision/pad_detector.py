# flake8: noqa
"""Home-pad marker detection helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(slots=True)
class CameraIntrinsics:
    fx: float = 600.0
    fy: float = 600.0
    cx: float = 320.0
    cy: float = 240.0


@dataclass(slots=True)
class PadDetection:
    marker_id: int
    offset_xy_body_m: tuple[float, float]
    range_m: float
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "marker_id": int(self.marker_id),
            "offset_xy_body_m": [self.offset_xy_body_m[0], self.offset_xy_body_m[1]],
            "range_m": float(self.range_m),
            "confidence": float(self.confidence),
        }


def detect_pad_marker(
    bgr_frame: np.ndarray,
    camera_info: CameraIntrinsics | None = None,
    *,
    marker_size_m: float = 0.35,
) -> PadDetection | None:
    """Detect a 4x4 ArUco home-pad marker and estimate body-frame offset."""

    if marker_size_m <= 0.0:
        raise ValueError("marker_size_m must be positive")
    frame = np.asarray(bgr_frame)
    if frame.ndim not in {2, 3}:
        raise ValueError("bgr_frame must be grayscale or BGR image")
    if not hasattr(cv2, "aruco"):
        return None

    intr = camera_info or CameraIntrinsics(
        fx=max(float(frame.shape[1]), 1.0),
        fy=max(float(frame.shape[1]), 1.0),
        cx=float(frame.shape[1]) / 2.0,
        cy=float(frame.shape[0]) / 2.0,
    )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) == 0:
        return None

    camera_matrix = np.array(
        [[intr.fx, 0.0, intr.cx], [0.0, intr.fy, intr.cy], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    dist = np.zeros((5, 1), dtype=np.float32)
    half = marker_size_m / 2.0
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )
    best_idx = max(range(len(corners)), key=lambda idx: cv2.contourArea(corners[idx]))
    image_points = np.asarray(corners[best_idx], dtype=np.float32).reshape(4, 2)
    ok, _rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None
    tx, ty, tz = (float(v) for v in tvec.reshape(3))
    rng = math.sqrt(tx * tx + ty * ty + tz * tz)
    area = max(cv2.contourArea(image_points), 1.0)
    frame_area = max(float(gray.shape[0] * gray.shape[1]), 1.0)
    confidence = min(1.0, 0.35 + (area / frame_area) * 25.0)
    return PadDetection(
        marker_id=int(ids[best_idx][0]),
        offset_xy_body_m=(tx, ty),
        range_m=rng,
        confidence=confidence,
    )

