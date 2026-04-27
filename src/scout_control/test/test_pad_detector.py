# flake8: noqa
"""Tests for ArUco pad detection helper."""

import cv2
import numpy as np
import pytest

from scout_control.vision.pad_detector import CameraIntrinsics, detect_pad_marker


@pytest.mark.unit
def test_detects_synthetic_marker_when_aruco_available():
    if not hasattr(cv2, "aruco"):
        pytest.skip("cv2.aruco is unavailable")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 160)
    elif hasattr(cv2.aruco, "drawMarker"):
        marker = cv2.aruco.drawMarker(dictionary, 7, 160)
    else:
        pytest.skip("cv2.aruco marker generation is unavailable")
    frame = np.full((320, 320, 3), 255, dtype=np.uint8)
    frame[80:240, 80:240] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    detection = detect_pad_marker(
        frame,
        CameraIntrinsics(fx=500.0, fy=500.0, cx=160.0, cy=160.0),
        marker_size_m=0.35,
    )
    assert detection is not None
    assert detection.marker_id == 7
    assert detection.range_m > 0.0


@pytest.mark.unit
def test_returns_none_without_marker():
    frame = np.full((120, 120, 3), 255, dtype=np.uint8)
    assert detect_pad_marker(frame) is None
