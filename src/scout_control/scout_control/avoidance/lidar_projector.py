"""Lightweight LaserScan projection helpers for optional obstacle ingestion."""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from scout_control.avoidance.types import PointBatch


def _finite_range_limits(range_min_m: float, range_max_m: float) -> tuple[float, float]:
    min_m = max(0.0, float(range_min_m)) if math.isfinite(float(range_min_m)) else 0.0
    max_m = float(range_max_m) if math.isfinite(float(range_max_m)) else 0.0
    if max_m <= 0.0:
        max_m = math.inf
    return min_m, max_m


def laser_scan_to_body_points(
    *,
    ranges: Any,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float = 0.0,
    range_max_m: float = 0.0,
    stamp_s: float | None = None,
    source: str = "lidar",
    confidence: float = 1.0,
    stride: int = 1,
) -> PointBatch:
    """Convert a planar LaserScan into body FRD points on the collision plane.

    The convention matches the depth path's body frame: +x forward, +y right,
    +z down. A scan angle of zero points forward.
    """

    values = np.asarray(list(ranges or []), dtype=np.float32)
    stamp = time.time() if stamp_s is None else float(stamp_s)
    if values.size == 0:
        return PointBatch.empty(
            source=source,
            frame="body_frd",
            stamp_s=stamp,
            sensor_range_m=float(range_max_m or 0.0),
        )

    step = max(1, int(stride))
    if step > 1:
        values = values[::step]
        index = np.arange(0, values.size * step, step, dtype=np.float32)
    else:
        index = np.arange(values.size, dtype=np.float32)

    min_m, max_m = _finite_range_limits(range_min_m, range_max_m)
    valid = np.isfinite(values) & (values >= min_m) & (values <= max_m)
    if not np.any(valid):
        return PointBatch.empty(
            source=source,
            frame="body_frd",
            stamp_s=stamp,
            sensor_range_m=float(range_max_m or 0.0),
        )

    valid_ranges = values[valid]
    angles = float(angle_min_rad) + index[valid] * float(angle_increment_rad)
    forward = valid_ranges * np.cos(angles)
    right = valid_ranges * np.sin(angles)
    down = np.zeros_like(forward, dtype=np.float32)
    points = np.column_stack((forward, right, down)).astype(np.float32)
    return PointBatch(
        source=source,
        frame="body_frd",
        stamp_s=stamp,
        points_xyz=points,
        confidence=float(confidence),
        sensor_range_m=float(range_max_m or 0.0),
        is_dense_scan=False,
    )


def body_to_world_points(
    batch: PointBatch,
    *,
    origin_ned: tuple[float, float, float],
    yaw_rad: float,
    source: str | None = None,
) -> PointBatch:
    """Project body FRD points into world NED using the current local pose."""

    if batch.frame != "body_frd":
        raise ValueError("PointBatch frame must be 'body_frd'")
    points = np.asarray(batch.points_xyz, dtype=np.float32).reshape((-1, 3))
    if points.size == 0:
        return PointBatch.empty(
            source=source or batch.source,
            frame="world_ned",
            stamp_s=batch.stamp_s,
            sensor_range_m=batch.sensor_range_m,
        )

    forward = points[:, 0]
    right = points[:, 1]
    down = points[:, 2]
    cos_yaw = math.cos(float(yaw_rad))
    sin_yaw = math.sin(float(yaw_rad))
    world_x = float(origin_ned[0]) + forward * cos_yaw - right * sin_yaw
    world_y = float(origin_ned[1]) + forward * sin_yaw + right * cos_yaw
    world_z = float(origin_ned[2]) + down
    return PointBatch(
        source=source or batch.source,
        frame="world_ned",
        stamp_s=batch.stamp_s,
        points_xyz=np.column_stack((world_x, world_y, world_z)).astype(np.float32),
        confidence=batch.confidence,
        sensor_range_m=batch.sensor_range_m,
        is_dense_scan=batch.is_dense_scan,
    )
