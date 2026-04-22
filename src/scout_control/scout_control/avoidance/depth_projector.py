"""Depth-frame projection helpers for local mapper / planner inputs."""

from __future__ import annotations

import math
import time

import numpy as np

from .types import PointBatch


class DepthProjector:
    """Project depth frames into body-local points and world XY points."""

    def __init__(
        self,
        *,
        camera_hfov_deg: float = 71.9,
        min_range_m: float = 0.3,
        max_range_m: float = 20.0,
        default_stride: int = 4,
        collision_band_m: tuple[float, float] = (-1.0, 1.0),
        ground_epsilon_m: float = 0.05,
    ) -> None:
        self._camera_hfov_rad = math.radians(camera_hfov_deg)
        self._min_range_m = float(min_range_m)
        self._max_range_m = float(max_range_m)
        self._default_stride = max(1, int(default_stride))
        self._collision_band_m = (
            float(collision_band_m[0]),
            float(collision_band_m[1]),
        )
        self._ground_epsilon_m = float(ground_epsilon_m)

    def depth_to_body_points(
        self,
        depth_frame: np.ndarray,
        *,
        pixel_stride: int | None = None,
        stamp_s: float | None = None,
        source: str = "depth_camera",
        is_dense_scan: bool = False,
    ) -> PointBatch:
        """Return body-frame points in forward-right-down coordinates."""

        depth = np.asarray(depth_frame, dtype=np.float32)
        if depth.ndim != 2:
            raise ValueError("depth_frame must be a 2D array")

        stride = self._default_stride if pixel_stride is None else max(1, int(pixel_stride))
        height, width = depth.shape
        fx = (width / 2.0) / math.tan(self._camera_hfov_rad / 2.0)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        rows = np.arange(0, height, stride, dtype=np.int32)
        cols = np.arange(0, width, stride, dtype=np.int32)
        uu, vv = np.meshgrid(cols, rows)
        d_sub = depth[::stride, ::stride]

        valid = (
            np.isfinite(d_sub)
            & (d_sub >= self._min_range_m)
            & (d_sub <= self._max_range_m)
        )
        if not np.any(valid):
            return PointBatch.empty(
                source=source,
                frame="body_frd",
                stamp_s=time.time() if stamp_s is None else float(stamp_s),
                sensor_range_m=self._max_range_m,
            )

        d_v = d_sub[valid]
        u_v = uu[valid].astype(np.float32)
        v_v = vv[valid].astype(np.float32)

        right_m = (u_v - cx) * d_v / fx
        down_m = (v_v - cy) * d_v / fy
        forward_m = d_v
        points = np.column_stack((forward_m, right_m, down_m)).astype(np.float32)
        return PointBatch(
            source=source,
            frame="body_frd",
            stamp_s=time.time() if stamp_s is None else float(stamp_s),
            points_xyz=points,
            sensor_range_m=self._max_range_m,
            is_dense_scan=bool(is_dense_scan),
        )

    def project_to_local_xy(
        self,
        depth_or_points: np.ndarray | PointBatch,
        *,
        pixel_stride: int | None = None,
        collision_band_m: tuple[float, float] | None = None,
    ) -> np.ndarray:
        """Project depth to local forward-right XY with vertical filtering."""

        body_batch = self._as_body_batch(depth_or_points, pixel_stride=pixel_stride)
        points = self._filter_collision_band(
            body_batch.points_xyz,
            collision_band_m=collision_band_m,
        )
        return points[:, :2].copy()

    def project_to_world_points(
        self,
        depth_or_points: np.ndarray | PointBatch,
        *,
        origin_ned: tuple[float, float, float],
        yaw_rad: float,
        ground_z_ned: float = 0.0,
        pixel_stride: int | None = None,
        collision_band_m: tuple[float, float] | None = None,
        source: str | None = None,
    ) -> PointBatch:
        """Project body-local points into world NED and remove ground clutter."""

        body_batch = self._as_body_batch(depth_or_points, pixel_stride=pixel_stride)
        points = body_batch.points_xyz
        if points.size == 0:
            return PointBatch.empty(
                source=source or body_batch.source,
                frame="world_ned",
                stamp_s=body_batch.stamp_s,
                sensor_range_m=body_batch.sensor_range_m,
            )

        forward = points[:, 0]
        right = points[:, 1]
        down = points[:, 2]

        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        world_x = float(origin_ned[0]) + forward * cos_yaw - right * sin_yaw
        world_y = float(origin_ned[1]) + forward * sin_yaw + right * cos_yaw
        world_z = float(origin_ned[2]) + down
        world_points = np.column_stack((world_x, world_y, world_z)).astype(np.float32)

        band = self._resolve_collision_band(collision_band_m)
        in_band = (down >= band[0]) & (down <= band[1])
        above_ground = world_z < (float(ground_z_ned) - self._ground_epsilon_m)
        keep = in_band | above_ground
        filtered = world_points[keep]

        return PointBatch(
            source=source or body_batch.source,
            frame="world_ned",
            stamp_s=body_batch.stamp_s,
            points_xyz=filtered,
            confidence=body_batch.confidence,
            sensor_range_m=body_batch.sensor_range_m,
            is_dense_scan=body_batch.is_dense_scan,
        )

    def project_to_world_xy(
        self,
        depth_or_points: np.ndarray | PointBatch,
        *,
        origin_ned: tuple[float, float, float],
        yaw_rad: float,
        ground_z_ned: float = 0.0,
        pixel_stride: int | None = None,
        collision_band_m: tuple[float, float] | None = None,
    ) -> np.ndarray:
        """Project depth to world XY, filtering invalid and ground-only points."""

        batch = self.project_to_world_points(
            depth_or_points,
            origin_ned=origin_ned,
            yaw_rad=yaw_rad,
            ground_z_ned=ground_z_ned,
            pixel_stride=pixel_stride,
            collision_band_m=collision_band_m,
        )
        return batch.xy.copy()

    def _as_body_batch(
        self,
        depth_or_points: np.ndarray | PointBatch,
        *,
        pixel_stride: int | None = None,
    ) -> PointBatch:
        if isinstance(depth_or_points, PointBatch):
            if depth_or_points.frame != "body_frd":
                raise ValueError("PointBatch frame must be 'body_frd'")
            return depth_or_points
        return self.depth_to_body_points(depth_or_points, pixel_stride=pixel_stride)

    def _filter_collision_band(
        self,
        points_xyz: np.ndarray,
        *,
        collision_band_m: tuple[float, float] | None = None,
    ) -> np.ndarray:
        if points_xyz.size == 0:
            return points_xyz.reshape(0, 3)
        band = self._resolve_collision_band(collision_band_m)
        keep = (points_xyz[:, 2] >= band[0]) & (points_xyz[:, 2] <= band[1])
        return points_xyz[keep]

    def _resolve_collision_band(
        self,
        collision_band_m: tuple[float, float] | None,
    ) -> tuple[float, float]:
        band = self._collision_band_m if collision_band_m is None else collision_band_m
        return float(band[0]), float(band[1])
