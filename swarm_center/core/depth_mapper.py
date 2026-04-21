from __future__ import annotations

import io
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from PIL import Image

from core.field_manager import FieldGrid
from core.mavlink_manager import DroneTelemetry


DEFAULT_HFOV_DEG = 71.9
DEFAULT_MAP_RES_M = 0.5
GRID_BUFFER_RATIO = 0.1
PIXEL_STRIDE = 16
DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 30.0
MAP_DECAY_S = 20.0


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


class DepthMapper:
    """
    Accumulates a lightweight terrain height map from downward depth frames.

    Assumptions:
    - camera is nadir-facing (looking down),
    - camera frame follows optical convention (x right, y down, z forward),
    - camera is mounted near the vehicle origin, so translation offset is ignored.
    """

    def __init__(self, resolution_m: float = DEFAULT_MAP_RES_M) -> None:
        self._resolution_m = max(0.1, float(resolution_m))
        self._grid: Optional[FieldGrid] = None
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._sum_elev: Optional[np.ndarray] = None
        self._count: Optional[np.ndarray] = None
        self._last_t: Optional[np.ndarray] = None
        self._intrinsics: dict[str, CameraIntrinsics] = {}
        self._mapped_points = 0

    def set_grid(self, grid: FieldGrid) -> None:
        self._grid = grid
        field_w = max(grid.cell_size_m, grid.x_max - grid.x_min)
        field_h = max(grid.cell_size_m, grid.y_max - grid.y_min)

        self._origin_x = grid.x_min - field_w * GRID_BUFFER_RATIO
        self._origin_y = grid.y_min - field_h * GRID_BUFFER_RATIO

        map_w = field_w * (1.0 + 2 * GRID_BUFFER_RATIO)
        map_h = field_h * (1.0 + 2 * GRID_BUFFER_RATIO)
        rows = int(math.ceil(map_w / self._resolution_m)) + 1
        cols = int(math.ceil(map_h / self._resolution_m)) + 1

        self._sum_elev = np.zeros((rows, cols), dtype=np.float32)
        self._count = np.zeros((rows, cols), dtype=np.uint16)
        self._last_t = np.zeros((rows, cols), dtype=np.float32)
        self._mapped_points = 0

    def set_camera_info(self, drone_id: str, width: int, height: int, k: list[float]) -> None:
        if width <= 0 or height <= 0 or len(k) < 9:
            return
        fx = float(k[0]) if float(k[0]) > 1e-3 else 0.0
        fy = float(k[4]) if float(k[4]) > 1e-3 else 0.0
        cx = float(k[2])
        cy = float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        self._intrinsics[drone_id] = CameraIntrinsics(
            width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy
        )

    def ingest_depth_frame(
        self,
        drone_id: str,
        png_bytes: bytes,
        width: int,
        height: int,
        encoding: str,
        telem: Optional[DroneTelemetry],
    ) -> None:
        if telem is None or not telem.connected or self._sum_elev is None:
            return
        if not png_bytes or width <= 0 or height <= 0:
            return

        depth_m = self._decode_depth_png(png_bytes, encoding)
        if depth_m is None or depth_m.size == 0:
            return

        intr = self._intrinsics.get(drone_id)
        if intr is None or intr.width != width or intr.height != height:
            intr = self._fallback_intrinsics(width, height)

        self._accumulate_depth(depth_m, intr, telem)

    def mapped_cells_count(self) -> int:
        if self._count is None:
            return 0
        return int(np.count_nonzero(self._count))

    def mapped_points_count(self) -> int:
        return int(self._mapped_points)

    def iter_surface_cells(self, max_cells: int = 12000) -> list[tuple[float, float, float, float]]:
        if self._count is None or self._sum_elev is None or self._last_t is None:
            return []

        self._decay()
        valid = self._count > 0
        idx = np.argwhere(valid)
        if idx.size == 0:
            return []

        if len(idx) > max_cells:
            step = int(math.ceil(len(idx) / max_cells))
            idx = idx[::step]

        out: list[tuple[float, float, float, float]] = []
        for gx, gy in idx:
            elev = float(self._sum_elev[gx, gy] / max(1, int(self._count[gx, gy])))
            x = self._origin_x + (float(gx) + 0.5) * self._resolution_m
            y = self._origin_y + (float(gy) + 0.5) * self._resolution_m
            age_s = float(time.time() - float(self._last_t[gx, gy]))
            out.append((x, y, elev, age_s))
        return out

    def resolution_m(self) -> float:
        return self._resolution_m

    def _decode_depth_png(self, png_bytes: bytes, encoding: str) -> Optional[np.ndarray]:
        try:
            arr = np.array(Image.open(io.BytesIO(png_bytes)))
        except Exception:
            return None
        if arr.ndim != 2:
            return None

        if arr.dtype == np.uint16:
            return arr.astype(np.float32) / 1000.0
        if arr.dtype == np.int32:
            return arr.astype(np.float32) / 1000.0
        if encoding == "32FC1":
            return arr.astype(np.float32) / 1000.0
        return arr.astype(np.float32)

    def _fallback_intrinsics(self, width: int, height: int) -> CameraIntrinsics:
        hfov = math.radians(DEFAULT_HFOV_DEG)
        fx = (width / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        return CameraIntrinsics(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=width / 2.0,
            cy=height / 2.0,
        )

    def _accumulate_depth(self, depth_m: np.ndarray, intr: CameraIntrinsics, telem: DroneTelemetry) -> None:
        rows = np.arange(0, depth_m.shape[0], PIXEL_STRIDE)
        cols = np.arange(0, depth_m.shape[1], PIXEL_STRIDE)
        uu, vv = np.meshgrid(cols, rows)
        d_sub = depth_m[::PIXEL_STRIDE, ::PIXEL_STRIDE]

        valid = (
            np.isfinite(d_sub)
            & (d_sub >= DEPTH_MIN_M)
            & (d_sub <= DEPTH_MAX_M)
        )
        if not np.any(valid):
            return

        d = d_sub[valid].astype(np.float32)
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)

        x_cam = (u - intr.cx) * d / intr.fx      # right
        y_cam = (v - intr.cy) * d / intr.fy      # down in image
        z_cam = d                                # optical axis, points downward

        # Camera optical -> body FRD for nadir camera:
        # optical x(right) -> body y(right)
        # optical y(down in image) -> body -x(forward)
        # optical z(forward) -> body z(down)
        body = np.stack((-y_cam, x_cam, z_cam), axis=0)
        world = _body_to_ned(body, telem.roll, telem.pitch, telem.yaw)

        wx = telem.x_ned + world[0]
        wy = telem.y_ned + world[1]
        wz = telem.z_ned + world[2]
        elev = -wz

        gx = ((wx - self._origin_x) / self._resolution_m).astype(np.int32)
        gy = ((wy - self._origin_y) / self._resolution_m).astype(np.int32)

        assert self._sum_elev is not None and self._count is not None and self._last_t is not None
        max_x, max_y = self._sum_elev.shape
        in_bounds = (
            (gx >= 0) & (gx < max_x) &
            (gy >= 0) & (gy < max_y)
        )
        if not np.any(in_bounds):
            return

        gx = gx[in_bounds]
        gy = gy[in_bounds]
        elev = elev[in_bounds]

        now = float(time.time())
        np.add.at(self._sum_elev, (gx, gy), elev)
        np.add.at(self._count, (gx, gy), 1)
        self._last_t[gx, gy] = now
        self._mapped_points += int(len(gx))

    def _decay(self) -> None:
        assert self._count is not None and self._sum_elev is not None and self._last_t is not None
        now = float(time.time())
        stale = (self._count > 0) & ((now - self._last_t) > MAP_DECAY_S)
        if np.any(stale):
            self._count[stale] = 0
            self._sum_elev[stale] = 0.0
            self._last_t[stale] = 0.0


def _body_to_ned(v_body: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)

    r = np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ], dtype=np.float32)
    return r @ v_body
