# flake8: noqa
"""Numpy-backed 2.5D terrain heightmap for mapping outputs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(slots=True)
class Heightmap2D:
    """Store minimum NED z observed per grid cell."""

    origin_ned: tuple[float, float]
    cell_size_m: float
    width: int
    height: int
    min_z: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.origin_ned = (float(self.origin_ned[0]), float(self.origin_ned[1]))
        self.cell_size_m = float(self.cell_size_m)
        self.width = int(self.width)
        self.height = int(self.height)
        if self.cell_size_m <= 0.0:
            raise ValueError("cell_size_m must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("heightmap dimensions must be positive")
        if self.min_z is None:
            self.min_z = np.full((self.height, self.width), np.nan, dtype=np.float32)
        else:
            self.min_z = np.asarray(self.min_z, dtype=np.float32)
            if self.min_z.shape != (self.height, self.width):
                raise ValueError("min_z shape must match height/width")

    def world_to_grid(self, x_ned: float, y_ned: float) -> tuple[int, int]:
        gx = int(np.floor((float(x_ned) - self.origin_ned[0]) / self.cell_size_m))
        gy = int(np.floor((float(y_ned) - self.origin_ned[1]) / self.cell_size_m))
        return gx, gy

    def update_from_points(self, points_ned: np.ndarray) -> int:
        """Update min_z from Nx3 NED points and return accepted point count."""

        pts = np.asarray(points_ned, dtype=np.float32)
        if pts.size == 0:
            return 0
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("points_ned must be an Nx3 array")

        gx = np.floor((pts[:, 0] - self.origin_ned[0]) / self.cell_size_m).astype(np.int64)
        gy = np.floor((pts[:, 1] - self.origin_ned[1]) / self.cell_size_m).astype(np.int64)
        valid = (
            np.isfinite(pts[:, 2])
            & (gx >= 0)
            & (gy >= 0)
            & (gx < self.width)
            & (gy < self.height)
        )
        accepted = int(np.count_nonzero(valid))
        for x_idx, y_idx, z_val in zip(gx[valid], gy[valid], pts[:, 2][valid]):
            current = self.min_z[y_idx, x_idx]
            if np.isnan(current) or z_val < current:
                self.min_z[y_idx, x_idx] = z_val
        return accepted

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "origin_ned": [self.origin_ned[0], self.origin_ned[1]],
            "cell_size_m": self.cell_size_m,
            "width": self.width,
            "height": self.height,
            "min_z": self.min_z.tolist(),
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "Heightmap2D":
        return cls(
            origin_ned=tuple(payload["origin_ned"]),
            cell_size_m=float(payload["cell_size_m"]),
            width=int(payload["width"]),
            height=int(payload["height"]),
            min_z=np.asarray(payload["min_z"], dtype=np.float32),
        )

