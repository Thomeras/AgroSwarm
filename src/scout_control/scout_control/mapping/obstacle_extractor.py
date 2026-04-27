# flake8: noqa
"""Grid-based static obstacle extraction from mapping point batches."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(slots=True)
class Obstacle:
    centroid_ned: tuple[float, float, float]
    bbox_ned: tuple[float, float, float, float, float, float]
    point_count: int
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "centroid_ned": list(self.centroid_ned),
            "bbox_ned": list(self.bbox_ned),
            "point_count": int(self.point_count),
            "confidence": float(self.confidence),
        }


def _neighbors(cell: tuple[int, int]) -> list[tuple[int, int]]:
    x, y = cell
    return [(x + dx, y + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]


def extract_obstacles(
    points_ned: np.ndarray,
    *,
    cell_size_m: float = 0.75,
    min_points: int = 3,
) -> list[Obstacle]:
    """Cluster points by occupied grid adjacency without external deps."""

    pts = np.asarray(points_ned, dtype=np.float32)
    if pts.size == 0:
        return []
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points_ned must be an Nx3 array")
    pts = pts[np.all(np.isfinite(pts[:, :3]), axis=1), :3]
    if pts.size == 0:
        return []
    if cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be positive")

    cells = np.floor(pts[:, :2] / float(cell_size_m)).astype(np.int64)
    cell_to_indices: dict[tuple[int, int], list[int]] = {}
    for idx, cell in enumerate(cells):
        cell_to_indices.setdefault((int(cell[0]), int(cell[1])), []).append(idx)

    remaining = set(cell_to_indices)
    obstacles: list[Obstacle] = []
    while remaining:
        start = remaining.pop()
        stack = [start]
        cluster_cells = [start]
        while stack:
            cell = stack.pop()
            for nb in _neighbors(cell):
                if nb in remaining:
                    remaining.remove(nb)
                    stack.append(nb)
                    cluster_cells.append(nb)
        indices: list[int] = []
        for cell in cluster_cells:
            indices.extend(cell_to_indices[cell])
        if len(indices) < min_points:
            continue
        cpts = pts[indices]
        mins = np.min(cpts, axis=0)
        maxs = np.max(cpts, axis=0)
        centroid = np.mean(cpts, axis=0)
        confidence = min(1.0, len(indices) / max(float(min_points * 4), 1.0))
        obstacles.append(
            Obstacle(
                centroid_ned=(float(centroid[0]), float(centroid[1]), float(centroid[2])),
                bbox_ned=(
                    float(mins[0]),
                    float(mins[1]),
                    float(mins[2]),
                    float(maxs[0]),
                    float(maxs[1]),
                    float(maxs[2]),
                ),
                point_count=len(indices),
                confidence=confidence,
            )
        )
    obstacles.sort(key=lambda item: item.point_count, reverse=True)
    return obstacles

