"""
mission_package_builder.py — Phase 4A

Pure-Python, no ROS2 dependencies.
Assigns filtered grid cells (inside / edge / caution) to drones as ordered lists.

Strategies:
  "sector"      — split sorted columns into N contiguous sectors
  "round_robin" — interleave cells across drones
"""

from __future__ import annotations

import json
import os
import time
from typing import Any


_ASSIGNABLE_CLASSES = {"inside", "edge", "caution"}


class MissionPackageBuilder:

    def __init__(
        self,
        strategy: str = "sector",
        altitude_m: float = 5.0,
    ) -> None:
        if strategy not in ("sector", "round_robin"):
            raise ValueError(f"Unknown strategy: {strategy!r}")
        self.strategy = strategy
        self.altitude_m = altitude_m

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(
        self,
        cells: list[dict[str, Any]],
        drone_ids: list[str],
    ) -> dict[str, dict[str, Any]]:
        """Return a dict mapping drone_id → package dict (in-memory only)."""
        if not drone_ids:
            return {}

        assignable = self._filter_cells(cells)
        sorted_cells = self._sort_boustrophedon(assignable)

        if self.strategy == "sector":
            splits = self._split_sector(sorted_cells, len(drone_ids))
        else:
            splits = self._split_round_robin(sorted_cells, len(drone_ids))

        packages: dict[str, dict[str, Any]] = {}
        for drone_id, drone_cells in zip(drone_ids, splits):
            annotated = [
                dict(c, altitude_m=self.altitude_m, service_type="survey")
                for c in drone_cells
            ]
            packages[drone_id] = {
                "drone_id": drone_id,
                "cells": annotated,
                "total_cells": len(annotated),
                "strategy": self.strategy,
                "estimated_flight_time_s": None,
            }
        return packages

    def save(
        self,
        cells: list[dict[str, Any]],
        drone_ids: list[str],
        mission_id: str,
        output_dir: str,
    ) -> None:
        """Build packages and write per-drone JSON files."""
        packages = self.build(cells, drone_ids)
        ts = time.time()
        for drone_id, pkg in packages.items():
            drone_dir = os.path.join(output_dir, mission_id)
            os.makedirs(drone_dir, exist_ok=True)
            payload: dict[str, Any] = {
                "version": 1,
                "mission_id": mission_id,
                "drone_id": drone_id,
                "created_at_s": ts,
                **pkg,
            }
            out_path = os.path.join(drone_dir, f"{drone_id}.json")
            with open(out_path, "w") as f:
                json.dump(payload, f, indent=2)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _filter_cells(self, cells: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [c for c in cells if c.get("cell_class", "inside") in _ASSIGNABLE_CLASSES]

    def _sort_boustrophedon(
        self, cells: list[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        """Sort cells in lawnmower (boustrophedon) order by x-column then y-row."""
        if not cells:
            return []
        # Determine unique x values (columns)
        xs = sorted({c["x"] for c in cells})
        by_col: dict[float, list[dict[str, Any]]] = {x: [] for x in xs}
        for c in cells:
            by_col[c["x"]].append(c)

        result: list[dict[str, Any]] = []
        for i, x in enumerate(xs):
            col_cells = sorted(by_col[x], key=lambda c: c["y"])
            if i % 2 == 1:
                col_cells = list(reversed(col_cells))
            result.extend(col_cells)
        return result

    def _split_sector(
        self, cells: list[dict[str, Any]], n: int
    ) -> list[list[dict[str, Any]]]:
        total = len(cells)
        if n <= 0:
            return []
        size = total // n
        remainder = total % n
        splits: list[list[dict[str, Any]]] = []
        idx = 0
        for i in range(n):
            extra = 1 if i < remainder else 0
            end = idx + size + extra
            splits.append(cells[idx:end])
            idx = end
        return splits

    def _split_round_robin(
        self, cells: list[dict[str, Any]], n: int
    ) -> list[list[dict[str, Any]]]:
        splits: list[list[dict[str, Any]]] = [[] for _ in range(n)]
        for i, cell in enumerate(cells):
            splits[i % n].append(cell)
        return splits
