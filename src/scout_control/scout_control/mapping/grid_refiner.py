"""
grid_refiner.py — Phase 4A

Pure-Python, no ROS2 dependencies.
Consumes Phase 3 field_model obstacles and a base grid (field_grid.json cells)
and produces:
  - refined_grid.json  (adds no_go / caution cell_class values)
  - no_go_zones.json   (inflated AABB polygons per obstacle)
"""

from __future__ import annotations

import json
import os
import time
from typing import Any

from scout_control.mapping.obstacle_extractor import Obstacle


class GridRefiner:
    """Refine a flat grid with obstacle-derived no_go / caution zones.

    inflation_m:       horizontal padding added to each obstacle bbox
    caution_buffer_m:  additional buffer outside the no_go zone
    """

    def __init__(
        self,
        inflation_m: float = 1.5,
        caution_buffer_m: float = 1.0,
    ) -> None:
        self.inflation_m = inflation_m
        self.caution_buffer_m = caution_buffer_m

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build_no_go_zones(self, obstacles: list[Obstacle]) -> list[dict[str, Any]]:
        """Return inflated AABB per obstacle (horizontal only, z unchanged)."""
        zones = []
        for obs in obstacles:
            x_min, y_min, z_min, x_max, y_max, z_max = obs.bbox_ned
            inf = self.inflation_m
            zones.append({
                "centroid_ned": list(obs.centroid_ned),
                "bbox_inflated": [
                    x_min - inf,
                    y_min - inf,
                    x_max + inf,
                    y_max + inf,
                ],
                "confidence": obs.confidence,
                "original_bbox": list(obs.bbox_ned),
            })
        return zones

    def refine_grid(
        self,
        cells: list[dict[str, Any]],
        no_go_zones: list[dict[str, Any]],
        cell_size: float,
    ) -> list[dict[str, Any]]:
        """Return a new cells list with updated cell_class values.

        Priority: no_go > caution > original class.
        Input list is not mutated.
        """
        half = cell_size / 2.0
        buf = self.caution_buffer_m
        refined: list[dict[str, Any]] = []
        for cell in cells:
            cx = cell["x"] + half
            cy = cell["y"] + half
            new_class = self._classify_cell(cx, cy, no_go_zones, buf)
            if new_class is not None:
                refined.append(dict(cell, cell_class=new_class))
            else:
                refined.append(dict(cell))
        return refined

    def save(
        self,
        refined_cells: list[dict[str, Any]],
        no_go_zones: list[dict[str, Any]],
        output_dir: str,
        base_payload: dict[str, Any],
    ) -> None:
        """Write refined_grid.json and no_go_zones.json to output_dir."""
        os.makedirs(output_dir, exist_ok=True)

        # refined_grid.json — inherit base payload, bump version, mark refined
        grid_payload: dict[str, Any] = {
            **base_payload,
            "cells": refined_cells,
            "version": 2,
            "refined": True,
            "created_at_s": time.time(),
        }
        with open(os.path.join(output_dir, "refined_grid.json"), "w") as f:
            json.dump(grid_payload, f, indent=2)

        # no_go_zones.json
        zones_payload: dict[str, Any] = {
            "version": 1,
            "created_at_s": time.time(),
            "zones": no_go_zones,
        }
        with open(os.path.join(output_dir, "no_go_zones.json"), "w") as f:
            json.dump(zones_payload, f, indent=2)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _classify_cell(
        self,
        cx: float,
        cy: float,
        zones: list[dict[str, Any]],
        caution_buf: float,
    ) -> str | None:
        best: str | None = None
        for zone in zones:
            xmin, ymin, xmax, ymax = zone["bbox_inflated"]
            if xmin <= cx <= xmax and ymin <= cy <= ymax:
                return "no_go"
            outer_xmin = xmin - caution_buf
            outer_ymin = ymin - caution_buf
            outer_xmax = xmax + caution_buf
            outer_ymax = ymax + caution_buf
            if outer_xmin <= cx <= outer_xmax and outer_ymin <= cy <= outer_ymax:
                best = "caution"
        return best
