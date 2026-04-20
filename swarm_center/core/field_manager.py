"""
field_manager.py — Field grid + boundary state

Loads field_grid.json produced by scout_control/grid_generator.py.
Provides utility functions for:
  • converting a drone NED position to a grid cell ID
  • iterating cells for rendering
  • tracking per-cell status (unvisited / hovering / visited / sprayed)

The grid in field_grid.json uses the same format written by GridGenerator:
    {
      "cell_size_m": 5.0,
      "cols": 20,
      "rows": 20,
      "cells": [
        {"id": "x0_y0", "col": 0, "row": 0,
         "x": 22.5, "y": -47.5, "status": "unvisited"},
        ...
      ]
    }

x, y are NED-frame cell CENTRES.
col/row are zero-indexed grid coordinates (col = East axis, row = North axis).

In Milestone 1 we don't write changes back — Swarm Center is read-only
on the grid file. Mission execution happens inside ROS2.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Cell:
    id: str
    col: int
    row: int
    x: float          # NED North — cell centre
    y: float          # NED East  — cell centre
    status: str = "unvisited"
    # Optional per-cell scan data — populated later by the AI pipeline
    health: Optional[float] = None    # 0..1
    pest_score: Optional[float] = None
    last_scanned_s: Optional[float] = None


@dataclass
class FieldGrid:
    cell_size_m: float
    cols: int
    rows: int
    cells: list[Cell] = field(default_factory=list)

    # Perimeter corners (NED North, East)
    corners: list[tuple[float, float]] = field(default_factory=list)
    # Landing pad positions (NED North, East)
    landing_pads: list[tuple[float, float]] = field(default_factory=list)

    # Bounding box of the grid — computed from cells
    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 0.0
    y_max: float = 0.0

    def __post_init__(self) -> None:
        if not self.cells:
            return
        # Each cell centre is half a cell in from the edge
        half = self.cell_size_m / 2.0
        xs = [c.x for c in self.cells]
        ys = [c.y for c in self.cells]
        self.x_min = min(xs) - half
        self.x_max = max(xs) + half
        self.y_min = min(ys) - half
        self.y_max = max(ys) + half

    # ── I/O ─────────────────────────────────────────────────────────────────

    @classmethod
    def from_file(cls, path: str) -> "FieldGrid":
        with open(path) as f:
            data = json.load(f)
        cells = [
            Cell(
                id=c["id"],
                col=c.get("col", _col_from_id(c["id"])),
                row=c.get("row", _row_from_id(c["id"])),
                x=float(c["x"]),
                y=float(c["y"]),
                status=c.get("status", "unvisited"),
            )
            for c in data["cells"]
        ]
        return cls(
            cell_size_m=float(data["cell_size_m"]),
            cols=int(data["cols"]),
            rows=int(data["rows"]),
            cells=cells,
        )

    @classmethod
    def synthetic(
        cls,
        cell_size_m: float = 5.0,
        field_size_m: float = 100.0,
        origin_x: float = 20.0,
        origin_y: float = -50.0,
    ) -> "FieldGrid":
        """
        Build a synthetic square grid matching scout_control's sim preset
        (100×100 m field, 5 m cells). Used when no grid file is loaded yet.
        """
        cols = max(1, int(math.ceil(field_size_m / cell_size_m)))
        rows = max(1, int(math.ceil(field_size_m / cell_size_m)))
        cells: list[Cell] = []
        for row in range(rows):
            for col in range(cols):
                cx = origin_x + (col + 0.5) * cell_size_m
                cy = origin_y + (row + 0.5) * cell_size_m
                cells.append(Cell(
                    id=f"x{col}_y{row}",
                    col=col,
                    row=row,
                    x=round(cx, 4),
                    y=round(cy, 4),
                ))
        return cls(
            cell_size_m=cell_size_m,
            cols=cols,
            rows=rows,
            cells=cells,
        )

    def regrid(self, new_cell_size: float) -> "FieldGrid":
        """Return a new FieldGrid with the same field bounds but a different cell size."""
        new_cell_size = max(0.1, new_cell_size)
        cols = max(1, math.ceil((self.x_max - self.x_min) / new_cell_size))
        rows = max(1, math.ceil((self.y_max - self.y_min) / new_cell_size))
        cells: list[Cell] = []
        for row in range(rows):
            for col in range(cols):
                cx = self.x_min + (col + 0.5) * new_cell_size
                cy = self.y_min + (row + 0.5) * new_cell_size
                cells.append(Cell(
                    id=f"x{col}_y{row}",
                    col=col,
                    row=row,
                    x=round(cx, 4),
                    y=round(cy, 4),
                ))
        return FieldGrid(
            cell_size_m=new_cell_size,
            cols=cols,
            rows=rows,
            cells=cells,
        )

    # ── Lookups ─────────────────────────────────────────────────────────────

    def cell_at_ned(self, x: float, y: float) -> Optional[Cell]:
        """
        Return the cell containing NED position (x, y), or None if outside.
        """
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            return None
        col = int((y - self.y_min) / self.cell_size_m)
        row = int((x - self.x_min) / self.cell_size_m)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return self.cell_by_coords(col, row)

    def cell_by_coords(self, col: int, row: int) -> Optional[Cell]:
        for c in self.cells:
            if c.col == col and c.row == row:
                return c
        return None

    def cell_by_id(self, cell_id: str) -> Optional[Cell]:
        for c in self.cells:
            if c.id == cell_id:
                return c
        return None


# ── Helpers ─────────────────────────────────────────────────────────────────


def _col_from_id(cid: str) -> int:
    # "x4_y2" → 4
    try:
        return int(cid.split("_")[0][1:])
    except Exception:
        return 0


def _row_from_id(cid: str) -> int:
    # "x4_y2" → 2
    try:
        return int(cid.split("_")[1][1:])
    except Exception:
        return 0


# ── Find a grid file in common locations ────────────────────────────────────


def find_default_grid_file() -> Optional[str]:
    """
    Hunt for field_grid.json in the likely places. Returns first match or None.
    """
    candidates = [
        os.path.expanduser("~/_Data/_Projekty/TJlabs/scout_ws/perimeters/field_grid.json"),
        os.path.expanduser("~/scout_ws/perimeters/field_grid.json"),
        "./field_grid.json",
        "./perimeters/field_grid.json",
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None
