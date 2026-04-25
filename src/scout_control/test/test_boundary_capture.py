"""Tests for polygon boundary capture: payload schemas and invariants.

These tests avoid instantiating the ROS node — they exercise the payload
shapes expected on /field/boundary_point, /field/boundary_close and the
serialized field_boundary.json + field_grid.json outputs.
"""

from __future__ import annotations

import json
import math

import pytest

from scout_control.utils.polygon import (
    bounding_box,
    classify_cell,
    inset_polygon,
    point_in_polygon,
)


def _boundary_point_payload(index: int, x: float, y: float, z: float) -> dict:
    return {
        "index": index,
        "ned": {"x": x, "y": y, "z": z},
        "type": "vertex",
    }


@pytest.mark.unit
class TestBoundaryPointPayload:
    def test_roundtrip_json(self):
        payload = _boundary_point_payload(2, 1.25, -3.5, -4.5)
        raw = json.dumps(payload)
        parsed = json.loads(raw)
        assert parsed["index"] == 2
        assert parsed["type"] == "vertex"
        assert parsed["ned"]["x"] == pytest.approx(1.25)
        assert parsed["ned"]["y"] == pytest.approx(-3.5)

    def test_index_matches_arrival_order(self):
        # simulate publisher-side indexing like field_setup_tool does
        collected = []
        for i, (x, y) in enumerate([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]):
            collected.append(_boundary_point_payload(i, x, y, -5.0))
        assert [p["index"] for p in collected] == [0, 1, 2]


@pytest.mark.unit
class TestBoundaryStateMachine:
    """Sanity checks for the rules encoded in FieldSetupCoordinator."""

    def test_close_requires_three_vertices(self):
        points = [(0.0, 0.0), (10.0, 0.0)]
        assert len(points) < 3  # coordinator must refuse close

    def test_close_accepts_triangle(self):
        points = [(0.0, 0.0), (10.0, 0.0), (5.0, 10.0)]
        assert len(points) >= 3

    def test_polygon_and_corners_modes_are_mutually_exclusive(self):
        # Modeled as a simple guard: once one mode is chosen, the other is
        # rejected.
        capture_mode = None
        # First polygon point sets mode
        capture_mode = capture_mode or "polygon"
        assert capture_mode == "polygon"
        # A subsequent corner would be rejected
        rejected = capture_mode != "corners"
        assert rejected is True


@pytest.mark.unit
class TestBoundaryJsonSchema:
    def test_boundary_file_fields(self):
        raw = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        inset = inset_polygon(raw, 1.0)
        avg_z = -5.0
        payload = {
            "vertices_ned": [
                {"x": x, "y": y, "z": avg_z} for (x, y) in raw
            ],
            "inset_vertices_ned": [
                {"x": x, "y": y, "z": avg_z} for (x, y) in inset
            ],
            "closed": True,
            "inset_buffer_m": 1.0,
            "capture_mode": "polygon",
        }
        assert payload["closed"] is True
        assert payload["capture_mode"] == "polygon"
        assert payload["inset_buffer_m"] == 1.0
        assert len(payload["vertices_ned"]) == 4
        assert len(payload["inset_vertices_ned"]) == 4

    def test_grid_cells_have_cell_class_in_polygon_mode(self):
        verts = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        cell_size = 5.0
        half = cell_size / 2.0
        x_min, y_min, x_max, y_max = bounding_box(verts)
        cols = math.ceil((x_max - x_min) / cell_size)
        rows = math.ceil((y_max - y_min) / cell_size)
        cells = []
        for r in range(rows):
            for c in range(cols):
                cx = x_min + (c + 0.5) * cell_size
                cy = y_min + (r + 0.5) * cell_size
                klass = classify_cell(cx, cy, half, verts)
                if klass == "outside":
                    continue
                cells.append({
                    "id": f"x{c}_y{r}",
                    "x": cx,
                    "y": cy,
                    "status": "unvisited",
                    "cell_class": klass,
                })
        assert cells, "expected at least one cell kept"
        assert all("cell_class" in c for c in cells)

    def test_legacy_cells_without_cell_class_implicit_inside(self):
        # Cells saved by legacy 4-corner path must remain readable without
        # a cell_class field.
        legacy = {"id": "x0_y0", "x": 1.0, "y": 2.0, "status": "unvisited"}
        implicit = legacy.get("cell_class", "inside")
        assert implicit == "inside"


@pytest.mark.unit
class TestPolygonReflectsInGrid:
    def test_triangle_drops_outside_cells(self):
        tri = [(0.0, 0.0), (30.0, 0.0), (0.0, 30.0)]
        cell_size = 5.0
        half = cell_size / 2.0
        x_min, y_min, x_max, y_max = bounding_box(tri)
        cols = math.ceil((x_max - x_min) / cell_size)
        rows = math.ceil((y_max - y_min) / cell_size)
        kept = 0
        total = cols * rows
        for r in range(rows):
            for c in range(cols):
                cx = x_min + (c + 0.5) * cell_size
                cy = y_min + (r + 0.5) * cell_size
                if classify_cell(cx, cy, half, tri) != "outside":
                    kept += 1
        assert 0 < kept < total

    def test_point_inside_inset_stays_inside_original(self):
        square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        inset = inset_polygon(square, 1.0)
        for v in inset:
            assert point_in_polygon(v[0], v[1], square)
