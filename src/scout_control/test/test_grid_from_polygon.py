"""Unit tests for polygon helpers and polygon-aware grid generation."""

import math

import pytest

from scout_control.utils.polygon import (
    bounding_box,
    classify_cell,
    inset_polygon,
    is_ccw,
    point_in_polygon,
    signed_area,
)


@pytest.mark.unit
class TestPointInPolygon:
    def test_square_inside(self):
        square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        assert point_in_polygon(5.0, 5.0, square) is True

    def test_square_outside(self):
        square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        assert point_in_polygon(-1.0, 5.0, square) is False
        assert point_in_polygon(11.0, 5.0, square) is False
        assert point_in_polygon(5.0, -1.0, square) is False
        assert point_in_polygon(5.0, 11.0, square) is False

    def test_concave_polygon(self):
        # L-shape: outside of the notch but within bbox
        l_shape = [
            (0.0, 0.0), (10.0, 0.0), (10.0, 4.0),
            (4.0, 4.0), (4.0, 10.0), (0.0, 10.0),
        ]
        # In the body of the L
        assert point_in_polygon(2.0, 2.0, l_shape) is True
        assert point_in_polygon(2.0, 8.0, l_shape) is True
        # In the notch (outside)
        assert point_in_polygon(8.0, 8.0, l_shape) is False

    def test_degenerate(self):
        assert point_in_polygon(0.0, 0.0, []) is False
        assert point_in_polygon(0.0, 0.0, [(0.0, 0.0), (1.0, 1.0)]) is False


@pytest.mark.unit
class TestSignedAreaAndWinding:
    def test_ccw_positive(self):
        ccw = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        assert signed_area(ccw) > 0
        assert is_ccw(ccw)

    def test_cw_negative(self):
        cw = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
        assert signed_area(cw) < 0
        assert not is_ccw(cw)

    def test_bounding_box(self):
        poly = [(-2.0, 3.0), (5.0, 3.0), (5.0, 10.0), (-2.0, 10.0)]
        assert bounding_box(poly) == (-2.0, 3.0, 5.0, 10.0)


@pytest.mark.unit
class TestInsetPolygon:
    def test_square_inset_shrinks_by_buffer(self):
        square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        inset = inset_polygon(square, 1.0)
        assert len(inset) == 4
        xs = sorted(v[0] for v in inset)
        ys = sorted(v[1] for v in inset)
        assert xs[0] == pytest.approx(1.0, abs=1e-6)
        assert xs[-1] == pytest.approx(9.0, abs=1e-6)
        assert ys[0] == pytest.approx(1.0, abs=1e-6)
        assert ys[-1] == pytest.approx(9.0, abs=1e-6)

    def test_inset_handles_cw_input(self):
        cw_square = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
        inset = inset_polygon(cw_square, 1.0)
        # Still a ~8x8 square regardless of orientation
        x_min, y_min, x_max, y_max = bounding_box(inset)
        assert (x_max - x_min) == pytest.approx(8.0, abs=1e-6)
        assert (y_max - y_min) == pytest.approx(8.0, abs=1e-6)

    def test_inset_zero_returns_copy(self):
        square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        assert inset_polygon(square, 0.0) == square

    def test_inset_too_big_fallback(self):
        # 2x2 square with 5m inset would collapse — expect fallback.
        tiny = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        out = inset_polygon(tiny, 5.0)
        assert out == tiny


@pytest.mark.unit
class TestClassifyCell:
    def _square(self):
        return [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]

    def test_inside_cell(self):
        verts = self._square()
        assert classify_cell(10.0, 10.0, 1.0, verts) == "inside"

    def test_outside_cell(self):
        verts = self._square()
        assert classify_cell(-5.0, -5.0, 1.0, verts) == "outside"

    def test_edge_cell(self):
        verts = self._square()
        # Centre at (19.5, 10) with half=1 → cell spans x ∈ [18.5, 20.5],
        # so x=20.5 corner is outside the square.
        assert classify_cell(19.5, 10.0, 1.0, verts) == "edge"


@pytest.mark.unit
class TestGridFromPolygon:
    """Simulate what field_setup_coordinator._generate_grid_polygon does."""

    def test_grid_drops_outside_cells(self):
        # Triangle-ish polygon
        tri = [(0.0, 0.0), (20.0, 0.0), (0.0, 20.0)]
        cell_size = 5.0
        half = cell_size / 2.0
        x_min, y_min, x_max, y_max = bounding_box(tri)
        cols = max(1, math.ceil((x_max - x_min) / cell_size))
        rows = max(1, math.ceil((y_max - y_min) / cell_size))

        kept = 0
        for r in range(rows):
            for c in range(cols):
                cx = x_min + (c + 0.5) * cell_size
                cy = y_min + (r + 0.5) * cell_size
                if classify_cell(cx, cy, half, tri) != "outside":
                    kept += 1
        # Triangle has strictly fewer cells than full bbox (4x4=16)
        assert 0 < kept < cols * rows
