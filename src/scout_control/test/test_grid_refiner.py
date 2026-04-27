"""Unit tests for GridRefiner — Phase 4A."""
import json
import os
import tempfile

import pytest

from scout_control.mapping.obstacle_extractor import Obstacle
from scout_control.mapping.grid_refiner import GridRefiner


def _make_obstacle(x=5.0, y=5.0, size=1.0, z_min=-3.0, z_max=0.0) -> Obstacle:
    return Obstacle(
        centroid_ned=(x, y, (z_min + z_max) / 2),
        bbox_ned=(x - size, y - size, z_min, x + size, y + size, z_max),
        point_count=10,
        confidence=0.9,
    )


def _make_cells(grid_size=20, cell_size=2.0):
    cells = []
    cols = int(grid_size / cell_size)
    rows = int(grid_size / cell_size)
    for row in range(rows):
        for col in range(cols):
            cx = col * cell_size + cell_size / 2
            cy = row * cell_size + cell_size / 2
            cells.append({
                "id": f"x{col}_y{row}",
                "x": cx,
                "y": cy,
                "cell_class": "inside",
                "status": "unvisited",
            })
    return cells, cell_size


# Test 1: buňka přímo uvnitř inflated bbox → cell_class == "no_go"
def test_cell_inside_no_go_zone():
    refiner = GridRefiner(inflation_m=1.5, caution_buffer_m=1.0)
    obstacle = _make_obstacle(x=5.0, y=5.0, size=1.0)
    no_go_zones = refiner.build_no_go_zones([obstacle])
    cells = [{"id": "c0", "x": 4.0, "y": 4.0, "cell_class": "inside", "status": "unvisited"}]
    refined = refiner.refine_grid(cells, no_go_zones, cell_size=2.0)
    # center is at 4.0+1.0=5.0, 4.0+1.0=5.0 — right inside bbox_inflated [5-1-1.5, 5-1-1.5] = [2.5, 2.5] to [7.5, 7.5]
    assert refined[0]["cell_class"] == "no_go"


# Test 2: buňka v caution buffer → cell_class == "caution"
def test_cell_in_caution_buffer():
    refiner = GridRefiner(inflation_m=1.0, caution_buffer_m=2.0)
    obstacle = _make_obstacle(x=0.0, y=0.0, size=0.5)
    no_go_zones = refiner.build_no_go_zones([obstacle])
    # inflated bbox: [-1.5, -1.5] to [1.5, 1.5]
    # caution outer: [-3.5, -3.5] to [3.5, 3.5]
    # cell center at (3.0, 0.0) — outside no_go but inside caution
    cells = [{"id": "c0", "x": 2.5, "y": -0.5, "cell_class": "inside", "status": "unvisited"}]
    refined = refiner.refine_grid(cells, no_go_zones, cell_size=1.0)
    # center at 2.5+0.5=3.0, -0.5+0.5=0.0 → outside inflated (>1.5) but inside caution (<=3.5)
    assert refined[0]["cell_class"] == "caution"


# Test 3: buňka daleko od překážky → cell_class zachována ("inside")
def test_cell_far_from_obstacle_unchanged():
    refiner = GridRefiner(inflation_m=1.5, caution_buffer_m=1.0)
    obstacle = _make_obstacle(x=0.0, y=0.0, size=0.5)
    no_go_zones = refiner.build_no_go_zones([obstacle])
    cells = [{"id": "c0", "x": 20.0, "y": 20.0, "cell_class": "inside", "status": "unvisited"}]
    refined = refiner.refine_grid(cells, no_go_zones, cell_size=2.0)
    assert refined[0]["cell_class"] == "inside"


# Test 4: prázdný obstacles list → refined == base grid beze změny
def test_empty_obstacles_returns_base_grid():
    refiner = GridRefiner()
    cells, cell_size = _make_cells(grid_size=10, cell_size=2.0)
    original_classes = [c["cell_class"] for c in cells]
    no_go_zones = refiner.build_no_go_zones([])
    refined = refiner.refine_grid(cells, no_go_zones, cell_size=cell_size)
    assert len(refined) == len(cells)
    assert [c["cell_class"] for c in refined] == original_classes


# Test 5: refined_grid.json má správný JSON formát (version, cells, refined=True)
def test_save_refined_grid_json_format():
    refiner = GridRefiner()
    cells, cell_size = _make_cells(grid_size=6, cell_size=2.0)
    no_go_zones = refiner.build_no_go_zones([])
    refined = refiner.refine_grid(cells, no_go_zones, cell_size=cell_size)
    base_payload = {"cell_size_m": cell_size, "cells": cells}
    with tempfile.TemporaryDirectory() as tmpdir:
        refiner.save(refined, no_go_zones, tmpdir, base_payload)
        out_path = os.path.join(tmpdir, "refined_grid.json")
        assert os.path.exists(out_path)
        data = json.loads(open(out_path).read())
        assert data.get("version") == 2
        assert data.get("refined") is True
        assert "cells" in data
        assert isinstance(data["cells"], list)


# Test 6: no_go_zones.json má správný formát (zones list)
def test_save_no_go_zones_json_format():
    refiner = GridRefiner(inflation_m=1.5)
    obstacle = _make_obstacle(x=5.0, y=5.0, size=1.0)
    no_go_zones = refiner.build_no_go_zones([obstacle])
    with tempfile.TemporaryDirectory() as tmpdir:
        refiner.save([], no_go_zones, tmpdir, {})
        out_path = os.path.join(tmpdir, "no_go_zones.json")
        assert os.path.exists(out_path)
        data = json.loads(open(out_path).read())
        assert data.get("version") == 1
        assert "zones" in data
        assert isinstance(data["zones"], list)
        assert len(data["zones"]) == 1
        zone = data["zones"][0]
        assert "bbox_inflated" in zone
        assert len(zone["bbox_inflated"]) == 4


# Test 7: GridRefiner nemutuje vstupní cells list
def test_refiner_does_not_mutate_input():
    refiner = GridRefiner(inflation_m=1.5, caution_buffer_m=1.0)
    obstacle = _make_obstacle(x=5.0, y=5.0, size=1.0)
    no_go_zones = refiner.build_no_go_zones([obstacle])
    cells = [
        {"id": "c0", "x": 4.0, "y": 4.0, "cell_class": "inside", "status": "unvisited"},
        {"id": "c1", "x": 20.0, "y": 20.0, "cell_class": "inside", "status": "unvisited"},
    ]
    import copy
    original = copy.deepcopy(cells)
    refiner.refine_grid(cells, no_go_zones, cell_size=2.0)
    assert cells == original


# Test 8: build_no_go_zones inflates bbox correctly
def test_build_no_go_zones_inflation():
    refiner = GridRefiner(inflation_m=2.0)
    obstacle = _make_obstacle(x=10.0, y=10.0, size=1.0)
    # bbox_ned: [9.0, 9.0, ..., 11.0, 11.0, ...]
    zones = refiner.build_no_go_zones([obstacle])
    assert len(zones) == 1
    xmin, ymin, xmax, ymax = zones[0]["bbox_inflated"]
    assert abs(xmin - (10.0 - 1.0 - 2.0)) < 1e-6  # 7.0
    assert abs(ymin - (10.0 - 1.0 - 2.0)) < 1e-6  # 7.0
    assert abs(xmax - (10.0 + 1.0 + 2.0)) < 1e-6  # 13.0
    assert abs(ymax - (10.0 + 1.0 + 2.0)) < 1e-6  # 13.0
