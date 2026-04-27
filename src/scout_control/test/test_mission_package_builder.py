"""Unit tests for MissionPackageBuilder — Phase 4A."""
import json
import os
import tempfile

import pytest

from scout_control.mapping.mission_package_builder import MissionPackageBuilder


def _make_cells(n=10, with_no_go=True):
    cells = []
    for i in range(n):
        klass = "no_go" if (with_no_go and i == 0) else "inside"
        cells.append({
            "id": f"x{i}_y0",
            "x": float(i * 2),
            "y": 0.0,
            "cell_class": klass,
            "status": "unvisited",
        })
    return cells


# Test 1: sector strategie rozdělí buňky rovnoměrně mezi drony
def test_sector_splits_evenly():
    cells = _make_cells(n=10, with_no_go=False)
    builder = MissionPackageBuilder(strategy="sector", altitude_m=5.0)
    packages = builder.build(cells, drone_ids=["d0", "d1"])
    total = sum(len(p["cells"]) for p in packages.values())
    assert total == 10
    # Each drone should get roughly half
    assert abs(len(packages["d0"]["cells"]) - len(packages["d1"]["cells"])) <= 1


# Test 2: no_go buňky nejsou v žádném package
def test_no_go_cells_excluded():
    cells = _make_cells(n=6, with_no_go=True)
    builder = MissionPackageBuilder()
    packages = builder.build(cells, drone_ids=["d0", "d1"])
    for drone_id, pkg in packages.items():
        for cell in pkg["cells"]:
            assert cell["cell_class"] != "no_go", (
                f"Drone {drone_id} got a no_go cell"
            )


# Test 3: package soubory mají správný JSON formát
def test_package_files_json_format():
    cells = _make_cells(n=4, with_no_go=False)
    builder = MissionPackageBuilder(altitude_m=7.0)
    with tempfile.TemporaryDirectory() as tmpdir:
        mission_id = "mission_test_001"
        builder.save(cells, drone_ids=["d0"], mission_id=mission_id, output_dir=tmpdir)
        pkg_path = os.path.join(tmpdir, mission_id, "d0.json")
        assert os.path.exists(pkg_path)
        data = json.loads(open(pkg_path).read())
        assert data["version"] == 1
        assert data["mission_id"] == mission_id
        assert data["drone_id"] == "d0"
        assert isinstance(data["cells"], list)
        assert data["total_cells"] == len(data["cells"])
        assert data["strategy"] in ("sector", "round_robin")
        for cell in data["cells"]:
            assert "altitude_m" in cell
            assert cell["altitude_m"] == 7.0


# Test 4: round_robin přiřazuje buňky střídavě
def test_round_robin_alternates():
    cells = _make_cells(n=6, with_no_go=False)
    builder = MissionPackageBuilder(strategy="round_robin")
    packages = builder.build(cells, drone_ids=["d0", "d1", "d2"])
    total = sum(len(p["cells"]) for p in packages.values())
    assert total == 6
    # each drone gets exactly 2 cells
    for pkg in packages.values():
        assert len(pkg["cells"]) == 2


# Test 5: 1 dron dostane všechny buňky
def test_single_drone_gets_all():
    cells = _make_cells(n=5, with_no_go=False)
    builder = MissionPackageBuilder(strategy="sector")
    packages = builder.build(cells, drone_ids=["solo"])
    assert len(packages["solo"]["cells"]) == 5


# Test 6: více dronů než buněk → někteří dostanou prázdný package
def test_more_drones_than_cells():
    cells = _make_cells(n=2, with_no_go=False)
    builder = MissionPackageBuilder(strategy="sector")
    packages = builder.build(cells, drone_ids=["d0", "d1", "d2", "d3"])
    total = sum(len(p["cells"]) for p in packages.values())
    assert total == 2
    empty_count = sum(1 for p in packages.values() if len(p["cells"]) == 0)
    assert empty_count == 2


# Test 7: caution buňky jsou zahrnuty (jen no_go se vylučují)
def test_caution_cells_included():
    cells = [
        {"id": "c0", "x": 0.0, "y": 0.0, "cell_class": "caution", "status": "unvisited"},
        {"id": "c1", "x": 2.0, "y": 0.0, "cell_class": "edge",    "status": "unvisited"},
        {"id": "c2", "x": 4.0, "y": 0.0, "cell_class": "no_go",   "status": "unvisited"},
        {"id": "c3", "x": 6.0, "y": 0.0, "cell_class": "inside",  "status": "unvisited"},
    ]
    builder = MissionPackageBuilder()
    packages = builder.build(cells, drone_ids=["d0"])
    ids = {c["id"] for c in packages["d0"]["cells"]}
    assert "c0" in ids   # caution included
    assert "c1" in ids   # edge included
    assert "c2" not in ids  # no_go excluded
    assert "c3" in ids   # inside included
