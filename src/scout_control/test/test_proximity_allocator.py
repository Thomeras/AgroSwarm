from scout_control.utils.proximity_allocator import assign_initial, assign_one, order_route


def _cell(cid: str, x: float, y: float) -> dict:
    return {"id": cid, "x": x, "y": y}


def test_assign_initial_empty_cells_and_no_drones() -> None:
    assert assign_initial([], {"drone_0": (0.0, 0.0)}) == {"drone_0": []}
    assert assign_initial([_cell("a", 0.0, 0.0)], {}) == {}


def test_assign_initial_balances_with_cap() -> None:
    cells = [_cell(f"c{i}", float(i), 0.0) for i in range(10)]
    positions = {
        "drone_0": (0.0, 0.0),
        "drone_1": (5.0, 0.0),
        "drone_2": (9.0, 0.0),
    }
    assigned = assign_initial(cells, positions)
    assert max(len(v) for v in assigned.values()) <= 5
    assert sorted(c["id"] for route in assigned.values() for c in route) == [
        f"c{i}" for i in range(10)
    ]


def test_assign_initial_proximity_clustering() -> None:
    cells = [
        _cell("left_a", 0.0, 0.0),
        _cell("left_b", 1.0, 0.0),
        _cell("right_a", 100.0, 0.0),
        _cell("right_b", 101.0, 0.0),
    ]
    assigned = assign_initial(
        cells,
        {"drone_0": (0.0, 0.0), "drone_1": (100.0, 0.0)},
    )
    assert {c["id"] for c in assigned["drone_0"]} == {"left_a", "left_b"}
    assert {c["id"] for c in assigned["drone_1"]} == {"right_a", "right_b"}


def test_order_route_empty_singleton_and_nearest_first() -> None:
    assert order_route((0.0, 0.0), []) == []
    one = [_cell("one", 4.0, 0.0)]
    assert order_route((0.0, 0.0), one) == one
    ordered = order_route(
        (0.0, 0.0),
        [_cell("far", 10.0, 0.0), _cell("near", 1.0, 0.0)],
    )
    assert [c["id"] for c in ordered] == ["near", "far"]


def test_assign_one_voronoi_and_fallback() -> None:
    pool = [_cell("mine", 0.0, 0.0), _cell("theirs", 10.0, 0.0)]
    selected = assign_one(
        pool,
        "drone_0",
        (0.0, 0.0),
        {"drone_1": (10.0, 0.0)},
    )
    assert selected is not None
    assert selected["id"] == "mine"

    fallback = assign_one(
        [_cell("only_peer_side", 10.0, 0.0)],
        "drone_0",
        (0.0, 0.0),
        {"drone_1": (9.0, 0.0)},
    )
    assert fallback is not None
    assert fallback["id"] == "only_peer_side"


def test_assign_initial_is_deterministic_and_does_not_mutate_inputs() -> None:
    cells = [_cell("b", 1.0, 0.0), {"id": "a", "center": (0.0, 0.0)}]
    positions = {"drone_1": (1.0, 0.0), "drone_0": (0.0, 0.0)}
    before = [dict(c) for c in cells]
    first = assign_initial(cells, positions)
    second = assign_initial(cells, positions)
    assert first == second
    assert cells == before
