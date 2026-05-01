from copy import deepcopy

from scout_control.utils.route_conflict import build_legs, find_conflicts, resolve


def _cell(cid: str, x: float, y: float) -> dict:
    return {"id": cid, "x": x, "y": y}


def test_build_legs_math_and_zero_speed_fallback() -> None:
    routes = {"drone_0": [_cell("a", 3.0, 4.0), _cell("b", 6.0, 8.0)]}
    legs = build_legs(routes, {"drone_0": (0.0, 0.0)}, cruise_speed=5.0, dwell_s=2.0)
    assert legs[0].t_enter == 1.0
    assert legs[0].t_exit == 3.0
    assert legs[1].t_enter == 4.0
    assert legs[1].t_exit == 6.0

    fallback = build_legs(
        {"drone_0": [_cell("a", 3.0, 4.0)]},
        {"drone_0": (0.0, 0.0)},
        cruise_speed=0.0,
        dwell_s=1.0,
    )
    assert fallback[0].t_enter == 5.0
    assert fallback[0].t_exit == 6.0


def test_find_conflicts_cell_overlap_and_disjoint() -> None:
    legs = build_legs(
        {
            "drone_0": [_cell("a", 1.0, 0.0)],
            "drone_1": [_cell("b", 1.2, 0.0)],
        },
        {"drone_0": (0.0, 0.0), "drone_1": (0.0, 0.0)},
        cruise_speed=1.0,
        dwell_s=5.0,
    )
    conflicts = find_conflicts(legs, nfz_radius=1.0, time_window_s=0.1)
    assert len(conflicts) == 1
    assert conflicts[0]["type"] == "cell"

    far = build_legs(
        {
            "drone_0": [_cell("a", 1.0, 0.0)],
            "drone_1": [_cell("b", 10.0, 0.0)],
        },
        {"drone_0": (0.0, 0.0), "drone_1": (0.0, 0.0)},
        cruise_speed=1.0,
        dwell_s=1.0,
    )
    assert find_conflicts(far, nfz_radius=1.0, time_window_s=0.1) == []


def test_find_conflicts_crossing() -> None:
    legs = build_legs(
        {
            "drone_0": [_cell("a", 1.0, 0.0)],
            "drone_1": [_cell("b", 1.2, 0.0)],
        },
        {"drone_0": (0.0, 0.0), "drone_1": (0.0, 0.0)},
        cruise_speed=1.0,
        dwell_s=0.0,
    )
    conflicts = find_conflicts(legs, nfz_radius=1.0, time_window_s=1.0)
    assert len(conflicts) == 1
    assert conflicts[0]["type"] == "crossing"


def test_resolve_wait_for_lower_priority() -> None:
    routes = {
        "drone_0": [_cell("a", 0.0, 0.0)],
        "drone_1": [_cell("b", 0.5, 0.0)],
    }
    conflicts = [{
        "a": "drone_0",
        "b": "drone_1",
        "cell_a": "a",
        "cell_b": "b",
        "type": "crossing",
        "t_overlap": 2.0,
    }]
    resolved, actions = resolve(conflicts, routes, priority_fn=lambda d: d)
    assert resolved == routes
    assert actions == [{"kind": "WAIT", "drone_id": "drone_1", "t_s": 2.0}]


def test_resolve_swap_for_same_cell_and_no_oscillation() -> None:
    routes = {
        "drone_0": [_cell("a", 0.0, 0.0), _cell("same", 1.0, 0.0)],
        "drone_1": [_cell("b", 2.0, 0.0), _cell("same", 1.0, 0.0)],
    }
    conflicts = [
        {
            "a": "drone_0",
            "b": "drone_1",
            "cell_a": "same",
            "cell_b": "same",
            "type": "cell",
            "t_overlap": 1.0,
        },
        {
            "a": "drone_1",
            "b": "drone_0",
            "cell_a": "same",
            "cell_b": "same",
            "type": "cell",
            "t_overlap": 1.0,
        },
    ]
    resolved, actions = resolve(conflicts, routes, priority_fn=lambda d: d)
    assert len(actions) == 1
    assert actions[0]["kind"] == "SWAP"
    assert [c["id"] for c in resolved["drone_0"]] == ["a", "same"]
    assert [c["id"] for c in resolved["drone_1"]] == ["b", "same"]


def test_resolve_does_not_mutate_inputs() -> None:
    routes = {"drone_0": [_cell("a", 0.0, 0.0)], "drone_1": [_cell("b", 1.0, 0.0)]}
    before = deepcopy(routes)
    resolve(
        [{
            "a": "drone_0",
            "b": "drone_1",
            "cell_a": "a",
            "cell_b": "b",
            "type": "crossing",
            "t_overlap": 1.0,
        }],
        routes,
        priority_fn=lambda d: d,
    )
    assert routes == before
