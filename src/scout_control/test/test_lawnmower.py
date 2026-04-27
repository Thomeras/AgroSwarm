# flake8: noqa
"""Tests for Phase 3 lawnmower route generation."""

import pytest

from scout_control.utils.lawnmower import generate_lawnmower
from scout_control.utils.polygon import point_in_polygon


def _square():
    return [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]


@pytest.mark.unit
def test_generates_routes_for_each_drone():
    routes = generate_lawnmower(_square(), 2, 5.0, 8.0, 0.0)
    assert set(routes) == {0, 1}
    assert routes[0]
    assert routes[1]
    assert all(wp[2] == pytest.approx(-8.0) for route in routes.values() for wp in route)


@pytest.mark.unit
def test_waypoints_are_clipped_to_polygon():
    poly = [(0.0, 0.0), (20.0, 0.0), (10.0, 10.0)]
    routes = generate_lawnmower(poly, 1, 3.0, 5.0, 20.0)
    assert routes[0]
    for x, y, _z in routes[0]:
        assert point_in_polygon(x, y, poly)


@pytest.mark.unit
def test_overlap_increases_number_of_waypoints():
    no_overlap = generate_lawnmower(_square(), 1, 5.0, 5.0, 0.0)
    overlap = generate_lawnmower(_square(), 1, 5.0, 5.0, 50.0)
    assert len(overlap[0]) > len(no_overlap[0])


@pytest.mark.unit
def test_invalid_drone_count_rejected():
    with pytest.raises(ValueError):
        generate_lawnmower(_square(), 0, 5.0, 5.0)

