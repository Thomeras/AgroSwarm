# flake8: noqa
"""Tests for Phase 3 heightmap persistence helper."""

import numpy as np
import pytest

from scout_control.mapping.heightmap import Heightmap2D


@pytest.mark.unit
def test_update_from_points_keeps_min_z():
    hm = Heightmap2D(origin_ned=(0.0, 0.0), cell_size_m=1.0, width=4, height=4)
    accepted = hm.update_from_points(np.array([[0.2, 0.2, -1.0], [0.7, 0.8, -2.0]]))
    assert accepted == 2
    assert hm.min_z[0, 0] == pytest.approx(-2.0)


@pytest.mark.unit
def test_out_of_bounds_are_ignored():
    hm = Heightmap2D(origin_ned=(0.0, 0.0), cell_size_m=1.0, width=2, height=2)
    accepted = hm.update_from_points(np.array([[4.0, 4.0, -1.0], [1.0, 1.0, -3.0]]))
    assert accepted == 1
    assert hm.min_z[1, 1] == pytest.approx(-3.0)


@pytest.mark.unit
def test_json_roundtrip():
    hm = Heightmap2D(origin_ned=(-1.0, -2.0), cell_size_m=0.5, width=2, height=2)
    hm.update_from_points(np.array([[-0.8, -1.8, -4.0]]))
    restored = Heightmap2D.from_dict(hm.to_dict())
    assert restored.origin_ned == pytest.approx((-1.0, -2.0))
    assert restored.min_z[0, 0] == pytest.approx(-4.0)

