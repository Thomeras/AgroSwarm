# flake8: noqa
"""Tests for grid-based obstacle extraction."""

import numpy as np
import pytest

from scout_control.mapping.obstacle_extractor import extract_obstacles


@pytest.mark.unit
def test_empty_points_return_no_obstacles():
    assert extract_obstacles(np.empty((0, 3))) == []


@pytest.mark.unit
def test_clusters_synthetic_cloud():
    points = np.array([
        [0.0, 0.0, -1.0],
        [0.2, 0.1, -1.1],
        [0.3, 0.0, -0.9],
        [5.0, 5.0, -1.0],
        [5.1, 5.1, -1.2],
        [5.2, 5.0, -1.1],
    ])
    obstacles = extract_obstacles(points, cell_size_m=0.5, min_points=3)
    assert len(obstacles) == 2
    assert obstacles[0].point_count == 3


@pytest.mark.unit
def test_single_point_below_threshold_is_ignored():
    points = np.array([[0.0, 0.0, -1.0]])
    assert extract_obstacles(points, min_points=2) == []

