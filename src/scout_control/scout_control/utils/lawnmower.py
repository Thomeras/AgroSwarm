# flake8: noqa
"""Pure-Python lawnmower route generation for mapping missions.

Coordinates use local NED convention: x=North, y=East, z=Down.  The generator
returns waypoints as ``(x, y, z)`` tuples, where ``z`` is negative altitude.
"""

from __future__ import annotations

import math
from typing import Sequence

from scout_control.utils.polygon import bounding_box, point_in_polygon

Point2 = tuple[float, float]
Waypoint = tuple[float, float, float]


def _line_polygon_intervals(x: float, polygon: Sequence[Point2]) -> list[tuple[float, float]]:
    """Return y-intervals where a vertical line at x lies inside polygon."""

    ys: list[float] = []
    n = len(polygon)
    if n < 3:
        return []
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if abs(x2 - x1) < 1e-9:
            continue
        lo = min(x1, x2)
        hi = max(x1, x2)
        if lo <= x < hi or math.isclose(x, hi, abs_tol=1e-9):
            t = (x - x1) / (x2 - x1)
            if -1e-9 <= t <= 1.0 + 1e-9:
                ys.append(y1 + t * (y2 - y1))
    ys = sorted(ys)
    intervals: list[tuple[float, float]] = []
    for idx in range(0, len(ys) - 1, 2):
        y0, y1 = ys[idx], ys[idx + 1]
        if y1 - y0 > 1e-6:
            intervals.append((y0, y1))
    return intervals


def _sample_lines(x_min: float, x_max: float, spacing: float) -> list[float]:
    width = x_max - x_min
    if width <= 1e-9:
        return [x_min]
    count = max(1, int(math.floor(width / spacing)) + 1)
    xs = [x_min + i * spacing for i in range(count)]
    if xs[-1] < x_max - spacing * 0.25:
        xs.append(x_max)
    return xs


def generate_lawnmower(
    polygon_vertices_ned: Sequence[Sequence[float]],
    drone_count: int,
    line_spacing_m: float,
    altitude_m: float,
    side_overlap_pct: float = 30.0,
) -> dict[int, list[Waypoint]]:
    """Generate per-drone boustrophedon mapping routes clipped to a polygon.

    The polygon is split along the AABB x axis into equal-width stripes.  Each
    drone receives vertical sweep lines in its stripe and alternates line
    direction to avoid unnecessary transit hops.
    """

    if drone_count <= 0:
        raise ValueError("drone_count must be positive")
    if line_spacing_m <= 0.0:
        raise ValueError("line_spacing_m must be positive")
    if altitude_m < 0.0:
        raise ValueError("altitude_m must be non-negative")

    polygon: list[Point2] = [(float(p[0]), float(p[1])) for p in polygon_vertices_ned]
    if len(polygon) < 3:
        raise ValueError("polygon must contain at least three vertices")

    overlap = min(95.0, max(0.0, float(side_overlap_pct))) / 100.0
    effective_spacing = max(0.1, float(line_spacing_m) * (1.0 - overlap))
    x_min, _y_min, x_max, _y_max = bounding_box(polygon)
    stripe_width = (x_max - x_min) / float(drone_count)
    z_ned = -float(altitude_m)
    routes: dict[int, list[Waypoint]] = {idx: [] for idx in range(drone_count)}

    for drone_id in range(drone_count):
        stripe_x0 = x_min + drone_id * stripe_width
        stripe_x1 = x_max if drone_id == drone_count - 1 else stripe_x0 + stripe_width
        if stripe_x1 < stripe_x0:
            stripe_x0, stripe_x1 = stripe_x1, stripe_x0
        xs = _sample_lines(stripe_x0, stripe_x1, effective_spacing)
        reverse = False
        for x in xs:
            intervals = _line_polygon_intervals(x, polygon)
            for y0, y1 in intervals:
                a = (x, y0)
                b = (x, y1)
                # Nudge exact edge samples through the same point-in-polygon
                # helper used by Phase 2; edge cases count as inside there.
                if not point_in_polygon(a[0], a[1], polygon):
                    a = (x, y0 + 1e-6)
                if not point_in_polygon(b[0], b[1], polygon):
                    b = (x, y1 - 1e-6)
                line_points = [a, b]
                if reverse:
                    line_points.reverse()
                for px, py in line_points:
                    if point_in_polygon(px, py, polygon):
                        routes[drone_id].append((float(px), float(py), z_ned))
                reverse = not reverse

    return routes

