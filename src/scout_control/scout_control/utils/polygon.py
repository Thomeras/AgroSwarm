"""
polygon.py — pure-Python 2D polygon helpers used by the field setup path.

No ROS, no numpy — importable in tests and in both field_setup_coordinator
and grid_generator.

Coordinate convention: all vertices are (x, y) pairs. The callers use NED
(x=North, y=East), but the helpers are orientation-agnostic: shoelace sign
is used internally to detect winding order.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

Point = tuple[float, float]


def signed_area(verts: Sequence[Point]) -> float:
    """Return signed polygon area (positive if CCW in standard math axes)."""
    n = len(verts)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def is_ccw(verts: Sequence[Point]) -> bool:
    return signed_area(verts) > 0.0


def bounding_box(verts: Sequence[Point]) -> tuple[float, float, float, float]:
    """Return (x_min, y_min, x_max, y_max)."""
    xs = [p[0] for p in verts]
    ys = [p[1] for p in verts]
    return min(xs), min(ys), max(xs), max(ys)


def point_in_polygon(x: float, y: float, verts: Sequence[Point]) -> bool:
    """Ray casting point-in-polygon test. Edge cases count as inside."""
    n = len(verts)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        # Ray going to +x direction from (x,y) — count edge crossings.
        if ((yi > y) != (yj > y)):
            x_intersect = (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi
            if x <= x_intersect:
                inside = not inside
        j = i
    return inside


def _line_intersection(
    p1: Point, p2: Point, p3: Point, p4: Point
) -> Point | None:
    """Return intersection point of infinite lines p1p2 and p3p4, or None."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def inset_polygon(verts: Sequence[Point], inset_m: float) -> list[Point]:
    """Shrink a simple polygon inward by `inset_m` metres.

    Works for convex and mildly non-convex polygons by offsetting each edge
    inward along its normal and intersecting consecutive offset edges.

    If inset_m <= 0 returns a copy of the input. If the polygon is too small
    to inset (collapses), returns the original vertices unchanged as a safe
    fallback.
    """
    if inset_m <= 0.0 or len(verts) < 3:
        return list(verts)

    poly = list(verts)
    # Normalize winding so the "inward" normal is consistent. Work in CCW.
    if not is_ccw(poly):
        poly = list(reversed(poly))

    n = len(poly)
    offset_edges: list[tuple[Point, Point]] = []
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        # For CCW polygon, inward normal is (-dy, dx) / length.
        nx = -dy / length
        ny = dx / length
        offset_edges.append(
            (
                (x1 + nx * inset_m, y1 + ny * inset_m),
                (x2 + nx * inset_m, y2 + ny * inset_m),
            )
        )

    m = len(offset_edges)
    if m < 3:
        return list(verts)

    new_verts: list[Point] = []
    for i in range(m):
        e1 = offset_edges[i]
        e2 = offset_edges[(i + 1) % m]
        pt = _line_intersection(e1[0], e1[1], e2[0], e2[1])
        if pt is None:
            pt = e1[1]
        new_verts.append(pt)

    # Safety: if the inset collapsed the polygon (area flipped sign, area
    # near zero, or area grew because offset edges intersected on the wrong
    # side of the original), fall back to the original.
    new_area = signed_area(new_verts)
    orig_area = signed_area(poly)
    if abs(new_area) < 1e-3:
        return list(verts)
    if (new_area > 0) != (orig_area > 0):
        return list(verts)
    if abs(new_area) > abs(orig_area):
        return list(verts)

    return new_verts


def cell_overlaps_polygon_edge(
    cx: float,
    cy: float,
    half_size: float,
    verts: Sequence[Point],
) -> bool:
    """True if the axis-aligned cell square crosses any polygon edge.

    Cheap test: check if any of the 4 cell corners is outside while centre
    is inside. Used to classify a cell as 'edge' vs 'inside'.
    """
    corners = [
        (cx - half_size, cy - half_size),
        (cx + half_size, cy - half_size),
        (cx + half_size, cy + half_size),
        (cx - half_size, cy + half_size),
    ]
    inside_flags = [point_in_polygon(x, y, verts) for (x, y) in corners]
    return not all(inside_flags)


def classify_cell(
    cx: float,
    cy: float,
    half_size: float,
    verts: Sequence[Point],
) -> str:
    """Return 'inside', 'edge', or 'outside' for a cell centred at (cx, cy)."""
    if not point_in_polygon(cx, cy, verts):
        return "outside"
    if cell_overlaps_polygon_edge(cx, cy, half_size, verts):
        return "edge"
    return "inside"


def verts_from_dicts(items: Iterable[dict]) -> list[Point]:
    """Parse a list of {'x':, 'y':} or {'ned': {'x','y'}} dicts into points."""
    out: list[Point] = []
    for it in items:
        if "ned" in it and isinstance(it["ned"], dict):
            out.append((float(it["ned"]["x"]), float(it["ned"]["y"])))
        else:
            out.append((float(it["x"]), float(it["y"])))
    return out
