"""Proximity-based cell allocation helpers."""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Optional


def _cell_id(cell: dict) -> str:
    return str(cell.get("cell_id", cell.get("id", "")))


def _xy(cell: dict) -> tuple[float, float]:
    center = cell.get("center")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        return (float(center[0]), float(center[1]))
    return (float(cell.get("x", 0.0)), float(cell.get("y", 0.0)))


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _clone_cells(cells: list[dict]) -> list[dict]:
    return [dict(cell) for cell in cells]


def assign_initial(
    cells: list[dict],
    drone_positions: dict[str, tuple[float, float]],
    n_per_drone_hint: Optional[int] = None,
) -> dict[str, list[dict]]:
    """Assign cells to drones by proximity, with deterministic balancing."""
    drone_ids = sorted(drone_positions)
    if not drone_ids:
        return {}

    result: dict[str, list[dict]] = {drone_id: [] for drone_id in drone_ids}
    if not cells:
        return result

    cap = math.ceil(len(cells) / max(1, len(drone_ids))) + 1
    if n_per_drone_hint is not None:
        cap = max(cap, int(n_per_drone_hint))

    indexed_cells = sorted(
        [(idx, dict(cell), _xy(cell)) for idx, cell in enumerate(cells)],
        key=lambda item: (_cell_id(item[1]), item[0]),
    )

    try:
        from scipy.optimize import linear_sum_assignment

        slots: list[str] = []
        for drone_id in drone_ids:
            slots.extend([drone_id] * cap)
        costs = [
            [_dist(cell_xy, drone_positions[drone_id]) for drone_id in slots]
            for _, _, cell_xy in indexed_cells
        ]
        row_ind, col_ind = linear_sum_assignment(costs)
        assignments = sorted(
            (int(row), int(col)) for row, col in zip(row_ind, col_ind)
        )
        for row, col in assignments:
            _, cell, _ = indexed_cells[row]
            result[slots[col]].append(cell)
    except Exception:
        counts = {drone_id: 0 for drone_id in drone_ids}
        for _, cell, cell_xy in indexed_cells:
            candidates = sorted(
                (
                    _dist(cell_xy, drone_positions[drone_id]),
                    counts[drone_id],
                    drone_id,
                )
                for drone_id in drone_ids
                if counts[drone_id] < cap
            )
            if not candidates:
                candidates = sorted(
                    (
                        _dist(cell_xy, drone_positions[drone_id]),
                        counts[drone_id],
                        drone_id,
                    )
                    for drone_id in drone_ids
                )
            drone_id = candidates[0][2]
            result[drone_id].append(cell)
            counts[drone_id] += 1

    for drone_id in result:
        result[drone_id] = sorted(result[drone_id], key=_cell_id)
    return result


def order_route(
    start: tuple[float, float],
    cells: list[dict],
) -> list[dict]:
    """Order cells by nearest-neighbor from start, then bounded 2-opt."""
    remaining = _clone_cells(cells)
    route: list[dict] = []
    current = start
    while remaining:
        best_idx = min(
            range(len(remaining)),
            key=lambda idx: (_dist(current, _xy(remaining[idx])), _cell_id(remaining[idx])),
        )
        cell = remaining.pop(best_idx)
        route.append(cell)
        current = _xy(cell)

    n = len(route)
    if n < 4:
        return route

    def route_cost(items: list[dict]) -> float:
        prev = start
        total = 0.0
        for item in items:
            pt = _xy(item)
            total += _dist(prev, pt)
            prev = pt
        return total

    iterations = 0
    limit = n * n
    improved = True
    while improved and iterations < limit:
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n):
                if iterations >= limit:
                    break
                iterations += 1
                candidate = route[:i + 1] + list(reversed(route[i + 1:j + 1])) + route[j + 1:]
                if route_cost(candidate) + 1e-9 < route_cost(route):
                    route = candidate
                    improved = True
                    break
            if improved or iterations >= limit:
                break
    return route


def assign_one(
    cell_pool: list[dict],
    drone_id: str,
    drone_pos: tuple[float, float],
    peers: dict[str, tuple[float, float]],
) -> dict | None:
    """Pick one nearest Voronoi-valid cell, falling back to nearest overall."""
    if not cell_pool:
        return None

    ordered = sorted(
        ((cell, _xy(cell), _cell_id(cell)) for cell in cell_pool),
        key=lambda item: (_dist(drone_pos, item[1]), item[2]),
    )
    other_peers = {pid: pos for pid, pos in peers.items() if pid != drone_id}
    for cell, xy, _ in ordered:
        own_dist = _dist(drone_pos, xy)
        if all(own_dist <= _dist(peer_pos, xy) for peer_pos in other_peers.values()):
            return deepcopy(cell)
    return deepcopy(ordered[0][0])
