"""Planned-route conflict detection and lightweight resolution."""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Literal, NamedTuple, TypedDict


class RouteLeg(NamedTuple):
    drone_id: str
    cell_id: str
    x: float
    y: float
    t_enter: float
    t_exit: float


class Conflict(TypedDict):
    a: str
    b: str
    cell_a: str
    cell_b: str
    type: Literal["cell", "crossing"]
    t_overlap: float


class ResolutionAction(TypedDict, total=False):
    kind: Literal["WAIT", "SWAP"]
    drone_id: str
    t_s: float
    swap_indices: tuple[int, int]


def _cell_id(cell: dict) -> str:
    return str(cell.get("cell_id", cell.get("id", "")))


def _xy(cell: dict) -> tuple[float, float]:
    center = cell.get("center")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        return (float(center[0]), float(center[1]))
    return (float(cell.get("x", 0.0)), float(cell.get("y", 0.0)))


def _start_xy(start_pos: dict, drone_id: str) -> tuple[float, float]:
    raw = start_pos.get(drone_id, (0.0, 0.0))
    if isinstance(raw, dict):
        return (float(raw.get("x", 0.0)), float(raw.get("y", 0.0)))
    return (float(raw[0]), float(raw[1]))


def build_legs(routes, start_pos, cruise_speed, dwell_s) -> list[RouteLeg]:
    """Build cumulative timing legs for every route cell."""
    speed = float(cruise_speed) if cruise_speed and cruise_speed > 0 else 1.0
    dwell = max(0.0, float(dwell_s))
    legs: list[RouteLeg] = []
    for drone_id in sorted(routes):
        t = 0.0
        prev = _start_xy(start_pos, drone_id)
        for cell in routes.get(drone_id, []):
            xy = _xy(cell)
            t += math.hypot(xy[0] - prev[0], xy[1] - prev[1]) / speed
            enter = t
            exit_t = enter + dwell
            legs.append(RouteLeg(drone_id, _cell_id(cell), xy[0], xy[1], enter, exit_t))
            t = exit_t
            prev = xy
    return legs


def find_conflicts(legs, nfz_radius, time_window_s) -> list[Conflict]:
    """Find cell-overlap and near-simultaneous crossing conflicts."""
    radius = max(0.0, float(nfz_radius))
    window = max(0.0, float(time_window_s))
    conflicts: list[Conflict] = []
    ordered = sorted(legs, key=lambda leg: (leg.drone_id, leg.cell_id, leg.t_enter))
    for i, a in enumerate(ordered):
        for b in ordered[i + 1:]:
            if a.drone_id == b.drone_id:
                continue
            if math.hypot(a.x - b.x, a.y - b.y) > radius:
                continue
            overlap = min(a.t_exit, b.t_exit) - max(a.t_enter, b.t_enter)
            if overlap > 0.0:
                conflicts.append({
                    "a": a.drone_id,
                    "b": b.drone_id,
                    "cell_a": a.cell_id,
                    "cell_b": b.cell_id,
                    "type": "cell",
                    "t_overlap": round(overlap, 6),
                })
            elif abs(a.t_enter - b.t_enter) <= window:
                conflicts.append({
                    "a": a.drone_id,
                    "b": b.drone_id,
                    "cell_a": a.cell_id,
                    "cell_b": b.cell_id,
                    "type": "crossing",
                    "t_overlap": round(abs(a.t_enter - b.t_enter), 6),
                })
    return sorted(
        conflicts,
        key=lambda c: (
            min(c["a"], c["b"]),
            max(c["a"], c["b"]),
            c["cell_a"],
            c["cell_b"],
            c["type"],
        ),
    )


def resolve(conflicts, routes, priority_fn) -> tuple[dict, list[ResolutionAction]]:
    """Return a resolved route copy and a deterministic action list."""
    resolved = deepcopy(routes)
    actions: list[ResolutionAction] = []
    seen_swaps: set[tuple[frozenset[str], str]] = set()

    for conflict in conflicts:
        a = conflict["a"]
        b = conflict["b"]
        lower = max((a, b), key=lambda drone_id: (priority_fn(drone_id), drone_id))
        if conflict["cell_a"] == conflict["cell_b"]:
            key = (frozenset({a, b}), conflict["cell_a"])
            if key in seen_swaps:
                continue
            seen_swaps.add(key)
            idx_a = _route_index(resolved.get(a, []), conflict["cell_a"])
            idx_b = _route_index(resolved.get(b, []), conflict["cell_b"])
            if idx_a is None or idx_b is None:
                continue
            suffix_a = resolved[a][idx_a:]
            suffix_b = resolved[b][idx_b:]
            resolved[a] = resolved[a][:idx_a] + suffix_b
            resolved[b] = resolved[b][:idx_b] + suffix_a
            actions.append({
                "kind": "SWAP",
                "drone_id": lower,
                "swap_indices": (idx_a, idx_b),
            })
        else:
            actions.append({
                "kind": "WAIT",
                "drone_id": lower,
                "t_s": float(conflict.get("t_overlap", 0.0)),
            })
    return resolved, actions


def _route_index(route: list[dict], cell_id: str) -> int | None:
    for idx, cell in enumerate(route):
        if _cell_id(cell) == cell_id:
            return idx
    return None
