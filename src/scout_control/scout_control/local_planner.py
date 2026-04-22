"""Short-horizon local path planner used by obstacle avoidance runtime."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class PlannerResultStatus(str, Enum):
    DIRECT = "DIRECT"
    DETOUR = "DETOUR"
    NO_PATH = "NO_PATH"
    BLOCKED = "BLOCKED"


class LocalPlannerState(str, Enum):
    READY = "READY"
    NO_MAP = "NO_MAP"
    NO_PATH = "NO_PATH"
    DEGRADED = "DEGRADED"


@dataclass(frozen=True)
class PlannerPose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class PlannerTarget:
    x: float
    y: float
    label: str = "mission"


@dataclass(frozen=True)
class BlockedHistoryEntry:
    x: float
    y: float
    radius_m: float = 1.0
    score: float = 1.0


@dataclass(frozen=True)
class DynamicMaskDisk:
    x: float
    y: float
    radius_m: float
    hard: bool = True
    cost: float = 0.0


@dataclass(frozen=True)
class LocalGridSnapshot:
    occupancy: np.ndarray
    resolution_m: float
    origin_x: float
    origin_y: float
    inflation_cost: np.ndarray | None = None
    blocked_cost: np.ndarray | None = None
    unknown_mask: np.ndarray | None = None
    state: str = "TRACKING"
    stamp_s: float = 0.0

    def shape(self) -> tuple[int, int]:
        return tuple(int(v) for v in self.occupancy.shape)

    def is_empty(self) -> bool:
        return self.occupancy.size == 0

    def contains_world(self, x: float, y: float) -> bool:
        row, col = self.world_to_grid(x, y)
        return 0 <= row < self.occupancy.shape[0] and 0 <= col < self.occupancy.shape[1]

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution_m))
        row = int(math.floor((y - self.origin_y) / self.resolution_m))
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (float(col) + 0.5) * self.resolution_m
        y = self.origin_y + (float(row) + 0.5) * self.resolution_m
        return x, y


@dataclass(frozen=True)
class PlanResult:
    status: PlannerResultStatus
    planner_state: LocalPlannerState
    path_xy: tuple[tuple[float, float], ...] = field(default_factory=tuple)
    subgoal_xy: tuple[float, float] | None = None
    reason: str = ""
    failure_reason: str = ""
    corridor_width_m: float | None = None
    path_cost: float = 0.0
    target_label: str = "mission"


@dataclass(frozen=True)
class LocalPlannerConfig:
    planning_horizon_m: float = 12.0
    subgoal_distance_m: float = 10.0
    ring_min_radius_m: float = 8.0
    ring_max_radius_m: float = 12.0
    ring_angle_step_deg: float = 15.0
    drift_angle_step_deg: float = 12.0
    max_drift_angle_deg: float = 24.0
    obstacle_margin_cost: float = 20.0
    blocked_history_weight: float = 10.0
    peer_soft_cost: float = 40.0
    heading_weight: float = 1.0
    turn_weight: float = 0.75
    unknown_penalty: float = 3.0
    blocked_cost_scale: float = 0.1
    peer_cost_scale: float = 0.1
    direct_inflation_cost_limit: float = 12.0
    corridor_sample_step_m: float = 0.2
    blocked_escape_radius_m: float = 1.5


class LocalPlanner:
    def __init__(self, config: LocalPlannerConfig | None = None) -> None:
        self._config = config or LocalPlannerConfig()

    def plan(
        self,
        grid: LocalGridSnapshot | None,
        start: PlannerPose,
        mission_target: PlannerTarget,
        home_target: PlannerTarget | None = None,
        blocked_history: tuple[BlockedHistoryEntry, ...] | list[BlockedHistoryEntry] = (),
        peer_drone_mask: tuple[DynamicMaskDisk, ...] | list[DynamicMaskDisk] = (),
    ) -> PlanResult:
        if grid is None or grid.is_empty():
            return PlanResult(
                status=PlannerResultStatus.NO_PATH,
                planner_state=LocalPlannerState.NO_MAP,
                reason="planner_no_map",
                failure_reason="missing_grid_snapshot",
            )

        if not grid.contains_world(start.x, start.y):
            return PlanResult(
                status=PlannerResultStatus.BLOCKED,
                planner_state=LocalPlannerState.DEGRADED,
                reason="start_outside_grid",
                failure_reason="start_pose_not_covered",
            )

        layers = self._build_layers(grid, blocked_history, peer_drone_mask)
        start_cell = grid.world_to_grid(start.x, start.y)
        if self._is_hard_blocked(start_cell, layers):
            return PlanResult(
                status=PlannerResultStatus.BLOCKED,
                planner_state=LocalPlannerState.DEGRADED,
                reason="start_cell_blocked",
                failure_reason="start_pose_in_forbidden_cell",
            )

        if self._is_locally_trapped(grid, start_cell, layers):
            return PlanResult(
                status=PlannerResultStatus.BLOCKED,
                planner_state=LocalPlannerState.NO_PATH,
                reason="blocked_escape_corridor",
                failure_reason="no_safe_escape_near_start",
            )

        primary_goal = self._select_forward_goal(start, mission_target)
        direct = self._try_direct_path(
            grid=grid,
            layers=layers,
            start=start,
            goal=primary_goal,
            target_label=mission_target.label,
        )
        if direct is not None:
            return direct

        drift = self._try_drift_path(grid, layers, start, mission_target)
        if drift is not None:
            return drift

        detour = self._try_astar_candidates(
            grid=grid,
            layers=layers,
            start=start,
            mission_target=mission_target,
            blocked_history=blocked_history,
        )
        if detour is not None:
            return detour

        if home_target is not None:
            home_detour = self._try_astar_candidates(
                grid=grid,
                layers=layers,
                start=start,
                mission_target=home_target,
                blocked_history=blocked_history,
            )
            if home_detour is not None:
                return PlanResult(
                    status=home_detour.status,
                    planner_state=LocalPlannerState.DEGRADED,
                    path_xy=home_detour.path_xy,
                    subgoal_xy=home_detour.subgoal_xy,
                    reason="fallback_home_target",
                    failure_reason="mission_target_unreachable",
                    corridor_width_m=home_detour.corridor_width_m,
                    path_cost=home_detour.path_cost,
                    target_label=home_target.label,
                )

        return PlanResult(
            status=PlannerResultStatus.NO_PATH,
            planner_state=LocalPlannerState.NO_PATH,
            reason="no_candidate_path",
            failure_reason="direct_drift_and_detour_failed",
        )

    def _build_layers(
        self,
        grid: LocalGridSnapshot,
        blocked_history: tuple[BlockedHistoryEntry, ...] | list[BlockedHistoryEntry],
        peer_drone_mask: tuple[DynamicMaskDisk, ...] | list[DynamicMaskDisk],
    ) -> dict[str, np.ndarray]:
        shape = grid.occupancy.shape
        hard_blocked = np.array(grid.occupancy, dtype=bool, copy=True)
        inflation = np.zeros(shape, dtype=np.float32)
        if grid.inflation_cost is not None:
            inflation += np.asarray(grid.inflation_cost, dtype=np.float32)

        blocked_cost = np.zeros(shape, dtype=np.float32)
        if grid.blocked_cost is not None:
            blocked_cost += np.asarray(grid.blocked_cost, dtype=np.float32)
        blocked_cost += self._rasterize_blocked_history(grid, blocked_history)

        peer_cost = np.zeros(shape, dtype=np.float32)
        if peer_drone_mask:
            peer_hard, peer_soft = self._rasterize_peer_mask(grid, peer_drone_mask)
            hard_blocked |= peer_hard
            peer_cost += peer_soft

        unknown_cost = np.zeros(shape, dtype=np.float32)
        if grid.unknown_mask is not None:
            unknown_cost += (
                np.asarray(grid.unknown_mask, dtype=np.float32) * self._config.unknown_penalty
            )

        return {
            "hard_blocked": hard_blocked,
            "inflation": inflation,
            "blocked_cost": blocked_cost,
            "peer_cost": peer_cost,
            "unknown_cost": unknown_cost,
        }

    def _rasterize_blocked_history(
        self,
        grid: LocalGridSnapshot,
        blocked_history: tuple[BlockedHistoryEntry, ...] | list[BlockedHistoryEntry],
    ) -> np.ndarray:
        result = np.zeros(grid.occupancy.shape, dtype=np.float32)
        if not blocked_history:
            return result

        yy, xx = np.indices(grid.occupancy.shape, dtype=np.float32)
        world_x = grid.origin_x + (xx + 0.5) * grid.resolution_m
        world_y = grid.origin_y + (yy + 0.5) * grid.resolution_m
        for entry in blocked_history:
            radius = max(entry.radius_m, grid.resolution_m)
            dist = np.hypot(world_x - entry.x, world_y - entry.y)
            mask = dist <= radius
            result[mask] += float(entry.score) * self._config.blocked_history_weight
        return result

    def _rasterize_peer_mask(
        self,
        grid: LocalGridSnapshot,
        peer_drone_mask: tuple[DynamicMaskDisk, ...] | list[DynamicMaskDisk],
    ) -> tuple[np.ndarray, np.ndarray]:
        hard = np.zeros(grid.occupancy.shape, dtype=bool)
        soft = np.zeros(grid.occupancy.shape, dtype=np.float32)
        yy, xx = np.indices(grid.occupancy.shape, dtype=np.float32)
        world_x = grid.origin_x + (xx + 0.5) * grid.resolution_m
        world_y = grid.origin_y + (yy + 0.5) * grid.resolution_m
        for disk in peer_drone_mask:
            dist = np.hypot(world_x - disk.x, world_y - disk.y)
            mask = dist <= disk.radius_m
            if disk.hard:
                hard |= mask
            else:
                soft[mask] += max(float(disk.cost), self._config.peer_soft_cost)
        return hard, soft

    def _select_forward_goal(self, start: PlannerPose, target: PlannerTarget) -> PlannerTarget:
        dx = target.x - start.x
        dy = target.y - start.y
        dist = math.hypot(dx, dy)
        if dist <= self._config.planning_horizon_m:
            return target
        scale = self._config.planning_horizon_m / max(dist, 1e-6)
        return PlannerTarget(
            x=start.x + dx * scale,
            y=start.y + dy * scale,
            label=target.label,
        )

    def _try_direct_path(
        self,
        *,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        start: PlannerPose,
        goal: PlannerTarget,
        target_label: str,
    ) -> PlanResult | None:
        clearance = self._line_clearance(grid, layers, (start.x, start.y), (goal.x, goal.y))
        if clearance is None:
            return None
        corridor_width, peak_inflation = clearance
        if peak_inflation > self._config.direct_inflation_cost_limit:
            return None
        return PlanResult(
            status=PlannerResultStatus.DIRECT,
            planner_state=LocalPlannerState.READY,
            path_xy=((start.x, start.y), (goal.x, goal.y)),
            subgoal_xy=(goal.x, goal.y),
            reason="direct_corridor_clear",
            corridor_width_m=corridor_width,
            target_label=target_label,
        )

    def _try_drift_path(
        self,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        start: PlannerPose,
        mission_target: PlannerTarget,
    ) -> PlanResult | None:
        bearing = math.atan2(mission_target.y - start.y, mission_target.x - start.x)
        forward_dist = min(
            self._config.subgoal_distance_m,
            max(grid.resolution_m * 2.0, math.hypot(mission_target.x - start.x, mission_target.y - start.y)),
        )
        candidates: list[tuple[float, tuple[float, float]]] = []
        step = self._config.drift_angle_step_deg
        max_angle = self._config.max_drift_angle_deg
        angle = step
        while angle <= max_angle + 1e-6:
            for sign in (-1.0, 1.0):
                ang = bearing + math.radians(angle * sign)
                candidates.append(
                    (
                        abs(angle),
                        (
                            start.x + forward_dist * math.cos(ang),
                            start.y + forward_dist * math.sin(ang),
                        ),
                    )
                )
            angle += step

        for _, candidate in candidates:
            if not grid.contains_world(candidate[0], candidate[1]):
                continue
            first_leg = self._line_clearance(grid, layers, (start.x, start.y), candidate)
            if first_leg is None:
                continue
            second_leg = self._line_clearance(
                grid,
                layers,
                candidate,
                (mission_target.x, mission_target.y),
            )
            if second_leg is None:
                continue
            corridor_width = min(first_leg[0], second_leg[0])
            return PlanResult(
                status=PlannerResultStatus.DETOUR,
                planner_state=LocalPlannerState.READY,
                path_xy=((start.x, start.y), candidate, (mission_target.x, mission_target.y)),
                subgoal_xy=candidate,
                reason="slight_drift_candidate",
                corridor_width_m=corridor_width,
                target_label=mission_target.label,
            )
        return None

    def _try_astar_candidates(
        self,
        *,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        start: PlannerPose,
        mission_target: PlannerTarget,
        blocked_history: tuple[BlockedHistoryEntry, ...] | list[BlockedHistoryEntry],
    ) -> PlanResult | None:
        candidates = self._build_astar_candidates(grid, start, mission_target)
        start_cell = grid.world_to_grid(start.x, start.y)
        best: tuple[float, tuple[tuple[int, int], ...], tuple[float, float], float] | None = None
        bearing = math.atan2(mission_target.y - start.y, mission_target.x - start.x)

        for candidate in candidates:
            if not grid.contains_world(candidate[0], candidate[1]):
                continue
            if not self._candidate_makes_progress(start, mission_target, candidate):
                continue
            goal_cell = grid.world_to_grid(candidate[0], candidate[1])
            if self._is_hard_blocked(goal_cell, layers):
                continue
            path_cells, path_cost = self._astar(
                grid=grid,
                layers=layers,
                start_cell=start_cell,
                goal_cell=goal_cell,
                mission_bearing=bearing,
            )
            if not path_cells:
                continue
            path_xy = tuple(grid.grid_to_world(row, col) for row, col in path_cells)
            path_xy = self._simplify_path(grid, layers, path_xy)
            corridor_width = self._path_corridor_width(grid, layers, path_xy)
            score = path_cost - corridor_width + self._candidate_alignment_penalty(start, mission_target, candidate)
            if best is None or score < best[0]:
                best = (score, path_cells, candidate, corridor_width)

        if best is None:
            return None

        _, path_cells, candidate, corridor_width = best
        path_xy = tuple(grid.grid_to_world(row, col) for row, col in path_cells)
        path_xy = self._simplify_path(grid, layers, path_xy)
        status = PlannerResultStatus.DETOUR
        reason = "astar_subgoal_path"
        if blocked_history:
            reason = "astar_detour_with_blocked_history"
        return PlanResult(
            status=status,
            planner_state=LocalPlannerState.READY,
            path_xy=path_xy,
            subgoal_xy=candidate,
            reason=reason,
            corridor_width_m=corridor_width,
            path_cost=float(best[0]),
            target_label=mission_target.label,
        )

    def _build_astar_candidates(
        self,
        grid: LocalGridSnapshot,
        start: PlannerPose,
        mission_target: PlannerTarget,
    ) -> list[tuple[float, float]]:
        direct_goal = self._select_forward_goal(start, mission_target)
        bearing = math.atan2(mission_target.y - start.y, mission_target.x - start.x)
        candidates = [(direct_goal.x, direct_goal.y)]

        angle = 0.0
        while angle <= 180.0 + 1e-6:
            for radius in (self._config.ring_min_radius_m, self._config.ring_max_radius_m):
                if angle == 0.0:
                    world = (
                        start.x + radius * math.cos(bearing),
                        start.y + radius * math.sin(bearing),
                    )
                    candidates.append(world)
                    continue
                for sign in (-1.0, 1.0):
                    ang = bearing + math.radians(angle * sign)
                    candidates.append(
                        (
                            start.x + radius * math.cos(ang),
                            start.y + radius * math.sin(ang),
                        )
                    )
            angle += self._config.ring_angle_step_deg
        return candidates

    def _candidate_alignment_penalty(
        self,
        start: PlannerPose,
        mission_target: PlannerTarget,
        candidate: tuple[float, float],
    ) -> float:
        mission_bearing = math.atan2(mission_target.y - start.y, mission_target.x - start.x)
        candidate_bearing = math.atan2(candidate[1] - start.y, candidate[0] - start.x)
        diff = abs(self._wrap_angle(candidate_bearing - mission_bearing))
        return math.degrees(diff) * self._config.heading_weight

    def _candidate_makes_progress(
        self,
        start: PlannerPose,
        mission_target: PlannerTarget,
        candidate: tuple[float, float],
    ) -> bool:
        mission_dx = mission_target.x - start.x
        mission_dy = mission_target.y - start.y
        mission_dist = math.hypot(mission_dx, mission_dy)
        if mission_dist < 1e-6:
            return True

        unit_x = mission_dx / mission_dist
        unit_y = mission_dy / mission_dist
        progress = (candidate[0] - start.x) * unit_x + (candidate[1] - start.y) * unit_y
        candidate_dist = math.hypot(mission_target.x - candidate[0], mission_target.y - candidate[1])
        return progress > 0.25 and candidate_dist < (mission_dist - 0.25)

    def _astar(
        self,
        *,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        mission_bearing: float,
    ) -> tuple[tuple[tuple[int, int], ...], float]:
        if start_cell == goal_cell:
            return (start_cell,), 0.0

        neighbors = (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        )
        open_heap: list[tuple[float, float, tuple[int, int], tuple[int, int] | None]] = []
        heapq.heappush(open_heap, (0.0, 0.0, start_cell, None))
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {}
        g_cost: dict[tuple[int, int], float] = {start_cell: 0.0}
        heading_from: dict[tuple[int, int], float] = {}
        visited: set[tuple[int, int]] = set()

        while open_heap:
            _, current_cost, current, parent = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)
            came_from[current] = parent

            if current == goal_cell:
                return self._reconstruct_path(came_from, goal_cell), current_cost

            current_heading = heading_from.get(current, mission_bearing)
            for dr, dc in neighbors:
                nxt = (current[0] + dr, current[1] + dc)
                if not self._in_bounds(grid, nxt):
                    continue
                if self._is_hard_blocked(nxt, layers):
                    continue

                step_dist = math.sqrt(2.0) if dr != 0 and dc != 0 else 1.0
                step_cost = step_dist
                step_cost += float(layers["inflation"][nxt]) * 0.1
                step_cost += float(layers["blocked_cost"][nxt]) * self._config.blocked_cost_scale
                step_cost += float(layers["peer_cost"][nxt]) * self._config.peer_cost_scale
                step_cost += float(layers["unknown_cost"][nxt]) * 0.1

                step_heading = math.atan2(float(dr), float(dc))
                heading_penalty = abs(self._wrap_angle(step_heading - mission_bearing))
                turn_penalty = abs(self._wrap_angle(step_heading - current_heading))
                step_cost += heading_penalty * self._config.heading_weight
                step_cost += turn_penalty * self._config.turn_weight

                tentative_g = current_cost + step_cost
                if tentative_g >= g_cost.get(nxt, float("inf")):
                    continue

                g_cost[nxt] = tentative_g
                heading_from[nxt] = step_heading
                heuristic = math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                heapq.heappush(open_heap, (tentative_g + heuristic, tentative_g, nxt, current))

        return tuple(), float("inf")

    def _reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int] | None],
        goal_cell: tuple[int, int],
    ) -> tuple[tuple[int, int], ...]:
        path = [goal_cell]
        current = goal_cell
        while came_from.get(current) is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return tuple(path)

    def _simplify_path(
        self,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        path_xy: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[float, float], ...]:
        if len(path_xy) <= 2:
            return path_xy

        simplified = [path_xy[0]]
        anchor_idx = 0
        probe_idx = 2
        while probe_idx < len(path_xy):
            if self._line_clearance(grid, layers, path_xy[anchor_idx], path_xy[probe_idx]) is None:
                simplified.append(path_xy[probe_idx - 1])
                anchor_idx = probe_idx - 1
            probe_idx += 1
        simplified.append(path_xy[-1])
        deduped: list[tuple[float, float]] = []
        for point in simplified:
            if not deduped or point != deduped[-1]:
                deduped.append(point)
        return tuple(deduped)

    def _line_clearance(
        self,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
    ) -> tuple[float, float] | None:
        dx = end_xy[0] - start_xy[0]
        dy = end_xy[1] - start_xy[1]
        dist = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(dist / max(grid.resolution_m, self._config.corridor_sample_step_m))))
        min_clearance = float("inf")
        max_inflation = 0.0

        for idx in range(steps + 1):
            t = float(idx) / float(steps)
            wx = start_xy[0] + dx * t
            wy = start_xy[1] + dy * t
            if not grid.contains_world(wx, wy):
                return None
            cell = grid.world_to_grid(wx, wy)
            if self._is_hard_blocked(cell, layers):
                return None
            max_inflation = max(max_inflation, float(layers["inflation"][cell]))
            clearance = self._approx_clearance(grid, layers["hard_blocked"], cell)
            min_clearance = min(min_clearance, clearance)

        return min_clearance, max_inflation

    def _approx_clearance(
        self,
        grid: LocalGridSnapshot,
        hard_blocked: np.ndarray,
        cell: tuple[int, int],
    ) -> float:
        max_radius_cells = max(1, int(math.ceil(self._config.planning_horizon_m / grid.resolution_m)))
        for radius in range(1, max_radius_cells + 1):
            r0 = max(0, cell[0] - radius)
            r1 = min(hard_blocked.shape[0], cell[0] + radius + 1)
            c0 = max(0, cell[1] - radius)
            c1 = min(hard_blocked.shape[1], cell[1] + radius + 1)
            patch = hard_blocked[r0:r1, c0:c1]
            if np.any(patch):
                return float(radius) * grid.resolution_m
        return float(max_radius_cells) * grid.resolution_m

    def _path_corridor_width(
        self,
        grid: LocalGridSnapshot,
        layers: dict[str, np.ndarray],
        path_xy: tuple[tuple[float, float], ...],
    ) -> float:
        if not path_xy:
            return 0.0
        min_clearance = float("inf")
        for point in path_xy:
            cell = grid.world_to_grid(point[0], point[1])
            min_clearance = min(
                min_clearance,
                self._approx_clearance(grid, layers["hard_blocked"], cell),
            )
        return 0.0 if not math.isfinite(min_clearance) else min_clearance

    def _is_locally_trapped(
        self,
        grid: LocalGridSnapshot,
        start_cell: tuple[int, int],
        layers: dict[str, np.ndarray],
    ) -> bool:
        radius_cells = max(1, int(math.ceil(self._config.blocked_escape_radius_m / grid.resolution_m)))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                cell = (start_cell[0] + dr, start_cell[1] + dc)
                if not self._in_bounds(grid, cell):
                    continue
                if not self._is_hard_blocked(cell, layers):
                    return False
        return True

    def _is_hard_blocked(self, cell: tuple[int, int], layers: dict[str, np.ndarray]) -> bool:
        return bool(layers["hard_blocked"][cell])

    def _in_bounds(self, grid: LocalGridSnapshot, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < grid.occupancy.shape[0] and 0 <= cell[1] < grid.occupancy.shape[1]

    def _wrap_angle(self, angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
