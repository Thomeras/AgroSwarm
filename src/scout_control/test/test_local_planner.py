import numpy as np

from scout_control.avoidance.local_planner import (
    BlockedHistoryEntry,
    DynamicMaskDisk,
    LocalGridSnapshot,
    LocalPlanner,
    PlannerPose,
    PlannerResultStatus,
    PlannerTarget,
)


def _make_grid(size: int = 48, resolution_m: float = 0.5) -> LocalGridSnapshot:
    half = (size * resolution_m) / 2.0
    occupancy = np.zeros((size, size), dtype=bool)
    return LocalGridSnapshot(
        occupancy=occupancy,
        resolution_m=resolution_m,
        origin_x=-half,
        origin_y=-half,
    )


def _set_block(grid: LocalGridSnapshot, x: float, y: float) -> None:
    row, col = grid.world_to_grid(x, y)
    grid.occupancy[row, col] = True


def _set_wall(grid: LocalGridSnapshot, x: float, y_min: float, y_max: float, step: float = 0.5) -> None:
    y = y_min
    while y <= y_max + 1e-6:
        _set_block(grid, x, y)
        y += step


def _set_full_vertical_barrier(grid: LocalGridSnapshot, x: float) -> None:
    row_count = grid.occupancy.shape[0]
    for row in range(row_count):
        _, col = grid.world_to_grid(x, grid.grid_to_world(row, 0)[1])
        grid.occupancy[row, col] = True


def test_local_planner_returns_direct_for_open_corridor() -> None:
    planner = LocalPlanner()
    grid = _make_grid()

    result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(5.0, 0.0),
    )

    assert result.status == PlannerResultStatus.DIRECT
    assert result.subgoal_xy == (5.0, 0.0)
    assert len(result.path_xy) == 2
    assert result.corridor_width_m is not None
    assert result.corridor_width_m > 0.0


def test_local_planner_returns_detour_when_direct_line_is_blocked() -> None:
    planner = LocalPlanner()
    grid = _make_grid()
    _set_wall(grid, x=2.0, y_min=-1.0, y_max=1.0)

    result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
    )

    assert result.status == PlannerResultStatus.DETOUR
    assert len(result.path_xy) >= 2
    assert result.subgoal_xy is not None
    assert any(abs(point[1]) > 0.75 for point in result.path_xy[1:-1])


def test_local_planner_returns_no_path_when_wall_closes_map() -> None:
    planner = LocalPlanner()
    grid = _make_grid()
    _set_full_vertical_barrier(grid, x=2.0)

    result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
    )

    assert result.status == PlannerResultStatus.NO_PATH
    assert not result.path_xy
    assert result.failure_reason


def test_local_planner_refuses_stale_or_unready_map() -> None:
    planner = LocalPlanner()
    grid = _make_grid()
    stale_grid = LocalGridSnapshot(
        occupancy=grid.occupancy,
        resolution_m=grid.resolution_m,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        state="STALE_INPUT",
        valid_for_planning=False,
        validity_reason="stale_input_age_2.00s",
    )

    result = planner.plan(
        grid=stale_grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
    )

    assert result.status == PlannerResultStatus.NO_PATH
    assert result.failure_reason == "stale_input_age_2.00s"


def test_local_planner_returns_blocked_when_start_cell_is_occupied() -> None:
    planner = LocalPlanner()
    grid = _make_grid()
    _set_block(grid, 0.0, 0.0)

    result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(5.0, 0.0),
    )

    assert result.status == PlannerResultStatus.BLOCKED
    assert result.reason == "start_cell_blocked"


def test_local_planner_peer_mask_forces_non_direct_result() -> None:
    planner = LocalPlanner()
    grid = _make_grid()

    result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
        peer_drone_mask=[DynamicMaskDisk(x=2.5, y=0.0, radius_m=1.0, hard=True)],
    )

    assert result.status in {PlannerResultStatus.DETOUR, PlannerResultStatus.NO_PATH}
    assert result.status != PlannerResultStatus.DIRECT


def test_local_planner_blocked_history_biases_path_away_from_penalized_side() -> None:
    planner = LocalPlanner()
    grid = _make_grid()
    _set_wall(grid, x=2.0, y_min=-2.0, y_max=2.0)

    base_result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
    )
    history_result = planner.plan(
        grid=grid,
        start=PlannerPose(0.0, 0.0),
        mission_target=PlannerTarget(6.0, 0.0),
        blocked_history=[BlockedHistoryEntry(x=2.5, y=-3.0, radius_m=2.0, score=5.0)],
    )

    assert base_result.status == PlannerResultStatus.DETOUR
    assert history_result.status == PlannerResultStatus.DETOUR
    assert min(point[1] for point in history_result.path_xy) > -3.0
