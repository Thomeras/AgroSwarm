from scout_control.utils.task_allocator import DroneStatus, TaskAllocator


class _Logger:
    def info(self, _msg: str) -> None:
        return

    def warn(self, _msg: str) -> None:
        return

    def error(self, _msg: str) -> None:
        return

    def fatal(self, _msg: str) -> None:
        return

    def debug(self, _msg: str) -> None:
        return


def _grid(cols: int = 4, rows: int = 2) -> dict:
    cells = []
    for x in range(cols):
        for y in range(rows):
            cells.append({"id": f"x{x}_y{y}", "x": float(x), "y": float(y)})
    return {"cols": cols, "rows": rows, "cell_size_m": 1.0, "cells": cells}


def _mk_allocator() -> tuple[TaskAllocator, list[tuple[str, str]], list[dict], list[dict], list[str]]:
    emitted_next: list[tuple[str, str]] = []
    task_status: list[dict] = []
    mission_complete: list[dict] = []
    rth_calls: list[str] = []

    alloc = TaskAllocator(
        grid_data=_grid(),
        n_drones=2,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda drone_id, cell: emitted_next.append((drone_id, cell["id"])),
        on_task_status=lambda payload: task_status.append(payload),
        on_mission_complete=lambda payload: mission_complete.append(payload),
        on_rth=lambda drone_id: rth_calls.append(drone_id),
        deferred_retry_delay_s=0.0,
        hard_block_cooldown_s=0.2,
        max_deferrals_per_cell=3,
        strategy="snake",
    )
    return alloc, emitted_next, task_status, mission_complete, rth_calls


def _start_mission(alloc: TaskAllocator) -> None:
    alloc.start_ready_timeout()
    alloc.handle_drone_status({"drone_id": "drone_0", "status": "READY"})
    alloc.handle_drone_status({"drone_id": "drone_1", "status": "READY"})
    alloc.tick_ready_watchdog()


def test_hard_blocked_defers_current_cell_and_marks_temp_blocked() -> None:
    alloc, emitted_next, _, _, _ = _mk_allocator()
    _start_mission(alloc)
    assert emitted_next

    current = alloc._drones["drone_0"].current_cell
    assert current is not None
    cell_id = current["id"]

    alloc.handle_drone_status(
        {
            "drone_id": "drone_0",
            "status": "BLOCKED",
            "blocked_severity": "HARD",
            "blocked_reason": "no_local_path",
        }
    )

    rec0 = alloc._drones["drone_0"]
    assert rec0.status == DroneStatus.TEMP_BLOCKED
    assert rec0.current_cell is None
    assert len(alloc._deferred_cells) == 1
    assert alloc._deferred_cells[0]["id"] == cell_id
    assert alloc._deferred_meta[cell_id]["last_reason"] == "no_local_path"


def test_cell_deferred_event_requeues_cell_to_other_drone_queue() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)

    rec0 = alloc._drones["drone_0"]
    rec1 = alloc._drones["drone_1"]
    assert rec0.current_cell is not None
    deferred_id = rec0.current_cell["id"]

    alloc.handle_drone_status(
        {
            "drone_id": "drone_0",
            "status": "CELL_DEFERRED",
            "cell_id": deferred_id,
            "blocked_severity": "HARD",
            "reason": "peer_conflict",
        }
    )
    alloc._deferred_meta[deferred_id]["next_eligible_s"] = 0.0
    alloc.tick_status_publish()

    assert any(cell["id"] == deferred_id for cell in rec1.assigned_cells)
    assert rec0.status == DroneStatus.TEMP_BLOCKED


def test_temp_blocked_drone_recovers_after_cooldown() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)

    alloc.handle_drone_status(
        {
            "drone_id": "drone_0",
            "status": "BLOCKED",
            "blocked_severity": "HARD",
            "blocked_reason": "repeated_no_path",
        }
    )
    assert alloc._drones["drone_0"].status == DroneStatus.TEMP_BLOCKED

    alloc._drones["drone_0"].blocked_until_s = 0.0
    alloc.tick_status_publish()

    assert alloc._drones["drone_0"].status == DroneStatus.WORKING


def test_nav_blocked_hard_event_is_treated_as_hard_blocked() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)
    rec0 = alloc._drones["drone_0"]
    assert rec0.current_cell is not None
    current_id = rec0.current_cell["id"]

    alloc.handle_drone_status(
        {
            "drone_id": "drone_0",
            "status": "NAV_BLOCKED_HARD",
            "blocked_reason": "no_local_path",
        }
    )

    assert rec0.status == DroneStatus.TEMP_BLOCKED
    assert rec0.current_cell is None
    assert alloc._deferred_meta[current_id]["severity"] == "HARD"


def test_nfz_conflict_drone_advances_after_maintenance_tick() -> None:
    """Drone stuck on NFZ conflict gets next_cell once the blocking drone moves away.

    Scenario: 6-col grid, drone_0 has 3 cells (x0,x1,x2). Mission starts with
    no NFZ → drone_0 gets x0 + prefetch x1. After completing x0 (fast-path to x1),
    drone_1 parks at x=2 blocking x2. drone_0 completes x1 → _advance → NFZ conflict
    on x2 → stuck. After drone_1 moves, maintenance_tick must retry _advance.
    """
    emitted_next: list[tuple[str, str]] = []
    task_status: list[dict] = []
    mission_complete: list[dict] = []
    rth_calls: list[str] = []

    grid = {"cols": 6, "rows": 1, "cell_size_m": 1.0, "cells": [
        {"id": f"x{x}_y0", "x": float(x), "y": 0.0} for x in range(6)
    ]}
    alloc = TaskAllocator(
        grid_data=grid,
        n_drones=2,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda drone_id, cell: emitted_next.append((drone_id, cell["id"])),
        on_task_status=lambda payload: task_status.append(payload),
        on_mission_complete=lambda payload: mission_complete.append(payload),
        on_rth=lambda drone_id: rth_calls.append(drone_id),
        nfz_radius=1.5,  # cells 1m apart; adjacent = conflict
        deferred_retry_delay_s=0.0,
        hard_block_cooldown_s=0.0,
        max_deferrals_per_cell=3,
        strategy="snake",
    )
    alloc.start_ready_timeout()
    alloc.handle_drone_status({"drone_id": "drone_0", "status": "READY"})
    alloc.handle_drone_status({"drone_id": "drone_1", "status": "READY"})
    alloc.tick_ready_watchdog()

    rec0 = alloc._drones["drone_0"]
    # drone_0 got x0_y0 (current) + x1_y0 (prefetch) at mission start (no NFZ yet)
    assert rec0.current_cell is not None

    # Park drone_1 at x=2 BEFORE x0 completion so the x2_y0 prefetch gets blocked
    alloc.update_drone_position("drone_1", 2.0, 0.0)

    # drone_0 completes x0_y0 → fast-path: current = x1_y0, try_prefetch x2_y0 → NFZ blocked
    alloc.handle_drone_status({"drone_id": "drone_0", "status": "CELL_COMPLETE", "cell_id": "x0_y0"})
    assert rec0.current_cell is not None  # x1_y0 via fast-path

    # drone_0 completes x1_y0 → _advance → x2_y0 has NFZ conflict → stuck
    alloc.handle_drone_status({"drone_id": "drone_0", "status": "CELL_COMPLETE", "cell_id": "x1_y0"})

    emitted_d0_before = [c for d, c in emitted_next if d == "drone_0"]
    # drone_0 must have no current_cell (stuck in NFZ)
    assert rec0.current_cell is None, "drone_0 should be stuck with no current_cell"
    # Only x0 and x1 should have been emitted so far
    assert "x2_y0" not in emitted_d0_before

    # drone_1 moves away — NFZ clears
    alloc.update_drone_position("drone_1", 10.0, 0.0)

    import time
    alloc._maintenance_tick(now_s=time.monotonic())

    emitted_d0_after = [c for d, c in emitted_next if d == "drone_0"]
    assert "x2_y0" in emitted_d0_after, (
        "drone_0 should have received x2_y0 after NFZ cleared via maintenance_tick"
    )


def test_nav_completed_without_cell_id_uses_target_id() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)
    rec0 = alloc._drones["drone_0"]
    assert rec0.current_cell is not None
    cell_id = rec0.current_cell["id"]

    alloc.handle_drone_status(
        {
            "drone_id": "drone_0",
            "status": "NAV_COMPLETED",
            "target_id": cell_id,
        }
    )

    assert cell_id in rec0.completed_cells


def test_snake_backward_compat_assigns_existing_sectors() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)
    rec0 = alloc._drones["drone_0"]
    assert rec0.current_cell is not None
    assert rec0.current_cell["id"] == "x0_y0"


def test_proximity_assigns_all_cells_and_generates_plan() -> None:
    emitted_next: list[tuple[str, str]] = []
    alloc = TaskAllocator(
        grid_data=_grid(cols=3, rows=2),
        n_drones=2,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda drone_id, cell: emitted_next.append((drone_id, cell["id"])),
        on_task_status=lambda _payload: None,
        on_mission_complete=lambda _payload: None,
        on_rth=lambda _drone_id: None,
        nfz_radius=0.0,
        strategy="proximity",
    )
    alloc.update_drone_position("drone_0", 0.0, 0.0)
    alloc.update_drone_position("drone_1", 2.0, 1.0)
    _start_mission(alloc)

    planned = alloc.planned_routes()
    planned_ids = [
        cell["id"]
        for route in planned["routes"].values()
        for cell in route
    ]
    assert sorted(planned_ids) == sorted(c["id"] for c in _grid(cols=3, rows=2)["cells"])
    assert emitted_next


def test_planned_routes_returns_deepcopy() -> None:
    alloc, _, _, _, _ = _mk_allocator()
    _start_mission(alloc)
    first = alloc.planned_routes()
    first["routes"]["drone_0"].append({"id": "mutated"})
    second = alloc.planned_routes()
    assert all(c.get("id") != "mutated" for c in second["routes"]["drone_0"])


def test_planned_routes_callback_shape() -> None:
    callbacks: list[dict] = []
    alloc = TaskAllocator(
        grid_data=_grid(cols=2, rows=1),
        n_drones=2,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda _drone_id, _cell: None,
        on_task_status=lambda _payload: None,
        on_mission_complete=lambda _payload: None,
        on_rth=lambda _drone_id: None,
        strategy="proximity",
        on_planned_routes=lambda payload: callbacks.append(payload),
    )
    _start_mission(alloc)
    alloc.tick_status_publish()
    assert callbacks
    assert set(callbacks[-1]) == {
        "routes",
        "route_points",
        "conflicts",
        "generated_t",
        "dynamic_blocked_zones",
    }
    assert isinstance(callbacks[-1]["routes"], dict)
    assert isinstance(callbacks[-1]["conflicts"], list)


def test_conflict_pass_runs_after_start() -> None:
    alloc = TaskAllocator(
        grid_data={"cols": 1, "rows": 2, "cell_size_m": 1.0, "cells": [
            {"id": "x0_y0", "x": 0.0, "y": 0.0},
            {"id": "x0_y1", "x": 0.5, "y": 0.0},
        ]},
        n_drones=2,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda _drone_id, _cell: None,
        on_task_status=lambda _payload: None,
        on_mission_complete=lambda _payload: None,
        on_rth=lambda _drone_id: None,
        strategy="proximity",
        route_conflict_window_s=10.0,
    )
    _start_mission(alloc)
    assert alloc.planned_routes()["generated_t"] > 0.0


def test_blocked_status_creates_dynamic_zone_and_replans_route() -> None:
    grid = {"cols": 3, "rows": 1, "cell_size_m": 0.25, "cells": [
        {"id": "x0_y0", "x": 0.0, "y": 0.0},
        {"id": "x1_y0", "x": 1.0, "y": 0.0},
        {"id": "x2_y0", "x": 2.0, "y": 0.0},
    ]}
    alloc = TaskAllocator(
        grid_data=grid,
        n_drones=1,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda _drone_id, _cell: None,
        on_task_status=lambda _payload: None,
        on_mission_complete=lambda _payload: None,
        on_rth=lambda _drone_id: None,
        strategy="proximity",
        dynamic_obstacle_radius_m=0.6,
        dynamic_obstacle_ttl_s=60.0,
    )
    _start_mission(alloc)

    alloc.handle_drone_status({
        "drone_id": "drone_0",
        "status": "BLOCKED",
        "blocked_severity": "SOFT",
        "blocked_reason": "tree_detected",
        "drone_ned": [1.0, 0.0, -5.0],
    })

    planned = alloc.planned_routes()
    planned_ids = [
        cell["id"]
        for route in planned["routes"].values()
        for cell in route
    ]
    assert "x1_y0" not in planned_ids
    assert any(cell["id"] == "x1_y0" for cell in alloc._deferred_cells)
    assert planned["dynamic_blocked_zones"]


def test_dynamic_zone_inserts_detour_for_crossing_route() -> None:
    grid = {"cols": 2, "rows": 1, "cell_size_m": 0.5, "cells": [
        {"id": "x0_y0", "x": 0.0, "y": 0.0},
        {"id": "x1_y0", "x": 4.0, "y": 0.0},
    ]}
    alloc = TaskAllocator(
        grid_data=grid,
        n_drones=1,
        ready_timeout=0.01,
        logger=_Logger(),
        on_next_cell=lambda _drone_id, _cell: None,
        on_task_status=lambda _payload: None,
        on_mission_complete=lambda _payload: None,
        on_rth=lambda _drone_id: None,
        strategy="proximity",
        dynamic_obstacle_radius_m=0.6,
        dynamic_obstacle_ttl_s=60.0,
    )
    alloc.update_drone_position("drone_0", 0.0, 0.0)
    _start_mission(alloc)

    rec = alloc._drones["drone_0"]
    rec.current_cell = None
    rec.assigned_cells = [alloc._cells["x1_y0"]]
    alloc._dynamic_blocked_zones["tree"] = {
        "id": "tree",
        "x": 2.0,
        "y": 0.0,
        "radius_m": 0.6,
        "source_drone": "drone_0",
        "reason": "test",
        "expires_s": 9999999999.0,
    }
    alloc._run_conflict_pass()

    planned = alloc.planned_routes()["routes"]["drone_0"]
    assert planned[0]["kind"] == "detour_waypoint"
    assert planned[1]["kind"] == "detour_waypoint"
    assert planned[-1]["id"] == "x1_y0"
    assert abs(float(planned[0]["y"])) > 0.5
