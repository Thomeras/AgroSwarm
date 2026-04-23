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
