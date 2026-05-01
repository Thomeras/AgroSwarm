"""
task_allocator.py — Pure Python task allocation logic for agricultural swarm

No ROS2 dependencies.  Instantiated and driven by SwarmCoordinator.

Architecture — Variant C:
  Phase 1  Static sector assignment on mission start.
           Grid columns split evenly across N drones.
           Each sector sorted in snake / boustrophedon order.
  Phase 2  Dynamic rebalancing: when a drone finishes its sector and another
           drone still has >3 cells queued, steal ceil(remaining/2) from the
           back of that queue.

Interface (called by SwarmCoordinator):
  tick_ready_watchdog()      — 1 Hz  — start mission when all drones ready
  tick_status_publish()      — 1 Hz  — emit task_status via on_task_status callback
  tick_progress_log()        — 30 s  — log progress
  handle_drone_status(data)  — on each /swarm/drone_status message

Publish callbacks (injected via constructor):
  on_next_cell(drone_id, cell)   — called when a cell is assigned/prefetched
  on_task_status(payload)        — called at 1 Hz with mission progress
  on_mission_complete(payload)   — called once when all cells visited
  on_rth(drone_id)               — called per drone on mission complete
"""

import math
import threading
import time
from collections import defaultdict
from copy import deepcopy
from enum import Enum, auto
from typing import Callable, Literal, Optional

from scout_control.utils import proximity_allocator, route_conflict


# ── Drone state ───────────────────────────────────────────────────────────────
class DroneStatus(Enum):
    WAITING      = auto()   # not yet READY
    WORKING      = auto()   # flying cells
    SECTOR_DONE  = auto()   # finished own sector, may be rebalanced
    TEMP_BLOCKED = auto()   # temporary unavailable for new work
    MISSION_DONE = auto()   # all cells globally done
    RTH          = auto()   # returning home


class DroneRecord:
    """All mutable state for one drone — always accessed under the global lock."""

    def __init__(self, drone_id: str) -> None:
        self.drone_id:        str            = drone_id
        self.status:          DroneStatus    = DroneStatus.WAITING
        self.assigned_cells:  list[dict]     = []   # snake-ordered queue
        self.current_cell:    Optional[dict] = None
        self.prefetched_cell: Optional[dict] = None  # already published, not yet popped
        self.completed_cells: list[str]      = []   # cell ids
        self.blocked_until_s: float          = 0.0
        self.blocked_reason: str             = ""
        self.deferred_cells: int             = 0

    @property
    def queue_remaining(self) -> int:
        return len(self.assigned_cells)

    @property
    def completed(self) -> int:
        return len(self.completed_cells)


# ── Snake-pattern builder ─────────────────────────────────────────────────────
def _snake_pattern(cells: list[dict], by_cols: bool = False) -> list[dict]:
    """
    Sort cells in boustrophedon order.

    by_cols=False  Group by row (y-index), alternate left↔right per row.
                   Best when sector is wider than tall (fewer row-turns).
    by_cols=True   Group by column (x-index), alternate top↕bottom per column.
                   Best when sector is taller than wide (fewer column-turns).
    """
    if by_cols:
        by_col: dict[int, list[dict]] = defaultdict(list)
        for cell in cells:
            x_part, _ = cell["id"].split("_")
            col = int(x_part[1:])
            by_col[col].append(cell)

        result: list[dict] = []
        for i, col in enumerate(sorted(by_col)):
            col_cells = sorted(by_col[col], key=lambda c: int(c["id"].split("_")[1][1:]))
            if i % 2 == 1:          # odd columns: bottom to top
                col_cells = list(reversed(col_cells))
            result.extend(col_cells)
        return result
    else:
        by_row: dict[int, list[dict]] = defaultdict(list)
        for cell in cells:
            _, y_part = cell["id"].split("_")
            row = int(y_part[1:])
            by_row[row].append(cell)

        result: list[dict] = []
        for i, row in enumerate(sorted(by_row)):
            row_cells = sorted(by_row[row], key=lambda c: int(c["id"].split("_")[0][1:]))
            if i % 2 == 1:          # odd rows: right to left
                row_cells = list(reversed(row_cells))
            result.extend(row_cells)
        return result


# ── Pure Python allocator ─────────────────────────────────────────────────────
class TaskAllocator:
    """
    Stateful task allocator — no ROS2 dependencies.

    All I/O goes through constructor-injected callbacks and public tick methods.
    Thread-safe: all mutable state protected by self._lock.
    """

    def __init__(
        self,
        grid_data: dict,
        n_drones: int,
        ready_timeout: float,
        logger,
        on_next_cell: Callable[[str, dict], None],
        on_task_status: Callable[[dict], None],
        on_mission_complete: Callable[[dict], None],
        on_rth: Callable[[str], None],
        nfz_radius: float = 3.0,
        deferred_retry_delay_s: float = 12.0,
        hard_block_cooldown_s: float = 30.0,
        max_deferrals_per_cell: int = 3,
        strategy: Literal["snake", "proximity"] = "proximity",
        cruise_speed_mps: float = 2.0,
        route_conflict_window_s: float = 6.0,
        on_planned_routes: Callable[[dict], None] | None = None,
    ) -> None:
        """
        Parameters
        ----------
        grid_data          Already-parsed field_grid.json dict.
        n_drones           Number of drones expected in the swarm.
        ready_timeout      Seconds to wait for all drones to report READY.
        logger             Object with .info/.warn/.error/.fatal/.debug methods
                           (e.g. rclpy node logger or logging.Logger).
        on_next_cell       (drone_id: str, cell: dict) -> None
        on_task_status     (payload: dict) -> None
        on_mission_complete(payload: dict) -> None
        on_rth             (drone_id: str) -> None
        nfz_radius         Min distance (m) between drones before cell is skipped.
        """
        self._n_drones      = n_drones
        self._ready_timeout = ready_timeout
        self._log           = logger
        self._nfz_radius    = nfz_radius
        self._deferred_retry_delay_s = max(1.0, float(deferred_retry_delay_s))
        self._hard_block_cooldown_s = max(3.0, float(hard_block_cooldown_s))
        self._max_deferrals_per_cell = max(1, int(max_deferrals_per_cell))
        self._strategy = strategy if strategy in {"snake", "proximity"} else "proximity"
        self._cruise_speed_mps = float(cruise_speed_mps)
        self._route_conflict_window_s = float(route_conflict_window_s)
        self._on_planned_routes = on_planned_routes
        self._planned_routes_cache: dict = {}
        self._last_conflicts: list = []
        self._planned_generated_t: float = 0.0

        self._on_next_cell_cb        = on_next_cell
        self._on_task_status_cb      = on_task_status
        self._on_mission_complete_cb = on_mission_complete
        self._on_rth_cb              = on_rth

        self._lock = threading.Lock()

        # ── Grid ──────────────────────────────────────────────────────────────
        self._cols_total:  int   = grid_data["cols"]
        self._rows_total:  int   = grid_data["rows"]
        self._cell_size_m: float = float(grid_data.get("cell_size_m", 1.0))

        self._cells: dict[str, dict] = {}
        for cell in grid_data["cells"]:
            self._cells[cell["id"]] = dict(cell)
            self._cells[cell["id"]]["status"] = "unvisited"

        _est_cells_per_drone = len(self._cells) / max(self._n_drones, 1)
        _est_time_s          = (_est_cells_per_drone * self._cell_size_m) / 2.0

        self._log.info(
            f"Grid loaded: {self._cols_total} cols × {self._rows_total} rows "
            f"= {len(self._cells)} cells | cell_size={self._cell_size_m} m | "
            f"waypoint spacing={self._cell_size_m} m | "
            f"est. {_est_time_s/60:.1f} min/drone at 2 m/s"
        )

        # ── Drone records ─────────────────────────────────────────────────────
        self._drones: dict[str, DroneRecord] = {
            f"drone_{i}": DroneRecord(f"drone_{i}") for i in range(self._n_drones)
        }
        self._drone_positions: dict[str, dict[str, float]] = {}  # drone_id -> {x, y}
        self._rebalance_count: int            = 0
        self._mission_started: bool           = False
        self._mission_done:    bool           = False
        self._mission_start_t: float          = 0.0
        self._ready_deadline:  Optional[float] = None   # set by start_ready_timeout()
        self._deferred_cells: list[dict] = []
        self._deferred_meta: dict[str, dict] = {}

        self._log.info(
            f"TaskAllocator waiting for {self._n_drones} drone(s) to publish READY "
            f"(timeout {self._ready_timeout:.0f}s) — timer starts on /swarm/mission_ready"
        )

    # ── Public tick interface (driven by SwarmCoordinator timers) ─────────────

    def update_drone_position(self, drone_id: str, x: float, y: float) -> None:
        """Update tracker with latest position (from coordinator callbacks)."""
        with self._lock:
            self._drone_positions[drone_id] = {"x": x, "y": y}

    def _nfz_conflict(self, drone_id: str, cell: dict) -> bool:
        """Returns True if the target cell is too close to another drone's current position."""
        for other_id, pos in self._drone_positions.items():
            if other_id == drone_id:
                continue
            dx = cell["x"] - pos["x"]
            dy = cell["y"] - pos["y"]
            dist = math.sqrt(dx**2 + dy**2)
            if dist < self._nfz_radius:
                self._log.debug(
                    f"NFZ CONFLICT: {drone_id} target {cell['id']} is {dist:.2f}m "
                    f"from {other_id} @ ({pos['x']:.1f},{pos['y']:.1f})"
                )
                return True
        return False

    def start_ready_timeout(self) -> None:
        """Start the ready-timeout countdown.

        Must be called by SwarmCoordinator when /swarm/mission_ready is received.
        Before this call tick_ready_watchdog() is a no-op so the timeout cannot
        fire before drones have even been armed.
        """
        with self._lock:
            if self._ready_deadline is not None or self._mission_started:
                return
            self._ready_deadline = time.monotonic() + self._ready_timeout
            self._log.info(
                f"Ready timeout started — mission will begin in ≤{self._ready_timeout:.0f}s "
                "once all drones report READY"
            )

    def tick_ready_watchdog(self) -> None:
        """Call at 1 Hz. Starts mission once all drones ready (or after timeout)."""
        with self._lock:
            if self._mission_started:
                return

            # Don't start until start_ready_timeout() has been called
            # (i.e. /swarm/mission_ready received by SwarmCoordinator).
            if self._ready_deadline is None:
                return

            ready = [d for d in self._drones.values()
                     if d.status != DroneStatus.WAITING]

            all_ready = len(ready) == self._n_drones
            timed_out = time.monotonic() >= self._ready_deadline

            if not (all_ready or (timed_out and ready)):
                if timed_out and not ready:
                    self._log.error(
                        "Ready timeout: no drones responded — mission aborted."
                    )
                return

            if timed_out and not all_ready:
                self._log.warn(
                    f"Ready timeout: starting with {len(ready)}/{self._n_drones} drone(s)"
                )

            self._start_mission(ready)

    def tick_status_publish(self) -> None:
        """Call at 1 Hz. Emits task_status dict via on_task_status callback."""
        if not self._mission_started:
            return

        planned_snapshot = None
        with self._lock:
            self._maintenance_tick(now_s=time.monotonic())
            visited  = sum(1 for c in self._cells.values() if c["status"] == "visited")
            total    = len(self._cells)
            progress = visited / total if total else 0.0

            drones_payload: dict[str, dict] = {}
            for rec in self._drones.values():
                drones_payload[rec.drone_id] = {
                    "current_cell":    rec.current_cell["id"] if rec.current_cell else None,
                    "queue_remaining": rec.queue_remaining,
                    "completed":       rec.completed,
                    "status":          rec.status.name,
                    "blocked_until_s": round(max(0.0, rec.blocked_until_s - time.monotonic()), 2),
                    "blocked_reason":  rec.blocked_reason,
                    "deferred_cells":  rec.deferred_cells,
                }

            payload = {
                "mission_progress": round(progress, 4),
                "drones":           drones_payload,
                "rebalance_count":  self._rebalance_count,
                "total_cells":      total,
                "completed_cells":  visited,
                "cell_size_m":      self._cell_size_m,
                "deferred_cells":   len(self._deferred_cells),
            }
            if self._on_planned_routes and self._planned_routes_cache:
                planned_snapshot = {
                    "routes": {
                        drone_id: [c.get("cell_id", c.get("id")) for c in cells]
                        for drone_id, cells in self._planned_routes_cache.items()
                    },
                    "conflicts": list(self._last_conflicts),
                    "generated_t": self._planned_generated_t,
                }

        self._on_task_status_cb(payload)
        if planned_snapshot:
            try:
                self._on_planned_routes(planned_snapshot)
            except Exception:
                pass

    def tick_progress_log(self) -> None:
        """Call at 30 s. Logs mission progress to logger."""
        if not self._mission_started:
            return
        with self._lock:
            visited = sum(1 for c in self._cells.values() if c["status"] == "visited")
            total   = len(self._cells)
            elapsed = time.monotonic() - self._mission_start_t
            self._log.info(
                f"[PROGRESS] {visited}/{total} cells ({visited/total*100:.1f}%) "
                f"in {elapsed:.0f}s | rebalances={self._rebalance_count}"
            )
            for rec in self._drones.values():
                self._log.info(
                    f"  {rec.drone_id}: status={rec.status.name} "
                    f"done={rec.completed} queued={rec.queue_remaining} "
                    f"current={rec.current_cell['id'] if rec.current_cell else 'none'}"
                )

    def handle_drone_status(self, data: dict) -> None:
        """Call when a parsed /swarm/drone_status JSON dict is received."""
        drone_id = data.get("drone_id")
        status = str(data.get("status", "")).upper()
        normalized = dict(data)
        if status == "NAV_ACTIVE":
            status = "NAVIGATING"
        elif status == "NAV_COMPLETED":
            status = "CELL_COMPLETE"
            if not normalized.get("cell_id"):
                normalized["cell_id"] = normalized.get("target_id", "")
        elif status == "NAV_BLOCKED_SOFT":
            status = "BLOCKED"
            normalized.setdefault("blocked_severity", "SOFT")
        elif status == "NAV_BLOCKED_HARD":
            status = "BLOCKED"
            normalized.setdefault("blocked_severity", "HARD")
            normalized.setdefault("reassign_recommended", True)
        elif status in {"NAV_ABORTED", "DRONE_ABORTED"}:
            status = "ABORT"
        elif status in {"DRONE_BLOCKED", "REPEATED_BLOCKED"}:
            status = "BLOCKED"
            normalized.setdefault("blocked_severity", "HARD")

        if drone_id not in self._drones:
            return

        with self._lock:
            rec = self._drones[drone_id]
            now_s = time.monotonic()

            if status == "READY":
                if rec.status == DroneStatus.WAITING:
                    rec.status = DroneStatus.WORKING
                    rec.blocked_until_s = 0.0
                    rec.blocked_reason = ""
                    self._log.info(f"{drone_id}: READY")
                self._maintenance_tick(now_s=now_s)
                return

            if status == "CELL_COMPLETE":
                if not self._mission_started or self._mission_done:
                    return
                cell_id = normalized.get("cell_id")
                self._on_cell_complete(rec, cell_id)
                self._maintenance_tick(now_s=now_s)
                return

            if status == "CELL_DEFERRED":
                if not self._mission_started or self._mission_done:
                    return
                self._handle_cell_deferred(rec, normalized, now_s=now_s)
                self._maintenance_tick(now_s=now_s)
                return

            if status in {
                "IDLE",
                "TAKEOFF",
                "NAVIGATING",
                "AVOIDANCE_WARN",
                "HOVER",
                "SCAN_360",
                "LOCAL_REPLAN",
                "AVOIDING",
            }:
                if self._mission_started and not self._mission_done:
                    self._set_drone_working(rec)
                    self._maybe_backfill_current_cell(rec, data)
                self._maintenance_tick(now_s=now_s)
                return

            if status == "BLOCKED":
                if not self._mission_started or self._mission_done:
                    return
                self._handle_blocked_status(rec, normalized, now_s=now_s)
                self._maintenance_tick(now_s=now_s)
                return

            if status in {"RTH", "LANDING", "ABORT"}:
                if not self._mission_started or self._mission_done:
                    return
                if rec.current_cell is not None:
                    self._defer_cell(
                        rec.current_cell,
                        reason=f"{status.lower()}_while_active",
                        deferred_by=rec.drone_id,
                        severity="HARD",
                    )
                    rec.current_cell = None
                rec.prefetched_cell = None
                rec.status = DroneStatus.TEMP_BLOCKED
                rec.blocked_until_s = now_s + self._hard_block_cooldown_s
                rec.blocked_reason = status.lower()
                self._maintenance_tick(now_s=now_s)
                return

    # ── Mission start ─────────────────────────────────────────────────────────

    def _start_mission(self, active_drones: list[DroneRecord]) -> None:
        if self._strategy == "snake":
            self._start_mission_snake(active_drones)
        else:
            self._start_mission_proximity(active_drones)

    def _start_mission_snake(self, active_drones: list[DroneRecord]) -> None:
        """Assign snake-ordered sectors and publish first next_cell for each drone."""
        n       = len(active_drones)
        cols_pp = math.ceil(self._cols_total / n)

        all_cells = list(self._cells.values())

        for idx, rec in enumerate(active_drones):
            col_start   = idx * cols_pp
            col_end     = min(col_start + cols_pp, self._cols_total)
            sector_cols = col_end - col_start
            sector_rows = self._rows_total

            by_cols         = sector_rows > sector_cols
            turns           = (sector_cols - 1) if by_cols else (sector_rows - 1)
            direction_label = "columns" if by_cols else "rows"

            sector = [
                c for c in all_cells
                if col_start <= int(c["id"].split("_")[0][1:]) < col_end
            ]
            rec.assigned_cells = _snake_pattern(sector, by_cols=by_cols)
            rec.status         = DroneStatus.WORKING

            self._log.info(
                f"  {rec.drone_id}: {sector_cols}col×{sector_rows}row sector "
                f"→ snake by {direction_label}, {turns} turn(s) "
                f"→ {rec.queue_remaining} cells"
            )

        self._mission_started = True
        self._mission_start_t = time.monotonic()
        self._log.info(
            f"Mission starting with {n} drone(s), {len(self._cells)} cells total"
        )

        self._run_conflict_pass()
        for rec in active_drones:
            self._advance(rec)

    def _start_mission_proximity(self, active_drones: list[DroneRecord]) -> None:
        """Assign proximity-ordered routes and publish first next_cell for each drone."""
        positions: dict[str, tuple[float, float]] = {}
        for rec in active_drones:
            pos = self._drone_positions.get(rec.drone_id, {})
            positions[rec.drone_id] = (
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
            )

        assigned = proximity_allocator.assign_initial(list(self._cells.values()), positions)
        for rec in active_drones:
            rec.assigned_cells = proximity_allocator.order_route(
                positions.get(rec.drone_id, (0.0, 0.0)),
                assigned.get(rec.drone_id, []),
            )
            rec.status = DroneStatus.WORKING
            self._log.info(
                f"  {rec.drone_id}: proximity route → {rec.queue_remaining} cells"
            )

        self._mission_started = True
        self._mission_start_t = time.monotonic()
        self._log.info(
            f"Mission starting with {len(active_drones)} drone(s), "
            f"{len(self._cells)} cells total | strategy=proximity"
        )

        self._run_conflict_pass()
        for rec in active_drones:
            self._advance(rec)

    # ── Cell completion ───────────────────────────────────────────────────────

    def _on_cell_complete(self, rec: DroneRecord, cell_id: Optional[str]) -> None:
        """Mark cell visited, check rebalance, advance drone or declare done."""
        if cell_id and cell_id in self._cells:
            self._cells[cell_id]["status"] = "visited"

        if rec.current_cell and rec.current_cell["id"] == cell_id:
            rec.completed_cells.append(cell_id)
            rec.current_cell = None
        else:
            if cell_id:
                rec.completed_cells.append(cell_id)
            rec.current_cell = None

        self._log.info(
            f"{rec.drone_id}: cell {cell_id} complete "
            f"(done={rec.completed}, queued={rec.queue_remaining})"
        )

        if self._all_cells_done():
            self._on_mission_complete()
            return

        if rec.assigned_cells:
            next_q = rec.assigned_cells[0]
            if (rec.prefetched_cell is not None
                    and next_q["id"] == rec.prefetched_cell["id"]
                    and self._cells.get(next_q["id"], {}).get("status") != "visited"):
                # Fast-path: next cell already sent to drone, just update tracking
                rec.assigned_cells.pop(0)
                rec.current_cell    = next_q
                rec.prefetched_cell = None
                rec.status          = DroneStatus.WORKING
                self._log.debug(
                    f"{rec.drone_id}: tracking prefetched cell {rec.current_cell['id']}"
                )
                self._try_prefetch(rec)
            else:
                self._advance(rec)
        else:
            rec.status = DroneStatus.SECTOR_DONE
            self._log.info(f"{rec.drone_id}: sector done — checking rebalance")
            self._check_rebalance(rec)

    # ── Advancing / prefetching ───────────────────────────────────────────────

    def _advance(self, rec: DroneRecord) -> None:
        """Pop next unvisited cell from queue, invoke on_next_cell, prefetch."""
        if rec.status == DroneStatus.TEMP_BLOCKED:
            return
        while rec.assigned_cells:
            cell = rec.assigned_cells.pop(0)
            if self._cells.get(cell["id"], {}).get("status") == "visited":
                continue

            # Check for NFZ conflict with other drones
            if self._nfz_conflict(rec.drone_id, cell):
                # Put back at the end and skip for now
                rec.assigned_cells.append(cell)
                self._log.info(
                    f"{rec.drone_id}: skipping {cell['id']} (NFZ conflict) — "
                    "re-queued at back"
                )
                # To prevent busy-wait if only one cell remains and it conflicts,
                # we stop here and let the next event (cell complete / status) re-trigger.
                rec.status = DroneStatus.WORKING
                return

            rec.current_cell    = cell
            rec.prefetched_cell = None
            rec.status          = DroneStatus.WORKING
            self._emit_next_cell(rec.drone_id, cell)
            self._try_prefetch(rec)
            return

        # Queue exhausted (all remaining were already visited)
        rec.status = DroneStatus.SECTOR_DONE
        self._check_rebalance(rec)

    def _try_prefetch(self, rec: DroneRecord) -> None:
        """Pre-publish the next queued cell so the drone can queue it locally."""
        if rec.prefetched_cell is not None or not rec.assigned_cells:
            return
        for i, cell in enumerate(rec.assigned_cells):
            if self._cells.get(cell["id"], {}).get("status") != "visited":
                # Prefetch also respects NFZ
                if self._nfz_conflict(rec.drone_id, cell):
                    continue
                rec.prefetched_cell = cell
                self._emit_next_cell(rec.drone_id, cell)
                self._log.debug(f"{rec.drone_id}: prefetch → {cell['id']}")
                return

    def _emit_next_cell(self, drone_id: str, cell: dict) -> None:
        """Invoke on_next_cell callback and log."""
        self._on_next_cell_cb(drone_id, cell)
        self._log.info(
            f"{drone_id} → next_cell: {cell['id']} "
            f"NED({cell['x']:.2f},{cell['y']:.2f})"
        )

    # ── Rebalancing ───────────────────────────────────────────────────────────

    def _check_rebalance(self, finished: DroneRecord) -> None:
        """Steal cells from the drone with the most remaining work."""
        if finished.status == DroneStatus.TEMP_BLOCKED:
            return
        if self._assign_deferred_to(finished):
            self._log.info(f"{finished.drone_id}: resumed via deferred queue")
            self._advance(finished)
            self._run_conflict_pass()
            return

        best_donor: Optional[DroneRecord] = None
        best_count: int = 3   # minimum threshold to trigger steal

        for rec in self._drones.values():
            if rec.drone_id == finished.drone_id:
                continue
            if rec.status == DroneStatus.TEMP_BLOCKED:
                continue
            if rec.queue_remaining > best_count:
                best_count = rec.queue_remaining
                best_donor = rec

        if best_donor is None:
            self._log.info(
                f"{finished.drone_id}: no donor with >3 cells — idle"
            )
            self._run_conflict_pass()
            return

        n_steal   = math.ceil(best_donor.queue_remaining / 2)
        stealable = best_donor.assigned_cells
        stolen    = stealable[-n_steal:]
        best_donor.assigned_cells = stealable[:-n_steal]

        # Invalidate donor prefetch if stolen
        if (best_donor.prefetched_cell is not None
                and best_donor.prefetched_cell in stolen):
            best_donor.prefetched_cell = None

        before_finished          = finished.queue_remaining
        finished.assigned_cells  = stolen + finished.assigned_cells
        finished.prefetched_cell = None
        finished.status          = DroneStatus.WORKING
        self._rebalance_count   += 1

        self._log.info(
            f"Rebalanced: {finished.drone_id} took {n_steal} cells from "
            f"{best_donor.drone_id} "
            f"(donor queue: {best_donor.queue_remaining+n_steal} → {best_donor.queue_remaining}, "
            f"finished drone: {before_finished} → {finished.queue_remaining})"
        )

        self._advance(finished)
        self._run_conflict_pass()

    def _set_drone_working(self, rec: DroneRecord) -> None:
        rec.blocked_until_s = 0.0
        rec.blocked_reason = ""
        if rec.status in {DroneStatus.WAITING, DroneStatus.MISSION_DONE, DroneStatus.RTH}:
            return
        rec.status = DroneStatus.WORKING

    def _maybe_backfill_current_cell(self, rec: DroneRecord, payload: dict) -> None:
        if rec.current_cell is not None:
            return
        cell_id = payload.get("cell_id")
        if isinstance(cell_id, str) and cell_id in self._cells:
            rec.current_cell = self._cells[cell_id]
            return
        target_id = payload.get("target_id")
        if isinstance(target_id, str) and target_id in self._cells:
            rec.current_cell = self._cells[target_id]

    def _infer_blocked_severity(self, payload: dict) -> str:
        severity_raw = str(payload.get("blocked_severity", "")).strip().upper()
        if severity_raw in {"HARD", "SOFT"}:
            return severity_raw
        if bool(payload.get("reassign_recommended", False)):
            return "HARD"
        return "SOFT"

    def _handle_blocked_status(
        self, rec: DroneRecord, payload: dict, *, now_s: float
    ) -> None:
        severity = self._infer_blocked_severity(payload)
        reason = str(payload.get("blocked_reason", "blocked")).strip() or "blocked"
        if severity == "HARD":
            deferred_cell = rec.current_cell
            if deferred_cell is None:
                cell_id = payload.get("cell_id") or payload.get("target_id")
                if isinstance(cell_id, str):
                    deferred_cell = self._cells.get(cell_id)
            if deferred_cell is not None:
                self._defer_cell(
                    deferred_cell,
                    reason=reason,
                    deferred_by=rec.drone_id,
                    severity=severity,
                )
            rec.current_cell = None
            rec.prefetched_cell = None
            rec.status = DroneStatus.TEMP_BLOCKED
            rec.blocked_reason = reason
            rec.blocked_until_s = now_s + self._hard_block_cooldown_s
            self._log.warn(
                f"{rec.drone_id}: HARD BLOCKED ({reason}) "
                f"→ temp blocked {self._hard_block_cooldown_s:.0f}s"
            )
            return
        rec.status = DroneStatus.WORKING
        rec.blocked_reason = reason
        rec.blocked_until_s = now_s + min(5.0, self._deferred_retry_delay_s / 2.0)
        self._log.info(f"{rec.drone_id}: SOFT BLOCKED ({reason}) — keep assignment")

    def _handle_cell_deferred(
        self, rec: DroneRecord, payload: dict, *, now_s: float
    ) -> None:
        cell_id = payload.get("cell_id")
        severity = self._infer_blocked_severity(payload)
        reason = str(payload.get("reason", payload.get("blocked_reason", "cell_deferred")))
        target_cell = None
        if isinstance(cell_id, str):
            target_cell = self._cells.get(cell_id)
        if target_cell is None:
            target_cell = rec.current_cell
        if target_cell is None:
            return
        self._defer_cell(
            target_cell,
            reason=reason,
            deferred_by=rec.drone_id,
            severity=severity,
        )
        if rec.current_cell and rec.current_cell["id"] == target_cell["id"]:
            rec.current_cell = None
        rec.prefetched_cell = None
        if severity == "HARD":
            rec.status = DroneStatus.TEMP_BLOCKED
            rec.blocked_reason = reason
            rec.blocked_until_s = now_s + self._hard_block_cooldown_s

    def _defer_cell(
        self,
        cell: dict,
        *,
        reason: str,
        deferred_by: str,
        severity: str,
    ) -> None:
        cell_id = cell.get("id")
        if not isinstance(cell_id, str) or cell_id not in self._cells:
            return
        if self._cells[cell_id]["status"] == "visited":
            return
        meta = self._deferred_meta.setdefault(
            cell_id,
            {
                "attempts": 0,
                "last_reason": "",
                "last_drone_id": "",
                "severity": "SOFT",
                "next_eligible_s": 0.0,
            },
        )
        meta["attempts"] = int(meta["attempts"]) + 1
        meta["last_reason"] = str(reason)
        meta["last_drone_id"] = str(deferred_by)
        meta["severity"] = str(severity).upper()
        meta["next_eligible_s"] = time.monotonic() + self._deferred_retry_delay_s
        self._cells[cell_id]["status"] = "deferred"

        if not any(existing.get("id") == cell_id for existing in self._deferred_cells):
            self._deferred_cells.append(dict(self._cells[cell_id]))

        rec = self._drones.get(deferred_by)
        if rec is not None:
            rec.deferred_cells += 1
        self._log.warn(
            f"{deferred_by}: CELL_DEFERRED {cell_id} reason={reason} "
            f"severity={severity} attempts={meta['attempts']}"
        )

    def _eligible_for_reassign(self, cell_id: str, *, now_s: float) -> bool:
        meta = self._deferred_meta.get(cell_id)
        if meta is None:
            return True
        if int(meta.get("attempts", 0)) > self._max_deferrals_per_cell:
            return False
        return now_s >= float(meta.get("next_eligible_s", 0.0))

    def _assign_deferred_to(self, rec: DroneRecord) -> bool:
        if rec.status == DroneStatus.TEMP_BLOCKED:
            return False
        now_s = time.monotonic()
        for idx, cell in enumerate(self._deferred_cells):
            cell_id = cell.get("id")
            if not isinstance(cell_id, str):
                continue
            if self._cells.get(cell_id, {}).get("status") == "visited":
                continue
            if rec.current_cell is not None and rec.current_cell.get("id") == cell_id:
                continue
            if any(item.get("id") == cell_id for item in rec.assigned_cells):
                continue
            if not self._eligible_for_reassign(cell_id, now_s=now_s):
                continue
            rec.assigned_cells.insert(0, self._cells[cell_id])
            self._deferred_cells.pop(idx)
            self._cells[cell_id]["status"] = "unvisited"
            self._log.info(f"{rec.drone_id}: deferred {cell_id} re-assigned")
            return True
        return False

    def _maintenance_tick(self, *, now_s: float) -> None:
        for rec in self._drones.values():
            if rec.status != DroneStatus.TEMP_BLOCKED:
                continue
            if rec.blocked_until_s <= now_s:
                rec.blocked_until_s = 0.0
                rec.blocked_reason = ""
                rec.status = DroneStatus.WORKING
                self._log.info(f"{rec.drone_id}: temp block cooldown elapsed")

        for rec in self._drones.values():
            if rec.status == DroneStatus.TEMP_BLOCKED:
                continue
            if self._assign_deferred_to(rec):
                if rec.current_cell is None:
                    self._advance(rec)
            elif (
                self._mission_started
                and not self._mission_done
                and rec.status == DroneStatus.WORKING
                and rec.current_cell is None
                and rec.assigned_cells
            ):
                # Retry advance for drones stuck on NFZ conflict — other drone
                # may have moved away since the last _advance attempt.
                self._advance(rec)

    def _route_snapshot_unlocked(self) -> dict[str, list[dict]]:
        routes: dict[str, list[dict]] = {}
        for drone_id, rec in self._drones.items():
            route: list[dict] = []
            if (
                rec.current_cell is not None
                and self._cells.get(rec.current_cell["id"], {}).get("status") != "visited"
            ):
                route.append(dict(rec.current_cell))
            for cell in rec.assigned_cells:
                if self._cells.get(cell["id"], {}).get("status") != "visited":
                    route.append(dict(cell))
            routes[drone_id] = route
        return routes

    def _run_conflict_pass(self) -> list[dict]:
        """Detect and resolve route conflicts.

        Caller must hold self._lock.
        """
        routes = self._route_snapshot_unlocked()
        start_pos = {
            drone_id: (float(pos.get("x", 0.0)), float(pos.get("y", 0.0)))
            for drone_id, pos in self._drone_positions.items()
        }
        for drone_id in self._drones:
            start_pos.setdefault(drone_id, (0.0, 0.0))

        legs = route_conflict.build_legs(
            routes,
            start_pos,
            self._cruise_speed_mps,
            dwell_s=1.0,
        )
        conflicts = route_conflict.find_conflicts(
            legs,
            nfz_radius=2.0,
            time_window_s=self._route_conflict_window_s,
        )
        resolved, _actions = route_conflict.resolve(
            conflicts,
            routes,
            priority_fn=lambda d: d,
        )
        self._apply_resolved_routes_unlocked(resolved)
        self._planned_routes_cache = deepcopy(resolved)
        self._last_conflicts = list(conflicts)
        self._planned_generated_t = time.time()
        return self._last_conflicts

    def _apply_resolved_routes_unlocked(self, routes: dict[str, list[dict]]) -> None:
        for drone_id, route in routes.items():
            rec = self._drones.get(drone_id)
            if rec is None:
                continue
            route_copy = [dict(cell) for cell in route]
            if rec.current_cell is not None:
                current_id = rec.current_cell.get("id")
                if route_copy and route_copy[0].get("id") == current_id:
                    rec.assigned_cells = route_copy[1:]
                else:
                    rec.assigned_cells = route_copy
            else:
                rec.assigned_cells = route_copy
            if (
                rec.prefetched_cell is not None
                and not any(
                    cell.get("id") == rec.prefetched_cell.get("id")
                    for cell in rec.assigned_cells
                )
            ):
                rec.prefetched_cell = None

    def planned_routes(self) -> dict:
        with self._lock:
            return deepcopy({
                "routes": self._planned_routes_cache,
                "conflicts": self._last_conflicts,
                "generated_t": self._planned_generated_t,
            })

    # ── Mission complete ──────────────────────────────────────────────────────

    def _all_cells_done(self) -> bool:
        return (
            sum(1 for c in self._cells.values() if c["status"] == "visited")
            >= len(self._cells)
        )

    def _on_mission_complete(self) -> None:
        if self._mission_done:
            return
        self._mission_done = True
        elapsed = time.monotonic() - self._mission_start_t

        for rec in self._drones.values():
            rec.status = DroneStatus.MISSION_DONE

        self._log.info(
            f"MISSION COMPLETE — {len(self._cells)} cells in {elapsed:.1f}s"
        )

        self._on_mission_complete_cb({
            "status":          "COMPLETE",
            "total_time_s":    round(elapsed, 2),
            "cells_completed": len(self._cells),
            "cell_size_m":     self._cell_size_m,
            "area_covered_m2": round(len(self._cells) * self._cell_size_m ** 2, 1),
        })

        for rec in self._drones.values():
            rec.status = DroneStatus.RTH
            self._on_rth_cb(rec.drone_id)
            self._log.info(f"RTH request → {rec.drone_id}")
