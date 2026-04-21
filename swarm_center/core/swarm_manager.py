"""
swarm_manager.py — Central state tracker for the swarm

Holds the authoritative record of:
  • all drone telemetry (from MAVLink)
  • each drone's current grid cell
  • cross-drone awareness (who is where, in grid-cell granularity)

Design:
  This class is a pure-Python model — no Qt, no MAVLink. It's updated
  from the UI thread (which receives MAVLink signals) and read by the UI
  widgets. Keeping it free of threading is intentional: all mutation
  happens on the main thread.

  In future milestones this is where cross-drone coordination lives —
  collision avoidance ("don't enter an occupied grid"), task allocation
  feedback, and the data-aggregation pipeline that feeds AI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from core.field_manager import Cell, FieldGrid
from core.mavlink_manager import DroneTelemetry


@dataclass
class DroneRecord:
    """Authoritative state for one drone, as seen by Swarm Center."""
    drone_id: int
    telemetry: DroneTelemetry
    cell: Optional[Cell] = None

    # ── From ROS2 bridge (task_allocator state) ─────────────────────────────
    assigned_cell: Optional[str] = None   # cell_id currently targeted by allocator
    completed: int = 0                     # total cells the drone has completed
    queue_remaining: int = 0               # cells still queued for this drone
    allocator_status: str = "UNKNOWN"      # WAITING/WORKING/SECTOR_DONE/MISSION_DONE/RTH

    @property
    def did(self) -> str:
        return f"drone_{self.drone_id}"


@dataclass
class MissionState:
    """High-level mission progress from scout_ws task_allocator."""
    ready: bool = False                    # /swarm/mission_ready received
    complete: bool = False                 # /swarm/mission_complete received
    progress: float = 0.0                  # 0..1 — visited/total
    total_cells: int = 0
    completed_cells: int = 0
    rebalance_count: int = 0
    setup_status: str = ""                 # latest /field/setup_status text
    field_ready: bool = False              # field_setup_coordinator reached READY_FOR_MISSION
    setup_state: str = ""                  # raw setup state from coordinator


class SwarmManager:
    """
    Central registry: drone_id → DroneRecord.

    Callers can subscribe to changes via `add_listener(fn)`. The listener
    is called with the updated DroneRecord each time state changes.
    """

    def __init__(self, grid: FieldGrid) -> None:
        self._grid = grid
        self._drones: dict[int, DroneRecord] = {}
        self._listeners: list[Callable[[DroneRecord], None]] = []
        self._mission_listeners: list[Callable[[MissionState], None]] = []
        self._mission = MissionState()
        self._selected_drone_id: Optional[int] = None

    # ── Grid ────────────────────────────────────────────────────────────────

    @property
    def grid(self) -> FieldGrid:
        return self._grid

    def set_grid(self, grid: FieldGrid) -> None:
        """Swap the active grid (e.g. user loaded a new field_grid.json)."""
        self._grid = grid
        self._mission = MissionState()
        # Re-classify every drone with the new grid
        for rec in self._drones.values():
            rec.cell = self._grid.cell_at_ned(
                rec.telemetry.x_ned, rec.telemetry.y_ned)
            self._notify(rec)
        self._notify_mission()

    # ── Drones ──────────────────────────────────────────────────────────────

    def drones(self) -> list[DroneRecord]:
        return sorted(self._drones.values(), key=lambda r: r.drone_id)

    def drone(self, drone_id: int) -> Optional[DroneRecord]:
        return self._drones.get(drone_id)

    def update_telemetry(self, telem: DroneTelemetry) -> None:
        """Called from the UI thread whenever MAVLink emits new telemetry."""
        cell = self._grid.cell_at_ned(telem.x_ned, telem.y_ned)
        # Mutate the telemetry object so the UI sees the cell too
        telem.grid_cell = cell.id if cell is not None else None

        rec = self._drones.get(telem.drone_id)
        if rec is None:
            rec = DroneRecord(drone_id=telem.drone_id, telemetry=telem, cell=cell)
            self._drones[telem.drone_id] = rec
        else:
            rec.telemetry = telem
            rec.cell = cell

        self._notify(rec)

    def set_connection(self, drone_id: int, connected: bool) -> None:
        rec = self._drones.get(drone_id)
        if rec is None:
            # Create a stub record so the UI shows the disconnected drone
            from core.mavlink_manager import DroneTelemetry as _T
            stub = _T(drone_id=drone_id, connected=connected)
            rec = DroneRecord(drone_id=drone_id, telemetry=stub, cell=None)
            self._drones[drone_id] = rec
        else:
            rec.telemetry.connected = connected
        self._notify(rec)

    # ── Swarm awareness broadcast ───────────────────────────────────────────

    def other_drone_cells(self, drone_id: int) -> dict[int, Optional[str]]:
        """
        What would be broadcast to `drone_id` about the others.

        Returns {other_drone_id: "x4_y2" or None}.
        This is the grid-level awareness payload — drones need only cell
        granularity for collision avoidance, not exact NED positions.
        """
        out: dict[int, Optional[str]] = {}
        for other_id, rec in self._drones.items():
            if other_id == drone_id:
                continue
            out[other_id] = rec.cell.id if rec.cell is not None else None
        return out

    # ── Listeners ───────────────────────────────────────────────────────────

    def add_listener(self, fn: Callable[[DroneRecord], None]) -> None:
        self._listeners.append(fn)

    def add_mission_listener(self, fn: Callable[[MissionState], None]) -> None:
        self._mission_listeners.append(fn)

    def _notify(self, rec: DroneRecord) -> None:
        for fn in self._listeners:
            try:
                fn(rec)
            except Exception as exc:
                # Listeners must never crash the swarm state machine
                print(f"[swarm_manager] listener error: {exc}")

    def _notify_mission(self) -> None:
        for fn in self._mission_listeners:
            try:
                fn(self._mission)
            except Exception as exc:
                print(f"[swarm_manager] mission listener error: {exc}")

    # ── Bridge handlers — called from UI thread on bridge signals ──────────

    @property
    def mission(self) -> MissionState:
        return self._mission

    def apply_task_status(self, data: dict) -> None:
        """
        Handle an MSG_TASK_STATUS payload from scout_ws.

        Updates:
          • each drone's allocator status (current_cell, completed, queue)
          • field grid cell statuses — cells currently assigned go to
            "hovering", previously completed ones to "visited". We can't
            know from task_status alone when a cell flipped from
            unvisited → visited, so we reconstruct based on the completed
            count per drone and the current_cell pointer.
          • mission progress
        """
        self._mission.progress = float(data.get("mission_progress", 0.0))
        self._mission.total_cells = int(data.get("total_cells", 0))
        self._mission.completed_cells = int(data.get("completed_cells", 0))
        self._mission.rebalance_count = int(data.get("rebalance_count", 0))

        drones = data.get("drones", {})
        currently_targeted: set[str] = set()

        for drone_id_str, info in drones.items():
            # drone_id_str is like "drone_0" — extract the integer
            try:
                num = int(drone_id_str.split("_")[-1])
            except (ValueError, IndexError):
                continue

            rec = self._drones.get(num)
            if rec is None:
                # No MAVLink yet for this drone — stub it so the panel shows it
                from core.mavlink_manager import DroneTelemetry as _T
                stub = _T(drone_id=num)
                rec = DroneRecord(drone_id=num, telemetry=stub)
                self._drones[num] = rec

            rec.assigned_cell = info.get("current_cell")
            rec.completed = int(info.get("completed", 0))
            rec.queue_remaining = int(info.get("queue_remaining", 0))
            rec.allocator_status = str(info.get("status", "UNKNOWN"))

            if rec.assigned_cell:
                currently_targeted.add(rec.assigned_cell)

            self._notify(rec)

        # Cell status update: anything currently targeted is "hovering".
        # Cells not in targets but previously "hovering" become "visited".
        # We don't flip cells to "visited" here proactively because
        # task_status is a snapshot — use drone_status CELL_COMPLETE events
        # for the authoritative transition (see apply_drone_status).
        for cell in self._grid.cells:
            if cell.id in currently_targeted:
                if cell.status == "unvisited":
                    cell.status = "hovering"
            else:
                if cell.status == "hovering":
                    # Drone moved on — mark as visited
                    cell.status = "visited"

        self._notify_mission()

    def apply_drone_status(self, data: dict) -> None:
        """
        Handle an MSG_DRONE_STATUS payload (READY / CELL_COMPLETE).

        CELL_COMPLETE is the authoritative "this cell is done" event — use
        it to flip the grid cell to "visited" regardless of what
        task_status says.
        """
        status = data.get("status")
        if status != "CELL_COMPLETE":
            return
        cell_id = data.get("cell_id")
        if not cell_id:
            return
        cell = self._grid.cell_by_id(cell_id)
        if cell is not None:
            cell.status = "visited"

    def apply_mission_ready(self, data: dict) -> None:
        self._mission.ready = True
        self._notify_mission()

    def apply_mission_complete(self, data: dict) -> None:
        self._mission.complete = True
        # Flip all cells that were "hovering" to "visited" — mission's over
        for cell in self._grid.cells:
            if cell.status == "hovering":
                cell.status = "visited"
        self._notify_mission()

    def apply_setup_status(self, data: dict) -> None:
        text = str(data.get("text", ""))
        state = str(data.get("state", ""))
        self._mission.setup_status = text
        self._mission.setup_state = state
        self._mission.field_ready = state == "READY_FOR_MISSION"
        
        # Extract metadata (corners, pads) for visualisation
        corners = data.get("corners", {})
        if isinstance(corners, dict):
            # Convert dict {label: {x,y,z}} to list of tuples [(x,y), ...]
            pts = []
            for lbl in ["NE", "NW", "SW", "SE"]:
                c = corners.get(lbl)
                if c:
                    pts.append((float(c.get("x", 0)), float(c.get("y", 0))))
            self._grid.corners = pts

        pads = data.get("pads", {})
        if isinstance(pads, dict):
            # Convert {pad_id: {x,y,z}} to list of tuples
            pts = []
            for p in pads.values():
                pts.append((float(p.get("x", 0)), float(p.get("y", 0))))
            self._grid.landing_pads = pts

        self._notify_mission()

    # ── Drone selection ─────────────────────────────────────────────────────

    @property
    def selected_drone_id(self) -> Optional[int]:
        return self._selected_drone_id

    def select_drone(self, drone_id: Optional[int]) -> None:
        self._selected_drone_id = drone_id

    def reset_mission(self) -> None:
        """Called when operator loads a fresh grid or starts a new mission."""
        self._mission = MissionState()
        for cell in self._grid.cells:
            cell.status = "unvisited"
        self._notify_mission()
