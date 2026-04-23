"""
field_commander.py — Interactive grid-based drone commander

Loads ~/scout_ws/field_grid.json, shows a curses terminal UI with the full
field grid, and lets the user navigate cells and send the drone to any of
them. Cell statuses (unvisited / hovering / visited) are tracked in real-time
from /fmu/out/vehicle_local_position_v1 and saved back to JSON on exit.

Usage:
  ros2 run scout_control field_commander
"""

import curses
import json
import math
import os
import threading
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

# ── Paths ─────────────────────────────────────────────────────────────────────
from scout_control.utils.paths import GRID_FILE, PERIMETER_FILE

# ── QoS (must match PX4) ─────────────────────────────────────────────────────
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# RTH target arrives latched from home_manager — match its QoS
QOS_RTH_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Swarm event messages (requests, confirmations) — ephemeral
QOS_SWARM = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── Flight state machine ──────────────────────────────────────────────────────
class FlightPhase(Enum):
    IDLE    = auto()   # on ground / not armed — VSP holds position
    TAKEOFF = auto()   # rising to target altitude (XY fixed)
    ROTATE  = auto()   # holding altitude, yawing toward target
    CRUISE  = auto()   # flying horizontally toward target (Z fixed)
    HOVER   = auto()   # at target cell, holding position
    RTH     = auto()   # returning home: descending to pad, then land

# ── Constants ─────────────────────────────────────────────────────────────────
DT            = 0.1   # s — timer period (10 Hz)
CRUISE_SPEED  = 2.0   # m/s — VSP movement speed
REACH_DIST    = 0.5   # m — cell considered reached
ALT_TOL       = 0.4   # m — altitude error to consider takeoff complete
ROTATE_TICKS  = 10    # minimum ticks before checking yaw-settled (1 s at 10 Hz)
MAX_ROTATE_TICKS = 80 # safety cap — exit ROTATE after 8 s even if yaw not settled
YAW_RATE      = math.radians(35)   # rad/s — gradual yaw VSP speed (~35 °/s)
YAW_TOL       = math.radians(8)    # rad  — yaw considered settled within 8°
DEFAULT_ALT   = 5.0   # m — fallback if perimeter JSON missing
ARM_TICKS     = 10    # send this many setpoints before arming
RTH_HOVER_Z   = -0.5  # m NED — hover height above pad before issuing land cmd
RTH_SPEED     = 0.8   # m/s — descent rate during RTH phase

# Cell status → display char
STATUS_CHAR: dict[str, str] = {
    "unvisited": ".",
    "hovering":  "H",
    "visited":   "X",
    "unknown":   "?",
}

# ── Curses colour pairs ───────────────────────────────────────────────────────
# 1 normal  2 cursor(selected)  3 title  4 hover(drone)  5 visited  6 dim  7 accent
CP_NORMAL  = 1
CP_CURSOR  = 2
CP_TITLE   = 3
CP_HOVER   = 4
CP_VISITED = 5
CP_DIM     = 6
CP_ACCENT  = 7


def _setup_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(CP_NORMAL,  curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_CURSOR,  curses.COLOR_BLACK,  curses.COLOR_CYAN)
    curses.init_pair(CP_TITLE,   curses.COLOR_YELLOW, -1)
    curses.init_pair(CP_HOVER,   curses.COLOR_BLACK,  curses.COLOR_GREEN)
    curses.init_pair(CP_VISITED, curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_DIM,     curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_ACCENT,  curses.COLOR_CYAN,   -1)


# ── Node ──────────────────────────────────────────────────────────────────────
class FieldCommander(Node):

    def __init__(self) -> None:
        super().__init__("field_commander")

        # ── Load grid ─────────────────────────────────────────────────────────
        grid_data = self._load_json(GRID_FILE)
        if grid_data is None:
            raise RuntimeError(
                f"Cannot load {GRID_FILE} — run grid_generator first."
            )

        self._cell_size: float = grid_data["cell_size_m"]
        self._cols: int        = grid_data["cols"]
        self._rows: int        = grid_data["rows"]

        # cells as plain dicts, mutated in-place for status
        self._cells: list[dict] = grid_data["cells"]

        # fast lookups
        self._cell_by_id:  dict[str, dict]        = {c["id"]: c for c in self._cells}
        self._cell_by_cr:  dict[tuple, dict]       = {}
        for c in self._cells:
            col, row = self._parse_id(c["id"])
            c["_col"] = col
            c["_row"] = row
            self._cell_by_cr[(col, row)] = c

        # ── Load altitude from perimeter JSON ──────────────────────────────
        pdata = self._load_json(PERIMETER_FILE)
        self._altitude: float = (
            float(pdata["altitude_m"]) if pdata and "altitude_m" in pdata
            else DEFAULT_ALT
        )
        self._target_z: float = -self._altitude  # NED z

        # ── Shared mutable state (guarded by _lock) ────────────────────────
        self._lock = threading.Lock()

        self._drone_x: float = 0.0
        self._drone_y: float = 0.0
        self._drone_z: float = 0.0
        self._pos_valid: bool = False

        # virtual setpoint (moves smoothly toward target)
        self._vsp: list[float] = [0.0, 0.0, self._target_z]

        # current flight target (NED)
        self._target_x: float = 0.0
        self._target_y: float = 0.0

        self._hover_cell_id:  Optional[str] = None  # id of cell drone is in
        self._armed:          bool = False
        self._arm_requested:  bool = False
        self._land_requested: bool = False
        self._vsp_initialized: bool = False  # True after first valid position received
        self._phase:          FlightPhase = FlightPhase.IDLE
        self._rotate_ticks:   int  = 0
        self._ticks:          int  = 0

        # RTH state
        self._rth_active:    bool  = False   # True during RTH approach/descent
        self._rth_land_now:  bool  = False   # flag: send land cmd next timer tick
        self._home_ned_x:    float = 0.0     # pad NED x (from home_manager)
        self._home_ned_y:    float = 0.0     # pad NED y

        # Gradual yaw VSP
        self._drone_yaw: float = 0.0          # current heading (rad, from VehicleLocalPosition)
        self._vsp_yaw:   float = float("nan") # virtual yaw setpoint; nan = not active

        # ── UI state (main thread only — no lock needed) ───────────────────
        self._cursor_col: int = 0
        self._cursor_row: int = 0
        self._quit: bool = False
        self._status_msg: str = "Ready — press ENTER to arm and send drone"

        # ── ROS2 ───────────────────────────────────────────────────────────
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PUB)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PUB)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PUB)
        self._state_pub = self.create_publisher(
            String, "/field/grid_state", 10)
        self._rth_req_pub = self.create_publisher(
            String, "/swarm/rth_request", QOS_SWARM)
        self._landed_pub = self.create_publisher(
            String, "/swarm/landed_confirmation", QOS_SWARM)

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, QOS_SUB)
        self.create_subscription(
            Point, "/drone_0/rth_target",
            self._rth_target_cb, QOS_RTH_SUB)

        self.create_timer(DT, self._timer_cb)

        self.get_logger().info(
            f"FieldCommander ready | grid {self._cols}×{self._rows} | "
            f"altitude {self._altitude} m"
        )

    # ── Data loading ──────────────────────────────────────────────────────────
    @staticmethod
    def _load_json(path: str) -> Optional[dict]:
        try:
            with open(path) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return None

    @staticmethod
    def _parse_id(cell_id: str) -> tuple[int, int]:
        """'x4_y2' → (4, 2)"""
        parts = cell_id.split("_")
        return int(parts[0][1:]), int(parts[1][1:])

    # ── ROS2: position subscriber ─────────────────────────────────────────────
    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._drone_x  = msg.x
            self._drone_y  = msg.y
            self._drone_z  = msg.z
            self._drone_yaw = msg.heading
            self._pos_valid = True

            # Initialise VSP to drone's actual position (incl. z) on first callback
            if not self._vsp_initialized:
                self._vsp = [msg.x, msg.y, msg.z]
                self._vsp_initialized = True

            self._refresh_hover_cell()

    def _rth_target_cb(self, msg: Point) -> None:
        """Received RTH waypoint from home_manager — enter RTH flow."""
        with self._lock:
            self._home_ned_x = msg.x
            self._home_ned_y = msg.y
            self._target_x   = msg.x
            self._target_y   = msg.y
            self._rth_active = True

            if not self._armed and not self._arm_requested:
                self._arm_requested = True
                self._status_msg = "RTH: arming then returning home..."
            elif self._phase in (FlightPhase.HOVER, FlightPhase.IDLE,
                                 FlightPhase.CRUISE):
                self._phase        = FlightPhase.ROTATE
                self._rotate_ticks = 0
                self._vsp_yaw      = float("nan")
                self._status_msg   = "RTH: rotating toward home pad..."
            # TAKEOFF / ROTATE: target updated; phase continues naturally

        self.get_logger().info(
            f"RTH target received → NED({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})"
        )

    def _refresh_hover_cell(self) -> None:
        """Update which cell the drone is currently hovering in (call under lock)."""
        best_id:   Optional[str] = None
        best_dist: float         = float("inf")

        for c in self._cells:
            d = math.dist((self._drone_x, self._drone_y), (c["x"], c["y"]))
            if d < best_dist:
                best_dist = d
                best_id   = c["id"]

        if best_dist <= REACH_DIST:
            if self._hover_cell_id != best_id:
                # Drone left previous cell → mark visited
                if self._hover_cell_id:
                    prev = self._cell_by_id.get(self._hover_cell_id)
                    if prev and prev["status"] == "hovering":
                        prev["status"] = "visited"
                # Drone entered new cell → mark hovering
                self._hover_cell_id = best_id
                if best_id:
                    self._cell_by_id[best_id]["status"] = "hovering"
        else:
            # Drone not in any cell
            if self._hover_cell_id:
                prev = self._cell_by_id.get(self._hover_cell_id)
                if prev and prev["status"] == "hovering":
                    prev["status"] = "visited"
            self._hover_cell_id = None

    # ── ROS2: 10 Hz timer ────────────────────────────────────────────────────
    def _timer_cb(self) -> None:
        with self._lock:
            self._ticks += 1

            # Arm when user requested it and enough setpoints were sent
            if self._arm_requested and not self._armed and self._ticks >= ARM_TICKS:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._armed         = True
                self._arm_requested = False
                # Snap VSP xy to drone position so takeoff starts from here
                if self._vsp_initialized:
                    self._vsp[0] = self._drone_x
                    self._vsp[1] = self._drone_y
                    self._vsp[2] = self._drone_z
                self._phase = FlightPhase.TAKEOFF
                self._status_msg = f"Taking off to {self._altitude:.0f} m..."

            # Land when requested (L key or RTH descent complete)
            if self._land_requested:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_requested = False
                self._armed          = False
                self._phase          = FlightPhase.IDLE

            # RTH descent reached pad — send land command + confirmation
            if self._rth_land_now:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._armed       = False
                self._rth_land_now = False
                need_landed_pub    = True
            else:
                need_landed_pub = False

            self._step_vsp()
            tx, ty, tz = self._target_x, self._target_y, self._target_z
            vsp        = list(self._vsp)
            phase      = self._phase
            vsp_yaw    = self._vsp_yaw
            dx_t       = tx - self._drone_x
            dy_t       = ty - self._drone_y

        self._publish_offboard_mode()
        self._publish_setpoint(vsp, (dx_t, dy_t), phase, vsp_yaw)

        if need_landed_pub:
            self._publish_landed_confirmation()

        # Publish grid state every 10 ticks (1 s)
        if self._ticks % 10 == 0:
            self._publish_grid_state()

    def _step_vsp(self) -> None:
        """Phase-aware VSP step. Call under lock.

        IDLE    — hold position, do nothing
        TAKEOFF — move VSP z toward target altitude; XY fixed
                  → ROTATE when drone altitude reached
        ROTATE  — hold XY + altitude, let PX4 yaw toward target
                  → CRUISE after ROTATE_TICKS ticks
        CRUISE  — move VSP xy toward target; z fixed at altitude
                  → HOVER when XY reached
        HOVER   — VSP already at target, nothing to do
        """
        step = CRUISE_SPEED * DT

        if self._phase == FlightPhase.IDLE:
            pass

        elif self._phase == FlightPhase.TAKEOFF:
            # Move only z; clamp to target_z
            dz = self._target_z - self._vsp[2]
            if abs(dz) > step:
                self._vsp[2] += math.copysign(step, dz)
            else:
                self._vsp[2] = self._target_z
            # Transition when drone is physically close to target altitude
            if self._pos_valid and abs(self._drone_z - self._target_z) < ALT_TOL:
                self._phase        = FlightPhase.ROTATE
                self._rotate_ticks = 0
                self._vsp_yaw      = float("nan")  # will be initialised on first ROTATE tick
                self._status_msg   = "Rotating toward target..."

        elif self._phase == FlightPhase.ROTATE:
            # Gradually move _vsp_yaw toward the target heading at YAW_RATE rad/s
            dx_t = self._target_x - self._drone_x
            dy_t = self._target_y - self._drone_y
            horiz = math.sqrt(dx_t * dx_t + dy_t * dy_t)

            if horiz > REACH_DIST:
                target_yaw = math.atan2(dy_t, dx_t)
            else:
                target_yaw = self._drone_yaw  # already at target — keep current heading

            # Initialise VSP yaw to current heading on first ROTATE tick
            if math.isnan(self._vsp_yaw):
                self._vsp_yaw = self._drone_yaw

            # Angular difference, normalised to [-π, π]
            yaw_diff = (target_yaw - self._vsp_yaw + math.pi) % (2 * math.pi) - math.pi
            yaw_step = YAW_RATE * DT
            if abs(yaw_diff) > yaw_step:
                self._vsp_yaw += math.copysign(yaw_step, yaw_diff)
                # Keep in [-π, π]
                self._vsp_yaw = (self._vsp_yaw + math.pi) % (2 * math.pi) - math.pi
            else:
                self._vsp_yaw = target_yaw

            # Current heading error (drone actual yaw vs target)
            actual_diff = (target_yaw - self._drone_yaw + math.pi) % (2 * math.pi) - math.pi

            self._rotate_ticks += 1
            settled = (
                abs(actual_diff) < YAW_TOL and self._rotate_ticks >= ROTATE_TICKS
            ) or self._rotate_ticks >= MAX_ROTATE_TICKS

            if settled:
                if horiz < REACH_DIST:
                    # Already at target XY
                    if self._rth_active:
                        self._phase      = FlightPhase.RTH
                        self._status_msg = "RTH: descending to pad..."
                    else:
                        self._phase      = FlightPhase.HOVER
                        self._status_msg = "Hovering at target"
                else:
                    self._phase      = FlightPhase.CRUISE
                    self._status_msg = "Cruising to target..."

        elif self._phase == FlightPhase.CRUISE:
            # Move only xy; z stays fixed at target altitude
            dx = self._target_x - self._vsp[0]
            dy = self._target_y - self._vsp[1]
            d  = math.sqrt(dx * dx + dy * dy)
            if d > step:
                self._vsp[0] += (dx / d) * step
                self._vsp[1] += (dy / d) * step
            else:
                self._vsp[0] = self._target_x
                self._vsp[1] = self._target_y
                if self._rth_active:
                    self._phase      = FlightPhase.RTH
                    self._status_msg = "RTH: descending to pad..."
                else:
                    self._phase      = FlightPhase.HOVER
                    self._status_msg = "Hovering at target"

        elif self._phase == FlightPhase.RTH:
            # Descend slowly from cruise altitude to RTH_HOVER_Z above the pad
            rth_step = RTH_SPEED * DT
            dz = RTH_HOVER_Z - self._vsp[2]
            if abs(dz) > rth_step:
                self._vsp[2] += math.copysign(rth_step, dz)
            else:
                # Reached pad hover altitude — trigger land command
                self._vsp[2]     = RTH_HOVER_Z
                self._rth_active  = False
                self._rth_land_now = True
                self._phase       = FlightPhase.IDLE
                self._status_msg  = "RTH: landed on pad"

        # HOVER: VSP is at target, nothing to move

    def _publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position  = True
        msg.velocity  = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_landed_confirmation(self) -> None:
        msg      = String()
        msg.data = json.dumps({"drone_id": "drone_0"})
        self._landed_pub.publish(msg)
        self.get_logger().info("Landed confirmation → /swarm/landed_confirmation")

    def _publish_setpoint(
        self,
        vsp: list[float],
        drone_to_target: tuple[float, float],
        phase: FlightPhase,
        vsp_yaw: float = float("nan"),
    ) -> None:
        nan = float("nan")
        # ROTATE: use gradual _vsp_yaw (smooth rotation)
        # CRUISE: point instantly toward target (already facing it after ROTATE)
        # TAKEOFF / HOVER / RTH: PX4 keeps current heading (nan)
        if phase == FlightPhase.ROTATE:
            yaw = vsp_yaw  # may be nan on very first tick — PX4 holds heading
        elif phase == FlightPhase.CRUISE:
            tx, ty = drone_to_target
            horiz  = math.sqrt(tx * tx + ty * ty)
            yaw    = math.atan2(ty, tx) if horiz > 0.3 else nan
        else:
            yaw = nan

        msg = TrajectorySetpoint()
        msg.position     = [vsp[0], vsp[1], vsp[2]]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = yaw
        msg.timestamp    = self._now_us()
        self._traj_pub.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._now_us()
        self._cmd_pub.publish(msg)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _publish_grid_state(self) -> None:
        with self._lock:
            snapshot = [
                {"id": c["id"], "x": c["x"], "y": c["y"], "status": c["status"]}
                for c in self._cells
            ]
        msg = String()
        msg.data = json.dumps(snapshot, separators=(",", ":"))
        self._state_pub.publish(msg)

    # ── Send drone to cell ────────────────────────────────────────────────────
    def _go_to_cell(self, col: int, row: int) -> None:
        cell = self._cell_by_cr.get((col, row))
        if cell is None:
            return
        with self._lock:
            self._target_x   = cell["x"]
            self._target_y   = cell["y"]
            self._rth_active = False   # cancel any pending RTH

            if not self._armed and not self._arm_requested:
                # Not armed yet — request arm; phase will be set to TAKEOFF on arm
                self._arm_requested = True
                self._status_msg = f"Arming... then taking off to {self._altitude:.0f} m"
            elif self._phase in (FlightPhase.HOVER, FlightPhase.IDLE):
                # Already airborne and stationary — start rotate+cruise to new cell
                self._phase        = FlightPhase.ROTATE
                self._rotate_ticks = 0
                self._vsp_yaw      = float("nan")
                self._status_msg   = f"Rotating toward {cell['id']}..."
            elif self._phase == FlightPhase.CRUISE:
                # Already flying — update target and restart rotate from current VSP position
                self._phase        = FlightPhase.ROTATE
                self._rotate_ticks = 0
                self._vsp_yaw      = float("nan")
                self._status_msg   = f"Re-routing to {cell['id']}..."
            # TAKEOFF / ROTATE: just update target; current phase continues naturally

    # ── Save grid JSON ────────────────────────────────────────────────────────
    def _save_grid(self) -> None:
        with self._lock:
            exported = [
                {"id": c["id"], "x": c["x"], "y": c["y"], "status": c["status"]}
                for c in self._cells
            ]
        payload = {
            "cell_size_m": self._cell_size,
            "cols":        self._cols,
            "rows":        self._rows,
            "cells":       exported,
        }
        with open(GRID_FILE, "w") as f:
            json.dump(payload, f, indent=2)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def on_shutdown(self) -> None:
        self._save_grid()
        self.get_logger().info(f"Grid saved → {GRID_FILE}")

    # =========================================================================
    # ── Curses UI ─────────────────────────────────────────────────────────────
    # =========================================================================
    def run_ui(self, stdscr: "curses._CursesWindow") -> None:
        _setup_colors()
        curses.curs_set(0)
        stdscr.keypad(True)
        stdscr.nodelay(True)  # non-blocking getch

        while not self._quit:
            self._draw(stdscr)
            key = stdscr.getch()
            if key != -1:
                self._handle_key(key)
            time.sleep(0.05)  # ~20 fps

    # ── Drawing ───────────────────────────────────────────────────────────────
    def _draw(self, stdscr: "curses._CursesWindow") -> None:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # Layout constants
        GRID_LEFT    = 5         # left margin for grid (row label width)
        GRID_TOP     = 3         # rows used by header
        PANEL_WIDTH  = 32        # right info panel width
        FOOTER_ROWS  = 3         # rows for bottom bar
        grid_w_avail = w - GRID_LEFT - PANEL_WIDTH - 2
        grid_h_avail = h - GRID_TOP - FOOTER_ROWS

        # Viewport: how many cols/rows fit on screen
        vis_cols = min(self._cols, max(1, grid_w_avail))
        vis_rows = min(self._rows, max(1, grid_h_avail))

        # Scroll to keep cursor visible
        v_col0 = max(0, min(self._cursor_col - vis_cols // 2,
                            self._cols - vis_cols))
        v_row0 = max(0, min(self._cursor_row - vis_rows // 2,
                            self._rows - vis_rows))

        # ── Header ────────────────────────────────────────────────────────
        title = (f"  Field Commander — {self._cols}×{self._rows} grid  "
                 f"(cell {self._cell_size} m, alt {self._altitude} m)")
        self._safe_addstr(stdscr, 0, 0, title[:w],
                          curses.color_pair(CP_TITLE) | curses.A_BOLD)

        with self._lock:
            armed_str   = "ARMED" if self._armed else "disarmed"
            drone_x     = self._drone_x
            drone_y     = self._drone_y
            drone_z     = self._drone_z
            hover_id    = self._hover_cell_id
            pos_valid   = self._pos_valid
            phase       = self._phase
            # Snapshot of per-cell status — avoids data race with _pos_cb
            cell_status: dict[str, str] = {c["id"]: c["status"] for c in self._cells}

        # Phase label with color
        _PHASE_LABEL = {
            FlightPhase.IDLE:    ("IDLE",    CP_DIM),
            FlightPhase.TAKEOFF: ("TAKEOFF", CP_ACCENT),
            FlightPhase.ROTATE:  ("ROTATE",  CP_ACCENT),
            FlightPhase.CRUISE:  ("CRUISE",  CP_HOVER),
            FlightPhase.HOVER:   ("HOVER",   CP_CURSOR),
            FlightPhase.RTH:     ("RTH",     CP_VISITED),  # red-ish via visited color
        }
        phase_txt, phase_cp = _PHASE_LABEL.get(phase, ("?", CP_DIM))

        armed_color = curses.color_pair(CP_HOVER) if self._armed else curses.color_pair(CP_DIM) | curses.A_DIM
        self._safe_addstr(stdscr, 1, 2, f"[{armed_str}]", armed_color)
        self._safe_addstr(stdscr, 1, 12, f"[{phase_txt}]",
                          curses.color_pair(phase_cp) | curses.A_BOLD)
        if pos_valid:
            self._safe_addstr(
                stdscr, 1, 22,
                f"x={drone_x:6.2f}  y={drone_y:6.2f}  z={drone_z:5.2f}",
                curses.color_pair(CP_ACCENT))

        # ── Column header (col numbers mod 10) ────────────────────────────
        col_hdr_y = GRID_TOP - 1
        for sc in range(vis_cols):
            col_idx = v_col0 + sc
            label   = str(col_idx % 10)
            self._safe_addstr(stdscr, col_hdr_y, GRID_LEFT + sc, label,
                              curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Grid rows ──────────────────────────────────────────────────────
        for sr in range(vis_rows):
            row_idx    = v_row0 + sr
            screen_row = GRID_TOP + sr

            # Row label (right-aligned in 4 chars)
            self._safe_addstr(stdscr, screen_row, 0, f"{row_idx:3d} ",
                              curses.color_pair(CP_DIM) | curses.A_DIM)

            for sc in range(vis_cols):
                col_idx = v_col0 + sc
                cell    = self._cell_by_cr.get((col_idx, row_idx))
                if cell is None:
                    continue

                status = cell_status.get(cell["id"], "unknown")
                ch     = STATUS_CHAR.get(status, "?")
                is_cursor = (col_idx == self._cursor_col and
                             row_idx == self._cursor_row)
                is_hover  = (cell["id"] == hover_id)

                if is_cursor and is_hover:
                    attr = curses.color_pair(CP_HOVER) | curses.A_BOLD | curses.A_REVERSE
                elif is_cursor:
                    attr = curses.color_pair(CP_CURSOR) | curses.A_BOLD
                elif is_hover:
                    attr = curses.color_pair(CP_HOVER) | curses.A_BOLD
                elif status == "visited":
                    attr = curses.color_pair(CP_VISITED) | curses.A_DIM
                else:
                    attr = curses.color_pair(CP_NORMAL)

                self._safe_addstr(stdscr, screen_row, GRID_LEFT + sc, ch, attr)

        # ── Right info panel ───────────────────────────────────────────────
        px = w - PANEL_WIDTH
        sel_cell = self._cell_by_cr.get((self._cursor_col, self._cursor_row))
        sel_status = cell_status.get(sel_cell["id"], "unknown") if sel_cell else "unknown"
        self._draw_panel(stdscr, px, GRID_TOP, PANEL_WIDTH,
                         sel_cell, sel_status, drone_x, drone_y, drone_z, hover_id, pos_valid)

        # ── Scroll hints ───────────────────────────────────────────────────
        if self._cols > vis_cols or self._rows > vis_rows:
            hint = (f" view {v_col0}–{v_col0+vis_cols-1} / "
                    f"{v_row0}–{v_row0+vis_rows-1} ")
            self._safe_addstr(stdscr, GRID_TOP + vis_rows, GRID_LEFT,
                              hint[:w - GRID_LEFT],
                              curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Footer ────────────────────────────────────────────────────────
        footer_y = h - FOOTER_ROWS
        sep = "─" * (w - 1)
        self._safe_addstr(stdscr, footer_y,     0, sep[:w - 1],
                          curses.color_pair(CP_DIM) | curses.A_DIM)
        self._safe_addstr(stdscr, footer_y + 1, 1,
                          "↑↓←→ navigate   ENTER send drone   R RTH   L land   Q quit",
                          curses.color_pair(CP_ACCENT))
        self._safe_addstr(stdscr, footer_y + 2, 1,
                          self._status_msg[:w - 2],
                          curses.color_pair(CP_DIM) | curses.A_DIM)

        stdscr.refresh()

    def _draw_panel(
        self,
        stdscr: "curses._CursesWindow",
        px: int, py: int, pw: int,
        sel_cell: Optional[dict],
        sel_status: str,
        drone_x: float, drone_y: float, drone_z: float,
        hover_id: Optional[str],
        pos_valid: bool,
    ) -> None:
        h, w = stdscr.getmaxyx()
        if px >= w:
            return

        def row(y: int, label: str, value: str = "",
                attr: int = curses.color_pair(CP_NORMAL)) -> None:
            line = f"{label:<12}{value}"
            self._safe_addstr(stdscr, py + y, px, line[:pw - 1], attr)

        self._safe_addstr(stdscr, py, px,
                          "┄ Selected Cell " + "┄" * max(0, pw - 17),
                          curses.color_pair(CP_ACCENT))

        if sel_cell:
            row(1, "Cell:", sel_cell["id"], curses.color_pair(CP_TITLE) | curses.A_BOLD)
            row(2, "NED x:", f"{sel_cell['x']:.2f} m  (North)")
            row(3, "NED y:", f"{sel_cell['y']:.2f} m  (East)")
            row(4, "NED z:", f"{self._target_z:.2f} m  (alt {self._altitude} m)")
            s_color = (curses.color_pair(CP_HOVER) if sel_status == "hovering"
                       else curses.color_pair(CP_VISITED) | curses.A_DIM
                       if sel_status == "visited"
                       else curses.color_pair(CP_NORMAL))
            row(5, "Status:", sel_status, s_color)
        else:
            row(1, "—", "", curses.color_pair(CP_DIM) | curses.A_DIM)

        self._safe_addstr(stdscr, py + 7, px,
                          "┄ Drone Position " + "┄" * max(0, pw - 18),
                          curses.color_pair(CP_ACCENT))

        if pos_valid:
            row(8,  "x:", f"{drone_x:7.3f}  (North)")
            row(9,  "y:", f"{drone_y:7.3f}  (East)")
            row(10, "z:", f"{drone_z:7.3f}  (NED down)")
            hover_label = hover_id if hover_id else "—"
            row(11, "In cell:", hover_label,
                curses.color_pair(CP_HOVER) if hover_id
                else curses.color_pair(CP_DIM) | curses.A_DIM)
        else:
            row(8, "waiting for", "position...",
                curses.color_pair(CP_DIM) | curses.A_DIM)

        # Legend
        self._safe_addstr(stdscr, py + 13, px,
                          "┄ Legend " + "┄" * max(0, pw - 10),
                          curses.color_pair(CP_ACCENT))
        legends = [
            (".", "unvisited"),
            ("H", "drone here"),
            ("X", "visited"),
            ("?", "unknown"),
        ]
        for i, (ch, label) in enumerate(legends):
            self._safe_addstr(stdscr, py + 14 + i, px, f"  {ch}  {label}",
                              curses.color_pair(CP_NORMAL))

    # ── Key handling ──────────────────────────────────────────────────────────
    def _handle_key(self, key: int) -> None:
        if key == curses.KEY_UP:
            self._cursor_row = max(0, self._cursor_row - 1)
        elif key == curses.KEY_DOWN:
            self._cursor_row = min(self._rows - 1, self._cursor_row + 1)
        elif key == curses.KEY_LEFT:
            self._cursor_col = max(0, self._cursor_col - 1)
        elif key == curses.KEY_RIGHT:
            self._cursor_col = min(self._cols - 1, self._cursor_col + 1)

        elif key in (curses.KEY_ENTER, 10, 13):
            self._go_to_cell(self._cursor_col, self._cursor_row)

        elif key in (ord("r"), ord("R")):
            # Request RTH via home_manager (it will reply on /drone_0/rth_target)
            msg      = String()
            msg.data = json.dumps({"drone_id": "drone_0", "reason": "manual_rth"})
            self._rth_req_pub.publish(msg)
            self._status_msg = "RTH requested → waiting for home_manager..."

        elif key in (ord("l"), ord("L")):
            with self._lock:
                self._land_requested = True
                self._rth_active     = False
                self._target_x       = self._drone_x
                self._target_y       = self._drone_y
                self._phase          = FlightPhase.IDLE
            self._status_msg = "Landing..."

        elif key in (ord("q"), ord("Q"), 27):
            self._quit = True
            self._status_msg = "Saving and quitting..."

    # ── curses safe write ──────────────────────────────────────────────────────
    @staticmethod
    def _safe_addstr(
        win: "curses._CursesWindow",
        y: int, x: int,
        text: str,
        attr: int = 0,
    ) -> None:
        h, w = win.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        max_len = w - x - 1
        if max_len <= 0:
            return
        try:
            win.addstr(y, x, text[:max_len], attr)
        except curses.error:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = FieldCommander()
    except RuntimeError as e:
        print(f"[field_commander] ERROR: {e}")
        rclpy.try_shutdown()
        return

    # ROS2 spin in background thread — keeps callbacks running while curses runs
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(node.run_ui)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
