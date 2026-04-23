"""
manual_commander.py — WSAD manual flight for field perimeter mapping

Fly the drone manually to define the field boundary and home pad position.

Controls:
  W / S     — fly North / South  (NED +x / -x)
  A / D     — fly West  / East   (NED -y / +y)
  H         — mark current position as home / landing pad
  R         — record current position as a perimeter corner
  ENTER     — save field_perimeter.json + home_positions.json and quit
  L         — land (disarm)
  Q         — quit without saving perimeter

Workflow:
  1. Drone arms and takes off to altitude automatically.
  2. Fly to the home pad → press H.
  3. Fly to each field corner → press R at each one.
  4. Press ENTER to close the polygon, save files, and exit.
  5. Run grid_generator to create the flight grid.

Output files:
  <ws_root>/perimeters/field_perimeter.json   — perimeter corners (input for grid_generator)
  <ws_root>/perimeters/home_positions.json    — home pad position (input for home_manager)

Usage:
  ros2 run scout_control manual_commander
  ros2 run scout_control manual_commander --ros-args -p altitude:=5.0
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

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

# ── Output paths ──────────────────────────────────────────────────────────────
from scout_control.utils.paths import PERIMETER_FILE, HOME_POS_FILE, PERIMETERS_DIR

# ── QoS (must match PX4) ──────────────────────────────────────────────────────
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

# ── Flight parameters ─────────────────────────────────────────────────────────
DEFAULT_ALT  = 5.0   # m — fallback altitude when not set via ROS param
MANUAL_SPEED = 2.0   # m/s — VSP movement speed per second (applied at UI fps)
ARM_TICKS    = 10    # timer ticks before arming (1 s at 10 Hz)
ALT_TOL      = 0.4   # m — altitude error to consider takeoff done
DT           = 0.1   # s — timer period (10 Hz)
UI_FPS       = 20    # Hz — UI refresh rate
UI_STEP      = MANUAL_SPEED / UI_FPS  # m per UI frame when key held

# ── Flight phases ─────────────────────────────────────────────────────────────
class Phase(Enum):
    IDLE    = auto()   # on ground, not armed
    TAKEOFF = auto()   # rising to target altitude
    FLY     = auto()   # manual WSAD control active

# ── Curses colour pairs ───────────────────────────────────────────────────────
CP_NORMAL  = 1
CP_TITLE   = 2
CP_ARMED   = 3
CP_DIM     = 4
CP_ACCENT  = 5
CP_CORNER  = 6
CP_HOME    = 7

def _setup_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(CP_NORMAL,  curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_TITLE,   curses.COLOR_YELLOW, -1)
    curses.init_pair(CP_ARMED,   curses.COLOR_BLACK,  curses.COLOR_GREEN)
    curses.init_pair(CP_DIM,     curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_ACCENT,  curses.COLOR_CYAN,   -1)
    curses.init_pair(CP_CORNER,  curses.COLOR_GREEN,  -1)
    curses.init_pair(CP_HOME,    curses.COLOR_YELLOW, -1)


# ── Node ──────────────────────────────────────────────────────────────────────
class ManualCommander(Node):

    def __init__(self) -> None:
        super().__init__("manual_commander")

        self.declare_parameter("altitude", DEFAULT_ALT)
        self._altitude: float = float(
            self.get_parameter("altitude").value
        )
        self._target_z: float = -self._altitude   # NED z (negative = up)

        # ── Shared state (guarded by _lock) ───────────────────────────────
        self._lock = threading.Lock()

        self._drone_x:    float = 0.0
        self._drone_y:    float = 0.0
        self._drone_z:    float = 0.0
        self._pos_valid:  bool  = False

        # virtual setpoint (updated by UI thread, published by timer thread)
        self._vsp: list[float] = [0.0, 0.0, 0.0]
        self._vsp_initialized: bool = False

        self._armed:         bool  = False
        self._arm_requested: bool  = False
        self._phase:         Phase = Phase.IDLE
        self._ticks:         int   = 0

        # recorded data
        self._corners:   list[tuple[float, float, float]] = []  # NED (x,y,z)
        self._home_ned:  Optional[tuple[float, float]]    = None # NED (x,y)

        # ── UI state (main thread only) ────────────────────────────────────
        self._quit:       bool = False
        self._status_msg: str  = "Waiting for position... then arming automatically"
        self._flash_msg:  str  = ""
        self._flash_time: float = 0.0

        # ── ROS2 ──────────────────────────────────────────────────────────
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PUB)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PUB)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PUB)

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, QOS_SUB)

        self.create_timer(DT, self._timer_cb)

        # Auto-request arm on startup
        self._arm_requested = True

        self.get_logger().info(
            f"ManualCommander ready | altitude={self._altitude} m"
        )

    # ── Position callback ─────────────────────────────────────────────────────
    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._drone_x  = msg.x
            self._drone_y  = msg.y
            self._drone_z  = msg.z
            self._pos_valid = True
            if not self._vsp_initialized:
                self._vsp = [msg.x, msg.y, msg.z]
                self._vsp_initialized = True

    # ── 10 Hz timer ───────────────────────────────────────────────────────────
    def _timer_cb(self) -> None:
        with self._lock:
            self._ticks += 1

            # Arm once we have position and enough setpoints sent
            if self._arm_requested and not self._armed and self._ticks >= ARM_TICKS:
                if self._vsp_initialized:
                    self._vsp[2] = self._drone_z   # snap z to ground
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._armed         = True
                self._arm_requested = False
                self._phase         = Phase.TAKEOFF
                self._status_msg    = f"Taking off to {self._altitude:.0f} m..."

            # Takeoff: move VSP z toward target altitude
            if self._phase == Phase.TAKEOFF:
                step = MANUAL_SPEED * DT
                dz   = self._target_z - self._vsp[2]
                if abs(dz) > step:
                    self._vsp[2] += math.copysign(step, dz)
                else:
                    self._vsp[2] = self._target_z
                # Transition to FLY when drone reaches altitude
                if self._pos_valid and abs(self._drone_z - self._target_z) < ALT_TOL:
                    self._phase      = Phase.FLY
                    self._status_msg = "Flying — use WSAD. H=home  R=corner  ENTER=save"

            vsp   = list(self._vsp)
            phase = self._phase

        self._publish_offboard_mode()
        self._publish_setpoint(vsp, phase)

    # ── Publishers ────────────────────────────────────────────────────────────
    def _publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position  = True
        msg.velocity  = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_setpoint(self, vsp: list[float], phase: Phase) -> None:
        nan = float("nan")
        msg = TrajectorySetpoint()
        msg.position     = [vsp[0], vsp[1], vsp[2]]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = nan   # PX4 holds current heading
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

    # ── Manual movement (called from UI thread, under lock) ───────────────────
    def _move_vsp(self, dx: float, dy: float) -> None:
        """Shift VSP by (dx, dy) in NED. Z stays fixed. Call under lock."""
        self._vsp[0] += dx
        self._vsp[1] += dy

    # ── Data actions ──────────────────────────────────────────────────────────
    def mark_home(self) -> None:
        with self._lock:
            if not self._pos_valid:
                self._flash("No position yet")
                return
            x, y = self._drone_x, self._drone_y
            self._home_ned = (x, y)
        self._flash(f"Home marked: NED({x:.2f}, {y:.2f})")

    def record_corner(self) -> None:
        with self._lock:
            if not self._pos_valid:
                self._flash("No position yet")
                return
            z = self._target_z
            x, y = self._drone_x, self._drone_y
            self._corners.append((x, y, z))
            n = len(self._corners)
        self._flash(f"Corner #{n} recorded: ({x:.2f}, {y:.2f})")

    def save_and_exit(self) -> bool:
        """Save perimeter + home position. Returns True if saved, False if not enough data."""
        with self._lock:
            corners = list(self._corners)
            home    = self._home_ned

        if len(corners) < 2:
            self._flash("Need at least 2 corners (press R at each corner)")
            return False

        os.makedirs(PERIMETERS_DIR, exist_ok=True)

        # ── field_perimeter.json ──────────────────────────────────────────
        waypoints_ned = [[c[0], c[1], c[2]] for c in corners]
        perimeter_payload = {
            "altitude_m":    self._altitude,
            "waypoints_ned": waypoints_ned,
        }
        with open(PERIMETER_FILE, "w") as f:
            json.dump(perimeter_payload, f, indent=2)

        # ── home_positions.json ───────────────────────────────────────────
        if home is not None:
            hx, hy = home
            # Gazebo ENU: gz_x = NED_y, gz_y = NED_x
            home_payload = {
                "home_positions": [
                    {
                        "pad_id":   "pad_0",
                        "drone_id": "drone_0",
                        "ned":      {"x": round(hx, 3), "y": round(hy, 3), "z": -0.5},
                        "gz_pose":  {"x": round(hy, 3), "y": round(hx, 3), "z": 0.0},
                        "status":   "available",
                    }
                ]
            }
            with open(HOME_POS_FILE, "w") as f:
                json.dump(home_payload, f, indent=2)

        self.get_logger().info(
            f"Saved perimeter ({len(corners)} corners) → {PERIMETER_FILE}"
        )
        if home:
            self.get_logger().info(f"Saved home position → {HOME_POS_FILE}")
        return True

    def land(self) -> None:
        with self._lock:
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._armed = False
            self._phase = Phase.IDLE
            self._status_msg = "Landing..."

    def _flash(self, msg: str) -> None:
        """Set a flash message visible for 2 seconds (no lock needed)."""
        self._flash_msg  = msg
        self._flash_time = time.monotonic()

    # =========================================================================
    # ── Curses UI ─────────────────────────────────────────────────────────────
    # =========================================================================
    def run_ui(self, stdscr: "curses._CursesWindow") -> None:
        _setup_colors()
        curses.curs_set(0)
        stdscr.keypad(True)
        stdscr.nodelay(True)

        frame_time = 1.0 / UI_FPS

        while not self._quit:
            t0 = time.monotonic()
            self._draw(stdscr)
            key = stdscr.getch()
            if key != -1:
                self._handle_key(key)
            elapsed = time.monotonic() - t0
            remaining = frame_time - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ── Key handler ───────────────────────────────────────────────────────────
    def _handle_key(self, key: int) -> None:
        with self._lock:
            fly = self._phase == Phase.FLY

        if key in (ord('q'), ord('Q')):
            self._quit = True

        elif key in (ord('l'), ord('L')):
            self.land()

        elif key in (ord('h'), ord('H')):
            self.mark_home()

        elif key in (ord('r'), ord('R')):
            self.record_corner()

        elif key in (curses.KEY_ENTER, 10, 13):   # ENTER
            if self.save_and_exit():
                self.land()
                time.sleep(0.5)
                self._quit = True

        # ── WSAD movement (only in FLY phase) ─────────────────────────────
        elif fly:
            dx, dy = 0.0, 0.0
            if key in (ord('w'), ord('W')):
                dx = +UI_STEP   # North (+x)
            elif key in (ord('s'), ord('S')):
                dx = -UI_STEP   # South (-x)
            elif key in (ord('a'), ord('A')):
                dy = -UI_STEP   # West  (-y)
            elif key in (ord('d'), ord('D')):
                dy = +UI_STEP   # East  (+y)
            if dx != 0.0 or dy != 0.0:
                with self._lock:
                    self._move_vsp(dx, dy)

    # ── Drawing ───────────────────────────────────────────────────────────────
    @staticmethod
    def _safe_addstr(
        stdscr: "curses._CursesWindow",
        y: int, x: int, text: str, attr: int = 0,
    ) -> None:
        h, w = stdscr.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        try:
            stdscr.addstr(y, x, text[: w - x], attr)
        except curses.error:
            pass

    def _draw(self, stdscr: "curses._CursesWindow") -> None:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        with self._lock:
            drone_x   = self._drone_x
            drone_y   = self._drone_y
            drone_z   = self._drone_z
            pos_valid = self._pos_valid
            armed     = self._armed
            phase     = self._phase
            vsp_x     = self._vsp[0]
            vsp_y     = self._vsp[1]
            corners   = list(self._corners)
            home      = self._home_ned
            status    = self._status_msg

        sa = self._safe_addstr

        # ── Title bar ─────────────────────────────────────────────────────
        title = f"  Manual Commander  |  alt={self._altitude:.0f} m  |  " \
                f"{'ARMED' if armed else 'disarmed'}  |  {phase.name}"
        sa(stdscr, 0, 0, title[:w],
           curses.color_pair(CP_TITLE) | curses.A_BOLD)

        # ── Position row ──────────────────────────────────────────────────
        if pos_valid:
            pos_str = (f"  Drone  x={drone_x:7.2f}  y={drone_y:7.2f}  "
                       f"z={drone_z:6.2f}  (alt={-drone_z:.2f} m)")
            sa(stdscr, 1, 0, pos_str[:w], curses.color_pair(CP_ACCENT))
        else:
            sa(stdscr, 1, 0, "  Waiting for position...",
               curses.color_pair(CP_DIM) | curses.A_DIM)

        vsp_str = (f"  VSP    x={vsp_x:7.2f}  y={vsp_y:7.2f}  "
                   f"z={self._target_z:6.2f}")
        sa(stdscr, 2, 0, vsp_str[:w], curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Separator ─────────────────────────────────────────────────────
        sa(stdscr, 3, 0, "─" * (w - 1),
           curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Home pad ──────────────────────────────────────────────────────
        sa(stdscr, 4, 2, "Home pad:", curses.color_pair(CP_HOME) | curses.A_BOLD)
        if home:
            sa(stdscr, 4, 14,
               f"NED({home[0]:.2f}, {home[1]:.2f})  [press H to update]",
               curses.color_pair(CP_HOME))
        else:
            sa(stdscr, 4, 14, "not set — fly to pad and press H",
               curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Perimeter corners ─────────────────────────────────────────────
        sa(stdscr, 6, 2,
           f"Perimeter corners ({len(corners)}) — press R at each corner:",
           curses.color_pair(CP_CORNER) | curses.A_BOLD)

        max_list_rows = h - 12
        for i, (cx, cy, cz) in enumerate(corners[:max_list_rows]):
            sa(stdscr, 7 + i, 4,
               f"#{i + 1:2d}  x={cx:7.2f}  y={cy:7.2f}",
               curses.color_pair(CP_CORNER))

        if len(corners) > max_list_rows:
            sa(stdscr, 7 + max_list_rows, 4,
               f"  ... and {len(corners) - max_list_rows} more",
               curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Flash message ─────────────────────────────────────────────────
        flash_row = h - 4
        if self._flash_msg and (time.monotonic() - self._flash_time) < 2.0:
            sa(stdscr, flash_row, 2, f"  {self._flash_msg}  ",
               curses.color_pair(CP_ARMED) | curses.A_BOLD)

        # ── Status + footer ───────────────────────────────────────────────
        sa(stdscr, h - 3, 0, "─" * (w - 1),
           curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, h - 2, 1, status[:w - 2],
           curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, h - 1, 1,
           "W/S=N/S  A/D=W/E  H=home  R=corner  ENTER=save+land  L=land  Q=quit",
           curses.color_pair(CP_ACCENT))

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualCommander()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(node.run_ui)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
