"""
manual_controller.py — Dual-drone manual controller for E2E field setup

Extends manual_commander.py with:
  • Tab   — switch active drone (drone_0 ↔ drone_1)
  • H     — record drone_0 current position as pad_0, publish to:
              /swarm/pad_assignment  {"drone_id":"drone_0","pad_id":"pad_0","x":…,"y":…,"z":…}
              /drone_0/rth_target    geometry_msgs/Point (NED)
  • J     — record drone_1 current position as pad_1, publish to:
              /swarm/pad_assignment  {"drone_id":"drone_1","pad_id":"pad_1","x":…,"y":…,"z":…}
              /drone_1/rth_target    geometry_msgs/Point (NED)
  • C     — corner marking submenu: next key 1=NE 2=NW 3=SE 4=SW
              publishes drone_0 position on /field/corner_marked as JSON String
  • M     — confirm mission start: publishes {"operator":"confirmed"} on
              /field/mission_confirm → field_setup_coordinator fires mission_ready
  • W/S/A/D — fly active drone N/S/W/E (NED)
  • ↑↓    — altitude up/down for active drone
  • L     — land active drone
  • Q     — quit

Both drones publish VSP setpoints continuously (inactive drone holds position).
drone_0 uses bare  /fmu/in/…  topics (PX4 sysid=1)
drone_1 uses /px4_1/fmu/in/…  topics (PX4 sysid=2)

Usage:
  ros2 run scout_control manual_controller
  ros2 run scout_control manual_controller --ros-args -p altitude:=5.0
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

from scout_control.paths import PERIMETERS_DIR

# ── QoS ───────────────────────────────────────────────────────────────────────
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
QOS_SWARM = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
# Latched RELIABLE publisher — used for /drone_N/rth_target so that swarm_agent
# (which subscribes RELIABLE+TRANSIENT_LOCAL) actually receives the messages.
# BEST_EFFORT publisher + RELIABLE subscriber → ROS2 drops all messages (QoS mismatch).
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Constants ─────────────────────────────────────────────────────────────────
DEFAULT_ALT  = 5.0
MANUAL_SPEED = 2.0   # m/s
ALT_SPEED    = 1.0   # m/s altitude change
ARM_TICKS    = 10
ALT_TOL      = 0.4
DT           = 0.1
UI_FPS       = 20
UI_STEP      = MANUAL_SPEED / UI_FPS
ALT_STEP     = ALT_SPEED / UI_FPS

CORNER_LABELS = {ord('1'): 'NE', ord('2'): 'NW', ord('3'): 'SE', ord('4'): 'SW'}

# ── Flight phases ─────────────────────────────────────────────────────────────
class Phase(Enum):
    IDLE    = auto()
    TAKEOFF = auto()
    FLY     = auto()

# ── Curses colour pairs ───────────────────────────────────────────────────────
CP_NORMAL  = 1
CP_TITLE   = 2
CP_ARMED   = 3
CP_DIM     = 4
CP_ACCENT  = 5
CP_CORNER  = 6
CP_HOME    = 7
CP_DRONE1  = 8

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
    curses.init_pair(CP_DRONE1,  curses.COLOR_MAGENTA, -1)


# ── Per-drone state ───────────────────────────────────────────────────────────
class DroneCtrl:
    """Mutable state for one drone, always accessed under outer lock."""

    def __init__(self, drone_id: int, px4_ns: str) -> None:
        self.drone_id  = drone_id
        self.px4_ns    = px4_ns       # "" for drone_0, "/px4_1" for drone_1
        self.did       = f"drone_{drone_id}"

        self.x: float  = 0.0
        self.y: float  = 0.0
        self.z: float  = 0.0
        # pos_valid: True only when PX4 reports xy_valid=True (EKF has converged).
        # PX4 may publish VehicleLocalPosition messages with xy_valid=False and
        # position near (0,0) before the EKF has a proper fix — we must not treat
        # those as valid pad positions (Bug 6).
        self.pos_valid:  bool = False
        self.xy_valid:   bool = False   # mirrors msg.xy_valid from PX4

        self.vsp: list[float] = [0.0, 0.0, 0.0]
        self.vsp_init  = False
        self.armed     = False
        self.phase     = Phase.IDLE
        self.ticks     = 0

        # Landing sequence: stop offboard heartbeat, then switch to AUTO.LAND
        self.landing:       bool = False
        self.landing_ticks: int  = 0

        # Autonomous RTH state (triggered by home_manager /drone_N/rth_target)
        self.rth_active:    bool = False
        self.rth_vsp:       list[float] = [0.0, 0.0, 0.0]


# ── Node ──────────────────────────────────────────────────────────────────────
class ManualController(Node):

    def __init__(self) -> None:
        super().__init__("manual_controller")

        self.declare_parameter("altitude", DEFAULT_ALT)
        self._altitude: float = float(self.get_parameter("altitude").value)
        self._target_z: float = -self._altitude

        self._lock = threading.Lock()

        # Two drone control blocks
        self._d = [
            DroneCtrl(0, ""),
            DroneCtrl(1, "/px4_1"),
        ]
        self._active: int = 0   # index into self._d; Tab switches

        # Corner state
        self._corners: dict[str, tuple[float, float, float]] = {}  # label → NED
        self._corner_submenu: bool = False  # waiting for 1/2/3/4 keypress

        # Pad assignment state
        self._pads: dict[str, Optional[tuple[float, float]]] = {
            "pad_0": None,
            "pad_1": None,
        }

        # UI state
        self._quit       = False
        self._flash_msg  = ""
        self._flash_time = 0.0
        self._status_msg = "Arming automatically…"
        self._mission_started = False   # True after /swarm/mission_ready → stops offboard output

        # ── Publishers per drone ──────────────────────────────────────────────
        self._offboard_pubs: list = []
        self._traj_pubs:    list = []
        self._cmd_pubs:     list = []

        for d in self._d:
            ns = d.px4_ns
            self._offboard_pubs.append(self.create_publisher(
                OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", QOS_PUB))
            self._traj_pubs.append(self.create_publisher(
                TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", QOS_PUB))
            self._cmd_pubs.append(self.create_publisher(
                VehicleCommand, f"{ns}/fmu/in/vehicle_command", QOS_PUB))

        # Pad assignment + corner marking + mission confirm publishers
        self._pad_assign_pub = self.create_publisher(
            String, "/swarm/pad_assignment", QOS_SWARM)
        self._corner_pub = self.create_publisher(
            String, "/field/corner_marked", QOS_SWARM)
        self._mission_confirm_pub = self.create_publisher(
            String, "/field/mission_confirm", QOS_SWARM)
        self._landed_pub = self.create_publisher(
            String, "/swarm/landed_confirmation", QOS_SWARM)
        self._rth_pubs: dict[str, object] = {
            "drone_0": self.create_publisher(Point, "/drone_0/rth_target", QOS_LATCHED),
            "drone_1": self.create_publisher(Point, "/drone_1/rth_target", QOS_LATCHED),
        }

        # ── Subscribers per drone ─────────────────────────────────────────────
        for i, d in enumerate(self._d):
            ns = d.px4_ns
            self.create_subscription(
                VehicleLocalPosition,
                f"{ns}/fmu/out/vehicle_local_position_v1",
                self._make_pos_cb(i),
                QOS_SUB,
            )
            self.create_subscription(
                Point,
                f"/{d.did}/rth_target",
                self._make_rth_cb(i),
                QOS_LATCHED,
            )

        # Stop publishing offboard commands when mission starts so swarm_agents
        # can take over without conflicting setpoints.
        # VOLATILE: intentionally ignores stale latched messages from previous
        # sessions. manual_controller is always running before mission_ready is
        # published (operator presses M), so it will always catch the live message.
        _qos_mission_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/swarm/mission_ready",
            self._mission_ready_cb, _qos_mission_volatile)

        self.create_subscription(
            String, "/swarm/landed_confirmation",
            self._landed_cb, QOS_SWARM)

        self.create_timer(DT, self._timer_cb)

        self.get_logger().info(
            f"ManualController ready | altitude={self._altitude} m | 2 drones"
        )

    # ── Position callbacks ────────────────────────────────────────────────────
    def _make_pos_cb(self, idx: int):
        def _cb(msg: VehicleLocalPosition) -> None:
            with self._lock:
                d = self._d[idx]
                d.x        = msg.x
                d.y        = msg.y
                d.z        = msg.z
                d.xy_valid = bool(msg.xy_valid)
                # Only mark position as valid when PX4's EKF reports xy_valid.
                # Early messages arrive with xy_valid=False and position near (0,0)
                # before the EKF converges — treating them as valid would record
                # the origin instead of the actual spawn position (Bug 6).
                if msg.xy_valid:
                    d.pos_valid = True
                if not d.vsp_init and msg.xy_valid:
                    d.vsp = [msg.x, msg.y, msg.z]
                    d.vsp_init = True
        return _cb

    def _make_rth_cb(self, idx: int):
        def _cb(msg: Point) -> None:
            with self._lock:
                d = self._d[idx]
                if d.rth_active:
                    return
                d.rth_vsp    = [msg.x, msg.y, msg.z]
                d.rth_active = True
                self.get_logger().info(
                    f"{d.did}: RTH target received NED({msg.x:.2f},{msg.y:.2f}) — locking input"
                )
            self._flash(f"{d.did.upper()} RTH — returning to pad, manual input locked")
        return _cb

    def _landed_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            drone_id = data.get("drone_id")
        except json.JSONDecodeError:
            return

        with self._lock:
            for d in self._d:
                if d.did == drone_id and d.rth_active:
                    d.rth_active = False
                    self.get_logger().info(f"{d.did}: landed confirmation received — unlocking input")
                    self._flash(f"{d.did.upper()} landed — manual input unlocked")

    def _mission_ready_cb(self, msg: String) -> None:
        """Mission is starting — swarm_agents are taking over.
        Stop publishing offboard setpoints so they don't conflict."""
        with self._lock:
            if self._mission_started:
                return
            self._mission_started = True
            self._status_msg = "Mission started — swarm agents in control. Press Q to quit."
        self._flash("Mission started! Swarm agents are flying. Press Q to quit.")
        self.get_logger().info("mission_ready received — manual_controller going passive")

    # ── 10 Hz timer ───────────────────────────────────────────────────────────
    def _timer_cb(self) -> None:
        with self._lock:
            if self._mission_started:
                return   # swarm_agents have taken over — stop all offboard output

            for i, d in enumerate(self._d):
                d.ticks += 1
                # Arm once position valid and enough setpoints sent
                if not d.armed and d.ticks >= ARM_TICKS and d.vsp_init:
                    self._arm(i)
                    d.armed  = True
                    d.phase  = Phase.TAKEOFF
                    d.vsp[2] = d.z   # snap z to ground

                # Takeoff: ramp VSP z toward target altitude
                if d.phase == Phase.TAKEOFF:
                    step = MANUAL_SPEED * DT
                    dz   = self._target_z - d.vsp[2]
                    if abs(dz) > step:
                        d.vsp[2] += math.copysign(step, dz)
                    else:
                        d.vsp[2] = self._target_z
                    if d.pos_valid and abs(d.z - self._target_z) < ALT_TOL:
                        d.phase = Phase.FLY
                        if i == 0:
                            self._status_msg = (
                                "WSAD=fly  Tab=switch drone  H=pad_0  J=pad_1  "
                                "C=corner  M=start mission  ↑↓=alt  L=land"
                            )

                if d.landing:
                    d.landing_ticks += 1

                # Autonomous RTH movement (step toward rth_vsp)
                if d.rth_active and not d.landing:
                    # XY first, then Z
                    step_xy = MANUAL_SPEED * DT
                    dx = d.rth_vsp[0] - d.vsp[0]
                    dy = d.rth_vsp[1] - d.vsp[1]
                    dist_sq = dx**2 + dy**2
                    if dist_sq > step_xy**2:
                        dist = math.sqrt(dist_sq)
                        d.vsp[0] += (dx / dist) * step_xy
                        d.vsp[1] += (dy / dist) * step_xy
                    else:
                        d.vsp[0] = d.rth_vsp[0]
                        d.vsp[1] = d.rth_vsp[1]
                        # XY reached — descend
                        step_z = ALT_SPEED * DT
                        dz = d.rth_vsp[2] - d.vsp[2]
                        if abs(dz) > step_z:
                            d.vsp[2] += math.copysign(step_z, dz)
                        else:
                            d.vsp[2] = d.rth_vsp[2]
                            # At pad — trigger land
                            self._land(i)
                            self.get_logger().info(f"{d.did}: at RTH target — triggering AUTO.LAND")

            vsps           = [list(d.vsp) for d in self._d]
            phases         = [d.phase for d in self._d]
            landing_ticks  = [d.landing_ticks for d in self._d]
            landing_active = [d.landing for d in self._d]
            drone_ids      = [d.did for d in self._d]

        for i in range(len(self._d)):
            if landing_active[i]:
                # Heartbeat is intentionally NOT published for this drone so PX4
                # exits offboard mode (~0.5 s).  After 3 ticks (0.3 s) we can
                # safely send the AUTO.LAND mode switch.
                if landing_ticks[i] == 3:
                    self._send_command(
                        i, VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                        param1=1.0, param2=4.0, param3=6.0)
                    self.get_logger().info(
                        f"drone_{i}: AUTO.LAND sent (offboard heartbeat stopped)")
                    
                    # Notify other nodes (and ourselves via callback) that we've committed to landing
                    lc_msg = String()
                    lc_msg.data = json.dumps({"drone_id": drone_ids[i]})
                    self._landed_pub.publish(lc_msg)
                continue
            self._pub_offboard(i)
            self._pub_setpoint(i, vsps[i], phases[i])

    # ── Publisher helpers ─────────────────────────────────────────────────────
    def _pub_offboard(self, idx: int) -> None:
        msg = OffboardControlMode()
        msg.position  = True
        msg.velocity  = False
        msg.timestamp = self._now_us()
        self._offboard_pubs[idx].publish(msg)

    def _pub_setpoint(self, idx: int, vsp: list[float], phase: Phase) -> None:
        nan = float("nan")
        msg = TrajectorySetpoint()
        msg.position     = [vsp[0], vsp[1], vsp[2]]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = nan
        msg.timestamp    = self._now_us()
        self._traj_pubs[idx].publish(msg)

    def _arm(self, idx: int) -> None:
        self._send_command(idx, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self._send_command(idx, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _send_command(self, idx: int, command: int, **kwargs) -> None:
        d = self._d[idx]
        msg = VehicleCommand()
        msg.command          = command
        msg.target_system    = d.drone_id + 1   # MAVLink sysid
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._now_us()
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self._cmd_pubs[idx].publish(msg)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    # ── Pad assignment ────────────────────────────────────────────────────────
    def _assign_pad(self, drone_idx: int, pad_id: str) -> None:
        """Record drone's current position as pad, publish to /swarm/pad_assignment
        and /drone_N/rth_target."""
        with self._lock:
            d = self._d[drone_idx]
            x, y = d.x, d.y

            # Guard 1: no message received at all or EKF not converged
            if not d.pos_valid or not d.xy_valid:
                self._flash(
                    f"{pad_id.upper()} REJECTED: drone_{drone_idx} EKF not ready — wait for fix"
                )
                self.get_logger().warn(
                    f"_assign_pad: drone_{drone_idx} EKF not ready — pad NOT saved"
                )
                return

            # Guard 2: Drone near origin (spawn artifact)
            if abs(x) < 0.5 and abs(y) < 0.5:
                self._flash(
                    f"{pad_id.upper()} REJECTED: drone near origin NED({x:.2f}, {y:.2f}) — move to pad first"
                )
                self.get_logger().warn(
                    f"_assign_pad: drone_{drone_idx} near origin NED({x:.2f},{y:.2f}) — pad NOT saved"
                )
                return

        pad_key = pad_id
        did     = f"drone_{drone_idx}"
        ned_z   = -0.5  # landing hover height NED

        # /swarm/pad_assignment — includes coordinates
        payload = {
            "drone_id": did,
            "pad_id":   pad_id,
            "x":        round(x, 3),
            "y":        round(y, 3),
            "z":        ned_z,
        }
        msg_s = String()
        msg_s.data = json.dumps(payload)
        self._pad_assign_pub.publish(msg_s)

        # /drone_N/rth_target — geometry_msgs/Point
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = ned_z
        self._rth_pubs[did].publish(pt)

        with self._lock:
            self._pads[pad_key] = (x, y)

        # Show saved coordinates prominently so operator can verify
        self._flash(f"{pad_id.upper()} SET: NED({x:.2f}, {y:.2f}) — OK")
        self.get_logger().info(
            f"Pad assigned | {did} → {pad_id} NED({x:.2f},{y:.2f}) [xy_valid=True]"
        )

    # ── Corner marking ────────────────────────────────────────────────────────
    def _mark_corner(self, label: str) -> None:
        """Publish drone_0's current position as a field corner."""
        with self._lock:
            d0 = self._d[0]
            if not d0.pos_valid:
                self._flash("No drone_0 position — fly first")
                return
            x, y, z = d0.x, d0.y, d0.z

        payload = {
            "corner": label,
            "ned":    {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._corner_pub.publish(msg)

        with self._lock:
            self._corners[label] = (x, y, z)
        self._flash(f"Corner {label} marked: NED({x:.2f}, {y:.2f})")
        self.get_logger().info(f"Corner {label} → NED({x:.2f},{y:.2f},{z:.2f})")

    # ── Land ──────────────────────────────────────────────────────────────────
    def _land(self, drone_idx: int) -> None:
        """Trigger AUTO.LAND for one drone.

        PX4 ignores VEHICLE_CMD_NAV_LAND while the offboard heartbeat is
        running (offboard setpoints keep overriding the mode change).
        Correct sequence (same as swarm_agent RTH):
          1. Stop publishing OffboardControlMode + TrajectorySetpoint (heartbeat).
          2. Wait ~0.3 s for PX4 to exit offboard mode automatically.
          3. Send VEHICLE_CMD_DO_SET_MODE → AUTO.LAND.
        The landing_ticks counter in _timer_cb drives steps 2→3.
        """
        with self._lock:
            d = self._d[drone_idx]
            d.landing       = True
            d.landing_ticks = 0
            d.armed         = False
            d.phase         = Phase.IDLE

    # ── Flash helper ──────────────────────────────────────────────────────────
    def _flash(self, msg: str) -> None:
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
            rem = frame_time - elapsed
            if rem > 0:
                time.sleep(rem)

    def _handle_key(self, key: int) -> None:
        # Corner submenu: next key 1-4 selects corner label
        with self._lock:
            submenu = self._corner_submenu
        if submenu:
            with self._lock:
                self._corner_submenu = False
            if key in CORNER_LABELS:
                self._mark_corner(CORNER_LABELS[key])
            else:
                self._flash("Corner cancelled (press C then 1/2/3/4)")
            return

        if key in (ord('q'), ord('Q')):
            self._quit = True

        elif key == ord('\t'):   # Tab — switch active drone
            with self._lock:
                self._active = 1 - self._active
            active = self._active
            self._flash(f"Active drone: drone_{active}")

        elif key in (ord('h'), ord('H')):
            self._assign_pad(0, "pad_0")

        elif key in (ord('j'), ord('J')):
            self._assign_pad(1, "pad_1")

        elif key in (ord('c'), ord('C')):
            with self._lock:
                self._corner_submenu = True
            self._flash("Mark corner: [1]NE  [2]NW  [3]SE  [4]SW")

        elif key in (ord('m'), ord('M')):
            confirm_msg = String()
            confirm_msg.data = json.dumps({"operator": "confirmed"})
            self._mission_confirm_pub.publish(confirm_msg)
            self._flash("Mission confirmed — starting spray mission!")
            self.get_logger().info("Operator pressed M — mission confirm published")

        elif key in (ord('l'), ord('L')):
            with self._lock:
                active = self._active
            self._land(active)
            self._flash(f"Land command → drone_{active}")

        # ── Altitude up/down (arrow keys) ─────────────────────────────────
        elif key == curses.KEY_UP:
            with self._lock:
                active = self._active
                locked = self._d[active].rth_active
                if not locked:
                    self._d[active].vsp[2] -= ALT_STEP   # NED: more negative = higher
        elif key == curses.KEY_DOWN:
            with self._lock:
                active = self._active
                locked = self._d[active].rth_active
                if not locked:
                    self._d[active].vsp[2] += ALT_STEP

        # ── WSAD movement ─────────────────────────────────────────────────
        else:
            with self._lock:
                active = self._active
                fly    = self._d[active].phase == Phase.FLY
                locked = self._d[active].rth_active
            if fly and not locked:
                dx, dy = 0.0, 0.0
                if key in (ord('w'), ord('W')):
                    dx = +UI_STEP   # North (+x NED)
                elif key in (ord('s'), ord('S')):
                    dx = -UI_STEP   # South (-x NED)
                elif key in (ord('a'), ord('A')):
                    dy = -UI_STEP   # West  (-y NED)
                elif key in (ord('d'), ord('D')):
                    dy = +UI_STEP   # East  (+y NED)
                if dx != 0.0 or dy != 0.0:
                    with self._lock:
                        self._d[active].vsp[0] += dx
                        self._d[active].vsp[1] += dy

    # ── Drawing ───────────────────────────────────────────────────────────────
    @staticmethod
    def _safe_addstr(stdscr, y, x, text, attr=0):
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
        sa = self._safe_addstr

        with self._lock:
            active   = self._active
            d0       = self._d[0]
            d1       = self._d[1]
            corners  = dict(self._corners)
            pads     = dict(self._pads)
            status   = self._status_msg
            submenu  = self._corner_submenu

        # ── Title bar ─────────────────────────────────────────────────────
        title = (
            f"  Manual Controller  |  alt={self._altitude:.0f} m  |  "
            f"Active: drone_{active}  [Tab=switch]"
        )
        sa(stdscr, 0, 0, title[:w], curses.color_pair(CP_TITLE) | curses.A_BOLD)

        # ── Drone positions ────────────────────────────────────────────────
        for i, (d, cp) in enumerate([(d0, CP_ACCENT), (d1, CP_DRONE1)]):
            mark = " ◄" if i == active else "  "
            row = 1 + i
            if d.pos_valid:
                # xy_valid shown so operator can see when EKF is ready before pressing H/J
                xy_tag = "[xy OK]" if d.xy_valid else "[xy?]"
                phase_name = "RTH" if d.rth_active else d.phase.name
                s = (f"  drone_{i}{mark}  x={d.x:7.2f}  y={d.y:7.2f}  "
                     f"z={d.z:6.2f}  alt={-d.z:.2f}m  {phase_name}  {xy_tag}")
            else:
                xy_tag = "(EKF not ready)" if not d.xy_valid else ""
                s = f"  drone_{i}{mark}  waiting for position… {xy_tag}"
            sa(stdscr, row, 0, s[:w], curses.color_pair(cp))
            if d.rth_active:
                sa(stdscr, row, w-45, " [RTH ACTIVE - INPUT LOCKED] ", curses.color_pair(CP_ARMED) | curses.A_BOLD)

        # ── Separator ─────────────────────────────────────────────────────
        sa(stdscr, 3, 0, "─" * (w - 1), curses.color_pair(CP_DIM) | curses.A_DIM)

        # ── Landing pads ──────────────────────────────────────────────────
        sa(stdscr, 4, 2, "Landing pads:", curses.color_pair(CP_HOME) | curses.A_BOLD)
        p0 = pads.get("pad_0")
        p1 = pads.get("pad_1")
        p0_str = f"NED({p0[0]:.2f},{p0[1]:.2f})" if p0 else "not set — fly drone_0 → press H"
        p1_str = f"NED({p1[0]:.2f},{p1[1]:.2f})" if p1 else "not set — fly drone_1 → press J"
        sa(stdscr, 5, 4, f"pad_0 (drone_0): {p0_str}", curses.color_pair(CP_HOME))
        sa(stdscr, 6, 4, f"pad_1 (drone_1): {p1_str}", curses.color_pair(CP_DRONE1))

        # ── Field corners ──────────────────────────────────────────────────
        sa(stdscr, 8, 2,
           f"Field corners ({len(corners)}/4) — C then 1/2/3/4 to mark:",
           curses.color_pair(CP_CORNER) | curses.A_BOLD)
        for i, lbl in enumerate(['NE', 'NW', 'SE', 'SW']):
            c = corners.get(lbl)
            if c:
                s = f"{lbl}: NED({c[0]:.2f}, {c[1]:.2f})"
            else:
                s = f"{lbl}: ---"
            row = 9 + i
            if row < h - 6:
                sa(stdscr, row, 4, s, curses.color_pair(CP_CORNER if c else CP_DIM))

        # ── Corner submenu ─────────────────────────────────────────────────
        if submenu:
            sa(stdscr, h - 6, 2, "  Mark corner: [1]=NE  [2]=NW  [3]=SE  [4]=SW  [other]=cancel  ",
               curses.color_pair(CP_ARMED) | curses.A_BOLD)

        # ── Flash message ──────────────────────────────────────────────────
        flash_row = h - 4
        if self._flash_msg and (time.monotonic() - self._flash_time) < 2.5:
            sa(stdscr, flash_row, 2, f"  {self._flash_msg[:w - 6]}  ",
               curses.color_pair(CP_ARMED) | curses.A_BOLD)

        # ── Status + footer ────────────────────────────────────────────────
        sa(stdscr, h - 3, 0, "─" * (w - 1), curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, h - 2, 1, status[:w - 2], curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, h - 1, 1,
           "W/S=N/S  A/D=W/E  ↑↓=alt  Tab=switch  H=pad0  J=pad1  C=corner  M=start mission  L=land  Q=quit",
           curses.color_pair(CP_ACCENT))

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(node.run_ui)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
