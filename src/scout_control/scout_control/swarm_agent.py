"""
swarm_agent.py — Mission executor / route provider for one drone in the swarm

One instance per drone. Receives target cells from task_allocator,
forwards high-level targets to obstacle_avoidance_runtime, and reports
progress back to swarm_coordinator/task_allocator.

State machine:
  IDLE → TAKEOFF → ROTATE → CRUISE → HOVER → (next cell)
                                   ↓
                               AVOIDING → CRUISE (resume)
                                           → RTH → LAND

Interfaces:
  Subscribe:
    /drone_N/next_cell            String  "x4_y2"          from task_allocator
    /swarm/rth_request            String  JSON              mission complete / RTH
    /drone_N/obstacles/detected   String  JSON              from obstacle_detector
    /fmu/out/vehicle_local_position_v1  (drone 0)
    /px4_N/fmu/out/...                  (drone N)

  Publish:
    /swarm/drone_status      String  JSON
      {"drone_id":"drone_0","status":"READY"}
      {"drone_id":"drone_0","status":"CELL_COMPLETE","cell_id":"x4_y2"}
    /drone_N/avoidance/target_cmd  String JSON

Parameters:
  drone_id     int    0          which drone instance (0, 1, 2…)
  altitude_m   float  5.0        cruise altitude above ground
  home_ned_x   float  0.0        RTH target NED x
  home_ned_y   float  0.0        RTH target NED y (drone_0=0.0, drone_1=-3.0)
  cruise_speed float  2.0        m/s horizontal
  hover_secs   float  1.0        seconds to hover at cell before reporting done

Usage:
  ros2 run scout_control swarm_agent --ros-args -p drone_id:=0
  ros2 run scout_control swarm_agent --ros-args -p drone_id:=1
"""

import json
import math
import threading
from collections import deque
from enum import Enum, auto
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

# ── QoS ──────────────────────────────────────────────────────────────────────
QOS_PX4_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PX4_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── Constants ─────────────────────────────────────────────────────────────────
DT             = 0.1    # s — 10 Hz control loop
ARM_TICKS      = 10     # pre-arm setpoints before arming
ALT_TOL        = 0.4    # m — altitude reached threshold
REACH_DIST     = 0.8    # m — cell considered reached (larger than field_commander: coarser grid)
RTH_HOVER_Z    = -1.5   # m NED — altitude at which AUTO.LAND is triggered
RTH_SPEED      = 0.8    # m/s descent
YAW_RATE       = math.radians(40)  # rad/s
YAW_TOL        = math.radians(10)
ROTATE_TICKS   = 8
MAX_ROTATE_TICKS = 60
IDLE_RTH_TICKS   = 60   # 6 s idle at empty queue → self-trigger RTH (safety net)
MAX_TAKEOFF_TICKS = 200  # 20 s max in TAKEOFF before forcing transition (bad lidar guard)

# ── Obstacle avoidance ───────────────────────────────────────────────────────
AVOID_OFFSET_M      = 3.0    # m — perpendicular offset of detour waypoint
AVOID_TIMEOUT_TICKS = 100    # 10 s max in AVOIDING before RTH
OBSTACLE_STOP_DIST  = 2.0    # m — transition to AVOIDING
OBSTACLE_WARN_DIST  = 4.0    # m — start slowing down (cruise_speed * 0.5)

# ── Terrain following ─────────────────────────────────────────────────────────
RANGE_MIN_OK       = 0.15   # m — reject self-hit clamps (sensor range_min=0.1 → always reports
                             #     0.1 on self-hit; threshold > 0.1 filters these out)
RANGE_MAX_OK       = 80.0   # m — ignore lidar above this (out of range / no return)
ALT_DEADBAND       = 0.5    # m — don't correct if range error < this (wider band → less chatter)
TERRAIN_KP         = 1.2    # P gain [m/s per m error] — same as terrain_follower.py
TERRAIN_VZ_MAX     = 1.0    # m/s — max vertical velocity for terrain correction
                             # Kept low to avoid oscillation at high cruise speeds
NAV_BACKEND_DIRECT = "direct"
NAV_BACKEND_AVOIDANCE_RUNTIME = "avoidance_runtime"


class Phase(Enum):
    IDLE    = auto()
    TAKEOFF = auto()
    ROTATE  = auto()
    CRUISE  = auto()
    AVOIDING = auto()
    RTH     = auto()


class SwarmAgent(Node):

    def __init__(self) -> None:
        super().__init__("swarm_agent")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",     0)
        self.declare_parameter("altitude_m",   5.0)
        self.declare_parameter("home_ned_x",   0.0)
        self.declare_parameter("home_ned_y",   0.0)
        self.declare_parameter("cruise_speed", 2.0)
        self.declare_parameter("navigation_backend", NAV_BACKEND_AVOIDANCE_RUNTIME)

        self._drone_id:     int   = self.get_parameter("drone_id").value
        self._altitude:     float = self.get_parameter("altitude_m").value
        self._home_x:       float = self.get_parameter("home_ned_x").value
        self._home_y:       float = self.get_parameter("home_ned_y").value
        self._cruise_speed: float = self.get_parameter("cruise_speed").value
        self._did = f"drone_{self._drone_id}"
        backend_raw = str(self.get_parameter("navigation_backend").value)
        self._navigation_backend: str = self._normalize_navigation_backend(backend_raw)
        self._runtime_backend_active = (
            self._navigation_backend == NAV_BACKEND_AVOIDANCE_RUNTIME
        )

        # PX4 topic prefix: drone 0 = bare, drone N = /px4_N/
        _px4_ns = "" if self._drone_id == 0 else f"/px4_{self._drone_id}"

        self._lock   = threading.Lock()
        self._ticks  = 0

        # ── Flight state ──────────────────────────────────────────────────────
        self._phase:           Phase         = Phase.IDLE
        self._armed:           bool          = False
        self._arm_requested:   bool          = False
        self._land_requested:  bool          = False
        self._rth_active:      bool          = False
        self._rth_land_now:    bool          = False
        self._on_ground:       bool          = False   # True after AUTO.LAND; allows re-arm

        self._drone_x:  float = 0.0
        self._drone_y:  float = 0.0
        self._drone_z:  float = 0.0
        self._drone_yaw: float = 0.0
        self._pos_valid: bool = False
        self._vsp: list[float] = [0.0, 0.0, 0.0]
        self._vsp_initialized: bool = False
        self._vsp_yaw: float = float("nan")

        self._target_x:      float = 0.0
        self._target_y:      float = 0.0
        self._target_z:      float = -self._altitude   # NED (negative = up)
        self._rotate_ticks:  int   = 0
        self._takeoff_ticks: int   = 0

        # ── Terrain following (lidar) ─────────────────────────────────────────
        self._range:        float = 0.0
        self._range_valid:  bool  = False
        self._vz:           float = 0.0   # vertical velocity command from terrain P-controller

        # ── Task state ────────────────────────────────────────────────────────
        self._current_cell_id: Optional[str]        = None
        self._cell_queue:      deque                = deque()   # upcoming cells (cid, x, y)
        self._cell_reached:    bool                 = False     # VSP reached current target
        self._waiting_for_next: bool                = False     # at target, queue empty
        self._mission_done:    bool                 = False
        self._idle_ticks:      int                  = 0         # ticks spent idle (for RTH safety net)
        self._landing:         bool                 = False     # True after AUTO.LAND command sent
        self._just_armed:      bool                 = False     # pulsed True for one tick after arming
        self._passive:         bool                 = True      # silent until /swarm/mission_ready

        # ── Obstacle avoidance ────────────────────────────────────────────────
        self._obstacle_closest:     float          = 99.0
        self._obstacle_sectors:     dict           = {"left": 99.0, "center": 99.0, "right": 99.0}
        self._obstacle_critical:    bool           = False
        self._free_directions:      list           = ["left", "center", "right"]
        self._avoid_resume_target:  tuple          = (0.0, 0.0)   # original target before avoidance
        self._avoid_waypoint:       tuple          = (0.0, 0.0)   # lateral detour point
        self._avoid_ticks:          int            = 0
        self._avoiding_resume_cell: Optional[str]  = None         # for status publish on entry
        self._last_avoidance_payload: Optional[dict[str, Any]] = None
        self._last_runtime_status_signature: Optional[tuple[Any, ...]] = None
        self._runtime_active_cell_id: Optional[str] = None
        self._runtime_last_completed_target_id: str = ""
        self._runtime_rth_requested: bool = False
        self._runtime_return_home_sent: bool = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._offboard_pub = None
        self._traj_pub = None
        self._cmd_pub = None
        if not self._runtime_backend_active:
            self._offboard_pub = self.create_publisher(
                OffboardControlMode,
                f"{_px4_ns}/fmu/in/offboard_control_mode", QOS_PX4_PUB)
            self._traj_pub = self.create_publisher(
                TrajectorySetpoint,
                f"{_px4_ns}/fmu/in/trajectory_setpoint", QOS_PX4_PUB)
            self._cmd_pub = self.create_publisher(
                VehicleCommand,
                f"{_px4_ns}/fmu/in/vehicle_command", QOS_PX4_PUB)
        self._target_cmd_pub = self.create_publisher(
            String,
            f"/{self._did}/avoidance/target_cmd",
            QOS_VOL,
        )
        self._status_pub = self.create_publisher(
            String, "/swarm/drone_status", QOS_VOL)
        self._landed_pub = self.create_publisher(
            String, "/swarm/landed_confirmation", QOS_SENSOR)

        # ── Subscribers ───────────────────────────────────────────────────────
        if not self._runtime_backend_active:
            self.create_subscription(
                VehicleLocalPosition,
                f"{_px4_ns}/fmu/out/vehicle_local_position_v1",
                self._pos_cb, QOS_PX4_SUB)
        # Use depth=10 for next_cell so that a primary cell and its prefetch
        # arriving in rapid succession are never dropped (depth=1 could silently
        # discard the primary cell before the callback runs, causing the drone
        # to skip straight to the prefetched cell — Bug 2).
        _qos_next_cell = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            String, f"/{self._did}/next_cell",
            self._next_cell_cb, _qos_next_cell)
        self.create_subscription(
            String, "/swarm/rth_request",
            self._rth_request_cb, QOS_VOL)

        if not self._runtime_backend_active:
            # Subscribe to /drone_N/rth_target so that home position can be updated
            # dynamically by manual_controller (H/J keys) or field_setup_coordinator.
            # A message received BEFORE an rth_request updates home_x/y for the
            # upcoming RTH manoeuvre.
            self.create_subscription(
                Point,
                f"/{self._did}/rth_target",
                self._rth_target_cb,
                QOS_LATCHED,
            )

            # Downward lidar for terrain following.
            # Topic is per-drone: /drone_N/downward_lidar/scan
            # Bridged from Gazebo by the lidar bridge nodes in full_e2e_mission.launch.py.
            # If lidar is unavailable (model without lidar) swarm_agent falls back to
            # fixed altitude (_altitude parameter).
            self.create_subscription(
                LaserScan,
                f"/{self._did}/downward_lidar/scan",
                self._lidar_cb, QOS_SENSOR)

            # Obstacle detector output — per-drone JSON with closest distance + sector map.
            self.create_subscription(
                String,
                f"/{self._did}/obstacles/detected",
                self._on_obstacle, QOS_SENSOR)
        if self._navigation_backend == NAV_BACKEND_AVOIDANCE_RUNTIME:
            self.create_subscription(
                String,
                f"/{self._did}/avoidance/status",
                self._avoidance_status_cb,
                QOS_VOL,
            )

        # Subscribe to /swarm/mission_ready — arm only after setup is complete.
        # VOLATILE: intentionally ignores stale latched messages from previous
        # sessions. swarm_agents are started at launch time (2 s delay), well
        # before the operator finishes setup, so they will always be alive when
        # field_setup_coordinator publishes mission_ready.
        _qos_mission_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/swarm/mission_ready",
            self._mission_ready_cb, _qos_mission_volatile)

        if not self._runtime_backend_active:
            self.create_timer(DT, self._timer_cb)

        self.get_logger().info(
            f"SwarmAgent {self._did} ready | "
            f"alt={self._altitude}m | home NED({self._home_x},{self._home_y}) | "
            f"cruise={self._cruise_speed}m/s | backend={self._navigation_backend} | "
            f"runtime_backend_active={self._runtime_backend_active} | "
            "waiting for /swarm/mission_ready"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._drone_x   = msg.x
            self._drone_y   = msg.y
            self._drone_z   = msg.z
            self._drone_yaw = msg.heading
            self._pos_valid = True
            if not self._vsp_initialized:
                self._vsp = [msg.x, msg.y, msg.z]
                self._vsp_initialized = True

    def _next_cell_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            tx  = float(data["x"])
            ty  = float(data["y"])
            cid = str(data["cell_id"])
        except (json.JSONDecodeError, KeyError, ValueError):
            self.get_logger().warn(
                f"next_cell: bad payload '{msg.data[:80]}' — expected JSON with x,y,cell_id"
            )
            return

        with self._lock:
            if self._mission_done:
                return
            if self._runtime_backend_active:
                self._cell_queue.append((cid, tx, ty))
                self.get_logger().info(
                    f"{self._did}: queued {cid} NED({tx:.2f},{ty:.2f}) "
                    f"[queue={len(self._cell_queue)}]"
                )
                cmd = self._maybe_build_runtime_cmd_locked()
                if cmd is not None:
                    self.get_logger().info(
                        f"{self._did}: dispatching runtime target {cmd['target_id']} "
                        f"NED({cmd['target_ned'][0]:.2f},{cmd['target_ned'][1]:.2f})"
                    )
                else:
                    return
            else:
                cmd = None

                if self._waiting_for_next:
                    # Drone is holding at current cell centre — activate immediately
                    self._current_cell_id  = cid
                    self._target_x         = tx
                    self._target_y         = ty
                    self._cell_reached     = False
                    self._waiting_for_next = False
                    self._phase            = Phase.ROTATE
                    self._rotate_ticks     = 0
                    self._vsp_yaw          = float("nan")
                    self.get_logger().info(
                        f"{self._did}: resuming → {cid} NED({tx:.2f},{ty:.2f})"
                    )
                else:
                    # Queue for seamless pick-up when drone reaches current target.
                    # This includes cells that arrive during TAKEOFF phase — they are
                    # always queued here and activated at the TAKEOFF→ROTATE transition
                    # (see _step_vsp).  Previously, the first cell was "activated
                    # directly" during TAKEOFF, but that could race with the prefetch:
                    # if both primary and prefetch messages arrived before the callback
                    # ran (depth=1 queue) the primary was dropped and the drone jumped
                    # straight to the prefetch cell (Bug 2).
                    self._cell_queue.append((cid, tx, ty))
                    self.get_logger().info(
                        f"{self._did}: queued {cid} NED({tx:.2f},{ty:.2f}) "
                        f"[queue={len(self._cell_queue)}]"
                    )
        if cmd is not None:
            self._publish_runtime_cmd(cmd)

    def _rth_request_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("drone_id") != self._did:
            return
        with self._lock:
            if self._rth_active or self._mission_done:
                return
            if self._runtime_backend_active:
                self._rth_active = True
                self._mission_done = True
                self._runtime_rth_requested = True
                self._runtime_return_home_sent = True
                self._cell_queue.clear()
                self._runtime_active_cell_id = None
                cmd = self._build_runtime_return_home_cmd_locked()
            else:
                cmd = None
                self._rth_active      = True
                self._mission_done    = True
                self._target_x        = self._home_x
                self._target_y        = self._home_y
                self._phase           = Phase.ROTATE
                self._rotate_ticks    = 0
                self._vsp_yaw         = float("nan")
            if self._runtime_backend_active:
                self.get_logger().info(f"{self._did}: RTH requested (runtime backend)")
            else:
                self.get_logger().info(
                    f"{self._did}: RTH → home NED({self._home_x},{self._home_y})"
                )
        if cmd is not None:
            self._publish_runtime_cmd(cmd)

    def _rth_target_cb(self, msg: Point) -> None:
        """Update home position dynamically when /drone_N/rth_target is received.

        Published by manual_controller (H/J keys) or home_manager on RTH request.
        Updates _home_x/_home_y so the next RTH manoeuvre goes to the right pad.
        """
        with self._lock:
            self._home_x = float(msg.x)
            self._home_y = float(msg.y)
            # If already in RTH, update live target so drone heads to new home
            if self._rth_active and self._phase in (Phase.ROTATE, Phase.RTH):
                self._target_x = self._home_x
                self._target_y = self._home_y
        self.get_logger().info(
            f"{self._did}: home position updated → NED({msg.x:.2f},{msg.y:.2f})"
        )

    def _mission_ready_cb(self, msg: String) -> None:
        """Received /swarm/mission_ready — arm and start takeoff.

        Called once after field setup is complete and the operator presses M.
        Before this message arrives swarm_agent stays passive (no offboard output)
        so it doesn't conflict with manual_controller during the setup phase.

        Also handles re-arm after a previous mission: if the drone has landed
        (_on_ground=True) a new mission_ready clears mission state and arms again.
        """
        with self._lock:
            if self._runtime_backend_active:
                self._passive = False
                self._idle_ticks = 0
                self._mission_done = False
                self._rth_active = False
                self._cell_queue.clear()
                self._waiting_for_next = False
                self._runtime_active_cell_id = None
                self._runtime_last_completed_target_id = ""
                self._runtime_rth_requested = False
                self._runtime_return_home_sent = False
                self._last_runtime_status_signature = None
                self.get_logger().info(
                    f"{self._did}: /swarm/mission_ready received — runtime backend mission mode active"
                )
                return
            if self._arm_requested or (self._armed and not self._on_ground):
                return   # already armed and airborne — ignore duplicate
            self._passive          = False
            self._arm_requested    = True
            # Reset mission state so re-arm works cleanly after previous landing
            self._landing          = False   # restore offboard heartbeat
            self._mission_done     = False
            self._rth_active       = False
            self._on_ground        = False
            self._cell_queue.clear()
            self._current_cell_id  = None
            self._waiting_for_next = False
            self._cell_reached     = False
            self._idle_ticks       = 0
            self._takeoff_ticks    = 0
        self.get_logger().info(
            f"{self._did}: /swarm/mission_ready received — arming and taking off"
        )

    def _lidar_cb(self, msg: LaserScan) -> None:
        """Store latest downward lidar range for terrain following."""
        if not msg.ranges:
            return
        r = msg.ranges[0]
        with self._lock:
            if RANGE_MIN_OK <= r <= RANGE_MAX_OK:
                self._range       = r
                self._range_valid = True
            # else: out-of-range reading — keep last valid value

    def _on_obstacle(self, msg: String) -> None:
        """Update obstacle state from /drone_N/obstacles/detected JSON payload."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._obstacle_closest  = float(data.get("closest", 99.0))
            self._obstacle_sectors  = data.get("sectors", {"left": 99.0, "center": 99.0, "right": 99.0})
            self._obstacle_critical = bool(data.get("critical", False))
            self._free_directions   = data.get("free_directions", ["left", "center", "right"])

    def _avoidance_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        cell_complete_id: Optional[str] = None
        runtime_cmd: Optional[dict[str, Any]] = None
        with self._lock:
            self._last_avoidance_payload = payload
            if self._runtime_backend_active:
                completed_id_raw = str(payload.get("last_completed_target_id", "")).strip()
                if (
                    completed_id_raw
                    and completed_id_raw != self._runtime_last_completed_target_id
                ):
                    self._runtime_last_completed_target_id = completed_id_raw
                    if completed_id_raw == self._runtime_active_cell_id:
                        cell_complete_id = completed_id_raw
                        self._runtime_active_cell_id = None
                        self._waiting_for_next = False
                runtime_cmd = self._maybe_build_runtime_cmd_locked()

            mapped = self._build_swarm_status_from_avoidance(payload)
            if mapped is None or not self._runtime_backend_active:
                return
            status, extra = mapped
            signature = (
                status,
                extra.get("phase"),
                extra.get("target_id"),
                extra.get("planner_state"),
                extra.get("scan_state"),
                extra.get("blocked_severity"),
                extra.get("runtime_event"),
                extra.get("last_completed_target_id"),
            )
            if signature == self._last_runtime_status_signature:
                return
            self._last_runtime_status_signature = signature
        self._pub_status(status, **extra)
        if cell_complete_id:
            self._pub_cell_complete(cell_complete_id)
        if runtime_cmd is not None:
            self._publish_runtime_cmd(runtime_cmd)

    def _build_swarm_status_from_avoidance(
        self, payload: dict[str, Any]
    ) -> Optional[tuple[str, dict[str, Any]]]:
        phase = str(payload.get("phase", "")).upper()
        if not phase:
            return None

        if phase == "IDLE":
            status = "READY" if bool(payload.get("navigator_ready", False)) else "IDLE"
        else:
            phase_to_status = {
                "TAKEOFF": "TAKEOFF",
                "CRUISE_TO_TARGET": "NAVIGATING",
                "WARN_DRIFT": "AVOIDANCE_WARN",
                "STOP_HOVER": "HOVER",
                "SCAN_360": "SCAN_360",
                "LOCAL_REPLAN": "LOCAL_REPLAN",
                "DETOUR_EXECUTION": "AVOIDING",
                "BLOCKED": "BLOCKED",
                "RETURN_HOME": "RTH",
                "LANDING": "LANDING",
                "ABORT": "ABORT",
            }
            status = phase_to_status.get(phase)
            if status is None:
                return None

        event = payload.get("last_runtime_event") or {}
        runtime_event = str(event.get("event", "")).strip() if isinstance(event, dict) else ""
        blocked_severity = payload.get("blocked_severity")
        blocked_reason = payload.get("blocked_reason")
        blocked_since_s = payload.get("blocked_since_s")
        extra = {
            "backend": NAV_BACKEND_AVOIDANCE_RUNTIME,
            "navigation_backend": NAV_BACKEND_AVOIDANCE_RUNTIME,
            "phase": phase,
            "command": str(payload.get("command", "")),
            "target_id": str(payload.get("target_id", "")),
            "target_name": str(payload.get("target_name", "")),
            "target_ned": payload.get("target_ned"),
            "subgoal_ned": payload.get("subgoal_ned"),
            "planner_state": str(payload.get("planner_state", "")),
            "planner_mode": str(payload.get("planner_mode", "")),
            "scan_state": str(payload.get("scan_state", "")),
            "no_path_streak": int(payload.get("no_path_streak", 0)),
            "scan_attempts_for_target": int(payload.get("scan_attempts_for_target", 0)),
            "blocked_reason": None if blocked_reason is None else str(blocked_reason),
            "blocked_severity": None if blocked_severity is None else str(blocked_severity),
            "blocked_since_s": None if blocked_since_s is None else float(blocked_since_s),
            "dense_scan_points": int(payload.get("dense_scan_points", 0)),
            "mapper_state": str(payload.get("mapper_state", "")),
            "local_map_age_s": float(payload.get("local_map_age_s", 0.0)),
            "last_scan": payload.get("last_scan"),
            "last_runtime_event": payload.get("last_runtime_event"),
            "avoidance_active": bool(payload.get("avoidance_active", False)),
            "obstacle_warn": bool(payload.get("obstacle_warn", False)),
            "obstacle_critical": bool(payload.get("obstacle_critical", False)),
            "obstacle_closest_m": float(payload.get("obstacle_closest_m", 99.0)),
            "free_directions": payload.get("free_directions"),
            "drone_ned": payload.get("drone_ned"),
            "home_ned": payload.get("home_ned"),
            "home_captured": bool(payload.get("home_captured", False)),
            "last_completed_target_id": str(payload.get("last_completed_target_id", "")),
            "last_completed_target_name": str(payload.get("last_completed_target_name", "")),
            "runtime_event": runtime_event,
        }
        if not runtime_event:
            extra.pop("runtime_event")
        return (status, extra)

    def _normalize_navigation_backend(self, value: str) -> str:
        backend = value.strip().lower()
        if backend in {NAV_BACKEND_DIRECT, NAV_BACKEND_AVOIDANCE_RUNTIME}:
            return backend
        self.get_logger().warn(
            f"{self._did}: unsupported navigation_backend='{value}', fallback to "
            f"{NAV_BACKEND_DIRECT}"
        )
        return NAV_BACKEND_DIRECT

    def _build_runtime_return_home_cmd_locked(self) -> dict[str, Any]:
        return {
            "command": "return_home",
            "target_id": f"rth_{int(self.get_clock().now().nanoseconds // 1_000_000)}",
            "name": "Swarm RTH",
            "source": "swarm_agent",
            "priority": "mission",
        }

    def _maybe_build_runtime_cmd_locked(self) -> Optional[dict[str, Any]]:
        if not self._runtime_backend_active or self._passive:
            return None
        if self._runtime_rth_requested and not self._runtime_return_home_sent:
            self._runtime_return_home_sent = True
            return self._build_runtime_return_home_cmd_locked()
        if self._runtime_rth_requested:
            return None
        if self._runtime_active_cell_id is not None:
            return None
        if not self._cell_queue:
            self._waiting_for_next = True
            return None
        cid, tx, ty = self._cell_queue.popleft()
        self._runtime_active_cell_id = cid
        self._current_cell_id = cid
        self._target_x = tx
        self._target_y = ty
        self._waiting_for_next = False
        return {
            "command": "goto",
            "target_id": cid,
            "name": cid,
            "target_ned": [tx, ty],
            "source": "swarm_agent",
            "priority": "mission",
        }

    def _publish_runtime_cmd(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._target_cmd_pub.publish(msg)
        self.get_logger().info(
            f"{self._did}: → /{self._did}/avoidance/target_cmd "
            f"{payload.get('command', '')} {payload.get('target_id', '')}"
        )

    def _compute_avoid_waypoint(self) -> Optional[tuple]:
        """Compute lateral detour waypoint to clear an obstacle.

        Called under lock.  Returns (avoid_x, avoid_y) NED, or None when no
        lateral direction is free (caller should use vertical avoidance).
        Prefers right; falls back to left.
        """
        course = math.atan2(
            self._target_y - self._drone_y,
            self._target_x - self._drone_x,
        )
        if "right" in self._free_directions:
            perpendicular = course - math.pi / 2
        elif "left" in self._free_directions:
            perpendicular = course + math.pi / 2
        else:
            return None

        avoid_x = self._drone_x + AVOID_OFFSET_M * math.cos(perpendicular)
        avoid_y = self._drone_y + AVOID_OFFSET_M * math.sin(perpendicular)
        return (avoid_x, avoid_y)

    # ── Control loop (10 Hz) ──────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        if self._runtime_backend_active:
            return
        with self._lock:
            self._ticks += 1

            if self._arm_requested and not self._armed and self._ticks >= ARM_TICKS:
                self._send_arm()
                if self._vsp_initialized:
                    self._vsp[0] = self._drone_x
                    self._vsp[1] = self._drone_y
                    self._vsp[2] = self._drone_z
                self._phase = Phase.TAKEOFF
                # Publish READY after arming — task_allocator will send first cell
                self._arm_requested = False
                self._just_armed    = True

            if self._land_requested:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_requested = False
                self._armed          = False
                self._phase          = Phase.IDLE

            
            self._step_vsp()

            land_now = self._rth_land_now
            if self._rth_land_now:
                self._rth_land_now = False
                self._landing      = True   # stop offboard heartbeat so AUTO.LAND sticks
                self._on_ground    = True   # allow re-arm on next /swarm/mission_ready
                self._armed        = False  # PX4 auto-disarms after AUTO.LAND


            vsp      = list(self._vsp)
            phase    = self._phase
            vsp_yaw  = self._vsp_yaw
            vz       = self._vz
            dx_t     = self._target_x - self._drone_x
            dy_t     = self._target_y - self._drone_y
            ready_to_publish = self._just_armed   # publish READY once, the tick we arm
            if self._just_armed:
                self._just_armed = False
            avoiding_resume_cell = self._avoiding_resume_cell
            if self._avoiding_resume_cell is not None:
                self._avoiding_resume_cell = None

        if land_now:
            # Switch PX4 to AUTO.LAND mode.
            # param1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            # param2=4 (PX4_CUSTOM_MAIN_MODE_AUTO)
            # param3=6 (PX4_CUSTOM_SUB_MODE_AUTO_LAND)
            # Must stop the offboard heartbeat first (done via _landing flag) so PX4
            # doesn't immediately revert to offboard.
            self._send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0, param2=4.0, param3=6.0
            )
            # Notify field_setup_coordinator (and home_manager) that this drone
            # has committed to landing and the pad is being freed.
            lc_msg = String()
            lc_msg.data = json.dumps({"drone_id": self._did})
            self._landed_pub.publish(lc_msg)
            self.get_logger().info(f"{self._did}: published /swarm/landed_confirmation")

        if self._passive:
            return  # don't publish until mission_ready — avoids conflicting with manual_controller

        self._pub_offboard()
        self._pub_setpoint(vsp, phase, vsp_yaw, vz)

        if ready_to_publish:
            self._pub_status("READY")
        if avoiding_resume_cell is not None:
            self._pub_status("AVOIDING", resume_target=avoiding_resume_cell)

    def _terrain_vz(self) -> float:
        """Terrain-following P controller → NED vertical velocity command.

        Called under lock.  Mirrors terrain_follower.py exactly — drone_z is NOT
        used for Z control, only the lidar range drives the velocity.  This avoids
        the runaway-climbing bug where `target_z = drone_z + error` creates a moving
        position target that the drone chases indefinitely when lidar is stuck.

        NED sign convention: vz > 0 = descend, vz < 0 = climb.
        error = range − altitude:  > 0 → too high → vz positive (descend)
                                   < 0 → too low  → vz negative (climb)

        Fallback when lidar unavailable: climb/descend toward fixed altitude using
        drone_z from EKF (less precise, but safe when there is no lidar model).
        """
        if self._range_valid:
            error = self._range - self._altitude
            if abs(error) <= ALT_DEADBAND:
                return 0.0
            vz = TERRAIN_KP * error
            return max(-TERRAIN_VZ_MAX, min(TERRAIN_VZ_MAX, vz))

        # No lidar — use EKF z to approach desired altitude
        if self._pos_valid:
            # target_z = -altitude (NED negative = up); error_to_target = target_z - drone_z
            # negative error → drone below target → vz negative → climb
            error_to_target = -self._altitude - self._drone_z
            if abs(error_to_target) > ALT_DEADBAND:
                vz = TERRAIN_KP * error_to_target
                return max(-TERRAIN_VZ_MAX, min(TERRAIN_VZ_MAX, vz))
        return 0.0

    def _step_vsp(self) -> None:
        """VSP state machine — called under lock.

        Z axis: terrain-following phases (TAKEOFF/ROTATE/CRUISE) use a velocity
        P-controller via self._vz (mirrors terrain_follower.py).  RTH descent uses
        position control stepping _vsp[2] toward RTH_HOVER_Z (fixed target, no
        terrain following needed while coming home to land).
        """
        step = self._cruise_speed * DT   # horizontal step (m/tick)

        if self._phase == Phase.IDLE:
            self._vz = 0.0
            # Safety net: if we're waiting for the next cell and nothing arrives,
            # self-trigger RTH after IDLE_RTH_TICKS (mission probably done).
            if self._waiting_for_next and not self._rth_active and not self._mission_done:
                self._idle_ticks += 1
                if self._idle_ticks >= IDLE_RTH_TICKS:
                    self.get_logger().warn(
                        f"{self._did}: idle {self._idle_ticks} ticks with empty queue — "
                        "self-triggering RTH (safety net)"
                    )
                    self._rth_active   = True
                    self._mission_done = True
                    self._target_x     = self._home_x
                    self._target_y     = self._home_y
                    self._phase        = Phase.ROTATE
                    self._rotate_ticks = 0
                    self._vsp_yaw      = float("nan")
            else:
                self._idle_ticks = 0

        elif self._phase == Phase.TAKEOFF:
            self._takeoff_ticks += 1
            # Z: velocity P-controller (no position target for Z during terrain following)
            self._vz = self._terrain_vz()

            # Transition: altitude reached (lidar or EKF), OR timeout (bad lidar guard).
            if self._range_valid:
                altitude_ok = abs(self._range - self._altitude) < ALT_TOL
            else:
                altitude_ok = self._pos_valid and abs(self._drone_z + self._altitude) < ALT_TOL
            timed_out = self._takeoff_ticks >= MAX_TAKEOFF_TICKS
            if altitude_ok or timed_out:
                if timed_out and not altitude_ok:
                    self.get_logger().warn(
                        f"{self._did}: TAKEOFF timeout ({MAX_TAKEOFF_TICKS} ticks) — "
                        f"forcing ROTATE. range={'%.2f' % self._range if self._range_valid else 'N/A'} "
                        f"drone_z={self._drone_z:.2f} Check lidar bridge!"
                    )
                self._phase         = Phase.ROTATE
                self._rotate_ticks  = 0
                self._vsp_yaw       = float("nan")
                self._takeoff_ticks = 0
                # Activate first queued cell so ROTATE has a real target.
                # Cells are no longer activated directly in _next_cell_cb during
                # TAKEOFF (Bug 2 fix) — they are always queued and activated here.
                if self._current_cell_id is None and self._cell_queue:
                    _cid, _tx, _ty = self._cell_queue.popleft()
                    self._current_cell_id = _cid
                    self._target_x        = _tx
                    self._target_y        = _ty
                    self.get_logger().info(
                        f"{self._did}: TAKEOFF done — first cell from queue: "
                        f"{_cid} NED({_tx:.2f},{_ty:.2f})"
                    )
                elif self._current_cell_id is None:
                    self.get_logger().warn(
                        f"{self._did}: TAKEOFF done but cell queue empty — "
                        "waiting for next_cell in ROTATE/IDLE"
                    )

        elif self._phase == Phase.ROTATE:
            # Z: velocity P-controller (terrain following while rotating)
            self._vz = self._terrain_vz()

            dx_t  = self._target_x - self._drone_x
            dy_t  = self._target_y - self._drone_y
            horiz = math.sqrt(dx_t**2 + dy_t**2)
            target_yaw = (
                math.atan2(dy_t, dx_t) if horiz > REACH_DIST else self._drone_yaw
            )
            if math.isnan(self._vsp_yaw):
                self._vsp_yaw = self._drone_yaw
            yaw_diff = (target_yaw - self._vsp_yaw + math.pi) % (2*math.pi) - math.pi
            yaw_step = YAW_RATE * DT
            if abs(yaw_diff) > yaw_step:
                self._vsp_yaw += math.copysign(yaw_step, yaw_diff)
                self._vsp_yaw  = (self._vsp_yaw + math.pi) % (2*math.pi) - math.pi
            else:
                self._vsp_yaw = target_yaw

            actual_diff = (
                (target_yaw - self._drone_yaw + math.pi) % (2*math.pi) - math.pi
            )
            self._rotate_ticks += 1
            settled = (
                abs(actual_diff) < YAW_TOL and self._rotate_ticks >= ROTATE_TICKS
            ) or self._rotate_ticks >= MAX_ROTATE_TICKS

            if settled:
                if self._rth_active:
                    self._phase = Phase.RTH
                elif horiz < REACH_DIST:
                    # Already at target — pop next from queue or wait
                    if self._cell_queue:
                        next_cid, next_x, next_y = self._cell_queue.popleft()
                        self._current_cell_id = next_cid
                        self._target_x        = next_x
                        self._target_y        = next_y
                        self._cell_reached    = False
                        self._phase           = Phase.CRUISE
                    else:
                        self._waiting_for_next = True
                        self._phase            = Phase.IDLE
                else:
                    self._phase = Phase.CRUISE

        elif self._phase == Phase.CRUISE:
            # Obstacle avoidance — evaluated before any movement
            if self._obstacle_critical:
                self._avoid_resume_target  = (self._target_x, self._target_y)
                wp = self._compute_avoid_waypoint()
                if wp is not None:
                    self._avoid_waypoint       = wp
                    self._avoid_ticks          = 0
                    self._avoiding_resume_cell = self._current_cell_id
                    self._phase                = Phase.AVOIDING
                else:
                    self._target_z -= 2.0   # NED: decrease z = climb
                    self.get_logger().warn(
                        f"{self._did}: obstacle critical, no lateral escape — climbing 2 m"
                    )
                return

            effective_step = (
                self._cruise_speed * 0.5 if self._obstacle_closest < OBSTACLE_WARN_DIST
                else self._cruise_speed
            ) * DT

            # Z: velocity P-controller (terrain following while cruising)
            self._vz = self._terrain_vz()

            dx = self._target_x - self._vsp[0]
            dy = self._target_y - self._vsp[1]
            d  = math.sqrt(dx**2 + dy**2)

            # Continuously track yaw to face direction of travel (nose forward)
            if d > 0.05:
                self._vsp_yaw = math.atan2(dy, dx)

            if d > effective_step:
                self._vsp[0] += (dx / d) * effective_step
                self._vsp[1] += (dy / d) * effective_step
            else:
                # VSP arrived at target — snap and decide what to do next
                self._vsp[0] = self._target_x
                self._vsp[1] = self._target_y

                if not self._cell_reached:
                    self._cell_reached = True
                    self._cell_done_id = self._current_cell_id   # report outside lock

                if self._rth_active:
                    self._phase = Phase.RTH
                elif self._cell_queue:
                    next_cid, next_x, next_y = self._cell_queue.popleft()

                    # Detect direction change to decide: seamless cruise vs stop+rotate
                    next_dx = next_x - self._target_x
                    next_dy = next_y - self._target_y
                    if (next_dx != 0 or next_dy != 0) and not math.isnan(self._vsp_yaw):
                        next_heading  = math.atan2(next_dy, next_dx)
                        angle_diff    = abs(
                            (next_heading - self._vsp_yaw + math.pi) % (2 * math.pi) - math.pi
                        )
                        need_rotate   = angle_diff > math.radians(45)
                    else:
                        need_rotate   = False

                    self._current_cell_id = next_cid
                    self._target_x        = next_x
                    self._target_y        = next_y
                    self._cell_reached    = False

                    if need_rotate:
                        # Stop here, rotate to face new direction, then cruise
                        self._phase        = Phase.ROTATE
                        self._rotate_ticks = 0
                        self._vsp_yaw      = float("nan")  # ROTATE recalculates from drone_yaw
                    # else: same direction — stay in CRUISE, VSP flows into next segment
                else:
                    # Queue empty — hold until task_allocator sends next cell
                    self._waiting_for_next = True
                    self._phase            = Phase.IDLE

        elif self._phase == Phase.AVOIDING:
            self._vz = self._terrain_vz()
            self._avoid_ticks += 1

            if self._avoid_ticks > AVOID_TIMEOUT_TICKS:
                self.get_logger().error(
                    f"{self._did}: AVOIDING timeout ({AVOID_TIMEOUT_TICKS} ticks) — triggering RTH"
                )
                self._rth_active   = True
                self._mission_done = True
                self._target_x     = self._home_x
                self._target_y     = self._home_y
                self._phase        = Phase.RTH
                return

            aw_x, aw_y   = self._avoid_waypoint
            avoid_step   = self._cruise_speed * DT
            dx = aw_x - self._vsp[0]
            dy = aw_y - self._vsp[1]
            d  = math.sqrt(dx**2 + dy**2)

            if d > avoid_step:
                self._vsp[0] += (dx / d) * avoid_step
                self._vsp[1] += (dy / d) * avoid_step
            else:
                self._vsp[0] = aw_x
                self._vsp[1] = aw_y

                if self._obstacle_critical:
                    # Obstacle still blocking at waypoint — try opposite lateral direction
                    course = math.atan2(
                        self._target_y - self._drone_y,
                        self._target_x - self._drone_x,
                    )
                    if "right" in self._free_directions:
                        perpendicular = course - math.pi / 2
                    else:
                        perpendicular = course + math.pi / 2
                    self._avoid_waypoint = (
                        self._drone_x + AVOID_OFFSET_M * math.cos(perpendicular),
                        self._drone_y + AVOID_OFFSET_M * math.sin(perpendicular),
                    )
                    self._avoid_ticks = 0
                else:
                    # Waypoint reached and obstacle cleared — resume original target
                    self._target_x = self._avoid_resume_target[0]
                    self._target_y = self._avoid_resume_target[1]
                    self._phase    = Phase.CRUISE
                    self.get_logger().info(
                        f"{self._did}: obstacle avoided, resuming to original target"
                    )

        elif self._phase == Phase.RTH:
            # Z: position control stepping toward RTH_HOVER_Z (fixed descent, no terrain
            # following — drone is coming home to land, terrain following not needed).
            self._vz = float("nan")
            # Move XY to home, then descend to RTH_HOVER_Z, then switch to AUTO.LAND
            dx = self._home_x - self._vsp[0]
            dy = self._home_y - self._vsp[1]
            d  = math.sqrt(dx**2 + dy**2)
            if d > step:
                self._vsp[0] += (dx / d) * step
                self._vsp[1] += (dy / d) * step
            else:
                self._vsp[0] = self._home_x
                self._vsp[1] = self._home_y
                # Descend to RTH_HOVER_Z then hand off to AUTO.LAND
                rth_step = RTH_SPEED * DT
                dz = RTH_HOVER_Z - self._vsp[2]
                if abs(dz) > rth_step:
                    self._vsp[2] += math.copysign(rth_step, dz)
                else:
                    self._vsp[2]       = RTH_HOVER_Z
                    self._rth_land_now = True
                    self._phase        = Phase.IDLE
                    self.get_logger().info(
                        f"{self._did}: at home pad {RTH_HOVER_Z}m — triggering AUTO.LAND"
                    )

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _pub_offboard(self) -> None:
        # Stop the offboard heartbeat once AUTO.LAND is triggered; otherwise PX4
        # would keep reverting back to offboard mode, blocking the land command.
        if self._landing or self._offboard_pub is None:
            return
        msg           = OffboardControlMode()
        msg.position  = True
        msg.velocity  = True   # needed for mixed position(XY)+velocity(Z) terrain control
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _pub_setpoint(
        self, vsp: list[float], phase: Phase, vsp_yaw: float, vz: float
    ) -> None:
        if self._traj_pub is not None and not self._landing:
            nan = float("nan")
            msg = TrajectorySetpoint()
            msg.timestamp = self._now_us()
            msg.yaw       = vsp_yaw if not math.isnan(vsp_yaw) else nan

            # Terrain-following phases: XY position hold, Z driven by velocity P-controller.
            # This mirrors terrain_follower.py and avoids the runaway-climbing bug where
            # a position target `drone_z + error` moves together with the drone when the
            # lidar reads persistently low (stuck sensor / wrong model).
            # RTH/IDLE: position setpoint for Z (stepping toward RTH_HOVER_Z).
            if phase in (Phase.TAKEOFF, Phase.ROTATE, Phase.CRUISE, Phase.AVOIDING):
                msg.position = [vsp[0], vsp[1], nan]          # Z from velocity
                msg.velocity = [nan, nan, vz]                 # terrain P-controller output
            else:
                msg.position = [vsp[0], vsp[1], vsp[2]]       # full position (RTH/IDLE)
                msg.velocity = [nan, nan, nan]

            self._traj_pub.publish(msg)

        # Publish cell_done outside lock when hover complete
        cell_done = getattr(self, "_cell_done_id", None)
        if cell_done is not None:
            self._cell_done_id = None
            self._pub_cell_complete(cell_done)

    def _pub_status(self, status: str, **extra) -> None:
        payload = {"drone_id": self._did, "status": status, **extra}
        msg      = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
        self.get_logger().info(f"{self._did}: → /swarm/drone_status {status}")

    def _pub_cell_complete(self, cell_id: str) -> None:
        self._pub_status("CELL_COMPLETE", cell_id=cell_id)
        self.get_logger().info(f"{self._did}: cell {cell_id} COMPLETE")

    def _send_arm(self) -> None:
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self._armed = True
        self.get_logger().info(f"{self._did}: armed + offboard mode")

    def _send_command(self, command: int, **kwargs) -> None:
        if self._cmd_pub is None:
            return
        msg                  = VehicleCommand()
        msg.timestamp        = self._now_us()
        msg.command          = command
        msg.target_system    = self._drone_id + 1  # MAVLink sysid = instance+1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self._cmd_pub.publish(msg)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
