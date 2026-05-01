"""
swarm_agent.py — Mission executor / route provider for one drone in the swarm.

One instance per drone. Receives target cells from task_allocator,
forwards high-level targets to obstacle_avoidance_runtime, and reports
progress back to swarm_coordinator/task_allocator.

Interfaces:
  Subscribe:
    /drone_N/next_cell            String  "x4_y2"          from task_allocator
    /swarm/rth_request            String  JSON              mission complete / RTH
    /drone_N/avoidance/status     ScoutAvoidanceStatusMsg   from runtime

  Publish:
    /swarm/drone_status           String  JSON
    /drone_N/avoidance/target_cmd ScoutTargetCommandMsg     to runtime

Parameters:
  drone_id     int    0          which drone instance (0, 1, 2…)
  altitude_m   float  5.0        cruise altitude (passed to runtime)
  home_ned_x   float  0.0        RTH target NED x
  home_ned_y   float  0.0        RTH target NED y
  cruise_speed float  2.0        m/s horizontal
"""

import json
import threading
from collections import deque
from enum import Enum, auto
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleLandDetected
from std_msgs.msg import String
from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.avoidance.types import (
    TargetCommand,
    avoidance_status_from_msg,
    target_command_to_msg,
)

try:
    from scout_control_msgs.msg import (
        AvoidanceStatus as ScoutAvoidanceStatusMsg,
        TargetCommand as ScoutTargetCommandMsg,
    )
except ImportError:
    ScoutAvoidanceStatusMsg = None
    ScoutTargetCommandMsg = None

# ── QoS ──────────────────────────────────────────────────────────────────────
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
QOS_AVOIDANCE_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Constants ─────────────────────────────────────────────────────────────────
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
        self.declare_parameter("navigation_backend", NAV_BACKEND_AVOIDANCE_RUNTIME) # deprecated, always avoidance_runtime

        self._drone_id:     int   = self.get_parameter("drone_id").value
        self._altitude:     float = self.get_parameter("altitude_m").value
        self._home_x:       float = self.get_parameter("home_ned_x").value
        self._home_y:       float = self.get_parameter("home_ned_y").value
        self._cruise_speed: float = self.get_parameter("cruise_speed").value
        self._telemetry = TelemetryHub(drone_id=self._drone_id)
        topics = self._telemetry.topics
        swarm_topics = self._telemetry.swarm
        self._did = topics.drone_ns
        self._navigation_backend: str = NAV_BACKEND_AVOIDANCE_RUNTIME
        self._runtime_backend_active = True

        self._lock   = threading.Lock()
        self._ticks  = 0

        # ── Flight state ──────────────────────────────────────────────────────
        self._on_ground:       bool          = False   # True after AUTO.LAND; allows re-arm

        self._drone_x:  float = 0.0
        self._drone_y:  float = 0.0
        self._drone_z:  float = 0.0
        self._drone_yaw: float = 0.0
        self._pos_valid: bool = False

        self._target_x:      float = 0.0
        self._target_y:      float = 0.0

        # ── Task state ────────────────────────────────────────────────────────
        self._current_cell_id: Optional[str]        = None
        self._cell_queue:      deque                = deque()   # upcoming cells (cid, x, y)
        self._waiting_for_next: bool                = False     # at target, queue empty
        self._mission_done:    bool                 = False
        self._idle_ticks:      int                  = 0         # ticks spent idle (for RTH safety net)
        self._passive:         bool                 = True      # silent until /swarm/mission_ready

        # ── Obstacle avoidance ────────────────────────────────────────────────
        self._rth_active:           bool           = False
        self._last_avoidance_payload: Optional[dict[str, Any]] = None
        self._last_runtime_status_signature: Optional[tuple[Any, ...]] = None
        self._runtime_active_cell_id: Optional[str] = None
        self._runtime_last_completed_target_id: str = ""
        self._runtime_rth_requested: bool = False
        self._runtime_return_home_sent: bool = False
        self._runtime_landing_seen: bool = False
        self._landed_reported: bool = False
        self._suppress_setup_landing_status: bool = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._target_cmd_pub = self.create_publisher(
            ScoutTargetCommandMsg or String,
            topics.avoidance_target_cmd,
            QOS_VOL,
        )
        self._target_cmd_json_pub = self.create_publisher(
            String,
            topics.avoidance_target_cmd_json,
            QOS_VOL,
        )
        self._status_pub = self.create_publisher(
            String, swarm_topics.drone_status, QOS_VOL)
        self._landed_pub = self.create_publisher(
            String, swarm_topics.landed_confirmation, QOS_SENSOR)

        # ── Subscribers ───────────────────────────────────────────────────────
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
            String, topics.next_cell,
            self._next_cell_cb, _qos_next_cell)
        self.create_subscription(
            String, swarm_topics.rth_request,
            self._rth_request_cb, QOS_VOL)
        land_topic = f"{topics.px4_ns}/fmu/out/vehicle_land_detected"
        self.create_subscription(
            VehicleLandDetected,
            land_topic,
            self._land_detected_cb,
            QOS_SENSOR,
        )

        self.create_subscription(
            ScoutAvoidanceStatusMsg or String,
            topics.avoidance_status,
            self._avoidance_status_cb,
            QOS_AVOIDANCE_STATUS,
        )
        if ScoutAvoidanceStatusMsg is not None:
            self.create_subscription(
                String,
                topics.avoidance_status_json,
                self._avoidance_status_cb,
                QOS_AVOIDANCE_STATUS,
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
            String, swarm_topics.mission_ready,
            self._mission_ready_cb, _qos_mission_volatile)

        self.get_logger().info(
            f"SwarmAgent {self._did} ready | "
            f"alt={self._altitude}m | home NED({self._home_x},{self._home_y}) | "
            f"cruise={self._cruise_speed}m/s | backend={self._navigation_backend} | "
            f"runtime_backend_active={self._runtime_backend_active} | "
            "waiting for /swarm/mission_ready"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

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
            
            self._rth_active = True
            self._mission_done = True
            self._runtime_rth_requested = True
            self._runtime_return_home_sent = True
            self._runtime_landing_seen = False
            self._landed_reported = False
            self._cell_queue.clear()
            self._runtime_active_cell_id = None
            cmd = self._build_runtime_return_home_cmd_locked()
            
            self.get_logger().info(f"{self._did}: RTH requested (runtime backend)")

        if cmd is not None:
            self._publish_runtime_cmd(cmd)

    def _avoidance_status_cb(self, msg: Any) -> None:
        if ScoutAvoidanceStatusMsg is not None and isinstance(msg, ScoutAvoidanceStatusMsg):
            payload = avoidance_status_from_msg(msg).to_payload()
        else:
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
            if mapped is None:
                return
            phase = str(payload.get("phase", "")).upper()
            if phase == "LANDING":
                self._runtime_landing_seen = True
            elif phase:
                self._suppress_setup_landing_status = False
            status, extra = mapped
            if (
                status == "LANDING"
                and self._suppress_setup_landing_status
                and not self._runtime_rth_requested
            ):
                status = "READY"
                extra["phase"] = "MISSION_READY"
                extra["suppressed_phase"] = "LANDING"
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

    def _build_runtime_return_home_cmd_locked(self) -> dict[str, Any]:
        return {
            "command": "return_home",
            "target_id": f"rth_{int(self.get_clock().now().nanoseconds // 1_000_000)}",
            "name": "Swarm RTH",
            "source": "swarm_agent",
            "priority": "mission",
        }

    def _maybe_build_runtime_cmd_locked(self) -> Optional[dict[str, Any]]:
        if self._passive:
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
        json_msg = String()
        json_msg.data = json.dumps(payload)
        if ScoutTargetCommandMsg is not None:
            command = TargetCommand.from_payload(payload)
            self._target_cmd_pub.publish(
                target_command_to_msg(command, ScoutTargetCommandMsg())
            )
            self._target_cmd_json_pub.publish(json_msg)
        else:
            self._target_cmd_pub.publish(json_msg)
        self.get_logger().info(
            f"{self._did}: → /{self._did}/avoidance/target_cmd "
            f"{payload.get('command', '')} {payload.get('target_id', '')}"
        )

    def _mission_ready_cb(self, msg: String) -> None:
        """
        Received /swarm/mission_ready — starts mission delegator.

        Called once after field setup is complete and the operator presses M.
        Before this message arrives swarm_agent stays passive (no commands to runtime).
        """
        with self._lock:
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
            self._runtime_landing_seen = False
            self._landed_reported = True
            self._suppress_setup_landing_status = True
            self._last_runtime_status_signature = None
        self.get_logger().info(
            f"{self._did}: /swarm/mission_ready received — runtime backend mission mode active"
        )
        self._pub_status(
            "READY",
            backend=NAV_BACKEND_AVOIDANCE_RUNTIME,
            phase="MISSION_READY",
        )

    def _land_detected_cb(self, msg: VehicleLandDetected) -> None:
        if not bool(getattr(msg, "landed", False)):
            return
        if not self._runtime_landing_seen or self._landed_reported:
            return
        self._landed_reported = True
        out = String()
        out.data = json.dumps({"drone_id": self._did, "source": "vehicle_land_detected"})
        self._landed_pub.publish(out)
        self.get_logger().info(f"{self._did}: landed confirmation published")

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _pub_status(self, status: str, **extra) -> None:
        payload = {"drone_id": self._did, "status": status, **extra}
        msg      = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
        self.get_logger().info(f"{self._did}: → /swarm/drone_status {status}")

    def _pub_cell_complete(self, cell_id: str) -> None:
        self._pub_status("CELL_COMPLETE", cell_id=cell_id)
        self.get_logger().info(f"{self._did}: cell {cell_id} COMPLETE")

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
