"""
obstacle_avoidance_runtime.py — Generic per-drone obstacle avoidance runtime.

This node owns:
  - depth-based obstacle detection
  - local scan / point-cloud logging
  - reactive avoidance and PX4 offboard setpoints

Mission-specific route logic stays outside this node and is injected via:
  /drone_N/avoidance/target_cmd   std_msgs/String JSON

Supported commands:
  {"command":"goto","target_id":"pad_1","name":"North Wall","target_ned":[22.0, 0.0]}
  {"command":"return_home","target_id":"home_after_pad_1","name":"Return Home"}
  {"command":"hold","name":"Hold Position"}
  {"command":"land","name":"Land"}
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from enum import Enum, auto
from functools import partial
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from scout_control.avoidance.depth_projector import DepthProjector
from scout_control.avoidance.local_mapper import (
    LocalClearanceSummary,
    LocalMapper,
    LocalMapperConfig,
    LocalMapperSnapshot,
)
from scout_control.avoidance.local_planner import (
    BlockedHistoryEntry,
    LocalGridSnapshot,
    LocalPlanner,
    LocalPlannerConfig,
    LocalPlannerState,
    PlanResult,
    PlannerPose,
    PlannerResultStatus,
    PlannerTarget,
)
from scout_control.avoidance.scan_manager import (
    SCAN_POINT_MAX_RANGE_M,
    SCAN_POINT_MIN_RANGE_M,
    ScanManager,
)
from scout_control.avoidance.types import ScanStepResult
from scout_control.avoidance_logging import AvoidanceRunLogger

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
QOS_VIZ = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_EVENTS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

DT = 0.1
ARM_TICKS = 50
ALT_TOL = 0.5
WARN_SLOWDOWN = 0.5
DETOUR_REACHED_DIST = 1.0
MAX_PATH_LEN = 500
FALLBACK_DETOUR_MULTIPLIER = 1.5
SETPOINT_LOOKAHEAD_S = 1.0
MIN_COMMAND_STEP_M = 0.75
WARN_REPLAN_DIST_M = 2.8
OBSTACLE_MEMORY_CLEAR_TICKS = 12
LOOP_MEMORY_RADIUS_M = 2.0
WARN_DRIFT_TRIGGER_M = 5.0
WARN_DRIFT_BIAS_M = 2.0
SCAN_HOVER_TICKS = 15
SCAN_SPIN_TICKS = 80
SCAN_FREE_DISTANCE_M = 4.0
SCAN_POINT_STRIDE = 8
SCAN_CAM_HFOV_DEG = 71.9
PLAN_INTERVAL_TICKS = 5
STOP_HOVER_REPLAN_TICKS = 10
SCAN_RETRY_LIMIT = 2
NO_PATH_BLOCKED_THRESHOLD = 3
BLOCKED_RETRY_TICKS = 100


class RuntimePhase(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    CRUISE_TO_TARGET = auto()
    WARN_DRIFT = auto()
    STOP_HOVER = auto()
    SCAN_360 = auto()
    LOCAL_REPLAN = auto()
    DETOUR_EXECUTION = auto()
    BLOCKED = auto()
    RETURN_HOME = auto()
    LANDING = auto()
    ABORT = auto()


class ObstacleAvoidanceRuntime(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_runtime")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("default_altitude_m", 5.0)
        self.declare_parameter("default_cruise_speed", 2.5)
        self.declare_parameter("default_clear_dist", 2.5)
        self.declare_parameter("avoid_offset_m", 3.0)
        self.declare_parameter("home_dist", 1.5)
        self.declare_parameter("setpoint_lookahead_s", SETPOINT_LOOKAHEAD_S)
        self.declare_parameter("min_command_step_m", MIN_COMMAND_STEP_M)
        self.declare_parameter("warn_replan_dist_m", WARN_REPLAN_DIST_M)
        self.declare_parameter("planner_blocked_cost_scale", 0.1)
        self.declare_parameter("planner_peer_cost_scale", 0.1)
        self.declare_parameter("obstacle_memory_clear_ticks", OBSTACLE_MEMORY_CLEAR_TICKS)
        self.declare_parameter("loop_memory_radius_m", LOOP_MEMORY_RADIUS_M)
        self.declare_parameter("warn_drift_trigger_m", WARN_DRIFT_TRIGGER_M)
        self.declare_parameter("warn_drift_bias_m", WARN_DRIFT_BIAS_M)
        self.declare_parameter("scan_hover_ticks", SCAN_HOVER_TICKS)
        self.declare_parameter("scan_spin_ticks", SCAN_SPIN_TICKS)
        self.declare_parameter("scan_free_distance_m", SCAN_FREE_DISTANCE_M)
        self.declare_parameter("scan_point_stride", SCAN_POINT_STRIDE)
        self.declare_parameter("scan_cam_hfov_deg", SCAN_CAM_HFOV_DEG)
        self.declare_parameter("warn_distance", 4.0)
        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("local_map_resolution_m", 0.5)
        self.declare_parameter("local_map_span_m", 36.0)
        self.declare_parameter("local_map_stale_after_s", 1.5)
        self.declare_parameter("local_map_depth_stride", 6)
        self.declare_parameter("local_map_collision_band_min_m", -2.0)
        self.declare_parameter("local_map_collision_band_max_m", 2.5)
        self.declare_parameter("local_map_depth_half_life_s", 10.0)
        self.declare_parameter("local_map_scan_half_life_s", 75.0)
        self.declare_parameter("local_map_blocked_half_life_s", 140.0)
        self.declare_parameter("local_map_peer_cost_gain", 1.0)
        self.declare_parameter("local_map_blocked_cost_gain", 1.0)
        self.declare_parameter("peer_drone_ids", [])
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("publish_legacy_obstacle_topics", False)
        self.declare_parameter("log_run_label", "")

        self._drone_id = int(self.get_parameter("drone_id").value)
        self._default_alt = float(self.get_parameter("default_altitude_m").value)
        self._default_cruise = float(self.get_parameter("default_cruise_speed").value)
        self._default_clear_d = float(self.get_parameter("default_clear_dist").value)
        self._avoid_offset = float(self.get_parameter("avoid_offset_m").value)
        self._home_d = float(self.get_parameter("home_dist").value)
        self._setpoint_lookahead_s = float(self.get_parameter("setpoint_lookahead_s").value)
        self._min_command_step_m = float(self.get_parameter("min_command_step_m").value)
        self._warn_replan_dist_m = float(self.get_parameter("warn_replan_dist_m").value)
        self._planner_blocked_cost_scale = float(
            self.get_parameter("planner_blocked_cost_scale").value
        )
        self._planner_peer_cost_scale = float(
            self.get_parameter("planner_peer_cost_scale").value
        )
        self._obstacle_memory_clear_ticks = int(
            self.get_parameter("obstacle_memory_clear_ticks").value
        )
        self._loop_memory_radius_m = float(self.get_parameter("loop_memory_radius_m").value)
        self._warn_drift_trigger_m = float(self.get_parameter("warn_drift_trigger_m").value)
        self._warn_drift_bias_m = float(self.get_parameter("warn_drift_bias_m").value)
        self._scan_hover_ticks = max(1, int(self.get_parameter("scan_hover_ticks").value))
        self._scan_spin_ticks = max(10, int(self.get_parameter("scan_spin_ticks").value))
        self._scan_free_distance_m = float(self.get_parameter("scan_free_distance_m").value)
        self._scan_point_stride = max(1, int(self.get_parameter("scan_point_stride").value))
        self._scan_cam_hfov_deg = float(self.get_parameter("scan_cam_hfov_deg").value)
        self._warn_dist = float(self.get_parameter("warn_distance").value)
        self._stop_dist = float(self.get_parameter("stop_distance").value)
        self._local_map_resolution_m = float(self.get_parameter("local_map_resolution_m").value)
        self._local_map_span_m = float(self.get_parameter("local_map_span_m").value)
        self._local_map_stale_after_s = float(
            self.get_parameter("local_map_stale_after_s").value
        )
        self._local_map_depth_stride = max(
            1, int(self.get_parameter("local_map_depth_stride").value)
        )
        self._local_map_collision_band = (
            float(self.get_parameter("local_map_collision_band_min_m").value),
            float(self.get_parameter("local_map_collision_band_max_m").value),
        )
        self._local_map_depth_half_life_s = max(
            0.1, float(self.get_parameter("local_map_depth_half_life_s").value)
        )
        self._local_map_scan_half_life_s = max(
            0.1, float(self.get_parameter("local_map_scan_half_life_s").value)
        )
        self._local_map_blocked_half_life_s = max(
            0.1, float(self.get_parameter("local_map_blocked_half_life_s").value)
        )
        self._local_map_peer_cost_gain = float(
            self.get_parameter("local_map_peer_cost_gain").value
        )
        self._local_map_blocked_cost_gain = float(
            self.get_parameter("local_map_blocked_cost_gain").value
        )
        peer_drone_ids_param = self.get_parameter("peer_drone_ids").value or []
        self._peer_drone_ids = sorted(
            {
                int(peer_id)
                for peer_id in peer_drone_ids_param
                if int(peer_id) != self._drone_id
            }
        )
        self._log_run_label = str(self.get_parameter("log_run_label").value)
        self._publish_legacy_obstacle_topics = bool(
            self.get_parameter("publish_legacy_obstacle_topics").value
        )

        self._camera_topic = str(self.get_parameter("camera_topic").value).strip()
        self._depth_topic = str(self.get_parameter("depth_topic").value).strip()
        if not self._camera_topic:
            self._camera_topic = f"/drone_{self._drone_id}/camera/image_raw"
        if not self._depth_topic:
            self._depth_topic = f"/drone_{self._drone_id}/depth/image_raw"

        px4_ns = "" if self._drone_id == 0 else f"/px4_{self._drone_id}"
        drone_ns = f"drone_{self._drone_id}"

        self._phase = RuntimePhase.IDLE
        self._phase_ticks = 0
        self._phase_enter_ts = time.time()
        self._ticks = 0
        self._land_sent = False

        self._drone_x = 0.0
        self._drone_y = 0.0
        self._drone_z = 0.0
        self._drone_yaw = 0.0
        self._pos_valid = False

        self._vsp_x = 0.0
        self._vsp_y = 0.0
        self._vsp_z = 0.0

        self._home_x = 0.0
        self._home_y = 0.0
        self._home_captured = False

        self._active_command = "none"
        self._active_target_id = ""
        self._active_target_name = ""
        self._active_target_xy: tuple[float, float] | None = None
        self._active_target_alt = self._default_alt
        self._active_target_speed = self._default_cruise
        self._active_target_clear_dist = self._default_clear_d
        self._last_completed_target_id = ""
        self._last_completed_target_name = ""
        self._avoidance_active = False

        self._obstacle_warn = False
        self._obstacle_critical = False
        self._obstacle_closest = 99.0
        self._obstacle_sectors: dict[str, float] = {"left": 99.0, "center": 99.0, "right": 99.0}
        self._free_directions: list[str] = ["left", "center", "right"]
        self._last_obstacle_snapshot = {
            "warn": False,
            "critical": False,
            "closest": 99.0,
            "free_directions": ["left", "center", "right"],
        }

        self._detour_target: tuple[float, float] | None = None
        self._detour_side = "none"
        self._detour_strategy = "none"
        self._avoid_commit_side = "none"
        self._obstacle_memory_ticks = 0

        self._planner = LocalPlanner(
            LocalPlannerConfig(
                planning_horizon_m=15.0,
                subgoal_distance_m=12.0,
                obstacle_margin_cost=self._avoid_offset * 10.0,
                blocked_cost_scale=self._planner_blocked_cost_scale,
                peer_cost_scale=self._planner_peer_cost_scale,
            )
        )
        self._last_plan_result: PlanResult | None = None
        self._blocked_history: list[BlockedHistoryEntry] = []
        self._last_planner_state = LocalPlannerState.READY
        self._no_path_streak = 0
        self._scan_attempts_for_target = 0
        self._blocked_since_s = 0.0
        self._blocked_reason = ""
        self._blocked_severity = "none"

        self._actual_path: deque[tuple[float, float, float]] = deque(maxlen=MAX_PATH_LEN)
        self._start_time = time.time()
        self._last_status_log_ts = 0.0
        self._last_publish_log_ts = 0.0

        self._bridge = CvBridge()
        self._latest_rgb_frame: np.ndarray | None = None
        self._latest_depth_frame: np.ndarray | None = None
        self._latest_depth_ts = 0.0
        self._depth_frame_count = 0
        self._last_depth_stats = {
            "height": 0,
            "width": 0,
            "valid_samples": 0,
            "total_samples": 0,
            "closest": 99.0,
        }
        self._last_scan_summary: dict[str, Any] | None = None
        self._last_runtime_event: dict[str, Any] | None = None

        self._run_log = AvoidanceRunLogger(
            source="obstacle_avoidance_runtime",
            drone_id=self._drone_id,
            run_label=self._log_run_label,
        )
        self._depth_projector = DepthProjector(
            camera_hfov_deg=self._scan_cam_hfov_deg,
            min_range_m=SCAN_POINT_MIN_RANGE_M,
            max_range_m=SCAN_POINT_MAX_RANGE_M,
            default_stride=self._local_map_depth_stride,
            collision_band_m=self._local_map_collision_band,
        )
        self._local_mapper = LocalMapper(
            LocalMapperConfig(
                resolution_m=self._local_map_resolution_m,
                span_x_m=self._local_map_span_m,
                span_y_m=self._local_map_span_m,
                stale_after_s=self._local_map_stale_after_s,
                collision_band_min_m=self._local_map_collision_band[0],
                collision_band_max_m=self._local_map_collision_band[1],
                depth_half_life_s=self._local_map_depth_half_life_s,
                scan_half_life_s=self._local_map_scan_half_life_s,
                blocked_half_life_s=self._local_map_blocked_half_life_s,
                peer_cost_gain=self._local_map_peer_cost_gain,
                blocked_cost_gain=self._local_map_blocked_cost_gain,
                warn_distance_m=self._warn_dist,
                critical_distance_m=self._stop_dist,
            )
        )
        self._local_grid_snapshot: LocalMapperSnapshot = self._local_mapper.latest_snapshot
        self._clearance_summary: LocalClearanceSummary = self._local_mapper.latest_summary
        self._scan_manager = ScanManager(
            mapper=self._local_mapper,
            assets_dir=self._run_log.assets_dir,
            hover_ticks=self._scan_hover_ticks,
            spin_ticks=self._scan_spin_ticks,
            point_stride=self._scan_point_stride,
            free_distance_m=self._scan_free_distance_m,
            cam_hfov_deg=self._scan_cam_hfov_deg,
            camera_topic=self._camera_topic,
            depth_topic=self._depth_topic,
            log_cb=self.get_logger().info,
            run_log_cb=self._log_run_event,
            point_min_range_m=SCAN_POINT_MIN_RANGE_M,
            point_max_range_m=SCAN_POINT_MAX_RANGE_M,
        )

        self._pub_ocm = self.create_publisher(
            OffboardControlMode, f"{px4_ns}/fmu/in/offboard_control_mode", QOS_PX4_PUB
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, f"{px4_ns}/fmu/in/trajectory_setpoint", QOS_PX4_PUB
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, f"{px4_ns}/fmu/in/vehicle_command", QOS_PX4_PUB
        )

        self._pub_detected = self.create_publisher(
            String, f"/{drone_ns}/obstacles/detected", QOS_VIZ
        )
        self._pub_clear = self.create_publisher(
            Bool, f"/{drone_ns}/obstacles/clear", QOS_VIZ
        )
        self._pub_status = self.create_publisher(
            String, f"/{drone_ns}/avoidance/status", QOS_STATUS
        )
        self._pub_plan = self.create_publisher(
            Path, f"/{drone_ns}/avoidance/planned_path", QOS_VIZ
        )
        self._pub_actual = self.create_publisher(
            Path, f"/{drone_ns}/avoidance/actual_path", QOS_VIZ
        )
        self._pub_avoid = self.create_publisher(
            Bool, f"/{drone_ns}/avoidance/active", QOS_VIZ
        )
        self._pub_events = self.create_publisher(
            String, f"/{drone_ns}/avoidance/events", QOS_EVENTS
        )

        self._pub_status_legacy = None
        self._pub_plan_legacy = None
        self._pub_actual_legacy = None
        self._pub_avoid_legacy = None
        self._pub_events_legacy = None
        if self._drone_id == 0 and self._publish_legacy_obstacle_topics:
            self._pub_status_legacy = self.create_publisher(
                String, "/obstacle_avoidance/status", QOS_STATUS
            )
            self._pub_plan_legacy = self.create_publisher(
                Path, "/obstacle_avoidance/planned_path", QOS_VIZ
            )
            self._pub_actual_legacy = self.create_publisher(
                Path, "/obstacle_avoidance/actual_path", QOS_VIZ
            )
            self._pub_avoid_legacy = self.create_publisher(
                Bool, "/obstacle_avoidance/avoidance_active", QOS_VIZ
            )
            self._pub_events_legacy = self.create_publisher(
                String, "/obstacle_avoidance/events", QOS_EVENTS
            )

        self.create_subscription(
            VehicleLocalPosition,
            f"{px4_ns}/fmu/out/vehicle_local_position_v1",
            self._pos_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(Image, self._camera_topic, self._rgb_cb, QOS_SENSOR)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, QOS_SENSOR)
        self.create_subscription(
            String,
            f"/{drone_ns}/avoidance/target_cmd",
            self._target_cmd_cb,
            QOS_STATUS,
        )
        self._peer_pos_subs = []
        for peer_id in self._peer_drone_ids:
            peer_px4_ns = "" if peer_id == 0 else f"/px4_{peer_id}"
            self._peer_pos_subs.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    f"{peer_px4_ns}/fmu/out/vehicle_local_position_v1",
                    partial(self._peer_pos_cb, peer_id),
                    QOS_PX4_SUB,
                )
            )

        self.create_timer(DT, self._control_loop)
        self.create_timer(0.1, self._publish_obstacle_state)
        self.create_timer(1.0, self._pub_status_cb)
        self.create_timer(0.2, self._pub_viz_cb)

        self.get_logger().info(
            f"obstacle_avoidance_runtime ready — drone_id={self._drone_id} "
            f"default_alt={self._default_alt}m default_speed={self._default_cruise}m/s "
            f"legacy_topics={'on' if self._publish_legacy_obstacle_topics else 'off'}"
        )
        self._run_log.log(
            "runtime_started",
            default_altitude_m=float(self._default_alt),
            default_cruise_speed_mps=float(self._default_cruise),
            default_clear_distance_m=float(self._default_clear_d),
            warn_distance_m=float(self._warn_dist),
            stop_distance_m=float(self._stop_dist),
            camera_topic=self._camera_topic,
            depth_topic=self._depth_topic,
            target_cmd_topic=f"/{drone_ns}/avoidance/target_cmd",
            event_topic=f"/{drone_ns}/avoidance/events",
        )

    def _log_run_event(self, event: str, **fields: Any) -> None:
        self._run_log.log(event, **fields)

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x = msg.x
        self._drone_y = msg.y
        self._drone_z = msg.z
        self._drone_yaw = msg.heading
        self._pos_valid = True
        self._local_mapper.update_pose(
            self._drone_x,
            self._drone_y,
            self._drone_z,
            self._drone_yaw,
            time.time(),
        )

    def _peer_pos_cb(self, peer_id: int, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._local_mapper.ingest_peer_position(
            f"drone_{peer_id}",
            x=msg.x,
            y=msg.y,
            z=msg.z,
            stamp_s=time.time(),
            vx=msg.vx if msg.v_xy_valid else 0.0,
            vy=msg.vy if msg.v_xy_valid else 0.0,
        )

    def _rgb_cb(self, msg: Image) -> None:
        try:
            self._latest_rgb_frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB CvBridge conversion failed: {exc}")

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            ).astype(np.float32)
        except Exception as exc:
            self.get_logger().warn(f"Depth CvBridge conversion failed: {exc}")
            return

        self._latest_depth_frame = depth
        self._latest_depth_ts = time.time()
        self._update_depth_frame_stats(depth)
        self._ingest_depth_points(depth)

    def _target_cmd_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid target command JSON: {exc}")
            return

        cmd = str(data.get("command", "goto"))
        name = str(data.get("name", cmd))
        target_id = str(data.get("target_id", f"{cmd}_{int(time.time())}"))

        if cmd == "goto":
            target = data.get("target_ned")
            if not isinstance(target, (list, tuple)) or len(target) < 2:
                self.get_logger().warn("goto command missing target_ned=[x,y]")
                return
            self._activate_target(
                command=cmd,
                target_id=target_id,
                name=name,
                target_xy=(float(target[0]), float(target[1])),
                altitude_m=float(data.get("altitude_m", self._default_alt)),
                cruise_speed=float(data.get("cruise_speed_mps", self._default_cruise)),
                clear_dist=float(data.get("clear_radius_m", self._default_clear_d)),
            )
            return

        if cmd == "return_home":
            if not self._home_captured:
                self.get_logger().warn("return_home requested before home was captured")
                return
            self._activate_target(
                command=cmd,
                target_id=target_id,
                name=name,
                target_xy=(self._home_x, self._home_y),
                altitude_m=float(data.get("altitude_m", self._default_alt)),
                cruise_speed=float(data.get("cruise_speed_mps", self._default_cruise)),
                clear_dist=float(data.get("clear_radius_m", self._home_d)),
            )
            return

        if cmd == "hold":
            self._clear_active_target()
            self._transition_to(RuntimePhase.STOP_HOVER, reason="external_hold_command")
            return

        if cmd == "land":
            self._clear_active_target()
            self._land_sent = False
            self._transition_to(RuntimePhase.LANDING, reason="external_land_command")
            return

        if cmd == "cancel":
            self._clear_active_target()
            self._transition_to(RuntimePhase.STOP_HOVER, reason="external_cancel_command")
            return

        self.get_logger().warn(f"Unknown target command: {cmd}")

    def _activate_target(
        self,
        *,
        command: str,
        target_id: str,
        name: str,
        target_xy: tuple[float, float],
        altitude_m: float,
        cruise_speed: float,
        clear_dist: float,
    ) -> None:
        previous_target = self._active_target_xy
        self._active_command = command
        self._active_target_id = target_id
        self._active_target_name = name
        self._active_target_xy = target_xy
        self._active_target_alt = altitude_m
        self._active_target_speed = cruise_speed
        self._active_target_clear_dist = clear_dist
        self._detour_target = None
        self._detour_side = "none"
        self._detour_strategy = "none"
        self._avoid_commit_side = "none"
        self._local_mapper.clear_blocked_history()
        self._blocked_history.clear()
        self._no_path_streak = 0
        self._scan_attempts_for_target = 0
        self._blocked_reason = ""
        self._blocked_severity = "none"
        self._blocked_since_s = 0.0
        self._land_sent = False
        self._scan_manager.reset()
        if previous_target != target_xy:
            self._actual_path.clear()
        if self._phase == RuntimePhase.STOP_HOVER or self._phase == RuntimePhase.IDLE:
            next_phase = RuntimePhase.TAKEOFF if not self._is_at_target_altitude() else RuntimePhase.CRUISE_TO_TARGET
            self._transition_to(
                next_phase,
                reason="external_target_command",
                target_id=target_id,
                target_name=name,
            )
        self._run_log.log(
            "target_command_received",
            command=command,
            target_id=target_id,
            target_name=name,
            target_ned=[round(target_xy[0], 3), round(target_xy[1], 3)],
            altitude_m=round(float(altitude_m), 3),
            cruise_speed_mps=round(float(cruise_speed), 3),
            clear_dist_m=round(float(clear_dist), 3),
        )

    def _clear_active_target(self) -> None:
        self._active_command = "none"
        self._active_target_id = ""
        self._active_target_name = ""
        self._active_target_xy = None
        self._detour_target = None
        self._detour_side = "none"
        self._detour_strategy = "none"
        self._avoid_commit_side = "none"
        self._no_path_streak = 0
        self._scan_attempts_for_target = 0
        self._blocked_reason = ""
        self._blocked_severity = "none"
        self._blocked_since_s = 0.0
        self._scan_manager.reset()

    def _update_depth_frame_stats(self, depth: np.ndarray) -> None:
        h, w = depth.shape[:2]
        valid = (
            (depth >= SCAN_POINT_MIN_RANGE_M)
            & (depth <= SCAN_POINT_MAX_RANGE_M)
            & ~np.isnan(depth)
        )
        valid_samples = int(np.sum(valid))
        closest = float(np.min(depth[valid])) if valid_samples > 0 else 99.0
        self._depth_frame_count += 1
        self._last_depth_stats = {
            "height": int(h),
            "width": int(w),
            "valid_samples": valid_samples,
            "total_samples": int(depth.size),
            "closest": round(float(closest), 3),
        }

    def _ingest_depth_points(self, depth: np.ndarray) -> None:
        if not self._pos_valid:
            return
        body_batch = self._depth_projector.depth_to_body_points(
            depth,
            pixel_stride=self._local_map_depth_stride,
            stamp_s=self._latest_depth_ts,
            source="depth_projector",
            is_dense_scan=False,
        )
        body_batch.confidence = 0.7
        world_batch = self._depth_projector.project_to_world_points(
            body_batch,
            origin_ned=(self._drone_x, self._drone_y, self._drone_z),
            yaw_rad=self._drone_yaw,
            source="depth_projector",
            collision_band_m=self._local_map_collision_band,
        )
        self._local_mapper.ingest_point_batch(world_batch)

    def _publish_obstacle_state(self) -> None:
        closest = self._clearance_summary.closest_m
        warn = self._clearance_summary.warn
        critical = self._clearance_summary.critical
        free = list(self._clearance_summary.free_directions)

        self._obstacle_closest = closest
        self._obstacle_warn = warn
        self._obstacle_critical = critical
        self._free_directions = free

        payload = {
            "drone_id": f"drone_{self._drone_id}",
            "closest": round(float(closest), 2),
            "closest_m": round(float(closest), 2),
            "sectors": {k: round(float(v), 2) for k, v in self._obstacle_sectors.items()},
            "free_directions": free,
            "warn": warn,
            "critical": critical,
            "local_mapper_state": self._local_grid_snapshot.state.name,
        }
        self._pub_detected.publish(String(data=json.dumps(payload)))
        self._pub_clear.publish(Bool(data=not warn))

        snapshot = {
            "warn": warn,
            "critical": critical,
            "closest": round(float(closest), 2),
            "free_directions": list(free),
        }
        now = time.time()
        if snapshot != self._last_obstacle_snapshot or (now - self._last_publish_log_ts) >= 1.0:
            self._last_obstacle_snapshot = snapshot
            self._last_publish_log_ts = now
            self._run_log.log(
                "obstacle_update",
                obstacle=snapshot,
                sectors={k: round(float(v), 3) for k, v in self._obstacle_sectors.items()},
                phase=self._phase.name,
                active_target_id=self._active_target_id,
            )

    def _control_loop(self) -> None:
        self._ticks += 1
        self._phase_ticks += 1
        self._publish_offboard_heartbeat()
        if self._pos_valid:
            self._local_mapper.update_pose(
                self._drone_x,
                self._drone_y,
                self._drone_z,
                self._drone_yaw,
                time.time(),
            )
        self._local_grid_snapshot, self._clearance_summary = self._local_mapper.update(
            time.time()
        )
        self._sync_obstacle_summary()
        self._update_obstacle_memory()

        # Phase dispatcher
        if self._phase == RuntimePhase.IDLE:
            self._do_idle()
        elif self._phase == RuntimePhase.TAKEOFF:
            self._do_takeoff()
        elif self._phase == RuntimePhase.CRUISE_TO_TARGET:
            self._do_cruise_to_target()
        elif self._phase == RuntimePhase.WARN_DRIFT:
            self._do_warn_drift()
        elif self._phase == RuntimePhase.STOP_HOVER:
            self._do_stop_hover()
        elif self._phase == RuntimePhase.SCAN_360:
            self._do_scan_360()
        elif self._phase == RuntimePhase.LOCAL_REPLAN:
            self._do_local_replan()
        elif self._phase == RuntimePhase.DETOUR_EXECUTION:
            self._do_detour_execution()
        elif self._phase == RuntimePhase.BLOCKED:
            self._do_blocked()
        elif self._phase == RuntimePhase.RETURN_HOME:
            self._do_return_home()
        elif self._phase == RuntimePhase.LANDING:
            self._do_landing()
        elif self._phase == RuntimePhase.ABORT:
            self._do_abort()

    def _do_idle(self) -> None:
        self._vsp_x = self._drone_x
        self._vsp_y = self._drone_y
        self._vsp_z = 0.0
        if self._active_target_xy is not None and self._pos_valid and self._ticks >= ARM_TICKS:
            self._capture_home_if_needed()
            self._arm()
            self._set_offboard_mode()
            self._vsp_z = -self._active_target_alt
            self._transition_to(RuntimePhase.TAKEOFF, reason="target_active_arm_and_offboard")

    def _do_takeoff(self) -> None:
        target_alt = self._active_target_alt
        self._vsp_z = -target_alt
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
        if self._pos_valid and abs(self._drone_z + target_alt) < ALT_TOL:
            self._transition_to(RuntimePhase.CRUISE_TO_TARGET, reason="takeoff_altitude_reached")

    def _do_cruise_to_target(self) -> None:
        if self._active_target_xy is None:
            self._transition_to(RuntimePhase.IDLE, reason="cruise_without_target")
            return

        tx, ty = self._active_target_xy
        if self._distance_to(tx, ty) < self._active_target_clear_dist:
            self._complete_target(reason="target_reached")
            return

        # Periodic plan check
        if self._ticks % PLAN_INTERVAL_TICKS == 0 or self._obstacle_critical or self._obstacle_warn:
            plan = self._run_planner()
            self._last_plan_result = plan
            self._last_planner_state = plan.planner_state

            if plan.status == PlannerResultStatus.DIRECT:
                self._no_path_streak = 0
                if self._obstacle_warn:
                    # Even if direct, if warn is active, we might want to drift slightly
                    # but for now just slow down
                    pass
            elif plan.status == PlannerResultStatus.DETOUR:
                self._no_path_streak = 0
                if self._obstacle_critical:
                    self._transition_to(RuntimePhase.STOP_HOVER, reason="critical_obstacle_replan")
                elif self._obstacle_warn:
                    self._transition_to(RuntimePhase.WARN_DRIFT, reason="obstacle_warn_drift")
                else:
                    self._transition_to(RuntimePhase.DETOUR_EXECUTION, reason="periodic_detour_found")
                return
            elif plan.status in {PlannerResultStatus.NO_PATH, PlannerResultStatus.BLOCKED}:
                self._no_path_streak += 1
                self._mark_current_zone_blocked(score=1.0 + 0.25 * self._no_path_streak)
                self._transition_to(RuntimePhase.STOP_HOVER, reason="path_blocked_or_no_path")
                return

        speed = self._active_target_speed * WARN_SLOWDOWN if self._obstacle_warn else self._active_target_speed
        self._avoidance_active = self._obstacle_warn
        self._step_toward(tx, ty, speed)

    def _do_warn_drift(self) -> None:
        if self._active_target_xy is None:
            self._transition_to(RuntimePhase.IDLE, reason="warn_drift_without_target")
            return

        if self._obstacle_critical:
            self._transition_to(RuntimePhase.STOP_HOVER, reason="critical_during_drift")
            return

        plan = self._run_planner()
        self._last_plan_result = plan
        self._last_planner_state = plan.planner_state
        if plan.status == PlannerResultStatus.DIRECT and not self._obstacle_warn:
            self._transition_to(RuntimePhase.CRUISE_TO_TARGET, reason="drift_corridor_cleared")
            return

        if plan.status in {PlannerResultStatus.NO_PATH, PlannerResultStatus.BLOCKED}:
            self._no_path_streak += 1
            self._transition_to(RuntimePhase.STOP_HOVER, reason="drift_path_blocked")
            return

        # Follow drift subgoal
        if plan.subgoal_xy:
            self._avoidance_active = True
            self._no_path_streak = 0
            self._step_toward(
                plan.subgoal_xy[0],
                plan.subgoal_xy[1],
                self._active_target_speed * WARN_SLOWDOWN,
            )
        else:
            self._transition_to(RuntimePhase.STOP_HOVER, reason="drift_missing_subgoal")

    def _do_stop_hover(self) -> None:
        self._vsp_x = self._drone_x
        self._vsp_y = self._drone_y
        self._vsp_z = -self._active_target_alt if self._active_target_xy is not None else self._drone_z
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, self._drone_yaw)

        if self._phase_ticks < STOP_HOVER_REPLAN_TICKS:
            return
        if self._obstacle_critical or self._no_path_streak > 0:
            self._transition_to(RuntimePhase.SCAN_360, reason="map_enrichment_needed")
            return
        self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="hover_replan_trigger")

    def _do_scan_360(self) -> None:
        if self._active_target_xy is None:
            self._scan_manager.reset()
            self._transition_to(RuntimePhase.IDLE, reason="scan_without_target")
            return

        result = self._scan_manager.step(
            pose_ned=(self._drone_x, self._drone_y, self._drone_z),
            yaw=self._drone_yaw,
            rgb_frame=self._latest_rgb_frame,
            depth_frame=self._latest_depth_frame,
            depth_ts=self._latest_depth_ts,
            obstacle_sectors=self._obstacle_sectors,
        )

        command = result.command
        if command is not None and command.hold_position:
            self._publish_setpoint(self._drone_x, self._drone_y, -self._active_target_alt, command.desired_yaw)

        if result.finished:
            next_phase = self._handle_scan_result(result)
            self._scan_manager.reset()
            self._transition_to(next_phase, reason="scan_complete")

    def _do_local_replan(self) -> None:
        if self._active_target_xy is None:
            self._transition_to(RuntimePhase.IDLE, reason="local_replan_without_target")
            return
        plan = self._run_planner()
        self._last_plan_result = plan
        self._last_planner_state = plan.planner_state
        if plan.status == PlannerResultStatus.DIRECT:
            self._no_path_streak = 0
            self._transition_to(RuntimePhase.CRUISE_TO_TARGET, reason="replan_direct")
            return
        if plan.status == PlannerResultStatus.DETOUR:
            self._no_path_streak = 0
            self._transition_to(RuntimePhase.DETOUR_EXECUTION, reason="replan_detour")
            return

        self._no_path_streak += 1
        self._mark_current_zone_blocked(score=1.0 + 0.5 * self._no_path_streak)
        if self._no_path_streak >= NO_PATH_BLOCKED_THRESHOLD and self._scan_attempts_for_target >= SCAN_RETRY_LIMIT:
            self._blocked_reason = "replan_failed_after_scan_retries"
            self._blocked_severity = "hard"
            self._transition_to(RuntimePhase.BLOCKED, reason="replan_failed_repeatedly")
            return
        if self._scan_attempts_for_target < SCAN_RETRY_LIMIT:
            self._transition_to(RuntimePhase.SCAN_360, reason="replan_failed_trigger_scan")
            return
        self._transition_to(RuntimePhase.STOP_HOVER, reason="replan_failed_hover")

    def _do_detour_execution(self) -> None:
        if self._active_target_xy is None or self._last_plan_result is None:
            self._transition_to(RuntimePhase.IDLE, reason="detour_without_target_or_plan")
            return

        plan = self._last_plan_result

        # Check if direct path returned
        if self._ticks % PLAN_INTERVAL_TICKS == 0:
            new_plan = self._run_planner()
            self._last_plan_result = new_plan
            self._last_planner_state = new_plan.planner_state
            if new_plan.status == PlannerResultStatus.DIRECT and not self._obstacle_warn:
                self._no_path_streak = 0
                self._transition_to(RuntimePhase.CRUISE_TO_TARGET, reason="detour_direct_cleared")
                return
            if new_plan.status in {PlannerResultStatus.NO_PATH, PlannerResultStatus.BLOCKED}:
                self._no_path_streak += 1
                self._transition_to(RuntimePhase.STOP_HOVER, reason="detour_replan_no_path")
                return
            plan = new_plan

        if plan.subgoal_xy:
            if self._distance_to(plan.subgoal_xy[0], plan.subgoal_xy[1]) < DETOUR_REACHED_DIST:
                self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="detour_subgoal_reached")
                return
            speed = self._active_target_speed * WARN_SLOWDOWN if self._obstacle_warn else self._active_target_speed
            self._avoidance_active = True
            self._step_toward(plan.subgoal_xy[0], plan.subgoal_xy[1], speed)
        else:
            self._transition_to(RuntimePhase.STOP_HOVER, reason="detour_lost_path")

    def _do_blocked(self) -> None:
        self._vsp_x = self._drone_x
        self._vsp_y = self._drone_y
        self._vsp_z = -self._active_target_alt if self._active_target_xy is not None else self._drone_z
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, self._drone_yaw)

        # Periodic retry
        if self._phase_ticks % BLOCKED_RETRY_TICKS == 0 and self._active_target_xy is not None:
            self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="blocked_retry_replan")

    def _do_return_home(self) -> None:
        # Return home is basically a cruise to the home target
        self._do_cruise_to_target()

    def _do_landing(self) -> None:
        if self._land_sent:
            return
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=4.0,
            param3=6.0,
        )
        self._land_sent = True

    def _do_abort(self) -> None:
        self._vsp_x = self._drone_x
        self._vsp_y = self._drone_y
        self._vsp_z = self._drone_z
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, self._drone_yaw)

    def _run_planner(self) -> PlanResult:
        grid = self._sync_planner_grid()
        start = PlannerPose(x=self._drone_x, y=self._drone_y, yaw=self._drone_yaw)
        tx, ty = self._active_target_xy or (self._drone_x, self._drone_y)
        mission_target = PlannerTarget(x=tx, y=ty)

        home_target = None
        if self._home_captured:
            home_target = PlannerTarget(x=self._home_x, y=self._home_y, label="home")

        return self._planner.plan(
            grid=grid,
            start=start,
            mission_target=mission_target,
            home_target=home_target,
            blocked_history=tuple(self._blocked_history),
            peer_drone_mask=tuple(),
        )

    def _sync_planner_grid(self) -> LocalGridSnapshot:
        snap = self._local_grid_snapshot
        hard_occupancy = np.asarray(
            snap.occupied_mask | snap.dynamic_no_go_mask,
            dtype=np.bool_,
        )
        unknown_mask = np.asarray(snap.age_s > self._local_map_stale_after_s, dtype=np.bool_)
        if unknown_mask.ndim == 0:
            unknown_mask = np.full(hard_occupancy.shape, bool(unknown_mask), dtype=np.bool_)
        return LocalGridSnapshot(
            occupancy=hard_occupancy,
            resolution_m=snap.resolution_m,
            origin_x=snap.origin_ned[0],
            origin_y=snap.origin_ned[1],
            inflation_cost=snap.inflation_map,
            blocked_cost=snap.blocked_cost_layer,
            unknown_mask=unknown_mask,
            state=snap.state.name,
            stamp_s=snap.stamp_s,
        )

    def _complete_target(self, *, reason: str) -> None:
        self.get_logger().info(
            f"Target reached — {self._active_target_name} "
            f"NED({self._active_target_xy[0]:.1f}, {self._active_target_xy[1]:.1f})"
        )
        self._last_completed_target_id = self._active_target_id
        self._last_completed_target_name = self._active_target_name
        self._run_log.log(
            "target_completed",
            reason=reason,
            target_id=self._active_target_id,
            target_name=self._active_target_name,
            target_ned=[round(self._active_target_xy[0], 3), round(self._active_target_xy[1], 3)],
        )
        self._clear_active_target()
        self._avoidance_active = False
        self._transition_to(RuntimePhase.IDLE, reason=reason)

    def _distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self._drone_x, y - self._drone_y)

    def _is_at_target_altitude(self) -> bool:
        if not self._pos_valid:
            return False
        return abs(self._drone_z + self._active_target_alt) < ALT_TOL

    def _velocity_toward(self, x: float, y: float, speed: float) -> tuple[float, float]:
        dx = x - self._drone_x
        dy = y - self._drone_y
        d = math.hypot(dx, dy)
        if d < 0.05:
            return 0.0, 0.0
        return (dx / d) * speed, (dy / d) * speed

    def _update_obstacle_memory(self) -> None:
        # Decay/prune blocked history if it gets too large
        if len(self._blocked_history) > 24:
            self._blocked_history = self._blocked_history[-24:]

    def _transition_to(self, new_phase: RuntimePhase, *, reason: str, **fields: Any) -> None:
        old_phase = self._phase
        self._phase = new_phase
        self._phase_ticks = 0
        self._phase_enter_ts = time.time()

        if new_phase == RuntimePhase.SCAN_360 and self._active_target_xy is not None:
            self._scan_attempts_for_target += 1
            self._scan_manager.start_scan(
                reason=reason,
                pose_ned=(self._drone_x, self._drone_y, self._drone_z),
                yaw=self._drone_yaw,
                mission_target_ned=self._active_target_xy,
                target_id=self._active_target_id,
                target_name=self._active_target_name,
                phase_name=old_phase.name,
                closest_m=self._obstacle_closest,
                committed_side=self._avoid_commit_side,
            )

        if new_phase == RuntimePhase.BLOCKED:
            if not self._blocked_reason:
                self._blocked_reason = reason
            if self._blocked_severity == "none":
                self._blocked_severity = "hard"
            if self._blocked_since_s <= 0.0:
                self._blocked_since_s = self._phase_enter_ts

        if new_phase in {
            RuntimePhase.CRUISE_TO_TARGET,
            RuntimePhase.WARN_DRIFT,
            RuntimePhase.DETOUR_EXECUTION,
            RuntimePhase.TAKEOFF,
        }:
            self._blocked_reason = ""
            self._blocked_severity = "none"
            self._blocked_since_s = 0.0

        planner_mode = self._planner_mode()
        planner_state = (
            self._last_planner_state.name
            if isinstance(self._last_planner_state, Enum)
            else str(self._last_planner_state)
        )
        event_payload = {
            "event": "phase_transition",
            "from_phase": old_phase.name,
            "to_phase": new_phase.name,
            "reason": reason,
            "target_id": self._active_target_id,
            "target_name": self._active_target_name,
            "planner_mode": planner_mode,
            "planner_state": planner_state,
            "no_path_streak": int(self._no_path_streak),
            "scan_attempts_for_target": int(self._scan_attempts_for_target),
            **fields,
        }
        self._run_log.log(
            "phase_transition",
            from_phase=old_phase.name,
            to_phase=new_phase.name,
            reason=reason,
            target_id=self._active_target_id,
            target_name=self._active_target_name,
            drone_ned=[
                round(float(self._drone_x), 3),
                round(float(self._drone_y), 3),
                round(float(self._drone_z), 3),
            ],
            setpoint_ned=[
                round(float(self._vsp_x), 3),
                round(float(self._vsp_y), 3),
                round(float(self._vsp_z), 3),
            ],
            planner_mode=planner_mode,
            planner_state=planner_state,
            **fields,
        )
        self._publish_runtime_event(event_payload)

    def _planner_mode(self) -> str:
        if self._last_plan_result is None:
            return "NONE"
        return str(self._last_plan_result.status.value)

    def _handle_scan_result(self, result: ScanStepResult) -> RuntimePhase:
        event_payload = result.complete_event.as_dict() if result.complete_event is not None else {
            "event": "scan_complete",
            "success": bool(result.success),
            "failure_reason": result.failure_reason,
            "free_directions": list(result.free_directions),
            "scan_best_sectors": dict(result.sector_distances),
        }
        event_payload["phase"] = self._phase.name
        self._last_scan_summary = dict(event_payload)
        self._publish_runtime_event(event_payload)

        if result.success:
            self._no_path_streak = 0
            self._blocked_reason = ""
            self._blocked_severity = "none"
            return RuntimePhase.LOCAL_REPLAN

        self._no_path_streak += 1
        self._mark_current_zone_blocked(score=1.5 + 0.5 * self._no_path_streak)
        self._blocked_reason = result.failure_reason or "scan_failed"
        self._blocked_severity = "hard"
        return RuntimePhase.BLOCKED

    def _mark_current_zone_blocked(self, *, score: float) -> None:
        if not self._pos_valid:
            return
        radius = max(1.0, self._loop_memory_radius_m)
        entry = BlockedHistoryEntry(
            x=self._drone_x,
            y=self._drone_y,
            radius_m=radius,
            score=float(score),
        )
        self._blocked_history.append(entry)
        self._local_mapper.mark_blocked_zone(
            x=self._drone_x,
            y=self._drone_y,
            radius_m=radius,
            score=float(score),
            stamp_s=time.time(),
            label=f"runtime_no_path_{self._no_path_streak}",
        )

    def _publish_runtime_event(self, payload: dict[str, Any]) -> None:
        safe_payload = dict(payload)
        safe_payload.setdefault("stamp_s", round(float(time.time()), 3))
        self._last_runtime_event = safe_payload
        msg = String(data=json.dumps(safe_payload, ensure_ascii=True))
        self._pub_events.publish(msg)
        if self._pub_events_legacy is not None:
            self._pub_events_legacy.publish(msg)

    def _step_toward(self, target_x: float, target_y: float, speed: float) -> None:
        vx, vy = self._velocity_toward(target_x, target_y, speed)
        ref_x = self._drone_x if self._pos_valid else self._vsp_x
        ref_y = self._drone_y if self._pos_valid else self._vsp_y
        dist_to_target = self._distance_to(target_x, target_y)
        step_distance = min(
            max(self._min_command_step_m, abs(speed) * self._setpoint_lookahead_s),
            max(dist_to_target, self._min_command_step_m),
        )
        direction_norm = math.hypot(vx, vy)
        if direction_norm > 1e-6:
            dir_x = vx / direction_norm
            dir_y = vy / direction_norm
        else:
            dir_x = 0.0
            dir_y = 0.0
        self._vsp_x = ref_x + dir_x * step_distance
        self._vsp_y = ref_y + dir_y * step_distance
        self._vsp_z = -self._active_target_alt
        yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.1 else self._drone_yaw
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, yaw)
        if self._pos_valid:
            self._actual_path.append((self._drone_x, self._drone_y, self._drone_z))

    def _capture_home_if_needed(self) -> None:
        if self._home_captured or not self._pos_valid:
            return
        self._home_x = self._drone_x
        self._home_y = self._drone_y
        self._home_captured = True
        self._run_log.log(
            "home_captured",
            home_ned=[round(float(self._home_x), 3), round(float(self._home_y), 3)],
        )

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_ocm.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = float("nan")) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float("nan")] * 3
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_sp.publish(msg)

    def _send_command(
        self,
        cmd: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(msg)
        self._run_log.log(
            "vehicle_command",
            command=int(cmd),
            param1=float(param1),
            param2=float(param2),
            param3=float(param3),
        )

    def _arm(self) -> None:
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _set_offboard_mode(self) -> None:
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _pub_status_cb(self) -> None:
        mapper_summary = self._local_mapper.summary()
        blocked_severity = str(self._blocked_severity or "none").strip().upper()
        if blocked_severity not in {"NONE", "SOFT", "HARD"}:
            blocked_severity = "NONE"
        blocked_active = (self._phase == RuntimePhase.BLOCKED) or (blocked_severity != "NONE")
        runtime_result = "ACTIVE"
        if blocked_active:
            runtime_result = "BLOCKED"
        elif self._active_command == "none":
            runtime_result = "IDLE"
        scan_state_name = self._scan_manager.state.name
        payload = {
            "phase": self._phase.name,
            "state": self._phase.name,
            "result": runtime_result,
            "command": self._active_command,
            "target_id": self._active_target_id,
            "target_name": self._active_target_name,
            "mission_name": self._active_target_name,
            "target_ned": None if self._active_target_xy is None else [
                round(float(self._active_target_xy[0]), 2),
                round(float(self._active_target_xy[1]), 2),
            ],
            "subgoal_ned": None if not self._last_plan_result or not self._last_plan_result.subgoal_xy else [
                round(float(self._last_plan_result.subgoal_xy[0]), 2),
                round(float(self._last_plan_result.subgoal_xy[1]), 2),
            ],
            "home_ned": None if not self._home_captured else [
                round(float(self._home_x), 2),
                round(float(self._home_y), 2),
            ],
            "home_captured": bool(self._home_captured),
            "navigator_ready": bool(self._pos_valid),
            "target_reached": False,
            "last_completed_target_id": self._last_completed_target_id,
            "last_completed_target_name": self._last_completed_target_name,
            "avoidance_active": self._avoidance_active,
            "obstacle_warn": self._obstacle_warn,
            "obstacle_critical": self._obstacle_critical,
            "obstacle_closest_m": round(float(self._obstacle_closest), 2),
            "free_directions": self._free_directions,
            "committed_side": self._avoid_commit_side,
            "planner_mode": self._planner_mode(),
            "planner_state": self._last_planner_state.name
            if isinstance(self._last_planner_state, Enum)
            else str(self._last_planner_state),
            "corridor_width_m": round(float(self._last_plan_result.corridor_width_m or 0.0), 2) if self._last_plan_result else 0.0,
            "blocked_history_len": len(self._blocked_history),
            "no_path_streak": int(self._no_path_streak),
            "scan_attempts_for_target": int(self._scan_attempts_for_target),
            "blocked_reason": self._blocked_reason,
            "blocked_severity": blocked_severity,
            "reassign_recommended": blocked_severity == "HARD",
            "blocked_since_s": round(float(self._blocked_since_s), 2) if self._blocked_since_s > 0.0 else 0.0,
            "drone_ned": [
                round(float(self._drone_x), 2),
                round(float(self._drone_y), 2),
                round(float(self._drone_z), 2),
            ],
            "scan_state": scan_state_name,
            "scan_active": scan_state_name not in {"IDLE", "COMPLETE", "FAILED"},
            "mapper_state": mapper_summary["state"],
            "local_map_age_s": round(float(self._local_grid_snapshot.age_s), 2),
            "dense_scan_points": int(mapper_summary["dense_scan_points"]),
            "last_scan": self._last_scan_summary,
            "last_runtime_event": self._last_runtime_event,
            "elapsed_s": round(time.time() - self._start_time, 1),
        }
        msg = String(data=json.dumps(payload))
        self._pub_status.publish(msg)
        if self._pub_status_legacy is not None:
            self._pub_status_legacy.publish(msg)

        avoid_msg = Bool(data=self._avoidance_active)
        self._pub_avoid.publish(avoid_msg)
        if self._pub_avoid_legacy is not None:
            self._pub_avoid_legacy.publish(avoid_msg)

        now = time.time()
        if (now - self._last_status_log_ts) >= 1.0:
            self._last_status_log_ts = now
            self._run_log.log(
                "runtime_status",
                **payload,
                obstacle_sectors={k: round(float(v), 3) for k, v in self._obstacle_sectors.items()},
                subgoal_ned=payload["subgoal_ned"],
                depth_stats=self._last_depth_stats,
                mapper_summary=mapper_summary,
            )

    def _sync_obstacle_summary(self) -> None:
        self._obstacle_sectors = {
            "left": float(self._clearance_summary.left_m),
            "center": float(self._clearance_summary.center_m),
            "right": float(self._clearance_summary.right_m),
        }
        self._obstacle_closest = float(self._clearance_summary.closest_m)
        self._obstacle_warn = bool(self._clearance_summary.warn)
        self._obstacle_critical = bool(self._clearance_summary.critical)
        self._free_directions = list(self._clearance_summary.free_directions)

    def _pub_viz_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()

        plan_msg = Path()
        plan_msg.header.frame_id = "map"
        plan_msg.header.stamp = stamp
        
        # Add current position as start of visual path
        ps_start = PoseStamped()
        ps_start.header = plan_msg.header
        ps_start.pose.position.x = float(self._drone_y)
        ps_start.pose.position.y = float(self._drone_x)
        ps_start.pose.position.z = float(-self._drone_z)
        plan_msg.poses.append(ps_start)

        if self._last_plan_result and self._last_plan_result.path_xy:
            for px, py in self._last_plan_result.path_xy:
                ps = PoseStamped()
                ps.header = plan_msg.header
                ps.pose.position.x = float(py)
                ps.pose.position.y = float(px)
                ps.pose.position.z = float(self._active_target_alt)
                plan_msg.poses.append(ps)
        elif self._active_target_xy is not None:
            # Fallback to direct line to target for viz if no plan yet
            ps = PoseStamped()
            ps.header = plan_msg.header
            ps.pose.position.x = float(self._active_target_xy[1])
            ps.pose.position.y = float(self._active_target_xy[0])
            ps.pose.position.z = float(self._active_target_alt)
            plan_msg.poses.append(ps)
            
        self._pub_plan.publish(plan_msg)
        if self._pub_plan_legacy is not None:
            self._pub_plan_legacy.publish(plan_msg)

        actual_msg = Path()
        actual_msg.header.frame_id = "map"
        actual_msg.header.stamp = stamp
        for px, py, pz in self._actual_path:
            ps = PoseStamped()
            ps.header = actual_msg.header
            ps.pose.position.x = float(py)
            ps.pose.position.y = float(px)
            ps.pose.position.z = float(-pz)
            actual_msg.poses.append(ps)
        self._pub_actual.publish(actual_msg)
        if self._pub_actual_legacy is not None:
            self._pub_actual_legacy.publish(actual_msg)

    def close_log(self) -> None:
        self._run_log.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
