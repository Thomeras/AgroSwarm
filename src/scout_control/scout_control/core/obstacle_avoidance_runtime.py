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
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String

from scout_control.avoidance.altitude_controller import AltitudeController
from scout_control.avoidance.depth_projector import DepthProjector
from scout_control.avoidance.flight_phase_machine import FlightPhaseMachine
from scout_control.avoidance.health_monitor import HealthConfig, RuntimeHealthMonitor
from scout_control.avoidance.lidar_projector import (
    body_to_world_points,
    laser_scan_to_body_points,
)
from scout_control.avoidance.local_mapper import (
    LocalClearanceSummary,
    LocalMapper,
    LocalMapperConfig,
    LocalMapperSnapshot,
)
from scout_control.avoidance.local_planner import (
    BlockedHistoryEntry,
    DynamicMaskDisk,
    LocalGridSnapshot,
    LocalPlanner,
    LocalPlannerConfig,
    LocalPlannerState,
    PlanResult,
    PlannerPose,
    PlannerResultStatus,
    PlannerTarget,
)
from scout_control.avoidance.px4_publisher_adapter import PX4PublisherAdapter
from scout_control.avoidance.ros_io_adapter import RosIOAdapter
from scout_control.avoidance.scan_manager import (
    SCAN_POINT_MAX_RANGE_M,
    SCAN_POINT_MIN_RANGE_M,
    ScanManager,
)
from scout_control.avoidance.telemetry_hub import Px4InputOwnershipGuard, TelemetryHub
from scout_control.avoidance.types import ScanStepResult, TargetCommand
from scout_control.avoidance.types import (
    normalize_target_command_payload,
    target_command_from_msg,
)
from scout_control.avoidance.avoidance_logging import AvoidanceRunLogger

try:
    from scout_control_msgs.msg import (
        AvoidanceStatus as ScoutAvoidanceStatusMsg,
        PeerTelemetry,
        TargetCommand as ScoutTargetCommandMsg,
    )
except ImportError:  # Source-tree tests run before ROS interface generation.
    ScoutAvoidanceStatusMsg = None
    PeerTelemetry = None
    ScoutTargetCommandMsg = None

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
QOS_TARGET_CMD = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_RTH_TARGET = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
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
COMMAND_RETRY_INTERVAL_S = 1.0
MANUAL_VELOCITY_STALE_S = 0.35


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
    MANUAL_VELOCITY = auto()
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
        self.declare_parameter("local_map_self_filter_radius_m", 1.0)
        self.declare_parameter("local_map_peer_cost_gain", 1.0)
        self.declare_parameter("local_map_blocked_cost_gain", 1.0)
        self.declare_parameter("peer_drone_ids", [])
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("terrain_range_topic", "")
        self.declare_parameter("enable_lidar_obstacle_points", False)
        self.declare_parameter("lidar_obstacle_topic", "")
        self.declare_parameter("lidar_obstacle_confidence", 0.6)
        self.declare_parameter("lidar_obstacle_stride", 1)
        self.declare_parameter("lidar_obstacle_stale_after_s", 0.5)
        self.declare_parameter("publish_legacy_obstacle_topics", False)
        self.declare_parameter("log_run_label", "")
        self.declare_parameter("pose_stale_after_s", 0.5)
        self.declare_parameter("depth_stale_after_s", 1.0)
        self.declare_parameter("xy_reset_quarantine_s", 0.5)
        self.declare_parameter("require_depth_for_navigation", True)
        self.declare_parameter("relax_heading_gate", False)
        self.declare_parameter("relax_xy_gate", False)
        self.declare_parameter("relax_dead_reckoning_gate", False)
        self.declare_parameter("force_arm", False)
        self.declare_parameter("altitude_policy_mode", "FixedNED")
        self.declare_parameter("local_origin_ned_x", 0.0)
        self.declare_parameter("local_origin_ned_y", 0.0)

        self._drone_id = int(self.get_parameter("drone_id").value)
        self._local_origin_x = float(self.get_parameter("local_origin_ned_x").value)
        self._local_origin_y = float(self.get_parameter("local_origin_ned_y").value)
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
        self._local_map_self_filter_radius_m = max(
            0.0, float(self.get_parameter("local_map_self_filter_radius_m").value)
        )
        self._local_map_peer_cost_gain = float(
            self.get_parameter("local_map_peer_cost_gain").value
        )
        self._local_map_blocked_cost_gain = float(
            self.get_parameter("local_map_blocked_cost_gain").value
        )
        self._enable_lidar_obstacle_points = bool(
            self.get_parameter("enable_lidar_obstacle_points").value
        )
        self._lidar_obstacle_topic = str(
            self.get_parameter("lidar_obstacle_topic").value or ""
        ).strip()
        self._lidar_obstacle_confidence = max(
            0.0,
            float(self.get_parameter("lidar_obstacle_confidence").value),
        )
        self._lidar_obstacle_stride = max(
            1,
            int(self.get_parameter("lidar_obstacle_stride").value),
        )
        self._lidar_obstacle_stale_after_s = max(
            0.0,
            float(self.get_parameter("lidar_obstacle_stale_after_s").value),
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
        self._health_monitor = RuntimeHealthMonitor(
            HealthConfig(
                pose_stale_after_s=max(0.1, float(self.get_parameter("pose_stale_after_s").value)),
                depth_stale_after_s=max(
                    0.1,
                    float(self.get_parameter("depth_stale_after_s").value),
                ),
                xy_reset_quarantine_s=max(
                    0.0, float(self.get_parameter("xy_reset_quarantine_s").value)
                ),
                require_depth_for_navigation=bool(
                    self.get_parameter("require_depth_for_navigation").value
                ),
                relax_heading_gate=bool(
                    self.get_parameter("relax_heading_gate").value
                ),
                relax_xy_gate=bool(
                    self.get_parameter("relax_xy_gate").value
                ),
                relax_dead_reckoning_gate=bool(
                    self.get_parameter("relax_dead_reckoning_gate").value
                ),
            )
        )
        self._force_arm = bool(self.get_parameter("force_arm").value)
        self._runtime_readiness = self._health_monitor.evaluate(
            now_s=time.time(),
            command_active=False,
        )
        self._altitude_controller = AltitudeController(
            mode=str(self.get_parameter("altitude_policy_mode").value),
            default_altitude_m=self._default_alt,
        )

        self._telemetry = TelemetryHub(
            drone_id=self._drone_id,
            camera_topic=str(self.get_parameter("camera_topic").value),
            depth_topic=str(self.get_parameter("depth_topic").value),
            camera_info_topic=str(self.get_parameter("camera_info_topic").value),
            terrain_range_topic=str(self.get_parameter("terrain_range_topic").value),
        )
        topics = self._telemetry.topics
        swarm_topics = self._telemetry.swarm
        self._camera_topic = topics.camera_image
        self._depth_topic = topics.depth_image
        self._camera_info_topic = topics.camera_info
        self._terrain_range_topic = topics.terrain_range

        self._phase_machine: FlightPhaseMachine[RuntimePhase] = FlightPhaseMachine(
            RuntimePhase.IDLE
        )
        self._ticks = 0
        self._land_sent = False
        self._px4_armed = False
        self._px4_offboard_enabled = False
        self._px4_nav_state = -1
        self._px4_failsafe = False
        self._last_arm_request_ts = 0.0
        self._last_offboard_request_ts = 0.0
        self._last_vehicle_command_ack: dict[str, Any] | None = None

        self._drone_x = 0.0
        self._drone_y = 0.0
        self._drone_z = 0.0
        self._drone_yaw = 0.0
        self._desired_hover_yaw: float = float("nan")
        self._pos_valid = False

        self._vsp_x = 0.0
        self._vsp_y = 0.0
        self._vsp_z = 0.0
        self._manual_velocity_ned = (0.0, 0.0, 0.0)
        self._manual_yaw_rate = 0.0
        self._manual_velocity_last_ts = 0.0

        self._home_x = 0.0
        self._home_y = 0.0
        self._home_captured = False

        self._active_command = "none"
        self._active_target_id = ""
        self._active_target_name = ""
        self._active_target_xy: tuple[float, float] | None = None
        self._active_target_world_xy: tuple[float, float] | None = None
        self._active_target_alt = self._default_alt
        self._active_target_speed = self._default_cruise
        self._active_target_clear_dist = self._default_clear_d
        self._last_completed_target_id = ""
        self._last_completed_target_name = ""
        self._last_completed_target_ts = 0.0
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
        self._last_camera_info_ts = 0.0
        self._last_rgb_encoding = ""
        self._last_terrain_range_m: float | None = None
        self._last_terrain_range_ts = 0.0
        self._last_lidar_obstacle_ts = 0.0
        self._last_lidar_obstacle_points = 0
        self._last_scan_summary: dict[str, Any] | None = None
        self._last_runtime_event: dict[str, Any] | None = None
        self._last_setpoint_gate_reason = ""
        self._last_safety_action_reason = ""
        self._last_owner_conflict_log_ts = 0.0

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
                self_filter_radius_m=self._local_map_self_filter_radius_m,
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

        self._px4_in_topics = topics.px4_input_topics
        self._px4_publishers = PX4PublisherAdapter.create(
            self,
            topics=self._px4_in_topics,
            qos_profile=QOS_PX4_PUB,
        )
        self._px4_ownership_guard = Px4InputOwnershipGuard(
            topics=list(self._px4_in_topics.values()),
            expected_publishers=1,
        )
        self._ros_io = RosIOAdapter(
            string_type=String,
            bool_type=Bool,
            avoidance_status_type=ScoutAvoidanceStatusMsg,
        )

        self._pub_detected = self.create_publisher(
            String, topics.obstacles_detected, QOS_VIZ
        )
        self._pub_clear = self.create_publisher(
            Bool, topics.obstacles_clear, QOS_VIZ
        )
        self._pub_status = self.create_publisher(
            ScoutAvoidanceStatusMsg or String, topics.avoidance_status, QOS_STATUS
        )
        self._pub_status_json = self.create_publisher(
            String, topics.avoidance_status_json, QOS_STATUS
        )
        self._pub_plan = self.create_publisher(
            Path, topics.avoidance_planned_path, QOS_VIZ
        )
        self._pub_actual = self.create_publisher(
            Path, topics.avoidance_actual_path, QOS_VIZ
        )
        self._pub_avoid = self.create_publisher(
            Bool, topics.avoidance_active, QOS_VIZ
        )
        self._pub_events = self.create_publisher(
            String, topics.avoidance_events, QOS_EVENTS
        )
        self._pub_peer_telemetry = None
        if PeerTelemetry is not None:
            self._pub_peer_telemetry = self.create_publisher(
                PeerTelemetry, swarm_topics.peer_telemetry, QOS_SENSOR
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
            topics.vehicle_local_position,
            self._pos_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(
            VehicleStatus,
            topics.vehicle_status,
            self._vehicle_status_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(
            VehicleControlMode,
            topics.vehicle_control_mode,
            self._vehicle_control_mode_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(
            VehicleCommandAck,
            topics.vehicle_command_ack,
            self._vehicle_command_ack_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(Image, self._camera_topic, self._rgb_cb, QOS_SENSOR)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, QOS_SENSOR)
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_cb,
            QOS_SENSOR,
        )
        self.create_subscription(
            LaserScan,
            self._terrain_range_topic,
            self._terrain_range_cb,
            QOS_SENSOR,
        )
        self._lidar_obstacle_sub = None
        if self._enable_lidar_obstacle_points and not self._lidar_obstacle_topic:
            self.get_logger().warn(
                "enable_lidar_obstacle_points=true but lidar_obstacle_topic is empty; "
                "leaving LaserScan obstacle ingestion disabled so downward terrain range "
                "is not treated as a horizontal obstacle"
            )
            self._enable_lidar_obstacle_points = False
        if self._enable_lidar_obstacle_points:
            lidar_topic = self._lidar_obstacle_topic
            self._lidar_obstacle_sub = self.create_subscription(
                LaserScan,
                lidar_topic,
                self._lidar_obstacle_cb,
                QOS_SENSOR,
            )
        self.create_subscription(
            ScoutTargetCommandMsg or String,
            topics.avoidance_target_cmd,
            self._target_cmd_cb,
            QOS_TARGET_CMD,
        )
        if ScoutTargetCommandMsg is not None:
            self.create_subscription(
                String,
                topics.avoidance_target_cmd_json,
                self._target_cmd_json_cb,
                QOS_TARGET_CMD,
            )
        self._peer_pos_subs = []
        if PeerTelemetry is not None:
            self._peer_pos_subs.append(
                self.create_subscription(
                    PeerTelemetry,
                    swarm_topics.peer_telemetry,
                    self._peer_telemetry_cb,
                    QOS_SENSOR,
                )
            )
        self.create_subscription(
            Point,
            topics.rth_target,
            self._rth_target_cb,
            QOS_RTH_TARGET,
        )

        self.create_timer(DT, self._control_loop)
        self.create_timer(0.1, self._publish_obstacle_state)
        self.create_timer(1.0, self._pub_status_cb)
        self.create_timer(0.2, self._pub_viz_cb)
        self.create_timer(2.0, self._check_px4_input_ownership)

        self.get_logger().info(
            f"obstacle_avoidance_runtime ready — drone_id={self._drone_id} "
            f"default_alt={self._default_alt}m default_speed={self._default_cruise}m/s "
            f"local_origin_ned=({self._local_origin_x:.2f},{self._local_origin_y:.2f}) "
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
            camera_info_topic=self._camera_info_topic,
            terrain_range_topic=self._terrain_range_topic,
            lidar_obstacle_enabled=bool(self._enable_lidar_obstacle_points),
            lidar_obstacle_topic=self._lidar_obstacle_topic,
            target_cmd_topic=topics.avoidance_target_cmd,
            event_topic=topics.avoidance_events,
            local_origin_ned=[
                round(float(self._local_origin_x), 3),
                round(float(self._local_origin_y), 3),
            ],
        )

    @property
    def _phase(self) -> RuntimePhase:
        return self._phase_machine.phase

    @property
    def _phase_ticks(self) -> int:
        return self._phase_machine.ticks

    @property
    def _phase_enter_ts(self) -> float:
        return self._phase_machine.entered_at_s

    def _log_run_event(self, event: str, **fields: Any) -> None:
        self._run_log.log(event, **fields)

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        health = self._health_monitor.update_pose_message(msg, now_s=time.time())
        self._runtime_readiness = self._health_monitor.evaluate(
            now_s=time.time(),
            command_active=self._command_active(),
            owner_conflict=self._px4_ownership_guard.conflict,
        )
        self._pos_valid = bool(health.valid)
        if bool(getattr(msg, "xy_valid", False)):
            self._drone_x = msg.x
            self._drone_y = msg.y
            self._drone_z = msg.z
            self._drone_yaw = msg.heading
        if not health.valid:
            return
        self._local_mapper.update_pose(
            self._drone_x,
            self._drone_y,
            self._drone_z,
            self._drone_yaw,
            time.time(),
        )
        self._publish_peer_telemetry(msg)

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self._px4_armed = bool(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self._px4_nav_state = int(msg.nav_state)
        self._px4_failsafe = bool(msg.failsafe)

    def _vehicle_control_mode_cb(self, msg: VehicleControlMode) -> None:
        self._px4_armed = bool(msg.flag_armed)
        self._px4_offboard_enabled = bool(msg.flag_control_offboard_enabled)

    def _vehicle_command_ack_cb(self, msg: VehicleCommandAck) -> None:
        ack_payload = {
            "command": int(msg.command),
            "result": int(msg.result),
            "result_param1": int(msg.result_param1),
            "result_param2": int(msg.result_param2),
            "from_external": bool(msg.from_external),
        }
        self._last_vehicle_command_ack = ack_payload
        self._run_log.log("vehicle_command_ack", **ack_payload)
        if msg.command in {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        }:
            self._publish_runtime_event(
                {
                    "event": "vehicle_command_ack",
                    **ack_payload,
                }
            )

    def _publish_peer_telemetry(self, msg: VehicleLocalPosition) -> None:
        if self._pub_peer_telemetry is None or PeerTelemetry is None:
            return
        out = PeerTelemetry()
        out.drone_id = f"drone_{self._drone_id}"
        out.position_ned = [float(msg.x), float(msg.y), float(msg.z)]
        out.velocity_ned = [
            float(msg.vx) if bool(getattr(msg, "v_xy_valid", False)) else 0.0,
            float(msg.vy) if bool(getattr(msg, "v_xy_valid", False)) else 0.0,
            float(getattr(msg, "vz", 0.0)) if bool(getattr(msg, "v_z_valid", False)) else 0.0,
        ]
        out.heading = float(msg.heading)
        out.valid = bool(getattr(msg, "xy_valid", True))
        out.stamp_ms = int(time.time() * 1000)
        out.source = "obstacle_avoidance_runtime"
        out.json_payload = json.dumps(
            {
                "drone_id": out.drone_id,
                "position_ned": list(out.position_ned),
                "velocity_ned": list(out.velocity_ned),
                "heading": out.heading,
                "valid": out.valid,
                "stamp_ms": out.stamp_ms,
                "source": out.source,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self._pub_peer_telemetry.publish(out)

    def _peer_telemetry_cb(self, msg: Any) -> None:
        peer_name = str(getattr(msg, "drone_id", ""))
        if peer_name == f"drone_{self._drone_id}":
            return
        if self._peer_drone_ids:
            try:
                peer_id = int(peer_name.split("_")[-1])
            except (TypeError, ValueError):
                return
            if peer_id not in self._peer_drone_ids:
                return
        if not bool(getattr(msg, "valid", False)):
            return
        position = list(getattr(msg, "position_ned", []) or [])
        velocity = list(getattr(msg, "velocity_ned", []) or [])
        if len(position) < 3:
            return
        self._local_mapper.ingest_peer_position(
            peer_name,
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            stamp_s=time.time(),
            vx=float(velocity[0]) if len(velocity) >= 1 else 0.0,
            vy=float(velocity[1]) if len(velocity) >= 2 else 0.0,
        )

    def _rgb_cb(self, msg: Image) -> None:
        try:
            encoding = str(getattr(msg, "encoding", "") or "").lower()
            self._last_rgb_encoding = encoding
            if encoding == "rgb8":
                rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self._latest_rgb_frame = rgb[:, :, ::-1].copy()
            elif encoding in {"mono8", "8uc1"}:
                gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                self._latest_rgb_frame = np.repeat(gray[:, :, None], 3, axis=2)
            else:
                self._latest_rgb_frame = self._bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8",
                )
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB CvBridge conversion failed: {exc}")

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        try:
            self._depth_projector.set_camera_info(msg)
            self._scan_manager.set_camera_info(msg)
            self._last_camera_info_ts = time.time()
        except ValueError as exc:
            self.get_logger().warn(f"CameraInfo rejected by depth projector: {exc}")

    def _terrain_range_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(list(getattr(msg, "ranges", []) or []), dtype=np.float32)
        if ranges.size == 0:
            self._altitude_controller.update_terrain_reference(terrain_z_ned=None)
            self._last_terrain_range_m = None
            return
        range_min = float(getattr(msg, "range_min", 0.0) or 0.0)
        range_max = float(getattr(msg, "range_max", 0.0) or 0.0)
        valid = np.isfinite(ranges)
        if range_min > 0.0:
            valid &= ranges >= range_min
        if range_max > 0.0:
            valid &= ranges <= range_max
        if not np.any(valid):
            self._altitude_controller.update_terrain_reference(terrain_z_ned=None)
            self._last_terrain_range_m = None
            return

        terrain_range_m = float(np.median(ranges[valid]))
        self._last_terrain_range_m = terrain_range_m
        self._last_terrain_range_ts = time.time()
        if self._pos_valid:
            self._altitude_controller.update_terrain_reference(
                terrain_z_ned=float(self._drone_z) + terrain_range_m,
            )

    def _lidar_obstacle_cb(self, msg: LaserScan) -> None:
        if not self._enable_lidar_obstacle_points or not self._pos_valid:
            return

        now_s = time.time()
        stamp_s = self._stamp_from_msg(msg, fallback_s=now_s)
        if (
            self._lidar_obstacle_stale_after_s > 0.0
            and stamp_s > 0.0
            and self._same_time_epoch(now_s, stamp_s)
            and (now_s - stamp_s) > self._lidar_obstacle_stale_after_s
        ):
            return

        try:
            body_batch = laser_scan_to_body_points(
                ranges=getattr(msg, "ranges", []),
                angle_min_rad=float(getattr(msg, "angle_min", 0.0) or 0.0),
                angle_increment_rad=float(getattr(msg, "angle_increment", 0.0) or 0.0),
                range_min_m=float(getattr(msg, "range_min", 0.0) or 0.0),
                range_max_m=float(getattr(msg, "range_max", 0.0) or 0.0),
                stamp_s=stamp_s,
                source="lidar",
                confidence=self._lidar_obstacle_confidence,
                stride=self._lidar_obstacle_stride,
            )
            world_batch = body_to_world_points(
                body_batch,
                origin_ned=(self._drone_x, self._drone_y, self._drone_z),
                yaw_rad=self._drone_yaw,
                source="lidar",
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"LiDAR obstacle scan rejected: {exc}")
            return

        if world_batch.point_count <= 0:
            return
        inserted = self._local_mapper.ingest_point_batch(world_batch)
        self._last_lidar_obstacle_ts = now_s
        self._last_lidar_obstacle_points = int(world_batch.point_count)
        self._run_log.log(
            "lidar_obstacle_points_ingested",
            topic=self._lidar_obstacle_topic,
            points=int(world_batch.point_count),
            inserted=int(inserted),
            confidence=round(float(world_batch.confidence), 3),
        )

    def _stamp_from_msg(self, msg: Any, *, fallback_s: float) -> float:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return float(fallback_s)
        sec = float(getattr(stamp, "sec", 0.0) or 0.0)
        nanosec = float(getattr(stamp, "nanosec", 0.0) or 0.0)
        if sec <= 0.0 and nanosec <= 0.0:
            return float(fallback_s)
        return sec + nanosec * 1e-9

    def _same_time_epoch(self, a_s: float, b_s: float) -> bool:
        wall_epoch_floor = 1_000_000_000.0
        return (float(a_s) >= wall_epoch_floor) == (float(b_s) >= wall_epoch_floor)

    def _world_to_local_xy(self, x: float, y: float) -> tuple[float, float]:
        return (float(x) - self._local_origin_x, float(y) - self._local_origin_y)

    def _local_to_world_xy(self, x: float, y: float) -> tuple[float, float]:
        return (float(x) + self._local_origin_x, float(y) + self._local_origin_y)

    def _rth_target_cb(self, msg: Point) -> None:
        world_x = float(msg.x)
        world_y = float(msg.y)
        self._home_x, self._home_y = self._world_to_local_xy(world_x, world_y)
        self._home_captured = True
        if self._active_command == "return_home" and self._active_target_xy is not None:
            self._active_target_xy = (self._home_x, self._home_y)
            self._active_target_world_xy = (world_x, world_y)
            self._run_log.log(
                "return_home_target_updated",
                home_ned=[
                    round(float(world_x), 3),
                    round(float(world_y), 3),
                ],
                home_local_ned=[
                    round(float(self._home_x), 3),
                    round(float(self._home_y), 3),
                ],
            )
        self.get_logger().info(
            f"home set from rth_target: world NED({world_x:.2f}, {world_y:.2f}) "
            f"-> local NED({self._home_x:.2f}, {self._home_y:.2f})"
        )

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
        self._health_monitor.update_depth_frame(
            now_s=self._latest_depth_ts,
            valid_samples=int(self._last_depth_stats["valid_samples"]),
        )
        self._ingest_depth_points(depth)

    def _normalize_target_command_payload(self, payload: Any) -> dict[str, Any]:
        return normalize_target_command_payload(payload)

    def _publish_command_feedback(
        self,
        *,
        accepted: bool,
        command: str,
        target_id: str,
        reason: str = "",
        payload: dict[str, Any] | None = None,
    ) -> None:
        event_payload: dict[str, Any] = {
            "event": "command_accepted" if accepted else "command_rejected",
            "accepted": bool(accepted),
            "command": command,
            "target_id": target_id,
            "reason": reason,
            "phase": self._phase.name,
        }
        if payload is not None:
            event_payload["payload"] = payload
        self._publish_runtime_event(event_payload)

    def _target_cmd_json_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid target command JSON: {exc}")
            self._publish_command_feedback(
                accepted=False,
                command="",
                target_id="",
                reason=f"invalid_json:{exc}",
            )
            return

        self._handle_target_command_payload(data)

    def _target_cmd_cb(self, msg: Any) -> None:
        if ScoutTargetCommandMsg is not None and isinstance(msg, ScoutTargetCommandMsg):
            try:
                command = target_command_from_msg(msg)
                self._handle_target_command(command)
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(f"Invalid typed target command: {exc}")
                self._publish_command_feedback(
                    accepted=False,
                    command=str(getattr(msg, "command", "")),
                    target_id=str(getattr(msg, "target_id", "")),
                    reason=f"invalid_typed_payload:{exc}",
                )
            return
        self._target_cmd_json_cb(msg)

    def _handle_target_command_payload(self, data: Any) -> None:
        try:
            normalized = self._normalize_target_command_payload(data)
            has_clear_radius = (
                "clear_radius_m" in normalized
                and normalized.get("clear_radius_m") is not None
            )
            normalized.setdefault("altitude_m", self._default_alt)
            normalized.setdefault("cruise_speed_mps", self._default_cruise)
            command = TargetCommand.from_payload(normalized)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"Invalid target command payload: {exc}")
            raw = data if isinstance(data, dict) else {}
            self._publish_command_feedback(
                accepted=False,
                command=str(raw.get("command", raw.get("cmd", ""))),
                target_id=str(raw.get("target_id", raw.get("cmd_id", ""))),
                reason=f"invalid_payload:{exc}",
            )
            return
        self._handle_target_command(command, has_clear_radius=has_clear_radius)

    def _handle_target_command(
        self,
        command: TargetCommand,
        *,
        has_clear_radius: bool = True,
    ) -> None:
        cmd = command.command.strip().lower() or "goto"
        name = command.name or cmd
        target_id = command.target_id

        if cmd == "takeoff":
            if not self._pos_valid:
                self.get_logger().warn("takeoff command received before local position is valid")
                self._publish_command_feedback(
                    accepted=False,
                    command=cmd,
                    target_id=target_id,
                    reason="position_not_valid",
                )
                return
            self._vsp_x = self._drone_x
            self._vsp_y = self._drone_y
            world_x, world_y = self._local_to_world_xy(self._drone_x, self._drone_y)
            self._activate_target(
                command=cmd,
                target_id=target_id,
                name=name,
                target_xy=(self._drone_x, self._drone_y),
                world_target_xy=(world_x, world_y),
                altitude_m=float(command.altitude_m),
                cruise_speed=float(command.cruise_speed_mps),
                clear_dist=float(
                    command.clear_radius_m if has_clear_radius else self._default_clear_d
                ),
            )
            self._transition_to(
                RuntimePhase.TAKEOFF,
                reason="external_takeoff_command",
                target_id=target_id,
                target_name=name,
            )
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "goto":
            target = command.target_ned
            if target is None:
                self.get_logger().warn("goto command missing target_ned=[x,y]")
                self._publish_command_feedback(
                    accepted=False,
                    command=cmd,
                    target_id=target_id,
                    reason="missing_target_ned",
                )
                return
            local_target = self._world_to_local_xy(float(target[0]), float(target[1]))
            self._activate_target(
                command=cmd,
                target_id=target_id,
                name=name,
                target_xy=local_target,
                world_target_xy=(float(target[0]), float(target[1])),
                altitude_m=float(command.altitude_m),
                cruise_speed=float(command.cruise_speed_mps),
                clear_dist=float(
                    command.clear_radius_m if has_clear_radius else self._default_clear_d
                ),
            )
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "return_home":
            if not self._home_captured:
                self.get_logger().warn("return_home requested before home was captured")
                self._publish_command_feedback(
                    accepted=False,
                    command=cmd,
                    target_id=target_id,
                    reason="home_not_captured",
                )
                return
            self._activate_target(
                command=cmd,
                target_id=target_id,
                name=name,
                target_xy=(self._home_x, self._home_y),
                world_target_xy=self._local_to_world_xy(self._home_x, self._home_y),
                altitude_m=float(command.altitude_m),
                cruise_speed=float(command.cruise_speed_mps),
                clear_dist=float(
                    command.clear_radius_m if has_clear_radius else self._home_d
                ),
            )
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "hold":
            self._manual_velocity_ned = (0.0, 0.0, 0.0)
            self._manual_yaw_rate = 0.0
            self._manual_velocity_last_ts = 0.0
            self._clear_active_target()
            self._transition_to(RuntimePhase.STOP_HOVER, reason="external_hold_command")
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "manual_velocity":
            velocity = command.velocity_ned
            if velocity is None:
                self._publish_command_feedback(
                    accepted=False,
                    command=cmd,
                    target_id=target_id,
                    reason="missing_velocity_ned",
                )
                return
            self._clear_active_target()
            self._manual_velocity_ned = (
                float(velocity[0]),
                float(velocity[1]),
                float(velocity[2]),
            )
            self._manual_yaw_rate = (
                0.0 if math.isnan(command.yaw_rate_rad_s)
                else float(command.yaw_rate_rad_s)
            )
            self._manual_velocity_last_ts = time.time()
            self._transition_to(
                RuntimePhase.MANUAL_VELOCITY,
                reason="external_manual_velocity_command",
                target_id=target_id,
                target_name=name,
            )
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "land":
            self._manual_velocity_ned = (0.0, 0.0, 0.0)
            self._manual_yaw_rate = 0.0
            self._manual_velocity_last_ts = 0.0
            self._clear_active_target()
            self._land_sent = False
            self._transition_to(RuntimePhase.LANDING, reason="external_land_command")
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "cancel":
            self._manual_velocity_ned = (0.0, 0.0, 0.0)
            self._manual_yaw_rate = 0.0
            self._manual_velocity_last_ts = 0.0
            self._clear_active_target()
            self._transition_to(RuntimePhase.STOP_HOVER, reason="external_cancel_command")
            self._publish_command_feedback(
                accepted=True,
                command=cmd,
                target_id=target_id,
                payload=command.to_payload(),
            )
            return

        if cmd == "yaw_to":
            if not math.isnan(command.desired_yaw_rad):
                self._desired_hover_yaw = command.desired_yaw_rad
                if self._phase not in (RuntimePhase.STOP_HOVER,):
                    self._transition_to(RuntimePhase.STOP_HOVER, reason="yaw_to_command")
                self._publish_command_feedback(
                    accepted=True,
                    command=cmd,
                    target_id=target_id,
                    payload=command.to_payload(),
                )
            else:
                self._publish_command_feedback(
                    accepted=False,
                    command=cmd,
                    target_id=target_id,
                    reason="missing_desired_yaw_rad",
                )
            return

        self.get_logger().warn(f"Unknown target command: {cmd}")
        self._publish_command_feedback(
            accepted=False,
            command=cmd,
            target_id=target_id,
            reason="unknown_command",
        )

    def _activate_target(
        self,
        *,
        command: str,
        target_id: str,
        name: str,
        target_xy: tuple[float, float],
        world_target_xy: tuple[float, float],
        altitude_m: float,
        cruise_speed: float,
        clear_dist: float,
    ) -> None:
        previous_target = self._active_target_xy
        self._manual_velocity_ned = (0.0, 0.0, 0.0)
        self._manual_yaw_rate = 0.0
        self._manual_velocity_last_ts = 0.0
        self._active_command = command
        self._active_target_id = target_id
        self._active_target_name = name
        self._active_target_xy = target_xy
        self._active_target_world_xy = (
            world_target_xy if world_target_xy is not None
            else self._local_to_world_xy(target_xy[0], target_xy[1])
        )
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
        self._last_arm_request_ts = 0.0
        self._last_offboard_request_ts = 0.0
        self._desired_hover_yaw = float("nan")
        self._scan_manager.reset()
        if previous_target != target_xy:
            self._actual_path.clear()
        if self._phase in (
            RuntimePhase.STOP_HOVER,
            RuntimePhase.IDLE,
            RuntimePhase.LANDING,
        ):
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
            target_ned=[
                round(self._active_target_world_xy[0], 3),
                round(self._active_target_world_xy[1], 3),
            ],
            target_local_ned=[round(target_xy[0], 3), round(target_xy[1], 3)],
            altitude_m=round(float(altitude_m), 3),
            cruise_speed_mps=round(float(cruise_speed), 3),
            clear_dist_m=round(float(clear_dist), 3),
        )

    def _clear_active_target(self) -> None:
        self._active_command = "none"
        self._active_target_id = ""
        self._active_target_name = ""
        self._active_target_xy = None
        self._active_target_world_xy = None
        self._detour_target = None
        self._detour_side = "none"
        self._detour_strategy = "none"
        self._avoid_commit_side = "none"
        self._no_path_streak = 0
        self._scan_attempts_for_target = 0
        self._blocked_reason = ""
        self._last_arm_request_ts = 0.0
        self._last_offboard_request_ts = 0.0
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
        self._refresh_runtime_readiness()
        if not self._pos_valid:
            return
        body_batch = self._depth_projector.depth_to_body_points(
            depth,
            pixel_stride=self._local_map_depth_stride,
            stamp_s=self._latest_depth_ts,
            source="depth",
            is_dense_scan=False,
        )
        body_batch.confidence = 0.7
        world_batch = self._depth_projector.project_to_world_points(
            body_batch,
            origin_ned=(self._drone_x, self._drone_y, self._drone_z),
            yaw_rad=self._drone_yaw,
            source="depth",
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

    def _refresh_runtime_readiness(self) -> None:
        self._runtime_readiness = self._health_monitor.evaluate(
            now_s=time.time(),
            command_active=self._command_active(),
            owner_conflict=self._px4_ownership_guard.conflict,
        )
        self._pos_valid = bool(self._runtime_readiness.pose.valid)

    def _command_active(self) -> bool:
        return self._active_target_xy is not None or self._phase == RuntimePhase.MANUAL_VELOCITY

    def _enforce_runtime_safety(self) -> bool:
        readiness = self._runtime_readiness
        if readiness.navigation_allowed:
            if self._phase == RuntimePhase.ABORT and self._active_target_xy is not None:
                self._blocked_reason = ""
                self._blocked_severity = "none"
                self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="safety_recovered")
                return True
            self._last_safety_action_reason = ""
            return False

        reason = readiness.reason
        if reason == self._last_safety_action_reason and self._phase in {
            RuntimePhase.STOP_HOVER,
            RuntimePhase.BLOCKED,
            RuntimePhase.ABORT,
        }:
            if self._phase in {RuntimePhase.STOP_HOVER, RuntimePhase.BLOCKED}:
                self._publish_safety_hold_setpoint()
            elif self._phase == RuntimePhase.ABORT:
                self._do_abort()
            return True
        self._last_safety_action_reason = reason

        if readiness.severity == "hard":
            if self._phase == RuntimePhase.IDLE:
                return True
            self._blocked_reason = reason
            self._blocked_severity = "hard"
            self._transition_to(RuntimePhase.ABORT, reason=f"safety_abort:{reason}")
            self._do_abort()
            return True

        if self._active_target_xy is not None:
            self._blocked_reason = reason
            self._blocked_severity = "soft"
            if self._phase == RuntimePhase.IDLE:
                return True
            if self._phase == RuntimePhase.STOP_HOVER:
                self._transition_to(RuntimePhase.BLOCKED, reason=f"safety_blocked:{reason}")
                self._publish_safety_hold_setpoint()
            else:
                self._transition_to(RuntimePhase.STOP_HOVER, reason=f"safety_hold:{reason}")
                self._publish_safety_hold_setpoint()
            return True

        return False

    def _publish_safety_hold_setpoint(self) -> None:
        if not self._runtime_readiness.setpoint_publish_allowed:
            return
        self._vsp_x = self._drone_x
        self._vsp_y = self._drone_y
        self._vsp_z = self._target_z_setpoint() if self._active_target_xy is not None else self._drone_z
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, self._drone_yaw)

    def _target_z_setpoint(self) -> float:
        self._refresh_terrain_reference()
        return self._altitude_controller.setpoint_for(
            altitude_m=self._active_target_alt,
        ).z_ned

    def _refresh_terrain_reference(self) -> None:
        if self._last_terrain_range_m is None:
            return
        if (time.time() - self._last_terrain_range_ts) > 1.0:
            self._altitude_controller.update_terrain_reference(terrain_z_ned=None)
            self._last_terrain_range_m = None
            return
        if self._pos_valid:
            self._altitude_controller.update_terrain_reference(
                terrain_z_ned=float(self._drone_z) + float(self._last_terrain_range_m),
            )

    def _control_loop(self) -> None:
        self._ticks += 1
        self._phase_machine.tick()
        self._refresh_runtime_readiness()
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
        if self._enforce_runtime_safety():
            return

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
        elif self._phase == RuntimePhase.MANUAL_VELOCITY:
            self._do_manual_velocity()
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
            self._vsp_z = self._target_z_setpoint()
            self._transition_to(RuntimePhase.TAKEOFF, reason="target_active_begin_takeoff")

    def _do_takeoff(self) -> None:
        self._capture_home_if_needed()
        self._vsp_z = self._target_z_setpoint()
        self._ensure_takeoff_activation()
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
        if self._pos_valid and abs(self._drone_z - self._vsp_z) < ALT_TOL:
            self._local_mapper.clear_sensor_layers()
            if self._active_command == "takeoff":
                self._clear_active_target()
                self._transition_to(RuntimePhase.STOP_HOVER, reason="takeoff_hover_reached")
                return
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
                if self._planner_failure_should_block_current_zone(plan):
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
        self._vsp_z = self._target_z_setpoint() if self._active_target_xy is not None else self._drone_z
        hover_yaw = self._desired_hover_yaw if not math.isnan(self._desired_hover_yaw) else self._drone_yaw
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, hover_yaw)

        if self._phase_ticks < STOP_HOVER_REPLAN_TICKS:
            return
        if self._obstacle_critical or self._no_path_streak > 0:
            if self._scan_attempts_for_target >= SCAN_RETRY_LIMIT:
                self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="scan_retry_limit_replan")
                return
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
            self._publish_setpoint(
                self._drone_x,
                self._drone_y,
                self._target_z_setpoint(),
                command.desired_yaw,
            )

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
        if self._planner_failure_should_block_current_zone(plan):
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
        self._vsp_z = self._target_z_setpoint() if self._active_target_xy is not None else self._drone_z
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, self._drone_yaw)

        # Periodic retry
        if self._phase_ticks % BLOCKED_RETRY_TICKS == 0 and self._active_target_xy is not None:
            self._transition_to(RuntimePhase.LOCAL_REPLAN, reason="blocked_retry_replan")

    def _do_manual_velocity(self) -> None:
        if time.time() - self._manual_velocity_last_ts > MANUAL_VELOCITY_STALE_S:
            self._manual_velocity_ned = (0.0, 0.0, 0.0)
            self._manual_yaw_rate = 0.0
            self._transition_to(RuntimePhase.STOP_HOVER, reason="manual_velocity_stale")
            return

        vx, vy, vz = self._manual_velocity_ned
        self._publish_velocity_setpoint(vx, vy, vz, self._manual_yaw_rate)
        if self._pos_valid:
            self._vsp_x = self._drone_x
            self._vsp_y = self._drone_y
            self._vsp_z = self._drone_z
            self._actual_path.append((self._drone_x, self._drone_y, self._drone_z))

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
            peer_drone_mask=tuple(self._peer_planner_masks()),
        )

    def _peer_planner_masks(self) -> list[DynamicMaskDisk]:
        masks: list[DynamicMaskDisk] = []
        for item in self._local_mapper.peer_planner_mask_payload(now_s=time.time()):
            center = item.get("center_ned", [])
            if not isinstance(center, (list, tuple)) or len(center) < 2:
                continue
            hard_radius = float(item.get("hard_radius_m", 0.0) or 0.0)
            soft_radius = float(item.get("soft_radius_m", hard_radius) or hard_radius)
            if hard_radius > 0.0:
                masks.append(
                    DynamicMaskDisk(
                        x=float(center[0]),
                        y=float(center[1]),
                        radius_m=hard_radius,
                        hard=True,
                    )
                )
            if soft_radius > hard_radius:
                masks.append(
                    DynamicMaskDisk(
                        x=float(center[0]),
                        y=float(center[1]),
                        radius_m=soft_radius,
                        hard=False,
                        cost=float(item.get("weight", 1.0) or 1.0) * self._planner_peer_cost_scale,
                    )
                )
        return masks

    def _sync_planner_grid(self) -> LocalGridSnapshot:
        snap = self._local_grid_snapshot
        hard_occupancy = np.asarray(
            snap.occupied_mask,
            dtype=np.bool_,
        )
        unknown_mask = np.array(snap.unknown_mask, dtype=np.bool_, copy=True)
        if snap.age_s > self._local_map_stale_after_s:
            unknown_mask |= True
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
            valid_for_planning=bool(snap.valid_for_planning),
            validity_reason=str(snap.validity_reason),
        )

    def _complete_target(self, *, reason: str) -> None:
        self.get_logger().info(
            f"Target reached — {self._active_target_name} "
            f"NED({self._active_target_xy[0]:.1f}, {self._active_target_xy[1]:.1f})"
        )
        self._last_completed_target_id = self._active_target_id
        self._last_completed_target_name = self._active_target_name
        self._last_completed_target_ts = time.time()
        self._run_log.log(
            "target_completed",
            reason=reason,
            target_id=self._active_target_id,
            target_name=self._active_target_name,
            target_ned=[round(self._active_target_xy[0], 3), round(self._active_target_xy[1], 3)],
        )
        was_return_home = self._active_command == "return_home"
        self._clear_active_target()
        self._avoidance_active = False
        if was_return_home:
            self._transition_to(RuntimePhase.LANDING, reason="return_home_reached")
        else:
            self._transition_to(RuntimePhase.IDLE, reason=reason)

    def _distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self._drone_x, y - self._drone_y)

    def _is_at_target_altitude(self) -> bool:
        if not self._pos_valid:
            return False
        return abs(self._drone_z - self._target_z_setpoint()) < ALT_TOL

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
        transition = self._phase_machine.transition_to(
            new_phase,
            reason=reason,
            **fields,
        )
        old_phase = transition.old_phase

        if new_phase == RuntimePhase.SCAN_360 and self._active_target_xy is not None:
            self._scan_attempts_for_target += 1
            self._blocked_history.clear()
            self._local_mapper.clear_blocked_history()
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
                self._blocked_since_s = transition.entered_at_s

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
        log_fields = {
            "from_phase": old_phase.name,
            "to_phase": new_phase.name,
            "reason": reason,
            "target_id": self._active_target_id,
            "target_name": self._active_target_name,
            "drone_ned": [
                round(float(self._drone_x), 3),
                round(float(self._drone_y), 3),
                round(float(self._drone_z), 3),
            ],
            "setpoint_ned": [
                round(float(self._vsp_x), 3),
                round(float(self._vsp_y), 3),
                round(float(self._vsp_z), 3),
            ],
            "planner_mode": planner_mode,
            "planner_state": planner_state,
        }
        log_fields.update(fields)
        self._run_log.log("phase_transition", **log_fields)
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

    def _planner_failure_should_block_current_zone(self, plan: PlanResult | None) -> bool:
        if plan is None:
            return False
        if plan.planner_state == LocalPlannerState.DEGRADED:
            return False
        degraded_reasons = {
            "planner_map_not_ready",
            "start_cell_blocked",
            "start_outside_grid",
        }
        reason = str(plan.reason or "")
        return not any(reason.startswith(item) for item in degraded_reasons)

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
        self._last_runtime_event = self._ros_io.publish_runtime_event(
            publisher=self._pub_events,
            payload=payload,
            legacy_publisher=self._pub_events_legacy,
        )

    def _check_px4_input_ownership(self) -> None:
        ownership = self._px4_ownership_guard.update(self)
        conflicts = [item for item in ownership.values() if item.conflict]
        if not conflicts:
            return
        now = time.time()
        if (now - self._last_owner_conflict_log_ts) < 2.0:
            return
        self._last_owner_conflict_log_ts = now
        conflict_topics = ", ".join(
            f"{item.topic} publishers={item.publisher_count}" for item in conflicts
        )
        self.get_logger().error(
            f"PX4 input publisher ownership conflict detected: {conflict_topics}"
        )
        self._publish_runtime_event(
            {
                "event": "px4_input_publisher_conflict",
                "severity": "hard",
                "topics": [item.to_payload() for item in conflicts],
            }
        )

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
        self._vsp_z = self._target_z_setpoint()
        yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.1 else self._drone_yaw
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, yaw)
        if self._pos_valid:
            self._actual_path.append((self._drone_x, self._drone_y, self._drone_z))

    def _capture_home_if_needed(self) -> None:
        if self._home_captured or not self._pos_valid:
            return
        self._home_x = self._drone_x
        self._home_y = self._drone_y
        world_x, world_y = self._local_to_world_xy(self._home_x, self._home_y)
        self._home_captured = True
        self._run_log.log(
            "home_captured",
            home_ned=[round(float(world_x), 3), round(float(world_y), 3)],
            home_local_ned=[round(float(self._home_x), 3), round(float(self._home_y), 3)],
        )

    def _publish_offboard_heartbeat(self) -> None:
        if not self._runtime_readiness.setpoint_publish_allowed:
            return
        self._px4_publishers.publish_offboard_heartbeat(
            timestamp_us=self._px4_timestamp_us(),
            velocity=self._phase == RuntimePhase.MANUAL_VELOCITY,
        )

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = float("nan")) -> None:
        self._refresh_runtime_readiness()
        if not self._runtime_readiness.setpoint_publish_allowed:
            reason = self._runtime_readiness.reason
            if reason != self._last_setpoint_gate_reason:
                self._last_setpoint_gate_reason = reason
                self._run_log.log(
                    "setpoint_publish_blocked",
                    reason=reason,
                    readiness=self._runtime_readiness.to_payload(),
                    phase=self._phase.name,
                    target_id=self._active_target_id,
                )
            return
        self._last_setpoint_gate_reason = ""
        self._px4_publishers.publish_setpoint(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            current_yaw=self._drone_yaw,
            timestamp_us=self._px4_timestamp_us(),
        )

    def _publish_velocity_setpoint(
        self,
        vx: float,
        vy: float,
        vz: float,
        yawspeed: float,
    ) -> None:
        self._refresh_runtime_readiness()
        if not self._runtime_readiness.setpoint_publish_allowed:
            reason = self._runtime_readiness.reason
            if reason != self._last_setpoint_gate_reason:
                self._last_setpoint_gate_reason = reason
                self._run_log.log(
                    "velocity_setpoint_publish_blocked",
                    reason=reason,
                    readiness=self._runtime_readiness.to_payload(),
                    phase=self._phase.name,
                    target_id=self._active_target_id,
                )
            return
        self._last_setpoint_gate_reason = ""
        self._px4_publishers.publish_velocity_setpoint(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw=float("nan"),
            current_yaw=self._drone_yaw,
            yawspeed=yawspeed,
            timestamp_us=self._px4_timestamp_us(),
        )

    def _send_command(
        self,
        cmd: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
    ) -> None:
        self._px4_publishers.send_command(
            command=cmd,
            param1=param1,
            param2=param2,
            param3=param3,
            timestamp_us=self._px4_timestamp_us(),
            target_system=self._drone_id + 1,
        )
        self._run_log.log(
            "vehicle_command",
            command=int(cmd),
            param1=float(param1),
            param2=float(param2),
            param3=float(param3),
        )

    def _px4_timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _ensure_takeoff_activation(self) -> None:
        if self._active_target_xy is None or self._ticks < ARM_TICKS:
            return
        now = time.time()
        if not self._px4_offboard_enabled and (now - self._last_offboard_request_ts) >= COMMAND_RETRY_INTERVAL_S:
            self._set_offboard_mode()
            self._last_offboard_request_ts = now
        if not self._px4_armed and (now - self._last_arm_request_ts) >= COMMAND_RETRY_INTERVAL_S:
            self._arm()
            self._last_arm_request_ts = now

    def _arm(self) -> None:
        param2 = 21196.0 if self._force_arm else 0.0
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=param2,
        )

    def _set_offboard_mode(self) -> None:
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _pub_status_cb(self) -> None:
        mapper_summary = self._local_mapper.summary()
        self._refresh_runtime_readiness()
        blocked_severity = str(self._blocked_severity or "none").strip().upper()
        if blocked_severity not in {"NONE", "SOFT", "HARD"}:
            blocked_severity = "NONE"
        blocked_active = (self._phase == RuntimePhase.BLOCKED) or (blocked_severity != "NONE")
        runtime_result = "ACTIVE"
        if blocked_active:
            runtime_result = "BLOCKED"
        elif self._active_command == "none":
            runtime_result = "IDLE"
        now = time.time()
        command_active = self._active_target_xy is not None and self._active_command != "none"
        target_reached_recent = (
            bool(self._last_completed_target_id)
            and not command_active
            and (now - self._last_completed_target_ts) <= 3.0
        )
        mission_feedback_state = "ACTIVE" if command_active else "IDLE"
        if blocked_active:
            mission_feedback_state = "BLOCKED"
        elif target_reached_recent:
            mission_feedback_state = "TARGET_REACHED"
        scan_state_name = self._scan_manager.state.name
        payload = {
            "phase": self._phase.name,
            "state": self._phase.name,
            "result": runtime_result,
            "flight_control_owner": "obstacle_avoidance_runtime",
            "execution_owner": "obstacle_avoidance_runtime",
            "mission_feedback_state": mission_feedback_state,
            "command_active": command_active,
            "target_active": command_active,
            "command": self._active_command,
            "target_id": self._active_target_id,
            "target_name": self._active_target_name,
            "mission_name": self._active_target_name,
            "target_ned": None if self._active_target_world_xy is None else [
                round(float(self._active_target_world_xy[0]), 2),
                round(float(self._active_target_world_xy[1]), 2),
            ],
            "target_local_ned": None if self._active_target_xy is None else [
                round(float(self._active_target_xy[0]), 2),
                round(float(self._active_target_xy[1]), 2),
            ],
            "subgoal_ned": None if not self._last_plan_result or not self._last_plan_result.subgoal_xy else [
                round(float(self._local_to_world_xy(*self._last_plan_result.subgoal_xy)[0]), 2),
                round(float(self._local_to_world_xy(*self._last_plan_result.subgoal_xy)[1]), 2),
            ],
            "subgoal_local_ned": None if not self._last_plan_result or not self._last_plan_result.subgoal_xy else [
                round(float(self._last_plan_result.subgoal_xy[0]), 2),
                round(float(self._last_plan_result.subgoal_xy[1]), 2),
            ],
            "home_ned": None if not self._home_captured else [
                round(float(self._local_to_world_xy(self._home_x, self._home_y)[0]), 2),
                round(float(self._local_to_world_xy(self._home_x, self._home_y)[1]), 2),
            ],
            "home_local_ned": None if not self._home_captured else [
                round(float(self._home_x), 2),
                round(float(self._home_y), 2),
            ],
            "local_origin_ned": [
                round(float(self._local_origin_x), 2),
                round(float(self._local_origin_y), 2),
            ],
            "home_captured": bool(self._home_captured),
            "navigator_ready": bool(self._runtime_readiness.navigation_allowed),
            "runtime_ready": bool(self._runtime_readiness.ready),
            "px4_state": {
                "armed": bool(self._px4_armed),
                "offboard_enabled": bool(self._px4_offboard_enabled),
                "nav_state": int(self._px4_nav_state),
                "failsafe": bool(self._px4_failsafe),
                "last_vehicle_command_ack": self._last_vehicle_command_ack,
            },
            "readiness": self._runtime_readiness.to_payload(),
            "health": self._runtime_readiness.to_payload(),
            "px4_input_ownership": self._px4_ownership_guard.to_payload(),
            "altitude_policy": {
                "mode": self._altitude_controller.mode,
                "target_z_ned": round(float(self._target_z_setpoint()), 3),
                "terrain_valid": bool(
                    self._altitude_controller.setpoint_for(
                        altitude_m=self._active_target_alt,
                    ).terrain_valid
                ),
                "terrain_range_m": None
                if self._last_terrain_range_m is None
                else round(float(self._last_terrain_range_m), 3),
                "terrain_range_age_s": None
                if self._last_terrain_range_ts <= 0.0
                else round(float(now - self._last_terrain_range_ts), 3),
                "terrain_range_topic": self._terrain_range_topic,
            },
            "sensor_topics": {
                "camera": self._camera_topic,
                "depth": self._depth_topic,
                "camera_info": self._camera_info_topic,
                "lidar_obstacle": self._lidar_obstacle_topic
                if self._enable_lidar_obstacle_points
                else "",
            },
            "lidar_obstacle_enabled": bool(self._enable_lidar_obstacle_points),
            "lidar_obstacle_age_s": None
            if self._last_lidar_obstacle_ts <= 0.0
            else round(float(now - self._last_lidar_obstacle_ts), 3),
            "lidar_obstacle_points": int(self._last_lidar_obstacle_points),
            "camera_info_age_s": None
            if self._last_camera_info_ts <= 0.0
            else round(float(now - self._last_camera_info_ts), 3),
            "rgb_encoding": self._last_rgb_encoding,
            "target_reached": bool(target_reached_recent),
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
                round(float(self._local_to_world_xy(self._drone_x, self._drone_y)[0]), 2),
                round(float(self._local_to_world_xy(self._drone_x, self._drone_y)[1]), 2),
                round(float(self._drone_z), 2),
            ],
            "drone_local_ned": [
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
            "elapsed_s": round(now - self._start_time, 1),
        }
        self._ros_io.publish_avoidance_status(
            status_publisher=self._pub_status,
            status_json_publisher=self._pub_status_json,
            payload=payload,
            drone_id=f"drone_{self._drone_id}",
            legacy_publisher=self._pub_status_legacy,
        )

        self._ros_io.publish_bool(
            publisher=self._pub_avoid,
            value=self._avoidance_active,
            legacy_publisher=self._pub_avoid_legacy,
        )

        if (now - self._last_status_log_ts) >= 1.0:
            self._last_status_log_ts = now
            self._run_log.log(
                "runtime_status",
                **payload,
                obstacle_sectors={k: round(float(v), 3) for k, v in self._obstacle_sectors.items()},
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
