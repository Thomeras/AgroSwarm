"""
manual_controller.py - Swarm Center manual intent bridge.

This is the production manual-control node. It does not publish PX4 offboard
setpoints or vehicle commands. Swarm Center remains the UI, obstacle_avoidance
runtime remains the single flight owner, and this node only translates operator
intent into setup topics or runtime target commands.
"""

import json
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub


QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_RELIABLE_VOL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PAD_KEY_PREFIX = "pad_"
CORNER_LABELS = {"NE", "NW", "SE", "SW"}


class DronePosition:
    def __init__(self, drone_id: int) -> None:
        topics = TelemetryHub(drone_id=drone_id).topics
        self.drone_id = drone_id
        self.drone_ns = topics.drone_ns
        self.position_topic = topics.vehicle_local_position
        self.rth_target_topic = topics.rth_target
        self.target_cmd_topic = topics.avoidance_target_cmd_json
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.xy_valid = False
        self.pos_valid = False


class ManualController(Node):
    """Headless production bridge for Swarm Center manual/setup commands."""

    def __init__(self) -> None:
        super().__init__("manual_controller")

        self.declare_parameter("drone_count", 2)
        self.declare_parameter("reject_origin_pad", False)
        self.declare_parameter("default_altitude_m", 5.0)
        self.declare_parameter("manual_cruise_speed_mps", 2.0)
        self.declare_parameter("manual_clear_radius_m", 0.15)
        self.declare_parameter("local_origins_ned_json", "")

        self._drone_count = max(1, int(self.get_parameter("drone_count").value))
        self._reject_origin_pad = bool(self.get_parameter("reject_origin_pad").value)
        self._default_altitude = float(self.get_parameter("default_altitude_m").value)
        self._manual_speed = float(self.get_parameter("manual_cruise_speed_mps").value)
        self._manual_clear_radius = float(self.get_parameter("manual_clear_radius_m").value)
        self._local_origins = self._parse_local_origins(
            str(self.get_parameter("local_origins_ned_json").value or "")
        )

        self._swarm_topics = TelemetryHub(drone_id=0).swarm
        self._drones = [DronePosition(i) for i in range(self._drone_count)]

        self._pad_assign_pub = self.create_publisher(
            String, self._swarm_topics.pad_assignment, QOS_VOL
        )
        self._corner_pub = self.create_publisher(String, "/field/corner_marked", QOS_VOL)
        self._boundary_point_pub = self.create_publisher(
            String, "/field/boundary_point", QOS_VOL
        )
        self._boundary_close_pub = self.create_publisher(
            String, "/field/boundary_close", QOS_VOL
        )
        self._mission_confirm_pub = self.create_publisher(
            String, "/field/mission_confirm", QOS_VOL
        )
        self._generate_grid_pub = self.create_publisher(
            String, "/field/generate_grid", QOS_RELIABLE_VOL
        )
        self._rth_pubs = {
            d.drone_ns: self.create_publisher(Point, d.rth_target_topic, QOS_LATCHED)
            for d in self._drones
        }
        self._target_cmd_pubs = {
            d.drone_ns: self.create_publisher(
                String, d.target_cmd_topic, QOS_RELIABLE_VOL
            )
            for d in self._drones
        }

        self._pos_subs = []
        for idx, drone in enumerate(self._drones):
            self._pos_subs.append(self.create_subscription(
                VehicleLocalPosition,
                drone.position_topic,
                self._make_pos_cb(idx),
                QOS_SUB,
            ))

        self._manual_control_sub = self.create_subscription(
            String,
            self._swarm_topics.manual_control,
            self._manual_control_cb,
            QOS_VOL,
        )

        self.get_logger().info(
            "ManualController ready | headless Swarm Center bridge | "
            "no PX4 /fmu/in publishers | drone_count=%d | origins=%d"
            % (self._drone_count, len(self._local_origins))
        )

    def _parse_local_origins(self, raw: str) -> dict[str, tuple[float, float]]:
        if not raw.strip():
            return {}
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn("local_origins_ned_json is invalid; assuming zero origins")
            return {}
        origins: dict[str, tuple[float, float]] = {}
        items = data.get("origins", data) if isinstance(data, dict) else data
        if not isinstance(items, list):
            return origins
        for item in items:
            if not isinstance(item, dict):
                continue
            drone_id = str(item.get("drone_id", "")).strip()
            ned = item.get("ned", {})
            if drone_id and isinstance(ned, dict):
                origins[drone_id] = (float(ned.get("x", 0.0)), float(ned.get("y", 0.0)))
        return origins

    def _local_to_world_xy(self, drone: DronePosition, x: float, y: float) -> tuple[float, float]:
        ox, oy = self._local_origins.get(drone.drone_ns, (0.0, 0.0))
        return (float(x) + ox, float(y) + oy)

    def _make_pos_cb(self, idx: int):
        def _cb(msg: VehicleLocalPosition) -> None:
            d = self._drones[idx]
            d.x = float(msg.x)
            d.y = float(msg.y)
            d.z = float(msg.z)
            d.xy_valid = bool(msg.xy_valid)
            if d.xy_valid:
                d.pos_valid = True

        return _cb

    def _manual_control_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("manual_control: invalid JSON payload")
            return

        action = str(data.get("action", "")).strip().lower()
        drone = self._drone_from_payload(data)

        if action == "assign_pad":
            self._assign_pad(data, str(data.get("pad_id", "")).strip())
        elif action == "mark_corner":
            self._mark_corner(str(data.get("corner", "")).strip().upper())
        elif action == "mark_boundary":
            self._mark_boundary_point()
        elif action == "close_boundary":
            self._close_boundary()
        elif action == "clear_boundary":
            self.get_logger().info("clear_boundary requested; reset via field setup restart if needed")
        elif action == "generate_grid":
            self._publish_json(self._generate_grid_pub, {"source": "manual_controller"})
        elif action == "start_mission":
            self._confirm_mission(str(data.get("source", "swarm_center")))
        elif action == "takeoff":
            self._runtime_command(drone, "takeoff", data)
        elif action == "land":
            self._runtime_command(drone, "land", data)
        elif action == "hold":
            self._runtime_command(drone, "hold", data)
        elif action == "cancel":
            self._runtime_command(drone, "cancel", data)
        elif action == "move":
            self._move_as_runtime_goto(drone, data)
        elif action == "stop":
            self._runtime_command(drone, "hold", data)
        else:
            self.get_logger().warn(f"manual_control: unknown action '{action}'")

    def _drone_from_payload(self, data: dict[str, Any]) -> Optional[DronePosition]:
        raw = str(data.get("drone_id", "drone_0")).strip()
        try:
            idx = int(raw.split("_")[-1])
        except (ValueError, IndexError):
            self.get_logger().warn(f"manual_control: cannot parse drone_id '{raw}'")
            return None
        if 0 <= idx < self._drone_count:
            return self._drones[idx]
        self.get_logger().warn(
            f"manual_control: drone_id '{raw}' out of range (n={self._drone_count})"
        )
        return None

    def _drone_by_name(self, drone_id: str) -> Optional[DronePosition]:
        try:
            idx = int(str(drone_id).split("_")[-1])
        except (ValueError, IndexError):
            return None
        if 0 <= idx < self._drone_count:
            return self._drones[idx]
        return None

    def _assign_pad(self, data: dict[str, Any], pad_id: str) -> None:
        mapper_id = str(
            data.get("mapper_drone_id")
            or data.get("source_drone_id")
            or data.get("drone_id", "drone_0")
        )
        target_drone_id = str(
            data.get("assigned_drone_id")
            or data.get("target_drone_id")
            or data.get("drone_id", mapper_id)
        )
        mapper = self._drone_by_name(mapper_id)
        if mapper is None:
            self.get_logger().warn(f"assign_pad: unknown mapper drone '{mapper_id}'")
            return
        if target_drone_id not in self._rth_pubs:
            self.get_logger().warn(f"assign_pad: unknown target drone '{target_drone_id}'")
            return
        if not pad_id.startswith(PAD_KEY_PREFIX):
            self.get_logger().warn(f"assign_pad: invalid pad_id '{pad_id}'")
            return
        if not mapper.pos_valid or not mapper.xy_valid:
            self.get_logger().warn(
                f"assign_pad: {mapper.drone_ns} EKF not ready; pad not saved"
            )
            return
        if self._reject_origin_pad and abs(mapper.x) < 0.5 and abs(mapper.y) < 0.5:
            self.get_logger().warn(
                f"assign_pad: {mapper.drone_ns} near origin NED({mapper.x:.2f},{mapper.y:.2f})"
            )
            return

        world_x, world_y = self._local_to_world_xy(mapper, mapper.x, mapper.y)
        payload = {
            "drone_id": target_drone_id,
            "pad_id": pad_id,
            "x": round(world_x, 3),
            "y": round(world_y, 3),
            "z": -0.5,
            "mapped_by": mapper.drone_ns,
            "mapper_local_ned": {
                "x": round(mapper.x, 3),
                "y": round(mapper.y, 3),
                "z": round(mapper.z, 3),
            },
        }
        self._publish_json(self._pad_assign_pub, payload)

        rth = Point()
        rth.x = world_x
        rth.y = world_y
        rth.z = -0.5
        self._rth_pubs[target_drone_id].publish(rth)
        self.get_logger().info(
            f"Pad assigned | {target_drone_id} -> {pad_id} "
            f"world NED({world_x:.2f},{world_y:.2f}) mapped_by={mapper.drone_ns}"
        )

    def _mark_corner(self, label: str) -> None:
        if label not in CORNER_LABELS:
            self.get_logger().warn(f"mark_corner: invalid corner '{label}'")
            return
        drone0 = self._drones[0]
        if not drone0.pos_valid:
            self.get_logger().warn("mark_corner: drone_0 position not ready")
            return
        world_x, world_y = self._local_to_world_xy(drone0, drone0.x, drone0.y)
        self._publish_json(
            self._corner_pub,
            {
                "corner": label,
                "ned": {
                    "x": round(world_x, 3),
                    "y": round(world_y, 3),
                    "z": round(drone0.z, 3),
                },
            },
        )

    def _mark_boundary_point(self) -> None:
        drone0 = self._drones[0]
        if not drone0.pos_valid:
            self.get_logger().warn("mark_boundary: drone_0 position not ready")
            return
        world_x, world_y = self._local_to_world_xy(drone0, drone0.x, drone0.y)
        self._publish_json(
            self._boundary_point_pub,
            {
                "ned": {
                    "x": round(world_x, 3),
                    "y": round(world_y, 3),
                    "z": round(drone0.z, 3),
                },
                "type": "vertex",
                "source": "manual_controller",
            },
        )

    def _close_boundary(self) -> None:
        self._publish_json(
            self._boundary_close_pub,
            {"closed": True, "source": "manual_controller"},
        )

    def _confirm_mission(self, source: str) -> None:
        self._publish_json(
            self._mission_confirm_pub,
            {"source": source or "swarm_center", "confirmed": True},
        )

    def _runtime_command(
        self,
        drone: Optional[DronePosition],
        command: str,
        data: dict[str, Any],
    ) -> None:
        if drone is None:
            return
        payload = {
            "command": command,
            "target_id": self._target_id(command),
            "altitude_m": float(data.get("altitude_m", self._default_altitude)),
            "cruise_speed_mps": float(data.get("cruise_speed_mps", self._manual_speed)),
            "clear_radius_m": float(data.get("clear_radius_m", self._manual_clear_radius)),
        }
        self._publish_json(self._target_cmd_pubs[drone.drone_ns], payload)

    def _move_as_runtime_goto(
        self,
        drone: Optional[DronePosition],
        data: dict[str, Any],
    ) -> None:
        if drone is None or not drone.pos_valid:
            return
        vx = float(data.get("vx", 0.0))
        vy = float(data.get("vy", 0.0))
        lookahead_s = float(data.get("lookahead_s", 0.3))
        target_x, target_y = self._local_to_world_xy(
            drone,
            drone.x + vx * lookahead_s,
            drone.y + vy * lookahead_s,
        )
        payload = {
            "command": "goto",
            "target_id": self._target_id("manual_goto"),
            "target_ned": [round(target_x, 3), round(target_y, 3)],
            "altitude_m": float(data.get("altitude_m", self._default_altitude)),
            "cruise_speed_mps": float(data.get("cruise_speed_mps", self._manual_speed)),
            "clear_radius_m": float(data.get("clear_radius_m", self._manual_clear_radius)),
        }
        self._publish_json(self._target_cmd_pubs[drone.drone_ns], payload)

    def _target_id(self, prefix: str) -> str:
        return f"{prefix}_{int(time.time() * 1000)}"

    def _publish_json(self, pub, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
