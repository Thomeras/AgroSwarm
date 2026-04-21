"""
obstacle_avoidance_mission.py — Autonomní test obstacle avoidance s depth kamerou.

Dron letí po 4 testovacích trajektoriích na cíle schválně umístěné za překážkami.
Trajektorie cílů jsou pevně dané, ale samotné vyhýbání je reaktivní a bere data z:
  /drone_N/obstacles/detected  (output obstacle_detector.py)

Logika:
  - normálně letí přímo na aktuální target
  - při warn obstacle zpomalí
  - při critical obstacle vytvoří laterální detour waypoint podle volných sektorů
    z depth kamery (preferuje pravou stranu, fallback levá)
  - po projetí detour waypointu pokračuje na původní target

Spuštění:
  ros2 run scout_control obstacle_detector
  ros2 run scout_control obstacle_avoidance_mission
"""

import json
import math
import time
from collections import deque
from enum import Enum, auto

import rclpy
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
from std_msgs.msg import Bool, String

# ── QoS ───────────────────────────────────────────────────────────────────────
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

# ── Letové konstanty ──────────────────────────────────────────────────────────
DT = 0.1
ARM_TICKS = 50
ALT_TOL = 0.5
WARN_SLOWDOWN = 0.5
DETOUR_REACHED_DIST = 1.0
MAX_PATH_LEN = 500

# ── Testovací mise — cíle jsou za překážkami, aby se avoidance opravdu použil ─
MISSIONS: list[dict] = [
    {"name": "North Wall", "target": (22.0, 0.0)},
    {"name": "East Poles", "target": (0.0, 22.0)},
    {"name": "NE Building", "target": (18.0, 18.0)},
    {"name": "NNW Fence", "target": (22.0, -12.0)},
]

HOME_NED = (0.0, 0.0)


class Phase(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    APPROACH = auto()
    AVOIDING = auto()
    HOVER_CLEAR = auto()
    RTH_HOME = auto()
    FINAL_LAND = auto()


class ObstacleAvoidanceMission(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_mission")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("altitude_m", 5.0)
        self.declare_parameter("cruise_speed", 2.5)
        self.declare_parameter("avoid_offset_m", 3.0)
        self.declare_parameter("clear_dist", 2.5)
        self.declare_parameter("home_dist", 1.5)

        self._drone_id = int(self.get_parameter("drone_id").value)
        self._alt = float(self.get_parameter("altitude_m").value)
        self._cruise = float(self.get_parameter("cruise_speed").value)
        self._avoid_offset = float(self.get_parameter("avoid_offset_m").value)
        self._clear_d = float(self.get_parameter("clear_dist").value)
        self._home_d = float(self.get_parameter("home_dist").value)

        self._phase = Phase.IDLE
        self._ticks = 0
        self._hover_ticks = 0
        self._mission_idx = 0

        self._drone_x = 0.0
        self._drone_y = 0.0
        self._drone_z = 0.0
        self._drone_yaw = 0.0
        self._pos_valid = False

        self._vsp_x = 0.0
        self._vsp_y = 0.0
        self._vsp_z = 0.0

        self._avoidance_active = False
        self._obstacle_warn = False
        self._obstacle_critical = False
        self._obstacle_closest = 99.0
        self._obstacle_sectors: dict[str, float] = {
            "left": 99.0, "center": 99.0, "right": 99.0
        }
        self._free_directions: list[str] = ["left", "center", "right"]
        self._detour_target: tuple[float, float] | None = None

        self._actual_path: deque = deque(maxlen=MAX_PATH_LEN)
        self._start_time = time.time()

        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PX4_PUB
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PX4_PUB
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PX4_PUB
        )

        self._pub_status = self.create_publisher(
            String, "/obstacle_avoidance/status", QOS_STATUS
        )
        self._pub_plan = self.create_publisher(
            Path, "/obstacle_avoidance/planned_path", QOS_VIZ
        )
        self._pub_actual = self.create_publisher(
            Path, "/obstacle_avoidance/actual_path", QOS_VIZ
        )
        self._pub_avoid = self.create_publisher(
            Bool, "/obstacle_avoidance/avoidance_active", QOS_VIZ
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pos_cb,
            QOS_PX4_SUB,
        )
        self.create_subscription(
            String,
            f"/drone_{self._drone_id}/obstacles/detected",
            self._obstacle_cb,
            QOS_VIZ,
        )

        self.create_timer(DT, self._control_loop)
        self.create_timer(1.0, self._pub_status_cb)
        self.create_timer(0.2, self._pub_viz_cb)

        self.get_logger().info(
            f"obstacle_avoidance_mission ready — drone_id={self._drone_id} "
            f"alt={self._alt}m speed={self._cruise}m/s "
            f"4 missions: {[m['name'] for m in MISSIONS]}"
        )

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x = msg.x
        self._drone_y = msg.y
        self._drone_z = msg.z
        self._drone_yaw = msg.heading
        self._pos_valid = True

    def _obstacle_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._obstacle_closest = float(
            data.get("closest", data.get("closest_m", 99.0))
        )
        self._obstacle_sectors = data.get(
            "sectors", {"left": 99.0, "center": 99.0, "right": 99.0}
        )
        self._free_directions = data.get(
            "free_directions", ["left", "center", "right"]
        )
        self._obstacle_warn = bool(data.get("warn", False))
        self._obstacle_critical = bool(data.get("critical", False))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._ticks += 1
        self._publish_offboard_heartbeat()

        if self._phase == Phase.IDLE:
            self._vsp_x = self._drone_x
            self._vsp_y = self._drone_y
            self._vsp_z = 0.0
            if self._ticks >= ARM_TICKS and self._pos_valid:
                self._arm()
                self._set_offboard_mode()
                self._vsp_z = -self._alt
                self._phase = Phase.TAKEOFF
                self.get_logger().info("TAKEOFF")
            return

        if self._phase == Phase.TAKEOFF:
            self._vsp_z = -self._alt
            self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
            if self._pos_valid and abs(self._drone_z + self._alt) < ALT_TOL:
                self.get_logger().info(
                    f"Altitude reached — starting Mission {self._mission_idx + 1}: "
                    f"{MISSIONS[self._mission_idx]['name']}"
                )
                self._phase = Phase.APPROACH
                self._actual_path.clear()
            return

        if self._phase == Phase.APPROACH:
            self._do_approach()
            return

        if self._phase == Phase.AVOIDING:
            self._do_avoiding()
            return

        if self._phase == Phase.HOVER_CLEAR:
            self._hover_ticks += 1
            self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
            if self._hover_ticks >= 15:
                self._hover_ticks = 0
                self._mission_idx += 1
                if self._mission_idx >= len(MISSIONS):
                    self._phase = Phase.FINAL_LAND
                    self.get_logger().info("All 4 missions complete — FINAL LAND")
                else:
                    self._phase = Phase.RTH_HOME
                    self.get_logger().info(
                        f"RTH home before Mission {self._mission_idx + 1}"
                    )
            return

        if self._phase == Phase.RTH_HOME:
            self._do_rth_home()
            return

        if self._phase == Phase.FINAL_LAND:
            self._do_land()

    def _do_approach(self) -> None:
        mission = MISSIONS[self._mission_idx]
        tx, ty = mission["target"]

        if self._distance_to(tx, ty) < self._clear_d:
            self.get_logger().info(
                f"Mission {self._mission_idx + 1} CLEARED — "
                f"reached target NED({tx:.1f}, {ty:.1f})"
            )
            self._phase = Phase.HOVER_CLEAR
            self._avoidance_active = False
            self._detour_target = None
            return

        if self._obstacle_critical:
            detour = self._compute_detour_waypoint(tx, ty)
            if detour is not None:
                self._detour_target = detour
                self._avoidance_active = True
                self._phase = Phase.AVOIDING
                self.get_logger().info(
                    f"Obstacle critical at {self._obstacle_closest:.2f} m — "
                    f"detour via NED({detour[0]:.1f}, {detour[1]:.1f})"
                )
                self._do_avoiding()
                return
            self._avoidance_active = True
            self._publish_setpoint(self._vsp_x, self._vsp_y, -self._alt, self._drone_yaw)
            return

        speed = self._cruise * WARN_SLOWDOWN if self._obstacle_warn else self._cruise
        self._avoidance_active = self._obstacle_warn
        self._step_toward(tx, ty, speed)

    def _do_avoiding(self) -> None:
        if self._detour_target is None:
            self._phase = Phase.APPROACH
            self._avoidance_active = False
            return

        detour_x, detour_y = self._detour_target
        if self._distance_to(detour_x, detour_y) < DETOUR_REACHED_DIST:
            self._detour_target = None
            self._phase = Phase.APPROACH
            self._avoidance_active = self._obstacle_warn
            self.get_logger().info("Detour waypoint reached — resuming mission target")
            return

        speed = self._cruise * WARN_SLOWDOWN if self._obstacle_critical else self._cruise
        self._avoidance_active = True
        self._step_toward(detour_x, detour_y, speed)

    def _do_rth_home(self) -> None:
        hx, hy = HOME_NED
        if self._distance_to(hx, hy) < self._home_d:
            self.get_logger().info(
                f"Home reached — starting Mission {self._mission_idx + 1}: "
                f"{MISSIONS[self._mission_idx]['name']}"
            )
            self._phase = Phase.APPROACH
            self._actual_path.clear()
            self._detour_target = None
            return

        self._avoidance_active = False
        self._step_toward(hx, hy, self._cruise)

    def _do_land(self) -> None:
        self._vsp_x = HOME_NED[0]
        self._vsp_y = HOME_NED[1]
        self._vsp_z = -self._alt
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)

        if self._distance_to(HOME_NED[0], HOME_NED[1]) < self._home_d:
            self._send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=4.0,
                param3=6.0,
            )

    # ── Avoidance helpers ────────────────────────────────────────────────────

    def _distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self._drone_x, y - self._drone_y)

    def _velocity_toward(self, x: float, y: float, speed: float) -> tuple[float, float]:
        dx = x - self._drone_x
        dy = y - self._drone_y
        d = math.hypot(dx, dy)
        if d < 0.05:
            return 0.0, 0.0
        speed = min(speed, d)
        return (dx / d) * speed, (dy / d) * speed

    def _compute_detour_waypoint(
        self, target_x: float, target_y: float
    ) -> tuple[float, float] | None:
        course = math.atan2(target_y - self._drone_y, target_x - self._drone_x)

        if "right" in self._free_directions:
            perpendicular = course - math.pi / 2.0
        elif "left" in self._free_directions:
            perpendicular = course + math.pi / 2.0
        else:
            return None

        detour_x = self._drone_x + self._avoid_offset * math.cos(perpendicular)
        detour_y = self._drone_y + self._avoid_offset * math.sin(perpendicular)
        return detour_x, detour_y

    def _step_toward(self, target_x: float, target_y: float, speed: float) -> None:
        vx, vy = self._velocity_toward(target_x, target_y, speed)
        self._vsp_x += vx * DT
        self._vsp_y += vy * DT
        self._vsp_z = -self._alt
        yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.1 else self._drone_yaw
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, yaw)

        if self._pos_valid:
            self._actual_path.append((self._drone_x, self._drone_y, self._drone_z))

    # ── PX4 helpers ──────────────────────────────────────────────────────────

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_ocm.publish(msg)

    def _publish_setpoint(
        self, x: float, y: float, z: float, yaw: float = float("nan")
    ) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float("nan")] * 3
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_sp.publish(msg)

    def _send_command(
        self, cmd: int,
        param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
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

    def _arm(self) -> None:
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def _set_offboard_mode(self) -> None:
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    # ── Status + viz ────────────────────────────────────────────────────────

    def _pub_status_cb(self) -> None:
        mission_name = (
            MISSIONS[self._mission_idx]["name"]
            if self._mission_idx < len(MISSIONS) else "DONE"
        )
        payload = {
            "phase": self._phase.name,
            "mission_idx": self._mission_idx,
            "mission_name": mission_name,
            "avoidance_active": self._avoidance_active,
            "obstacle_warn": self._obstacle_warn,
            "obstacle_critical": self._obstacle_critical,
            "obstacle_closest_m": round(self._obstacle_closest, 2),
            "free_directions": self._free_directions,
            "drone_ned": [
                round(self._drone_x, 2),
                round(self._drone_y, 2),
                round(self._drone_z, 2),
            ],
            "elapsed_s": round(time.time() - self._start_time, 1),
        }
        self._pub_status.publish(String(data=json.dumps(payload)))
        self._pub_avoid.publish(Bool(data=self._avoidance_active))

    def _pub_viz_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()

        plan_msg = Path()
        plan_msg.header.frame_id = "map"
        plan_msg.header.stamp = stamp
        if self._mission_idx < len(MISSIONS):
            tx, ty = MISSIONS[self._mission_idx]["target"]
            for px, py in [HOME_NED, (tx, ty)]:
                ps = PoseStamped()
                ps.header = plan_msg.header
                ps.pose.position.x = float(py)
                ps.pose.position.y = float(px)
                ps.pose.position.z = self._alt
                plan_msg.poses.append(ps)
            if self._detour_target is not None:
                ps = PoseStamped()
                ps.header = plan_msg.header
                ps.pose.position.x = float(self._detour_target[1])
                ps.pose.position.y = float(self._detour_target[0])
                ps.pose.position.z = self._alt
                plan_msg.poses.append(ps)
        self._pub_plan.publish(plan_msg)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
