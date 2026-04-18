"""
obstacle_avoidance_mission.py — Autonomní test obstacle avoidance se 4 překážkami.

Dron vzlétne, postupně přeletí 4 trajektorie vedoucí přímo na překážky,
reaktivní APF (Artificial Potential Field) je odkloní. Po každé překážce
se dron vrátí na home, nakonec přistane.

APF logika:
  - Přitažlivá síla: konstantní rychlost cruise_speed směrem k cíli
  - Odpudivá síla: od každé překážky, lineárně klesá od obs_gain (m/s) na povrchu
    na 0 na vzdálenosti obs_influence_r od povrchu
  - Deadlock bypass: při čelní kolizi (repulze ≈ -atrakce) přidá tangenciální složku

Topiky:
  Pub:
    /obstacle_avoidance/status          String JSON   1 Hz
    /obstacle_avoidance/planned_path    nav_msgs/Path 5 Hz  (přímá čára home→cíl)
    /obstacle_avoidance/actual_path     nav_msgs/Path 5 Hz  (skutečná trajektorie)
    /obstacle_avoidance/avoidance_active std_msgs/Bool 10 Hz
  Sub:
    /fmu/out/vehicle_local_position_v1  VehicleLocalPosition
  Pub (PX4):
    /fmu/in/offboard_control_mode
    /fmu/in/trajectory_setpoint
    /fmu/in/vehicle_command

Parametry:
  altitude_m       float 5.0    výška letu nad zemí (m)
  cruise_speed     float 2.5    horizontální rychlost (m/s)
  obs_gain         float 6.0    max odpudivá rychlost APF (m/s)
  obs_influence_r  float 5.5    dosah APF za fyzickým povrchem překážky (m)
  clear_dist       float 2.5    vzdálenost od cíle = mise splněna (m)
  home_dist        float 1.5    vzdálenost od home = RTH splněno (m)

Spuštění:
  ros2 run scout_control obstacle_avoidance_mission
  ros2 run scout_control obstacle_avoidance_mission --ros-args -p altitude_m:=5.0
"""

import json
import math
import time
from collections import deque
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header, String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

# ── QoS ───────────────────────────────────────────────────────────────────────
QOS_PX4_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=10,
)
QOS_PX4_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)
QOS_VIZ = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5,
)
QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)

# ── Letové konstanty ──────────────────────────────────────────────────────────
DT          = 0.1    # s — 10 Hz řídicí smyčka
ARM_TICKS   = 10     # tiků před armingem
ALT_TOL     = 0.5    # m — tolerence výšky při takeoff
MAX_PATH_LEN = 500   # max. délka záznamu skutečné trajektorie

# ── Překážky — pozice v NED (musí odpovídat obstacle_course.world) ────────────
# Gz ENU(x, y) → NED(ned_x=y, ned_y=x)
OBSTACLES: list[dict] = [
    # wall_north: Gz ENU(0, 12) → NED(12, 0), 6m šíře (ve směru Y / NED-y)
    {"id": "wall_north",   "ned_x": 12.0, "ned_y":  0.0, "radius": 3.5, "bypass": +1},
    # poles_east: Gz ENU(12, 0) → NED(0, 12), cluster 3 tyčí (v NED-x ±1.5 m)
    {"id": "poles_east",   "ned_x":  0.0, "ned_y": 12.0, "radius": 3.0, "bypass": +1},
    # building_ne: Gz ENU(9, 9) → NED(9, 9), 4×4×9 m
    {"id": "building_ne",  "ned_x":  9.0, "ned_y":  9.0, "radius": 3.5, "bypass": +1},
    # fence_nnw: Gz ENU(-8, 12) → NED(12, -8), 7m fence (v NED-y/ENU-x směru)
    {"id": "fence_nnw",    "ned_x": 12.0, "ned_y": -8.0, "radius": 4.0, "bypass": -1},
]

# ── Testovací mise — 4 průlety (cíl JE ZA překážkou) ─────────────────────────
MISSIONS: list[dict] = [
    {"name": "North Wall",  "target": (22.0,   0.0), "obs_idx": 0},
    {"name": "East Poles",  "target": ( 0.0,  22.0), "obs_idx": 1},
    {"name": "NE Building", "target": (18.0,  18.0), "obs_idx": 2},
    {"name": "NNW Fence",   "target": (22.0, -12.0), "obs_idx": 3},
]

HOME_NED = (0.0, 0.0)    # NED home position


class Phase(Enum):
    IDLE        = auto()
    TAKEOFF     = auto()
    APPROACH    = auto()   # letí k cíli, APF aktivní
    HOVER_CLEAR = auto()   # krátká pauza po dosažení cíle
    RTH_HOME    = auto()   # návrat na home po každé misi
    FINAL_LAND  = auto()   # všechny 4 mise hotovy → přistání


class ObstacleAvoidanceMission(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_mission")

        # ── Parametry ─────────────────────────────────────────────────────────
        self.declare_parameter("altitude_m",      5.0)
        self.declare_parameter("cruise_speed",    2.5)
        self.declare_parameter("obs_gain",        6.0)
        self.declare_parameter("obs_influence_r", 5.5)
        self.declare_parameter("clear_dist",      2.5)
        self.declare_parameter("home_dist",       1.5)

        self._alt       = float(self.get_parameter("altitude_m").value)
        self._cruise    = float(self.get_parameter("cruise_speed").value)
        self._obs_gain  = float(self.get_parameter("obs_gain").value)
        self._obs_inf   = float(self.get_parameter("obs_influence_r").value)
        self._clear_d   = float(self.get_parameter("clear_dist").value)
        self._home_d    = float(self.get_parameter("home_dist").value)

        # ── Stav ──────────────────────────────────────────────────────────────
        self._phase: Phase   = Phase.IDLE
        self._ticks: int     = 0
        self._hover_ticks: int = 0
        self._mission_idx: int = 0

        self._drone_x: float = 0.0
        self._drone_y: float = 0.0
        self._drone_z: float = 0.0
        self._drone_yaw: float = 0.0
        self._pos_valid: bool = False

        self._vsp_x: float = 0.0
        self._vsp_y: float = 0.0
        self._vsp_z: float = 0.0

        self._avoidance_active: bool = False
        self._actual_path: deque = deque(maxlen=MAX_PATH_LEN)
        self._start_time: float = time.time()

        # ── Publishers PX4 ────────────────────────────────────────────────────
        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PX4_PUB)
        self._pub_sp  = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PX4_PUB)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PX4_PUB)

        # ── Publishers vizualizace ─────────────────────────────────────────────
        self._pub_status  = self.create_publisher(String,  "/obstacle_avoidance/status",           QOS_STATUS)
        self._pub_plan    = self.create_publisher(Path,    "/obstacle_avoidance/planned_path",      QOS_VIZ)
        self._pub_actual  = self.create_publisher(Path,    "/obstacle_avoidance/actual_path",       QOS_VIZ)
        self._pub_avoid   = self.create_publisher(Bool,    "/obstacle_avoidance/avoidance_active",  QOS_VIZ)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, QOS_PX4_SUB,
        )

        # ── Timery ────────────────────────────────────────────────────────────
        self.create_timer(DT,       self._control_loop)
        self.create_timer(1.0,      self._pub_status_cb)
        self.create_timer(0.2,      self._pub_viz_cb)   # 5 Hz

        self.get_logger().info(
            f"obstacle_avoidance_mission ready — "
            f"alt={self._alt}m speed={self._cruise}m/s "
            f"4 missions: {[m['name'] for m in MISSIONS]}"
        )

    # ── PX4 subscriber ────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x   = msg.x
        self._drone_y   = msg.y
        self._drone_z   = msg.z
        self._drone_yaw = msg.heading
        self._pos_valid = True

    # ── Řídicí smyčka ─────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._ticks += 1
        self._publish_offboard_heartbeat()

        if self._phase == Phase.IDLE:
            self._vsp_x = self._drone_x
            self._vsp_y = self._drone_y
            self._vsp_z = 0.0
            if self._ticks >= ARM_TICKS:
                self._arm()
                self._set_offboard_mode()
                self._vsp_z = -self._alt
                self._phase = Phase.TAKEOFF
                self.get_logger().info("TAKEOFF")

        elif self._phase == Phase.TAKEOFF:
            self._vsp_z = -self._alt
            self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
            if self._pos_valid and abs(self._drone_z + self._alt) < ALT_TOL:
                self.get_logger().info(
                    f"Altitude reached — starting Mission {self._mission_idx + 1}: "
                    f"{MISSIONS[self._mission_idx]['name']}"
                )
                self._phase = Phase.APPROACH
                self._actual_path.clear()

        elif self._phase == Phase.APPROACH:
            self._do_approach()

        elif self._phase == Phase.HOVER_CLEAR:
            self._hover_ticks += 1
            self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)
            if self._hover_ticks >= 15:   # 1.5 s hover
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

        elif self._phase == Phase.RTH_HOME:
            self._do_rth_home()

        elif self._phase == Phase.FINAL_LAND:
            self._do_land()

    def _do_approach(self) -> None:
        mission  = MISSIONS[self._mission_idx]
        tx, ty   = mission["target"]
        dx       = tx - self._drone_x
        dy       = ty - self._drone_y
        d_target = math.hypot(dx, dy)

        if d_target < self._clear_d:
            self.get_logger().info(
                f"Mission {self._mission_idx + 1} CLEARED — "
                f"reached target NED({tx:.1f}, {ty:.1f})"
            )
            self._phase = Phase.HOVER_CLEAR
            self._avoidance_active = False
            return

        vx, vy, avoided = self._apf_velocity(
            self._drone_x, self._drone_y, tx, ty,
            OBSTACLES[mission["obs_idx"]]["bypass"],
        )
        self._avoidance_active = avoided

        self._vsp_x += vx * DT
        self._vsp_y += vy * DT
        self._vsp_z  = -self._alt

        # Yaw toward movement direction
        yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.1 else self._drone_yaw

        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, yaw)

        # Record path
        if self._pos_valid:
            self._actual_path.append((self._drone_x, self._drone_y, self._drone_z))

    def _do_rth_home(self) -> None:
        hx, hy = HOME_NED
        dx = hx - self._drone_x
        dy = hy - self._drone_y
        d  = math.hypot(dx, dy)

        if d < self._home_d:
            self.get_logger().info(
                f"Home reached — starting Mission {self._mission_idx + 1}: "
                f"{MISSIONS[self._mission_idx]['name']}"
            )
            self._phase = Phase.APPROACH
            self._actual_path.clear()
            return

        speed = min(self._cruise, d)
        vx = (dx / d) * speed
        vy = (dy / d) * speed
        self._vsp_x += vx * DT
        self._vsp_y += vy * DT
        self._vsp_z  = -self._alt
        yaw = math.atan2(vy, vx)
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z, yaw)

    def _do_land(self) -> None:
        # Hover at home, then AUTO.LAND
        self._vsp_x = HOME_NED[0]
        self._vsp_y = HOME_NED[1]
        self._vsp_z = -self._alt
        self._publish_setpoint(self._vsp_x, self._vsp_y, self._vsp_z)

        # Wait until above home, then land
        dx = HOME_NED[0] - self._drone_x
        dy = HOME_NED[1] - self._drone_y
        if math.hypot(dx, dy) < self._home_d:
            self._send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=4.0,
                param3=6.0,
            )

    # ── APF velocity ──────────────────────────────────────────────────────────

    def _apf_velocity(
        self,
        drone_x: float, drone_y: float,
        target_x: float, target_y: float,
        bypass_sign: int,
    ) -> tuple[float, float, bool]:
        """Artificial Potential Field velocity (m/s in NED x, y)."""
        dx = target_x - drone_x
        dy = target_y - drone_y
        d_t = math.hypot(dx, dy)

        if d_t < 0.1:
            return 0.0, 0.0, False

        # Atraktivní složka (cruise speed k cíli)
        att_x = (dx / d_t) * self._cruise
        att_y = (dy / d_t) * self._cruise

        rep_x_total = 0.0
        rep_y_total = 0.0
        avoidance   = False

        for obs in OBSTACLES:
            ox, oy = obs["ned_x"], obs["ned_y"]
            d_obs  = math.hypot(drone_x - ox, drone_y - oy)
            d_inf  = obs["radius"] + self._obs_inf

            if d_obs >= d_inf:
                continue

            avoidance = True
            # Lineární pokles: plná síla na povrchu, nula na d_inf
            t   = max(0.0, (d_inf - d_obs) / (d_inf - obs["radius"]))
            rep = self._obs_gain * t

            # Smer odpudive sily — od stredu prekazky
            if d_obs > 0.05:
                rx = (drone_x - ox) / d_obs
                ry = (drone_y - oy) / d_obs
            else:
                rx, ry = 1.0, 0.0

            # Deadlock bypass: pokud odpudivá síla míří skoro přímo proti atrakci
            dot = rx * (dx / d_t) + ry * (dy / d_t)
            if dot < 0.2:
                # Tangenciální složka kolmá na odpudivý vektor
                t1x, t1y = -ry,  rx   # CCW
                t2x, t2y =  ry, -rx   # CW

                # Výběr strany: bypass_sign z definice mise, jemný tiebreaker
                if bypass_sign >= 0:
                    tang_x, tang_y = t1x, t1y
                else:
                    tang_x, tang_y = t2x, t2y

                # Blend: 30 % repulze + 70 % tangent (plynné obcházení)
                blend_x = 0.30 * rx + 0.70 * tang_x
                blend_y = 0.30 * ry + 0.70 * tang_y
                norm     = math.hypot(blend_x, blend_y)
                if norm > 0:
                    rx, ry = blend_x / norm, blend_y / norm

            rep_x_total += rep * rx
            rep_y_total += rep * ry

        vx = att_x + rep_x_total
        vy = att_y + rep_y_total

        # Omezení na 2× cruise speed
        speed = math.hypot(vx, vy)
        limit = self._cruise * 2.0
        if speed > limit:
            vx = vx * limit / speed
            vy = vy * limit / speed

        return vx, vy, avoidance

    # ── PX4 nízkoúrovňové příkazy ─────────────────────────────────────────────

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position  = True
        msg.velocity  = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_ocm.publish(msg)

    def _publish_setpoint(
        self, x: float, y: float, z: float, yaw: float = float("nan")
    ) -> None:
        msg = TrajectorySetpoint()
        msg.position  = [x, y, z]
        msg.velocity  = [float("nan")] * 3
        msg.yaw       = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_sp.publish(msg)

    def _send_command(
        self, cmd: int,
        param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.command          = cmd
        msg.param1           = param1
        msg.param2           = param2
        msg.param3           = param3
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(msg)

    def _arm(self) -> None:
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _set_offboard_mode(self) -> None:
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    # ── Vizualizace + status ──────────────────────────────────────────────────

    def _pub_status_cb(self) -> None:
        mission_name = (
            MISSIONS[self._mission_idx]["name"]
            if self._mission_idx < len(MISSIONS) else "DONE"
        )
        payload = {
            "phase":            self._phase.name,
            "mission_idx":      self._mission_idx,
            "mission_name":     mission_name,
            "avoidance_active": self._avoidance_active,
            "drone_ned":        [round(self._drone_x, 2),
                                 round(self._drone_y, 2),
                                 round(self._drone_z, 2)],
            "elapsed_s":        round(time.time() - self._start_time, 1),
        }
        self._pub_status.publish(String(data=json.dumps(payload)))
        self._pub_avoid.publish(Bool(data=self._avoidance_active))

    def _pub_viz_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()

        # Plánovaná cesta (přímá čára home → aktuální cíl)
        plan_msg = Path()
        plan_msg.header.frame_id = "map"
        plan_msg.header.stamp    = stamp
        if self._mission_idx < len(MISSIONS):
            tx, ty = MISSIONS[self._mission_idx]["target"]
            for px, py in [HOME_NED, (tx, ty)]:
                ps = PoseStamped()
                ps.header = plan_msg.header
                ps.pose.position.x = float(py)   # ENU x = NED y (East)
                ps.pose.position.y = float(px)   # ENU y = NED x (North)
                ps.pose.position.z = self._alt
                plan_msg.poses.append(ps)
        self._pub_plan.publish(plan_msg)

        # Skutečná trajektorie
        actual_msg = Path()
        actual_msg.header.frame_id = "map"
        actual_msg.header.stamp    = stamp
        for px, py, pz in self._actual_path:
            ps = PoseStamped()
            ps.header = actual_msg.header
            ps.pose.position.x = float(py)    # ENU x = NED y
            ps.pose.position.y = float(px)    # ENU y = NED x
            ps.pose.position.z = float(-pz)   # ENU z = -NED z
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
