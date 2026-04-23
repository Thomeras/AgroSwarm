"""
terrain_follower.py — Terrain-following offboard controller using downward lidar.

Udržuje konstantní výšku nad zemí (desired_height) bez ohledu na nerovnosti terénu.

WORKFLOW:
  1. Drží horizontální polohu na místě (hover nad bodu kde byl aktivován).
  2. Čte skutečnou vzdálenost k zemi z /downward_lidar/scan.
  3. Počítá chybu: error = range - desired_height
       error > 0 → příliš vysoko → klesej (vz > 0 v NED)
       error < 0 → příliš nízko  → stoupej (vz < 0 v NED)
  4. Z chyby počítá velocity setpoint ve svislé ose (P regulátor s nasycením).
  5. Horizontální pohyb je nulový — dron visí na místě kde byl armován.

POZNÁMKA K NÁVRHU:
  Výška je řízena VÝHRADNĚ z lidaru — drone_z (NED z EKF/GPS) se nepoužívá pro řízení.
  To zajišťuje funkčnost i na reálném hardware kde absolutní NED výška není spolehlivá.
  Position setpoint se používá pouze pro XY (zamčení horizontální polohy).
  Svislá osa je řízena velocity setpointem odvozeným z lidarové chyby.

SPUŠTĚNÍ:
  ros2 run scout_control terrain_follower
  ros2 run scout_control terrain_follower --ros-args -p desired_height:=3.0

PŘEDPOKLADY:
  - Před spuštěním: PX4 SITL + MicroXRCE + lidar_bridge.launch.py
  - Model: gz_x500_mono_cam_lidar
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

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
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PX4_PUB = QoSProfile(
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

# ── Konstanty ─────────────────────────────────────────────────────────────────
DT           = 0.1    # s  — perioda control loopu (10 Hz)
ARM_TICKS    = 10     # setpointů před armováním

# Lidar filtry
RANGE_MAX_OK = 80.0   # m — ignoruj out-of-range čtení
RANGE_MIN_OK = 0.15   # m — ignoruj self-hit čtení (sensor range_min=0.1)

# P regulátor pro svislou osu
K_P          = 1.2    # gain [m/s per m error]  — zvyšovat pokud reakce pomalá
VZ_MAX       = 2.0    # m/s — saturace výstupní rychlosti (bezpečnostní limit)
ALT_TOL      = 0.10   # m   — deadband: v tomto pásmu vz=0 (zabraňuje bzučení)


class TerrainFollower(Node):

    def __init__(self) -> None:
        super().__init__("terrain_follower")

        # ── Parametry ─────────────────────────────────────────────────────────
        self.declare_parameter("desired_height", 3.0)
        self._desired_height: float = (
            self.get_parameter("desired_height").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"TerrainFollower | desired_height={self._desired_height:.1f} m"
        )

        # ── Sdílený stav (chráněný _lock) ─────────────────────────────────────
        self._lock = threading.Lock()

        # XY pozice pro horizontal hold (nastaveno při prvním platném msg z PX4)
        self._hold_x:      float = 0.0
        self._hold_y:      float = 0.0
        self._hold_set:    bool  = False

        # Lidar
        self._range:       float = 0.0
        self._range_valid: bool  = False

        # Arm state
        self._armed:         bool = False
        self._arm_requested: bool = False
        self._ticks:         int  = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PX4_PUB)
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PX4_PUB)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PX4_PUB)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, QOS_PX4_SUB)

        self.create_subscription(
            LaserScan,
            "/downward_lidar/scan",
            self._range_cb, QOS_SENSOR)

        # ── 10 Hz control loop ────────────────────────────────────────────────
        self.create_timer(DT, self._timer_cb)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        """Používá se výhradně pro zamčení XY pozice. Výška se nepoužívá."""
        with self._lock:
            if not self._hold_set:
                self._hold_x  = msg.x
                self._hold_y  = msg.y
                self._hold_set = True

    def _range_cb(self, msg: LaserScan) -> None:
        """1-ray lidar: první hodnota = vzdálenost k zemi pod dronem."""
        if not msg.ranges:
            return
        r = msg.ranges[0]
        with self._lock:
            if RANGE_MIN_OK <= r <= RANGE_MAX_OK:
                self._range       = r
                self._range_valid = True
            # out-of-range → držíme poslední platnou hodnotu

    # ── Control loop ─────────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        with self._lock:
            self._ticks += 1

            # Armování po ARM_TICKS setpointech (PX4 vyžaduje stream před armem)
            if self._arm_requested and not self._armed and self._ticks >= ARM_TICKS:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._armed         = True
                self._arm_requested = False
                self.get_logger().info(
                    f"Armed | terrain following @ {self._desired_height:.1f} m")

            # ── P regulátor: lidar error → vz ─────────────────────────────────
            #
            # error = range - desired_height
            #   > 0  dron je příliš vysoko → chceme klesat → vz > 0 (NED: kladné vz = dolů)
            #   < 0  dron je příliš nízko  → chceme stoupat → vz < 0
            #
            # KLÍČOVÝ BOD: drone_z se záměrně nepoužívá.
            # vz je velocity command — PX4 flight controller integrace zajistí pohyb.
            # Nevznikají problémy s driftem EKF, barometrickým šumem ani GPS chybami.
            #
            if self._range_valid:
                error = self._range - self._desired_height
                if abs(error) <= ALT_TOL:
                    vz = 0.0   # v deadbandu — stůj
                else:
                    vz = K_P * error
                    # Saturace: nikdy neleť rychleji než VZ_MAX
                    vz = max(-VZ_MAX, min(VZ_MAX, vz))
            else:
                # Lidar ještě nemá platná data — čekej bez pohybu
                vz = 0.0

            hold_x = self._hold_x
            hold_y = self._hold_y

        self._publish_offboard()
        self._publish_setpoint(hold_x, hold_y, vz)

        # Debug log každou sekundu
        if self._ticks % 10 == 0:
            with self._lock:
                rng   = self._range if self._range_valid else float("nan")
                rv    = self._range_valid
                armed = self._armed
            err = rng - self._desired_height if rv else float("nan")
            self.get_logger().info(
                f"[DBG] armed={armed}  range={rng:.3f}m  valid={rv}  "
                f"error={err:+.3f}m  vz={vz:+.3f}m/s  desired={self._desired_height:.1f}m"
            )

    # ── PX4 publishing ────────────────────────────────────────────────────────

    def _publish_offboard(self) -> None:
        msg = OffboardControlMode()
        msg.position  = True   # XY position hold
        msg.velocity  = True   # Z velocity control
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_setpoint(self, hold_x: float, hold_y: float, vz: float) -> None:
        nan = float("nan")
        msg = TrajectorySetpoint()
        # XY: position hold na místě kde byl dron armován
        # Z position: nan → PX4 ignoruje, řídí vz
        msg.position     = [hold_x, hold_y, nan]
        # XY velocity: nan → PX4 ignoruje (řídí position)
        # Z velocity: svislá korekce z P regulátoru
        msg.velocity     = [nan, nan, vz]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = nan   # drž aktuální heading
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

    # ── Public API ────────────────────────────────────────────────────────────

    def arm(self) -> None:
        """Vyžádá arm (spustí terrain following po ARM_TICKS setpointech)."""
        with self._lock:
            if not self._armed and not self._arm_requested:
                self._arm_requested = True
                self.get_logger().info("Arm requested — armed after 10 setpoints")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainFollower()
    node.arm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()