"""
perimeter_flight.py — PX4 offboard perimeter survey

Dron vzlétne do ALTITUDE metrů a obletí obvod FIELD_SIZE×FIELD_SIZE m pole.
Startovní pozice dronu = SW roh pole (lokální NED origin).

Výstup:
  <ws_root>/perimeters/field_perimeter.json  — GPS log každé 2s
  /field/perimeter (Float32MultiArray) — [x0,y0, x1,y1, ...] bodů obvodu v NED

Spuštění:
  ros2 run scout_control perimeter_flight
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
)

# ── QoS (stejné jako offboard_control.py) ──────────────────────────────────
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

# ── Parametry mise ──────────────────────────────────────────────────────────
ALTITUDE     = 5.0    # m nad zemí (NED: z = -ALTITUDE)
FIELD_SIZE   = 25.0   # m — strana pole
CRUISE_SPEED = 2.0    # m/s — pohyb virtuálního setpointu
DT           = 0.1    # s  — perioda timeru (10 Hz)
REACH_DIST   = 1.0    # m  — vzdálenost pro přepnutí na další WP
ALT_TOL      = 0.5    # m  — tolerance výšky před přepnutím WP
LOG_INTERVAL = 2.0    # s  — interval GPS logování

from scout_control.paths import PERIMETER_FILE as OUTPUT_FILE, PERIMETERS_DIR


class PerimeterFlight(Node):
    def __init__(self):
        super().__init__('perimeter_flight')

        # ── Publishers ──────────────────────────────────────────────────────
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', QOS_PUB)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', QOS_PUB)
        self._command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', QOS_PUB)
        self._perimeter_pub = self.create_publisher(
            Float32MultiArray, '/field/perimeter', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, QOS_SUB)
        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._gps_cb, QOS_SUB)

        # ── Stav ────────────────────────────────────────────────────────────
        self._tick = 0
        self._wp   = 0
        self._x = self._y = self._z = 0.0
        self._lat = self._lon = self._alt_gps = 0.0
        self._gps_valid = False
        self._vsp     = [0.0, 0.0, 0.0]   # virtual setpoint
        self._armed   = False
        self._landing = False
        self._done    = False
        self._last_log_time = 0.0
        self._log_data: list = []

        # ── Waypoints obvodu v NED (relativně k místu spawnu = SW roh pole) ─
        #   x = sever, y = východ, z = -výška
        z = -ALTITUDE
        self._waypoints = [
            (0.0,        0.0,        z),   # WP 0 — vzlet (SW roh)
            (FIELD_SIZE, 0.0,        z),   # WP 1 — NW roh (sever)
            (FIELD_SIZE, FIELD_SIZE, z),   # WP 2 — NE roh
            (0.0,        FIELD_SIZE, z),   # WP 3 — SE roh (východ)
            (0.0,        0.0,        z),   # WP 4 — zpět home (SW roh)
        ]

        self.create_timer(DT, self._cb)
        self.get_logger().info(
            f'PerimeterFlight start | pole {FIELD_SIZE:.0f}×{FIELD_SIZE:.0f} m | '
            f'výška {ALTITUDE} m | log → {OUTPUT_FILE}'
        )

    # ────────────────────────────────────────────────────────────────────────
    def _pos_cb(self, msg: VehicleLocalPosition):
        self._x = msg.x
        self._y = msg.y
        self._z = msg.z
        # Inicializuj VSP na aktuální pozici při prvním callbacku
        if self._vsp == [0.0, 0.0, 0.0]:
            self._vsp = [self._x, self._y, self._z]

    def _gps_cb(self, msg: VehicleGlobalPosition):
        if msg.lat_lon_valid:
            self._lat = msg.lat
            self._lon = msg.lon
            self._alt_gps = float(msg.alt)
            self._gps_valid = True

    # ────────────────────────────────────────────────────────────────────────
    def _cb(self):
        if self._done:
            return

        self._publish_offboard_mode()
        self._publish_setpoint(self._waypoints[self._wp])

        # ── Arm + offboard na 10. tiku ───────────────────────────────────
        if self._tick == 10:
            self._send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Armed + offboard engaged')
            self._armed = True

        # ── GPS log každé 2 sekundy (jen za letu) ────────────────────────
        if self._armed and not self._landing:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_log_time >= LOG_INTERVAL:
                self._log_position()
                self._last_log_time = now

        # ── Navigace po waypointech ───────────────────────────────────────
        if self._tick > 10 and not self._landing:
            target = self._waypoints[self._wp]
            at_alt = abs(self._z - target[2]) < ALT_TOL
            dist   = math.dist((self._x, self._y, self._z), target)

            if at_alt and dist < REACH_DIST:
                self.get_logger().info(
                    f'WP {self._wp} reached | dist={dist:.2f} m | '
                    f'pos=({self._x:.1f}, {self._y:.1f}, {self._z:.1f})'
                )
                if self._wp < len(self._waypoints) - 1:
                    self._wp += 1
                    nxt = self._waypoints[self._wp]
                    self.get_logger().info(
                        f'→ WP {self._wp}: ({nxt[0]:.0f}, {nxt[1]:.0f}, {nxt[2]:.1f})')
                else:
                    self._finish()

        self._tick += 1

    # ────────────────────────────────────────────────────────────────────────
    def _log_position(self):
        entry = {
            'ts':    round(self.get_clock().now().nanoseconds / 1e9, 2),
            'wp':    self._wp,
            'local': {
                'x': round(self._x, 3),
                'y': round(self._y, 3),
                'z': round(self._z, 3),
            },
            'gps': {
                'lat':   self._lat if self._gps_valid else None,
                'lon':   self._lon if self._gps_valid else None,
                'alt_m': round(self._alt_gps, 2) if self._gps_valid else None,
                'valid': self._gps_valid,
            },
        }
        self._log_data.append(entry)
        gps_str = (f'lat={self._lat:.6f} lon={self._lon:.6f}'
                   if self._gps_valid else 'GPS N/A')
        self.get_logger().info(
            f'LOG #{len(self._log_data):3d} | wp={self._wp} | '
            f'x={self._x:6.1f} y={self._y:6.1f} z={self._z:5.1f} | {gps_str}'
        )

    # ────────────────────────────────────────────────────────────────────────
    def _finish(self):
        self.get_logger().info('Obvod dokončen — ukládám data a přistávám')
        self._landing = True

        # Uložit JSON log
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        payload = {
            'field_size_m':    FIELD_SIZE,
            'altitude_m':      ALTITUDE,
            'waypoints_ned':   [list(wp) for wp in self._waypoints],
            'log_count':       len(self._log_data),
            'log':             self._log_data,
        }
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f'JSON uložen → {OUTPUT_FILE} ({len(self._log_data)} záznamů)')

        # Publishnout obvod na /field/perimeter
        flat = []
        for wp in self._waypoints:
            flat.extend([float(wp[0]), float(wp[1])])
        msg = Float32MultiArray()
        msg.data = flat
        self._perimeter_pub.publish(msg)
        self.get_logger().info(
            f'Perimeter publishnut /field/perimeter | {len(self._waypoints)} bodů')

        # Přistání
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._done = True

    # ────────────────────────────────────────────────────────────────────────
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position  = True
        msg.velocity  = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_setpoint(self, wp: tuple):
        """Moving virtual setpoint — posun max CRUISE_SPEED×DT m/tick směrem k wp."""
        step = CRUISE_SPEED * DT
        dx = wp[0] - self._vsp[0]
        dy = wp[1] - self._vsp[1]
        dz = wp[2] - self._vsp[2]
        d  = math.sqrt(dx*dx + dy*dy + dz*dz)

        if d > step:
            self._vsp[0] += (dx / d) * step
            self._vsp[1] += (dy / d) * step
            self._vsp[2] += (dz / d) * step
        else:
            self._vsp = [float(wp[0]), float(wp[1]), float(wp[2])]

        # Yaw = směr předkem dronu k cílovému waypointu (NED: atan2(east, north))
        # Pokud jsme velmi blízko cíle (vzlet na místě), držíme aktuální heading (nan)
        tx = wp[0] - self._x
        ty = wp[1] - self._y
        horiz = math.sqrt(tx * tx + ty * ty)
        yaw = math.atan2(ty, tx) if horiz > 0.5 else float('nan')

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [self._vsp[0], self._vsp[1], self._vsp[2]]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = yaw
        msg.timestamp    = self._now_us()
        self._trajectory_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
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
        self._command_pub.publish(msg)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


# ────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PerimeterFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
