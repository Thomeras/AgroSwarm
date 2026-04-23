import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition,
)

# PX4 publishuje /fmu/out/* s TRANSIENT_LOCAL – subscriber musi matchovat
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# PX4 ocekava prikazy s BEST_EFFORT + TRANSIENT_LOCAL
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

WAYPOINTS = [
    (0.0,  0.0,  -2.0),
    (10.0, 0.0,  -2.0),
    (10.0, 10.0, -2.0),
    (0.0,  10.0, -2.0),
    (0.0,  0.0,  -2.0),
]

REACH_DIST   = 0.4   # m   – prepni waypoint kdyz je dron blize nez toto
ALT_TOL      = 0.3   # m   – pocitej dosazeni az kdyz je z blizko cile
CRUISE_SPEED = 1.0   # m/s – rychlost pohybu virtualniho setpointu (= rychlost dronu)
DT           = 0.1   # s   – perioda timeru (10 Hz)


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        self._offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', QOS_PUB)
        self._trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', QOS_PUB)
        self._command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', QOS_PUB)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._pos_cb, QOS_SUB)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, QOS_SUB)

        self._tick = 0
        self._wp = 0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._roll   = 0.0
        self._pitch  = 0.0
        self._yaw    = 0.0
        self._landing = False
        # Virtualni setpoint – pohybuje se po malych krocich k waypointu.
        # Position error je vzdy maly → PX4 nema proc agresivne akcelerovat.
        self._vsp = [0.0, 0.0, 0.0]  # nastavi se pri prvnim pos_cb

        self.create_timer(0.1, self._cb)

    def _pos_cb(self, msg):
        self._x = msg.x
        self._y = msg.y
        self._z = msg.z
        # Inicializuj virtualni setpoint na aktualni pozici pri prvnim callbacku
        if self._vsp == [0.0, 0.0, 0.0]:
            self._vsp = [self._x, self._y, self._z]
        if self._tick % 20 == 0:
            self.get_logger().info(
                f'POS  x={self._x:6.2f} y={self._y:6.2f} z={self._z:6.2f} | '
                f'ATT  roll={self._roll:6.1f}° pitch={self._pitch:6.1f}° yaw={self._yaw:6.1f}°'
            )

    def _att_cb(self, msg):
        # Quaternion Hamilton: q = [w, x, y, z]  (FRD body → NED earth)
        w, x, y, z = msg.q

        roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        sinp  = 2*(w*y - z*x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

        self._roll  = roll
        self._pitch = pitch
        self._yaw   = yaw

    def _cb(self):
        self._publish_offboard_mode()
        self._publish_setpoint(WAYPOINTS[self._wp])

        if self._tick == 10:
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Armed and offboard engaged')

        if self._tick > 10 and not self._landing:
            target = WAYPOINTS[self._wp]
            at_alt = abs(self._z - target[2]) < ALT_TOL
            dist   = math.dist((self._x, self._y, self._z), target)
            if at_alt and dist < REACH_DIST:
                self.get_logger().info(f'Waypoint {self._wp} reached, dist={dist:.2f}m')
                if self._wp < len(WAYPOINTS) - 1:
                    self._wp += 1
                    self.get_logger().info(f'Next waypoint {self._wp}: {WAYPOINTS[self._wp]}')
                else:
                    self._landing = True
                    self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.get_logger().info('All waypoints done, landing')

        self._tick += 1

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_setpoint(self, wp):
        # Moving virtual setpoint: posun VSP o max CRUISE_SPEED*DT smerem k waypointu.
        # PX4 vzdy dostava position setpoint blizko aktualni polohy dronu
        # → position error maly → maly tilt (<10°), zadny overshooting.
        step = CRUISE_SPEED * DT
        dx = wp[0] - self._vsp[0]
        dy = wp[1] - self._vsp[1]
        dz = wp[2] - self._vsp[2]
        d = math.sqrt(dx*dx + dy*dy + dz*dz)

        if d > step:
            self._vsp[0] += (dx / d) * step
            self._vsp[1] += (dy / d) * step
            self._vsp[2] += (dz / d) * step
        else:
            self._vsp[0] = float(wp[0])
            self._vsp[1] = float(wp[1])
            self._vsp[2] = float(wp[2])

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [self._vsp[0], self._vsp[1], self._vsp[2]]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = nan
        msg.timestamp    = self._now_us()
        self._trajectory_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._command_pub.publish(msg)

    def _now_us(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
