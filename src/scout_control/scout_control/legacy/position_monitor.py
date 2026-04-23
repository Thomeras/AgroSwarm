import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos
        )

    def position_callback(self, msg):
        self.get_logger().info(
            f'Pozice -> X: {msg.x:.2f} Y: {msg.y:.2f} Z: {msg.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()