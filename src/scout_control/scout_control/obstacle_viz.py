"""
obstacle_viz.py — RViz2 vizualizace překážek a trajektorií pro obstacle avoidance test.

Publikuje MarkerArray s modely překážek, trajektorii dronu (plánovanou vs skutečnou)
a point cloud jako hustou reprezentaci překážek.

Topiky (publish):
  /visualization/obstacle_markers   visualization_msgs/MarkerArray  1 Hz
  /visualization/drone_marker       visualization_msgs/Marker       10 Hz
  /visualization/planned_path       nav_msgs/Path                   5 Hz
  /visualization/actual_path        nav_msgs/Path                   5 Hz
  /visualization/obstacle_cloud     sensor_msgs/PointCloud2         0.5 Hz

Topiky (subscribe):
  /px4_N/fmu/out/vehicle_local_position_v1 or /fmu/out/... (drone_0)  VehicleLocalPosition
  /drone_N/avoidance/planned_path                                   nav_msgs/Path
  /drone_N/avoidance/actual_path                                    nav_msgs/Path
  /drone_N/avoidance/active                                         std_msgs/Bool

Souřadnicový systém pro RViz2 (frame_id="map"):
  rviz_x = ned_y   (East)
  rviz_y = ned_x   (North)
  rviz_z = -ned_z  (Up)

Spuštění:
  ros2 run scout_control obstacle_viz
  rviz2 -d /path/to/obstacle_avoidance.rviz
"""

import math
import struct

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from px4_msgs.msg import VehicleLocalPosition

# ── QoS ───────────────────────────────────────────────────────────────────────
QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=10,
)
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5,
)
QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5,
)

# ── Překážky — pozice NED + vizuální parametry ────────────────────────────────
# (shodné s obstacle_avoidance_mission.py)
OBSTACLES = [
    {
        "id":     "wall_north",
        "ned_x":  12.0, "ned_y": 0.0,
        "shape":  "box",
        "size_x": 0.4,  "size_y": 6.0,  "size_z": 9.0,   # NED: x=tloušťka, y=šířka, z=výška
        "color":  (0.9, 0.15, 0.15, 0.85),
        "label":  "WALL\nNORTH",
    },
    {
        "id":     "poles_east",
        "ned_x":  0.0,  "ned_y": 12.0,
        "shape":  "cylinder",
        "radius": 1.8,  "size_z": 9.0,   # reprezentace celého clusteru
        "color":  (0.9, 0.75, 0.0,  0.85),
        "label":  "POLES\nEAST",
    },
    {
        "id":     "building_ne",
        "ned_x":  9.0,  "ned_y": 9.0,
        "shape":  "box",
        "size_x": 4.0,  "size_y": 4.0,  "size_z": 9.0,
        "color":  (0.55, 0.55, 0.52, 0.85),
        "label":  "BUILDING\nNE",
    },
    {
        "id":     "fence_nnw",
        "ned_x":  12.0, "ned_y": -8.0,
        "shape":  "box",
        "size_x": 7.0,  "size_y": 0.3,  "size_z": 9.0,   # NED: x=délka plotu, y=tloušťka
        "color":  (0.85, 0.45, 0.08, 0.85),
        "label":  "FENCE\nNNW",
    },
]

DRONE_FLIGHT_ALT = 5.0    # m — výška letu pro vizualizaci
CLOUD_RESOLUTION = 0.3    # m — rozlišení point cloud mřížky


def _ned_to_rviz(ned_x: float, ned_y: float, alt: float = 0.0) -> tuple:
    """NED → RViz2 ENU (frame_id=map)."""
    return (float(ned_y), float(ned_x), float(alt))


class ObstacleViz(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_viz")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("subscribe_legacy_topics", False)
        drone_id = int(self.get_parameter("drone_id").value)
        subscribe_legacy = bool(self.get_parameter("subscribe_legacy_topics").value)
        drone_ns = f"drone_{drone_id}"
        px4_ns = "" if drone_id == 0 else f"/px4_{drone_id}"

        # ── Stav ──────────────────────────────────────────────────────────────
        self._drone_x: float       = 0.0
        self._drone_y: float       = 0.0
        self._drone_z: float       = 0.0
        self._avoidance: bool      = False
        self._planned_path: Path   = Path()
        self._actual_path:  Path   = Path()

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_obs_markers = self.create_publisher(
            MarkerArray, "/visualization/obstacle_markers", QOS_PUB)
        self._pub_drone = self.create_publisher(
            Marker, "/visualization/drone_marker", QOS_PUB)
        self._pub_planned = self.create_publisher(
            Path, "/visualization/planned_path", QOS_PUB)
        self._pub_actual = self.create_publisher(
            Path, "/visualization/actual_path", QOS_PUB)
        self._pub_cloud = self.create_publisher(
            PointCloud2, "/visualization/obstacle_cloud", QOS_PUB)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            f"{px4_ns}/fmu/out/vehicle_local_position_v1",
            self._pos_cb, QOS_PX4,
        )
        self.create_subscription(Bool, f"/{drone_ns}/avoidance/active", self._avoid_cb, QOS_SUB)
        self.create_subscription(Path, f"/{drone_ns}/avoidance/planned_path", self._plan_cb, QOS_SUB)
        self.create_subscription(Path, f"/{drone_ns}/avoidance/actual_path", self._actual_cb, QOS_SUB)
        if subscribe_legacy:
            self.create_subscription(
                Bool, "/obstacle_avoidance/avoidance_active", self._avoid_cb, QOS_SUB
            )
            self.create_subscription(
                Path, "/obstacle_avoidance/planned_path", self._plan_cb, QOS_SUB
            )
            self.create_subscription(
                Path, "/obstacle_avoidance/actual_path", self._actual_cb, QOS_SUB
            )

        # ── Timery ────────────────────────────────────────────────────────────
        self.create_timer(1.0,  self._pub_obs_cb)
        self.create_timer(0.1,  self._pub_drone_cb)
        self.create_timer(0.2,  self._pub_paths_cb)
        self.create_timer(2.0,  self._pub_cloud_cb)

        self.get_logger().info(
            f"obstacle_viz ready — drone_id={drone_id} legacy_topics="
            f"{'on' if subscribe_legacy else 'off'}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x = msg.x
        self._drone_y = msg.y
        self._drone_z = msg.z

    def _avoid_cb(self, msg: Bool) -> None:
        self._avoidance = msg.data

    def _plan_cb(self, msg: Path) -> None:
        self._planned_path = msg

    def _actual_cb(self, msg: Path) -> None:
        self._actual_path = msg

    # ── Obstacle markers ──────────────────────────────────────────────────────

    def _pub_obs_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr   = MarkerArray()
        mid   = 0

        for obs in OBSTACLES:
            rx, ry, rz_base = _ned_to_rviz(obs["ned_x"], obs["ned_y"])
            h_z = obs["size_z"]

            # Těleso překážky
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp    = stamp
            m.ns              = "obstacles"
            m.id              = mid
            m.action          = Marker.ADD
            mid += 1

            c = obs["color"]
            m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])

            if obs["shape"] == "box":
                m.type = Marker.CUBE
                # Konverze NED size → ENU/RViz size:
                # ned_x → rviz_y, ned_y → rviz_x
                m.scale.x = float(obs["size_y"])   # ENU-x = NED-y šíře
                m.scale.y = float(obs["size_x"])   # ENU-y = NED-x tloušťka/délka
                m.scale.z = float(h_z)
            else:  # cylinder
                m.type    = Marker.CYLINDER
                m.scale.x = float(obs.get("radius", 2.0)) * 2
                m.scale.y = float(obs.get("radius", 2.0)) * 2
                m.scale.z = float(h_z)

            m.pose.position.x = rx
            m.pose.position.y = ry
            m.pose.position.z = h_z / 2.0
            m.pose.orientation.w = 1.0
            arr.markers.append(m)

            # APF influence zone (průsvitná sféra)
            inf_r = obs.get("radius", math.hypot(obs.get("size_x", 0)/2, obs.get("size_y", 0)/2)) + 5.5
            mi = Marker()
            mi.header.frame_id = "map"
            mi.header.stamp    = stamp
            mi.ns              = "influence"
            mi.id              = mid
            mi.action          = Marker.ADD
            mi.type            = Marker.SPHERE
            mi.scale.x = mi.scale.y = mi.scale.z = inf_r * 2
            mi.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.08)
            mi.pose.position.x = rx
            mi.pose.position.y = ry
            mi.pose.position.z = DRONE_FLIGHT_ALT
            mi.pose.orientation.w = 1.0
            arr.markers.append(mi)
            mid += 1

            # Popisek
            mt = Marker()
            mt.header.frame_id = "map"
            mt.header.stamp    = stamp
            mt.ns              = "labels"
            mt.id              = mid
            mt.action          = Marker.ADD
            mt.type            = Marker.TEXT_VIEW_FACING
            mt.scale.z         = 1.0
            mt.color           = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            mt.text            = obs["label"]
            mt.pose.position.x = rx
            mt.pose.position.y = ry
            mt.pose.position.z = h_z + 1.0
            mt.pose.orientation.w = 1.0
            arr.markers.append(mt)
            mid += 1

        # HOME marker
        mh = Marker()
        mh.header.frame_id = "map"
        mh.header.stamp    = stamp
        mh.ns     = "home"
        mh.id     = mid
        mh.action = Marker.ADD
        mh.type   = Marker.CYLINDER
        mh.scale.x = mh.scale.y = 1.8
        mh.scale.z = 0.1
        mh.color  = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
        mh.pose.position.x = 0.0
        mh.pose.position.y = 0.0
        mh.pose.position.z = 0.05
        mh.pose.orientation.w = 1.0
        arr.markers.append(mh)

        self._pub_obs_markers.publish(arr)

    # ── Drone marker ──────────────────────────────────────────────────────────

    def _pub_drone_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        rx, ry, _ = _ned_to_rviz(self._drone_x, self._drone_y)
        rz = float(-self._drone_z)

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = stamp
        m.ns     = "drone"
        m.id     = 0
        m.action = Marker.ADD
        m.type   = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color  = (
            ColorRGBA(r=0.0, g=0.9, b=0.3, a=1.0) if not self._avoidance
            else ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
        )
        m.pose.position.x = rx
        m.pose.position.y = ry
        m.pose.position.z = rz
        m.pose.orientation.w = 1.0
        self._pub_drone.publish(m)

    # ── Cesty ─────────────────────────────────────────────────────────────────

    def _pub_paths_cb(self) -> None:
        self._pub_planned.publish(self._planned_path)
        self._pub_actual.publish(self._actual_path)

    # ── Point cloud překážek ──────────────────────────────────────────────────

    def _pub_cloud_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        points: list[tuple] = []

        for obs in OBSTACLES:
            if obs["shape"] == "box":
                hx = obs["size_x"] / 2
                hy = obs["size_y"] / 2
                hz = obs["size_z"]
            else:
                hx = hy = obs.get("radius", 1.8)
                hz = obs["size_z"]

            x_steps = max(1, int(2 * hy / CLOUD_RESOLUTION))  # ENU-x = NED-y
            y_steps = max(1, int(2 * hx / CLOUD_RESOLUTION))  # ENU-y = NED-x
            z_steps = max(1, int(hz / CLOUD_RESOLUTION))

            cx, cy = _ned_to_rviz(obs["ned_x"], obs["ned_y"])[:2]

            for xi in range(x_steps + 1):
                for yi in range(y_steps + 1):
                    for zi in range(z_steps + 1):
                        px = cx - hy + xi * CLOUD_RESOLUTION
                        py = cy - hx + yi * CLOUD_RESOLUTION
                        pz = zi * CLOUD_RESOLUTION
                        points.append((px, py, pz))

        if not points:
            return

        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for p in points:
            data += struct.pack("fff", *p)

        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp    = stamp
        msg.height          = 1
        msg.width           = len(points)
        msg.fields          = fields
        msg.is_bigendian    = False
        msg.point_step      = 12
        msg.row_step        = 12 * len(points)
        msg.data            = bytes(data)
        msg.is_dense        = True
        self._pub_cloud.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
