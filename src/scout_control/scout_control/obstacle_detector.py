"""
obstacle_detector.py — Obstacle detection from OakD-Lite depth camera.

Udržuje 2D occupancy mapu překážek v NED world frame.
Grid se inicializuje dynamicky z /field/grid topicu.

Interfaces:
  Subscribe:
    /drone_N/depth/image_raw                sensor_msgs/Image         QOS_SENSOR
    /fmu/out/vehicle_local_position_v1      px4_msgs/VehicleLocalPosition  QOS_PX4
      drone 0: bare topic
      drone N: /px4_N/fmu/out/...
    /field/grid                             std_msgs/String JSON       QOS_LATCHED

  Publish:
    /drone_N/obstacles/detected    std_msgs/String JSON    10 Hz
    /drone_N/obstacles/clear       std_msgs/Bool           10 Hz

Parameters:
  drone_id        int    0
  warn_distance   float  4.0    m — warn threshold
  stop_distance   float  2.0    m — critical threshold
  cell_size       float  0.5    m per occupancy grid cell
  map_decay_secs  float  30.0   s — clear cells older than this
  camera_hfov     float  71.9   degrees — OakD-Lite
  cam_width       int    640
  cam_height      int    400

Usage:
  ros2 run scout_control obstacle_detector --ros-args -p drone_id:=0
  ros2 run scout_control obstacle_detector --ros-args -p drone_id:=1
"""

import json
import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from px4_msgs.msg import VehicleLocalPosition

# ── QoS ───────────────────────────────────────────────────────────────────────
QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
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
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── Constants ─────────────────────────────────────────────────────────────────
DEPTH_MIN_M       = 0.1     # m — reject closer readings (noise / self-reflection)
DEPTH_MAX_M       = 20.0    # m — reject readings beyond this range
PIXEL_STRIDE      = 4       # subsample every Nth pixel for performance
GRID_BUFFER_RATIO = 0.1     # 10% buffer on each side of field bounds
DECAY_TIMER_SECS  = 5.0     # how often to run map decay (s)
PUBLISH_HZ        = 10.0    # Hz


class ObstacleDetector(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_detector")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("warn_distance", 4.0)
        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("cell_size", 0.5)
        self.declare_parameter("map_decay_secs", 30.0)
        self.declare_parameter("camera_hfov", 71.9)
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 400)

        self._drone_id    = self.get_parameter("drone_id").value
        self._warn_dist   = self.get_parameter("warn_distance").value
        self._stop_dist   = self.get_parameter("stop_distance").value
        self._cell_size   = self.get_parameter("cell_size").value
        self._decay_secs  = self.get_parameter("map_decay_secs").value
        self._hfov_rad    = math.radians(self.get_parameter("camera_hfov").value)
        self._cam_w       = self.get_parameter("cam_width").value
        self._cam_h       = self.get_parameter("cam_height").value

        drone_ns = f"drone_{self._drone_id}"
        px4_ns   = "" if self._drone_id == 0 else f"/px4_{self._drone_id}"

        # ── State ─────────────────────────────────────────────────────────────
        self._bridge      = CvBridge()
        self._lock        = threading.Lock()
        self._grid: Optional[np.ndarray] = None
        self._grid_origin_x = 0.0
        self._grid_origin_y = 0.0
        self._grid_ready  = False

        self._drone_x   = 0.0
        self._drone_y   = 0.0
        self._drone_yaw = 0.0
        self._pos_valid = False

        # per-sector min distances, updated each depth frame
        self._sector_dist: dict[str, float] = {
            "left": 99.0, "center": 99.0, "right": 99.0
        }

        # precompute camera focal length
        self._fx = (self._cam_w / 2.0) / math.tan(self._hfov_rad / 2.0)
        self._cx = self._cam_w / 2.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            f"/{drone_ns}/depth/image_raw",
            self._depth_cb,
            QOS_SENSOR,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{px4_ns}/fmu/out/vehicle_local_position_v1",
            self._pos_cb,
            QOS_PX4,
        )
        self.create_subscription(
            String,
            "/field/grid",
            self._grid_cb,
            QOS_LATCHED,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_detected = self.create_publisher(
            String,
            f"/{drone_ns}/obstacles/detected",
            QOS_PUB,
        )
        self._pub_clear = self.create_publisher(
            Bool,
            f"/{drone_ns}/obstacles/clear",
            QOS_PUB,
        )

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        self.create_timer(DECAY_TIMER_SECS, self._decay_cb)

        self.get_logger().info(
            f"obstacle_detector started — drone_{self._drone_id} "
            f"warn={self._warn_dist}m stop={self._stop_dist}m cell={self._cell_size}m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x   = msg.x
        self._drone_y   = msg.y
        self._drone_yaw = msg.heading
        self._pos_valid = True

    def _grid_cb(self, msg: String) -> None:
        if self._grid_ready:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"field/grid JSON decode error: {e}")
            return

        cells = data.get("cells", [])
        if not cells:
            self.get_logger().warn("field/grid received but 'cells' list is empty — ignoring")
            return

        xs = [c["x"] for c in cells]
        ys = [c["y"] for c in cells]

        # infer one-cell size from field_grid cell_size if present, else use parameter
        field_cell = data.get("cell_size_m", self._cell_size)

        field_w = (max(xs) - min(xs)) + field_cell
        field_h = (max(ys) - min(ys)) + field_cell

        grid_w = int((field_w * (1.0 + 2 * GRID_BUFFER_RATIO)) / self._cell_size) + 1
        grid_h = int((field_h * (1.0 + 2 * GRID_BUFFER_RATIO)) / self._cell_size) + 1

        origin_x = min(xs) - field_w * GRID_BUFFER_RATIO
        origin_y = min(ys) - field_h * GRID_BUFFER_RATIO

        with self._lock:
            self._grid          = np.zeros((grid_w, grid_h), dtype=np.float64)
            self._grid_origin_x = origin_x
            self._grid_origin_y = origin_y
            self._grid_ready    = True

        self.get_logger().info(
            f"occupancy grid initialized {grid_w}×{grid_h} cells "
            f"origin=({origin_x:.1f}, {origin_y:.1f}) cell={self._cell_size}m"
        )

    def _depth_cb(self, msg: Image) -> None:
        # Sector detection (left/center/right distances) works without field grid
        # or valid position — process unconditionally.  The occupancy-grid projection
        # inside _process_depth already short-circuits when self._grid is None.
        try:
            depth = self.get_bridge_image(msg)
        except Exception as e:
            self.get_logger().warn(f"CvBridge conversion failed: {e}")
            return

        self._process_depth(depth)

    def get_bridge_image(self, msg: Image) -> np.ndarray:
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)

    def _process_depth(self, depth: np.ndarray) -> None:
        h, w = depth.shape[:2]
        w3 = w // 3

        # sector min distances
        def sector_min(col_start: int, col_end: int) -> float:
            patch = depth[:, col_start:col_end]
            valid = patch[(patch >= DEPTH_MIN_M) & (patch <= DEPTH_MAX_M)]
            return float(np.min(valid)) if valid.size > 0 else 99.0

        sector_dist = {
            "left":   sector_min(0,    w3),
            "center": sector_min(w3,   2 * w3),
            "right":  sector_min(2 * w3, w),
        }
        self._sector_dist = sector_dist

        # world-point projection — subsample every PIXEL_STRIDE pixels
        drone_x   = self._drone_x
        drone_y   = self._drone_y
        yaw       = self._drone_yaw
        cos_yaw   = math.cos(yaw)
        sin_yaw   = math.sin(yaw)
        fx        = self._fx
        cx        = self._cx
        cell_size = self._cell_size

        rows = np.arange(0, h, PIXEL_STRIDE)
        cols = np.arange(0, w, PIXEL_STRIDE)
        uu, _ = np.meshgrid(cols, rows)         # (sub_h, sub_w)
        d_sub = depth[::PIXEL_STRIDE, ::PIXEL_STRIDE]

        # mask valid depth values
        valid = (d_sub >= DEPTH_MIN_M) & (d_sub <= DEPTH_MAX_M) & ~np.isnan(d_sub)
        d_v   = d_sub[valid]
        u_v   = uu[valid].astype(np.float32)

        if d_v.size == 0:
            return

        # camera → NED world
        x_cam = (u_v - cx) * d_v / fx   # horizontal offset
        z_cam = d_v                       # forward in camera frame

        wx = drone_x + z_cam * cos_yaw - x_cam * sin_yaw
        wy = drone_y + z_cam * sin_yaw + x_cam * cos_yaw

        with self._lock:
            if self._grid is None:
                return
            grid        = self._grid
            origin_x    = self._grid_origin_x
            origin_y    = self._grid_origin_y
            gw, gh      = grid.shape

            gx = ((wx - origin_x) / cell_size).astype(int)
            gy = ((wy - origin_y) / cell_size).astype(int)

            in_bounds = (gx >= 0) & (gx < gw) & (gy >= 0) & (gy < gh)
            ts = time.time()
            grid[gx[in_bounds], gy[in_bounds]] = ts

    # ── Publish timer ─────────────────────────────────────────────────────────

    def _publish_cb(self) -> None:
        sd       = self._sector_dist
        closest  = min(sd.values())
        warn     = closest < self._warn_dist
        critical = closest < self._stop_dist
        free     = [d for d, dist in sd.items() if dist > self._warn_dist]

        payload = json.dumps({
            "drone_id":        f"drone_{self._drone_id}",
            "closest":         round(closest, 2),
            "closest_m":       round(closest, 2),
            "sectors":         {k: round(v, 2) for k, v in sd.items()},
            "free_directions": free,
            "warn":            warn,
            "critical":        critical,
        })

        self._pub_detected.publish(String(data=payload))
        self._pub_clear.publish(Bool(data=not warn))

    # ── Decay timer ───────────────────────────────────────────────────────────

    def _decay_cb(self) -> None:
        with self._lock:
            if self._grid is None:
                return
            now     = time.time()
            stale   = (self._grid > 0.0) & ((now - self._grid) > self._decay_secs)
            count   = int(np.sum(stale))
            if count > 0:
                self._grid[stale] = 0.0
                self.get_logger().info(f"map decay cleared {count} stale cells")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
