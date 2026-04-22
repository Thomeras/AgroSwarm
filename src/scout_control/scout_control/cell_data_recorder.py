"""
cell_data_recorder.py — per-cell snapshot recorder for ML training data

Passive observer: listens for CELL_COMPLETE events from swarm agents and
saves a snapshot (camera JPG + meta.json) for each cell visit.

TOPICS:
  Subscribe:
    /swarm/drone_status                          (std_msgs/String JSON)
      {"drone_id":"drone_0","status":"CELL_COMPLETE","cell_id":"x4_y2"}
    /fmu/out/vehicle_local_position_v1           (px4_msgs/VehicleLocalPosition)  — drone_0
    /px4_1/fmu/out/vehicle_local_position_v1     (px4_msgs/VehicleLocalPosition)  — drone_1
    /camera/image_raw                            (sensor_msgs/Image, cv_bridge)

OUTPUT LAYOUT:
  <ws_root>/cell_data/{cell_id}/visit_{NNN}/
    image.jpg   — latest camera frame at event time (omitted if no frame available)
    meta.json   — {"timestamp_utc", "drone_id", "cell_id", "visit", "ned": {x,y,z}}

USAGE:
  ros2 run scout_control cell_data_recorder
"""

import json
import os
import time
from datetime import datetime, timezone

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from scout_control.avoidance.types import SwarmDroneStatusEvent
from scout_control.paths import CELL_DATA_DIR

# ── QoS ───────────────────────────────────────────────────────────────────────

QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

QOS_CAMERA = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

QOS_POS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# drone_id → position topic
_POS_TOPICS: dict[str, str] = {
    "drone_0": "/fmu/out/vehicle_local_position_v1",
    "drone_1": "/px4_1/fmu/out/vehicle_local_position_v1",
}

# drone_id → camera image topic (camera bridge outputs per-drone topics in E2E mission)
_CAMERA_TOPICS: dict[str, str] = {
    "drone_0": "/drone_0/camera/image_raw",
    "drone_1": "/drone_1/camera/image_raw",
}

# Seconds since last frame before a WARN is logged (camera may be stale)
_FRAME_TTL_S: float = 10.0


class CellDataRecorder(Node):
    """
    Passive ML-data collector.

    One instance covers the entire swarm.  Subscribes to all known
    drone position topics at startup and to the shared camera feed.
    On each CELL_COMPLETE event it flushes the latest data to disk.
    """

    def __init__(self) -> None:
        super().__init__("cell_data_recorder")

        self._bridge = CvBridge()

        # Latest camera frame per drone: drone_id → (frame_array, received_time_s)
        # TTL-checked at snapshot time to detect stale / missing camera bridges.
        self._frames: dict[str, tuple] = {}   # (np.ndarray, float)

        # Latest NED position per drone_id
        self._positions: dict[str, dict[str, float]] = {}

        # How many times each cell has been visited (used for visit_NNN numbering)
        self._visit_counts: dict[str, int] = {}

        # ── Camera subscribers (one per drone) ────────────────────────────────
        # Subscribes to per-drone topics that camera_bridge outputs in E2E mission
        # (e.g. /drone_0/camera/image_raw, /drone_1/camera/image_raw).
        # A single /camera/image_raw subscriber misses per-drone frames when the
        # bridge uses namespaced topics (Bug 4).
        for drone_id, cam_topic in _CAMERA_TOPICS.items():
            self.create_subscription(
                Image, cam_topic,
                lambda msg, did=drone_id: self._camera_cb(msg, did),
                QOS_CAMERA,
            )
            self.get_logger().info(f"Subscribing to camera topic: {cam_topic}")

        # ── Position subscribers (one per known drone) ────────────────────────
        for drone_id, topic in _POS_TOPICS.items():
            self.create_subscription(
                VehicleLocalPosition, topic,
                lambda msg, did=drone_id: self._position_cb(msg, did),
                QOS_POS,
            )
            self.get_logger().info(f"Subscribing to position topic: {topic}")

        # ── Status subscriber ─────────────────────────────────────────────────
        self.create_subscription(
            String, "/swarm/drone_status",
            self._status_cb, QOS_STATUS,
        )

        os.makedirs(CELL_DATA_DIR, exist_ok=True)
        self.get_logger().info(
            f"CellDataRecorder ready | output dir: {CELL_DATA_DIR}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _camera_cb(self, msg: Image, drone_id: str) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._frames[drone_id] = (frame, time.monotonic())
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge error ({drone_id}): {e}")

    def _position_cb(self, msg: VehicleLocalPosition, drone_id: str) -> None:
        self._positions[drone_id] = {
            "x": float(msg.x),
            "y": float(msg.y),
            "z": float(msg.z),
        }

    def _status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"cell_data_recorder: invalid JSON: {msg.data[:80]}"
            )
            return

        if not isinstance(data, dict):
            return

        event = SwarmDroneStatusEvent.from_payload(data)
        if event.status.upper() != "CELL_COMPLETE":
            return

        drone_id = event.drone_id
        cell_id = event.cell_id

        if not drone_id or not cell_id:
            self.get_logger().warn(
                f"cell_data_recorder: CELL_COMPLETE missing drone_id/cell_id: {data}"
            )
            return

        self._record_snapshot(drone_id, cell_id)

    # ── Snapshot logic ────────────────────────────────────────────────────────

    def _record_snapshot(self, drone_id: str, cell_id: str) -> None:
        # Increment visit counter for this cell
        visit_num = self._visit_counts.get(cell_id, 0) + 1
        self._visit_counts[cell_id] = visit_num

        visit_dir = os.path.join(
            CELL_DATA_DIR, cell_id, f"visit_{visit_num:03d}"
        )
        os.makedirs(visit_dir, exist_ok=True)

        timestamp = datetime.now(timezone.utc).isoformat()
        ned = self._positions.get(drone_id)

        # ── meta.json ─────────────────────────────────────────────────────────
        meta: dict = {
            "timestamp_utc": timestamp,
            "drone_id":      drone_id,
            "cell_id":       cell_id,
            "visit":         visit_num,
            "ned":           ned,  # None if position not yet received
        }
        meta_path = os.path.join(visit_dir, "meta.json")
        try:
            with open(meta_path, "w") as f:
                json.dump(meta, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"Failed to write {meta_path}: {e}")
            return

        # ── image.jpg ─────────────────────────────────────────────────────────
        frame_entry = self._frames.get(drone_id)   # (frame, received_time_s) or None
        now         = time.monotonic()

        if frame_entry is None:
            frame = None
            self.get_logger().warn(
                f"cell_data_recorder: no camera frame ever received for {drone_id} "
                f"— check that camera bridge is running for this drone"
            )
        else:
            frame, frame_time = frame_entry
            elapsed = now - frame_time
            if elapsed > _FRAME_TTL_S:
                self.get_logger().warn(
                    f"cell_data_recorder: last frame for {drone_id} is "
                    f"{elapsed:.1f}s old (TTL={_FRAME_TTL_S}s) — "
                    "camera bridge may have stopped; saving stale frame"
                )

        if frame is not None:
            img_path = os.path.join(visit_dir, "image.jpg")
            try:
                cv2.imwrite(img_path, frame)
            except Exception as e:
                self.get_logger().error(f"Failed to write {img_path}: {e}")
                frame = None   # mark as not saved for log line below

        ned_str = (
            f"x={ned['x']:.2f} y={ned['y']:.2f} z={ned['z']:.2f}"
            if ned else "ned=unknown"
        )
        self.get_logger().info(
            f"SNAPSHOT | {drone_id} @ {cell_id} visit={visit_num:03d} "
            f"| {ned_str} | img={'yes' if frame is not None else 'NO'}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CellDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
