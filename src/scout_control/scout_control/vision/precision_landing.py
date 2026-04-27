# flake8: noqa
"""Precision landing advisory node.

The node publishes detected pad offsets as advisory data.  It intentionally
does not publish PX4 setpoints or modify obstacle_avoidance_runtime.
"""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.vision.pad_detector import CameraIntrinsics, detect_pad_marker

QOS_SENSOR = QoSProfile(
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


class PrecisionLanding(Node):
    """Detect home pad marker and publish ``/{drone}/precision_landing/offset``."""

    def __init__(self) -> None:
        super().__init__("precision_landing")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("marker_size_m", 0.35)
        self.declare_parameter("active_phase", "RETURN_HOME")
        self.declare_parameter("max_active_altitude_m", 5.0)
        self.declare_parameter("advisory_only", True)
        self._drone_id = int(self.get_parameter("drone_id").value)
        self._hub = TelemetryHub.for_drone(self._drone_id)
        self._bridge = CvBridge()
        self._intrinsics: CameraIntrinsics | None = None
        self._runtime_phase = ""
        self._altitude_m = 999.0

        topics = self._hub.topics
        self.create_subscription(Image, topics.camera_image, self._on_image, QOS_SENSOR)
        self.create_subscription(CameraInfo, topics.camera_info, self._on_camera_info, QOS_SENSOR)
        self.create_subscription(String, topics.avoidance_status_json, self._on_status, QOS_STATUS)
        self.create_subscription(String, "/swarm/home_positions", self._on_home_positions, 10)
        self._offset_pub = self.create_publisher(
            String, topics.precision_landing_offset, 10
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self._intrinsics = CameraIntrinsics(
                fx=float(msg.k[0]),
                fy=float(msg.k[4]),
                cx=float(msg.k[2]),
                cy=float(msg.k[5]),
            )

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._runtime_phase = str(payload.get("phase", payload.get("runtime_phase", "")))
        pos = payload.get("position_ned") or payload.get("drone_ned") or []
        if isinstance(pos, list) and len(pos) >= 3:
            self._altitude_m = abs(float(pos[2]))
        elif "altitude_m" in payload:
            self._altitude_m = float(payload["altitude_m"])

    def _on_home_positions(self, _msg: String) -> None:
        pass

    def _active(self) -> bool:
        wanted = str(self.get_parameter("active_phase").value)
        max_alt = float(self.get_parameter("max_active_altitude_m").value)
        return (self._runtime_phase == wanted or wanted == "*") and self._altitude_m <= max_alt

    def _on_image(self, msg: Image) -> None:
        if not self._active():
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Camera frame conversion failed: {exc}")
            return
        detection = detect_pad_marker(
            frame,
            self._intrinsics,
            marker_size_m=float(self.get_parameter("marker_size_m").value),
        )
        if detection is None:
            return
        dx, dy = detection.offset_xy_body_m
        payload: dict[str, Any] = {
            "version": 1,
            "drone_id": self._drone_id,
            "dx_m": dx,
            "dy_m": dy,
            "range_m": detection.range_m,
            "marker_id": detection.marker_id,
            "confidence": detection.confidence,
            "valid_for_s": 1.0,
            "advisory_only": bool(self.get_parameter("advisory_only").value),
            "stamp_s": time.time(),
        }
        self._offset_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PrecisionLanding()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

