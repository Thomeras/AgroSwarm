# flake8: noqa
"""Build and persist mapping field-model artifacts."""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from scout_control.avoidance.depth_projector import DepthProjector
from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.mapping.heightmap import Heightmap2D
from scout_control.mapping.obstacle_extractor import extract_obstacles
from scout_control.utils.paths import FIELD_MODEL_DIR


class FieldModelBuilder(Node):
    """Accumulate mapping point batches and write versioned field-model files."""

    def __init__(self) -> None:
        super().__init__("field_model_builder")
        self.declare_parameter("origin_x", -50.0)
        self.declare_parameter("origin_y", -50.0)
        self.declare_parameter("width_m", 100.0)
        self.declare_parameter("height_m", 100.0)
        self.declare_parameter("cell_size_m", 0.5)
        self.declare_parameter("obstacle_cell_size_m", 0.75)
        self.declare_parameter("min_obstacle_points", 3)
        self.declare_parameter("drone_count", 1)
        self.declare_parameter("depth_stride", 8)

        cell = float(self.get_parameter("cell_size_m").value)
        width = max(1, int(np.ceil(float(self.get_parameter("width_m").value) / cell)))
        height = max(1, int(np.ceil(float(self.get_parameter("height_m").value) / cell)))
        self._bridge = CvBridge()
        self._projector = DepthProjector(default_stride=int(self.get_parameter("depth_stride").value))
        self._poses: dict[int, tuple[float, float, float, float]] = {}
        self._heightmap = Heightmap2D(
            origin_ned=(
                float(self.get_parameter("origin_x").value),
                float(self.get_parameter("origin_y").value),
            ),
            cell_size_m=cell,
            width=width,
            height=height,
        )
        self._point_batches: list[np.ndarray] = []
        self.create_subscription(String, "/swarm/mapping_points", self._on_points, 10)
        self.create_subscription(String, "/swarm/mapping_complete", self._on_complete, 10)
        for drone_id in range(max(1, int(self.get_parameter("drone_count").value))):
            hub = TelemetryHub.for_drone(drone_id)
            self.create_subscription(
                VehicleLocalPosition,
                hub.topics.vehicle_local_position,
                lambda msg, did=drone_id: self._on_pose(did, msg),
                10,
            )
            self.create_subscription(
                CameraInfo,
                hub.topics.camera_info,
                self._on_camera_info,
                10,
            )
            self.create_subscription(
                Image,
                hub.topics.depth_image,
                lambda msg, did=drone_id: self._on_depth(did, msg),
                10,
            )
        self._manifest_pub = self.create_publisher(String, "/swarm/field_model_manifest", 10)

    def _on_points(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            pts = np.asarray(payload.get("points_ned", []), dtype=np.float32)
            if pts.ndim != 2 or pts.shape[1] < 3:
                return
        except Exception as exc:
            self.get_logger().warning(f"Ignoring malformed mapping points: {exc}")
            return
        self._heightmap.update_from_points(pts[:, :3])
        self._point_batches.append(pts[:, :3])

    def _on_pose(self, drone_id: int, msg: VehicleLocalPosition) -> None:
        heading = float(getattr(msg, "heading", 0.0))
        self._poses[drone_id] = (float(msg.x), float(msg.y), float(msg.z), heading)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        try:
            self._projector.set_camera_info(msg)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid camera info: {exc}")

    def _on_depth(self, drone_id: int, msg: Image) -> None:
        pose = self._poses.get(drone_id)
        if pose is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")
            return
        # Use depth_to_body_points so the collision-band filter inside
        # project_to_world_points does not discard terrain returns at world_z ≈ 0.
        body_batch = self._projector.depth_to_body_points(
            np.asarray(depth, dtype=np.float32),
            pixel_stride=int(self.get_parameter("depth_stride").value),
            source=f"drone_{drone_id}_depth",
        )
        body_pts = body_batch.points_xyz
        if body_pts.size == 0:
            return
        ox, oy, oz, yaw = pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        fwd = body_pts[:, 0]
        rgt = body_pts[:, 1]
        dwn = body_pts[:, 2]
        world_x = ox + fwd * cos_yaw - rgt * sin_yaw
        world_y = oy + fwd * sin_yaw + rgt * cos_yaw
        world_z = oz + dwn
        points = np.column_stack((world_x, world_y, world_z)).astype(np.float32)
        self._heightmap.update_from_points(points)
        self._point_batches.append(points)

    def _on_complete(self, _msg: String) -> None:
        manifest = self.persist()
        self._manifest_pub.publish(String(data=json.dumps(manifest, sort_keys=True)))

    def persist(self) -> dict[str, Any]:
        os.makedirs(FIELD_MODEL_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        hm_json = f"heightmap_{stamp}.json"
        hm_npy = f"heightmap_{stamp}.npy"
        obs_json = f"obstacles_{stamp}.json"

        heightmap_payload = self._heightmap.to_dict()
        heightmap_payload["npy_file"] = hm_npy
        with open(os.path.join(FIELD_MODEL_DIR, hm_json), "w", encoding="utf-8") as fh:
            json.dump(heightmap_payload, fh, indent=2, sort_keys=True)
        np.save(os.path.join(FIELD_MODEL_DIR, hm_npy), self._heightmap.min_z)

        points = (
            np.concatenate(self._point_batches, axis=0)
            if self._point_batches
            else np.empty((0, 3), dtype=np.float32)
        )
        obstacles = extract_obstacles(
            points,
            cell_size_m=float(self.get_parameter("obstacle_cell_size_m").value),
            min_points=int(self.get_parameter("min_obstacle_points").value),
        )
        obstacle_payload = {
            "version": 1,
            "created_at_s": time.time(),
            "obstacles": [item.to_dict() for item in obstacles],
        }
        with open(os.path.join(FIELD_MODEL_DIR, obs_json), "w", encoding="utf-8") as fh:
            json.dump(obstacle_payload, fh, indent=2, sort_keys=True)

        manifest_path = os.path.join(FIELD_MODEL_DIR, "manifest.json")
        manifest = _load_manifest(manifest_path)
        entry = {
            "version": 1,
            "created_at_s": time.time(),
            "heightmap_json": hm_json,
            "heightmap_npy": hm_npy,
            "obstacles_json": obs_json,
            "point_count": int(points.shape[0]),
            "obstacle_count": len(obstacles),
        }
        manifest.setdefault("version", 1)
        manifest.setdefault("entries", []).append(entry)
        manifest["latest"] = entry
        with open(manifest_path, "w", encoding="utf-8") as fh:
            json.dump(manifest, fh, indent=2, sort_keys=True)
        return manifest


def _load_manifest(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        return {"version": 1, "entries": []}
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {"version": 1, "entries": []}
    except Exception:
        return {"version": 1, "entries": []}


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FieldModelBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
