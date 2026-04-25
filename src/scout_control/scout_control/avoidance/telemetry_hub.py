"""Central topic contracts and PX4 input ownership diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


def _clean_topic(value: str | None) -> str:
    return str(value or "").strip()


@dataclass(frozen=True, slots=True)
class DroneTopicContract:
    """Canonical per-drone ROS topic contract used by runtime, agents, and tools."""

    drone_id: int
    drone_ns: str
    px4_ns: str
    camera_image: str
    depth_image: str
    camera_info: str
    terrain_range: str
    vehicle_local_position: str
    vehicle_status: str
    vehicle_control_mode: str
    vehicle_command_ack: str
    px4_in_offboard_control_mode: str
    px4_in_trajectory_setpoint: str
    px4_in_vehicle_command: str
    avoidance_target_cmd: str
    avoidance_target_cmd_json: str
    avoidance_status: str
    avoidance_status_json: str
    avoidance_events: str
    avoidance_active: str
    avoidance_planned_path: str
    avoidance_actual_path: str
    obstacles_detected: str
    obstacles_clear: str
    next_cell: str
    rth_target: str
    downward_lidar_scan: str

    @property
    def px4_input_topics(self) -> dict[str, str]:
        return {
            "offboard_control_mode": self.px4_in_offboard_control_mode,
            "trajectory_setpoint": self.px4_in_trajectory_setpoint,
            "vehicle_command": self.px4_in_vehicle_command,
        }

    def sensor_payload(self) -> dict[str, str]:
        return {
            "camera": self.camera_image,
            "depth": self.depth_image,
            "camera_info": self.camera_info,
            "terrain_range": self.terrain_range,
        }


@dataclass(frozen=True, slots=True)
class SwarmTopicContract:
    peer_telemetry: str = "/swarm/peer_telemetry"
    drone_status: str = "/swarm/drone_status"
    rth_request: str = "/swarm/rth_request"
    landed_confirmation: str = "/swarm/landed_confirmation"
    pad_assignment: str = "/swarm/pad_assignment"
    pad_query: str = "/swarm/pad_query"
    pad_response: str = "/swarm/pad_response"
    home_positions: str = "/swarm/home_positions"
    task_status: str = "/swarm/task_status"
    mission_complete: str = "/swarm/mission_complete"
    cell_override: str = "/swarm/cell_override"
    mode: str = "/swarm/mode"
    peer_cells: str = "/swarm/peer_cells"
    manual_control: str = "/swarm/manual_control"
    mission_ready: str = "/swarm/mission_ready"
    start_mission: str = "/swarm/start_mission"


class TelemetryHub:
    """Build all per-drone and swarm topic names from one contract."""

    def __init__(
        self,
        *,
        drone_id: int,
        camera_topic: str = "",
        depth_topic: str = "",
        camera_info_topic: str = "",
        terrain_range_topic: str = "",
    ) -> None:
        self.drone_id = int(drone_id)
        self.drone_ns = f"drone_{self.drone_id}"
        self.px4_ns = "" if self.drone_id == 0 else f"/px4_{self.drone_id}"
        self.swarm = SwarmTopicContract()
        self.topics = DroneTopicContract(
            drone_id=self.drone_id,
            drone_ns=self.drone_ns,
            px4_ns=self.px4_ns,
            camera_image=_clean_topic(camera_topic) or f"/{self.drone_ns}/camera/image_raw",
            depth_image=_clean_topic(depth_topic) or f"/{self.drone_ns}/depth/image_raw",
            camera_info=_clean_topic(camera_info_topic) or f"/{self.drone_ns}/camera/camera_info",
            terrain_range=_clean_topic(terrain_range_topic)
            or f"/{self.drone_ns}/downward_lidar/scan",
            vehicle_local_position=f"{self.px4_ns}/fmu/out/vehicle_local_position_v1",
            vehicle_status=f"{self.px4_ns}/fmu/out/vehicle_status_v3",
            vehicle_control_mode=f"{self.px4_ns}/fmu/out/vehicle_control_mode",
            vehicle_command_ack=f"{self.px4_ns}/fmu/out/vehicle_command_ack_v1",
            px4_in_offboard_control_mode=f"{self.px4_ns}/fmu/in/offboard_control_mode",
            px4_in_trajectory_setpoint=f"{self.px4_ns}/fmu/in/trajectory_setpoint",
            px4_in_vehicle_command=f"{self.px4_ns}/fmu/in/vehicle_command",
            avoidance_target_cmd=f"/{self.drone_ns}/avoidance/target_cmd",
            avoidance_target_cmd_json=f"/{self.drone_ns}/avoidance/target_cmd_json",
            avoidance_status=f"/{self.drone_ns}/avoidance/status",
            avoidance_status_json=f"/{self.drone_ns}/avoidance/status_json",
            avoidance_events=f"/{self.drone_ns}/avoidance/events",
            avoidance_active=f"/{self.drone_ns}/avoidance/active",
            avoidance_planned_path=f"/{self.drone_ns}/avoidance/planned_path",
            avoidance_actual_path=f"/{self.drone_ns}/avoidance/actual_path",
            obstacles_detected=f"/{self.drone_ns}/obstacles/detected",
            obstacles_clear=f"/{self.drone_ns}/obstacles/clear",
            next_cell=f"/{self.drone_ns}/next_cell",
            rth_target=f"/{self.drone_ns}/rth_target",
            downward_lidar_scan=f"/{self.drone_ns}/downward_lidar/scan",
        )

    @staticmethod
    def for_drone(drone_id: int, **overrides: str) -> "TelemetryHub":
        return TelemetryHub(drone_id=int(drone_id), **overrides)


@dataclass(frozen=True, slots=True)
class TopicOwnership:
    topic: str
    publisher_count: int
    conflict: bool

    def to_payload(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "publisher_count": int(self.publisher_count),
            "conflict": bool(self.conflict),
        }


class Px4InputOwnershipGuard:
    """Detect competing publishers on PX4 `/fmu/in/*` topics owned by this node."""

    def __init__(self, *, topics: list[str], expected_publishers: int = 1) -> None:
        self._topics = list(topics)
        self._expected_publishers = max(1, int(expected_publishers))
        self._latest: dict[str, TopicOwnership] = {
            topic: TopicOwnership(topic=topic, publisher_count=0, conflict=False)
            for topic in self._topics
        }

    @property
    def conflict(self) -> bool:
        return any(item.conflict for item in self._latest.values())

    def update(self, node: Any) -> dict[str, TopicOwnership]:
        latest: dict[str, TopicOwnership] = {}
        for topic in self._topics:
            try:
                infos = node.get_publishers_info_by_topic(topic)
                count = len(infos)
            except Exception:
                count = 0
            latest[topic] = TopicOwnership(
                topic=topic,
                publisher_count=count,
                conflict=count > self._expected_publishers,
            )
        self._latest = latest
        return dict(self._latest)

    def to_payload(self) -> dict[str, Any]:
        return {
            "conflict": self.conflict,
            "topics": {topic: item.to_payload() for topic, item in self._latest.items()},
        }
