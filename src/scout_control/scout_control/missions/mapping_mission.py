# flake8: noqa
"""Mapping mission route provider for pre-operational field scans.

This node does not publish PX4 setpoints.  It sends high-level target commands
to ``obstacle_avoidance_runtime`` and waits for runtime completion status.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.avoidance.types import TargetCommand, target_command_to_msg
from scout_control.utils.lawnmower import generate_lawnmower
from scout_control.utils.paths import FIELD_BOUNDARY_FILE, PERIMETER_FILE

try:
    from scout_control_msgs.msg import (
        AvoidanceStatus as ScoutAvoidanceStatusMsg,
        TargetCommand as ScoutTargetCommandMsg,
    )
except ImportError:
    ScoutAvoidanceStatusMsg = None
    ScoutTargetCommandMsg = None

QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MappingPhase(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    MAPPING = auto()
    RTH = auto()
    DONE = auto()


@dataclass(slots=True)
class DroneMappingState:
    drone_id: int
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)
    index: int = 0
    active_target_id: str = ""
    completed: bool = False
    rth_sent: bool = False


class MappingMission(Node):
    """Generate and dispatch per-drone mapping routes through avoidance runtime."""

    def __init__(self) -> None:
        super().__init__("mapping_mission")
        self.declare_parameter("drone_count", 1)
        self.declare_parameter("altitude_m", 8.0)
        self.declare_parameter("line_spacing_m", 4.0)
        self.declare_parameter("side_overlap_pct", 30.0)
        self.declare_parameter("cruise_speed_mps", 2.5)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("tick_hz", 2.0)

        self._drone_count = max(1, int(self.get_parameter("drone_count").value))
        self._altitude_m = float(self.get_parameter("altitude_m").value)
        self._line_spacing_m = float(self.get_parameter("line_spacing_m").value)
        self._side_overlap_pct = float(self.get_parameter("side_overlap_pct").value)
        self._cruise_speed_mps = float(self.get_parameter("cruise_speed_mps").value)
        self._phase = MappingPhase.IDLE
        self._states: dict[int, DroneMappingState] = {}
        self._target_pubs: dict[int, Any] = {}
        self._progress_pub = self.create_publisher(String, "/swarm/mapping_progress", QOS_VOL)
        self._complete_pub = self.create_publisher(String, "/swarm/mapping_complete", QOS_VOL)

        self.create_subscription(String, "/swarm/home_positions", self._on_home_positions, QOS_VOL)
        for drone_id in range(self._drone_count):
            hub = TelemetryHub.for_drone(drone_id)
            msg_type = ScoutTargetCommandMsg or String
            self._target_pubs[drone_id] = self.create_publisher(
                msg_type, hub.topics.avoidance_target_cmd, QOS_VOL
            )
            self.create_subscription(
                ScoutAvoidanceStatusMsg or String,
                hub.topics.avoidance_status,
                lambda msg, did=drone_id: self._on_status(did, msg),
                QOS_STATUS,
            )
            self.create_subscription(
                String,
                hub.topics.avoidance_status_json,
                lambda msg, did=drone_id: self._on_status(did, msg),
                QOS_STATUS,
            )

        self._load_plan()
        if bool(self.get_parameter("auto_start").value):
            self._phase = MappingPhase.TAKEOFF
            self._publish_progress(event="started")
        tick_hz = max(0.2, float(self.get_parameter("tick_hz").value))
        self.create_timer(1.0 / tick_hz, self._tick)

    @property
    def phase(self) -> MappingPhase:
        return self._phase

    def _load_plan(self) -> None:
        boundary_path = (
            FIELD_BOUNDARY_FILE
            if os.path.exists(FIELD_BOUNDARY_FILE)
            else PERIMETER_FILE
        )
        polygon = _load_boundary_polygon(boundary_path)
        routes = generate_lawnmower(
            polygon,
            drone_count=self._drone_count,
            line_spacing_m=self._line_spacing_m,
            altitude_m=self._altitude_m,
            side_overlap_pct=self._side_overlap_pct,
        )
        self._states = {
            drone_id: DroneMappingState(drone_id=drone_id, waypoints=list(routes[drone_id]))
            for drone_id in range(self._drone_count)
        }
        self.get_logger().info(
            f"Loaded mapping routes for {self._drone_count} drones from {boundary_path}"
        )

    def _on_home_positions(self, msg: String) -> None:
        # Home manager remains authoritative; this callback is only retained so
        # the node observes the Phase 2 setup topic requested in the contract.
        try:
            json.loads(msg.data)
        except Exception:
            self.get_logger().warning("Ignoring malformed /swarm/home_positions payload")

    def _on_status(self, drone_id: int, msg: Any) -> None:
        payload = _status_payload(msg)
        if not payload:
            return
        severity = str(payload.get("blocked_severity", "NONE")).upper()
        state = self._states.get(drone_id)
        if state is None:
            return
        if severity == "HARD" and state.active_target_id:
            skipped = state.active_target_id
            state.active_target_id = ""
            state.index += 1
            self._publish_progress(event="waypoint_skipped", drone_id=drone_id, target_id=skipped)
        completed_id = str(payload.get("last_completed_target_id", "")).strip()
        if completed_id and completed_id == state.active_target_id:
            self._publish_progress(event="waypoint_complete", drone_id=drone_id, target_id=completed_id)
            state.active_target_id = ""
            state.index += 1

    def _tick(self) -> None:
        if self._phase in {MappingPhase.IDLE, MappingPhase.DONE}:
            return
        if self._phase in {MappingPhase.TAKEOFF, MappingPhase.MAPPING}:
            self._phase = MappingPhase.MAPPING
            all_done = True
            for state in self._states.values():
                if state.completed:
                    continue
                all_done = False
                if not state.active_target_id:
                    self._dispatch_next_waypoint(state)
            if all_done:
                self._phase = MappingPhase.RTH
                self._publish_progress(event="mapping_routes_complete")
        if self._phase == MappingPhase.RTH:
            all_rth_sent = True
            for state in self._states.values():
                if not state.rth_sent:
                    all_rth_sent = False
                    self._publish_command(state.drone_id, command="return_home", target_id=f"mapping_rth_{state.drone_id}")
                    state.rth_sent = True
            if all_rth_sent:
                self._phase = MappingPhase.DONE
                payload = {"event": "mapping_complete", "stamp_s": time.time()}
                self._complete_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
                self._publish_progress(event="done")

    def _dispatch_next_waypoint(self, state: DroneMappingState) -> None:
        if state.index >= len(state.waypoints):
            state.completed = True
            self._publish_progress(event="drone_route_complete", drone_id=state.drone_id)
            return
        x, y, z = state.waypoints[state.index]
        target_id = f"mapping_d{state.drone_id}_wp{state.index:04d}"
        state.active_target_id = target_id
        self._publish_command(
            state.drone_id,
            command="goto",
            target_id=target_id,
            target_ned=(x, y),
            altitude_m=abs(z),
            name=f"Mapping waypoint {state.index}",
        )
        self._publish_progress(
            event="waypoint_dispatched",
            drone_id=state.drone_id,
            target_id=target_id,
            waypoint=[x, y, z],
        )

    def _publish_command(
        self,
        drone_id: int,
        *,
        command: str,
        target_id: str,
        target_ned: tuple[float, float] | None = None,
        altitude_m: float | None = None,
        name: str = "",
    ) -> None:
        cmd = TargetCommand(
            command=command,
            target_id=target_id,
            cmd_id=target_id,
            name=name or command,
            target_ned=target_ned,
            altitude_m=self._altitude_m if altitude_m is None else altitude_m,
            cruise_speed_mps=self._cruise_speed_mps,
            acceptance_radius_m=1.5,
            clear_radius_m=2.5,
            source="mapping_mission",
            stamp_ms=int(time.time() * 1000),
        )
        pub = self._target_pubs[drone_id]
        if ScoutTargetCommandMsg is not None:
            pub.publish(target_command_to_msg(cmd, ScoutTargetCommandMsg()))
        else:
            pub.publish(String(data=json.dumps(cmd.to_payload(), sort_keys=True)))

    def _publish_progress(self, *, event: str, **extra: Any) -> None:
        payload = {"event": event, "phase": self._phase.name, "stamp_s": time.time(), **extra}
        self._progress_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def _load_boundary_polygon(path: str) -> list[tuple[float, float]]:
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    items = data.get("waypoints_ned") or data.get("vertices_ned") or data.get("polygon_ned")
    if not items:
        raise ValueError(f"No NED polygon vertices found in {path}")
    polygon: list[tuple[float, float]] = []
    for item in items:
        if isinstance(item, dict):
            ned = item.get("ned", item)
            polygon.append((float(ned["x"]), float(ned["y"])))
        else:
            polygon.append((float(item[0]), float(item[1])))
    return polygon


def _status_payload(msg: Any) -> dict[str, Any]:
    raw = getattr(msg, "data", "")
    if raw:
        try:
            payload = json.loads(raw)
            return payload if isinstance(payload, dict) else {}
        except json.JSONDecodeError:
            return {}
    json_payload = getattr(msg, "json_payload", "")
    if json_payload:
        try:
            payload = json.loads(json_payload)
            return payload if isinstance(payload, dict) else {}
        except json.JSONDecodeError:
            pass
    return {
        "blocked_severity": getattr(msg, "blocked_severity", "NONE"),
        "last_completed_target_id": getattr(msg, "last_completed_target_id", ""),
    }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
