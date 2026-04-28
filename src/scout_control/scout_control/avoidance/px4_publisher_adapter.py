"""PX4 input publisher boundary for obstacle avoidance runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True, slots=True)
class PX4MessageTypes:
    """PX4 message classes used by the publisher adapter."""

    offboard_control_mode: type[Any]
    trajectory_setpoint: type[Any]
    vehicle_command: type[Any]


def default_px4_message_types() -> PX4MessageTypes:
    """Load generated PX4 message classes for runtime use."""

    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

    return PX4MessageTypes(
        offboard_control_mode=OffboardControlMode,
        trajectory_setpoint=TrajectorySetpoint,
        vehicle_command=VehicleCommand,
    )


class PX4PublisherAdapter:
    """Publish offboard heartbeat, setpoint, and command messages owned by runtime."""

    def __init__(
        self,
        *,
        offboard_control_mode_pub: Any,
        trajectory_setpoint_pub: Any,
        vehicle_command_pub: Any,
        message_types: PX4MessageTypes,
    ) -> None:
        self._offboard_control_mode_pub = offboard_control_mode_pub
        self._trajectory_setpoint_pub = trajectory_setpoint_pub
        self._vehicle_command_pub = vehicle_command_pub
        self._types = message_types

    @classmethod
    def create(
        cls,
        node: Any,
        *,
        topics: Mapping[str, str],
        qos_profile: Any,
        message_types: PX4MessageTypes | None = None,
    ) -> "PX4PublisherAdapter":
        types = message_types or default_px4_message_types()
        return cls(
            offboard_control_mode_pub=node.create_publisher(
                types.offboard_control_mode,
                topics["offboard_control_mode"],
                qos_profile,
            ),
            trajectory_setpoint_pub=node.create_publisher(
                types.trajectory_setpoint,
                topics["trajectory_setpoint"],
                qos_profile,
            ),
            vehicle_command_pub=node.create_publisher(
                types.vehicle_command,
                topics["vehicle_command"],
                qos_profile,
            ),
            message_types=types,
        )

    def publish_offboard_heartbeat(self, *, timestamp_us: int) -> None:
        msg = self._types.offboard_control_mode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = int(timestamp_us)
        self._offboard_control_mode_pub.publish(msg)

    def publish_setpoint(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: float,
        current_yaw: float,
        timestamp_us: int,
    ) -> None:
        msg = self._types.trajectory_setpoint()
        msg.position = [x, y, z]
        msg.velocity = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = current_yaw if math.isnan(yaw) else yaw
        msg.yawspeed = float("nan")
        msg.timestamp = int(timestamp_us)
        self._trajectory_setpoint_pub.publish(msg)

    def send_command(
        self,
        *,
        command: int,
        timestamp_us: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
    ) -> None:
        msg = self._types.vehicle_command()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(timestamp_us)
        self._vehicle_command_pub.publish(msg)
