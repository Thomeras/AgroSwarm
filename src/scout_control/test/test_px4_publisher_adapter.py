from __future__ import annotations

import math

from scout_control.avoidance.px4_publisher_adapter import (
    PX4MessageTypes,
    PX4PublisherAdapter,
)


class _OffboardControlMode:
    pass


class _TrajectorySetpoint:
    pass


class _VehicleCommand:
    pass


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


def _adapter() -> tuple[PX4PublisherAdapter, _Publisher, _Publisher, _Publisher]:
    offboard_pub = _Publisher()
    setpoint_pub = _Publisher()
    command_pub = _Publisher()
    adapter = PX4PublisherAdapter(
        offboard_control_mode_pub=offboard_pub,
        trajectory_setpoint_pub=setpoint_pub,
        vehicle_command_pub=command_pub,
        message_types=PX4MessageTypes(
            offboard_control_mode=_OffboardControlMode,
            trajectory_setpoint=_TrajectorySetpoint,
            vehicle_command=_VehicleCommand,
        ),
    )
    return adapter, offboard_pub, setpoint_pub, command_pub


def test_publish_offboard_heartbeat_matches_runtime_wire_fields() -> None:
    adapter, offboard_pub, _, _ = _adapter()

    adapter.publish_offboard_heartbeat(timestamp_us=12345)

    msg = offboard_pub.messages[-1]
    assert msg.position is True
    assert msg.velocity is False
    assert msg.timestamp == 12345


def test_publish_setpoint_uses_current_yaw_when_yaw_is_nan() -> None:
    adapter, _, setpoint_pub, _ = _adapter()

    adapter.publish_setpoint(
        x=1.0,
        y=2.0,
        z=-5.0,
        yaw=float("nan"),
        current_yaw=0.75,
        timestamp_us=45678,
    )

    msg = setpoint_pub.messages[-1]
    assert msg.position == [1.0, 2.0, -5.0]
    assert all(math.isnan(value) for value in msg.velocity)
    assert all(math.isnan(value) for value in msg.acceleration)
    assert all(math.isnan(value) for value in msg.jerk)
    assert msg.yaw == 0.75
    assert math.isnan(msg.yawspeed)
    assert msg.timestamp == 45678


def test_send_command_matches_px4_vehicle_command_defaults() -> None:
    adapter, _, _, command_pub = _adapter()

    adapter.send_command(
        command=176,
        param1=1.0,
        param2=6.0,
        param3=0.0,
        timestamp_us=789,
    )

    msg = command_pub.messages[-1]
    assert msg.command == 176
    assert msg.param1 == 1.0
    assert msg.param2 == 6.0
    assert msg.param3 == 0.0
    assert msg.target_system == 1
    assert msg.target_component == 1
    assert msg.source_system == 1
    assert msg.source_component == 1
    assert msg.from_external is True
    assert msg.timestamp == 789
