from types import SimpleNamespace

from scout_control.avoidance.types import (
    AvoidanceStatus,
    TargetCommand,
    avoidance_status_from_msg,
    avoidance_status_to_msg,
    readiness_msg_to_payload,
    readiness_payload_to_msg,
    target_command_from_msg,
    target_command_to_msg,
)


def _avoidance_msg() -> SimpleNamespace:
    return SimpleNamespace(
        readiness=SimpleNamespace(),
        health=SimpleNamespace(readiness=SimpleNamespace()),
    )


def test_target_command_round_trips_through_typed_message_shape() -> None:
    command = TargetCommand.from_payload(
        {
            "command": "goto",
            "target_id": "cell_x1_y2",
            "target_ned": [4.5, -2.0],
            "altitude_m": 6.0,
            "cruise_speed_mps": 1.8,
            "source": "swarm_agent",
        }
    )

    msg = target_command_to_msg(command)
    parsed = target_command_from_msg(msg)

    assert msg.command == "goto"
    assert msg.target_ned == [4.5, -2.0]
    assert parsed.target_id == "cell_x1_y2"
    assert parsed.target_ned == (4.5, -2.0)
    assert parsed.source == "swarm_agent"


def test_readiness_message_keeps_agent_a_payload_shape() -> None:
    payload = {
        "ready": True,
        "navigation_allowed": False,
        "setpoint_publish_allowed": True,
        "reason": "depth_stale",
        "severity": "soft",
        "depth_ready": False,
        "depth_age_s": 1.7,
        "owner_conflict": False,
        "pose": {"valid": True, "age_s": 0.1},
    }

    msg = readiness_payload_to_msg(payload)
    parsed = readiness_msg_to_payload(msg)

    assert msg.ready is True
    assert msg.pose_valid is True
    assert parsed["reason"] == "depth_stale"
    assert parsed["pose"]["valid"] is True


def test_readiness_message_reads_runtime_nested_depth_payload() -> None:
    payload = {
        "ready": True,
        "navigation_allowed": True,
        "setpoint_publish_allowed": True,
        "reason": "ok",
        "severity": "none",
        "owner_conflict": False,
        "pose": {"valid": True, "age_s": 0.04},
        "depth": {"ready": True, "reason": "ok", "age_s": 0.12},
    }

    msg = readiness_payload_to_msg(payload)

    assert msg.depth_ready is True
    assert msg.depth_age_s == 0.12


def test_avoidance_status_typed_message_preserves_bridge_payload() -> None:
    payload = {
        "phase": "LOCAL_REPLAN",
        "state": "LOCAL_REPLAN",
        "result": "ACTIVE",
        "command": "goto",
        "target_id": "cell_x3_y4",
        "target_name": "cell_x3_y4",
        "target_ned": [8.0, -2.0],
        "drone_ned": [4.2, -1.0, -5.0],
        "navigator_ready": False,
        "runtime_ready": True,
        "readiness": {"ready": True, "reason": "ok", "pose": {"valid": True}},
        "health": {"ready": True, "reason": "ok", "pose": {"valid": True}},
        "px4_input_ownership": {"conflict": False},
        "altitude_policy": {"mode": "FixedNED"},
        "free_directions": ["left", "center"],
        "last_completed_target_id": "cell_x3_y3",
    }

    msg = avoidance_status_to_msg(
        AvoidanceStatus.from_payload(payload),
        _avoidance_msg(),
        drone_id="drone_1",
    )
    parsed = avoidance_status_from_msg(msg).to_payload()

    assert msg.drone_id == "drone_1"
    assert msg.phase == "LOCAL_REPLAN"
    assert msg.readiness.ready is True
    assert msg.free_directions == ["left", "center"]
    assert parsed["target_id"] == "cell_x3_y4"
    assert parsed["px4_input_ownership"] == {"conflict": False}
