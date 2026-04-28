import json
from types import SimpleNamespace

import pytest

from scout_control.avoidance.types import (
    AvoidanceStatus,
    FieldSetupComplete,
    MissionReadySignal,
    PadAssignment,
    ReturnHomeRequest,
    SwarmDroneStatusEvent,
    SwarmTaskStatus,
    TargetCommand,
    avoidance_status_from_msg,
    avoidance_status_to_msg,
    payload_from_string_msg,
    payload_to_string_msg,
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


def test_target_command_from_typed_message_rejects_negative_altitude_m() -> None:
    msg = SimpleNamespace(
        command="goto",
        target_id="bad_altitude",
        cmd_id="bad_altitude",
        route_id="",
        name="goto",
        frame="local_ned",
        target_ned=[3.0, 0.0],
        altitude_mode="relative_ned",
        altitude_m=-5.0,
        cruise_speed_mps=2.0,
        acceptance_radius_m=1.5,
        clear_radius_m=2.5,
        allow_replan=True,
        max_blocked_time_s=30.0,
        priority="mission",
        source="test",
        stamp_ms=1,
        json_payload="",
    )

    with pytest.raises(ValueError, match="altitude_m must be >= 0.0"):
        target_command_from_msg(msg)


def test_target_command_typed_adapter_normalizes_envelope_aliases() -> None:
    msg = SimpleNamespace(
        command="",
        target_id="",
        target_ned=[],
        json_payload=json.dumps(
            {
                "target_cmd": {
                    "action": "goto",
                    "cell_id": "cell_x7_y8",
                    "target_xy": [7.0, -8.0],
                    "target_altitude_m": 9.5,
                    "speed_mps": 1.25,
                    "radius_m": 3.0,
                }
            }
        ),
    )

    command = target_command_from_msg(msg)

    assert command.command == "goto"
    assert command.target_id == "cell_x7_y8"
    assert command.target_ned == (7.0, -8.0)
    assert command.altitude_m == 9.5
    assert command.cruise_speed_mps == 1.25
    assert command.clear_radius_m == 3.0


def test_target_command_json_string_fallback_uses_same_adapter_contract() -> None:
    msg = SimpleNamespace(
        data=json.dumps(
            {
                "payload": {
                    "command": {"op": "hold", "id": "pause_1"},
                    "x": 1.0,
                    "y": 2.0,
                }
            }
        )
    )

    command = target_command_from_msg(msg)

    assert command.command == "hold"
    assert command.target_id == "pause_1"
    assert command.target_ned == (1.0, 2.0)


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


@pytest.mark.parametrize(
    ("contract_cls", "payload", "expected"),
    [
        (
            SwarmDroneStatusEvent,
            {
                "drone_id": "drone_0",
                "status": "CELL_COMPLETE",
                "cell_id": "cell_x1_y2",
                "blocked_severity": "NONE",
            },
            {"drone_id": "drone_0", "status": "CELL_COMPLETE"},
        ),
        (
            SwarmTaskStatus,
            {
                "status": "running",
                "event": "progress",
                "mission_id": "mission_1",
                "total_cells": 24,
                "completed_cells": 12,
            },
            {"status": "running", "completed_cells": 12},
        ),
        (
            PadAssignment,
            {
                "drone_id": "drone_1",
                "pad_id": "pad_1",
                "pad_ned": [0.0, -5.0, 0.0],
            },
            {"drone_id": "drone_1", "pad_id": "pad_1"},
        ),
        (
            FieldSetupComplete,
            {
                "ready": True,
                "field_id": "field_a",
                "grid_file": "perimeters/field_grid.json",
                "drone_count": 2,
            },
            {"ready": True, "drone_count": 2},
        ),
        (
            ReturnHomeRequest,
            {
                "drone_id": "drone_0",
                "cmd_id": "rth_1",
                "reason": "mission_complete",
            },
            {"drone_id": "drone_0", "request_id": "rth_1"},
        ),
        (
            MissionReadySignal,
            {
                "ready": True,
                "mission_id": "mission_1",
                "source": "field_setup_coordinator",
                "drone_count": 2,
            },
            {"ready": True, "mission_id": "mission_1"},
        ),
    ],
)
def test_core_string_contracts_have_typed_json_compatible_helpers(
    contract_cls, payload, expected
) -> None:
    msg = payload_to_string_msg(payload)
    parsed_payload = payload_from_string_msg(msg)
    parsed_contract = contract_cls.from_payload(parsed_payload)
    round_trip = parsed_contract.to_payload()

    for key, value in expected.items():
        assert round_trip[key] == value
