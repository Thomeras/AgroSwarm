from __future__ import annotations

import ast
from pathlib import Path

from scout_control.avoidance.telemetry_hub import TelemetryHub


def test_telemetry_hub_builds_canonical_drone_zero_topics() -> None:
    hub = TelemetryHub(drone_id=0)
    topics = hub.topics

    assert topics.drone_ns == "drone_0"
    assert topics.px4_ns == ""
    assert topics.vehicle_local_position == "/fmu/out/vehicle_local_position_v1"
    assert topics.camera_image == "/drone_0/camera/image_raw"
    assert topics.depth_image == "/drone_0/depth/image_raw"
    assert topics.camera_info == "/drone_0/camera/camera_info"
    assert topics.terrain_range == "/drone_0/downward_lidar/scan"
    assert topics.px4_input_topics == {
        "offboard_control_mode": "/fmu/in/offboard_control_mode",
        "trajectory_setpoint": "/fmu/in/trajectory_setpoint",
        "vehicle_command": "/fmu/in/vehicle_command",
    }
    assert topics.vehicle_status == "/fmu/out/vehicle_status_v3"
    assert topics.vehicle_control_mode == "/fmu/out/vehicle_control_mode"
    assert topics.vehicle_command_ack == "/fmu/out/vehicle_command_ack_v1"
    assert topics.avoidance_target_cmd == "/drone_0/avoidance/target_cmd"
    assert topics.avoidance_status_json == "/drone_0/avoidance/status_json"
    assert topics.next_cell == "/drone_0/next_cell"
    assert hub.swarm.peer_telemetry == "/swarm/peer_telemetry"
    assert hub.swarm.mission_ready == "/swarm/mission_ready"


def test_telemetry_hub_builds_namespaced_drone_topics_and_sensor_overrides() -> None:
    hub = TelemetryHub(
        drone_id=2,
        camera_topic="/custom/rgb",
        depth_topic="/custom/depth",
        camera_info_topic="/custom/info",
        terrain_range_topic="/custom/range",
    )
    topics = hub.topics

    assert topics.drone_ns == "drone_2"
    assert topics.px4_ns == "/px4_2"
    assert topics.vehicle_local_position == "/px4_2/fmu/out/vehicle_local_position_v1"
    assert topics.camera_image == "/custom/rgb"
    assert topics.depth_image == "/custom/depth"
    assert topics.camera_info == "/custom/info"
    assert topics.terrain_range == "/custom/range"
    assert topics.vehicle_status == "/px4_2/fmu/out/vehicle_status_v3"
    assert topics.vehicle_control_mode == "/px4_2/fmu/out/vehicle_control_mode"
    assert topics.vehicle_command_ack == "/px4_2/fmu/out/vehicle_command_ack_v1"
    assert topics.px4_in_vehicle_command == "/px4_2/fmu/in/vehicle_command"
    assert topics.rth_target == "/drone_2/rth_target"


def test_legacy_flight_nodes_are_not_installed_as_console_scripts() -> None:
    setup_path = Path(__file__).resolve().parents[1] / "setup.py"
    tree = ast.parse(setup_path.read_text(encoding="utf-8"))
    console_scripts: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.keyword) or node.arg != "entry_points":
            continue
        if not isinstance(node.value, ast.Dict):
            continue
        for key, value in zip(node.value.keys, node.value.values):
            if isinstance(key, ast.Constant) and key.value == "console_scripts":
                console_scripts.extend(
                    item.value for item in value.elts if isinstance(item, ast.Constant)
                )

    installed_names = {item.split("=", 1)[0].strip() for item in console_scripts}
    assert "field_setup_tool" in installed_names
    assert not any(name.startswith("legacy_") for name in installed_names)
    assert "obstacle_avoidance_test_mission" not in installed_names
