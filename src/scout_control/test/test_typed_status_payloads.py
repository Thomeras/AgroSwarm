from scout_control.avoidance.types import AvoidanceStatus, SwarmDroneStatusEvent


def test_avoidance_status_parses_current_runtime_shape() -> None:
    payload = {
        "phase": "LOCAL_REPLAN",
        "planner_mode": "SUBGOAL_PATH",
        "scan_state": "COMPLETE",
        "avoidance_active": True,
        "target_id": "cell_x3_y4",
        "drone_ned": [4.2, -1.0, -5.0],
        "target_ned": [8.0, -2.0],
        "extra_runtime_field": 123,
    }

    status = AvoidanceStatus.from_payload(payload)

    assert status.phase == "LOCAL_REPLAN"
    assert status.state == "LOCAL_REPLAN"
    assert status.result == "ACTIVE"
    assert status.scan_active is False
    assert status.target_ned == (8.0, -2.0)
    assert status.drone_ned == (4.2, -1.0, -5.0)
    assert status.extras["extra_runtime_field"] == 123


def test_avoidance_status_parses_future_result_and_blocked_fields() -> None:
    payload = {
        "state": "BLOCKED",
        "result": "BLOCKED",
        "blocked_severity": "HARD",
        "blocked_reason": "repeated_detour_failure",
        "reassign_recommended": True,
        "scan_active": True,
    }

    status = AvoidanceStatus.from_payload(payload)

    assert status.phase == "BLOCKED"
    assert status.state == "BLOCKED"
    assert status.result == "BLOCKED"
    assert status.blocked_severity == "HARD"
    assert status.blocked_reason == "repeated_detour_failure"
    assert status.reassign_recommended is True
    assert status.scan_active is True


def test_swarm_drone_status_event_keeps_legacy_and_nav_fields() -> None:
    payload = {
        "drone_id": "drone_1",
        "status": "NAV_BLOCKED_HARD",
        "cell_id": "x4_y2",
        "state": "BLOCKED",
        "result": "BLOCKED",
        "blocked_severity": "HARD",
        "reassign_recommended": True,
        "custom_key": "ok",
    }

    event = SwarmDroneStatusEvent.from_payload(payload)

    assert event.drone_id == "drone_1"
    assert event.status == "NAV_BLOCKED_HARD"
    assert event.nav_state == "BLOCKED"
    assert event.nav_result == "BLOCKED"
    assert event.blocked_severity == "HARD"
    assert event.reassign_recommended is True
    assert event.extras["custom_key"] == "ok"
