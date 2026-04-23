from types import SimpleNamespace

from scout_control.avoidance.health_monitor import HealthConfig, RuntimeHealthMonitor


def _pose_msg(**overrides):
    data = {
        "xy_valid": True,
        "heading_good_for_control": True,
        "dead_reckoning": False,
        "xy_reset_counter": 0,
        "x": 1.0,
        "y": 2.0,
        "z": -5.0,
        "heading": 0.25,
    }
    data.update(overrides)
    return SimpleNamespace(**data)


def test_stale_pose_revokes_readiness_and_setpoint_publish_gate() -> None:
    monitor = RuntimeHealthMonitor(
        HealthConfig(
            pose_stale_after_s=0.5,
            depth_stale_after_s=2.0,
            require_depth_for_navigation=True,
        )
    )
    monitor.update_pose_message(_pose_msg(), now_s=10.0)
    monitor.update_depth_frame(now_s=10.0, valid_samples=12)

    ready = monitor.evaluate(now_s=10.2, command_active=True)
    stale = monitor.evaluate(now_s=10.7, command_active=True)

    assert ready.navigation_allowed is True
    assert ready.setpoint_publish_allowed is True
    assert stale.navigation_allowed is False
    assert stale.setpoint_publish_allowed is False
    assert stale.reason == "pose_stale"
    assert stale.severity == "hard"


def test_invalid_heading_and_dead_reckoning_block_runtime_readiness() -> None:
    monitor = RuntimeHealthMonitor(HealthConfig(require_depth_for_navigation=False))

    heading = monitor.update_pose_message(
        _pose_msg(heading_good_for_control=False),
        now_s=20.0,
    )
    heading_readiness = monitor.evaluate(now_s=20.0, command_active=True)
    dead_reckoning = monitor.update_pose_message(
        _pose_msg(heading_good_for_control=True, dead_reckoning=True),
        now_s=20.1,
    )
    dr_readiness = monitor.evaluate(now_s=20.1, command_active=True)

    assert heading.valid is False
    assert heading.reason == "heading_not_good_for_control"
    assert heading_readiness.setpoint_publish_allowed is False
    assert dead_reckoning.valid is False
    assert dead_reckoning.reason == "dead_reckoning"
    assert dr_readiness.navigation_allowed is False


def test_xy_reset_quarantine_blocks_current_setpoint_publish_only_until_stable() -> None:
    monitor = RuntimeHealthMonitor(
        HealthConfig(
            pose_stale_after_s=1.0,
            xy_reset_quarantine_s=0.25,
            require_depth_for_navigation=False,
        )
    )
    monitor.update_pose_message(_pose_msg(xy_reset_counter=1), now_s=30.0)
    reset_pose = monitor.update_pose_message(_pose_msg(xy_reset_counter=2), now_s=30.1)
    reset_readiness = monitor.evaluate(now_s=30.1, command_active=True)
    stable_pose = monitor.update_pose_message(_pose_msg(xy_reset_counter=2), now_s=30.4)
    stable_readiness = monitor.evaluate(now_s=30.4, command_active=True)

    assert reset_pose.valid is False
    assert reset_pose.reason == "xy_reset_quarantine"
    assert reset_readiness.setpoint_publish_allowed is False
    assert stable_pose.valid is True
    assert stable_readiness.setpoint_publish_allowed is True


def test_depth_stale_blocks_navigation_but_allows_pose_based_hold_setpoint() -> None:
    monitor = RuntimeHealthMonitor(
        HealthConfig(
            pose_stale_after_s=2.0,
            depth_stale_after_s=0.5,
            require_depth_for_navigation=True,
        )
    )
    monitor.update_pose_message(_pose_msg(), now_s=40.0)
    monitor.update_depth_frame(now_s=40.0, valid_samples=4)

    readiness = monitor.evaluate(now_s=40.7, command_active=True)

    assert readiness.pose.valid is True
    assert readiness.depth_ready is False
    assert readiness.reason == "depth_stale"
    assert readiness.severity == "soft"
    assert readiness.navigation_allowed is False
    assert readiness.setpoint_publish_allowed is True
