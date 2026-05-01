"""Standalone E2E setup-flow test for field setup orchestration.

The test drives FieldSetupCoordinator through its ROS message callbacks while
redirecting generated artifacts to pytest's tmp_path.
"""

from __future__ import annotations

import json

import pytest  # noqa: I201
import rclpy  # noqa: I201
from scout_control.core import field_setup_coordinator as fsc  # noqa: I201
from std_msgs.msg import String  # noqa: I201


class CapturingPublisher:
    """Small publisher double that records JSON string payloads."""

    def __init__(self) -> None:
        """Create an empty capture buffer."""
        self.messages: list[str] = []

    def publish(self, msg: String) -> None:
        """Record a published std_msgs/String payload."""
        self.messages.append(msg.data)


def _msg(payload: dict) -> String:
    msg = String()
    msg.data = json.dumps(payload)
    return msg


@pytest.fixture
def rclpy_context(tmp_path, monkeypatch):
    """Initialise rclpy with logs redirected into pytest tmp_path."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    rclpy.init(args=None)
    try:
        yield
    finally:
        rclpy.try_shutdown()


def test_e2e_pad_corner_grid_and_rth_flow(
    tmp_path,
    monkeypatch,
    rclpy_context,
) -> None:
    """Pads -> boundary capture -> grid generation -> RTH request."""
    monkeypatch.setattr(fsc, 'PERIMETERS_DIR', str(tmp_path))
    monkeypatch.setattr(fsc, 'GRID_FILE', str(tmp_path / 'field_grid.json'))
    monkeypatch.setattr(
        fsc,
        'HOME_POS_FILE',
        str(tmp_path / 'home_positions.json'),
    )
    monkeypatch.setattr(
        fsc,
        'BOUNDARY_FILE',
        str(tmp_path / 'field_boundary.json'),
    )

    node = fsc.FieldSetupCoordinator()
    status_pub = CapturingPublisher()
    complete_pub = CapturingPublisher()
    rth_pub = CapturingPublisher()
    ready_pub = CapturingPublisher()
    node._status_pub = status_pub
    node._complete_pub = complete_pub
    node._rth_pub = rth_pub
    node._ready_pub = ready_pub

    try:
        node._pad_assign_cb(_msg({
            'drone_id': 'drone_0',
            'pad_id': 'pad_0',
            'x': 10.0,
            'y': -8.0,
            'z': -0.5,
        }))
        node._pad_assign_cb(_msg({
            'drone_id': 'drone_1',
            'pad_id': 'pad_1',
            'x': 40.0,
            'y': -8.0,
            'z': -0.5,
        }))

        # TODO(core): product docs call this setup state MAP_FIELD; the current
        # implementation enum is CAPTURE_BOUNDARY.
        assert node._state == fsc.SetupState.CAPTURE_BOUNDARY
        assert (tmp_path / 'home_positions.json').exists()

        for corner, x, y in (
            ('NE', 20.0, 20.0),
            ('NW', 20.0, 0.0),
            ('SE', 0.0, 20.0),
            ('SW', 0.0, 0.0),
        ):
            node._corner_cb(_msg({
                'corner': corner,
                'ned': {'x': x, 'y': y, 'z': -5.0},
            }))

        assert any(
            json.loads(raw)['state'] == 'GENERATE_GRID'
            for raw in status_pub.messages
        )
        assert node._state == fsc.SetupState.WAITING_FOR_LANDING

        grid_path = tmp_path / 'field_grid.json'
        assert grid_path.exists()
        grid = json.loads(grid_path.read_text(encoding='utf-8'))
        assert grid['capture_mode'] == 'corners'
        assert grid['cell_size_m'] == pytest.approx(5.0)
        assert len(grid['cells']) == 16

        assert complete_pub.messages
        complete = json.loads(complete_pub.messages[-1])
        assert complete['status'] == 'ready'
        assert complete['cells'] == 16

        assert [json.loads(raw) for raw in rth_pub.messages] == [
            {'drone_id': 'drone_0', 'reason': 'setup_complete'},
            {'drone_id': 'drone_1', 'reason': 'setup_complete'},
        ]
        assert not ready_pub.messages
    finally:
        node.destroy_node()


def test_existing_setup_resumes_ready_for_mission(
    tmp_path,
    monkeypatch,
    rclpy_context,
) -> None:
    """Existing pads + grid should skip the 20-minute setup path on restart."""
    monkeypatch.setattr(fsc, 'PERIMETERS_DIR', str(tmp_path))
    monkeypatch.setattr(fsc, 'GRID_FILE', str(tmp_path / 'field_grid.json'))
    monkeypatch.setattr(
        fsc,
        'HOME_POS_FILE',
        str(tmp_path / 'home_positions.json'),
    )
    monkeypatch.setattr(
        fsc,
        'BOUNDARY_FILE',
        str(tmp_path / 'field_boundary.json'),
    )

    (tmp_path / 'home_positions.json').write_text(json.dumps({
        'home_positions': [
            {
                'pad_id': 'pad_0',
                'drone_id': 'drone_0',
                'ned': {'x': 10.0, 'y': -8.0, 'z': -0.5},
                'status': 'available',
            },
            {
                'pad_id': 'pad_1',
                'drone_id': 'drone_1',
                'ned': {'x': 40.0, 'y': -8.0, 'z': -0.5},
                'status': 'available',
            },
        ],
    }), encoding='utf-8')
    (tmp_path / 'field_grid.json').write_text(json.dumps({
        'cell_size_m': 5.0,
        'cols': 1,
        'rows': 1,
        'capture_mode': 'polygon',
        'cells': [{'id': 'x0_y0', 'x': 12.5, 'y': -5.5, 'status': 'unvisited'}],
    }), encoding='utf-8')
    (tmp_path / 'field_boundary.json').write_text(json.dumps({
        'vertices_ned': [
            {'x': 10.0, 'y': -8.0, 'z': -5.0},
            {'x': 15.0, 'y': -8.0, 'z': -5.0},
            {'x': 15.0, 'y': -3.0, 'z': -5.0},
        ],
        'closed': True,
        'capture_mode': 'polygon',
    }), encoding='utf-8')

    node = fsc.FieldSetupCoordinator()
    ready_pub = CapturingPublisher()
    node._ready_pub = ready_pub

    try:
        assert node._state == fsc.SetupState.READY_FOR_MISSION
        assert sorted(node._pads) == ['pad_0', 'pad_1']
        assert node._landed_drones == {'drone_0', 'drone_1'}

        node._mission_confirm_cb(_msg({'source': 'test'}))

        assert ready_pub.messages
        assert json.loads(ready_pub.messages[-1]) == {
            'drones': ['drone_0', 'drone_1'],
        }
    finally:
        node.destroy_node()
