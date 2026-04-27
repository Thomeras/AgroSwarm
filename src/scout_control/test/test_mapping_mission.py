# flake8: noqa
"""Small tests for mapping mission status handling helpers."""

import json

import pytest

pytest.importorskip("std_msgs")
from std_msgs.msg import String

from scout_control.missions.mapping_mission import _load_boundary_polygon, _status_payload


def test_status_payload_from_string_json():
    msg = String(data=json.dumps({"last_completed_target_id": "wp1", "blocked_severity": "NONE"}))
    payload = _status_payload(msg)
    assert payload["last_completed_target_id"] == "wp1"
    assert payload["blocked_severity"] == "NONE"


def test_status_payload_from_typed_like_message():
    msg = type("Msg", (), {"blocked_severity": "HARD", "last_completed_target_id": ""})()
    payload = _status_payload(msg)
    assert payload["blocked_severity"] == "HARD"


def test_loads_phase2_field_boundary_schema(tmp_path):
    path = tmp_path / "field_boundary.json"
    path.write_text(json.dumps({
        "capture_mode": "polygon",
        "closed": True,
        "vertices_ned": [
            {"x": 0.0, "y": 0.0, "z": -5.0},
            {"x": 10.0, "y": 0.0, "z": -5.0},
            {"x": 10.0, "y": 10.0, "z": -5.0},
        ],
    }))
    assert _load_boundary_polygon(str(path)) == [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
    ]
