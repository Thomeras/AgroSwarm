import json
from types import SimpleNamespace

from scout_control.avoidance.ros_io_adapter import RosIOAdapter


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _string(**kwargs):
    return SimpleNamespace(**kwargs)


def _bool(**kwargs):
    return SimpleNamespace(**kwargs)


def test_ros_io_adapter_publishes_runtime_event_with_stamp_and_legacy_mirror() -> None:
    adapter = RosIOAdapter(string_type=_string, bool_type=_bool, clock=lambda: 42.25)
    pub = _Publisher()
    legacy = _Publisher()

    payload = adapter.publish_runtime_event(
        publisher=pub,
        legacy_publisher=legacy,
        payload={"event": "phase_transition"},
    )

    assert payload["stamp_s"] == 42.25
    assert json.loads(pub.messages[0].data)["event"] == "phase_transition"
    assert legacy.messages[0] is pub.messages[0]


def test_ros_io_adapter_publishes_json_status_when_no_typed_msg_is_available() -> None:
    adapter = RosIOAdapter(string_type=_string, bool_type=_bool)
    pub = _Publisher()
    json_pub = _Publisher()

    adapter.publish_avoidance_status(
        status_publisher=pub,
        status_json_publisher=json_pub,
        payload={"phase": "IDLE", "state": "IDLE", "target_id": ""},
        drone_id="drone_0",
    )

    assert json.loads(pub.messages[0].data)["phase"] == "IDLE"
    assert json_pub.messages == []


def test_ros_io_adapter_publishes_bool_with_legacy_mirror() -> None:
    adapter = RosIOAdapter(string_type=_string, bool_type=_bool)
    pub = _Publisher()
    legacy = _Publisher()

    adapter.publish_bool(publisher=pub, legacy_publisher=legacy, value=True)

    assert pub.messages[0].data is True
    assert legacy.messages[0] is pub.messages[0]
