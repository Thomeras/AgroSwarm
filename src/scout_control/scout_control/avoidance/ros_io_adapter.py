"""ROS publisher helper boundary for runtime JSON/status outputs."""

from __future__ import annotations

import json
import time
from typing import Any, Mapping

from scout_control.avoidance.types import AvoidanceStatus, avoidance_status_to_msg


class RosIOAdapter:
    """Publish runtime ROS outputs without embedding serialization in phase logic."""

    def __init__(
        self,
        *,
        string_type: type[Any],
        bool_type: type[Any],
        avoidance_status_type: type[Any] | None = None,
        clock: Any = time.time,
    ) -> None:
        self._string_type = string_type
        self._bool_type = bool_type
        self._avoidance_status_type = avoidance_status_type
        self._clock = clock

    def json_msg(self, payload: Mapping[str, Any]) -> Any:
        return self._string_type(data=json.dumps(dict(payload), ensure_ascii=True))

    def bool_msg(self, value: bool) -> Any:
        return self._bool_type(data=bool(value))

    def publish_runtime_event(
        self,
        *,
        publisher: Any,
        payload: Mapping[str, Any],
        legacy_publisher: Any | None = None,
    ) -> dict[str, Any]:
        safe_payload = dict(payload)
        safe_payload.setdefault("stamp_s", round(float(self._clock()), 3))
        msg = self.json_msg(safe_payload)
        publisher.publish(msg)
        if legacy_publisher is not None:
            legacy_publisher.publish(msg)
        return safe_payload

    def publish_avoidance_status(
        self,
        *,
        status_publisher: Any,
        status_json_publisher: Any,
        payload: Mapping[str, Any],
        drone_id: str,
        legacy_publisher: Any | None = None,
    ) -> None:
        json_msg = self.json_msg(payload)
        if self._avoidance_status_type is not None:
            typed_msg = avoidance_status_to_msg(
                AvoidanceStatus.from_payload(payload),
                self._avoidance_status_type(),
                drone_id=drone_id,
            )
            status_publisher.publish(typed_msg)
            status_json_publisher.publish(json_msg)
        else:
            status_publisher.publish(json_msg)
        if legacy_publisher is not None:
            legacy_publisher.publish(json_msg)

    def publish_bool(
        self,
        *,
        publisher: Any,
        value: bool,
        legacy_publisher: Any | None = None,
    ) -> None:
        msg = self.bool_msg(value)
        publisher.publish(msg)
        if legacy_publisher is not None:
            legacy_publisher.publish(msg)
