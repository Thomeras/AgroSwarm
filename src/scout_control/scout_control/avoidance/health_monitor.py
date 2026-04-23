"""Runtime health/readiness checks for obstacle avoidance control."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class HealthConfig:
    pose_stale_after_s: float = 0.5
    depth_stale_after_s: float = 1.0
    xy_reset_quarantine_s: float = 0.5
    require_depth_for_navigation: bool = True


@dataclass(slots=True)
class PoseHealth:
    valid: bool
    reason: str
    stamp_s: float = 0.0
    age_s: float | None = None
    xy_valid: bool = False
    heading_good_for_control: bool = False
    dead_reckoning: bool = False
    xy_reset_counter: int | None = None


@dataclass(slots=True)
class RuntimeReadiness:
    ready: bool
    severity: str
    reason: str
    pose: PoseHealth
    depth_ready: bool
    depth_age_s: float | None
    depth_reason: str
    setpoint_publish_allowed: bool
    navigation_allowed: bool
    details: dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "ready": bool(self.ready),
            "severity": self.severity,
            "reason": self.reason,
            "setpoint_publish_allowed": bool(self.setpoint_publish_allowed),
            "navigation_allowed": bool(self.navigation_allowed),
            "pose": {
                "valid": bool(self.pose.valid),
                "reason": self.pose.reason,
                "age_s": None if self.pose.age_s is None else round(float(self.pose.age_s), 3),
                "xy_valid": bool(self.pose.xy_valid),
                "heading_good_for_control": bool(self.pose.heading_good_for_control),
                "dead_reckoning": bool(self.pose.dead_reckoning),
                "xy_reset_counter": self.pose.xy_reset_counter,
            },
            "depth": {
                "ready": bool(self.depth_ready),
                "reason": self.depth_reason,
                "age_s": None if self.depth_age_s is None else round(float(self.depth_age_s), 3),
            },
            **self.details,
        }


class RuntimeHealthMonitor:
    """Stateful freshness and validity monitor for runtime control inputs."""

    def __init__(self, config: HealthConfig | None = None) -> None:
        self.config = config or HealthConfig()
        self._last_pose_msg_s = 0.0
        self._last_valid_pose_s = 0.0
        self._last_pose = PoseHealth(valid=False, reason="pose_not_received")
        self._last_xy_reset_counter: int | None = None
        self._xy_reset_quarantine_until_s = 0.0
        self._last_depth_s = 0.0
        self._last_depth_valid = False
        self._last_depth_reason = "depth_not_received"

    @property
    def last_pose(self) -> PoseHealth:
        return self._last_pose

    def update_pose_message(self, msg: Any, *, now_s: float) -> PoseHealth:
        self._last_pose_msg_s = float(now_s)
        xy_reset_counter = self._optional_int(getattr(msg, "xy_reset_counter", None))
        xy_valid = bool(getattr(msg, "xy_valid", False))
        heading_good = bool(getattr(msg, "heading_good_for_control", False))
        dead_reckoning = bool(getattr(msg, "dead_reckoning", False))

        reset_changed = (
            self._last_xy_reset_counter is not None
            and xy_reset_counter is not None
            and xy_reset_counter != self._last_xy_reset_counter
        )
        self._last_xy_reset_counter = xy_reset_counter
        if reset_changed:
            self._xy_reset_quarantine_until_s = (
                float(now_s) + max(0.0, self.config.xy_reset_quarantine_s)
            )

        reason = "ok"
        finite_pose = all(
            math.isfinite(float(getattr(msg, name, float("nan"))))
            for name in ("x", "y", "z", "heading")
        )
        if not xy_valid:
            reason = "xy_invalid"
        elif not heading_good:
            reason = "heading_not_good_for_control"
        elif dead_reckoning:
            reason = "dead_reckoning"
        elif not finite_pose:
            reason = "non_finite_pose"
        elif float(now_s) < self._xy_reset_quarantine_until_s:
            reason = "xy_reset_quarantine"

        valid = reason == "ok"
        if valid:
            self._last_valid_pose_s = float(now_s)
        self._last_pose = PoseHealth(
            valid=valid,
            reason=reason,
            stamp_s=float(now_s),
            age_s=0.0,
            xy_valid=xy_valid,
            heading_good_for_control=heading_good,
            dead_reckoning=dead_reckoning,
            xy_reset_counter=xy_reset_counter,
        )
        return self._last_pose

    def update_depth_frame(self, *, now_s: float, valid_samples: int) -> None:
        self._last_depth_s = float(now_s)
        self._last_depth_valid = int(valid_samples) > 0
        self._last_depth_reason = "ok" if self._last_depth_valid else "depth_has_no_valid_samples"

    def evaluate(
        self,
        *,
        now_s: float,
        command_active: bool,
        owner_conflict: bool = False,
    ) -> RuntimeReadiness:
        pose = self._pose_with_freshness(now_s=now_s)
        depth_ready, depth_age_s, depth_reason = self._depth_readiness(now_s=now_s)
        navigation_depth_ready = (
            depth_ready or not (command_active and self.config.require_depth_for_navigation)
        )

        ready = pose.valid and navigation_depth_ready and not owner_conflict
        reason = "ok"
        severity = "none"
        if owner_conflict:
            reason = "px4_input_publisher_conflict"
            severity = "hard"
        elif not pose.valid:
            reason = pose.reason
            severity = "hard"
        elif not navigation_depth_ready:
            reason = depth_reason
            severity = "soft"

        return RuntimeReadiness(
            ready=ready,
            severity=severity,
            reason=reason,
            pose=pose,
            depth_ready=depth_ready,
            depth_age_s=depth_age_s,
            depth_reason=depth_reason,
            setpoint_publish_allowed=pose.valid and not owner_conflict,
            navigation_allowed=ready,
            details={"owner_conflict": bool(owner_conflict)},
        )

    def _pose_with_freshness(self, *, now_s: float) -> PoseHealth:
        if self._last_pose_msg_s <= 0.0:
            return PoseHealth(valid=False, reason="pose_not_received")
        age_s = max(0.0, float(now_s) - self._last_pose_msg_s)
        if age_s > self.config.pose_stale_after_s:
            return PoseHealth(
                valid=False,
                reason="pose_stale",
                stamp_s=self._last_pose.stamp_s,
                age_s=age_s,
                xy_valid=self._last_pose.xy_valid,
                heading_good_for_control=self._last_pose.heading_good_for_control,
                dead_reckoning=self._last_pose.dead_reckoning,
                xy_reset_counter=self._last_pose.xy_reset_counter,
            )
        pose = self._last_pose
        pose.age_s = age_s
        return pose

    def _depth_readiness(self, *, now_s: float) -> tuple[bool, float | None, str]:
        if self._last_depth_s <= 0.0:
            return False, None, "depth_not_received"
        age_s = max(0.0, float(now_s) - self._last_depth_s)
        if age_s > self.config.depth_stale_after_s:
            return False, age_s, "depth_stale"
        if not self._last_depth_valid:
            return False, age_s, self._last_depth_reason
        return True, age_s, "ok"

    @staticmethod
    def _optional_int(value: Any) -> int | None:
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None
