"""Shared datatypes for the avoidance runtime and planner helpers."""

from __future__ import annotations

import math
import time
import json
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Mapping

import numpy as np


class LocalMapperState(Enum):
    """Lifecycle state of the rolling local mapper."""

    EMPTY = auto()
    TRACKING = auto()
    STALE_INPUT = auto()


class ScanState(Enum):
    """Lifecycle state of the local scan manager."""

    IDLE = auto()
    PREPARE_HOVER = auto()
    SPIN_CAPTURE = auto()
    PROCESS = auto()
    COMPLETE = auto()
    FAILED = auto()


def _xy_tuple(value: Any) -> tuple[float, float] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError("Expected [x, y] sequence")
    return (float(value[0]), float(value[1]))


def _xyz_tuple(value: Any) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError("Expected [x, y, z] sequence")
    return (float(value[0]), float(value[1]), float(value[2]))


def _xy_list(values: list[tuple[float, float]]) -> list[list[float]]:
    return [[float(x), float(y)] for x, y in values]


def _safe_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(dict(payload), ensure_ascii=True, separators=(",", ":"))


def _payload_from_msg(msg: Any) -> dict[str, Any]:
    raw = getattr(msg, "json_payload", "")
    if not raw:
        raw = getattr(msg, "data", "")
    if raw:
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                return payload
        except json.JSONDecodeError:
            pass
    return {}


def payload_to_string_msg(payload: Mapping[str, Any], msg: Any | None = None) -> Any:
    """Fill a std_msgs/String-like message with canonical JSON payload."""

    if msg is None:
        msg = type("StringMsg", (), {})()
    msg.data = _safe_json(payload)
    return msg


def payload_from_string_msg(msg: Any) -> dict[str, Any]:
    """Parse canonical JSON from a std_msgs/String-like compatibility message."""

    return _payload_from_msg(msg)


def _finite_or_default(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def normalize_target_command_payload(payload: Any) -> dict[str, Any]:
    """Normalize target command aliases/envelopes used by JSON and typed adapters."""

    if not isinstance(payload, Mapping):
        raise ValueError("target command payload must be an object")
    normalized: dict[str, Any] = dict(payload)

    for envelope_key in ("payload", "target_cmd", "command_payload"):
        nested = normalized.get(envelope_key)
        if isinstance(nested, Mapping):
            normalized.update(nested)

    nested_command = normalized.get("command")
    if isinstance(nested_command, Mapping):
        normalized.pop("command", None)
        normalized.update(nested_command)

    command_value = normalized.get("command")
    if not isinstance(command_value, str) or not command_value.strip():
        for alias in ("cmd", "action", "op", "type"):
            alias_value = normalized.get(alias)
            if isinstance(alias_value, str) and alias_value.strip():
                normalized["command"] = alias_value
                break

    if not normalized.get("target_id"):
        for alias in ("id", "cell_id", "cmd_id", "route_id"):
            alias_value = normalized.get(alias)
            if alias_value:
                normalized["target_id"] = alias_value
                break

    if "target_ned" not in normalized:
        target_xy = normalized.get("target_xy")
        if isinstance(target_xy, (list, tuple)) and len(target_xy) >= 2:
            normalized["target_ned"] = [target_xy[0], target_xy[1]]
        elif (
            "x" in normalized
            and "y" in normalized
            and normalized.get("x") is not None
            and normalized.get("y") is not None
        ):
            normalized["target_ned"] = [normalized.get("x"), normalized.get("y")]

    if "clear_radius_m" not in normalized:
        for alias in ("acceptance_radius_m", "acceptance_m", "radius_m"):
            if alias in normalized:
                normalized["clear_radius_m"] = normalized.get(alias)
                break

    if "cruise_speed_mps" not in normalized and "speed_mps" in normalized:
        normalized["cruise_speed_mps"] = normalized.get("speed_mps")
    if "altitude_m" not in normalized and "target_altitude_m" in normalized:
        normalized["altitude_m"] = normalized.get("target_altitude_m")
    return normalized


def target_command_to_msg(command: "TargetCommand", msg: Any | None = None) -> Any:
    """Fill a scout_control_msgs/TargetCommand-like object from a TargetCommand."""

    if msg is None:
        msg = type("TargetCommandMsg", (), {})()
    payload = command.to_payload()
    msg.command = command.command
    msg.target_id = command.target_id
    msg.cmd_id = command.cmd_id or command.target_id
    msg.route_id = command.route_id
    msg.name = command.name
    msg.frame = command.frame
    msg.target_ned = [] if command.target_ned is None else [
        float(command.target_ned[0]),
        float(command.target_ned[1]),
    ]
    msg.altitude_mode = command.altitude_mode
    msg.altitude_m = float(command.altitude_m)
    msg.cruise_speed_mps = float(command.cruise_speed_mps)
    msg.acceptance_radius_m = float(command.acceptance_radius_m)
    msg.clear_radius_m = float(command.clear_radius_m)
    msg.allow_replan = bool(command.allow_replan)
    msg.max_blocked_time_s = float(command.max_blocked_time_s)
    msg.priority = command.priority
    msg.source = command.source
    msg.stamp_ms = int(command.stamp_ms)
    msg.json_payload = _safe_json(payload)
    return msg


def target_command_from_msg(msg: Any) -> "TargetCommand":
    """Create TargetCommand from a generated ROS message or compatible object."""

    payload = _payload_from_msg(msg)
    if not payload:
        target_ned = list(getattr(msg, "target_ned", []) or [])
        payload = {
            "command": getattr(msg, "command", "goto"),
            "target_id": getattr(msg, "target_id", ""),
            "cmd_id": getattr(msg, "cmd_id", ""),
            "route_id": getattr(msg, "route_id", ""),
            "name": getattr(msg, "name", ""),
            "frame": getattr(msg, "frame", "local_ned"),
            "altitude_mode": getattr(msg, "altitude_mode", "relative_ned"),
            "altitude_m": _finite_or_default(getattr(msg, "altitude_m", 5.0), 5.0),
            "cruise_speed_mps": _finite_or_default(
                getattr(msg, "cruise_speed_mps", 2.5), 2.5
            ),
            "acceptance_radius_m": _finite_or_default(
                getattr(msg, "acceptance_radius_m", 1.5), 1.5
            ),
            "clear_radius_m": _finite_or_default(getattr(msg, "clear_radius_m", 2.5), 2.5),
            "allow_replan": bool(getattr(msg, "allow_replan", True)),
            "max_blocked_time_s": _finite_or_default(
                getattr(msg, "max_blocked_time_s", 30.0), 30.0
            ),
            "priority": getattr(msg, "priority", "mission"),
            "source": getattr(msg, "source", ""),
            "stamp_ms": int(getattr(msg, "stamp_ms", 0) or 0),
        }
        if len(target_ned) >= 2:
            payload["target_ned"] = [target_ned[0], target_ned[1]]
    return TargetCommand.from_payload(payload)


@dataclass(slots=True)
class TargetCommand:
    """High-level mission target routed into obstacle avoidance runtime."""

    command: str = "goto"
    target_id: str = ""
    cmd_id: str = ""
    route_id: str = ""
    name: str = ""
    frame: str = "local_ned"
    target_ned: tuple[float, float] | None = None
    altitude_mode: str = "relative_ned"
    altitude_m: float = 5.0
    cruise_speed_mps: float = 2.5
    acceptance_radius_m: float = 1.5
    clear_radius_m: float = 2.5
    allow_replan: bool = True
    max_blocked_time_s: float = 30.0
    priority: str = "mission"
    source: str = ""
    stamp_ms: int = 0
    desired_yaw_rad: float = float("nan")
    velocity_ned: tuple[float, float, float] | None = None
    yaw_rate_rad_s: float = float("nan")

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "TargetCommand":
        """Create a normalized command from the current JSON payload shape."""

        payload = normalize_target_command_payload(payload)
        command = str(payload.get("command", payload.get("cmd", "goto")))
        target_id = str(
            payload.get("target_id")
            or payload.get("cmd_id")
            or payload.get("route_id")
            or f"{command}_{int(time.time() * 1000)}"
        )
        altitude_m = float(payload.get("altitude_m", 5.0))
        if altitude_m < 0.0:
            raise ValueError(
                f"altitude_m must be >= 0.0 (height above ground), got {altitude_m}"
            )
        return cls(
            command=command,
            target_id=target_id,
            cmd_id=str(payload.get("cmd_id", target_id)),
            route_id=str(payload.get("route_id", "")),
            name=str(payload.get("name", command)),
            frame=str(payload.get("frame", "local_ned")),
            target_ned=_xy_tuple(payload.get("target_ned")),
            altitude_mode=str(payload.get("altitude_mode", "relative_ned")),
            altitude_m=altitude_m,
            cruise_speed_mps=float(payload.get("cruise_speed_mps", 2.5)),
            acceptance_radius_m=float(
                payload.get(
                    "acceptance_radius_m",
                    payload.get("clear_radius_m", 1.5),
                )
            ),
            clear_radius_m=float(payload.get("clear_radius_m", 2.5)),
            allow_replan=bool(payload.get("allow_replan", True)),
            max_blocked_time_s=float(payload.get("max_blocked_time_s", 30.0)),
            priority=str(payload.get("priority", "mission")),
            source=str(payload.get("source", "")),
            stamp_ms=int(payload.get("stamp_ms", int(time.time() * 1000))),
            desired_yaw_rad=float(payload.get("desired_yaw_rad", float("nan"))),
            velocity_ned=_xyz_tuple(payload.get("velocity_ned")),
            yaw_rate_rad_s=float(payload.get("yaw_rate_rad_s", float("nan"))),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "command": self.command,
            "target_id": self.target_id,
            "cmd_id": self.cmd_id or self.target_id,
            "route_id": self.route_id,
            "name": self.name,
            "frame": self.frame,
            "altitude_mode": self.altitude_mode,
            "altitude_m": float(self.altitude_m),
            "cruise_speed_mps": float(self.cruise_speed_mps),
            "acceptance_radius_m": float(self.acceptance_radius_m),
            "clear_radius_m": float(self.clear_radius_m),
            "allow_replan": bool(self.allow_replan),
            "max_blocked_time_s": float(self.max_blocked_time_s),
            "priority": self.priority,
            "source": self.source,
            "stamp_ms": int(self.stamp_ms),
        }
        if self.target_ned is not None:
            payload["target_ned"] = [float(self.target_ned[0]), float(self.target_ned[1])]
        import math as _math
        if not _math.isnan(self.desired_yaw_rad):
            payload["desired_yaw_rad"] = float(self.desired_yaw_rad)
        if self.velocity_ned is not None:
            payload["velocity_ned"] = [
                float(self.velocity_ned[0]),
                float(self.velocity_ned[1]),
                float(self.velocity_ned[2]),
            ]
        if not _math.isnan(self.yaw_rate_rad_s):
            payload["yaw_rate_rad_s"] = float(self.yaw_rate_rad_s)
        return payload


@dataclass(slots=True)
class PlannerConfig:
    """Planner and perception defaults shared across helper modules."""

    cell_size_m: float = 0.5
    local_grid_radius_m: float = 12.0
    inflation_radius_m: float = 0.8
    obstacle_cost: float = 100.0
    collision_band_min_z_m: float = -1.0
    collision_band_max_z_m: float = 1.0
    depth_min_range_m: float = 0.3
    depth_max_range_m: float = 20.0
    depth_pixel_stride: int = 4
    peer_track_ttl_s: float = 3.0
    peer_base_radius_m: float = 1.5
    peer_soft_shell_m: float = 2.0
    peer_lookahead_s: float = 1.5
    soft_blocked_timeout_s: float = 15.0
    hard_blocked_timeout_s: float = 30.0
    max_scan_retries: int = 2


@dataclass(slots=True)
class PlanResult:
    """Planner output: either direct movement, a subgoal path, or a block."""

    mode: str = "HOLD"
    subgoal_ned: tuple[float, float] | None = None
    path_ned: list[tuple[float, float]] = field(default_factory=list)
    path_cost: float = math.inf
    clearance_m: float = 0.0
    reason: str = ""
    blocked_zone_ids: list[str] = field(default_factory=list)

    @property
    def has_path(self) -> bool:
        return bool(self.path_ned)

    def to_payload(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "subgoal_ned": None if self.subgoal_ned is None else list(self.subgoal_ned),
            "path_ned": _xy_list(self.path_ned),
            "path_cost": float(self.path_cost),
            "clearance_m": float(self.clearance_m),
            "reason": self.reason,
            "blocked_zone_ids": list(self.blocked_zone_ids),
        }


@dataclass(slots=True)
class LocalGridSnapshot:
    """Planner-ready local occupancy / cost snapshot around the drone."""

    stamp_s: float
    origin_ned: tuple[float, float]
    resolution_m: float
    occupancy: np.ndarray
    costs: np.ndarray | None = None
    blocked_zone_ids: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.origin_ned = _xy_tuple(self.origin_ned) or (0.0, 0.0)
        self.occupancy = np.asarray(self.occupancy, dtype=np.uint8)
        if self.occupancy.ndim != 2:
            raise ValueError("occupancy must be a 2D array")
        if self.costs is not None:
            self.costs = np.asarray(self.costs, dtype=np.float32)
            if self.costs.shape != self.occupancy.shape:
                raise ValueError("costs must match occupancy shape")

    @property
    def width(self) -> int:
        return int(self.occupancy.shape[1])

    @property
    def height(self) -> int:
        return int(self.occupancy.shape[0])

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, x_ned: float, y_ned: float) -> tuple[int, int]:
        gx = int(math.floor((x_ned - self.origin_ned[0]) / self.resolution_m))
        gy = int(math.floor((y_ned - self.origin_ned[1]) / self.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        x = self.origin_ned[0] + (gx + 0.5) * self.resolution_m
        y = self.origin_ned[1] + (gy + 0.5) * self.resolution_m
        return x, y

    def occupied_ratio(self) -> float:
        return float(np.count_nonzero(self.occupancy) / self.occupancy.size)


@dataclass(slots=True)
class BlockedEvent:
    """Structured blocked-state record for runtime status and logs."""

    stamp_s: float
    severity: str
    reason: str
    cmd_id: str = ""
    route_id: str = ""
    target_id: str = ""
    drone_ned: tuple[float, float, float] | None = None
    target_ned: tuple[float, float] | None = None
    duration_s: float = 0.0
    blocked_zone_ids: list[str] = field(default_factory=list)
    reassign_recommended: bool = False

    def __post_init__(self) -> None:
        self.drone_ned = _xyz_tuple(self.drone_ned)
        self.target_ned = _xy_tuple(self.target_ned)

    def to_payload(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "severity": self.severity,
            "reason": self.reason,
            "cmd_id": self.cmd_id,
            "route_id": self.route_id,
            "target_id": self.target_id,
            "drone_ned": None if self.drone_ned is None else list(self.drone_ned),
            "target_ned": None if self.target_ned is None else list(self.target_ned),
            "duration_s": float(self.duration_s),
            "blocked_zone_ids": list(self.blocked_zone_ids),
            "reassign_recommended": bool(self.reassign_recommended),
        }


@dataclass(slots=True)
class PointBatch:
    """Shared point-cloud batch format between projectors and mapper."""

    source: str
    frame: str
    stamp_s: float
    points_xyz: np.ndarray
    confidence: float = 1.0
    sensor_range_m: float = 0.0
    is_dense_scan: bool = False

    def __post_init__(self) -> None:
        self.points_xyz = np.asarray(self.points_xyz, dtype=np.float32)
        if self.points_xyz.ndim != 2 or self.points_xyz.shape[1] != 3:
            raise ValueError("points_xyz must have shape Nx3")

    @property
    def count(self) -> int:
        return int(self.points_xyz.shape[0])

    @property
    def point_count(self) -> int:
        return self.count

    @property
    def stamp(self) -> float:
        return float(self.stamp_s)

    @property
    def xy(self) -> np.ndarray:
        return self.points_xyz[:, :2]

    @classmethod
    def empty(
        cls,
        *,
        source: str,
        frame: str,
        stamp_s: float,
        sensor_range_m: float = 0.0,
    ) -> "PointBatch":
        return cls(
            source=source,
            frame=frame,
            stamp_s=stamp_s,
            points_xyz=np.empty((0, 3), dtype=np.float32),
            sensor_range_m=sensor_range_m,
        )


@dataclass(slots=True)
class ScanArtifactPaths:
    """Filesystem paths written by a completed scan cycle."""

    point_cloud_path: str = ""
    rgb_path: str = ""
    meta_path: str = ""

    def as_dict(self) -> dict[str, str]:
        return {
            "point_cloud_path": self.point_cloud_path,
            "rgb_path": self.rgb_path,
            "meta_path": self.meta_path,
        }


@dataclass(slots=True)
class ScanMeta:
    """Structured metadata persisted for scan artifacts and events."""

    scan_index: int
    target_id: str
    target_name: str
    reason: str
    phase: str
    state: str
    success: bool
    drone_ned: list[float]
    target_ned: list[float]
    points: int
    scan_best_sectors: dict[str, float]
    free_directions: list[str]
    committed_side: str
    rgb_saved: bool
    camera_topic: str
    depth_topic: str
    point_batch_source: str
    event_name: str = "scan_complete"
    failure_reason: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "scan_index": int(self.scan_index),
            "target_id": self.target_id,
            "target_name": self.target_name,
            "reason": self.reason,
            "phase": self.phase,
            "state": self.state,
            "success": bool(self.success),
            "drone_ned": list(self.drone_ned),
            "target_ned": list(self.target_ned),
            "points": int(self.points),
            "scan_best_sectors": dict(self.scan_best_sectors),
            "free_directions": list(self.free_directions),
            "committed_side": self.committed_side,
            "rgb_saved": bool(self.rgb_saved),
            "camera_topic": self.camera_topic,
            "depth_topic": self.depth_topic,
            "point_batch_source": self.point_batch_source,
            "event_name": self.event_name,
            "failure_reason": self.failure_reason,
        }


@dataclass(slots=True)
class ScanCompleteEvent:
    """Scan completion payload published back to runtime."""

    success: bool
    reason: str
    scan_index: int
    target_id: str
    target_name: str
    failure_reason: str = ""
    points: int = 0
    free_directions: list[str] = field(default_factory=list)
    scan_best_sectors: dict[str, float] = field(default_factory=dict)
    artifact_paths: dict[str, str] = field(default_factory=dict)
    scan_meta: dict[str, Any] = field(default_factory=dict)
    event: str = "scan_complete"

    def as_dict(self) -> dict[str, Any]:
        return {
            "success": bool(self.success),
            "reason": self.reason,
            "scan_index": int(self.scan_index),
            "target_id": self.target_id,
            "target_name": self.target_name,
            "failure_reason": self.failure_reason,
            "points": int(self.points),
            "free_directions": list(self.free_directions),
            "scan_best_sectors": dict(self.scan_best_sectors),
            "artifact_paths": dict(self.artifact_paths),
            "scan_meta": dict(self.scan_meta),
            "event": self.event,
        }


@dataclass(slots=True)
class ScanCommand:
    """Immediate hover/yaw request emitted by the scan manager."""

    hold_position: bool
    desired_yaw: float


@dataclass(slots=True)
class ScanStepResult:
    """Result of one scan-manager control tick."""

    state: ScanState
    command: ScanCommand | None = None
    finished: bool = False
    success: bool = False
    point_batch: PointBatch | None = None
    scan_meta: ScanMeta | None = None
    artifact_paths: ScanArtifactPaths | None = None
    complete_event: ScanCompleteEvent | None = None
    sector_distances: dict[str, float] = field(default_factory=dict)
    free_directions: list[str] = field(default_factory=list)
    failure_reason: str = ""


@dataclass(slots=True)
class AvoidanceStatus:
    """Typed, backward-compatible view of `/{drone_ns}/avoidance/status` payloads."""

    phase: str = "UNKNOWN"
    state: str = "UNKNOWN"
    result: str = "ACTIVE"
    planner_mode: str = "NONE"
    blocked_severity: str = "NONE"
    blocked_reason: str = ""
    reassign_recommended: bool = False
    scan_state: str = "IDLE"
    scan_active: bool = False
    avoidance_active: bool = False
    target_id: str = ""
    cmd_id: str = ""
    route_id: str = ""
    target_ned: tuple[float, float] | None = None
    subgoal_ned: tuple[float, float] | None = None
    drone_ned: tuple[float, float, float] | None = None
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "AvoidanceStatus":
        phase = str(payload.get("phase") or payload.get("state") or "UNKNOWN")
        state = str(payload.get("state") or phase)
        scan_state = str(payload.get("scan_state", "IDLE"))
        blocked_severity = str(payload.get("blocked_severity", "NONE"))
        return cls(
            phase=phase,
            state=state,
            result=str(payload.get("result", "ACTIVE")),
            planner_mode=str(payload.get("planner_mode", "NONE")),
            blocked_severity=blocked_severity,
            blocked_reason=str(payload.get("blocked_reason", "")),
            reassign_recommended=bool(payload.get("reassign_recommended", False)),
            scan_state=scan_state,
            scan_active=bool(
                payload.get(
                    "scan_active",
                    scan_state not in ("IDLE", "COMPLETE", "FAILED"),
                )
            ),
            avoidance_active=bool(payload.get("avoidance_active", False)),
            target_id=str(payload.get("target_id", "")),
            cmd_id=str(payload.get("cmd_id", "")),
            route_id=str(payload.get("route_id", "")),
            target_ned=_xy_tuple(payload.get("target_ned")),
            subgoal_ned=_xy_tuple(payload.get("subgoal_ned")),
            drone_ned=_xyz_tuple(payload.get("drone_ned")),
            extras={
                str(k): v
                for k, v in payload.items()
                if k
                not in {
                    "phase",
                    "state",
                    "result",
                    "planner_mode",
                    "blocked_severity",
                    "blocked_reason",
                    "reassign_recommended",
                    "scan_state",
                    "scan_active",
                    "avoidance_active",
                    "target_id",
                    "cmd_id",
                    "route_id",
                    "target_ned",
                    "subgoal_ned",
                    "drone_ned",
                }
            },
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "phase": self.phase,
            "state": self.state,
            "result": self.result,
            "planner_mode": self.planner_mode,
            "blocked_severity": self.blocked_severity,
            "blocked_reason": self.blocked_reason,
            "reassign_recommended": bool(self.reassign_recommended),
            "scan_state": self.scan_state,
            "scan_active": bool(self.scan_active),
            "avoidance_active": bool(self.avoidance_active),
            "target_id": self.target_id,
            "cmd_id": self.cmd_id,
            "route_id": self.route_id,
            "target_ned": None if self.target_ned is None else list(self.target_ned),
            "subgoal_ned": None if self.subgoal_ned is None else list(self.subgoal_ned),
            "drone_ned": None if self.drone_ned is None else list(self.drone_ned),
        }
        payload.update(self.extras)
        return payload


def readiness_payload_to_msg(payload: Mapping[str, Any], msg: Any | None = None) -> Any:
    """Fill a DroneReadiness-like message from RuntimeReadiness.to_payload()."""

    if msg is None:
        msg = type("DroneReadinessMsg", (), {})()
    pose = payload.get("pose", {}) if isinstance(payload.get("pose"), Mapping) else {}
    depth = payload.get("depth", {}) if isinstance(payload.get("depth"), Mapping) else {}
    msg.ready = bool(payload.get("ready", False))
    msg.navigation_allowed = bool(payload.get("navigation_allowed", False))
    msg.setpoint_publish_allowed = bool(payload.get("setpoint_publish_allowed", False))
    msg.pose_valid = bool(pose.get("valid", payload.get("pose_valid", False)))
    msg.depth_ready = bool(depth.get("ready", payload.get("depth_ready", False)))
    msg.owner_conflict = bool(payload.get("owner_conflict", False))
    msg.reason = str(payload.get("reason", ""))
    msg.severity = str(payload.get("severity", ""))
    msg.pose_age_s = _finite_or_default(pose.get("age_s", payload.get("pose_age_s", 0.0)))
    msg.depth_age_s = _finite_or_default(depth.get("age_s", payload.get("depth_age_s", 0.0)))
    msg.json_payload = _safe_json(payload)
    return msg


def readiness_msg_to_payload(msg: Any) -> dict[str, Any]:
    payload = _payload_from_msg(msg)
    if payload:
        return payload
    return {
        "ready": bool(getattr(msg, "ready", False)),
        "navigation_allowed": bool(getattr(msg, "navigation_allowed", False)),
        "setpoint_publish_allowed": bool(
            getattr(msg, "setpoint_publish_allowed", False)
        ),
        "reason": str(getattr(msg, "reason", "")),
        "severity": str(getattr(msg, "severity", "")),
        "depth_ready": bool(getattr(msg, "depth_ready", False)),
        "depth_age_s": _finite_or_default(getattr(msg, "depth_age_s", 0.0)),
        "owner_conflict": bool(getattr(msg, "owner_conflict", False)),
        "pose": {
            "valid": bool(getattr(msg, "pose_valid", False)),
            "age_s": _finite_or_default(getattr(msg, "pose_age_s", 0.0)),
        },
    }


def avoidance_status_to_msg(
    status: "AvoidanceStatus", msg: Any | None = None, *, drone_id: str = ""
) -> Any:
    """Fill a scout_control_msgs/AvoidanceStatus-like message."""

    if msg is None:
        msg = type("AvoidanceStatusMsg", (), {})()
    payload = status.to_payload()
    readiness_payload = payload.get("readiness", {})
    if not isinstance(readiness_payload, Mapping):
        readiness_payload = {}
    msg.drone_id = drone_id or str(payload.get("drone_id", ""))
    msg.phase = status.phase
    msg.state = status.state
    msg.result = status.result
    msg.command = str(payload.get("command", ""))
    msg.target_id = status.target_id
    msg.target_name = str(payload.get("target_name", payload.get("mission_name", "")))
    msg.target_ned = [] if status.target_ned is None else list(status.target_ned)
    msg.subgoal_ned = [] if status.subgoal_ned is None else list(status.subgoal_ned)
    msg.drone_ned = [] if status.drone_ned is None else list(status.drone_ned)
    msg.target_active = bool(payload.get("target_active", payload.get("command_active", False)))
    msg.navigator_ready = bool(payload.get("navigator_ready", False))
    msg.runtime_ready = bool(payload.get("runtime_ready", False))
    if hasattr(msg, "readiness"):
        readiness_payload_to_msg(readiness_payload, msg.readiness)
    if hasattr(msg, "health"):
        health_payload = payload.get("health", readiness_payload)
        if not isinstance(health_payload, Mapping):
            health_payload = readiness_payload
        msg.health.drone_id = msg.drone_id
        msg.health.runtime_ready = msg.runtime_ready
        msg.health.navigator_ready = msg.navigator_ready
        if hasattr(msg.health, "readiness"):
            readiness_payload_to_msg(health_payload, msg.health.readiness)
        msg.health.json_payload = _safe_json(health_payload)
    msg.px4_input_ownership_json = _safe_json(
        payload.get("px4_input_ownership", {})
        if isinstance(payload.get("px4_input_ownership"), Mapping)
        else {}
    )
    msg.altitude_policy_json = _safe_json(
        payload.get("altitude_policy", {})
        if isinstance(payload.get("altitude_policy"), Mapping)
        else {}
    )
    msg.avoidance_active = bool(status.avoidance_active)
    msg.obstacle_warn = bool(payload.get("obstacle_warn", False))
    msg.obstacle_critical = bool(payload.get("obstacle_critical", False))
    msg.obstacle_closest_m = _finite_or_default(payload.get("obstacle_closest_m", 99.0), 99.0)
    msg.free_directions = [str(x) for x in payload.get("free_directions", []) or []]
    msg.planner_mode = status.planner_mode
    msg.planner_state = str(payload.get("planner_state", ""))
    msg.scan_state = status.scan_state
    msg.scan_active = bool(status.scan_active)
    msg.mapper_state = str(payload.get("mapper_state", ""))
    msg.local_map_age_s = _finite_or_default(payload.get("local_map_age_s", 0.0))
    msg.dense_scan_points = int(payload.get("dense_scan_points", 0) or 0)
    msg.blocked_reason = status.blocked_reason
    msg.blocked_severity = status.blocked_severity
    msg.reassign_recommended = bool(status.reassign_recommended)
    msg.blocked_since_s = _finite_or_default(payload.get("blocked_since_s", 0.0))
    msg.target_reached = bool(payload.get("target_reached", False))
    msg.last_completed_target_id = str(payload.get("last_completed_target_id", ""))
    msg.last_completed_target_name = str(payload.get("last_completed_target_name", ""))
    msg.json_payload = _safe_json(payload)
    return msg


def avoidance_status_from_msg(msg: Any) -> "AvoidanceStatus":
    payload = _payload_from_msg(msg)
    if not payload:
        payload = {
            "phase": getattr(msg, "phase", "UNKNOWN"),
            "state": getattr(msg, "state", getattr(msg, "phase", "UNKNOWN")),
            "result": getattr(msg, "result", "ACTIVE"),
            "command": getattr(msg, "command", ""),
            "target_id": getattr(msg, "target_id", ""),
            "target_name": getattr(msg, "target_name", ""),
            "target_ned": list(getattr(msg, "target_ned", []) or []) or None,
            "subgoal_ned": list(getattr(msg, "subgoal_ned", []) or []) or None,
            "drone_ned": list(getattr(msg, "drone_ned", []) or []) or None,
            "target_active": bool(getattr(msg, "target_active", False)),
            "navigator_ready": bool(getattr(msg, "navigator_ready", False)),
            "runtime_ready": bool(getattr(msg, "runtime_ready", False)),
            "avoidance_active": bool(getattr(msg, "avoidance_active", False)),
            "obstacle_warn": bool(getattr(msg, "obstacle_warn", False)),
            "obstacle_critical": bool(getattr(msg, "obstacle_critical", False)),
            "obstacle_closest_m": _finite_or_default(
                getattr(msg, "obstacle_closest_m", 99.0), 99.0
            ),
            "free_directions": list(getattr(msg, "free_directions", []) or []),
            "planner_mode": getattr(msg, "planner_mode", "NONE"),
            "planner_state": getattr(msg, "planner_state", ""),
            "scan_state": getattr(msg, "scan_state", "IDLE"),
            "scan_active": bool(getattr(msg, "scan_active", False)),
            "mapper_state": getattr(msg, "mapper_state", ""),
            "local_map_age_s": _finite_or_default(getattr(msg, "local_map_age_s", 0.0)),
            "dense_scan_points": int(getattr(msg, "dense_scan_points", 0) or 0),
            "blocked_reason": getattr(msg, "blocked_reason", ""),
            "blocked_severity": getattr(msg, "blocked_severity", "NONE"),
            "reassign_recommended": bool(getattr(msg, "reassign_recommended", False)),
            "blocked_since_s": _finite_or_default(getattr(msg, "blocked_since_s", 0.0)),
            "target_reached": bool(getattr(msg, "target_reached", False)),
            "last_completed_target_id": getattr(msg, "last_completed_target_id", ""),
            "last_completed_target_name": getattr(msg, "last_completed_target_name", ""),
        }
        if hasattr(msg, "readiness"):
            payload["readiness"] = readiness_msg_to_payload(msg.readiness)
            payload["health"] = payload["readiness"]
    return AvoidanceStatus.from_payload(payload)


@dataclass(slots=True)
class SwarmDroneStatusEvent:
    """Typed helper for `/swarm/drone_status` event payloads."""

    drone_id: str = ""
    status: str = ""
    cell_id: str = ""
    cmd_id: str = ""
    route_id: str = ""
    backend: str = ""
    nav_state: str = ""
    nav_result: str = ""
    blocked_severity: str = "NONE"
    blocked_reason: str = ""
    reassign_recommended: bool = False
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "SwarmDroneStatusEvent":
        return cls(
            drone_id=str(payload.get("drone_id", "")),
            status=str(payload.get("status", "")),
            cell_id=str(payload.get("cell_id", "")),
            cmd_id=str(payload.get("cmd_id", "")),
            route_id=str(payload.get("route_id", "")),
            backend=str(payload.get("backend", payload.get("navigation_backend", ""))),
            nav_state=str(payload.get("nav_state", payload.get("state", ""))),
            nav_result=str(payload.get("nav_result", payload.get("result", ""))),
            blocked_severity=str(payload.get("blocked_severity", "NONE")),
            blocked_reason=str(payload.get("blocked_reason", "")),
            reassign_recommended=bool(payload.get("reassign_recommended", False)),
            extras={
                str(k): v
                for k, v in payload.items()
                if k
                not in {
                    "drone_id",
                    "status",
                    "cell_id",
                    "cmd_id",
                    "route_id",
                    "backend",
                    "navigation_backend",
                    "nav_state",
                    "state",
                    "nav_result",
                    "result",
                    "blocked_severity",
                    "blocked_reason",
                    "reassign_recommended",
                }
            },
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "drone_id": self.drone_id,
            "status": self.status,
            "cell_id": self.cell_id,
            "cmd_id": self.cmd_id,
            "route_id": self.route_id,
            "backend": self.backend,
            "navigation_backend": self.backend,
            "nav_state": self.nav_state,
            "nav_result": self.nav_result,
            "blocked_severity": self.blocked_severity,
            "blocked_reason": self.blocked_reason,
            "reassign_recommended": bool(self.reassign_recommended),
        }
        payload.update(self.extras)
        return payload


def _extras(payload: Mapping[str, Any], known: set[str]) -> dict[str, Any]:
    return {str(k): v for k, v in payload.items() if k not in known}


@dataclass(slots=True)
class SwarmTaskStatus:
    """Typed helper for `/swarm/task_status` JSON/String compatibility payloads."""

    status: str = ""
    event: str = ""
    mission_id: str = ""
    total_cells: int = 0
    completed_cells: int = 0
    pending_cells: int = 0
    assigned_cells: int = 0
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "SwarmTaskStatus":
        known = {
            "status",
            "event",
            "mission_id",
            "total_cells",
            "completed_cells",
            "pending_cells",
            "assigned_cells",
        }
        return cls(
            status=str(payload.get("status", "")),
            event=str(payload.get("event", "")),
            mission_id=str(payload.get("mission_id", "")),
            total_cells=int(payload.get("total_cells", 0) or 0),
            completed_cells=int(payload.get("completed_cells", 0) or 0),
            pending_cells=int(payload.get("pending_cells", 0) or 0),
            assigned_cells=int(payload.get("assigned_cells", 0) or 0),
            extras=_extras(payload, known),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "status": self.status,
            "event": self.event,
            "mission_id": self.mission_id,
            "total_cells": int(self.total_cells),
            "completed_cells": int(self.completed_cells),
            "pending_cells": int(self.pending_cells),
            "assigned_cells": int(self.assigned_cells),
        }
        payload.update(self.extras)
        return payload


@dataclass(slots=True)
class PadAssignment:
    """Typed helper for `/swarm/pad_assignment` payloads."""

    drone_id: str = ""
    pad_id: str = ""
    status: str = ""
    pad_ned: tuple[float, float, float] | None = None
    assignment_id: str = ""
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "PadAssignment":
        known = {"drone_id", "pad_id", "status", "pad_ned", "ned", "assignment_id"}
        return cls(
            drone_id=str(payload.get("drone_id", "")),
            pad_id=str(payload.get("pad_id", "")),
            status=str(payload.get("status", "")),
            pad_ned=_xyz_tuple(payload.get("pad_ned", payload.get("ned"))),
            assignment_id=str(payload.get("assignment_id", "")),
            extras=_extras(payload, known),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "drone_id": self.drone_id,
            "pad_id": self.pad_id,
            "status": self.status,
            "pad_ned": None if self.pad_ned is None else list(self.pad_ned),
            "assignment_id": self.assignment_id,
        }
        payload.update(self.extras)
        return payload


@dataclass(slots=True)
class FieldSetupComplete:
    """Typed helper for `/field/setup_complete` payloads."""

    ready: bool = False
    field_id: str = ""
    boundary_file: str = ""
    grid_file: str = ""
    home_positions_file: str = ""
    drone_count: int = 0
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "FieldSetupComplete":
        known = {
            "ready",
            "field_id",
            "boundary_file",
            "grid_file",
            "home_positions_file",
            "drone_count",
        }
        return cls(
            ready=bool(payload.get("ready", payload.get("complete", False))),
            field_id=str(payload.get("field_id", "")),
            boundary_file=str(payload.get("boundary_file", "")),
            grid_file=str(payload.get("grid_file", "")),
            home_positions_file=str(payload.get("home_positions_file", "")),
            drone_count=int(payload.get("drone_count", 0) or 0),
            extras=_extras(payload, known | {"complete"}),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "ready": bool(self.ready),
            "field_id": self.field_id,
            "boundary_file": self.boundary_file,
            "grid_file": self.grid_file,
            "home_positions_file": self.home_positions_file,
            "drone_count": int(self.drone_count),
        }
        payload.update(self.extras)
        return payload


@dataclass(slots=True)
class ReturnHomeRequest:
    """Typed helper for `/swarm/rth_request` payloads."""

    drone_id: str = ""
    request_id: str = ""
    reason: str = ""
    requester: str = ""
    target_pad_id: str = ""
    all_drones: bool = False
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ReturnHomeRequest":
        known = {
            "drone_id",
            "request_id",
            "cmd_id",
            "reason",
            "requester",
            "target_pad_id",
            "pad_id",
            "all_drones",
        }
        return cls(
            drone_id=str(payload.get("drone_id", "")),
            request_id=str(payload.get("request_id", payload.get("cmd_id", ""))),
            reason=str(payload.get("reason", "")),
            requester=str(payload.get("requester", "")),
            target_pad_id=str(payload.get("target_pad_id", payload.get("pad_id", ""))),
            all_drones=bool(payload.get("all_drones", False)),
            extras=_extras(payload, known),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "drone_id": self.drone_id,
            "request_id": self.request_id,
            "cmd_id": self.request_id,
            "reason": self.reason,
            "requester": self.requester,
            "target_pad_id": self.target_pad_id,
            "all_drones": bool(self.all_drones),
        }
        payload.update(self.extras)
        return payload


@dataclass(slots=True)
class MissionReadySignal:
    """Typed helper for `/swarm/mission_ready` payloads."""

    ready: bool = False
    mission_id: str = ""
    field_id: str = ""
    source: str = ""
    drone_count: int = 0
    extras: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "MissionReadySignal":
        known = {"ready", "mission_id", "field_id", "source", "drone_count"}
        return cls(
            ready=bool(payload.get("ready", True)),
            mission_id=str(payload.get("mission_id", "")),
            field_id=str(payload.get("field_id", "")),
            source=str(payload.get("source", "")),
            drone_count=int(payload.get("drone_count", 0) or 0),
            extras=_extras(payload, known),
        )

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "ready": bool(self.ready),
            "mission_id": self.mission_id,
            "field_id": self.field_id,
            "source": self.source,
            "drone_count": int(self.drone_count),
        }
        payload.update(self.extras)
        return payload
