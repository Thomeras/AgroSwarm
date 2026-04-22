"""Shared datatypes for the avoidance runtime and planner helpers."""

from __future__ import annotations

import math
import time
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

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "TargetCommand":
        """Create a normalized command from the current JSON payload shape."""

        command = str(payload.get("command", payload.get("cmd", "goto")))
        target_id = str(
            payload.get("target_id")
            or payload.get("cmd_id")
            or payload.get("route_id")
            or f"{command}_{int(time.time() * 1000)}"
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
            altitude_m=float(payload.get("altitude_m", 5.0)),
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


@dataclass(slots=True)
class SwarmDroneStatusEvent:
    """Typed helper for `/swarm/drone_status` event payloads."""

    drone_id: str = ""
    status: str = ""
    cell_id: str = ""
    cmd_id: str = ""
    route_id: str = ""
    navigation_backend: str = ""
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
            navigation_backend=str(payload.get("navigation_backend", "")),
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
            "navigation_backend": self.navigation_backend,
            "nav_state": self.nav_state,
            "nav_result": self.nav_result,
            "blocked_severity": self.blocked_severity,
            "blocked_reason": self.blocked_reason,
            "reassign_recommended": bool(self.reassign_recommended),
        }
        payload.update(self.extras)
        return payload
