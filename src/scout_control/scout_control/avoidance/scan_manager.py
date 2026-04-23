from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from scout_control.avoidance.depth_projector import DepthProjector
from scout_control.avoidance.local_mapper import LocalMapper
from scout_control.avoidance.types import (
    PointBatch,
    ScanArtifactPaths,
    ScanCommand,
    ScanCompleteEvent,
    ScanMeta,
    ScanState,
    ScanStepResult,
)

SCAN_POINT_MIN_RANGE_M = 0.3
SCAN_POINT_MAX_RANGE_M = 20.0
DEFAULT_MAX_SCAN_ARTIFACTS = 20


class ScanManager:
    def __init__(
        self,
        *,
        mapper: LocalMapper,
        assets_dir: Path,
        hover_ticks: int,
        spin_ticks: int,
        point_stride: int,
        free_distance_m: float,
        cam_hfov_deg: float,
        camera_topic: str,
        depth_topic: str,
        log_cb: Callable[[str], None],
        run_log_cb: Callable[[str, Any], None],
        point_min_range_m: float = SCAN_POINT_MIN_RANGE_M,
        point_max_range_m: float = SCAN_POINT_MAX_RANGE_M,
        depth_encoding: str = "32FC1",
        max_scan_artifacts: int = DEFAULT_MAX_SCAN_ARTIFACTS,
    ) -> None:
        self._mapper = mapper
        self._assets_dir = Path(assets_dir)
        self._hover_ticks = max(1, int(hover_ticks))
        self._spin_ticks = max(10, int(spin_ticks))
        self._point_stride = max(1, int(point_stride))
        self._free_distance_m = float(free_distance_m)
        self._cam_hfov_rad = math.radians(float(cam_hfov_deg))
        self._camera_topic = camera_topic
        self._depth_topic = depth_topic
        self._depth_encoding = str(depth_encoding)
        self._max_scan_artifacts = max(1, int(max_scan_artifacts))
        self._point_min_range_m = float(point_min_range_m)
        self._point_max_range_m = float(point_max_range_m)
        self._log_cb = log_cb
        self._run_log_cb = run_log_cb
        self._projector = DepthProjector(
            camera_hfov_deg=float(cam_hfov_deg),
            min_range_m=self._point_min_range_m,
            max_range_m=self._point_max_range_m,
            default_stride=self._point_stride,
        )

        self._state = ScanState.IDLE
        self._scan_index = 0
        self._phase_ticks = 0
        self._base_yaw = 0.0
        self._reason = ""
        self._target_id = ""
        self._target_name = ""
        self._phase_name = ""
        self._committed_side = "none"
        self._closest_m = 99.0
        self._target_ned = (0.0, 0.0)
        self._last_depth_ts = 0.0
        self._last_projection_meta: dict[str, Any] = {}
        self._depth_timestamp_provenance = "unavailable"
        self._point_keys: set[tuple[int, int, int]] = set()
        self._points_world: list[tuple[float, float, float]] = []
        self._best_sectors = {"left": 99.0, "center": 99.0, "right": 99.0}
        self._failure_reason = ""

    def set_camera_info(self, camera_info: Any | None) -> None:
        """Install CameraInfo intrinsics for subsequent scan captures."""

        self._projector.set_camera_info(camera_info)

    @property
    def state(self) -> ScanState:
        return self._state

    def start_scan(
        self,
        *,
        reason: str,
        pose_ned: tuple[float, float, float],
        yaw: float,
        mission_target_ned: tuple[float, float],
        target_id: str,
        target_name: str,
        phase_name: str,
        closest_m: float,
        committed_side: str,
    ) -> None:
        del pose_ned
        self._scan_index += 1
        self._state = ScanState.PREPARE_HOVER
        self._phase_ticks = 0
        self._base_yaw = float(yaw)
        self._reason = reason
        self._target_id = target_id
        self._target_name = target_name
        self._phase_name = phase_name
        self._closest_m = float(closest_m)
        self._committed_side = committed_side
        self._target_ned = (float(mission_target_ned[0]), float(mission_target_ned[1]))
        self._last_depth_ts = 0.0
        self._last_projection_meta = {}
        self._depth_timestamp_provenance = "unavailable"
        self._point_keys.clear()
        self._points_world = []
        self._best_sectors = {"left": 99.0, "center": 99.0, "right": 99.0}
        self._failure_reason = ""
        self._run_log_cb(
            "scan_started",
            scan_index=int(self._scan_index),
            reason=reason,
            target_id=target_id,
            target_name=target_name,
            mission_target_ned=[round(self._target_ned[0], 3), round(self._target_ned[1], 3)],
            closest_m=round(float(closest_m), 3),
            committed_side=committed_side,
        )

    def step(
        self,
        *,
        pose_ned: tuple[float, float, float],
        yaw: float,
        rgb_frame: np.ndarray | None,
        depth_frame: np.ndarray | None,
        depth_ts: float,
        obstacle_sectors: dict[str, float],
    ) -> ScanStepResult:
        if self._state == ScanState.IDLE:
            return ScanStepResult(state=self._state)

        self._capture_observation(pose_ned=pose_ned, yaw=yaw, depth_frame=depth_frame, depth_ts=depth_ts)

        if self._state == ScanState.PREPARE_HOVER:
            self._phase_ticks += 1
            command = ScanCommand(hold_position=True, desired_yaw=self._base_yaw)
            if self._phase_ticks >= self._hover_ticks:
                self._state = ScanState.SPIN_CAPTURE
                self._phase_ticks = 0
            return ScanStepResult(state=self._state, command=command)

        if self._state == ScanState.SPIN_CAPTURE:
            self._phase_ticks += 1
            spin_progress = min(1.0, float(self._phase_ticks) / float(self._spin_ticks))
            command = ScanCommand(
                hold_position=True,
                desired_yaw=self._base_yaw + spin_progress * (2.0 * math.pi),
            )
            if self._phase_ticks < self._spin_ticks:
                return ScanStepResult(state=self._state, command=command)
            self._state = ScanState.PROCESS
            return self._process_scan(
                pose_ned=pose_ned,
                rgb_frame=rgb_frame,
                obstacle_sectors=obstacle_sectors,
                command=command,
            )

        if self._state in {ScanState.COMPLETE, ScanState.FAILED}:
            command = ScanCommand(hold_position=True, desired_yaw=self._base_yaw)
            return ScanStepResult(state=self._state, command=command, finished=True, success=self._state == ScanState.COMPLETE)

        return ScanStepResult(state=self._state)

    def reset(self) -> None:
        self._state = ScanState.IDLE
        self._phase_ticks = 0
        self._reason = ""
        self._target_id = ""
        self._target_name = ""
        self._failure_reason = ""
        self._closest_m = 99.0
        self._committed_side = "none"
        self._point_keys.clear()
        self._points_world = []
        self._best_sectors = {"left": 99.0, "center": 99.0, "right": 99.0}

    def _capture_observation(
        self,
        *,
        pose_ned: tuple[float, float, float],
        yaw: float,
        depth_frame: np.ndarray | None,
        depth_ts: float,
    ) -> None:
        if depth_frame is None or depth_ts <= self._last_depth_ts:
            return

        depth = np.asarray(depth_frame, dtype=np.float32)
        if depth.ndim != 2:
            return

        body_batch = self._projector.depth_to_body_points(
            depth,
            pixel_stride=self._point_stride,
            stamp_s=float(depth_ts),
            source="scan_manager_depth",
            is_dense_scan=True,
            encoding=self._depth_encoding,
        )
        self._last_projection_meta = self._projector.last_projection_metadata
        self._depth_timestamp_provenance = "sensor" if depth_ts > 0.0 else "wall_time"
        if body_batch.point_count == 0:
            self._last_depth_ts = depth_ts
            return

        world_batch = self._projector.project_to_world_points(
            body_batch,
            origin_ned=pose_ned,
            yaw_rad=float(yaw),
            ground_z_ned=0.0,
            source="scan_manager_dense_scan",
        )

        for px, py, pz in world_batch.points_xyz.tolist():
            key = (int(round(px * 5.0)), int(round(py * 5.0)), int(round(pz * 5.0)))
            if key in self._point_keys:
                continue
            self._point_keys.add(key)
            self._points_world.append((float(px), float(py), float(pz)))

        self._last_depth_ts = depth_ts

    def _process_scan(
        self,
        *,
        pose_ned: tuple[float, float, float],
        rgb_frame: np.ndarray | None,
        obstacle_sectors: dict[str, float],
        command: ScanCommand,
    ) -> ScanStepResult:
        points_arr = np.asarray(self._points_world, dtype=np.float32)
        if points_arr.size == 0:
            points_arr = np.empty((0, 3), dtype=np.float32)
        else:
            points_arr = points_arr.reshape((-1, 3))

        self._best_sectors, free_directions = self._analyze_scan_points(
            pose_ned=pose_ned,
            points_world=points_arr,
            obstacle_sectors=obstacle_sectors,
        )

        point_batch = PointBatch(
            source="scan_manager_dense_scan",
            frame="map",
            stamp_s=float(self._last_depth_ts if self._last_depth_ts > 0.0 else time.time()),
            points_xyz=points_arr,
            confidence=1.0 if points_arr.shape[0] > 0 else 0.0,
            sensor_range_m=float(self._point_max_range_m),
            is_dense_scan=True,
        )
        inserted_voxels = self._mapper.ingest_point_batch(point_batch)

        success = point_batch.point_count > 0
        if not success:
            self._failure_reason = "no_depth_points_captured"

        artifact_paths, meta = self._save_artifacts(
            pose_ned=pose_ned,
            rgb_frame=rgb_frame,
            point_batch=point_batch,
            success=success,
            free_directions=free_directions,
        )
        event = ScanCompleteEvent(
            success=success,
            reason=self._reason,
            scan_index=int(self._scan_index),
            target_id=self._target_id,
            target_name=self._target_name,
            failure_reason=self._failure_reason,
            points=point_batch.point_count,
            free_directions=list(free_directions),
            scan_best_sectors={k: round(float(v), 3) for k, v in self._best_sectors.items()},
            artifact_paths=artifact_paths.as_dict(),
            scan_meta=meta.as_dict(),
        )
        run_fields = event.as_dict()
        run_fields.pop("event", None)
        self._run_log_cb(
            "scan_complete",
            **run_fields,
            mapper_inserted_voxels=int(inserted_voxels),
        )

        if success:
            self._state = ScanState.COMPLETE
        else:
            self._state = ScanState.FAILED
            self._log_cb(
                f"Scan {self._scan_index} finished without dense point cloud "
                f"(reason={self._reason})"
            )

        return ScanStepResult(
            state=self._state,
            command=command,
            finished=True,
            success=success,
            point_batch=point_batch,
            scan_meta=meta,
            artifact_paths=artifact_paths,
            complete_event=event,
            sector_distances=dict(self._best_sectors),
            free_directions=list(free_directions),
            failure_reason=self._failure_reason,
        )

    def _analyze_scan_points(
        self,
        *,
        pose_ned: tuple[float, float, float],
        points_world: np.ndarray,
        obstacle_sectors: dict[str, float],
    ) -> tuple[dict[str, float], list[str]]:
        if points_world.shape[0] == 0:
            fallback_sectors = {k: float(v) for k, v in obstacle_sectors.items()}
            fallback_free = [
                side for side, dist in fallback_sectors.items()
                if dist > self._free_distance_m
            ]
            return fallback_sectors, fallback_free

        course = math.atan2(self._target_ned[1] - pose_ned[1], self._target_ned[0] - pose_ned[0])
        sectors = {"left": 99.0, "center": 99.0, "right": 99.0}

        for px, py, _pz in points_world.tolist():
            dx = px - pose_ned[0]
            dy = py - pose_ned[1]
            dist = math.hypot(dx, dy)
            if dist < 0.1:
                continue
            heading = math.atan2(dy, dx)
            rel = math.atan2(math.sin(heading - course), math.cos(heading - course))
            rel_deg = math.degrees(rel)
            if abs(rel_deg) <= 30.0:
                sectors["center"] = min(sectors["center"], dist)
            elif 30.0 < rel_deg <= 140.0:
                sectors["left"] = min(sectors["left"], dist)
            elif -140.0 <= rel_deg < -30.0:
                sectors["right"] = min(sectors["right"], dist)

        free_directions = [
            side for side, dist in sectors.items()
            if dist > self._free_distance_m
        ]
        return sectors, free_directions

    def _save_artifacts(
        self,
        *,
        pose_ned: tuple[float, float, float],
        rgb_frame: np.ndarray | None,
        point_batch: PointBatch,
        success: bool,
        free_directions: list[str],
    ) -> tuple[ScanArtifactPaths, ScanMeta]:
        target_slug = "".join(
            c.lower() if c.isalnum() else "_"
            for c in (self._target_name or "target")
        ).strip("_")
        scan_dir = self._assets_dir / f"scan_{self._scan_index:03d}_{target_slug}"
        scan_dir.mkdir(parents=True, exist_ok=True)

        point_cloud_path = scan_dir / "scan_cloud.npz"
        rgb_path = scan_dir / "scan_rgb.png"
        meta_path = scan_dir / "scan_meta.json"

        self._write_point_cloud_npz(point_cloud_path, point_batch.points_xyz)

        rgb_saved = False
        if rgb_frame is not None:
            rgb_saved = bool(cv2.imwrite(str(rgb_path), rgb_frame))
        if not rgb_saved and rgb_path.exists():
            rgb_path.unlink()

        meta = ScanMeta(
            scan_index=int(self._scan_index),
            target_id=self._target_id,
            target_name=self._target_name,
            reason=self._reason,
            phase=self._phase_name,
            state=self._state.name,
            success=bool(success),
            drone_ned=[
                round(float(pose_ned[0]), 3),
                round(float(pose_ned[1]), 3),
                round(float(pose_ned[2]), 3),
            ],
            target_ned=[round(float(self._target_ned[0]), 3), round(float(self._target_ned[1]), 3)],
            points=int(point_batch.point_count),
            scan_best_sectors={k: round(float(v), 3) for k, v in self._best_sectors.items()},
            free_directions=list(free_directions),
            committed_side=self._committed_side,
            rgb_saved=bool(rgb_saved),
            camera_topic=self._camera_topic,
            depth_topic=self._depth_topic,
            point_batch_source=point_batch.source,
            failure_reason=self._failure_reason,
        )
        meta_payload = meta.as_dict()
        meta_payload.update(
            {
                "artifact_format": "npz",
                "artifact_retention_max_scans": int(self._max_scan_artifacts),
                "camera_intrinsics": dict(
                    self._last_projection_meta.get(
                        "camera_intrinsics",
                        self._projector.intrinsics_for_frame(1, 1).as_dict(),
                    )
                ),
                "depth_encoding": self._depth_encoding,
                "depth_stride": int(self._point_stride),
                "timestamp_provenance": {
                    "depth_ts": float(self._last_depth_ts),
                    "point_batch_stamp_s": float(point_batch.stamp_s),
                    "source": self._depth_timestamp_provenance,
                },
                "topics": {
                    "camera": self._camera_topic,
                    "depth": self._depth_topic,
                },
                "projection_path": "DepthProjector.depth_to_body_points -> DepthProjector.project_to_world_points",
            }
        )
        meta_path.write_text(
            json.dumps(meta_payload, indent=2, ensure_ascii=True),
            encoding="utf-8",
        )
        self._prune_scan_artifacts()

        return (
            ScanArtifactPaths(
                point_cloud_path=str(point_cloud_path),
                rgb_path=str(rgb_path) if rgb_saved else "",
                meta_path=str(meta_path),
            ),
            meta,
        )

    def _write_point_cloud_npz(self, path: Path, points_xyz: np.ndarray) -> None:
        points = np.asarray(points_xyz, dtype=np.float32)
        if points.size == 0:
            points = np.empty((0, 3), dtype=np.float32)
        else:
            points = points.reshape((-1, 3))

        np.savez_compressed(
            path,
            points_xyz=points,
            frame=np.array("map"),
            source=np.array("scan_manager_dense_scan"),
            stamp_s=np.array(float(self._last_depth_ts), dtype=np.float64),
        )

    def _prune_scan_artifacts(self) -> None:
        if not self._assets_dir.exists():
            return
        scan_dirs = [
            path for path in self._assets_dir.iterdir()
            if path.is_dir() and path.name.startswith("scan_")
        ]
        scan_dirs.sort(key=lambda path: (path.stat().st_mtime, path.name), reverse=True)
        for old_dir in scan_dirs[self._max_scan_artifacts:]:
            for child in old_dir.iterdir():
                if child.is_file() or child.is_symlink():
                    child.unlink()
            try:
                old_dir.rmdir()
            except OSError:
                continue
