from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from scout_control.avoidance.peer_tracks import PeerTrackStore
from scout_control.avoidance.types import LocalMapperState, PointBatch


def prob_to_logodds(probability: float) -> float:
    """Convert a bounded occupancy probability to log-odds."""

    p = min(1.0 - 1e-6, max(1e-6, float(probability)))
    return float(math.log(p / (1.0 - p)))


def logodds_to_prob(log_odds: np.ndarray | float) -> np.ndarray | float:
    """Convert log-odds to occupancy probability."""

    odds = np.exp(np.asarray(log_odds, dtype=np.float32))
    probability = odds / (1.0 + odds)
    if np.isscalar(log_odds):
        return float(probability)
    return probability.astype(np.float32, copy=False)


@dataclass(slots=True)
class LocalMapperConfig:
    resolution_m: float = 0.5
    span_x_m: float = 36.0
    span_y_m: float = 36.0
    depth_half_life_s: float = 10.0
    scan_half_life_s: float = 60.0
    blocked_half_life_s: float = 120.0
    stale_after_s: float = 1.5
    collision_band_min_m: float = -2.0
    collision_band_max_m: float = 2.5
    occupied_hit_probability: float = 0.70
    free_miss_probability: float = 0.12
    prior_occupancy_probability: float = 0.20
    min_log_odds: float = -3.5
    max_log_odds: float = 3.5
    obstacle_threshold: float = 0.65
    obstacle_inflation_radius_m: float = 1.2
    obstacle_soft_radius_m: float = 2.0
    peer_hard_radius_m: float = 3.0
    peer_soft_radius_m: float = 6.0
    peer_vertical_clearance_m: float = 2.5
    peer_prediction_s: float = 2.0
    peer_timeout_s: float = 2.5
    warn_distance_m: float = 4.0
    critical_distance_m: float = 2.0
    blocked_query_radius_m: float = 1.2
    blocked_query_offset_m: float = 2.0
    peer_cost_gain: float = 1.0
    blocked_cost_gain: float = 1.0
    voxel_size_m: float = 0.2
    max_batches: int = 16


@dataclass(slots=True)
class LocalClearanceSummary:
    state: LocalMapperState
    stamp_s: float
    valid_for_planning: bool
    validity_reason: str
    closest_m: float
    forward_m: float
    left_m: float
    center_m: float
    right_m: float
    warn: bool
    critical: bool
    free_directions: tuple[str, ...]


@dataclass(slots=True)
class LocalMapperSnapshot:
    state: LocalMapperState
    stamp_s: float
    age_s: float
    valid_for_planning: bool
    validity_reason: str
    observed_cell_count: int
    resolution_m: float
    width: int
    height: int
    origin_ned: tuple[float, float]
    drone_ned: tuple[float, float]
    drone_yaw_rad: float
    occupancy_confidence: np.ndarray
    occupied_mask: np.ndarray
    unknown_mask: np.ndarray
    inflation_map: np.ndarray
    dynamic_no_go_mask: np.ndarray
    blocked_cost_layer: np.ndarray
    combined_cost_map: np.ndarray


class LocalMapper:
    def __init__(
        self,
        config: LocalMapperConfig | None = None,
        *,
        logger: Any | None = None,
    ) -> None:
        self._config = config or LocalMapperConfig()
        self._logger = logger

        self._resolution = float(self._config.resolution_m)
        self._width = max(8, int(math.ceil(self._config.span_x_m / self._resolution)))
        self._height = max(8, int(math.ceil(self._config.span_y_m / self._resolution)))

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_ready = False

        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_z = 0.0
        self._pose_yaw = 0.0

        shape = (self._height, self._width)
        self._fast_layer = np.zeros(shape, dtype=np.float32)
        self._scan_layer = np.zeros(shape, dtype=np.float32)
        self._blocked_layer = np.zeros(shape, dtype=np.float32)
        self._observed_layer = np.zeros(shape, dtype=np.bool_)

        self._prior_log_odds = prob_to_logodds(self._config.prior_occupancy_probability)
        self._hit_log_odds_delta = (
            prob_to_logodds(self._config.occupied_hit_probability) - self._prior_log_odds
        )
        self._miss_log_odds_delta = (
            prob_to_logodds(self._config.free_miss_probability) - self._prior_log_odds
        )
        self._min_log_odds = min(
            float(self._config.min_log_odds),
            float(self._config.max_log_odds),
        )
        self._max_log_odds = max(
            float(self._config.min_log_odds),
            float(self._config.max_log_odds),
        )
        self._min_evidence_log_odds = self._min_log_odds - self._prior_log_odds
        self._max_evidence_log_odds = self._max_log_odds - self._prior_log_odds

        self._last_sensor_stamp = 0.0
        self._last_nonempty_insert_stamp = 0.0
        self._last_validity_reason = "empty_no_sensor_input"
        self._last_decay_stamp = 0.0
        self._recent_batches: deque[dict[str, Any]] = deque(maxlen=max(1, int(self._config.max_batches)))
        self._dense_scan_voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self._voxel_size_m = max(0.05, float(self._config.voxel_size_m))

        self._peer_store = PeerTrackStore(
            track_ttl_s=self._config.peer_timeout_s,
            base_radius_m=self._config.peer_hard_radius_m,
            soft_shell_m=max(0.0, self._config.peer_soft_radius_m - self._config.peer_hard_radius_m),
            lookahead_s=self._config.peer_prediction_s,
        )

        self._inflation_kernel = self._build_gradient_kernel(
            self._config.obstacle_inflation_radius_m,
            self._config.obstacle_soft_radius_m,
            peak=1.0,
        )

        self._latest_snapshot = self._empty_snapshot()
        self._latest_summary = LocalClearanceSummary(
            state=LocalMapperState.EMPTY,
            stamp_s=0.0,
            valid_for_planning=False,
            validity_reason="empty_no_sensor_input",
            closest_m=99.0,
            forward_m=99.0,
            left_m=99.0,
            center_m=99.0,
            right_m=99.0,
            warn=False,
            critical=False,
            free_directions=("left", "center", "right"),
        )

    @property
    def state(self) -> LocalMapperState:
        return self._resolve_state(time.time())

    @property
    def latest_snapshot(self) -> LocalMapperSnapshot:
        return self._latest_snapshot

    @property
    def latest_summary(self) -> LocalClearanceSummary:
        return self._latest_summary

    def update_pose(self, x: float, y: float, z: float, yaw: float, stamp_s: float) -> None:
        del stamp_s
        self._pose_x = float(x)
        self._pose_y = float(y)
        self._pose_z = float(z)
        self._pose_yaw = float(yaw)
        self._recenter_grid()

    def ingest_points(self, batch: PointBatch) -> int:
        return self.ingest_point_batch(batch)

    def ingest_point_batch(self, batch: PointBatch) -> int:
        points = np.asarray(batch.points_xyz, dtype=np.float32)
        if points.size == 0:
            self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
            self._last_validity_reason = "degraded_empty_point_batch"
            self._recent_batches.append(
                {
                    "stamp_s": float(batch.stamp_s),
                    "source": batch.source,
                    "point_count": 0,
                    "is_dense_scan": bool(batch.is_dense_scan),
                }
            )
            return 0

        if not self._origin_ready:
            self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
            self._last_validity_reason = "degraded_origin_not_ready"
            return 0

        self._decay_layers(batch.stamp_s)
        points = points.reshape((-1, 3))
        valid = (
            np.isfinite(points[:, 0])
            & np.isfinite(points[:, 1])
            & np.isfinite(points[:, 2])
        )
        if not np.any(valid):
            self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
            self._last_validity_reason = "degraded_invalid_points"
            return 0
        points = points[valid]

        rel_z = points[:, 2] - self._pose_z
        in_band = (
            (rel_z >= self._config.collision_band_min_m)
            & (rel_z <= self._config.collision_band_max_m)
        )
        if not np.any(in_band):
            self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
            self._last_validity_reason = "degraded_points_outside_collision_band"
            return 0
        points = points[in_band]

        gx = np.floor((points[:, 0] - self._origin_x) / self._resolution).astype(np.int32)
        gy = np.floor((points[:, 1] - self._origin_y) / self._resolution).astype(np.int32)
        in_bounds = (gx >= 0) & (gx < self._width) & (gy >= 0) & (gy < self._height)
        if not np.any(in_bounds):
            self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
            self._last_validity_reason = "degraded_points_outside_local_grid"
            return 0
        gx = gx[in_bounds]
        gy = gy[in_bounds]
        points = points[in_bounds]

        flat = np.unique(gy * self._width + gx)
        layer = self._scan_layer if batch.is_dense_scan else self._fast_layer
        source_gain = 1.25 if batch.is_dense_scan else 1.0
        evidence_scale = max(0.0, float(batch.confidence)) * source_gain

        free_flat = self._raytrace_free_cells(gx, gy)
        if free_flat.size > 0:
            np.add.at(layer.reshape(-1), free_flat, self._miss_log_odds_delta * evidence_scale)
            self._observed_layer.reshape(-1)[free_flat] = True

        np.add.at(layer.reshape(-1), flat, self._hit_log_odds_delta * evidence_scale)
        self._observed_layer.reshape(-1)[flat] = True
        np.clip(layer, self._min_evidence_log_odds, self._max_evidence_log_odds, out=layer)

        inserted_voxels = 0
        if batch.is_dense_scan:
            keys = np.rint(points / self._voxel_size_m).astype(np.int32)
            for idx, key in enumerate(keys):
                voxel = (int(key[0]), int(key[1]), int(key[2]))
                if voxel not in self._dense_scan_voxels:
                    inserted_voxels += 1
                px = points[idx]
                self._dense_scan_voxels[voxel] = (float(px[0]), float(px[1]), float(px[2]))

        self._last_sensor_stamp = max(self._last_sensor_stamp, float(batch.stamp_s))
        self._last_nonempty_insert_stamp = max(
            self._last_nonempty_insert_stamp,
            float(batch.stamp_s),
        )
        self._last_validity_reason = "tracking"
        self._recent_batches.append(
            {
                "stamp_s": float(batch.stamp_s),
                "source": batch.source,
                "point_count": int(points.shape[0]),
                "is_dense_scan": bool(batch.is_dense_scan),
                "confidence": float(batch.confidence),
                "sensor_range_m": float(batch.sensor_range_m),
                "inserted_voxels": int(inserted_voxels),
            }
        )
        return inserted_voxels

    def ingest_peer_position(
        self,
        peer_id: str,
        *,
        x: float,
        y: float,
        z: float,
        stamp_s: float,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> None:
        peer_int = int(str(peer_id).split("_")[-1]) if str(peer_id).startswith("drone_") else int(peer_id)
        self._peer_store.update_track(
            drone_id=peer_int,
            x=float(x),
            y=float(y),
            z=float(z),
            vx=float(vx),
            vy=float(vy),
            stamp_s=float(stamp_s),
        )

    def mark_blocked_zone(
        self,
        *,
        x: float,
        y: float,
        radius_m: float,
        score: float = 1.0,
        stamp_s: float | None = None,
        label: str = "",
    ) -> None:
        del label
        if not self._origin_ready:
            return
        ref = time.time() if stamp_s is None else float(stamp_s)
        self._decay_layers(ref)
        gx, gy = self.world_to_grid(x, y)
        if gx is None or gy is None:
            return
        kernel = self._build_gradient_kernel(radius_m, 0.0, peak=float(score))
        self._apply_kernel_add(self._blocked_layer, kernel, gx, gy)
        np.clip(self._blocked_layer, 0.0, 4.0, out=self._blocked_layer)

    def clear_blocked_history(self) -> None:
        self._blocked_layer.fill(0.0)

    def clear_sensor_layers(self) -> None:
        """Clear accumulated depth observations (fast + scan layers).

        Call this on takeoff completion so ground-level detections captured
        during ascent don't persist as ghost obstacles at cruise altitude.
        """
        self._fast_layer.fill(0.0)
        self._scan_layer.fill(0.0)
        self._observed_layer.fill(False)
        self._last_nonempty_insert_stamp = 0.0
        self._dense_scan_voxels.clear()

    def update(self, now_s: float | None = None) -> tuple[LocalMapperSnapshot, LocalClearanceSummary]:
        ref = time.time() if now_s is None else float(now_s)
        self._recenter_grid()
        self._decay_layers(ref)

        log_odds = np.clip(
            self._prior_log_odds + self._fast_layer + self._scan_layer,
            self._min_log_odds,
            self._max_log_odds,
        )
        occupancy = np.asarray(logodds_to_prob(log_odds), dtype=np.float32)
        unknown = ~self._observed_layer
        occupied = (~unknown) & (occupancy >= self._config.obstacle_threshold)
        observed_cell_count = int(np.count_nonzero(self._observed_layer))

        inflation = np.zeros_like(occupancy, dtype=np.float32)
        occ_y, occ_x = np.nonzero(occupied)
        for gy, gx in zip(occ_y.tolist(), occ_x.tolist()):
            self._apply_kernel_max(inflation, self._inflation_kernel, gx, gy)

        dynamic_no_go = np.zeros_like(occupied, dtype=np.bool_)
        peer_cost = np.zeros_like(occupancy, dtype=np.float32)
        for zone in self._peer_store.build_safety_disks(
            now_s=ref,
            own_z_ned=self._pose_z,
            vertical_clearance_m=self._config.peer_vertical_clearance_m,
        ):
            gx, gy = self.world_to_grid(zone.center_ned[0], zone.center_ned[1])
            if gx is None or gy is None:
                continue
            hard_kernel = self._build_gradient_kernel(zone.radius_m, 0.0, peak=1.0)
            soft_kernel = self._build_gradient_kernel(
                zone.radius_m,
                max(0.0, zone.soft_radius_m - zone.radius_m),
                peak=1.0,
            )
            peer_patch = np.zeros_like(peer_cost, dtype=np.float32)
            self._apply_kernel_max(peer_patch, hard_kernel, gx, gy)
            dynamic_no_go |= peer_patch > 0.5
            self._apply_kernel_max(peer_cost, soft_kernel, gx, gy)

        combined = np.clip(
            inflation
            + (peer_cost * float(self._config.peer_cost_gain))
            + (self._blocked_layer * float(self._config.blocked_cost_gain)),
            0.0,
            5.0,
        )
        age_s = 0.0 if self._last_sensor_stamp <= 0.0 else max(0.0, ref - self._last_sensor_stamp)
        state = self._resolve_state(ref)
        valid_for_planning, validity_reason = self._resolve_validity(
            ref,
            state=state,
            observed_cell_count=observed_cell_count,
        )
        snapshot = LocalMapperSnapshot(
            state=state,
            stamp_s=ref,
            age_s=age_s,
            valid_for_planning=valid_for_planning,
            validity_reason=validity_reason,
            observed_cell_count=observed_cell_count,
            resolution_m=self._resolution,
            width=self._width,
            height=self._height,
            origin_ned=(float(self._origin_x), float(self._origin_y)),
            drone_ned=(float(self._pose_x), float(self._pose_y)),
            drone_yaw_rad=float(self._pose_yaw),
            occupancy_confidence=self._freeze(occupancy),
            occupied_mask=self._freeze(occupied.astype(np.bool_)),
            unknown_mask=self._freeze(unknown.astype(np.bool_)),
            inflation_map=self._freeze(inflation),
            dynamic_no_go_mask=self._freeze(dynamic_no_go),
            blocked_cost_layer=self._freeze(self._blocked_layer),
            combined_cost_map=self._freeze(combined),
        )
        summary = self._build_clearance_summary(snapshot)
        self._latest_snapshot = snapshot
        self._latest_summary = summary
        return snapshot, summary

    def get_dense_scan_points(self) -> np.ndarray:
        if not self._dense_scan_voxels:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(list(self._dense_scan_voxels.values()), dtype=np.float32)

    def summary(self) -> dict[str, Any]:
        return {
            "state": self._latest_summary.state.name,
            "valid_for_planning": bool(self._latest_summary.valid_for_planning),
            "validity_reason": self._latest_summary.validity_reason,
            "dense_scan_points": int(len(self._dense_scan_voxels)),
            "recent_batches": list(self._recent_batches),
            "last_ingest_ts": round(float(self._last_sensor_stamp), 3),
            "last_nonempty_insert_ts": round(float(self._last_nonempty_insert_stamp), 3),
        }

    def peer_planner_mask_payload(self, *, now_s: float | None = None) -> list[dict[str, Any]]:
        """Return current peer safety disks in planner-mask payload format."""

        ref = time.time() if now_s is None else float(now_s)
        return self._peer_store.build_planner_mask_payload(
            now_s=ref,
            own_z_ned=self._pose_z,
            vertical_clearance_m=self._config.peer_vertical_clearance_m,
        )

    def world_to_grid(self, x: float, y: float) -> tuple[int | None, int | None]:
        if not self._origin_ready:
            return None, None
        gx = int(math.floor((x - self._origin_x) / self._resolution))
        gy = int(math.floor((y - self._origin_y) / self._resolution))
        if gx < 0 or gx >= self._width or gy < 0 or gy >= self._height:
            return None, None
        return gx, gy

    def is_direction_blocked(
        self,
        side: str,
        *,
        origin_x: float,
        origin_y: float,
        heading_rad: float,
        lateral_offset_m: float | None = None,
    ) -> bool:
        if side not in {"left", "right"}:
            return False
        offset = self._config.blocked_query_offset_m if lateral_offset_m is None else float(lateral_offset_m)
        angle = heading_rad + (math.pi / 2.0 if side == "left" else -math.pi / 2.0)
        cx = origin_x + offset * math.cos(angle)
        cy = origin_y + offset * math.sin(angle)
        gx, gy = self.world_to_grid(cx, cy)
        if gx is None or gy is None:
            return False

        radius_cells = max(1, int(math.ceil(self._config.blocked_query_radius_m / self._resolution)))
        x0 = max(0, gx - radius_cells)
        x1 = min(self._width, gx + radius_cells + 1)
        y0 = max(0, gy - radius_cells)
        y1 = min(self._height, gy + radius_cells + 1)
        return bool(np.max(self._blocked_layer[y0:y1, x0:x1]) >= 0.6)

    def _build_clearance_summary(self, snapshot: LocalMapperSnapshot) -> LocalClearanceSummary:
        obstacle_mask = snapshot.occupied_mask | snapshot.dynamic_no_go_mask
        yy, xx = np.nonzero(obstacle_mask)
        if xx.size == 0:
            return LocalClearanceSummary(
                state=snapshot.state,
                stamp_s=snapshot.stamp_s,
                valid_for_planning=snapshot.valid_for_planning,
                validity_reason=snapshot.validity_reason,
                closest_m=99.0,
                forward_m=99.0,
                left_m=99.0,
                center_m=99.0,
                right_m=99.0,
                warn=False,
                critical=False,
                free_directions=("left", "center", "right"),
            )

        wx = snapshot.origin_ned[0] + (xx.astype(np.float32) + 0.5) * snapshot.resolution_m
        wy = snapshot.origin_ned[1] + (yy.astype(np.float32) + 0.5) * snapshot.resolution_m
        dx = wx - snapshot.drone_ned[0]
        dy = wy - snapshot.drone_ned[1]
        dist = np.hypot(dx, dy)
        angle = np.arctan2(dy, dx) - snapshot.drone_yaw_rad
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        def _sector_min(mask: np.ndarray) -> float:
            if not np.any(mask):
                return 99.0
            return float(np.min(dist[mask]))

        left_m = _sector_min((angle > math.radians(25.0)) & (angle <= math.radians(140.0)))
        center_m = _sector_min(np.abs(angle) <= math.radians(30.0))
        right_m = _sector_min((angle < -math.radians(25.0)) & (angle >= -math.radians(140.0)))
        forward_m = _sector_min(np.abs(angle) <= math.radians(18.0))
        closest_m = float(np.min(dist))

        free = []
        if left_m > self._config.warn_distance_m:
            free.append("left")
        if center_m > self._config.warn_distance_m:
            free.append("center")
        if right_m > self._config.warn_distance_m:
            free.append("right")

        return LocalClearanceSummary(
            state=snapshot.state,
            stamp_s=snapshot.stamp_s,
            valid_for_planning=snapshot.valid_for_planning,
            validity_reason=snapshot.validity_reason,
            closest_m=closest_m,
            forward_m=forward_m,
            left_m=left_m,
            center_m=center_m,
            right_m=right_m,
            warn=closest_m < self._config.warn_distance_m,
            critical=closest_m < self._config.critical_distance_m,
            free_directions=tuple(free),
        )

    def _resolve_state(self, ref: float) -> LocalMapperState:
        if self._last_sensor_stamp <= 0.0:
            return LocalMapperState.EMPTY
        if (float(ref) - self._last_sensor_stamp) > self._config.stale_after_s:
            return LocalMapperState.STALE_INPUT
        return LocalMapperState.TRACKING

    def _resolve_validity(
        self,
        ref: float,
        *,
        state: LocalMapperState,
        observed_cell_count: int,
    ) -> tuple[bool, str]:
        if state == LocalMapperState.EMPTY:
            return False, "empty_no_sensor_input"
        if state == LocalMapperState.STALE_INPUT:
            age_s = max(0.0, float(ref) - float(self._last_sensor_stamp))
            return False, f"stale_input_age_{age_s:.2f}s"
        if not self._origin_ready:
            return False, "degraded_origin_not_ready"
        # TRACKING + origin ready → map is valid even if no obstacle cells were observed.
        # An open field with no obstacles is a valid (clear) map; requiring observed_cell_count > 0
        # would block planning whenever the depth camera sees only ground below the collision band.
        return True, "tracking"

    def _recenter_grid(self) -> None:
        desired_origin_x = self._pose_x - 0.5 * self._width * self._resolution
        desired_origin_y = self._pose_y - 0.5 * self._height * self._resolution
        if not self._origin_ready:
            self._origin_x = desired_origin_x
            self._origin_y = desired_origin_y
            self._origin_ready = True
            return

        shift_x = int(round((desired_origin_x - self._origin_x) / self._resolution))
        shift_y = int(round((desired_origin_y - self._origin_y) / self._resolution))
        if shift_x == 0 and shift_y == 0:
            return

        self._shift_layer(self._fast_layer, shift_x, shift_y)
        self._shift_layer(self._scan_layer, shift_x, shift_y)
        self._shift_layer(self._blocked_layer, shift_x, shift_y)
        self._shift_layer(self._observed_layer, shift_x, shift_y)
        self._origin_x += shift_x * self._resolution
        self._origin_y += shift_y * self._resolution

    def _shift_layer(self, layer: np.ndarray, shift_x: int, shift_y: int) -> None:
        if abs(shift_x) >= self._width or abs(shift_y) >= self._height:
            layer.fill(0.0)
            return

        original = layer.copy()
        layer.fill(0.0)

        src_x0 = max(0, -shift_x)
        src_x1 = min(self._width, self._width - shift_x) if shift_x >= 0 else self._width
        dst_x0 = max(0, shift_x)
        dst_x1 = dst_x0 + (src_x1 - src_x0)

        src_y0 = max(0, -shift_y)
        src_y1 = min(self._height, self._height - shift_y) if shift_y >= 0 else self._height
        dst_y0 = max(0, shift_y)
        dst_y1 = dst_y0 + (src_y1 - src_y0)

        if src_x1 <= src_x0 or src_y1 <= src_y0:
            return
        layer[dst_y0:dst_y1, dst_x0:dst_x1] = original[src_y0:src_y1, src_x0:src_x1]

    def _decay_layers(self, now_s: float) -> None:
        now_s = float(now_s)
        if self._last_decay_stamp <= 0.0:
            self._last_decay_stamp = now_s
            return
        dt = max(0.0, now_s - self._last_decay_stamp)
        if dt <= 0.0:
            return
        self._fast_layer *= self._decay_factor(dt, self._config.depth_half_life_s)
        self._scan_layer *= self._decay_factor(dt, self._config.scan_half_life_s)
        self._blocked_layer *= self._decay_factor(dt, self._config.blocked_half_life_s)
        self._last_decay_stamp = now_s

    def _raytrace_free_cells(self, hit_gx: np.ndarray, hit_gy: np.ndarray) -> np.ndarray:
        start_gx, start_gy = self.world_to_grid(self._pose_x, self._pose_y)
        if start_gx is None or start_gy is None:
            return np.empty((0,), dtype=np.int64)

        free_cells: list[int] = []
        for gx, gy in zip(hit_gx.tolist(), hit_gy.tolist()):
            for cx, cy in self._bresenham_cells(start_gx, start_gy, int(gx), int(gy)):
                if cx == int(gx) and cy == int(gy):
                    break
                free_cells.append(cy * self._width + cx)
        if not free_cells:
            return np.empty((0,), dtype=np.int64)
        return np.unique(np.asarray(free_cells, dtype=np.int64))

    def _bresenham_cells(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> list[tuple[int, int]]:
        cells: list[tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x = int(x0)
        y = int(y0)
        while True:
            if 0 <= x < self._width and 0 <= y < self._height:
                cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    def _build_gradient_kernel(self, hard_radius_m: float, soft_radius_m: float, *, peak: float) -> np.ndarray:
        total_radius = max(hard_radius_m + max(0.0, soft_radius_m), hard_radius_m, self._resolution)
        radius_cells = max(1, int(math.ceil(total_radius / self._resolution)))
        kernel = np.zeros((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=np.float32)
        yy, xx = np.indices(kernel.shape)
        dx = (xx - radius_cells) * self._resolution
        dy = (yy - radius_cells) * self._resolution
        dist = np.hypot(dx, dy)
        kernel[dist <= hard_radius_m] = float(peak)
        if soft_radius_m > 0.0:
            soft_mask = (dist > hard_radius_m) & (dist <= hard_radius_m + soft_radius_m)
            kernel[soft_mask] = (
                1.0 - ((dist[soft_mask] - hard_radius_m) / soft_radius_m)
            ) * float(peak)
        return kernel

    def _apply_kernel_max(self, layer: np.ndarray, kernel: np.ndarray, cx: int, cy: int) -> None:
        self._apply_kernel(layer, kernel, cx, cy, mode="max")

    def _apply_kernel_add(self, layer: np.ndarray, kernel: np.ndarray, cx: int, cy: int) -> None:
        self._apply_kernel(layer, kernel, cx, cy, mode="add")

    def _apply_kernel(self, layer: np.ndarray, kernel: np.ndarray, cx: int, cy: int, *, mode: str) -> None:
        radius_y = kernel.shape[0] // 2
        radius_x = kernel.shape[1] // 2
        x0 = max(0, cx - radius_x)
        x1 = min(layer.shape[1], cx + radius_x + 1)
        y0 = max(0, cy - radius_y)
        y1 = min(layer.shape[0], cy + radius_y + 1)
        if x1 <= x0 or y1 <= y0:
            return
        kx0 = radius_x - (cx - x0)
        kx1 = kx0 + (x1 - x0)
        ky0 = radius_y - (cy - y0)
        ky1 = ky0 + (y1 - y0)
        patch = kernel[ky0:ky1, kx0:kx1]
        if mode == "max":
            np.maximum(layer[y0:y1, x0:x1], patch, out=layer[y0:y1, x0:x1])
        else:
            layer[y0:y1, x0:x1] += patch

    def _freeze(self, array: np.ndarray) -> np.ndarray:
        frozen = np.array(array, copy=True)
        frozen.setflags(write=False)
        return frozen

    def _empty_snapshot(self) -> LocalMapperSnapshot:
        shape = (self._height, self._width)
        zeros_f = self._freeze(np.zeros(shape, dtype=np.float32))
        zeros_b = self._freeze(np.zeros(shape, dtype=np.bool_))
        ones_b = self._freeze(np.ones(shape, dtype=np.bool_))
        return LocalMapperSnapshot(
            state=LocalMapperState.EMPTY,
            stamp_s=0.0,
            age_s=0.0,
            valid_for_planning=False,
            validity_reason="empty_no_sensor_input",
            observed_cell_count=0,
            resolution_m=self._resolution,
            width=self._width,
            height=self._height,
            origin_ned=(0.0, 0.0),
            drone_ned=(0.0, 0.0),
            drone_yaw_rad=0.0,
            occupancy_confidence=zeros_f,
            occupied_mask=zeros_b,
            unknown_mask=ones_b,
            inflation_map=zeros_f,
            dynamic_no_go_mask=zeros_b,
            blocked_cost_layer=zeros_f,
            combined_cost_map=zeros_f,
        )

    def _decay_factor(self, dt: float, half_life_s: float) -> float:
        if half_life_s <= 0.0:
            return 0.0
        return float(0.5 ** (dt / half_life_s))
