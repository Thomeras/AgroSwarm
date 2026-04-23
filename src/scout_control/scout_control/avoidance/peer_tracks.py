"""Peer-drone track buffering and dynamic safety-zone generation."""

from __future__ import annotations

import json
import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Any, Iterable, Mapping


@dataclass(slots=True)
class PeerTrack:
    """Latest known peer state in local NED."""

    drone_id: int
    x: float
    y: float
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    status: str = "active"
    stamp_s: float = 0.0

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.vx, self.vy)

    def age_s(self, now_s: float | None = None) -> float:
        ref = time.time() if now_s is None else float(now_s)
        return max(0.0, ref - float(self.stamp_s))


@dataclass(slots=True)
class SafetyDiskZone:
    """Dynamic peer no-go disk with hard and soft shell radii."""

    zone_id: str
    drone_id: int
    center_ned: tuple[float, float]
    radius_m: float
    soft_radius_m: float
    speed_mps: float
    age_s: float
    status: str

    def to_payload(self) -> dict[str, Any]:
        return {
            "zone_id": self.zone_id,
            "drone_id": int(self.drone_id),
            "center_ned": [float(self.center_ned[0]), float(self.center_ned[1])],
            "radius_m": float(self.radius_m),
            "soft_radius_m": float(self.soft_radius_m),
            "speed_mps": float(self.speed_mps),
            "age_s": float(self.age_s),
            "status": self.status,
        }

    def to_planner_mask_payload(self) -> dict[str, Any]:
        hard_radius = max(0.0, float(self.radius_m))
        soft_radius = max(hard_radius, float(self.soft_radius_m))
        return {
            "zone_id": self.zone_id,
            "source": "peer_track",
            "drone_id": int(self.drone_id),
            "center_ned": [float(self.center_ned[0]), float(self.center_ned[1])],
            "hard_radius_m": hard_radius,
            "soft_radius_m": soft_radius,
            "weight": 1.0,
            "age_s": max(0.0, float(self.age_s)),
            "status": self.status,
        }

    @property
    def is_finite(self) -> bool:
        return (
            math.isfinite(float(self.center_ned[0]))
            and math.isfinite(float(self.center_ned[1]))
            and math.isfinite(float(self.radius_m))
            and math.isfinite(float(self.soft_radius_m))
        )


class PeerTrackStore:
    """Keep short peer history and derive planner safety disks."""

    def __init__(
        self,
        *,
        track_ttl_s: float = 3.0,
        max_history: int = 8,
        base_radius_m: float = 1.5,
        soft_shell_m: float = 2.0,
        lookahead_s: float = 1.5,
        velocity_inflation_gain: float = 0.75,
        age_inflation_gain: float = 0.3,
        max_extra_radius_m: float = 2.0,
        max_track_speed_mps: float = 12.0,
        velocity_smoothing: float = 0.6,
        min_velocity_dt_s: float = 0.05,
    ) -> None:
        self._track_ttl_s = float(track_ttl_s)
        self._base_radius_m = float(base_radius_m)
        self._soft_shell_m = float(soft_shell_m)
        self._lookahead_s = float(lookahead_s)
        self._velocity_inflation_gain = float(velocity_inflation_gain)
        self._age_inflation_gain = float(age_inflation_gain)
        self._max_extra_radius_m = float(max_extra_radius_m)
        self._max_track_speed_mps = max(0.1, float(max_track_speed_mps))
        self._velocity_smoothing = min(1.0, max(0.0, float(velocity_smoothing)))
        self._min_velocity_dt_s = max(1e-3, float(min_velocity_dt_s))
        self._tracks: dict[int, deque[PeerTrack]] = defaultdict(
            lambda: deque(maxlen=max(1, int(max_history)))
        )

    def _estimate_velocity(
        self,
        *,
        drone_id: int,
        x: float,
        y: float,
        stamp_s: float,
        vx_hint: float,
        vy_hint: float,
    ) -> tuple[float, float]:
        history = self._tracks.get(int(drone_id))
        if not history:
            return vx_hint, vy_hint
        prev = history[-1]
        dt = float(stamp_s) - float(prev.stamp_s)
        if dt < self._min_velocity_dt_s:
            return vx_hint, vy_hint

        est_vx = (float(x) - float(prev.x)) / dt
        est_vy = (float(y) - float(prev.y)) / dt
        alpha = self._velocity_smoothing
        blended_vx = alpha * vx_hint + (1.0 - alpha) * est_vx
        blended_vy = alpha * vy_hint + (1.0 - alpha) * est_vy
        speed = math.hypot(blended_vx, blended_vy)
        if speed <= self._max_track_speed_mps or speed <= 1e-6:
            return blended_vx, blended_vy
        scale = self._max_track_speed_mps / speed
        return blended_vx * scale, blended_vy * scale

    def update_track(
        self,
        *,
        drone_id: int,
        x: float,
        y: float,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        age_s: float = 0.0,
        status: str = "active",
        stamp_s: float | None = None,
    ) -> PeerTrack:
        now_s = time.time() if stamp_s is None else float(stamp_s)
        sample_stamp = now_s - max(0.0, float(age_s))
        vx_est, vy_est = self._estimate_velocity(
            drone_id=int(drone_id),
            x=float(x),
            y=float(y),
            stamp_s=sample_stamp,
            vx_hint=float(vx),
            vy_hint=float(vy),
        )
        track = PeerTrack(
            drone_id=int(drone_id),
            x=float(x),
            y=float(y),
            z=float(z),
            vx=float(vx_est),
            vy=float(vy_est),
            status=str(status),
            stamp_s=sample_stamp,
        )
        self._tracks[track.drone_id].append(track)
        return track

    def update_from_array(
        self,
        tracks: Iterable[Mapping[str, Any]],
        *,
        stamp_s: float | None = None,
    ) -> list[PeerTrack]:
        updated: list[PeerTrack] = []
        for item in tracks:
            try:
                drone_id = int(item.get("drone_id", item.get("id")))
            except (TypeError, ValueError):
                continue
            if drone_id < 0:
                continue
            position_ned = item.get("position_ned")
            velocity_ned = item.get("velocity_ned")
            if isinstance(position_ned, (list, tuple)) and len(position_ned) >= 2:
                x_val = position_ned[0]
                y_val = position_ned[1]
                z_val = position_ned[2] if len(position_ned) > 2 else item.get("z", 0.0)
            else:
                x_val = item.get("x")
                y_val = item.get("y")
                z_val = item.get("z", 0.0)
            if isinstance(velocity_ned, (list, tuple)) and len(velocity_ned) >= 2:
                vx_val = velocity_ned[0]
                vy_val = velocity_ned[1]
            else:
                vx_val = item.get("vx", 0.0)
                vy_val = item.get("vy", 0.0)
            try:
                updated.append(
                    self.update_track(
                        drone_id=drone_id,
                        x=float(x_val),
                        y=float(y_val),
                        z=float(z_val),
                        vx=float(vx_val),
                        vy=float(vy_val),
                        age_s=float(item.get("age_s", 0.0)),
                        status=str(item.get("status", "active")),
                        stamp_s=stamp_s,
                    )
                )
            except (TypeError, ValueError):
                continue
        return updated

    def update_from_json(
        self,
        payload: str | bytes,
        *,
        stamp_s: float | None = None,
    ) -> list[PeerTrack]:
        data = json.loads(payload)
        tracks = data.get("tracks", data if isinstance(data, list) else [])
        if not isinstance(tracks, list):
            raise ValueError("peer track payload must contain a list in 'tracks'")
        return self.update_from_array(tracks, stamp_s=stamp_s)

    def prune(self, *, now_s: float | None = None) -> None:
        ref = time.time() if now_s is None else float(now_s)
        stale_ids: list[int] = []
        for drone_id, history in self._tracks.items():
            fresh = [track for track in history if track.age_s(ref) <= self._track_ttl_s]
            if fresh:
                self._tracks[drone_id] = deque(fresh, maxlen=history.maxlen)
            else:
                stale_ids.append(drone_id)
        for drone_id in stale_ids:
            del self._tracks[drone_id]

    def latest_tracks(
        self,
        *,
        now_s: float | None = None,
        exclude_drone_id: int | None = None,
    ) -> list[PeerTrack]:
        self.prune(now_s=now_s)
        latest: list[PeerTrack] = []
        for drone_id, history in self._tracks.items():
            if exclude_drone_id is not None and drone_id == int(exclude_drone_id):
                continue
            if history:
                latest.append(history[-1])
        latest.sort(key=lambda track: track.drone_id)
        return latest

    def history_for(self, drone_id: int) -> list[PeerTrack]:
        return list(self._tracks.get(int(drone_id), ()))

    def build_safety_disks(
        self,
        *,
        now_s: float | None = None,
        exclude_drone_id: int | None = None,
        lookahead_s: float | None = None,
    ) -> list[SafetyDiskZone]:
        ref = time.time() if now_s is None else float(now_s)
        lead_s = self._lookahead_s if lookahead_s is None else float(lookahead_s)
        zones: list[SafetyDiskZone] = []

        for track in self.latest_tracks(now_s=ref, exclude_drone_id=exclude_drone_id):
            status = track.status.lower()
            if status in {"inactive", "landed", "offline"}:
                continue

            age_s = track.age_s(ref)
            pred_x = track.x + track.vx * lead_s
            pred_y = track.y + track.vy * lead_s
            extra_radius = min(
                self._max_extra_radius_m,
                track.speed_mps * self._velocity_inflation_gain
                + age_s * self._age_inflation_gain,
            )
            radius_m = self._base_radius_m + extra_radius
            zones.append(
                SafetyDiskZone(
                    zone_id=f"peer_{track.drone_id}",
                    drone_id=track.drone_id,
                    center_ned=(pred_x, pred_y),
                    radius_m=radius_m,
                    soft_radius_m=radius_m + self._soft_shell_m,
                    speed_mps=track.speed_mps,
                    age_s=age_s,
                    status=track.status,
                )
            )

        return zones

    def build_planner_mask_payload(
        self,
        *,
        now_s: float | None = None,
        exclude_drone_id: int | None = None,
        lookahead_s: float | None = None,
    ) -> list[dict[str, Any]]:
        """Return sanitized peer no-go disks ready for planner-mask rasterization."""

        payloads: list[dict[str, Any]] = []
        for zone in self.build_safety_disks(
            now_s=now_s,
            exclude_drone_id=exclude_drone_id,
            lookahead_s=lookahead_s,
        ):
            if not zone.is_finite:
                continue
            payload = zone.to_planner_mask_payload()
            if payload["soft_radius_m"] <= 0.0:
                continue
            payloads.append(payload)
        return payloads
