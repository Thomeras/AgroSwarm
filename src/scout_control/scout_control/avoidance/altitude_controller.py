"""Altitude policy hook for obstacle avoidance runtime."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class AltitudeSetpoint:
    z_ned: float
    mode: str
    terrain_valid: bool = False


class AltitudeController:
    """Resolve mission altitude into a PX4 NED Z setpoint."""

    FIXED_NED = "FixedNED"
    TERRAIN_FOLLOW = "TerrainFollow"

    def __init__(self, *, mode: str = FIXED_NED, default_altitude_m: float = 5.0) -> None:
        normalized = str(mode or self.FIXED_NED).strip().lower()
        self.mode = self.TERRAIN_FOLLOW if normalized in {"terrainfollow", "terrain_follow"} else self.FIXED_NED
        self.default_altitude_m = float(default_altitude_m)
        self._terrain_z_ned: float | None = None

    def update_terrain_reference(self, *, terrain_z_ned: float | None) -> None:
        self._terrain_z_ned = None if terrain_z_ned is None else float(terrain_z_ned)

    def setpoint_for(self, *, altitude_m: float | None = None) -> AltitudeSetpoint:
        altitude = self.default_altitude_m if altitude_m is None else float(altitude_m)
        if self.mode == self.TERRAIN_FOLLOW and self._terrain_z_ned is not None:
            return AltitudeSetpoint(
                z_ned=float(self._terrain_z_ned) - altitude,
                mode=self.mode,
                terrain_valid=True,
            )
        return AltitudeSetpoint(z_ned=-altitude, mode=self.mode, terrain_valid=False)
